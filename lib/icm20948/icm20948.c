#include <esp_timer.h>

#include "icm20948.h"

//non blocking
static void read_register(icm20948_t *icm, uint8_t bank, uint8_t address, uint8_t *buf, uint8_t len);
static void write_register(icm20948_t *icm, uint8_t bank, uint8_t address, uint8_t *buf, uint8_t len);

//blocking functions
static uint8_t icmRead8(icm20948_t *icm, uint8_t reg, uint8_t bank);
static void icmWrite8(icm20948_t *icm, uint8_t reg, uint8_t bank, uint8_t value);
static void set_slave_write(icm20948_t *icm);
static void set_slave_read(icm20948_t *icm);
static void set_sleep(icm20948_t *icm, bool sleep); 
static void set_clock_source(icm20948_t *icm);
static void init_mag(icm20948_t *icm);  
static void set_slave_I2C_speed(icm20948_t *icm); 
static void enable_I2C_master(icm20948_t *icm);  
static uint8_t read_mag(icm20948_t *icm, uint8_t reg); 
static void write_mag(icm20948_t *icm, uint8_t reg, uint8_t value);
static void delay(uint16_t ms);

void icm20948_init(icm20948_t *icm, nbe_i2c_t *nbe_i2c, uint8_t address) {
    icm->current_bank = 13; // random invalid number since we don't know
    icm->address = address;
    icm->nbe_i2c = nbe_i2c;

    set_sleep(icm, false);
    set_clock_source(icm);

    init_mag(icm); 
}

void icm20948_start_read_all(icm20948_t *icm) {
    //they are are all in sequence starting from accel_h (plus a temperature in the middle but i don't care)
    read_register(icm, 0, ICM20948_ACCEL_XOUT_H_REG, icm->rx_buf, 20);
}

void icm20948_parse_all_raw(icm20948_t *icm, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t i = 0;
    *ax = (icm->rx_buf[i++] << 8);
    *ax += icm->rx_buf[i++];
    *ay = (icm->rx_buf[i++] << 8);
    *ay += icm->rx_buf[i++];
    *az = (icm->rx_buf[i++] << 8);
    *az += icm->rx_buf[i++];

    *gx = (icm->rx_buf[i++] << 8);
    *gx += icm->rx_buf[i++];
    *gy = (icm->rx_buf[i++] << 8);
    *gy += icm->rx_buf[i++];
    *gz = (icm->rx_buf[i++] << 8);
    *gz += icm->rx_buf[i++];
    i += 2; // skip temperature
    *mx = (icm->rx_buf[i++] << 8);
    *mx += icm->rx_buf[i++];
    *my = (icm->rx_buf[i++] << 8);
    *my += icm->rx_buf[i++];
    *mz = (icm->rx_buf[i++] << 8);
    *mz += icm->rx_buf[i++];
}

void icm20948_parse_all_metric(icm20948_t *icm, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
    int16_t rax, ray, raz, rgx, rgy, rgz, rmx, rmy, rmz;
    icm20948_parse_all_raw(icm, &rax, &ray, &raz, &rgx, &rgy, &rgz, &rmx, &rmy, &rmz);
    *ax = rax * icm->accel_scale;
    *ay = ray * icm->accel_scale;
    *az = raz * icm->accel_scale;
    *gx = rgx * icm->gyro_scale;
    *gy = rgy * icm->gyro_scale;
    *gz = rgz * icm->gyro_scale;
    *mx = rmx * 0.15;
    *my = rmy * 0.15;
    *mz = rmz * 0.15;
}

void icm20948_set_accel_range(icm20948_t *icm, enum ICM20948_ACCEL_FS_SEL range) {
    uint8_t base = icmRead8(icm, ICM20948_ACCEL_CONFIG_REG, 2); 
    base &= 0b11111001;
    base |= (range << 1);
    icmWrite8(icm, ICM20948_ACCEL_CONFIG_REG, 2, base);
    switch(range){
        case(ICM20948_ACCEL_2_G):
            icm->accel_scale = 1/16384.0;
            break;
        case(ICM20948_ACCEL_4_G):
            icm->accel_scale = 1/8192.0;
            break;
        case(ICM20948_ACCEL_8_G):
            icm->accel_scale = 1/4096.0;
            break;
        case(ICM20948_ACCEL_16_G):
            icm->accel_scale = 1/2048.0;
            break;
    }
}

void icm20948_set_gyro_range(icm20948_t *icm, enum ICM20948_GYRO_FS_SEL range) {
    uint8_t base = icmRead8(icm, ICM20948_GYRO_CONFIG_1_REG, 2); //in bank 2
    base &= 0b11111001;
    base |= range << 1;
    icmWrite8(icm, ICM20948_GYRO_CONFIG_1_REG, 2, base); //in bank 2
    switch(range) {
        case(ICM20948_GYRO_250_DPS):
            icm->gyro_scale = 1/131.0;
            break;
        case(ICM20948_GYRO_500_DPS):
            icm->gyro_scale = 1/65.5;
            break;
        case(ICM20948_GYRO_1000_DPS):
            icm->gyro_scale = 1/32.8;
            break;
        case(ICM20948_GYRO_2000_DPS):
            icm->gyro_scale = 1/16.4;
            break;
    }
}


static void write_register(icm20948_t *icm, uint8_t bank, uint8_t reg_address, uint8_t *buf, uint8_t len) {
    nbe_i2c_reset(icm->nbe_i2c);
    nbe_i2c_set_tx_buf(icm->nbe_i2c, buf);
    uint8_t tmp_buf[3];

    if (bank != icm->current_bank) {
        tmp_buf[0] = i2c_first_byte_write(icm->address);
        tmp_buf[1] = ICM20948_REG_BANK_SEL_REG;
        tmp_buf[2] = bank << 4;
        nbe_i2c_start(icm->nbe_i2c);
        nbe_i2c_write_preamble(icm->nbe_i2c, tmp_buf, 3);
    }

    nbe_i2c_start(icm->nbe_i2c);
    tmp_buf[0] = i2c_first_byte_write(icm->address);
    tmp_buf[1] = reg_address;
    nbe_i2c_write_preamble(icm->nbe_i2c, tmp_buf, 2);
    nbe_i2c_write(icm->nbe_i2c, len);
    nbe_i2c_stop(icm->nbe_i2c);
    nbe_i2c_commit(icm->nbe_i2c);
}

static void read_register(icm20948_t *icm, uint8_t bank, uint8_t reg_address, uint8_t *buf, uint8_t len) {
    nbe_i2c_reset(icm->nbe_i2c);
    nbe_i2c_set_rx_buf(icm->nbe_i2c, buf);
    uint8_t tmp_buf[3];
    
    if (bank != icm->current_bank) {
        tmp_buf[0] = i2c_first_byte_write(icm->address);
        tmp_buf[1] = ICM20948_REG_BANK_SEL_REG;
        tmp_buf[2] = bank << 4;
        nbe_i2c_start(icm->nbe_i2c);
        nbe_i2c_write_preamble(icm->nbe_i2c, tmp_buf, 3);
    }

    nbe_i2c_start(icm->nbe_i2c);
    tmp_buf[0] = i2c_first_byte_write(icm->address);
    tmp_buf[1] = reg_address;
    nbe_i2c_write_preamble(icm->nbe_i2c, tmp_buf, 2);

    nbe_i2c_start(icm->nbe_i2c);
    tmp_buf[0] = i2c_first_byte_read(icm->address);
    nbe_i2c_write_preamble(icm->nbe_i2c, tmp_buf, 1);
    nbe_i2c_read(icm->nbe_i2c, len);
    nbe_i2c_stop(icm->nbe_i2c);
    nbe_i2c_commit(icm->nbe_i2c);
}

static uint8_t icmRead8(icm20948_t *icm, uint8_t reg, uint8_t bank) {
    uint8_t byte;
    read_register(icm, bank, reg, &byte, 1);
    while(nbe_i2c_is_busy(icm->nbe_i2c)) {};

    return byte;
}

static void icmWrite8(icm20948_t *icm, uint8_t reg, uint8_t bank, uint8_t value) {
    write_register(icm, bank, reg, &value, 1);
    while(nbe_i2c_is_busy(icm->nbe_i2c)) {};
}

static void set_slave_write(icm20948_t *icm) {
    uint8_t base = icmRead8(icm, ICM20948_I2C_SLV0_ADDR_REG, 3);
    base &= 0b01111111;
    return icmWrite8(icm, ICM20948_I2C_SLV0_ADDR_REG, 3, base);
}

static void set_slave_read(icm20948_t *icm) {
    uint8_t base = icmRead8(icm, ICM20948_I2C_SLV0_ADDR_REG, 3);
    base &= 0b01111111;
    base |= 1 << 7;
    return icmWrite8(icm, ICM20948_I2C_SLV0_ADDR_REG, 3, base);
}

static void init_mag(icm20948_t *icm) {
    set_slave_I2C_speed(icm);
    enable_I2C_master(icm);

    // set the slave 0 I2C address 
    icmWrite8(icm, ICM20948_I2C_SLV0_ADDR_REG, 3, ICM20948_MAGNETOMETER_ADDR);

    //reset_mag(icm);
    //give it a while to reset!
    delay(100);
    // set to continuous mode 4 
    write_mag(icm, ICM20948_MAG_CONTROL_2_REG, 0b01000); 
    
    //set up to place data in the ext sense data slots
    set_slave_read(icm);
    // set the register to begin reading from 
    icmWrite8(icm, ICM20948_I2C_SLV0_REG_REG, 3, ICM20948_MAG_X_DATA_LOW_REG);
    // set the expected read size 
    icmWrite8(icm, ICM20948_I2C_SLV0_CTRL_REG, 3, 1 << 7 | 8);
    // extra delay for caution 
    delay(10);
}

static void set_clock_source(icm20948_t *icm) {
    // grab the register's current content
    uint8_t base = icmRead8(icm, ICM20948_PWR_MGMT_1_REG, 0);
    base &= 0b11111100;
    // modify the necessary bit
    base |= 0x01;
    // write the correct value 
    icmWrite8(icm, ICM20948_PWR_MGMT_1_REG, 0, base);
}

static void set_sleep(icm20948_t *icm, bool sleep) {
    // grab the register's current content 
    uint8_t base = icmRead8(icm, ICM20948_PWR_MGMT_1_REG, 0);
    base &= 0b10111111;
    // modify the necessary bit 
    if(sleep) base |= (1 << 6);
    // write the correct value 
    icmWrite8(icm, ICM20948_PWR_MGMT_1_REG, 0, base);
}

static void enable_I2C_master(icm20948_t *icm) {
    uint8_t base = icmRead8(icm, ICM20948_USER_CTRL_REG, 0);
    base &= 0b11011111;
    base |= 1 << 5;
    icmWrite8(icm, ICM20948_USER_CTRL_REG, 0, 1 << 5);
}

static void set_slave_I2C_speed(icm20948_t *icm) {
    icmWrite8(icm, ICM20948_I2C_MST_CTRL_REG, 3, 0x07); 
}

static void write_mag(icm20948_t *icm, uint8_t reg, uint8_t value) {
    set_slave_write(icm);
    icmWrite8(icm, ICM20948_I2C_SLV0_REG_REG, 3, reg);
    icmWrite8(icm, ICM20948_I2C_SLV0_DO_REG, 3, value);
    icmWrite8(icm, ICM20948_I2C_SLV0_CTRL_REG, 3, 1 << 7|1);
}

static uint8_t read_mag(icm20948_t *icm, uint8_t reg) {
    set_slave_read(icm);
    icmWrite8(icm, ICM20948_I2C_SLV0_REG_REG, 3, reg);
    icmWrite8(icm, ICM20948_I2C_SLV0_CTRL_REG, 3, 1 << 7 | 1);
    delay(10);
    return icmRead8(icm, ICM20948_EXT_SLV_SENS_DATA_00_REG, 0);
}

static void delay(uint16_t ms) {
    uint32_t start = esp_timer_get_time();
    while (esp_timer_get_time() < start + ms * 1000) {}
}