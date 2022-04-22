#include "nbe_i2c.h"

//stole all defines from
//https://github.com/jeremymdunne/Arduino_ICM20948/blob/master/lib/Arduino_ICM20948/Arduino_ICM20948.h



#define ICM20948_MAGNETOMETER_ADDR              0x0C


// I2C Address specifications 
#define ICM20948_I2C_ADDRESS_ADO_LOW            0x68
#define ICM20948_I2C_ADDRESS_ADO_HIGH           0x69        
#define ICM20948_I2C_SPEED                      400000L
#define ICM20948_MAGNETOMETER_ADDR              0x0C

// register definitions 
// some are not currently used and therefore not defined 
// all High and Low register pairs are only mentioned by the High address 

// Bank 0 Reg adresses   
#define ICM20948_WHO_AM_I_REG                   0x00
#define ICM20948_USER_CTRL_REG                  0x03
#define ICM20948_LP_CONFIG_REG                  0x05
#define ICM20948_PWR_MGMT_1_REG                 0x06
#define ICM20948_PWR_MGMT_2_REG                 0x07
#define ICM20948_INT_PIN_CFG_REG                0x0F
#define ICM20948_INT_ENABLE_REG                 0x10
#define ICM20948_INT_ENABLE_1_REG               0x11
#define ICM20948_INT_ENABLE_2_REG               0x12
#define ICM20948_INT_ENABLE_3_REG               0x13
#define ICM20948_I2C_MST_STATUS_REG             0x17
#define ICM20948_INT_STATUS_REG                 0x19
#define ICM20948_INT_STATUS_1_REG               0x1A
#define ICM20948_INT_STATUS_2_REG               0x1B
#define ICM20948_INT_STATUS_3_REG               0x1C
#define ICM20948_DELAY_TIMEH_REG                0x28
#define ICM20948_DELAY_TIMEL_REG                0x29
#define ICM20948_ACCEL_XOUT_H_REG               0x2D
#define ICM20948_ACCEL_YOUT_H_REG               0x2F
#define ICM20948_ACCEL_ZOUT_H_REG               0x31
#define ICM20948_GYRO_XOUT_H_REG                0x33
#define ICM20948_GYRO_YOUT_H_REG                0x35
#define ICM20948_GYRO_ZOUT_H_REG                0x37
#define ICM20948_TEMP_OUT_H_REG                 0x39
#define ICM20948_EXT_SLV_SENS_DATA_00_REG       0x3B 

#define ICM20948_FIFO_EN_1_REG                  0x66
#define ICM20948_FIFO_EN_2_REG                  0x67
#define ICM20948_FIFO_RST_REG                   0x68
#define ICM20948_FIFO_MODE_REG                  0x69
#define ICM20948_FIFO_COUNTH_REG                0x70
#define ICM20948_FIFO_R_W_REG                   0x72
#define ICM20948_DATA_RDY_STATUS_REG            0x74
#define ICM20948_FIFO_CFG_REG                   0x76
#define ICM20948_REG_BANK_SEL_REG               0x7F

// Bank 1 Addresses 
#define ICM20948_SELF_TEST_X_GYRO_REG           0x02 
#define ICM20948_SELF_TEST_Y_GYRO_REG           0x03
#define ICM20948_SELF_TEST_Z_GYRO_REG           0x04
#define ICM20948_SELF_TEST_X_ACCEL_REG          0x0E
#define ICM20948_SELF_TEST_Y_ACCEL_REG          0x0F
#define ICM20948_SELF_TEST_Z_ACCEL_REG          0x10
#define ICM20948_XA_OFFS_H_REG                  0x14
#define ICM20948_YA_OFFS_H_REG                  0x17
#define ICM20948_ZA_OFFS_H_REG                  0x1A
#define ICM20948_TIMEBASE_CORRECTION_PLL_REG    0x28 

// Bank 2 Addresses 
#define ICM20948_GYRO_SMPLRT_DIV_REG            0x00
#define ICM20948_GYRO_CONFIG_1_REG              0x01
#define ICM20948_GYRO_CONFIG_2_REG              0x02
#define ICM20948_XG_OFFS_USRH_REG               0x03 
#define ICM20948_YG_OFFS_USRH_REG               0x05
#define ICM20948_ZG_OFFS_USRH_REG               0x07 
#define ICM20948_ODR_ALIGN_EN_REG               0x09 
#define ICM20948_ACCEL_SMPLRT_DIV_1_REG         0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2_REG         0x11
#define ICM20948_ACCEL_INTEL_CTRL_REG           0x12 
#define ICM20948_ACCEL_WOM_THR_CTRL_REG         0x13 
#define ICM20948_ACCEL_CONFIG_REG               0x14 
#define ICM20948_ACCEL_CONFIG_2_REG             0x15
#define ICM20948_FSYNC_CONFIG_REG               0x52
#define ICM20948_TEMP_CONFIG_REG                0x53 
#define ICM20948_MOD_CTRL_USR_REG               0x54 


// Bank 3 Addresses 
#define ICM20948_I2C_MST_ODR_CONFIG_REG         0x00
#define ICM20948_I2C_MST_CTRL_REG               0x01
#define ICM20948_I2C_MST_DELAY_CTRL_REG         0x02
#define ICM20948_I2C_SLV0_ADDR_REG              0x03
#define ICM20948_I2C_SLV0_REG_REG               0x04
#define ICM20948_I2C_SLV0_CTRL_REG              0x05
#define ICM20948_I2C_SLV0_DO_REG                0x06


// Magnetometer 
#define ICM20948_MAG_DEVICE_ID_REG              0x01
#define ICM20948_MAG_STATUS_1_REG               0x10
#define ICM20948_MAG_X_DATA_LOW_REG             0x11
#define ICM20948_MAG_Y_DATA_LOW_REG             0x13
#define ICM20948_MAG_Z_DATA_LOW_REG             0x15 
#define ICM20948_MAG_STATUS_2_REG               0x18
#define ICM20948_MAG_CONTROL_2_REG              0x31 
#define ICM20948_MAG_CONTROL_3_REG              0x32 



typedef struct icm20948 {
    nbe_i2c_t *nbe_i2c;
    uint8_t address;
    uint8_t rx_buf[25];
    uint8_t tx_buf[25];
    float gyro_scale;
    float accel_scale;
    uint8_t current_bank;
} icm20948_t;

enum ICM20948_GYRO_FS_SEL{
    ICM20948_GYRO_250_DPS = 0x00,
    ICM20948_GYRO_500_DPS = 0x01, 
    ICM20948_GYRO_1000_DPS = 0x02,
    ICM20948_GYRO_2000_DPS = 0x03  
};

// accel range select 
enum ICM20948_ACCEL_FS_SEL{ 
    ICM20948_ACCEL_2_G = 0x00,
    ICM20948_ACCEL_4_G = 0x01,
    ICM20948_ACCEL_8_G = 0x02,
    ICM20948_ACCEL_16_G = 0x03
}; 

void icm20948_init(icm20948_t *icm, nbe_i2c_t *nbe_i2c, uint8_t address);

void icm20948_set_accel_range(icm20948_t *icm, enum ICM20948_ACCEL_FS_SEL range);
void icm20948_set_gyro_range(icm20948_t *icm, enum ICM20948_GYRO_FS_SEL range);

void icm20948_start_read_all(icm20948_t *icm);
void icm20948_parse_all_metric(icm20948_t *icm, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz);
void icm20948_parse_all_raw(icm20948_t *icm, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz);

