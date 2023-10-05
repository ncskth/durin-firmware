#include "imu.h"
#include "durin.h"
#include "icm20948.h"
#include "pt.h"
#include "protocol.h"
#include "max1704x.h"

icm20948_t icm;
max1704x_t batman;


void max_read(uint8_t *buf, uint8_t len, uint8_t addr, uint8_t reg, void* ctx) {
    nbe_i2c_full_register_read(ctx, addr, reg, NBE_I2C_REGISTER_8, buf, len);
    while (nbe_i2c_is_busy(ctx)) {}
}

void max_read_non_blocking(uint8_t *buf, uint8_t len, uint8_t addr, uint8_t reg, void* ctx) {
    nbe_i2c_full_register_read(ctx, addr, reg, NBE_I2C_REGISTER_8, buf, len);
}

void max_write(uint8_t *buf, uint8_t len, uint8_t addr, uint8_t reg, void* ctx) {
    nbe_i2c_full_register_write(ctx, addr, reg, NBE_I2C_REGISTER_8, buf, len);
    while (nbe_i2c_is_busy(ctx)) {}
}



void init_imu() {
    printf("starting init icm20948\n");
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    int err = icm20948_init(&icm, &durin.hw.i2c_imu, 0x68);
    if (err < 0) {
        printf("icm20948 returned %d for address 0x68\n", err);
        // AAAAAAAAAAH I SOLDERED ONE WRONG
        // test address 69 as well :(
        err = icm20948_init(&icm, &durin.hw.i2c_imu, 0x69);
        if (err < 0) {
            printf("icm20948 returned %d for address 0x69\n", err);
            set_led(PINK);
        }
    }
    printf("icm20948 initialized\n");
    icm20948_configAccel(&icm, ICM20948_ACCEL_RANGE_4G, ICM20948_ACCEL_DLPF_BANDWIDTH_473HZ);
    icm20948_configGyro(&icm, ICM20948_GYRO_RANGE_500DPS, ICM20948_GYRO_DLPF_BANDWIDTH_361HZ);
    icm20948_setAccelSrd(&icm, 0);
    icm20948_setGyroSrd(&icm, 0);

    icm20948_readSensorSync(&icm);

    icm20948_parseAllRaw(&icm,
        &durin.telemetry.raw_ax, &durin.telemetry.raw_ay, &durin.telemetry.raw_az,
        &durin.telemetry.raw_gx, &durin.telemetry.raw_gy, &durin.telemetry.raw_gz,
        &durin.telemetry.raw_mx, &durin.telemetry.raw_my, &durin.telemetry.raw_mz);

    icm20948_parseAllMetric(&icm,
        &durin.telemetry.ax, &durin.telemetry.ay, &durin.telemetry.az,
        &durin.telemetry.gx, &durin.telemetry.gy, &durin.telemetry.gz,
        &durin.telemetry.mx, &durin.telemetry.my, &durin.telemetry.mz);

    printf("imu raw: %d %d %d %d %d %d %d %d %d\n",
        durin.telemetry.raw_ax, durin.telemetry.raw_ay, durin.telemetry.raw_az,
        durin.telemetry.raw_gx, durin.telemetry.raw_gy, durin.telemetry.raw_gz,
        durin.telemetry.raw_mx, durin.telemetry.raw_my, durin.telemetry.raw_mz);


    printf("Init battery monitor");
    max1704x_platform_t batman_platform;

    batman_platform.ctx = &durin.hw.i2c_imu;
    batman_platform.read = max_read;
    batman_platform.write = max_write;
    batman_platform.read_non_blocking = max_read_non_blocking;

    max1704x_begin(&batman, MAX1704X_ADDR, batman_platform, true);
}

void send_imu_telemetry() {
    struct capn c;
    struct capn_segment *cs;
    struct DurinBase msg;
    init_durinbase(&c, &cs, &msg);
    struct ImuMeasurement data;
    data.accelerometerX = durin.telemetry.raw_ax;
    data.accelerometerY = durin.telemetry.raw_ay;
    data.accelerometerZ = durin.telemetry.raw_az;
    data.gyroscopeX = durin.telemetry.raw_gx;
    data.gyroscopeY = durin.telemetry.raw_gy;
    data.gyroscopeZ = durin.telemetry.raw_gz;
    data.magnetometerX = durin.telemetry.raw_mx;
    data.magnetometerY = durin.telemetry.raw_my;
    data.magnetometerZ = durin.telemetry.raw_mz;
    msg.imuMeasurement = new_ImuMeasurement(cs);
    write_ImuMeasurement(&data, msg.imuMeasurement);
    msg.which = DurinBase_imuMeasurement;

    uint8_t buf[256];
    uint16_t len = sizeof(buf);
    finish_durinbase(&c, &cs, &msg, buf, &len);
    send_telemetry(buf, len);
}

void update_imu(struct pt *pt) {
    PT_BEGIN(pt);
    static uint64_t last_telemetry_update = 0;
    static uint64_t last_batman_sample = 0;
    while (1) {
        icm20948_readSensorAsync(&icm);
        // icm20948_readSensorSync(&icm);
        do { PT_YIELD(pt); } while (nbe_i2c_is_busy(&durin.hw.i2c_imu));
        icm20948_parseAllRaw(&icm,
            &durin.telemetry.raw_ax, &durin.telemetry.raw_ay, &durin.telemetry.raw_az,
            &durin.telemetry.raw_gx, &durin.telemetry.raw_gy, &durin.telemetry.raw_gz,
            &durin.telemetry.raw_mx, &durin.telemetry.raw_my, &durin.telemetry.raw_mz);
        icm20948_parseAllMetric(&icm,
            &durin.telemetry.ax, &durin.telemetry.ay, &durin.telemetry.az,
            &durin.telemetry.gx, &durin.telemetry.gy, &durin.telemetry.gz,
            &durin.telemetry.mx, &durin.telemetry.my, &durin.telemetry.mz);
        if (esp_timer_get_time() - last_telemetry_update > durin.info.imu_stream_period * 1000) {
            last_telemetry_update = esp_timer_get_time();
            send_imu_telemetry();
        }

        if (esp_timer_get_time() - last_batman_sample > 1000000) {
            last_batman_sample = esp_timer_get_time();
            float volt;
            float charge;
            float discharge;
            max1704x_get_interesting_async_start(&batman);
            do { PT_YIELD(pt); } while (nbe_i2c_is_busy(&durin.hw.i2c_imu));
            max1704x_get_interesting_async_end(&batman, &volt, &charge, &discharge);
            durin.telemetry.battery_charge_percent = charge;
            durin.telemetry.battery_discharge_rate = discharge;
            durin.telemetry.battery_voltage = volt;
            // printf("non blocking %f %f %f\n", volt, charge, discharge);
        }

    }
    PT_END(pt);
}