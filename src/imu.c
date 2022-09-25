#include "imu.h"
#include "durin.h"
#include "icm20948.h"
#include "pt.h"

icm20948_t icm;

void init_imu() {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    icm20948_init(&icm, &durin.hw.i2c_imu, 0x68);
    vTaskDelay(100 / portTICK_PERIOD_MS); // this works but i dont have time to debug why...

    icm20948_configAccel(&icm, ICM20948_ACCEL_RANGE_16G, ICM20948_ACCEL_DLPF_BANDWIDTH_473HZ);
    icm20948_configGyro(&icm, ICM20948_GYRO_RANGE_2000DPS, ICM20948_GYRO_DLPF_BANDWIDTH_12106HZ);
    icm20948_setAccelSrd(&icm, 0);
    icm20948_setGyroSrd(&icm, 0);

    icm20948_readSensorSync(&icm);

    icm20948_parseAllRaw(&icm, 
        &durin.telemetry.ax, &durin.telemetry.ay, &durin.telemetry.az,
        &durin.telemetry.gx, &durin.telemetry.gy, &durin.telemetry.gz,
        &durin.telemetry.mx, &durin.telemetry.my, &durin.telemetry.mz);

    icm20948_parseAllMetric(&icm, &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    printf("accelerometer metric: %f %f %f %f %f %f %f %f %f\n", ax, ay, az, gx, gy, gz, mx, my, mz);

    printf("accelerometer raw: %d %d %d %d %d %d %d %d %d\n",
            durin.telemetry.ax, durin.telemetry.ay, durin.telemetry.az,
            durin.telemetry.gx, durin.telemetry.gy, durin.telemetry.gz,
            durin.telemetry.mx, durin.telemetry.my, durin.telemetry.mz);
}

void update_imu(struct pt *pt) {
    PT_BEGIN(pt);

    while (1) {
        icm20948_readSensorAsync(&icm);
        do { PT_YIELD(pt); } while (nbe_i2c_is_busy(&durin.hw.i2c_imu));
        icm20948_parseAllRaw(&icm, 
                &durin.telemetry.ax, &durin.telemetry.ay, &durin.telemetry.az,
                &durin.telemetry.gx, &durin.telemetry.gy, &durin.telemetry.gz,
                &durin.telemetry.mx, &durin.telemetry.my, &durin.telemetry.mz);
    }
    PT_END(pt);
}