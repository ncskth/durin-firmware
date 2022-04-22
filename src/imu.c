#include "imu.h"
#include "durin.h"
#include "icm20948.h"
#include "pt.h"

#define I2C_WAIT() do { PT_YIELD(pt); } while (nbe_i2c_is_busy(&durin.hw.i2c_tof))

icm20948_t icm;

void init_imu() {
    icm20948_init(&icm, &durin.hw.i2c_imu, 0x68);

    icm20948_start_read_all(&icm);
    while (nbe_i2c_is_busy(&durin.hw.i2c_tof)) {};

    icm20948_parse_all_metric(&icm, 
        &durin.telemetry.ax, &durin.telemetry.ay, &durin.telemetry.az,
        &durin.telemetry.gx, &durin.telemetry.gy, &durin.telemetry.gz,
        &durin.telemetry.mx, &durin.telemetry.my, &durin.telemetry.mz);

    
    printf("big chungus %f\n", durin.telemetry.ax);    
}

void update_imu(struct pt *pt) {
    PT_BEGIN(pt);

    while (1) {
        //icm20948_start_read_all(&icm);
        I2C_WAIT();
        icm20948_parse_all_metric(&icm, 
                &durin.telemetry.ax, &durin.telemetry.ay, &durin.telemetry.az,
                &durin.telemetry.gx, &durin.telemetry.gy, &durin.telemetry.gz,
                &durin.telemetry.mx, &durin.telemetry.my, &durin.telemetry.mz);
    }
    PT_END(pt);
}