#include "nbe_i2c.h"

enum Icm20948GyroRange {
    ICM20948_GYRO_RANGE_250DPS,
    ICM20948_GYRO_RANGE_500DPS,
    ICM20948_GYRO_RANGE_1000DPS,
    ICM20948_GYRO_RANGE_2000DPS
};

enum Icm20948AccelRange
{
    ICM20948_ACCEL_RANGE_2G,
    ICM20948_ACCEL_RANGE_4G,
    ICM20948_ACCEL_RANGE_8G,
    ICM20948_ACCEL_RANGE_16G    
};

enum Icm20948AccelDlpfBandwidth {
    ICM20948_ACCEL_DLPF_BANDWIDTH_1209HZ,
    ICM20948_ACCEL_DLPF_BANDWIDTH_246HZ,
    ICM20948_ACCEL_DLPF_BANDWIDTH_111HZ,
    ICM20948_ACCEL_DLPF_BANDWIDTH_50HZ,
    ICM20948_ACCEL_DLPF_BANDWIDTH_24HZ,
    ICM20948_ACCEL_DLPF_BANDWIDTH_12HZ,
    ICM20948_ACCEL_DLPF_BANDWIDTH_6HZ,
    ICM20948_ACCEL_DLPF_BANDWIDTH_473HZ
};

enum Icm20948GyroDlpfBandwidth {
    ICM20948_GYRO_DLPF_BANDWIDTH_12106HZ,
    ICM20948_GYRO_DLPF_BANDWIDTH_197HZ,
    ICM20948_GYRO_DLPF_BANDWIDTH_152HZ,
    ICM20948_GYRO_DLPF_BANDWIDTH_120HZ,
    ICM20948_GYRO_DLPF_BANDWIDTH_51HZ,
    ICM20948_GYRO_DLPF_BANDWIDTH_24HZ,
    ICM20948_GYRO_DLPF_BANDWIDTH_12HZ,
    ICM20948_GYRO_DLPF_BANDWIDTH_6HZ,
    ICM20948_GYRO_DLPF_BANDWIDTH_361HZ
};

typedef struct icm20948 {
    nbe_i2c_t *nbe_i2c;
    uint8_t address;
    uint8_t rxBuf[25];
    uint8_t txBuf[25];
    float gyroScale;
    float accelScale;
    uint8_t currentBank;
} icm20948_t;

int icm20948_init(icm20948_t *icm, nbe_i2c_t *nbe_i2c, uint8_t address);
int icm20948_configAccel(icm20948_t *icm, enum Icm20948AccelRange range, enum Icm20948AccelDlpfBandwidth bandwidth);
int icm20948_configGyro(icm20948_t *icm, enum Icm20948GyroRange range, enum Icm20948GyroDlpfBandwidth bandwidth);
int icm20948_setGyroSrd(icm20948_t *icm, uint8_t srd);
int icm20948_setAccelSrd(icm20948_t *icm, uint16_t srd);
int icm20948_enableDataReadyInterrupt(icm20948_t *icm);
int icm20948_disableDataReadyInterrupt(icm20948_t *icm);
int icm20948_readSensorSync(icm20948_t *icm);
int icm20948_readSensorAsync(icm20948_t *icm);
void icm20948_parseAllRaw(icm20948_t *icm, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz);
// G, deg/s and uT
void icm20948_parseAllMetric(icm20948_t *icm, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz);