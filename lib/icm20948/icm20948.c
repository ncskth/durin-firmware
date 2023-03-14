/*
ICM20948.cpp
Copyright (c) 2019 David Törnqvist
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

#include "icm20948.h"

enum UserBank {
    USER_BANK_0,
    USER_BANK_1,
    USER_BANK_2,
    USER_BANK_3,
};

const uint8_t ICM20948_WHO_AM_I = 0xEA;

const uint8_t UB0_WHO_AM_I = 0x00;
const uint8_t UB0_USER_CTRL = 0x03;
const uint8_t UB0_USER_CTRL_I2C_MST_EN = 0x20;

const uint8_t UB0_PWR_MGMNT_1 = 0x06;
const uint8_t UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO = 0x01;
const uint8_t UB0_PWR_MGMNT_1_DEV_RESET = 0x80;

const uint8_t UB0_PWR_MGMNT_2 = 0x07;
const uint8_t UB0_PWR_MGMNT_2_SEN_ENABLE = 0x00;

const uint8_t UB0_INT_PIN_CFG = 0x0F;
const uint8_t UB0_INT_PIN_CFG_HIGH_50US = 0x00;

const uint8_t UB0_INT_ENABLE_1 = 0x11;
const uint8_t UB0_INT_ENABLE_1_RAW_RDY_EN = 0x01;
const uint8_t UB0_INT_ENABLE_1_DIS = 0x00;

const uint8_t UB0_ACCEL_XOUT_H = 0x2D;

const uint8_t UB0_EXT_SLV_SENS_DATA_00 = 0x3B;

// User bank 2
const uint8_t UB2_GYRO_SMPLRT_DIV = 0x00;

const uint8_t UB2_GYRO_CONFIG_1 = 0x01;
const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_250DPS = 0x00;
const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_500DPS = 0x02;
const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_1000DPS = 0x04;
const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_2000DPS = 0x06;
const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_12106HZ = 0x00;
const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_197HZ = 0x00 | 0x01;
const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_152HZ = 0b00001000 | 0x01;
const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_120HZ = 0b00010000 | 0x01;
const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_51HZ = 0b00011000 | 0x01;
const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_24HZ = 0b00100000 | 0x01;
const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_12HZ = 0b00101000 | 0x01;
const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_6HZ = 0b00110000 | 0x01;
const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_361HZ = 0b00111000 | 0x01;

const uint8_t UB2_ACCEL_SMPLRT_DIV_1 = 0x10;
const uint8_t UB2_ACCEL_SMPLRT_DIV_2 = 0x11;

const uint8_t UB2_ACCEL_CONFIG = 0x14;
const uint8_t UB2_ACCEL_CONFIG_FS_SEL_2G = 0x00;
const uint8_t UB2_ACCEL_CONFIG_FS_SEL_4G = 0x02;
const uint8_t UB2_ACCEL_CONFIG_FS_SEL_8G = 0x04;
const uint8_t UB2_ACCEL_CONFIG_FS_SEL_16G = 0x06;
const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_1209HZ = 0x00;
const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_246HZ = 0x00 | 0x01;
const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_111HZ = 0b00010000 | 0x01;
const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_50HZ = 0b00011000 | 0x01;
const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_24HZ = 0b00100000 | 0x01;
const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_12HZ = 0b00101000 | 0x01;
const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_6HZ = 0b00110000 | 0x01;
const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_473HZ = 0b00111000 | 0x01;

// User bank 3
const uint8_t UB3_I2C_MST_CTRL = 0x01;
const uint8_t UB3_I2C_MST_CTRL_CLK_400KHZ = 0x07;  // Gives 345.6kHz and is recommended to achieve max 400kHz

const uint8_t UB3_I2C_SLV0_ADDR = 0x03;
const uint8_t UB3_I2C_SLV0_ADDR_READ_FLAG = 0x80;

const uint8_t UB3_I2C_SLV0_REG = 0x04;

const uint8_t UB3_I2C_SLV0_CTRL = 0x05;
const uint8_t UB3_I2C_SLV0_CTRL_EN = 0x80;

const uint8_t UB3_I2C_SLV0_DO = 0x06;

// Common to all user banks
const uint8_t REG_BANK_SEL = 0x7F;
const uint8_t REG_BANK_SEL_USER_BANK_0 = 0x00;
const uint8_t REG_BANK_SEL_USER_BANK_1 = 0x10;
const uint8_t REG_BANK_SEL_USER_BANK_2 = 0x20;
const uint8_t REG_BANK_SEL_USER_BANK_3 = 0x30;

// Magnetometer constants
const uint8_t MAG_AK09916_I2C_ADDR = 0x0C;
const uint16_t MAG_AK09916_WHO_AM_I = 0x4809;
const uint8_t MAG_DATA_LENGTH = 8;  // Bytes

// Magnetometer (AK09916) registers
const uint8_t MAG_WHO_AM_I = 0x00;

const uint8_t MAG_HXL = 0x11;

const uint8_t MAG_CNTL2 = 0x31;
const uint8_t MAG_CNTL2_POWER_DOWN = 0x00;
const uint8_t MAG_CNTL2_MODE_10HZ = 0x02;
const uint8_t MAG_CNTL2_MODE_50HZ = 0x06;
const uint8_t MAG_CNTL2_MODE_100HZ = 0x08;

const uint8_t MAG_CNTL3 = 0x32;
const uint8_t MAG_CNTL3_RESET = 0x01;

const float accRawScaling = 32767.5f;   // =(2^16-1)/2 16 bit representation of acc value to cover +/- range
const float gyroRawScaling = 32767.5f;  // =(2^16-1)/2 16 bit representation of gyro value to cover +/- range
const float magRawScaling = 32767.5f;   // =(2^16-1)/2 16 bit representation of gyro value to cover +/- range

int enableI2cMaster(icm20948_t *icm);
int selectAutoClockSource(icm20948_t *icm);
int enableAccelGyro(icm20948_t *icm);
int reset(icm20948_t *icm);
int changeUserBankForce(icm20948_t *icm, enum UserBank userBank, uint8_t force);
int changeUserBank(icm20948_t *icm, enum UserBank userBank);
int writeRegister(icm20948_t *icm, uint8_t subAddress, uint8_t data);
int readRegisters(icm20948_t *icm, uint8_t subAddress, uint8_t count, uint8_t* dest);
int readRegistersAsync(icm20948_t *icm, uint8_t subAddress, enum UserBank userBank, uint8_t count, uint8_t *dest);
int writeMagRegister(icm20948_t *icm, uint8_t subAddress, uint8_t data);
int readMagRegisters(icm20948_t *icm, uint8_t subAddress, uint8_t count, uint8_t* dest);
int whoAmI(icm20948_t *icm);
int whoAmIMag(icm20948_t *icm);
int powerDownMag(icm20948_t *icm);
int resetMag(icm20948_t *icm);
int configMag(icm20948_t *icm);
void delay(uint16_t ms);

/* starts communication with the ICM-20948 */
int icm20948_init(icm20948_t *icm, nbe_i2c_t *nbe_i2c, uint8_t address) {
    icm->address = address;
    icm->nbe_i2c = nbe_i2c;
    if (changeUserBankForce(icm, USER_BANK_0, 1) < 0) {  // Make sure that the user bank selection is in sync
        return -1;
    }

    if (selectAutoClockSource(icm) < 0) {  // TODO: Why set clock source here? It is resetted anyway...
        return -1;
    }
    // enable I2C master mode
    if (enableI2cMaster(icm) < 0) {
        return -2;
    }
    if (powerDownMag(icm) < 0) {
        return -3;
    }
    reset(icm);     // reset the ICM20948. Don't check return value as a reset clears the register and can't be verified.
    delay(10);       // wait for ICM-20948 to come back up
    resetMag(icm);  // Don't check return value as a reset clears the register and can't be verified.
    delay(10);
    if (selectAutoClockSource(icm) < 0) {
        return -6;
    }
    if (whoAmI(icm) != ICM20948_WHO_AM_I) {
        return -7;
    }
    if (enableAccelGyro(icm) < 0) {
        return -8;
    }
    if (icm20948_configAccel(icm, ICM20948_ACCEL_RANGE_16G, ICM20948_ACCEL_DLPF_BANDWIDTH_246HZ) < 0) {
        return -9;
    }
    if (icm20948_configGyro(icm, ICM20948_GYRO_RANGE_2000DPS, ICM20948_GYRO_DLPF_BANDWIDTH_197HZ) < 0) {
        return -10;
    }
    if (icm20948_setGyroSrd(icm, 0) < 0) {
        return -11;
    }
    if (icm20948_setAccelSrd(icm, 0) < 0) {
        return -12;
    }
    if (enableI2cMaster(icm) < 0) {
        return -13;
    }
    if (whoAmIMag(icm) != MAG_AK09916_WHO_AM_I) {
        return -14;
    }
    if (configMag(icm) < 0) {
        return -18;
    }
    if (selectAutoClockSource(icm) < 0) {  // TODO: Why do this again here?
        return -19;
    }
    readMagRegisters(icm, MAG_HXL, MAG_DATA_LENGTH, icm->rxBuf);  // instruct the ICM20948 to get data from the magnetometer at the sample rate
    return 1;
}

int enableI2cMaster(icm20948_t *icm) {
    if (changeUserBank(icm, USER_BANK_0) < 0) {
        return -1;
    }
    if (writeRegister(icm, UB0_USER_CTRL, UB0_USER_CTRL_I2C_MST_EN) < 0) {
        return -2;
    }
    if (changeUserBank(icm, USER_BANK_3) < 0) {
        return -3;
    }
    if (writeRegister(icm, UB3_I2C_MST_CTRL, UB3_I2C_MST_CTRL_CLK_400KHZ) < 0) {
        return -4;
    }
    return 1;
}

/* enables the data ready interrupt */
int enableDataReadyInterrupt(icm20948_t *icm) {
    if (changeUserBank(icm, USER_BANK_0) < 0) {
        return -1;
    }
    if (writeRegister(icm, UB0_INT_PIN_CFG, UB0_INT_PIN_CFG_HIGH_50US) < 0) {  // setup interrupt, 50 us pulse
        return -2;
    }
    if (writeRegister(icm, UB0_INT_ENABLE_1, UB0_INT_ENABLE_1_RAW_RDY_EN) < 0) {  // set to data ready
        return -3;
    }
    return 1;
}

/* disables the data ready interrupt */
int disableDataReadyInterrupt(icm20948_t *icm) {
    if (changeUserBank(icm, USER_BANK_0) < 0) {
        return -1;
    }
    if (writeRegister(icm, UB0_INT_ENABLE_1, UB0_INT_ENABLE_1_DIS) < 0) {  // disable interrupt
        return -1;
    }
    return 1;
}

int reset(icm20948_t *icm) {
    if (changeUserBank(icm, USER_BANK_0) < 0) {
        return -1;
    }
    if (writeRegister(icm, UB0_PWR_MGMNT_1, UB0_PWR_MGMNT_1_DEV_RESET) < 0) {
        return -2;
    }
    return 1;
}

int selectAutoClockSource(icm20948_t *icm) {
    if (changeUserBank(icm, USER_BANK_0) < 0 || writeRegister(icm, UB0_PWR_MGMNT_1, UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO) < 0) {
        return -1;
    }
    return 1;
}

int enableAccelGyro(icm20948_t *icm) {
    if (changeUserBank(icm, USER_BANK_0) < 0 || writeRegister(icm, UB0_PWR_MGMNT_2, UB0_PWR_MGMNT_2_SEN_ENABLE) < 0) {
        return -1;
    }
    return 1;
}

int icm20948_configAccel(icm20948_t *icm, enum Icm20948AccelRange range, enum Icm20948AccelDlpfBandwidth bandwidth) {
    if (changeUserBank(icm, USER_BANK_2) < 0) {
        return -1;
    }
    uint8_t accelRangeRegValue = 0x00;
    switch (range) {
        case ICM20948_ACCEL_RANGE_2G: {
            accelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_2G;
            icm->accelScale = 2.0f / accRawScaling;
            break;
        }
        case ICM20948_ACCEL_RANGE_4G: {
            accelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_4G;
            icm->accelScale = 4.0f / accRawScaling;
            break;
        }
        case ICM20948_ACCEL_RANGE_8G: {
            accelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_8G;
            icm->accelScale = 8.0f / accRawScaling;
            break;
        }
        case ICM20948_ACCEL_RANGE_16G: {
            accelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_16G;
            icm->accelScale = 16.0f / accRawScaling;  // setting the accel scale to 16G
            break;
        }
    }
    uint8_t dlpfRegValue = 0x00;
    switch (bandwidth) {
        case ICM20948_ACCEL_DLPF_BANDWIDTH_1209HZ:
            dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_1209HZ;
            break;
        case ICM20948_ACCEL_DLPF_BANDWIDTH_246HZ:
            dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_246HZ;
            break;
        case ICM20948_ACCEL_DLPF_BANDWIDTH_111HZ:
            dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_111HZ;
            break;
        case ICM20948_ACCEL_DLPF_BANDWIDTH_50HZ:
            dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_50HZ;
            break;
        case ICM20948_ACCEL_DLPF_BANDWIDTH_24HZ:
            dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_24HZ;
            break;
        case ICM20948_ACCEL_DLPF_BANDWIDTH_12HZ:
            dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_12HZ;
            break;
        case ICM20948_ACCEL_DLPF_BANDWIDTH_6HZ:
            dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_6HZ;
            break;
        case ICM20948_ACCEL_DLPF_BANDWIDTH_473HZ:
            dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_473HZ;
            break;
    }
    if (writeRegister(icm, UB2_ACCEL_CONFIG, accelRangeRegValue | dlpfRegValue) < 0) {
        return -1;
    }
    return 1;
}

/* sets the gyro full scale range to values other than default */
int icm20948_configGyro(icm20948_t *icm, enum Icm20948GyroRange range, enum Icm20948GyroDlpfBandwidth bandwidth) {
    if (changeUserBank(icm, USER_BANK_2) < 0) {
        return -1;
    }
    uint8_t gyroConfigRegValue = 0x00;
    switch (range) {
        case ICM20948_GYRO_RANGE_250DPS: {
            gyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_250DPS;
            icm->gyroScale = 250.0f / gyroRawScaling;  // setting the gyro scale to 250DPS
            break;
        }
        case ICM20948_GYRO_RANGE_500DPS: {
            gyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_500DPS;
            icm->gyroScale = 500.0f / gyroRawScaling;  // setting the gyro scale to 500DPS
            break;
        }
        case ICM20948_GYRO_RANGE_1000DPS: {
            gyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_1000DPS;
            icm->gyroScale = 1000.0f / gyroRawScaling;  // setting the gyro scale to 1000DPS
            break;
        }
        case ICM20948_GYRO_RANGE_2000DPS: {
            gyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_2000DPS;
            icm->gyroScale = 2000.0f / gyroRawScaling;  // setting the gyro scale to 2000DPS
            break;
        }
    }
    uint8_t dlpfRegValue = 0x00;
    switch (bandwidth) {
        case ICM20948_GYRO_DLPF_BANDWIDTH_12106HZ:
            dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_12106HZ;
            break;
        case ICM20948_GYRO_DLPF_BANDWIDTH_197HZ:
            dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_197HZ;
            break;
        case ICM20948_GYRO_DLPF_BANDWIDTH_152HZ:
            dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_152HZ;
            break;
        case ICM20948_GYRO_DLPF_BANDWIDTH_120HZ:
            dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_120HZ;
            break;
        case ICM20948_GYRO_DLPF_BANDWIDTH_51HZ:
            dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_51HZ;
            break;
        case ICM20948_GYRO_DLPF_BANDWIDTH_24HZ:
            dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_24HZ;
            break;
        case ICM20948_GYRO_DLPF_BANDWIDTH_12HZ:
            dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_12HZ;
            break;
        case ICM20948_GYRO_DLPF_BANDWIDTH_6HZ:
            dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_6HZ;
            break;
        case ICM20948_GYRO_DLPF_BANDWIDTH_361HZ:
            dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_361HZ;
            break;
    }
    if (writeRegister(icm, UB2_GYRO_CONFIG_1, gyroConfigRegValue | dlpfRegValue) < 0) {
        return -1;
    }
    return 1;
}

int configMag(icm20948_t *icm) {  // TODO: Add possibility to use other modes
    if (writeMagRegister(icm, MAG_CNTL2, MAG_CNTL2_MODE_100HZ) < 0) {
        return -1;
    }
    return 1;
}

int icm20948_setGyroSrd(icm20948_t *icm, uint8_t srd) {
    if (changeUserBank(icm, USER_BANK_2) < 0 || writeRegister(icm, UB2_GYRO_SMPLRT_DIV, srd) < 0) {
        return -1;
    }
    return 1;
}

int icm20948_setAccelSrd(icm20948_t *icm, uint16_t srd) {
    if (changeUserBank(icm, USER_BANK_2) < 0) {
        return -1;
    }
    uint8_t srdHigh = srd >> 8 & 0x0F;  // Only last 4 bits can be set
    if (writeRegister(icm, UB2_ACCEL_SMPLRT_DIV_1, srdHigh) < 0) {
        return -1;
    }
    uint8_t srdLow = srd & 0x0F;  // Only last 4 bits can be set
    if (writeRegister(icm, UB2_ACCEL_SMPLRT_DIV_2, srdLow) < 0) {
        return -1;
    }
    return 1;
}

/* reads the most current data from MPU9250 and stores in buffer */
int icm20948_readSensor(icm20948_t *icm) {
    if (changeUserBank(icm, USER_BANK_0) < 0) {
        return -1;
    }
    if (readRegisters(icm, UB0_ACCEL_XOUT_H, 20, icm->rxBuf) < 0) {
        return -1;
    }

    return 1;
}

/* gets the WHO_AM_I register value, expected to be 0xEA */
int whoAmI(icm20948_t *icm) {
    if (changeUserBank(icm, USER_BANK_0) < 0) {
        return -1;
    }
    // read the WHO AM I register
    if (readRegisters(icm, UB0_WHO_AM_I, 1, icm->rxBuf) < 0) {
        return -1;
    }
    // return the register value
    return icm->rxBuf[0];
}

int whoAmIMag(icm20948_t *icm) {
    if (readMagRegisters(icm, MAG_WHO_AM_I, 2, icm->rxBuf) < 0) {
        return -1;
    }
    return (icm->rxBuf[0] << 8) + icm->rxBuf[1];
}

int powerDownMag(icm20948_t *icm) {
    if (writeMagRegister(icm, MAG_CNTL2, MAG_CNTL2_POWER_DOWN) < 0) {
        return -1;
    }
    return 1;
}

int resetMag(icm20948_t *icm) {
    if (writeMagRegister(icm, MAG_CNTL3, MAG_CNTL3_RESET) < 0) {
        return -1;
    }
    return 1;
}

int changeUserBank(icm20948_t *icm, enum UserBank userBank) {
    return changeUserBankForce(icm, userBank, 0);
}

int changeUserBankForce(icm20948_t *icm, enum UserBank userBank, uint8_t force) {
    if (!force && userBank == icm->currentBank) {
        return 2;  // No need to change
    }
    uint8_t userBankRegValue = 0x00;
    switch (userBank) {
        case USER_BANK_0: {
            userBankRegValue = REG_BANK_SEL_USER_BANK_0;
            break;
        }
        case USER_BANK_1: {
            userBankRegValue = REG_BANK_SEL_USER_BANK_1;
            break;
        }
        case USER_BANK_2: {
            userBankRegValue = REG_BANK_SEL_USER_BANK_2;
            break;
        }
        case USER_BANK_3: {
            userBankRegValue = REG_BANK_SEL_USER_BANK_3;
            break;
        }
    }
    if (writeRegister(icm, REG_BANK_SEL, userBankRegValue) < 0) {
        return -1;
    }
    icm->currentBank = userBank;
    return 1;
}

int writeMagRegister(icm20948_t *icm, uint8_t subAddress, uint8_t data) {
    if (changeUserBank(icm, USER_BANK_3) < 0) {
        return -1;
    }
    if (writeRegister(icm, UB3_I2C_SLV0_ADDR, MAG_AK09916_I2C_ADDR) < 0) {
        return -2;
    }
    // set the register to the desired magnetometer sub address
    if (writeRegister(icm, UB3_I2C_SLV0_REG, subAddress) < 0) {
        return -3;
    }
    // store the data for write
    if (writeRegister(icm, UB3_I2C_SLV0_DO, data) < 0) {
        return -4;
    }
    // enable I2C and send 1 byte
    if (writeRegister(icm, UB3_I2C_SLV0_CTRL, UB3_I2C_SLV0_CTRL_EN | (uint8_t)1) < 0) {
        return -5;
    }
    // read the register and confirm
    if (readMagRegisters(icm, subAddress, 1, icm->rxBuf) < 0) {
        return -6;
    }
    if (icm->rxBuf[0] != data) {
        return -7;
    }
    return 1;
}

int readMagRegisters(icm20948_t *icm, uint8_t subAddress, uint8_t count, uint8_t *dest) {
    if (changeUserBank(icm, USER_BANK_3) < 0) {
        return -1;
    }
    if (writeRegister(icm, UB3_I2C_SLV0_ADDR, MAG_AK09916_I2C_ADDR | UB3_I2C_SLV0_ADDR_READ_FLAG) < 0) {
        return -2;
    }
    // set the register to the desired magnetometer sub address
    if (writeRegister(icm, UB3_I2C_SLV0_REG, subAddress) < 0) {
        return -3;
    }
    // enable I2C and request the bytes
    if (writeRegister(icm, UB3_I2C_SLV0_CTRL, UB3_I2C_SLV0_CTRL_EN | count) < 0) {
        return -4;
    }
    delay(1);  // takes some time for these registers to fill
    // read the bytes off the ICM-20948 EXT_SLV_SENS_DATA registers
    if (changeUserBank(icm, USER_BANK_0) < 0) {
        return -5;
    }
    uint8_t status = readRegisters(icm, UB0_EXT_SLV_SENS_DATA_00, count, dest);
    return status;
}

/* writes a byte to ICM20948 register given a register address and data */
int writeRegister(icm20948_t *icm, uint8_t subAddress, uint8_t data) {
    nbe_i2c_start_write(icm->nbe_i2c, icm->address, NULL, NULL);
    nbe_i2c_write_preamble(icm->nbe_i2c, &subAddress, 1);
    nbe_i2c_write_preamble(icm->nbe_i2c, &data, 1);
    nbe_i2c_stop(icm->nbe_i2c);
    nbe_i2c_commit(icm->nbe_i2c);
    while(nbe_i2c_is_busy(icm->nbe_i2c)) {};
    delay(10);
    readRegisters(icm, subAddress, 1, icm->rxBuf);
    if (data != icm->rxBuf[0]) {
        printf("imu panic\n");
        return -1;
    }
    return 1;
}

/* reads registers from ICM20948 given a starting register address, number of bytes, and a pointer to store data */
int readRegisters(icm20948_t *icm, uint8_t subAddress, uint8_t count, uint8_t *dest) {
    readRegistersAsync(icm, subAddress, icm->currentBank, count, dest);
    while (nbe_i2c_is_busy(icm->nbe_i2c)){}
    return 1;
}

int readRegistersAsync(icm20948_t *icm, uint8_t subAddress, enum UserBank userBank, uint8_t count, uint8_t *dest) {
    uint8_t tmpBuf[3];
    nbe_i2c_reset(icm->nbe_i2c);
    nbe_i2c_set_rx_buf(icm->nbe_i2c, dest);
    if (userBank != icm->currentBank) {
        nbe_i2c_start(icm->nbe_i2c);
        tmpBuf[0] = i2c_first_byte_write(icm->address);
        tmpBuf[1] = REG_BANK_SEL;
        tmpBuf[2] = userBank << 4;
        icm->currentBank = userBank;
        nbe_i2c_write_preamble(icm->nbe_i2c, tmpBuf, 3);
    }
    nbe_i2c_start(icm->nbe_i2c);
    tmpBuf[0] = i2c_first_byte_write(icm->address);
    tmpBuf[1] = subAddress;
    nbe_i2c_write_preamble(icm->nbe_i2c, tmpBuf, 2);
    nbe_i2c_start(icm->nbe_i2c);
    tmpBuf[0] = i2c_first_byte_read(icm->address);
    nbe_i2c_write_preamble(icm->nbe_i2c, tmpBuf, 1);
    if (count > 1) {
        nbe_i2c_read_ack(icm->nbe_i2c, count - 1);
    }
    nbe_i2c_read_nak(icm->nbe_i2c, 1);
    nbe_i2c_stop(icm->nbe_i2c);
    nbe_i2c_commit(icm->nbe_i2c);
    return 1;
}

int icm20948_readSensorSync(icm20948_t *icm) {
    changeUserBank(icm, USER_BANK_0);
    readRegisters(icm, UB0_ACCEL_XOUT_H, 20, icm->rxBuf);
    return 1;
}

int icm20948_readSensorAsync(icm20948_t *icm) {
    readRegistersAsync(icm, UB0_ACCEL_XOUT_H, USER_BANK_0, 20, icm->rxBuf);
    return 1;
}

void icm20948_parseAllRaw(icm20948_t *icm, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t i = 0;
    *ax = (icm->rxBuf[i++] << 8);
    *ax += icm->rxBuf[i++];
    *ay = (icm->rxBuf[i++] << 8);
    *ay += icm->rxBuf[i++];
    *az = (icm->rxBuf[i++] << 8);
    *az += icm->rxBuf[i++];

    *gx = (icm->rxBuf[i++] << 8);
    *gx += icm->rxBuf[i++];
    *gy = (icm->rxBuf[i++] << 8);
    *gy += icm->rxBuf[i++];
    *gz = (icm->rxBuf[i++] << 8);
    *gz += icm->rxBuf[i++];
    i += 2; // skip temperature
    *mx = (icm->rxBuf[i++] << 8);
    *mx += icm->rxBuf[i++];
    *my = (icm->rxBuf[i++] << 8);
    *my += icm->rxBuf[i++];
    *mz = (icm->rxBuf[i++] << 8);
    *mz += icm->rxBuf[i++];
}

void icm20948_parseAllMetric(icm20948_t *icm, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
    int16_t rax, ray, raz, rgx, rgy, rgz, rmx, rmy, rmz;
    icm20948_parseAllRaw(icm, &rax, &ray, &raz, &rgx, &rgy, &rgz, &rmx, &rmy, &rmz);
    *ax = rax * icm->accelScale;
    *ay = ray * icm->accelScale;
    *az = raz * icm->accelScale;
    *gx = rgx * icm->gyroScale;
    *gy = rgy * icm->gyroScale;
    *gz = rgz * icm->gyroScale;
    *mx = rmx * 0.15;
    *my = rmy * 0.15;
    *mz = rmz * 0.15;
}

void delay(uint16_t ms) {
    vTaskDelay(ms/portTICK_PERIOD_MS + 1);
}