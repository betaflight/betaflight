#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include "common/axis.h"

#include "drivers/accgyro_common.h"

#ifdef FY90Q
#include "drivers/accgyro_fy90q.h"
#else
#include "drivers/accgyro_adxl345.h"
#include "drivers/accgyro_bma280.h"
#include "drivers/accgyro_l3g4200d.h"
#include "drivers/accgyro_mma845x.h"
#include "drivers/accgyro_mpu3050.h"
#include "drivers/accgyro_mpu6050.h"
#ifdef STM32F3DISCOVERY
#include "drivers/accgyro_lsm303dlhc.h"
#endif
#endif

#include "drivers/barometer_common.h"
#include "drivers/barometer_bmp085.h"
#include "drivers/barometer_ms5611.h"
#include "drivers/compass_hmc5883l.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/system_common.h"

#include "flight_common.h"
#include "runtime_config.h"

#include "sensors_common.h"
#include "sensors_acceleration.h"
#include "sensors_barometer.h"
#include "sensors_gyro.h"
#include "sensors_compass.h"
#include "sensors_sonar.h"

extern uint16_t batteryWarningVoltage;
extern uint8_t batteryCellCount;
extern float magneticDeclination;

extern gyro_t gyro;
extern baro_t baro;
extern acc_t acc;

static void fakeGyroInit(void) {}
static void fakeGyroRead(int16_t *gyroData) {}
static void fakeGyroReadTemp(int16_t *tempData) {}

bool fakeGyroDetect(gyro_t *gyro, uint16_t lpf)
{
    gyro->init = fakeGyroInit;
    gyro->read = fakeGyroRead;
    gyro->temperature = fakeGyroReadTemp;
    return true;
}

static void fakeAccInit(void) {}
static void fakeAccRead(int16_t *gyroData) {}

bool fakeAccDetect(acc_t *acc)
{
    acc->init = fakeAccInit;
    acc->read = fakeAccRead;
    acc->revisionCode = 0;
    return true;
}


#ifdef FY90Q
// FY90Q analog gyro/acc
void sensorsAutodetect(sensorAlignmentConfig_t *sensorAlignmentConfig, uint16_t gyroLpf, uint8_t accHardwareToUse, int16_t magDeclinationFromConfig)
{
    memset(&acc, sizeof(acc), 0);
    memset(&gyro, sizeof(gyro), 0);
    adcSensorInit(&acc, &gyro);
}
#else
void sensorsAutodetect(sensorAlignmentConfig_t *sensorAlignmentConfig, uint16_t gyroLpf, uint8_t accHardwareToUse, int16_t magDeclinationFromConfig)
{
    int16_t deg, min;
    drv_adxl345_config_t acc_params;
    bool haveMpu6k = false;

    memset(&acc, sizeof(acc), 0);
    memset(&gyro, sizeof(gyro), 0);

    // Autodetect gyro hardware. We have MPU3050 or MPU6050.
    if (mpu6050Detect(&acc, &gyro, gyroLpf)) {
        haveMpu6k = true;
        gyroAlign = CW0_DEG; // default NAZE alignment
    } else if (l3g4200dDetect(&gyro, gyroLpf)) {
        gyroAlign = CW0_DEG;
    } else if (mpu3050Detect(&gyro, gyroLpf)) {
        gyroAlign = CW0_DEG;
#ifdef STM32F3DISCOVERY
    } else if (fakeGyroDetect(&gyro, gyroLpf)) {
        gyroAlign = ALIGN_DEFAULT;
#endif
    } else {
        // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
        failureMode(3);
    }

    // Accelerometer. Fuck it. Let user break shit.
retry:
    switch (accHardwareToUse) {
        case ACC_NONE: // disable ACC
            sensorsClear(SENSOR_ACC);
            break;
        case ACC_DEFAULT: // autodetect
        case ACC_ADXL345: // ADXL345
            acc_params.useFifo = false;
            acc_params.dataRate = 800; // unused currently
            if (adxl345Detect(&acc_params, &acc)) {
                accHardware = ACC_ADXL345;
                accAlign = CW270_DEG; // default NAZE alignment
            }
            if (accHardwareToUse == ACC_ADXL345)
                break;
            ; // fallthrough
        case ACC_MPU6050: // MPU6050
            if (haveMpu6k) {
                mpu6050Detect(&acc, &gyro, gyroLpf); // yes, i'm rerunning it again.  re-fill acc struct
                accHardware = ACC_MPU6050;
                accAlign = CW0_DEG; // default NAZE alignment
                if (accHardwareToUse == ACC_MPU6050)
                    break;
            }
            ; // fallthrough
#if !defined(OLIMEXINO) && !defined(STM32F3DISCOVERY)
        case ACC_MMA8452: // MMA8452
            if (mma8452Detect(&acc)) {
                accHardware = ACC_MMA8452;
                accAlign = CW90_DEG; // default NAZE alignment
                if (accHardwareToUse == ACC_MMA8452)
                    break;
            }
            ; // fallthrough
        case ACC_BMA280: // BMA280
            if (bma280Detect(&acc)) {
                accHardware = ACC_BMA280;
                accAlign = CW0_DEG; //
                if (accHardwareToUse == ACC_BMA280)
                    break;
            }
#endif
#ifdef STM32F3DISCOVERY
        case ACC_LSM303DLHC:
            if (lsm303dlhcAccDetect(&acc)) {
                accHardware = ACC_LSM303DLHC;
                accAlign = ALIGN_DEFAULT; //
                if (accHardwareToUse == ACC_LSM303DLHC)
                    break;
            }
//        default:
//            if (fakeAccDetect(&acc)) {
//                accHardware = ACC_FAKE;
//                accAlign = CW0_DEG; //
//                if (accHardwareToUse == ACC_FAKE)
//                    break;
//            }
#endif
    }

    // Found anything? Check if user fucked up or ACC is really missing.
    if (accHardware == ACC_DEFAULT) {
        if (accHardwareToUse > ACC_DEFAULT) {
            // Nothing was found and we have a forced sensor type. Stupid user probably chose a sensor that isn't present.
            accHardwareToUse = ACC_DEFAULT;
            goto retry;
        } else {
            // We're really screwed
            sensorsClear(SENSOR_ACC);
        }
    }

#ifdef BARO
    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function
    if (!ms5611Detect(&baro)) {
        // ms5611 disables BMP085, and tries to initialize + check PROM crc. if this works, we have a baro
        if (!bmp085Detect(&baro)) {
            // if both failed, we don't have anything
            sensorsClear(SENSOR_BARO);
        }
    }
#endif


    if (sensorAlignmentConfig->gyro_align != ALIGN_DEFAULT) {
        gyroAlign = sensorAlignmentConfig->gyro_align;
    }
    if (sensorAlignmentConfig->acc_align != ALIGN_DEFAULT) {
        accAlign = sensorAlignmentConfig->acc_align;
    }
    if (sensorAlignmentConfig->mag_align != ALIGN_DEFAULT) {
        magAlign = sensorAlignmentConfig->mag_align;
    }


    // Now time to init things, acc first
    if (sensors(SENSOR_ACC))
        acc.init();
    // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.
    gyro.init();

#ifdef MAG
    if (hmc5883lDetect()) {
        magAlign = CW180_DEG; // default NAZE alignment
    } else {
        sensorsClear(SENSOR_MAG);
    }
#endif

    // FIXME extract to a method to reduce dependencies, maybe move to sensors_compass.c
    if (sensors(SENSOR_MAG)) {
        // calculate magnetic declination
        deg = magDeclinationFromConfig / 100;
        min = magDeclinationFromConfig % 100;

        magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
    } else {
        magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
    }
}
#endif

