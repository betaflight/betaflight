#include "board.h"
#include "mw.h"

#include "sensors_acceleration.h"
#include "sensors_barometer.h"
#include "sensors_gyro.h"
#include "sensors_compass.h"

#include "sensors_common.h"

int16_t heading, magHold;

extern uint16_t batteryWarningVoltage;
extern uint8_t batteryCellCount;
extern float magneticDeclination;

extern gyro_t gyro;
extern baro_t baro;
extern acc_t acc;

#ifdef FY90Q
// FY90Q analog gyro/acc
void sensorsAutodetect(void)
{
    memset(&acc, sizeof(acc), 0);
    memset(&gyro, sizeof(gyro), 0);
    adcSensorInit(&acc, &gyro);
}
#else
// AfroFlight32 i2c sensors
void sensorsAutodetect(void)
{
    int16_t deg, min;
    drv_adxl345_config_t acc_params;
    bool haveMpu6k = false;

    memset(&acc, sizeof(acc), 0);
    memset(&gyro, sizeof(gyro), 0);

    // Autodetect gyro hardware. We have MPU3050 or MPU6050.
    if (mpu6050Detect(&acc, &gyro, mcfg.gyro_lpf)) {
        haveMpu6k = true;
        gyroAlign = CW0_DEG; // default NAZE alignment
    } else if (l3g4200dDetect(&gyro, mcfg.gyro_lpf)) {
        gyroAlign = CW0_DEG;
    } else if (mpu3050Detect(&gyro, mcfg.gyro_lpf)) {
        gyroAlign = CW0_DEG;
    } else {
        // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
        failureMode(3);
    }

    // Accelerometer. Fuck it. Let user break shit.
retry:
    switch (mcfg.acc_hardware) {
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
            if (mcfg.acc_hardware == ACC_ADXL345)
                break;
            ; // fallthrough
        case ACC_MPU6050: // MPU6050
            if (haveMpu6k) {
                mpu6050Detect(&acc, &gyro, mcfg.gyro_lpf); // yes, i'm rerunning it again.  re-fill acc struct
                accHardware = ACC_MPU6050;
                accAlign = CW0_DEG; // default NAZE alignment
                if (mcfg.acc_hardware == ACC_MPU6050)
                    break;
            }
            ; // fallthrough
#ifndef OLIMEXINO
        case ACC_MMA8452: // MMA8452
            if (mma8452Detect(&acc)) {
                accHardware = ACC_MMA8452;
                accAlign = CW90_DEG; // default NAZE alignment
                if (mcfg.acc_hardware == ACC_MMA8452)
                    break;
            }
            ; // fallthrough
        case ACC_BMA280: // BMA280
            if (bma280Detect(&acc)) {
                accHardware = ACC_BMA280;
                accAlign = CW0_DEG; //
                if (mcfg.acc_hardware == ACC_BMA280)
                    break;
            }
#endif
    }

    // Found anything? Check if user fucked up or ACC is really missing.
    if (accHardware == ACC_DEFAULT) {
        if (mcfg.acc_hardware > ACC_DEFAULT) {
            // Nothing was found and we have a forced sensor type. Stupid user probably chose a sensor that isn't present.
            mcfg.acc_hardware = ACC_DEFAULT;
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

    if (mcfg.gyro_align != ALIGN_DEFAULT) {
        gyroAlign = mcfg.gyro_align;
    }
    if (mcfg.acc_align != ALIGN_DEFAULT) {
        accAlign = mcfg.acc_align;
    }
    if (mcfg.mag_align != ALIGN_DEFAULT) {
        magAlign = mcfg.mag_align;
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

    if (sensors(SENSOR_MAG)) {
    // calculate magnetic declination
        deg = cfg.mag_declination / 100;
        min = cfg.mag_declination % 100;

        magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
    } else {
        magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
    }
}
#endif

