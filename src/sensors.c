#include "board.h"
#include "mw.h"

uint16_t calibratingA = 0;       // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingG = 0;
uint16_t acc_1G = 256;         // this is the 1G measured acceleration.
int16_t heading, magHold;

extern uint16_t InflightcalibratingA;
extern int16_t AccInflightCalibrationArmed;
extern uint16_t AccInflightCalibrationMeasurementDone;
extern uint16_t AccInflightCalibrationSavetoEEProm;
extern uint16_t AccInflightCalibrationActive;
extern uint16_t batteryWarningVoltage;
extern uint8_t batteryCellCount;
extern float magneticDeclination;

sensor_t acc;                       // acc access functions
sensor_t gyro;                      // gyro access functions
uint8_t accHardware = ACC_DEFAULT;  // which accel chip is used/detected

#ifdef FY90Q
// FY90Q analog gyro/acc
void sensorsAutodetect(void)
{
    adcSensorInit(&acc, &gyro);
}
#else
// AfroFlight32 i2c sensors
void sensorsAutodetect(void)
{
    int16_t deg, min;
    drv_adxl345_config_t acc_params;
    bool haveMpu6k = false;
    bool havel3g4200d = false;

    // Autodetect gyro hardware. We have MPU3050 or MPU6050.
    if (mpu6050Detect(&acc, &gyro, cfg.mpu6050_scale)) {
        // this filled up  acc.* struct with init values
        haveMpu6k = true;
    } else if (l3g4200dDetect(&gyro)) {
        havel3g4200d = true;
    } else if (!mpu3050Detect(&gyro)) {
        // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
        failureMode(3);
    }

    // Accelerometer. Fuck it. Let user break shit.
retry:
    switch (cfg.acc_hardware) {
        case 0: // autodetect
        case 1: // ADXL345
            acc_params.useFifo = false;
            acc_params.dataRate = 800; // unused currently
            if (adxl345Detect(&acc_params, &acc))
                accHardware = ACC_ADXL345;
            if (cfg.acc_hardware == ACC_ADXL345)
                break;
            ; // fallthrough
       case 2: // MPU6050
            if (haveMpu6k) {
                mpu6050Detect(&acc, &gyro, cfg.mpu6050_scale); // yes, i'm rerunning it again.  re-fill acc struct
                accHardware = ACC_MPU6050;
                if (cfg.acc_hardware == ACC_MPU6050)
                    break;
            }
            ; // fallthrough
        case 3: // MMA8452
            if (mma8452Detect(&acc)) {
                accHardware = ACC_MMA8452;
                if (cfg.acc_hardware == ACC_MMA8452)
                    break;
            }
    }

    // Found anything? Check if user fucked up or ACC is really missing.
    if (accHardware == ACC_DEFAULT) {
        if (cfg.acc_hardware > ACC_DEFAULT) {
            // Nothing was found and we have a forced sensor type. Stupid user probably chose a sensor that isn't present.
            cfg.acc_hardware = ACC_DEFAULT;
            goto retry;
        } else {
            // We're really screwed
            sensorsClear(SENSOR_ACC);
        }
    }

    // Detect what else is available
    if (!bmp085Init())
        sensorsClear(SENSOR_BARO);
    if (!hmc5883lDetect())
        sensorsClear(SENSOR_MAG);

    // Now time to init them, acc first
    if (sensors(SENSOR_ACC))
        acc.init();
    if (sensors(SENSOR_BARO))
        bmp085Init();
    // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.
    gyro.init();

    // todo: this is driver specific :(
    if (havel3g4200d) {
        l3g4200dConfig(cfg.gyro_lpf);
    } else {
        if (!haveMpu6k)
            mpu3050Config(cfg.gyro_lpf);
    }

    // calculate magnetic declination
    deg = cfg.mag_declination / 100;
    min = cfg.mag_declination % 100;
    magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
}
#endif

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 4095 = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return (((src) * 3.3f) / 4095) * cfg.vbatscale;
}

void batteryInit(void)
{
    uint32_t i;
    uint32_t voltage = 0;

    // average up some voltage readings
    for (i = 0; i < 32; i++) {
        voltage += adcGetBattery();
        delay(10);
    }

    voltage = batteryAdcToVoltage((uint16_t)(voltage / 32));

    // autodetect cell count, going from 2S..6S
    for (i = 2; i < 6; i++) {
        if (voltage < i * cfg.vbatmaxcellvoltage)
            break;
    }
    batteryCellCount = i;
    batteryWarningVoltage = i * cfg.vbatmincellvoltage; // 3.3V per cell minimum, configurable in CLI
}

static void ACC_Common(void)
{
    static int32_t a[3];
    int axis;

    if (calibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (calibratingA == 400)
                a[axis] = 0;
            // Sum up 400 readings
            a[axis] += accADC[axis];
            // Clear global variables for next reading
            accADC[axis] = 0;
            cfg.accZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (calibratingA == 1) {
            cfg.accZero[ROLL] = a[ROLL] / 400;
            cfg.accZero[PITCH] = a[PITCH] / 400;
            cfg.accZero[YAW] = a[YAW] / 400 - acc_1G;       // for nunchuk 200=1G
            cfg.angleTrim[ROLL] = 0;
            cfg.angleTrim[PITCH] = 0;
            writeParams(1);      // write accZero in EEPROM
        }
        calibratingA--;
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        static int32_t b[3];
        static int16_t accZero_saved[3] = { 0, 0, 0 };
        static int16_t angleTrim_saved[2] = { 0, 0 };
        // Saving old zeropoints before measurement
        if (InflightcalibratingA == 50) {
            accZero_saved[ROLL] = cfg.accZero[ROLL];
            accZero_saved[PITCH] = cfg.accZero[PITCH];
            accZero_saved[YAW] = cfg.accZero[YAW];
            angleTrim_saved[ROLL] = cfg.angleTrim[ROLL];
            angleTrim_saved[PITCH] = cfg.angleTrim[PITCH];
        }
        if (InflightcalibratingA > 0) {
            for (axis = 0; axis < 3; axis++) {
                // Reset a[axis] at start of calibration
                if (InflightcalibratingA == 50)
                    b[axis] = 0;
                // Sum up 50 readings
                b[axis] += accADC[axis];
                // Clear global variables for next reading
                accADC[axis] = 0;
                cfg.accZero[axis] = 0;
            }
            // all values are measured
            if (InflightcalibratingA == 1) {
                AccInflightCalibrationActive = 0;
                AccInflightCalibrationMeasurementDone = 1;
                toggleBeep = 2;      // buzzer for indicatiing the end of calibration
                // recover saved values to maintain current flight behavior until new values are transferred
                cfg.accZero[ROLL] = accZero_saved[ROLL];
                cfg.accZero[PITCH] = accZero_saved[PITCH];
                cfg.accZero[YAW] = accZero_saved[YAW];
                cfg.angleTrim[ROLL] = angleTrim_saved[ROLL];
                cfg.angleTrim[PITCH] = angleTrim_saved[PITCH];
            }
            InflightcalibratingA--;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (AccInflightCalibrationSavetoEEProm == 1) {      // the copter is landed, disarmed and the combo has been done again
            AccInflightCalibrationSavetoEEProm = 0;
            cfg.accZero[ROLL] = b[ROLL] / 50;
            cfg.accZero[PITCH] = b[PITCH] / 50;
            cfg.accZero[YAW] = b[YAW] / 50 - acc_1G;    // for nunchuk 200=1G
            cfg.angleTrim[ROLL] = 0;
            cfg.angleTrim[PITCH] = 0;
            writeParams(1);          // write accZero in EEPROM
        }
    }

    accADC[ROLL] -= cfg.accZero[ROLL];
    accADC[PITCH] -= cfg.accZero[PITCH];
    accADC[YAW] -= cfg.accZero[YAW];
}

void ACC_getADC(void)
{
    acc.read(accADC);
    acc.align(accADC);

    ACC_Common();
}

#ifdef BARO
static uint32_t baroDeadline = 0;
static uint8_t baroState = 0;
static uint16_t baroUT = 0;
static uint32_t baroUP = 0;

void Baro_update(void)
{
    int32_t pressure;

    if (currentTime < baroDeadline)
        return;

    baroDeadline = currentTime;

    switch (baroState) {
        case 0:
            bmp085_start_ut();
            baroState++;
            baroDeadline += 4600;
            break;
        case 1:
            baroUT = bmp085_get_ut();
            baroState++;
            break;
        case 2:
            bmp085_start_up();
            baroState++;
            baroDeadline += 26000;
            break;
        case 3:
            baroUP = bmp085_get_up();
            bmp085_get_temperature(baroUT);
            pressure = bmp085_get_pressure(baroUP);
            BaroAlt = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f; // centimeter
            baroState = 0;
            baroDeadline += 5000;
            break;
    }
}
#endif /* BARO */

static void GYRO_Common(void)
{
    static int16_t previousGyroADC[3] = { 0, 0, 0 };
    static int32_t g[3];
    int axis;

    if (calibratingG > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset g[axis] at start of calibration
            if (calibratingG == 1000)
                g[axis] = 0;
            // Sum up 1000 readings
            g[axis] += gyroADC[axis];
            // g[axis] += (1000 - calibratingG) >> 1;
            // Clear global variables for next reading
            gyroADC[axis] = 0;
            gyroZero[axis] = 0;
            if (calibratingG == 1) {
                gyroZero[axis] = g[axis] / 1000;
                blinkLED(10, 15, 1);
            }
        }
        calibratingG--;
    }
    for (axis = 0; axis < 3; axis++) {
        gyroADC[axis] -= gyroZero[axis];
        //anti gyro glitch, limit the variation between two consecutive readings
        gyroADC[axis] = constrain(gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
        previousGyroADC[axis] = gyroADC[axis];
    }
}

void Gyro_getADC(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    gyro.read(gyroADC);
    gyro.align(gyroADC);

    GYRO_Common();
}

#ifdef MAG
static float magCal[3] = { 1.0, 1.0, 1.0 };     // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

static void Mag_getRawADC(void)
{
    static int16_t rawADC[3];
    hmc5883lRead(rawADC);

    // no way? is THIS finally the proper orientation?? (by GrootWitBaas)
    magADC[ROLL] = rawADC[2]; // X
    magADC[PITCH] = -rawADC[0]; // Y
    magADC[YAW] = -rawADC[1]; // Z
}

void Mag_init(void)
{
    uint8_t calibration_gain = 0x60; // HMC5883
#if 1
    uint32_t numAttempts = 0, good_count = 0;
    bool success = false;
    uint16_t expected_x = 766;       // default values for HMC5883
    uint16_t expected_yz = 713;
    float gain_multiple = 660.0f / 1090.0f; // adjustment for runtime vs calibration gain
    float cal[3];

    // initial calibration
    hmc5883lInit();

    magCal[0] = 0;
    magCal[1] = 0;
    magCal[2] = 0;

    while (success == false && numAttempts < 20 && good_count < 5) {
        // record number of attempts at initialisation
        numAttempts++;
        // enter calibration mode
        hmc5883lCal(calibration_gain);
        delay(100);
        Mag_getRawADC();
        delay(10);

        cal[0] = fabsf(expected_x / (float)magADC[ROLL]);
        cal[1] = fabsf(expected_yz / (float)magADC[PITCH]);
        cal[2] = fabsf(expected_yz / (float)magADC[ROLL]);

        if (cal[0] > 0.7f && cal[0] < 1.3f && cal[1] > 0.7f && cal[1] < 1.3f && cal[2] > 0.7f && cal[2] < 1.3f) {
            good_count++;
            magCal[0] += cal[0];
            magCal[1] += cal[1];
            magCal[2] += cal[2];
        }
    }

    if (good_count >= 5) {
        magCal[0] = magCal[0] * gain_multiple / (float)good_count;
        magCal[1] = magCal[1] * gain_multiple / (float)good_count;
        magCal[2] = magCal[2] * gain_multiple / (float)good_count;
        success = true;
    } else {
        /* best guess */
        magCal[0] = 1.0f;
        magCal[1] = 1.0f;
        magCal[2] = 1.0f;
    }

    hmc5883lFinishCal();
#else
    // initial calibration
    hmc5883lInit();

    magCal[0] = 0;
    magCal[1] = 0;
    magCal[2] = 0;

    // enter calibration mode
    hmc5883lCal(calibration_gain);
    delay(100);
    Mag_getRawADC();
    delay(10);

    magCal[ROLL] = 1160.0f / abs(magADC[ROLL]);
    magCal[PITCH] = 1160.0f / abs(magADC[PITCH]);
    magCal[YAW] = 1080.0f / abs(magADC[YAW]);

    hmc5883lFinishCal();
#endif

    magInit = 1;
}

void Mag_getADC(void)
{
    static uint32_t t, tCal = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint32_t axis;
    
    if (currentTime < t)
        return;                 //each read is spaced by 100ms
    t = currentTime + 100000;

    // Read mag sensor
    Mag_getRawADC();

    magADC[ROLL]  = magADC[ROLL]  * magCal[ROLL];
    magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
    magADC[YAW]   = magADC[YAW]   * magCal[YAW];

    if (f.CALIBRATE_MAG) {
        tCal = t;
        for (axis = 0; axis < 3; axis++) {
            cfg.magZero[axis] = 0;
            magZeroTempMin[axis] = magADC[axis];
            magZeroTempMax[axis] = magADC[axis];
        }
        f.CALIBRATE_MAG = 0;
    }

    if (magInit) {              // we apply offset only once mag calibration is done
        magADC[ROLL] -= cfg.magZero[ROLL];
        magADC[PITCH] -= cfg.magZero[PITCH];
        magADC[YAW] -= cfg.magZero[YAW];
    }

    if (tCal != 0) {
        if ((t - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            LED0_TOGGLE;
            for (axis = 0; axis < 3; axis++) {
                if (magADC[axis] < magZeroTempMin[axis])
                    magZeroTempMin[axis] = magADC[axis];
                if (magADC[axis] > magZeroTempMax[axis])
                    magZeroTempMax[axis] = magADC[axis];
            }
        } else {
            tCal = 0;
            for (axis = 0; axis < 3; axis++)
                cfg.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2; // Calculate offsets
            writeParams(1);
        }
    }
}
#endif

#ifdef SONAR

void Sonar_init(void) 
{
    hcsr04_init(sonar_rc78);
    sensorsSet(SENSOR_SONAR);
    sonarAlt = 0;
}

void Sonar_update(void) 
{
    hcsr04_get_distance(&sonarAlt);
}

#endif
