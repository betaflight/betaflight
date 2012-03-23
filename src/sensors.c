#include "board.h"
#include "mw.h"

uint8_t calibratedACC = 0;
uint16_t calibratingA = 0;       // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingG = 0;
uint8_t calibratingM = 0;
uint16_t acc_1G = 256;         // this is the 1G measured acceleration
int16_t heading, magHold;

extern uint16_t InflightcalibratingA;
extern int16_t AccInflightCalibrationArmed;
extern uint16_t AccInflightCalibrationMeasurementDone;
extern uint16_t AccInflightCalibrationSavetoEEProm;
extern uint16_t AccInflightCalibrationActive;
extern uint16_t batteryWarningVoltage;
extern uint8_t batteryCellCount;

sensor_t acc;                   // acc access functions
sensor_t gyro;                  // gyro access functions

void sensorsAutodetect(void)
{
    // Detect what's available
    if (!adxl345Detect(&acc))
        sensorsClear(SENSOR_ACC);
    if (!bmp085Init())
        sensorsClear(SENSOR_BARO);
    if (!hmc5883lDetect())
        sensorsClear(SENSOR_MAG);

    // Init sensors
    if (sensors(SENSOR_ACC))
        acc.init();
    if (sensors(SENSOR_BARO))
        bmp085Init();

    // special case for supported gyros - MPU3050 and MPU6050
    if (mpu6050Detect(&acc, &gyro)) { // first, try MPU6050, and re-enable acc (if ADXL345 is missing) since this chip has it built in
        sensorsSet(SENSOR_ACC);
        acc.init();
    } else if (!mpu3050Detect(&gyro)) {
        // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
        failureMode(3);
    }
    // this is safe because either mpu6050 or mpu3050 sets it, and in case of fail, none do.
    gyro.init();
}

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 4095 = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return (((src) * 3.3f) / 4095) * cfg.vbatscale;
}

void batteryInit(void)
{
    uint8_t i;
    uint32_t voltage = 0;

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
    uint8_t axis;

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
            cfg.accTrim[ROLL] = 0;
            cfg.accTrim[PITCH] = 0;
            writeParams();      // write accZero in EEPROM
        }
        calibratingA--;
    }
    
    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        static int32_t b[3];
        static int16_t accZero_saved[3] = { 0, 0, 0 };
        static int16_t accTrim_saved[2] = { 0, 0 };
        //Saving old zeropoints before measurement
        if (InflightcalibratingA == 50) {
            accZero_saved[ROLL] = cfg.accZero[ROLL];
            accZero_saved[PITCH] = cfg.accZero[PITCH];
            accZero_saved[YAW] = cfg.accZero[YAW];
            accTrim_saved[ROLL] = cfg.accTrim[ROLL];
            accTrim_saved[PITCH] = cfg.accTrim[PITCH];
        }
        if (InflightcalibratingA > 0) {
            uint8_t axis;
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
            //all values are measured
            if (InflightcalibratingA == 1) {
                AccInflightCalibrationActive = 0;
                AccInflightCalibrationMeasurementDone = 1;
                blinkLED(10, 10, 2);        //buzzer for indicatiing the start inflight
                // recover saved values to maintain current flight behavior until new values are transferred
                cfg.accZero[ROLL] = accZero_saved[ROLL];
                cfg.accZero[PITCH] = accZero_saved[PITCH];
                cfg.accZero[YAW] = accZero_saved[YAW];
                cfg.accTrim[ROLL] = accTrim_saved[ROLL];
                cfg.accTrim[PITCH] = accTrim_saved[PITCH];
            }
            InflightcalibratingA--;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (AccInflightCalibrationSavetoEEProm == 1) {      //the copter is landed, disarmed and the combo has been done again
            AccInflightCalibrationSavetoEEProm = 0;
            cfg.accZero[ROLL] = b[ROLL] / 50;
            cfg.accZero[PITCH] = b[PITCH] / 50;
            cfg.accZero[YAW] = b[YAW] / 50 - acc_1G;    // for nunchuk 200=1G
            cfg.accTrim[ROLL] = 0;
            cfg.accTrim[PITCH] = 0;
            writeParams();          // write accZero in EEPROM
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

static uint32_t baroDeadline = 0;
static uint8_t baroState = 0;
static uint16_t baroUT = 0;
static uint32_t baroUP = 0;
static int16_t baroTemp = 0;

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
            baroDeadline += 14000;
            break;
        case 3:
            baroUP = bmp085_get_up();
            baroTemp = bmp085_get_temperature(baroUT);
            pressure = bmp085_get_pressure(baroUP);

            BaroAlt = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f; // centimeter
            baroState = 0;
            baroDeadline += 5000;
            break;
    }
}

static void GYRO_Common(void)
{
    static int16_t previousGyroADC[3] = { 0, 0, 0 };
    static int32_t g[3];
    uint8_t axis;
    
#if defined MMGYRO       
    // Moving Average Gyros by Magnetron1
    //---------------------------------------------------
    static int16_t mediaMobileGyroADC[3][MMGYROVECTORLENGTH];
    static int32_t mediaMobileGyroADCSum[3];
    static uint8_t mediaMobileGyroIDX;
    //---------------------------------------------------
#endif

    if (calibratingG > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset g[axis] at start of calibration
            if (calibratingG == 400)
                g[axis] = 0;
            // Sum up 400 readings
            g[axis] += gyroADC[axis];
            // Clear global variables for next reading
            gyroADC[axis] = 0;
            gyroZero[axis] = 0;
            if (calibratingG == 1) {
                gyroZero[axis] = g[axis] / 400;
                blinkLED(10, 15, 1);
            }
        }
        calibratingG--;
    }

#ifdef MMGYRO       
    mediaMobileGyroIDX = ++mediaMobileGyroIDX % MMGYROVECTORLENGTH;
    for (axis = 0; axis < 3; axis++) {
        gyroADC[axis]  -= gyroZero[axis];
        mediaMobileGyroADCSum[axis] -= mediaMobileGyroADC[axis][mediaMobileGyroIDX];
        //anti gyro glitch, limit the variation between two consecutive readings
        mediaMobileGyroADC[axis][mediaMobileGyroIDX] = constrain(gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
        mediaMobileGyroADCSum[axis] += mediaMobileGyroADC[axis][mediaMobileGyroIDX];
        gyroADC[axis] = mediaMobileGyroADCSum[axis] / MMGYROVECTORLENGTH;
        previousGyroADC[axis] = gyroADC[axis];
    }
#else
    for (axis = 0; axis < 3; axis++) {
        gyroADC[axis] -= gyroZero[axis];
        //anti gyro glitch, limit the variation between two consecutive readings
        gyroADC[axis] = constrain(gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
        previousGyroADC[axis] = gyroADC[axis];
    }
#endif
}

void Gyro_getADC(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    gyro.read(gyroADC);
    gyro.align(gyroADC);

    GYRO_Common();
}

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
    // initial calibration
    hmc5883lInit();
    delay(100);
    Mag_getRawADC();
    delay(10);

    magCal[ROLL] = 1160.0f / abs(magADC[ROLL]);
    magCal[PITCH] = 1160.0f / abs(magADC[PITCH]);
    magCal[YAW] = 1080.0f / abs(magADC[YAW]);

    hmc5883lFinishCal();
    magInit = 1;
}

void Mag_getADC(void)
{
    static uint32_t t, tCal = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint8_t axis;
    
    if (currentTime < t)
        return;                 //each read is spaced by 100ms
    t = currentTime + 100000;

    // Read mag sensor
    Mag_getRawADC();

    magADC[ROLL]  = magADC[ROLL]  * magCal[ROLL];
    magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
    magADC[YAW]   = magADC[YAW]   * magCal[YAW];

    if (calibratingM == 1) {
        tCal = t;
        for (axis = 0; axis < 3; axis++) {
            cfg.magZero[axis] = 0;
            magZeroTempMin[axis] = magADC[axis];
            magZeroTempMax[axis] = magADC[axis];
        }
        calibratingM = 0;
    }
    magADC[ROLL] = magADC[ROLL] * magCal[ROLL];
    magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
    magADC[YAW] = magADC[YAW] * magCal[YAW];
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
                cfg.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2;
            writeParams();
        }
    }
}
