#include "board.h"
#include "mw.h"

uint8_t calibratedACC = 0;
uint16_t calibratingA = 0;       // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingG = 0;
uint8_t calibratingM = 0;
uint16_t acc_1G = 256;         // this is the 1G measured acceleration
int16_t accTrim[2] = { 0, 0 };
int16_t heading, magHold;

void sensorsAutodetect(void)
{
    if (!adxl345Detect())
        sensorsClear(SENSOR_ACC);
    if (!bmp085Init())
        sensorsClear(SENSOR_BARO);
    if (!hmc5883lDetect())
        sensorsClear(SENSOR_MAG);
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
            accZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (calibratingA == 1) {
            accZero[ROLL] = a[ROLL] / 400;
            accZero[PITCH] = a[PITCH] / 400;
            accZero[YAW] = a[YAW] / 400 - acc_1G;       // for nunchuk 200=1G
            accTrim[ROLL] = 0;
            accTrim[PITCH] = 0;
            writeParams();      // write accZero in EEPROM
        }
        calibratingA--;
    }
#if defined(InflightAccCalibration)
    static int32_t b[3];
    static int16_t accZero_saved[3] = { 0, 0, 0 };
    static int16_t accTrim_saved[2] = { 0, 0 };
    //Saving old zeropoints before measurement
    if (InflightcalibratingA == 50) {
        accZero_saved[ROLL] = accZero[ROLL];
        accZero_saved[PITCH] = accZero[PITCH];
        accZero_saved[YAW] = accZero[YAW];
        accTrim_saved[ROLL] = accTrim[ROLL];
        accTrim_saved[PITCH] = accTrim[PITCH];
    }
    if (InflightcalibratingA > 0) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (InflightcalibratingA == 50)
                b[axis] = 0;
            // Sum up 50 readings
            b[axis] += accADC[axis];
            // Clear global variables for next reading
            accADC[axis] = 0;
            accZero[axis] = 0;
        }
        //all values are measured
        if (InflightcalibratingA == 1) {
            AccInflightCalibrationActive = 0;
            AccInflightCalibrationMeasurementDone = 1;
            blinkLED(10, 10, 2);        //buzzer for indicatiing the start inflight
            // recover saved values to maintain current flight behavior until new values are transferred
            accZero[ROLL] = accZero_saved[ROLL];
            accZero[PITCH] = accZero_saved[PITCH];
            accZero[YAW] = accZero_saved[YAW];
            accTrim[ROLL] = accTrim_saved[ROLL];
            accTrim[PITCH] = accTrim_saved[PITCH];
        }
        InflightcalibratingA--;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (AccInflightCalibrationSavetoEEProm == 1) {      //the copter is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = 0;
        accZero[ROLL] = b[ROLL] / 50;
        accZero[PITCH] = b[PITCH] / 50;
        accZero[YAW] = b[YAW] / 50 - acc_1G;    // for nunchuk 200=1G
        accTrim[ROLL] = 0;
        accTrim[PITCH] = 0;
        writeParams();          // write accZero in EEPROM
    }
#endif
    accADC[ROLL] -= accZero[ROLL];
    accADC[PITCH] -= accZero[PITCH];
    accADC[YAW] -= accZero[YAW];
}


void ACC_getADC(void)
{
    int16_t rawADC[3];
    
    adxl345Read(rawADC);

    ACC_ORIENTATION(-(rawADC[1]), (rawADC[0]), (rawADC[2]));
    ACC_Common();
}

static uint32_t baroDeadline = 0;
static uint8_t baroState = 0;
static uint16_t baroUT = 0;
static uint32_t baroUP = 0;
static int16_t baroTemp = 0;

void Baro_update(void)
{
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

    for (axis = 0; axis < 3; axis++) {
        gyroADC[axis] -= gyroZero[axis];
        //anti gyro glitch, limit the variation between two consecutive readings
        gyroADC[axis] = constrain(gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
        previousGyroADC[axis] = gyroADC[axis];
    }
}

void Gyro_getADC(void)
{
    int16_t rawADC[3];
    
    mpu3050Read(rawADC);
    
    // range: +/- 8192; +/- 2000 deg/sec
    GYRO_ORIENTATION(+((rawADC[1]) / 4), -((rawADC[0]) / 4), -((rawADC[2]) / 4));
    gyroADC[ROLL] = rawADC[0] / 4;
    gyroADC[PITCH] = rawADC[1] / 4;
    gyroADC[YAW] = -rawADC[2] / 4;

    GYRO_Common();
}

static float magCal[3] = { 1.0, 1.0, 1.0 };     // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

static void Mag_getRawADC(void)
{
    static int16_t rawADC[3];
    hmc5883lRead(rawADC);
    
    // Hearty FUCK-YOU goes to all teh breakout sensor faggots who make a new orientation for each shitty board they make
    // sensor order: X Z Y
    magADC[ROLL] = rawADC[0]; // X or negative? who knows mag stuff in multiwii is broken hardcore
    magADC[PITCH] = rawADC[2]; // Y
    magADC[YAW] = rawADC[1]; // Z
}

void Mag_init(void)
{
    // initial calibration
    hmc5883lInit();
    delay(100);
    Mag_getRawADC();
    delay(10);

    magCal[ROLL] = 1000.0 / abs(magADC[ROLL]);
    magCal[PITCH] = 1000.0 / abs(magADC[PITCH]);
    magCal[YAW] = 1000.0 / abs(magADC[YAW]);
    
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

    if (calibratingM == 1) {
        tCal = t;
        for (axis = 0; axis < 3; axis++) {
            magZero[axis] = 0;
            magZeroTempMin[axis] = 0;
            magZeroTempMax[axis] = 0;
        }
        calibratingM = 0;
    }
    magADC[ROLL] = magADC[ROLL] * magCal[ROLL];
    magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
    magADC[YAW] = magADC[YAW] * magCal[YAW];
    if (magInit) {              // we apply offset only once mag calibration is done
        magADC[ROLL] -= magZero[ROLL];
        magADC[PITCH] -= magZero[PITCH];
        magADC[YAW] -= magZero[YAW];
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
                magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2;
            writeParams();
        }
    }
}
