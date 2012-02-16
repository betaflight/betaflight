/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
December  2011     V1.dev
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include "config.h"
#include "def.h"
#include <avr/pgmspace.h>
#define   VERSION  19

/*********** RC alias *****************/
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define AUX3       6
#define AUX4       7

#define PIDALT     3
#define PIDVEL     4
#define PIDGPS     5
#define PIDLEVEL   6
#define PIDMAG     7

#define BOXACC      0
#define BOXBARO     1
#define BOXMAG      2
#define BOXCAMSTAB  3
#define BOXCAMTRIG  4
#define BOXARM      5
#define BOXGPSHOME  6
#define BOXGPSHOLD  7
#define BOXPASSTHRU 8
#define BOXHEADFREE 9
#define BOXBEEPERON  10

#define CHECKBOXITEMS 11
#define PIDITEMS 8

static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;  // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;       // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint8_t calibratingM = 0;
static uint16_t calibratingG;
static uint8_t armed = 0;
static uint16_t acc_1G;         // this is the 1G measured acceleration
static int16_t acc_25deg;
static uint8_t nunchuk = 0;
static uint8_t accMode = 0;     // if level mode is a activated
static uint8_t magMode = 0;     // if compass heading hold is a activated
static uint8_t baroMode = 0;    // if altitude hold is activated
static uint8_t GPSModeHome = 0; // if GPS RTH is activated
static uint8_t GPSModeHold = 0; // if GPS PH is activated
static uint8_t headFreeMode = 0;        // if head free mode is a activated
static uint8_t passThruMode = 0;        // if passthrough mode is activated
static int16_t headFreeModeHold;
static int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
static int16_t accTrim[2] = { 0, 0 };
static int16_t heading, magHold;
static uint8_t calibratedACC = 0;
static uint8_t vbat;            // battery voltage in 0.1V steps
static uint8_t okToArm = 0;
static uint8_t rcOptions1, rcOptions2;
static int32_t pressure;
static int16_t BaroAlt;
static int16_t EstAlt;          // in cm
static int16_t zVelocity;
static uint8_t buzzerState = 0;
static int16_t debug1, debug2, debug3, debug4;

//for log
static uint16_t cycleTimeMax = 0;       // highest ever cycle timen
static uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
static uint16_t powerMax = 0;   // highest ever current

static int16_t i2c_errors_count = 0;
static int16_t annex650_overrun_count = 0;

// **********************
//Automatic ACC Offset Calibration
// **********************
static uint16_t InflightcalibratingA = 0;
static int16_t AccInflightCalibrationArmed;
static uint16_t AccInflightCalibrationMeasurementDone = 0;
static uint16_t AccInflightCalibrationSavetoEEProm = 0;

// **********************
// power meter
// **********************
#define PMOTOR_SUM 8            // index into pMeter[] for sum
static uint32_t pMeter[PMOTOR_SUM + 1]; //we use [0:7] for eight motors,one extra for sum
static uint8_t pMeterV;         // dummy to satisfy the paramStruct logic in ConfigurationLoop()
static uint32_t pAlarm;         // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
static uint8_t powerTrigger1 = 0;       // trigger for alarm based on power consumption
static uint16_t powerValue = 0; // last known current
static uint16_t intPowerMeterSum, intPowerTrigger1;

// **********************
// telemetry
// **********************
static uint8_t telemetry = 0;
static uint8_t telemetry_auto = 0;

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

volatile int16_t failsafeCnt = 0;
static int16_t failsafeEvents = 0;
static int16_t rcData[8];       // interval [1000;2000]
static int16_t rcCommand[4];    // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static uint8_t rcRate8;
static uint8_t rcExpo8;
static int16_t lookupRX[7];     //  lookup table for expo & RC rate
volatile uint8_t rcFrameComplete;       //for serial rc receiver Spektrum


// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = { 0, 0, 0 };
static int16_t gyroZero[3] = { 0, 0, 0 };
static int16_t accZero[3] = { 0, 0, 0 };
static int16_t magZero[3] = { 0, 0, 0 };
static int16_t angle[2] = { 0, 0 };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
static int8_t smallAngle25 = 1;

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[8];
static int16_t servo[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
static uint16_t wing_left_mid = WING_LEFT_MID;
static uint16_t wing_right_mid = WING_RIGHT_MID;
static uint16_t tri_yaw_middle = TRI_YAW_MIDDLE;

// **********************
// EEPROM & LCD functions
// **********************
static uint8_t P8[8], I8[8], D8[8];     //8 bits is much faster and the code is much shorter
static uint8_t dynP8[3], dynI8[3], dynD8[3];
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t activate1[CHECKBOXITEMS];
static uint8_t activate2[CHECKBOXITEMS];

// **********************
// GPS
// **********************
static int32_t GPS_latitude, GPS_longitude;
static int32_t GPS_latitude_home, GPS_longitude_home;
static uint8_t GPS_fix, GPS_fix_home = 0;
static uint8_t GPS_numSat;
static uint16_t GPS_distanceToHome;     // in meters
static int16_t GPS_directionToHome = 0; // in degrees
static uint8_t GPS_update = 0;  // it's a binary toogle to distinct a GPS position update
static int16_t GPS_angle[2];    // it's the angles that must be applied for GPS correction

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;
    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LEDPIN_TOGGLE;      //switch LEDPIN state
            BUZZERPIN_ON;
            delay(wait);
            BUZZERPIN_OFF;
        }
        delay(60);
    }
}

void annexCode()
{                               //this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
    static uint32_t buzzerTime, calibratedAccTime;
#if defined(LCD_TELEMETRY)
    static uint16_t telemetryTimer = 0, telemetryAutoTimer = 0, psensorTimer = 0;
#endif
#if defined(LCD_TELEMETRY)
    static uint8_t telemetryAutoIndex = 0;
    static char telemetryAutoSequence[] = LCD_TELEMETRY_AUTO;
#endif
#ifdef VBAT
    static uint8_t vbatTimer = 0;
#endif
    static uint8_t buzzerFreq;  //delay between buzzer ring
    uint8_t axis, prop1, prop2;
#if defined(POWERMETER_HARD)
    uint16_t pMeterRaw;         //used for current reading
#endif

    //PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < 1500) {
        prop2 = 100;
    } else if (rcData[THROTTLE] < 2000) {
        prop2 = 100 - (uint16_t) dynThrPID *(rcData[THROTTLE] - 1500) / 500;
    } else {
        prop2 = 100 - dynThrPID;
    }

    for (axis = 0; axis < 3; axis++) {
        uint16_t tmp = min(abs(rcData[axis] - MIDRC), 500);
#if defined(DEADBAND)
        if (tmp > DEADBAND) {
            tmp -= DEADBAND;
        } else {
            tmp = 0;
        }
#endif
        if (axis != 2) {        //ROLL & PITCH
            uint16_t tmp2 = tmp / 100;
            rcCommand[axis] = lookupRX[tmp2] + (tmp - tmp2 * 100) * (lookupRX[tmp2 + 1] - lookupRX[tmp2]) / 100;
            prop1 = 100 - (uint16_t) rollPitchRate *tmp / 500;
            prop1 = (uint16_t) prop1 *prop2 / 100;
        } else {                //YAW
            rcCommand[axis] = tmp;
            prop1 = 100 - (uint16_t) yawRate *tmp / 500;
        }
        dynP8[axis] = (uint16_t) P8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t) D8[axis] * prop1 / 100;
        if (rcData[axis] < MIDRC)
            rcCommand[axis] = -rcCommand[axis];
    }
    rcCommand[THROTTLE] = MINTHROTTLE + (int32_t) (MAXTHROTTLE - MINTHROTTLE) * (rcData[THROTTLE] - MINCHECK) / (2000 - MINCHECK);

    if (headFreeMode) {
        float radDiff = (heading - headFreeModeHold) * 0.0174533f;      // where PI/180 ~= 0.0174533
        float cosDiff = cos(radDiff);
        float sinDiff = sin(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }
#if defined(POWERMETER_HARD)
    if (!(++psensorTimer % PSENSORFREQ)) {
        pMeterRaw = analogRead(PSENSORPIN);
        powerValue = (PSENSORNULL > pMeterRaw ? PSENSORNULL - pMeterRaw : pMeterRaw - PSENSORNULL);     // do not use abs(), it would induce implicit cast to uint and overrun
        if (powerValue < 333) { // only accept reasonable values. 333 is empirical
#ifdef LOG_VALUES
            if (powerValue > powerMax)
                powerMax = powerValue;
#endif
        } else {
            powerValue = 333;
        }
        pMeter[PMOTOR_SUM] += (uint32_t) powerValue;
    }
#endif

#if defined(VBAT)
    static uint8_t ind = 0;
    uint16_t vbatRaw = 0;
    static uint16_t vbatRawArray[8];
    if (!(++vbatTimer % VBATFREQ)) {
        ADCSRA |= _BV(ADPS2);
        ADCSRA &= ~_BV(ADPS1);
        ADCSRA &= ~_BV(ADPS0);  //this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
        vbatRawArray[(ind++) % 8] = analogRead(V_BATPIN);
        for (uint8_t i = 0; i < 8; i++)
            vbatRaw += vbatRawArray[i];
        vbat = vbatRaw / (VBATSCALE / 2);       // result is Vbatt in 0.1V steps
    }
    if ((rcOptions1 & activate1[BOXBEEPERON]) || (rcOptions2 & activate2[BOXBEEPERON])) {       // unconditional beeper on via AUXn switch 
        buzzerFreq = 7;
    } else if (((vbat > VBATLEVEL1_3S)
#if defined(POWERMETER)
                && ((pMeter[PMOTOR_SUM] < pAlarm) || (pAlarm == 0))
#endif
               ) || (NO_VBAT > vbat))   // ToLuSe
    {                           //VBAT ok AND powermeter ok, buzzer off
        buzzerFreq = 0;
        buzzerState = 0;
        BUZZERPIN_OFF;
#if defined(POWERMETER)
    } else if (pMeter[PMOTOR_SUM] > pAlarm) {   // sound alarm for powermeter
        buzzerFreq = 4;
#endif
    } else if (vbat > VBATLEVEL2_3S)
        buzzerFreq = 1;
    else if (vbat > VBATLEVEL3_3S)
        buzzerFreq = 2;
    else
        buzzerFreq = 4;
    if (buzzerFreq) {
        if (buzzerState && (currentTime > buzzerTime + 250000)) {
            buzzerState = 0;
            BUZZERPIN_OFF;
            buzzerTime = currentTime;
        } else if (!buzzerState && (currentTime > (buzzerTime + (2000000 >> buzzerFreq)))) {
            buzzerState = 1;
            BUZZERPIN_ON;
            buzzerTime = currentTime;
        }
    }
#endif

    if ((calibratingA > 0 && (ACC || nunchuk)) || (calibratingG > 0)) { // Calibration phasis
        LEDPIN_TOGGLE;
    } else {
        if (calibratedACC == 1) {
            LEDPIN_OFF;
        }
        if (armed) {
            LEDPIN_ON;
        }
    }

#if defined(LED_RING)
    static uint32_t LEDTime;
    if (currentTime > LEDTime) {
        LEDTime = currentTime + 50000;
        i2CLedRingState();
    }
#endif

    if (currentTime > calibratedAccTime) {
        if (smallAngle25 == 0) {
            calibratedACC = 0;  //the multi uses ACC and is not calibrated or is too much inclinated
            LEDPIN_TOGGLE;
            calibratedAccTime = currentTime + 500000;
        } else
            calibratedACC = 1;
    }

    serialCom();

#if defined(POWERMETER)
    intPowerMeterSum = (pMeter[PMOTOR_SUM] / PLEVELDIV);
    intPowerTrigger1 = powerTrigger1 * PLEVELSCALE;
#endif

#ifdef LCD_TELEMETRY_AUTO
    if ((telemetry_auto)
        && (!(++telemetryAutoTimer % LCD_TELEMETRY_AUTO_FREQ))) {
        telemetry = telemetryAutoSequence[++telemetryAutoIndex % strlen(telemetryAutoSequence)];
        LCDclear();             // make sure to clear away remnants
    }
#endif
#ifdef LCD_TELEMETRY
    if (!(++telemetryTimer % LCD_TELEMETRY_FREQ)) {
#if (LCD_TELEMETRY_DEBUG+0 > 0)
        telemetry = LCD_TELEMETRY_DEBUG;
#endif
        if (telemetry)
            lcd_telemetry();
    }
#endif

#if defined(GPS)
    static uint32_t GPSLEDTime;
    if (currentTime > GPSLEDTime && (GPS_fix_home == 1)) {
        GPSLEDTime = currentTime + 150000;
        LEDPIN_TOGGLE;
    }
#endif
}


void setup()
{
    SerialOpen(0, 115200);
    LEDPIN_PINMODE;
    POWERPIN_PINMODE;
    BUZZERPIN_PINMODE;
    STABLEPIN_PINMODE;
    POWERPIN_OFF;
    initOutput();
    readEEPROM();
    checkFirstTime();
    configureReceiver();
    initSensors();
    previousTime = micros();
#if defined(GIMBAL)
    calibratingA = 400;
#endif
    calibratingG = 400;
#if defined(POWERMETER)
    for (uint8_t i = 0; i <= PMOTOR_SUM; i++)
        pMeter[i] = 0;
#endif
#if defined(GPS)
    SerialOpen(GPS_SERIAL, GPS_BAUD);
#endif
#if defined(LCD_ETPP)
    initLCD();
#elif defined(LCD_LCD03)
    initLCD();
#endif
#ifdef LCD_TELEMETRY_DEBUG
    telemetry_auto = 1;
#endif
#ifdef LCD_CONF_DEBUG
    configurationLoop();
#endif


}

// ******** Main Loop *********
void loop()
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    uint8_t axis, i;
    int16_t error, errorAngle;
    int16_t delta, deltaSum;
    int16_t PTerm, ITerm, DTerm;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int16_t delta1[3], delta2[3];
    static int16_t errorGyroI[3] = { 0, 0, 0 };
    static int16_t errorAngleI[2] = { 0, 0 };
    static uint32_t rcTime = 0;
    static int16_t initialThrottleHold;
    static int16_t errorAltitudeI = 0;
    int16_t AltPID = 0;
    static int16_t AltHold;

#if defined(SPEKTRUM)
    if (rcFrameComplete)
        computeRC();
#endif

    if (currentTime > rcTime) { // 50Hz
        rcTime = currentTime + 20000;
#if !(defined(SPEKTRUM) ||defined(BTSERIAL))
        computeRC();
#endif
        // Failsafe routine - added by MIS
#if defined(FAILSAFE)
        if (failsafeCnt > (5 * FAILSAVE_DELAY) && armed == 1) { // Stabilize, and set Throttle to specified level
            for (i = 0; i < 3; i++)
                rcData[i] = MIDRC;      // after specified guard time after RC signal is lost (in 0.1sec)
            rcData[THROTTLE] = FAILSAVE_THR0TTLE;
            if (failsafeCnt > 5 * (FAILSAVE_DELAY + FAILSAVE_OFF_DELAY)) {      // Turn OFF motors after specified Time (in 0.1sec)
                armed = 0;      //This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                okToArm = 0;    //to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
            }
            failsafeEvents++;
        }
        failsafeCnt++;
#endif
        // end of failsave routine - next change is made with RcOptions setting
        if (rcData[THROTTLE] < MINCHECK) {
            errorGyroI[ROLL] = 0;
            errorGyroI[PITCH] = 0;
            errorGyroI[YAW] = 0;
            errorAngleI[ROLL] = 0;
            errorAngleI[PITCH] = 0;
            rcDelayCommand++;
            if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && armed == 0) {
                if (rcDelayCommand == 20)
                    calibratingG = 400;
            } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
                if (rcDelayCommand == 20) {
                    servo[0] = 1500;    //we center the yaw gyro in conf mode
                    writeServos();
#if defined(LCD_CONF)
                    configurationLoop();        //beginning LCD configuration
#endif
                    previousTime = micros();
                }
            }
#if defined(InflightAccCalibration)
            else if (armed == 0 && rcData[YAW] < MINCHECK && rcData[PITCH] > MAXCHECK && rcData[ROLL] > MAXCHECK) {
                if (rcDelayCommand == 20) {
                    if (AccInflightCalibrationMeasurementDone) {        //trigger saving into eeprom after landing
                        AccInflightCalibrationMeasurementDone = 0;
                        AccInflightCalibrationSavetoEEProm = 1;
                    } else {
                        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
                        if (AccInflightCalibrationArmed) {
                            blinkLED(10, 1, 2);
                        } else {
                            blinkLED(10, 10, 3);
                        }
                    }
                }
            }
#endif
            else if ((activate1[BOXARM] > 0) || (activate2[BOXARM] > 0)) {
                if (((rcOptions1 & activate1[BOXARM])
                     || (rcOptions2 & activate2[BOXARM])) && okToArm) {
                    armed = 1;
                    headFreeModeHold = heading;
                } else if (armed)
                    armed = 0;
                rcDelayCommand = 0;
            } else if ((rcData[YAW] < MINCHECK || rcData[ROLL] < MINCHECK)
                       && armed == 1) {
                if (rcDelayCommand == 20)
                    armed = 0;  // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
            } else if ((rcData[YAW] > MAXCHECK || rcData[ROLL] > MAXCHECK)
                       && rcData[PITCH] < MAXCHECK && armed == 0 && calibratingG == 0 && calibratedACC == 1) {
                if (rcDelayCommand == 20) {
                    armed = 1;
                    headFreeModeHold = heading;
                }
#ifdef LCD_TELEMETRY_AUTO
            } else if (rcData[ROLL] < MINCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
                if (rcDelayCommand == 20) {
                    if (telemetry_auto) {
                        telemetry_auto = 0;
                        telemetry = 0;
                    } else
                        telemetry_auto = 1;
                }
#endif
            } else
                rcDelayCommand = 0;
        } else if (rcData[THROTTLE] > MAXCHECK && armed == 0) {
            if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {   //throttle=max, yaw=left, pitch=min
                if (rcDelayCommand == 20)
                    calibratingA = 400;
                rcDelayCommand++;
            } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MINCHECK) {    //throttle=max, yaw=right, pitch=min  
                if (rcDelayCommand == 20)
                    calibratingM = 1;   // MAG calibration request
                rcDelayCommand++;
            } else if (rcData[PITCH] > MAXCHECK) {
                accTrim[PITCH] += 2;
                writeParams();
#if defined(LED_RING)
                blinkLedRing();
#endif
            } else if (rcData[PITCH] < MINCHECK) {
                accTrim[PITCH] -= 2;
                writeParams();
#if defined(LED_RING)
                blinkLedRing();
#endif
            } else if (rcData[ROLL] > MAXCHECK) {
                accTrim[ROLL] += 2;
                writeParams();
#if defined(LED_RING)
                blinkLedRing();
#endif
            } else if (rcData[ROLL] < MINCHECK) {
                accTrim[ROLL] -= 2;
                writeParams();
#if defined(LED_RING)
                blinkLedRing();
#endif
            } else {
                rcDelayCommand = 0;
            }
        }
#ifdef LOG_VALUES
        if (cycleTime > cycleTimeMax)
            cycleTimeMax = cycleTime;   // remember highscore
        if (cycleTime < cycleTimeMin)
            cycleTimeMin = cycleTime;   // remember lowscore
#endif

#if defined(InflightAccCalibration)
        if (AccInflightCalibrationArmed && armed == 1 && rcData[THROTTLE] > MINCHECK && !((rcOptions1 & activate1[BOXARM]) || (rcOptions2 & activate2[BOXARM]))) {      // Copter is airborne and you are turning it off via boxarm : start measurement
            InflightcalibratingA = 50;
            AccInflightCalibrationArmed = 0;
        }
        if ((rcOptions1 & activate1[BOXPASSTHRU]) || (rcOptions2 & activate2[BOXPASSTHRU])) {   //Use the Passthru Option to activate : Passthru = TRUE Meausrement started, Land and passtrhu = 0 measurement stored
            if (!AccInflightCalibrationArmed) {
                AccInflightCalibrationArmed = 1;
                InflightcalibratingA = 50;
            }
        } else if (AccInflightCalibrationMeasurementDone && armed == 0) {
            AccInflightCalibrationArmed = 0;
            AccInflightCalibrationMeasurementDone = 0;
            AccInflightCalibrationSavetoEEProm = 1;
        }
#endif

        rcOptions1 = (rcData[AUX1] < 1300) + (1300 < rcData[AUX1]
                                              && rcData[AUX1] < 1700) * 2 + (rcData[AUX1] > 1700) * 4 + (rcData[AUX2] < 1300) * 8 + (1300 < rcData[AUX2]
                                                                                                                                     && rcData[AUX2] < 1700) * 16 + (rcData[AUX2] > 1700) * 32;
        rcOptions2 = (rcData[AUX3] < 1300) + (1300 < rcData[AUX3]
                                              && rcData[AUX3] < 1700) * 2 + (rcData[AUX3] > 1700) * 4 + (rcData[AUX4] < 1300) * 8 + (1300 < rcData[AUX4]
                                                                                                                                     && rcData[AUX4] < 1700) * 16 + (rcData[AUX4] > 1700) * 32;

        //note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
        if (((rcOptions1 & activate1[BOXACC])
             || (rcOptions2 & activate2[BOXACC])
             || (failsafeCnt > 5 * FAILSAVE_DELAY)) && (ACC || nunchuk)) {
            // bumpless transfer to Level mode
            if (!accMode) {
                errorAngleI[ROLL] = 0;
                errorAngleI[PITCH] = 0;
                accMode = 1;
            }
        } else
            accMode = 0;        // modified by MIS for failsave support

        if ((rcOptions1 & activate1[BOXARM]) == 0 || (rcOptions2 & activate2[BOXARM]) == 0)
            okToArm = 1;
        if (accMode == 1)
            STABLEPIN_ON;
        else
            STABLEPIN_OFF;

        if (BARO) {
            if ((rcOptions1 & activate1[BOXBARO])
                || (rcOptions2 & activate2[BOXBARO])) {
                if (baroMode == 0) {
                    baroMode = 1;
                    AltHold = EstAlt;
                    initialThrottleHold = rcCommand[THROTTLE];
                    errorAltitudeI = 0;
                }
            } else
                baroMode = 0;
        }
        if (MAG) {
            if ((rcOptions1 & activate1[BOXMAG])
                || (rcOptions2 & activate2[BOXMAG])) {
                if (magMode == 0) {
                    magMode = 1;
                    magHold = heading;
                }
            } else
                magMode = 0;
            if ((rcOptions1 & activate1[BOXHEADFREE])
                || (rcOptions2 & activate2[BOXHEADFREE])) {
                if (headFreeMode == 0) {
                    headFreeMode = 1;
                }
            } else
                headFreeMode = 0;
        }
#if defined(GPS)
        if ((rcOptions1 & activate1[BOXGPSHOME])
            || (rcOptions2 & activate2[BOXGPSHOME])) {
            GPSModeHome = 1;
        } else
            GPSModeHome = 0;
        if ((rcOptions1 & activate1[BOXGPSHOLD])
            || (rcOptions2 & activate2[BOXGPSHOLD])) {
            GPSModeHold = 1;
        } else
            GPSModeHold = 0;
#endif
        if ((rcOptions1 & activate1[BOXPASSTHRU])
            || (rcOptions2 & activate2[BOXPASSTHRU])) {
            passThruMode = 1;
        } else
            passThruMode = 0;
    }

    computeIMU();
    // Measure loop rate just afer reading the sensors
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;

    if (MAG) {
        if (abs(rcCommand[YAW]) < 70 && magMode) {
            int16_t dif = heading - magHold;
            if (dif <= -180)
                dif += 360;
            if (dif >= +180)
                dif -= 360;
            if (smallAngle25)
                rcCommand[YAW] -= dif * P8[PIDMAG] / 30;        //18 deg
        } else
            magHold = heading;
    }

    if (BARO) {
        if (baroMode) {
            if (abs(rcCommand[THROTTLE] - initialThrottleHold) > 20) {
                AltHold = EstAlt;
                initialThrottleHold = rcCommand[THROTTLE];
                errorAltitudeI = 0;
            }
            //**** Alt. Set Point stabilization PID ****
            error = constrain(AltHold - EstAlt, -100, 100);     //  +/-10m,  1 decimeter accuracy
            errorAltitudeI += error;
            errorAltitudeI = constrain(errorAltitudeI, -5000, 5000);

            PTerm = P8[PIDALT] * error / 10;    // 16 bits is ok here

            if (abs(error) > 5) // under 50cm error, we neutralize Iterm 
                ITerm = (int32_t) I8[PIDALT] * errorAltitudeI / 4000;
            else
                ITerm = 0;

            AltPID = PTerm + ITerm;

            //AltPID is reduced, depending of the zVelocity magnitude
            AltPID = AltPID * (D8[PIDALT] - min(abs(zVelocity), D8[PIDALT] * 4 / 5)) / (D8[PIDALT] + 1);
            debug3 = AltPID;

            rcCommand[THROTTLE] = initialThrottleHold + constrain(AltPID, -100, +100);
        }
    }
#if defined(GPS)
    if ((GPSModeHome == 1)) {
        float radDiff = (GPS_directionToHome - heading) * 0.0174533f;
        GPS_angle[ROLL] = constrain(P8[PIDGPS] * sin(radDiff) * GPS_distanceToHome / 10, -D8[PIDGPS] * 10, +D8[PIDGPS] * 10);   // with P=5, 1 meter = 0.5deg inclination
        GPS_angle[PITCH] = constrain(P8[PIDGPS] * cos(radDiff) * GPS_distanceToHome / 10, -D8[PIDGPS] * 10, +D8[PIDGPS] * 10);  // max inclination = D deg
    } else {
        GPS_angle[ROLL] = 0;
        GPS_angle[PITCH] = 0;
    }
#endif

    //**** PITCH & ROLL & YAW PID ****    
    for (axis = 0; axis < 3; axis++) {
        if (accMode == 1 && axis < 2) { //LEVEL MODE
            // 50 degrees max inclination
            errorAngle = constrain(2 * rcCommand[axis] - GPS_angle[axis], -500, +500) - angle[axis] + accTrim[axis];    //16 bits is ok here
#ifdef LEVEL_PDF
            PTerm = -(int32_t) angle[axis] * P8[PIDLEVEL] / 100;
#else
            PTerm = (int32_t) errorAngle *P8[PIDLEVEL] / 100;   //32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
#endif
            PTerm = constrain(PTerm, -D8[PIDLEVEL], +D8[PIDLEVEL]);

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000);      //WindUp     //16 bits is ok here
            ITerm = ((int32_t) errorAngleI[axis] * I8[PIDLEVEL]) >> 12; //32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
        } else {                //ACRO MODE or YAW axis
            if (abs(rcCommand[axis]) < 350)
                error = rcCommand[axis] * 10 * 8 / P8[axis];    //16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
            else
                error = (int32_t) rcCommand[axis] * 10 * 8 / P8[axis];  //32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
            error -= gyroData[axis];

            PTerm = rcCommand[axis];

            errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000);     //WindUp //16 bits is ok here
            if (abs(gyroData[axis]) > 640)
                errorGyroI[axis] = 0;
            ITerm = (errorGyroI[axis] / 125 * I8[axis]) >> 6;   //16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
        }
        if (abs(gyroData[axis]) < 160)
            PTerm -= gyroData[axis] * dynP8[axis] / 10 / 8;     //16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
        else
            PTerm -= (int32_t) gyroData[axis] * dynP8[axis] / 10 / 8;   //32 bits is needed for calculation   

        delta = gyroData[axis] - lastGyro[axis];        //16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastGyro[axis] = gyroData[axis];
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;

        if (abs(deltaSum) < 640)
            DTerm = (deltaSum * dynD8[axis]) >> 5;      //16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result 
        else
            DTerm = ((int32_t) deltaSum * dynD8[axis]) >> 5;    //32 bits is needed for calculation

        axisPID[axis] = PTerm + ITerm - DTerm;
    }

    mixTable();
    writeServos();
    writeMotors();

#if defined(GPS)
    while (SerialAvailable(GPS_SERIAL)) {
        if (GPS_newFrame(SerialRead(GPS_SERIAL))) {
            if (GPS_update == 1)
                GPS_update = 0;
            else
                GPS_update = 1;
            if (GPS_fix == 1 && GPS_numSat == 4) {
                if (GPS_fix_home == 0) {
                    GPS_fix_home = 1;
                    GPS_latitude_home = GPS_latitude;
                    GPS_longitude_home = GPS_longitude;
                }
                GPS_distance(GPS_latitude_home, GPS_longitude_home, GPS_latitude, GPS_longitude, &GPS_distanceToHome, &GPS_directionToHome);
            }
        }
    }
#endif
}
