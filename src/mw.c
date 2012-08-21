#include "board.h"
#include "mw.h"

// July 2012     V2.1

flags_t f;
int16_t debug[4];
uint8_t toggleBeep = 0;
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
int16_t headFreeModeHold;

int16_t annex650_overrun_count = 0;
uint8_t vbat;                   // battery voltage in 0.1V steps
int16_t telemTemperature1;      // gyro sensor temperature

int16_t failsafeCnt = 0;
int16_t failsafeEvents = 0;
int16_t rcData[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 }; // interval [1000;2000]
int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
int16_t lookupPitchRollRC[6];   // lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];   // lookup table for expo & mid THROTTLE
rcReadRawDataPtr rcReadRawFunc = NULL;  // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)

uint8_t dynP8[3], dynI8[3], dynD8[3];
uint8_t rcOptions[CHECKBOXITEMS];

int16_t axisPID[3];

// **********************
// GPS
// **********************
int32_t GPS_coord[2];
int32_t GPS_home[2];
int32_t GPS_hold[2];
uint8_t GPS_numSat;
uint16_t GPS_distanceToHome;                            // distance to home point in meters
int16_t GPS_directionToHome;                            // direction to home or hol point in degrees
uint16_t GPS_altitude, GPS_speed;       // altitude in 0.1m and speed in 0.1m/s
uint8_t GPS_update = 0;             // it's a binary toogle to distinct a GPS position update
int16_t GPS_angle[2] = { 0, 0 };    // it's the angles that must be applied for GPS correction
uint16_t GPS_ground_course = 0;     // degrees*10
uint8_t GPS_Present = 0;            // Checksum from Gps serial
uint8_t GPS_Enable = 0;
int16_t nav[2];
int16_t nav_rated[2];               // Adding a rate controller to the navigation to make it smoother
int8_t nav_mode = NAV_MODE_NONE;    // Navigation mode

//Automatic ACC Offset Calibration
// **********************
uint16_t InflightcalibratingA = 0;
int16_t AccInflightCalibrationArmed;
uint16_t AccInflightCalibrationMeasurementDone = 0;
uint16_t AccInflightCalibrationSavetoEEProm = 0;
uint16_t AccInflightCalibrationActive = 0;

// Battery monitoring stuff
uint8_t batteryCellCount = 3;   // cell count
uint16_t batteryWarningVoltage; // annoying buzzer after this one, battery ready to be dead

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LED0_TOGGLE;        // switch LEDPIN state
            BEEP_ON;
            delay(wait);
            BEEP_OFF;
        }
        delay(60);
    }
}

#define BREAKPOINT 1500

// this code is executed at each loop and won't interfere with control loop if it lasts less than 650 microseconds
void annexCode(void)
{
    static uint32_t calibratedAccTime;
    uint16_t tmp, tmp2;
    static uint8_t buzzerFreq;  //delay between buzzer ring
    static uint8_t vbatTimer = 0;
    uint8_t axis, prop1, prop2;
    static uint8_t ind = 0;
    uint16_t vbatRaw = 0;
    static uint16_t vbatRawArray[8];
    uint8_t i;

    // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < BREAKPOINT) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t) cfg.dynThrPID * (rcData[THROTTLE] - BREAKPOINT) / (2000 - BREAKPOINT);
        } else {
            prop2 = 100 - cfg.dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++) {
        tmp = min(abs(rcData[axis] - cfg.midrc), 500);
        if (axis != 2) {        // ROLL & PITCH
            if (cfg.deadband) {
                if (tmp > cfg.deadband) {
                    tmp -= cfg.deadband;
                } else {
                    tmp = 0;
                }
            }

            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
            prop1 = 100 - (uint16_t) cfg.rollPitchRate * tmp / 500;
            prop1 = (uint16_t) prop1 *prop2 / 100;
        } else {                // YAW
            if (cfg.yawdeadband) {
                if (tmp > cfg.yawdeadband) {
                    tmp -= cfg.yawdeadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = tmp;
            prop1 = 100 - (uint16_t) cfg.yawRate * tmp / 500;
        }
        dynP8[axis] = (uint16_t) cfg.P8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t) cfg.D8[axis] * prop1 / 100;
        if (rcData[axis] < cfg.midrc)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rcData[THROTTLE], cfg.mincheck, 2000);
    tmp = (uint32_t) (tmp - cfg.mincheck) * 1000 / (2000 - cfg.mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    if(f.HEADFREE_MODE) {
        float radDiff = (heading - headFreeModeHold) * M_PI / 180.0f;
        float cosDiff = cosf(radDiff);
        float sinDiff = sinf(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }

    if (feature(FEATURE_VBAT)) {
        if (!(++vbatTimer % VBATFREQ)) {
            vbatRawArray[(ind++) % 8] = adcGetBattery();
            for (i = 0; i < 8; i++)
                vbatRaw += vbatRawArray[i];
            vbat = batteryAdcToVoltage(vbatRaw / 8);
        }
        if ((vbat > batteryWarningVoltage) || (vbat < cfg.vbatmincellvoltage)) { // VBAT ok, buzzer off
            buzzerFreq = 0;
        } else
            buzzerFreq = 4;     // low battery
    }

    buzzer(buzzerFreq);         // external buzzer routine that handles buzzer events globally now

    if ((calibratingA > 0 && sensors(SENSOR_ACC)) || (calibratingG > 0)) {      // Calibration phasis
        LED0_TOGGLE;
    } else {
        if (f.ACC_CALIBRATED)
            LED0_OFF;
        if (f.ARMED)
            LED0_ON;
        // This will switch to/from 9600 or 115200 baud depending on state. Of course, it should only do it on changes.
        if (feature(FEATURE_TELEMETRY))
            initTelemetry(f.ARMED);
    }

#ifdef LEDRING
    if (feature(FEATURE_LED_RING)) {
        static uint32_t LEDTime;
        if ((int32_t)(currentTime - LEDTime) >= 0) {
            LEDTime = currentTime + 50000;
            ledringState();
        }
    }
#endif

    if ((int32_t)(currentTime - calibratedAccTime) >= 0) {
        if (!f.SMALL_ANGLES_25) {
            f.ACC_CALIBRATED = 0; // the multi uses ACC and is not calibrated or is too much inclinated
            LED0_TOGGLE;
            calibratedAccTime = currentTime + 500000;
        } else {
            f.ACC_CALIBRATED = 1;
        }
    }

    serialCom();

    if (sensors(SENSOR_GPS)) {
        static uint32_t GPSLEDTime;
        if ((int32_t)(currentTime - GPSLEDTime) >= 0 && (GPS_numSat >= 5)) {
            GPSLEDTime = currentTime + 150000;
            LED1_TOGGLE;
        }
    }

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature)
        gyro.temperature(&telemTemperature1);
    else {
        // TODO MCU temp
    }
}

uint16_t pwmReadRawRC(uint8_t chan)
{
    uint16_t data;

    data = pwmRead(cfg.rcmap[chan]);
    if (data < 750 || data > 2250)
        data = cfg.midrc;

    return data;
}

void computeRC(void)
{
    static int16_t rcData4Values[8][4], rcDataMean[8];
    static uint8_t rc4ValuesIndex = 0;
    uint8_t chan, a;

    rc4ValuesIndex++;
    for (chan = 0; chan < 8; chan++) {
        rcData4Values[chan][rc4ValuesIndex % 4] = rcReadRawFunc(chan);
        rcDataMean[chan] = 0;
        for (a = 0; a < 4; a++)
            rcDataMean[chan] += rcData4Values[chan][a];

        rcDataMean[chan] = (rcDataMean[chan] + 2) / 4;
        if (rcDataMean[chan] < rcData[chan] - 3)
            rcData[chan] = rcDataMean[chan] + 2;
        if (rcDataMean[chan] > rcData[chan] + 3)
            rcData[chan] = rcDataMean[chan] - 2;
    }
}

// #define TIMINGDEBUG

#ifdef TIMINGDEBUG
uint32_t trollTime = 0;
uint16_t cn = 0xffff, cx = 0x0;
#endif

void loop(void)
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
    static uint32_t loopTime;
    uint16_t auxState = 0;

    // this will return false if spektrum is disabled. shrug.
    if (spektrumFrameComplete())
        computeRC();

    if ((int32_t)(currentTime - rcTime) >= 0) { // 50Hz
        rcTime = currentTime + 20000;
        // TODO clean this up. computeRC should handle this check
        if (!feature(FEATURE_SPEKTRUM))
            computeRC();

        // Failsafe routine
        if (feature(FEATURE_FAILSAFE)) {
            if (failsafeCnt > (5 * cfg.failsafe_delay) && f.ARMED) { // Stabilize, and set Throttle to specified level
                for (i = 0; i < 3; i++)
                    rcData[i] = cfg.midrc;      // after specified guard time after RC signal is lost (in 0.1sec)
                rcData[THROTTLE] = cfg.failsafe_throttle;
                if (failsafeCnt > 5 * (cfg.failsafe_delay + cfg.failsafe_off_delay)) {  // Turn OFF motors after specified Time (in 0.1sec)
                    f.ARMED = 0;  // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                    f.OK_TO_ARM = 0;        // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
                }
                failsafeEvents++;
            }
            if (failsafeCnt > (5 * cfg.failsafe_delay) && !f.ARMED) {  // Turn off "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
                f.ARMED = 0;        // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                f.OK_TO_ARM = 0;    // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
            }
            failsafeCnt++;
        }

        if (rcData[THROTTLE] < cfg.mincheck) {
            errorGyroI[ROLL] = 0;
            errorGyroI[PITCH] = 0;
            errorGyroI[YAW] = 0;
            errorAngleI[ROLL] = 0;
            errorAngleI[PITCH] = 0;
            rcDelayCommand++;
            if (rcData[YAW] < cfg.mincheck && rcData[PITCH] < cfg.mincheck && !f.ARMED) {
                if (rcDelayCommand == 20) {
                    calibratingG = 1000;
                    if (feature(FEATURE_GPS))
                        GPS_reset_home_position();
                }
            } else if (feature(FEATURE_INFLIGHT_ACC_CAL) && (!f.ARMED && rcData[YAW] < cfg.mincheck && rcData[PITCH] > cfg.maxcheck && rcData[ROLL] > cfg.maxcheck)) {
                if (rcDelayCommand == 20) {
                    if (AccInflightCalibrationMeasurementDone) {        // trigger saving into eeprom after landing
                        AccInflightCalibrationMeasurementDone = 0;
                        AccInflightCalibrationSavetoEEProm = 1;
                    } else {
                        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
                        if (AccInflightCalibrationArmed) {
                            toggleBeep = 2;
                        } else {
                            toggleBeep = 3;
                        }
                    }
                }
            } else if (cfg.activate[BOXARM] > 0) {
                if (rcOptions[BOXARM] && f.OK_TO_ARM) {
                    // TODO: feature(FEATURE_FAILSAFE) && failsafeCnt == 0
                    f.ARMED = 1;
                    headFreeModeHold = heading;
                } else if (f.ARMED)
                    f.ARMED = 0;
                rcDelayCommand = 0;
            } else if ((rcData[YAW] < cfg.mincheck || (cfg.retarded_arm == 1 && rcData[ROLL] < cfg.mincheck)) && f.ARMED) {
                if (rcDelayCommand == 20)
                    f.ARMED = 0;  // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
            } else if ((rcData[YAW] > cfg.maxcheck || (rcData[ROLL] > cfg.maxcheck && cfg.retarded_arm == 1)) && rcData[PITCH] < cfg.maxcheck && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {
                if (rcDelayCommand == 20) {
                    f.ARMED = 1;
                    headFreeModeHold = heading;
                }
            } else
                rcDelayCommand = 0;
        } else if (rcData[THROTTLE] > cfg.maxcheck && !f.ARMED) {
            if (rcData[YAW] < cfg.mincheck && rcData[PITCH] < cfg.mincheck) {   // throttle=max, yaw=left, pitch=min
                if (rcDelayCommand == 20)
                    calibratingA = 400;
                rcDelayCommand++;
            } else if (rcData[YAW] > cfg.maxcheck && rcData[PITCH] < cfg.mincheck) {    // throttle=max, yaw=right, pitch=min
                if (rcDelayCommand == 20)
                    f.CALIBRATE_MAG = 1;   // MAG calibration request
                rcDelayCommand++;
            } else if (rcData[PITCH] > cfg.maxcheck) {
                cfg.angleTrim[PITCH] += 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED_RING))
                    ledringBlink();
#endif
            } else if (rcData[PITCH] < cfg.mincheck) {
                cfg.angleTrim[PITCH] -= 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED_RING))
                    ledringBlink();
#endif
            } else if (rcData[ROLL] > cfg.maxcheck) {
                cfg.angleTrim[ROLL] += 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED_RING))
                    ledringBlink();
#endif
            } else if (rcData[ROLL] < cfg.mincheck) {
                cfg.angleTrim[ROLL] -= 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED_RING))
                    ledringBlink();
#endif
            } else {
                rcDelayCommand = 0;
            }
        }

        if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
            if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > cfg.mincheck && !rcOptions[BOXARM]) {   // Copter is airborne and you are turning it off via boxarm : start measurement
                InflightcalibratingA = 50;
                AccInflightCalibrationArmed = 0;
            }
            if (rcOptions[BOXPASSTHRU]) {      // Use the Passthru Option to activate : Passthru = TRUE Meausrement started, Land and passtrhu = 0 measurement stored
                if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
                    InflightcalibratingA = 50;
            } else if (AccInflightCalibrationMeasurementDone && !f.ARMED) {
                AccInflightCalibrationMeasurementDone = 0;
                AccInflightCalibrationSavetoEEProm = 1;
            }
        }

        for(i = 0; i < 4; i++)
            auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
        for(i = 0; i < CHECKBOXITEMS; i++)
            rcOptions[i] = (auxState & cfg.activate[i]) > 0;

        // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
        if ((rcOptions[BOXACC] || (failsafeCnt > 5 * cfg.failsafe_delay)) && (sensors(SENSOR_ACC))) {
            // bumpless transfer to Level mode
            if (!f.ACC_MODE) {
                errorAngleI[ROLL] = 0;
                errorAngleI[PITCH] = 0;
                f.ACC_MODE = 1;
            }
        } else
            f.ACC_MODE = 0;        // failsave support

        if ((rcOptions[BOXARM]) == 0)
            f.OK_TO_ARM = 1;
        if (f.ACC_MODE) {
            LED1_ON;
        } else {
            LED1_OFF;
        }

#ifdef BARO
        if (sensors(SENSOR_BARO)) {
            if (rcOptions[BOXBARO]) {
                if (!f.BARO_MODE) {
                    f.BARO_MODE = 1;
                    AltHold = EstAlt;
                    initialThrottleHold = rcCommand[THROTTLE];
                    errorAltitudeI = 0;
                    BaroPID = 0;
                }
            } else
                f.BARO_MODE = 0;
        }
#endif

#ifdef  MAG
        if (sensors(SENSOR_MAG)) {
            if (rcOptions[BOXMAG]) {
                if (!f.MAG_MODE) {
                    f.MAG_MODE = 1;
                    magHold = heading;
                }
            } else
                f.MAG_MODE = 0;
            if (rcOptions[BOXHEADFREE]) {
                if (!f.HEADFREE_MODE) {
                    f.HEADFREE_MODE = 1;
                }
            } else {
                f.HEADFREE_MODE = 0;
            }
            if (rcOptions[BOXHEADADJ]) {
                headFreeModeHold = heading; // acquire new heading
            }
        }
#endif

        if (sensors(SENSOR_GPS)) {
            if (f.GPS_FIX && GPS_numSat >= 5) {
                if (rcOptions[BOXGPSHOME]) {
                    if (!f.GPS_HOME_MODE) {
                        f.GPS_HOME_MODE = 1;
                        GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
                        nav_mode = NAV_MODE_WP;
                    }
                } else {
                    f.GPS_HOME_MODE = 0;
                }
                if (rcOptions[BOXGPSHOLD]) {
                    if (!f.GPS_HOLD_MODE) {
                        f.GPS_HOLD_MODE = 1;
                        GPS_hold[LAT] = GPS_coord[LAT];
                        GPS_hold[LON] = GPS_coord[LON];
                        GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
                        nav_mode = NAV_MODE_POSHOLD;
                    }
                } else {
                    f.GPS_HOLD_MODE = 0;
                }
            }
        }

        if (rcOptions[BOXPASSTHRU]) {
            f.PASSTHRU_MODE = 1;
        } else {
            f.PASSTHRU_MODE = 0;
        }
    } else {                    // not in rc loop
        static int8_t taskOrder = 0;    // never call all function in the same loop, to avoid high delay spikes
        switch (taskOrder++ % 4) {
        case 0:
#ifdef MAG
            if (sensors(SENSOR_MAG))
                Mag_getADC();
#endif
            break;
        case 1:
#ifdef BARO
            if (sensors(SENSOR_BARO))
                Baro_update();
#endif
            break;
        case 2:
#ifdef BARO
            if (sensors(SENSOR_BARO))
                getEstimatedAltitude();
#endif
            break;
        case 3:
#ifdef SONAR
            if (sensors(SENSOR_SONAR)) {
                Sonar_update();
                debug[2] = sonarAlt;
            }
#endif
            break;
        default:
            taskOrder = 0;
            break;
        }
    }

    currentTime = micros();
    if (cfg.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + cfg.looptime;

        computeIMU();
        // Measure loop rate just afer reading the sensors
        currentTime = micros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;
    
#ifdef MPU6050_DMP
        mpu6050DmpLoop();
#endif
    
#ifdef MAG
        if (sensors(SENSOR_MAG)) {
            if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) {
                int16_t dif = heading - magHold;
                if (dif <= -180)
                    dif += 360;
                if (dif >= +180)
                    dif -= 360;
                if (f.SMALL_ANGLES_25)
                    rcCommand[YAW] -= dif * cfg.P8[PIDMAG] / 30;    // 18 deg
            } else
                magHold = heading;
        }
#endif

#ifdef BARO
        if (sensors(SENSOR_BARO)) {
            if (f.BARO_MODE) {
                if (abs(rcCommand[THROTTLE] - initialThrottleHold) > 20) {
                    f.BARO_MODE = 0;   // so that a new althold reference is defined
                }
                rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
            }
        }
#endif

        if (sensors(SENSOR_GPS)) {
            // Check that we really need to navigate ?
            if ((!f.GPS_HOME_MODE && !f.GPS_HOLD_MODE) || (!f.GPS_FIX_HOME)) {
                // If not. Reset nav loops and all nav related parameters
                GPS_reset_nav();
            } else {
                float sin_yaw_y = sinf(heading * 0.0174532925f);
                float cos_yaw_x = cosf(heading * 0.0174532925f);
                if (cfg.nav_slew_rate) {
                    nav_rated[LON] += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -cfg.nav_slew_rate, cfg.nav_slew_rate); // TODO check this on uint8
                    nav_rated[LAT] += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]),-cfg.nav_slew_rate, cfg.nav_slew_rate);
                    GPS_angle[ROLL] = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
                    GPS_angle[PITCH] = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
                } else {
                    GPS_angle[ROLL] = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
                    GPS_angle[PITCH] = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
                }
            }
        }
        // **** PITCH & ROLL & YAW PID ****    
        for (axis = 0; axis < 3; axis++) {
            if (f.ACC_MODE && axis < 2) { // LEVEL MODE
                // 50 degrees max inclination
                errorAngle = constrain(2 * rcCommand[axis] - GPS_angle[axis], -500, +500) - angle[axis] + cfg.angleTrim[axis];
#ifdef LEVEL_PDF
                PTerm = -(int32_t) angle[axis] * cfg.P8[PIDLEVEL] / 100;
#else
                PTerm = (int32_t) errorAngle *cfg.P8[PIDLEVEL] / 100;       //32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
#endif
                PTerm = constrain(PTerm, -cfg.D8[PIDLEVEL] * 5, +cfg.D8[PIDLEVEL] * 5);
    
                errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000);      // WindUp     // 16 bits is ok here
                ITerm = ((int32_t) errorAngleI[axis] * cfg.I8[PIDLEVEL]) >> 12;     // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
            } else {                // ACRO MODE or YAW axis
                error = (int32_t) rcCommand[axis] * 10 * 8 / cfg.P8[axis];  //32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
                error -= gyroData[axis];
    
                PTerm = rcCommand[axis];
    
                errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000);     // WindUp // 16 bits is ok here
                if (abs(gyroData[axis]) > 640)
                    errorGyroI[axis] = 0;
                ITerm = (errorGyroI[axis] / 125 * cfg.I8[axis]) >> 6;       // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
            }
            PTerm -= (int32_t) gyroData[axis] * dynP8[axis] / 10 / 8;       // 32 bits is needed for calculation
    
            delta = gyroData[axis] - lastGyro[axis];        //16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
            lastGyro[axis] = gyroData[axis];
            deltaSum = delta1[axis] + delta2[axis] + delta;
            delta2[axis] = delta1[axis];
            delta1[axis] = delta;
    
            DTerm = ((int32_t) deltaSum * dynD8[axis]) >> 5;        //32 bits is needed for calculation
    
            axisPID[axis] = PTerm + ITerm - DTerm;
        }
    
        mixTable();
        writeServos();
        writeMotors();
    }
}
