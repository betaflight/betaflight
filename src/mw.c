#include "board.h"
#include "mw.h"

// October 2012     V2.1-dev

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
int16_t rcData[8] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // interval [1000;2000]
int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
int16_t lookupPitchRollRC[6];   // lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];   // lookup table for expo & mid THROTTLE
uint16_t rssi;                  // range: [0;1023]
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
uint16_t GPS_distanceToHome;        // distance to home point in meters
int16_t GPS_directionToHome;        // direction to home or hol point in degrees
uint16_t GPS_altitude, GPS_speed;   // altitude in 0.1m and speed in 0.1m/s
uint8_t GPS_update = 0;             // it's a binary toogle to distinct a GPS position update
int16_t GPS_angle[2] = { 0, 0 };    // it's the angles that must be applied for GPS correction
uint16_t GPS_ground_course = 0;     // degrees * 10
uint8_t GPS_Present = 0;            // Checksum from Gps serial
uint8_t GPS_Enable = 0;
int16_t nav[2];
int16_t nav_rated[2];               // Adding a rate controller to the navigation to make it smoother
int8_t nav_mode = NAV_MODE_NONE;    // Navigation mode
uint8_t  GPS_numCh;                 // Number of channels
uint8_t  GPS_svinfo_chn[16];        // Channel number
uint8_t  GPS_svinfo_svid[16];       // Satellite ID
uint8_t  GPS_svinfo_quality[16];    // Bitfield Qualtity
uint8_t  GPS_svinfo_cno[16];        // Carrier to Noise Ratio (Signal Strength)

// Automatic ACC Offset Calibration
uint16_t InflightcalibratingA = 0;
int16_t AccInflightCalibrationArmed;
uint16_t AccInflightCalibrationMeasurementDone = 0;
uint16_t AccInflightCalibrationSavetoEEProm = 0;
uint16_t AccInflightCalibrationActive = 0;

// Battery monitoring stuff
uint8_t batteryCellCount = 3;       // cell count
uint16_t batteryWarningVoltage;     // annoying buzzer after this one, battery ready to be dead

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LED0_TOGGLE;            // switch LEDPIN state
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
        tmp = min(abs(rcData[axis] - mcfg.midrc), 500);
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
        if (rcData[axis] < mcfg.midrc)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rcData[THROTTLE], mcfg.mincheck, 2000);
    tmp = (uint32_t) (tmp - mcfg.mincheck) * 1000 / (2000 - mcfg.mincheck);       // [MINCHECK;2000] -> [0;1000]
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
            vbatRawArray[(ind++) % 8] = adcGetChannel(ADC_BATTERY);
            for (i = 0; i < 8; i++)
                vbatRaw += vbatRawArray[i];
            vbat = batteryAdcToVoltage(vbatRaw / 8);
        }
        if ((vbat > batteryWarningVoltage) || (vbat < mcfg.vbatmincellvoltage)) { // VBAT ok, buzzer off
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

    data = pwmRead(mcfg.rcmap[chan]);
    if (data < 750 || data > 2250)
        data = mcfg.midrc;

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

static void mwArm(void)
{
    if (calibratingG == 0 && f.ACC_CALIBRATED) {
        // TODO: feature(FEATURE_FAILSAFE) && failsafeCnt < 2
        // TODO: && ( !feature || ( feature && ( failsafecnt > 2) )
        if (!f.ARMED) {         // arm now!
            f.ARMED = 1;
            headFreeModeHold = heading;
        }
    } else if (!f.ARMED) {
        blinkLED(2, 255, 1);
    }
}

static void mwDisarm(void)
{
    if (f.ARMED)
        f.ARMED = 0;
}

static void mwVario(void)
{
    
}

void loop(void)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint8_t rcSticks;            // this hold sticks position for command combos
    uint8_t stTmp = 0;
    uint8_t axis, i;
    int16_t error, errorAngle;
    int16_t PTerm, ITerm, PTermACC, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm;
    static int16_t errorGyroI[3] = { 0, 0, 0 };
    static int16_t errorAngleI[2] = { 0, 0 };
    int16_t delta;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int16_t delta1[3], delta2[3];
    int16_t deltaSum;
    static uint32_t rcTime = 0;
    static int16_t initialThrottleHold;
    static uint32_t loopTime;
    uint16_t auxState = 0;
    int16_t prop;
    static uint8_t GPSNavReset = 1;

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
                    rcData[i] = mcfg.midrc;      // after specified guard time after RC signal is lost (in 0.1sec)
                rcData[THROTTLE] = cfg.failsafe_throttle;
                if (failsafeCnt > 5 * (cfg.failsafe_delay + cfg.failsafe_off_delay)) {  // Turn OFF motors after specified Time (in 0.1sec)
                    mwDisarm();             // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                    f.OK_TO_ARM = 0;        // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
                }
                failsafeEvents++;
            }
            if (failsafeCnt > (5 * cfg.failsafe_delay) && !f.ARMED) {  // Turn off "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
                mwDisarm();         // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                f.OK_TO_ARM = 0;    // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
            }
            failsafeCnt++;
        }
        // end of failsafe routine - next change is made with RcOptions setting

        // ------------------ STICKS COMMAND HANDLER --------------------
        // checking sticks positions
        for (i = 0; i < 4; i++) {
            stTmp >>= 2;
            if (rcData[i] > mcfg.mincheck)
                stTmp |= 0x80;  // check for MIN
            if (rcData[i] < mcfg.maxcheck)
                stTmp |= 0x40;  // check for MAX
        }
        if (stTmp == rcSticks) {
            if (rcDelayCommand < 250)
                rcDelayCommand++;
        } else
            rcDelayCommand = 0;
        rcSticks = stTmp;

        // perform actions
        if (rcData[THROTTLE] < mcfg.mincheck) {
            errorGyroI[ROLL] = 0;
            errorGyroI[PITCH] = 0;
            errorGyroI[YAW] = 0;
            errorAngleI[ROLL] = 0;
            errorAngleI[PITCH] = 0;
            if (cfg.activate[BOXARM] > 0) { // Arming/Disarming via ARM BOX
                if (rcOptions[BOXARM] && f.OK_TO_ARM)
                    mwArm();
                else if (f.ARMED)
                    mwDisarm();
            }
        }

        if (rcDelayCommand == 20) {
            if (f.ARMED) {      // actions during armed
                // Disarm on throttle down + yaw
                if (cfg.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE))
                    mwDisarm();
                // Disarm on roll (only when retarded_arm is enabled)
                if (mcfg.retarded_arm && cfg.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO))
                    mwDisarm();
            } else {            // actions during not armed
                i = 0;
                // GYRO calibration
                if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
                    calibratingG = 1000;
                    if (feature(FEATURE_GPS))
                        GPS_reset_home_position();
                    if (sensors(SENSOR_BARO))
                        calibratingB = 10; // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
                // Inflight ACC Calibration
                } else if (feature(FEATURE_INFLIGHT_ACC_CAL) && (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI)) {
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

                // Multiple configuration profiles
                if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO)          // ROLL left  -> Profile 1
                    i = 1;
                else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE)     // PITCH up   -> Profile 2
                    i = 2;
                else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)     // ROLL right -> Profile 3
                    i = 3;
                if (i) {
                    mcfg.current_profile = i - 1;
                    writeEEPROM(0, false);
                    blinkLED(2, 40, i);
                    // TODO alarmArray[0] = i;
                }

                // Arm via YAW
                if (cfg.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE))
                    mwArm();
                // Arm via ROLL
                else if (mcfg.retarded_arm && cfg.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI))
                    mwArm();
                // Calibrating Acc
                else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    calibratingA = 400;
                // Calibrating Mag
                else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE)
                    f.CALIBRATE_MAG = 1;
                i = 0;
                // Acc Trim
                if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
                    cfg.angleTrim[PITCH] += 2;
                    i = 1;
                } else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
                    cfg.angleTrim[PITCH] -= 2;
                    i = 1;
                } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
                    cfg.angleTrim[ROLL] += 2;
                    i = 1;
                } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
                    cfg.angleTrim[ROLL] -= 2;
                    i = 1;
                }
                if (i) {
                    writeEEPROM(1, false);
                    rcDelayCommand = 0; // allow autorepetition
                }
            }
        }

        if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
            if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > mcfg.mincheck && !rcOptions[BOXARM]) {   // Copter is airborne and you are turning it off via boxarm : start measurement
                InflightcalibratingA = 50;
                AccInflightCalibrationArmed = 0;
            }
            if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
                if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
                    InflightcalibratingA = 50;
            } else if (AccInflightCalibrationMeasurementDone && !f.ARMED) {
                AccInflightCalibrationMeasurementDone = 0;
                AccInflightCalibrationSavetoEEProm = 1;
            }
        }

        // Check AUX switches
        for (i = 0; i < 4; i++)
            auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
        for (i = 0; i < CHECKBOXITEMS; i++)
            rcOptions[i] = (auxState & cfg.activate[i]) > 0;

        // note: if FAILSAFE is disable, failsafeCnt > 5 * FAILSAVE_DELAY is always false
        if ((rcOptions[BOXANGLE] || (failsafeCnt > 5 * cfg.failsafe_delay)) && (sensors(SENSOR_ACC))) {
            // bumpless transfer to Level mode
            if (!f.ANGLE_MODE) {
                errorAngleI[ROLL] = 0;
                errorAngleI[PITCH] = 0;
                f.ANGLE_MODE = 1;
            }
        } else {
            f.ANGLE_MODE = 0;        // failsave support
        }

        if (rcOptions[BOXHORIZON]) {
            f.ANGLE_MODE = 0;
            if (!f.HORIZON_MODE) {
                errorAngleI[ROLL] = 0; 
                errorAngleI[PITCH] = 0;
                f.HORIZON_MODE = 1;
            }
        } else {
            f.HORIZON_MODE = 0;
        }

        if ((rcOptions[BOXARM]) == 0)
            f.OK_TO_ARM = 1;
        if (f.ANGLE_MODE || f.HORIZON_MODE) {
            LED1_ON;
        } else {
            LED1_OFF;
        }

#ifdef BARO
        if (sensors(SENSOR_BARO)) {
            // Baro alt hold activate
            if (rcOptions[BOXBARO]) {
                if (!f.BARO_MODE) {
                    f.BARO_MODE = 1;
                    AltHold = EstAlt;
                    initialThrottleHold = rcCommand[THROTTLE];
                    errorAltitudeI = 0;
                    BaroPID = 0;
                }
            } else {
                f.BARO_MODE = 0;
            }
            // Vario signalling activate
            if (feature(FEATURE_VARIO)) {
                if (rcOptions[BOXVARIO]) {
                    if (!f.VARIO_MODE) {
                        f.VARIO_MODE = 1;
                    }
                } else {
                    f.VARIO_MODE = 0;
                }
            }
        }
#endif

#ifdef  MAG
        if (sensors(SENSOR_MAG)) {
            if (rcOptions[BOXMAG]) {
                if (!f.MAG_MODE) {
                    f.MAG_MODE = 1;
                    magHold = heading;
                }
            } else {
                f.MAG_MODE = 0;
            }
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
                // if both GPS_HOME & GPS_HOLD are checked => GPS_HOME is the priority
                if (rcOptions[BOXGPSHOME]) {
                    if (!f.GPS_HOME_MODE) {
                        f.GPS_HOME_MODE = 1;
                        f.GPS_HOLD_MODE = 0;
                        GPSNavReset = 0;
                        GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
                        nav_mode = NAV_MODE_WP;
                    }
                } else {
                    f.GPS_HOME_MODE = 0;
                    if (rcOptions[BOXGPSHOLD] && abs(rcCommand[ROLL]) < cfg.ap_mode && abs(rcCommand[PITCH]) < cfg.ap_mode) {
                        if (!f.GPS_HOLD_MODE) {
                            f.GPS_HOLD_MODE = 1;
                            GPSNavReset = 0;
                            GPS_hold[LAT] = GPS_coord[LAT];
                            GPS_hold[LON] = GPS_coord[LON];
                            GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
                            nav_mode = NAV_MODE_POSHOLD;
                        }
                    } else {
                        f.GPS_HOLD_MODE = 0;
                        // both boxes are unselected here, nav is reset if not already done
                        if (GPSNavReset == 0) {
                            GPSNavReset = 1;
                            GPS_reset_nav();
                        }
                    }
                }
            } else {
                f.GPS_HOME_MODE = 0;
                f.GPS_HOLD_MODE = 0;
                nav_mode = NAV_MODE_NONE;
            }
        }

        if (rcOptions[BOXPASSTHRU]) {
            f.PASSTHRU_MODE = 1;
        } else {
            f.PASSTHRU_MODE = 0;
        }

        if (mcfg.mixerConfiguration == MULTITYPE_FLYING_WING || mcfg.mixerConfiguration == MULTITYPE_AIRPLANE) {
            f.HEADFREE_MODE = 0;
        }
    } else {                    // not in rc loop
        static int taskOrder = 0;    // never call all function in the same loop, to avoid high delay spikes
        if (taskOrder > 3)
            taskOrder -= 4;
        switch (taskOrder) {
        case 0:
            taskOrder++;
#ifdef MAG
            if (sensors(SENSOR_MAG) && Mag_getADC())
                break;
#endif
        case 1:
            taskOrder++;
#ifdef BARO
            if (sensors(SENSOR_BARO) && Baro_update())
                break;
#endif
        case 2:
            taskOrder++;
#ifdef BARO
            if (sensors(SENSOR_BARO) && getEstimatedAltitude())
            break;
#endif
        case 3:
            taskOrder++;
#ifdef SONAR
            if (sensors(SENSOR_SONAR)) {
                Sonar_update();
                debug[2] = sonarAlt;
            }
#endif
            if (feature(FEATURE_VARIO) && f.VARIO_MODE)
                mwVario();
            break;
        }
    }

    currentTime = micros();
    if (mcfg.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + mcfg.looptime;

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
                static uint8_t isAltHoldChanged = 0;
                static int16_t AltHoldCorr = 0;
                if (cfg.alt_hold_fast_change) {
                    // rapid alt changes
                    if (abs(rcCommand[THROTTLE] - initialThrottleHold) > cfg.alt_hold_throttle_neutral) {
                        errorAltitudeI = 0;
                        isAltHoldChanged = 1;
                        rcCommand[THROTTLE] += (rcCommand[THROTTLE] > initialThrottleHold) ? -cfg.alt_hold_throttle_neutral : cfg.alt_hold_throttle_neutral;
                    } else {
                        if (isAltHoldChanged) {
                            AltHold = EstAlt;
                            isAltHoldChanged = 0;
                        }
                        rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
                    }
                } else {
                    // slow alt changes for apfags
                    if (abs(rcCommand[THROTTLE] - initialThrottleHold) > cfg.alt_hold_throttle_neutral) {
                        // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
                        AltHoldCorr += rcCommand[THROTTLE] - initialThrottleHold;
                        if (abs(AltHoldCorr) > 500) {
                            AltHold += AltHoldCorr / 500;
                            AltHoldCorr %= 500;
                        }
                        errorAltitudeI = 0;
                        isAltHoldChanged = 1;
                    } else if (isAltHoldChanged) {
                        AltHold = EstAlt;
                        isAltHoldChanged = 0;
                    }
                    rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
                }
            }
        }
#endif

        if (sensors(SENSOR_GPS)) {
            if ((f.GPS_HOME_MODE || f.GPS_HOLD_MODE) && f.GPS_FIX_HOME) {
                float sin_yaw_y = sinf(heading * 0.0174532925f);
                float cos_yaw_x = cosf(heading * 0.0174532925f);
                if (cfg.nav_slew_rate) {
                    nav_rated[LON] += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -cfg.nav_slew_rate, cfg.nav_slew_rate); // TODO check this on uint8
                    nav_rated[LAT] += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -cfg.nav_slew_rate, cfg.nav_slew_rate);
                    GPS_angle[ROLL] = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
                    GPS_angle[PITCH] = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
                } else {
                    GPS_angle[ROLL] = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
                    GPS_angle[PITCH] = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
                }
            }
        }

        // **** PITCH & ROLL & YAW PID ****
        prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])); // range [0;500]
        for (axis = 0; axis < 3; axis++) {
            if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2) { // MODE relying on ACC
                // 50 degrees max inclination
                errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -500, +500) - angle[axis] + cfg.angleTrim[axis];
                PTermACC = (int32_t)errorAngle * cfg.P8[PIDLEVEL] / 100; // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
                PTermACC = constrain(PTermACC, -cfg.D8[PIDLEVEL] * 5, +cfg.D8[PIDLEVEL] * 5);

                errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
                ITermACC = ((int32_t)errorAngleI[axis] * cfg.I8[PIDLEVEL]) >> 12;
            }
            if (!f.ANGLE_MODE || f.HORIZON_MODE || axis == 2) { // MODE relying on GYRO or YAW axis
                error = (int32_t)rcCommand[axis] * 10 * 8 / cfg.P8[axis];
                error -= gyroData[axis];

                PTermGYRO = rcCommand[axis];

                errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
                if (abs(gyroData[axis]) > 640) 
                    errorGyroI[axis] = 0;
                ITermGYRO = (errorGyroI[axis] / 125 * cfg.I8[axis]) >> 6;
            }
            if (f.HORIZON_MODE && axis < 2) {
                PTerm = ((int32_t)PTermACC * (500 - prop) + (int32_t)PTermGYRO * prop) / 500;
                ITerm = ((int32_t)ITermACC * (500 - prop) + (int32_t)ITermGYRO * prop) / 500;
            } else {
                if (f.ANGLE_MODE && axis < 2) {
                    PTerm = PTermACC;
                    ITerm = ITermACC;
                } else {
                    PTerm = PTermGYRO;
                    ITerm = ITermGYRO;
                }
            }

            PTerm -= (int32_t)gyroData[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation
            delta = gyroData[axis] - lastGyro[axis]; // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
            lastGyro[axis] = gyroData[axis];
            deltaSum = delta1[axis] + delta2[axis] + delta;
            delta2[axis] = delta1[axis];
            delta1[axis] = delta;
            DTerm = ((int32_t)deltaSum * dynD8[axis]) >> 5; // 32 bits is needed for calculation
            axisPID[axis] =  PTerm + ITerm - DTerm;
        }

        mixTable();
        writeServos();
        writeMotors();
    }
}
