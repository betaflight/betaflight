#pragma once

#define QUADX

#define MINCHECK 1100
#define MAXCHECK 1900

#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1

/* this is the value for the ESCs when they are not armed
   in some cases, this value must be lowered down to 900 for some specific ESCs */
#define MINCOMMAND 1000

/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
   This is the minimum value that allow motors to run at a idle speed  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
//#define MINTHROTTLE 1220
#define MINTHROTTLE 1150 
/* this is the maximum value for the ESCs at full power this value can be increased up to 2000 */
#define MAXTHROTTLE 1850
/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500

/* This option should be uncommented if ACC Z is accurate enough when motors are running*/
/* should now be ok with BMA020 and BMA180 ACC */
#define TRUSTED_ACCZ

/* Failsave settings - added by MIS
   Failsafe check pulse on THROTTLE channel. If the pulse is OFF (on only THROTTLE or on all channels) the failsafe procedure is initiated.
   After FAILSAVE_DELAY time of pulse absence, the level mode is on (if ACC or nunchuk is avaliable), PITCH, ROLL and YAW is centered
   and THROTTLE is set to FAILSAVE_THR0TTLE value. You must set this value to descending about 1m/s or so for best results. 
   This value is depended from your configuration, AUW and some other params. 
   Next, afrer FAILSAVE_OFF_DELAY the copter is disarmed, and motors is stopped.
   If RC pulse coming back before reached FAILSAVE_OFF_DELAY time, after the small quard time the RC control is returned to normal.
   If you use serial sum PPM, the sum converter must completly turn off the PPM SUM pusles for this FailSafe functionality.*/
// #define FAILSAFE                                  // Alex: comment this line if you want to deactivate the failsafe function
#define FAILSAVE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAVE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAVE_THR0TTLE  (MINTHROTTLE + 200)    // Throttle level used for landing - may be relative to MINTHROTTLE - as in this case

/* you can change the tricopter servo travel here */
#define TRI_YAW_CONSTRAINT_MIN 1020
#define TRI_YAW_CONSTRAINT_MAX 2000
#define TRI_YAW_MIDDLE 1500 // tail servo center pos. - use this for initial trim; later trim midpoint via LCD

/* Flying Wing: you can change change servo orientation and servo min/max values here */
/* valid for all flight modes, even passThrough mode */
/* need to setup servo directions here; no need to swap servos amongst channels at rx */ 
#define PITCH_DIRECTION_L 1 // left servo - pitch orientation
#define PITCH_DIRECTION_R -1  // right servo - pitch orientation (opposite sign to PITCH_DIRECTION_L, if servos are mounted in mirrored orientation)
#define ROLL_DIRECTION_L 1 // left servo - roll orientation
#define ROLL_DIRECTION_R 1  // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)
#define WING_LEFT_MID  1500 // left servo center pos. - use this for initial trim; later trim midpoint via LCD
#define WING_RIGHT_MID 1500 // right servo center pos. - use this for initial trim; later trim midpoint via LCD
#define WING_LEFT_MIN  1020 // limit servo travel range must be inside [1020;2000]
#define WING_LEFT_MAX  2000 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MIN 1020 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MAX 2000 // limit servo travel range must be inside [1020;2000]

/* for V BAT monitoring
   after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
   with R1=33k and R2=51k
   vbat = [0;1023]*16/VBATSCALE */
#define VBATFREQ 6        // to read battery voltage - keep equal to PSENSORFREQ (6) unless you know what you are doing
#define VBATSCALE     131 // change this value if readed Battery voltage is different than real voltage
#define VBATLEVEL1_3S 107 // 10,7V
#define VBATLEVEL2_3S 103 // 10,3V
#define VBATLEVEL3_3S 99  // 9.9V
#define NO_VBAT       16 // Avoid beeping without any battery


#define   VERSION  19

#if defined(TRI)
  #define MULTITYPE 1
#elif defined(QUADP)
  #define MULTITYPE 2
#elif defined(QUADX)
  #define MULTITYPE 3
#elif defined(BI)
  #define MULTITYPE 4
#elif defined(GIMBAL)
  #define MULTITYPE 5
#elif defined(Y6)
  #define MULTITYPE 6
#elif defined(HEX6)
  #define MULTITYPE 7
#elif defined(FLYING_WING)
  #define MULTITYPE 8
#elif defined(Y4)
  #define MULTITYPE 9
#elif defined(HEX6X)
  #define MULTITYPE 10
#elif defined(OCTOX8)
  #define MULTITYPE 11
#elif defined(OCTOFLATP)
  #define MULTITYPE 11      //the GUI is the same for all 8 motor configs
#elif defined(OCTOFLATX)
  #define MULTITYPE 11      //the GUI is the same for all 8 motor configs
#endif

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

#define ACC_ORIENTATION(X, Y, Z)  { accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  = Z; }
#define GYRO_ORIENTATION(X, Y, Z) { gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = Z; }
#define MAG_ORIENTATION(X, Y, Z)  { magADC[ROLL]  = X;  magADC[PITCH]  =  Y; magADC[YAW]  = Z; }

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

extern int16_t accZero[3];
extern int16_t accTrim[2];
extern int16_t gyroZero[3];
extern int16_t magZero[3];
extern int16_t gyroData[3];
extern int16_t angle[2];
extern int16_t EstAlt;
extern int16_t axisPID[3];
extern int16_t rcCommand[4];

extern int16_t debug1, debug2, debug3, debug4;
extern uint8_t armed;
extern int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
extern uint16_t acc_1G;
extern uint32_t currentTime;
extern uint32_t previousTime;
extern uint16_t cycleTime;
extern uint8_t calibratedACC;
extern uint16_t calibratingA;
extern uint8_t calibratingM;
extern uint16_t calibratingG;
extern int16_t heading;
extern int16_t annex650_overrun_count;
extern int16_t i2c_errors_count;
extern int32_t pressure;
extern int16_t BaroAlt;
extern uint8_t headFreeMode;
extern int16_t headFreeModeHold;
extern int8_t smallAngle25;
extern int16_t zVelocity;
extern int16_t heading, magHold;
extern int16_t motor[8];
extern int16_t servo[8];
extern int16_t rcData[8];
extern uint8_t accMode;
extern uint8_t magMode;
extern uint8_t baroMode;
extern uint8_t P8[8], I8[8], D8[8];
extern uint8_t dynThrPID;
extern uint8_t activate1[CHECKBOXITEMS];
extern uint8_t activate2[CHECKBOXITEMS];
extern uint16_t intPowerMeterSum, intPowerTrigger1;
extern uint8_t rollPitchRate;
extern uint8_t yawRate;
extern uint8_t dynThrPID;
extern uint8_t rcRate8;
extern uint8_t rcExpo8;
extern int32_t GPS_latitude, GPS_longitude;
extern int32_t GPS_latitude_home, GPS_longitude_home;
extern uint8_t GPS_fix, GPS_fix_home;
extern uint8_t GPS_numSat;
extern uint16_t GPS_distanceToHome;
extern int16_t GPS_directionToHome;
extern uint8_t GPS_update;
extern uint8_t GPSModeHome;
extern uint8_t GPSModeHold;
extern uint8_t vbat;
extern uint8_t powerTrigger1;
extern int16_t lookupRX[7];     //  lookup table for expo & RC rate

// main
void loop(void);

// IMU
void imuInit(void);
void annexCode(void);
void computeIMU(void);
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);

// Sensors
void sensorsAutodetect(void);
void ACC_getADC(void);
void Baro_update(void);
void Gyro_getADC(void);
void Mag_init(void);
void Mag_getADC(void);

// Output
void writeServos(void);
void writeMotors(void);
void mixTable(void);

// Serial
void serialCom(void);

// Config
void readEEPROM(void);
void writeParams(void);
void checkFirstTime(void);
