#pragma once

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

/* Flying Wing: you can change change servo orientation and servo min/max values here */
/* valid for all flight modes, even passThrough mode */
/* need to setup servo directions here; no need to swap servos amongst channels at rx */ 
#define PITCH_DIRECTION_L 1 // left servo - pitch orientation
#define PITCH_DIRECTION_R -1  // right servo - pitch orientation (opposite sign to PITCH_DIRECTION_L, if servos are mounted in mirrored orientation)
#define ROLL_DIRECTION_L 1 // left servo - roll orientation
#define ROLL_DIRECTION_R 1  // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)
#define WING_LEFT_MIN  1020 // limit servo travel range must be inside [1020;2000]
#define WING_LEFT_MAX  2000 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MIN 1020 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MAX 2000 // limit servo travel range must be inside [1020;2000]

/* experimental
   camera trigger function : activated via Rc Options in the GUI, servo output=A2 on promini */
#define CAM_SERVO_HIGH 2000  // the position of HIGH state servo
#define CAM_SERVO_LOW 1020   // the position of LOW state servo
#define CAM_TIME_HIGH 1000   // the duration of HIGH state servo expressed in ms
#define CAM_TIME_LOW 1000    // the duration of LOW state servo expressed in ms

/* for VBAT monitoring frequency */
#define VBATFREQ 6        // to read battery voltage - keep equal to PSENSORFREQ (6) unless you know what you are doing

// Moving Average Gyros by Magnetron1 (Michele Ardito) ########## beta
//#define MMGYRO                         // Active Moving Average Function for Gyros
//#define MMGYROVECTORLENGHT 10          // Lenght of Moving Average Vector
// Moving Average ServoGimbal Signal Output
//#define MMSERVOGIMBAL                  // Active Output Moving Average Function for Servos Gimbal
//#define MMSERVOGIMBALVECTORLENGHT 32   // Lenght of Moving Average Vector

#define  VERSION  20

// Syncronized with GUI. Only exception is mixer > 11, which is always returned as 11 during serialization.
typedef enum MultiType
{
    MULTITYPE_TRI = 1,
    MULTITYPE_QUADP = 2,
    MULTITYPE_QUADX = 3,
    MULTITYPE_BI = 4,
    MULTITYPE_GIMBAL = 5,
    MULTITYPE_Y6 = 6,
    MULTITYPE_HEX6 = 7,
    MULTITYPE_FLYING_WING = 8,
    MULTITYPE_Y4 = 9,
    MULTITYPE_HEX6X = 10,
    MULTITYPE_OCTOX8 = 11,
    MULTITYPE_OCTOFLATP = 12,       // the GUI is the same for all 8 motor configs
    MULTITYPE_OCTOFLATX = 13,       // the GUI is the same for all 8 motor configs
    MULTITYPE_AIRPLANE = 14,
    MULTITYPE_HELI_120_CCPM = 15,   // simple model
    MULTITYPE_HELI_90_DEG = 16,     // simple model
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_LAST = 18
} MultiType;

typedef enum GimbalFlags {
    GIMBAL_NORMAL = 1 << 0,
    GIMBAL_TILTONLY = 1 << 1,
    GIMBAL_DISABLEAUX34 = 1 << 2,
} GimbalFlags;

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

#define BOXACC       0
#define BOXBARO      1
#define BOXMAG       2
#define BOXCAMSTAB   3
#define BOXCAMTRIG   4
#define BOXARM       5
#define BOXGPSHOME   6
#define BOXGPSHOLD   7
#define BOXPASSTHRU  8
#define BOXHEADFREE  9
#define BOXBEEPERON  10

#define CHECKBOXITEMS 11
#define PIDITEMS 8

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

typedef struct config_t {
    uint8_t version;
    uint8_t mixerConfiguration;
    uint32_t enabledFeatures;

    uint8_t P8[8];
    uint8_t I8[8];
    uint8_t D8[8];

    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t rollPitchRate;
    uint8_t yawRate;

    uint8_t dynThrPID;
    int16_t accZero[3];
    int16_t magZero[3];
    int16_t accTrim[2];

    // sensor-related stuff
    uint8_t acc_lpf_factor;                 // Set the Low Pass Filter factor for ACC. Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter
    uint16_t gyro_lpf;                      // mpuX050 LPF setting
    uint32_t gyro_smoothing_factor;         // How much to smoothen with per axis (32bit value with Roll, Pitch, Yaw in bits 24, 16, 8 respectively

    uint8_t activate1[CHECKBOXITEMS];
    uint8_t activate2[CHECKBOXITEMS];
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V)

    // Radio/ESC-related configuration
    uint8_t rcmap[8];                       // mapping of radio channels to internal RPYTA+ order
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yawdeadband;                    // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t spektrum_hires;                 // spektrum high-resolution y/n (1024/2048bit)
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end

    // motor/esc/servo related stuff
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t motor_pwm_rate;                // The update rate of motor outputs (50-498Hz)
    uint16_t servo_pwm_rate;                // The update rate of servo outputs (50-498Hz)

    // mixer-related configuration
    int8_t yaw_direction;
    uint16_t wing_left_mid;                 // left servo center pos. - use this for initial trim
    uint16_t wing_right_mid;                // right servo center pos. - use this for initial trim
    uint16_t tri_yaw_middle;                // tail servo center pos. - use this for initial trim
    uint16_t tri_yaw_min;                   // tail servo min
    uint16_t tri_yaw_max;                   // tail servo max

    // gimbal-related configuration
    int8_t gimbal_pitch_gain;               // gimbal pitch servo gain (tied to angle) can be negative to invert movement
    int8_t gimbal_roll_gain;                // gimbal roll servo gain (tied to angle) can be negative to invert movement
    uint8_t gimbal_flags;                   // in servotilt mode, various things that affect stuff
    uint16_t gimbal_pitch_min;              // gimbal pitch servo min travel
    uint16_t gimbal_pitch_max;              // gimbal pitch servo max travel
    uint16_t gimbal_pitch_mid;              // gimbal pitch servo neutral value
    uint16_t gimbal_roll_min;               // gimbal roll servo min travel
    uint16_t gimbal_roll_max;               // gimbal roll servo max travel
    uint16_t gimbal_roll_mid;               // gimbal roll servo neutral value

    // gps baud-rate
    uint32_t gps_baudrate;    

   // serial(uart1) baudrate
    uint32_t serial_baudrate;
} config_t;

extern int16_t gyroZero[3];
extern int16_t gyroData[3];
extern int16_t angle[2];
extern int16_t axisPID[3];
extern int16_t rcCommand[4];
extern uint8_t rcOptions[CHECKBOXITEMS];

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
extern int32_t pressure;
extern int32_t BaroAlt;
extern int32_t EstAlt;
extern int32_t  AltHold;
extern int16_t  errorAltitudeI;
extern int16_t  BaroPID;
extern uint8_t headFreeMode;
extern int16_t headFreeModeHold;
extern uint8_t passThruMode;
extern int8_t smallAngle25;
extern int16_t zVelocity;
extern int16_t heading, magHold;
extern int16_t motor[8];
extern int16_t servo[8];
extern int16_t rcData[8];
extern uint8_t accMode;
extern uint8_t magMode;
extern uint8_t baroMode;
extern int32_t GPS_latitude, GPS_longitude;
extern int32_t GPS_latitude_home, GPS_longitude_home;
extern int32_t GPS_latitude_hold, GPS_longitude_hold;
extern uint8_t GPS_fix, GPS_fix_home;
extern uint8_t GPS_numSat;
extern uint16_t GPS_distanceToHome, GPS_distanceToHold;
extern int16_t GPS_directionToHome, GPS_directionToHold;
extern uint8_t GPS_update;
extern uint8_t GPSModeHome;
extern uint8_t GPSModeHold;
extern uint16_t GPS_altitude;
extern uint16_t GPS_speed;                      // altitude in 0.1m and speed in 0.1m/s - Added by Mis
extern uint8_t vbat;
extern int16_t lookupRX[7];     //  lookup table for expo & RC rate

extern config_t cfg;
extern sensor_t acc;
extern sensor_t gyro;

// main
void loop(void);

// IMU
void imuInit(void);
void annexCode(void);
void computeIMU(void);
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
void getEstimatedAltitude(void);

// Sensors
void sensorsAutodetect(void);
void batteryInit(void);
uint16_t batteryAdcToVoltage(uint16_t src);
void ACC_getADC(void);
void Baro_update(void);
void Gyro_getADC(void);
void Mag_init(void);
void Mag_getADC(void);

// Output
void mixerInit(void);
void writeServos(void);
void writeMotors(void);
void writeAllMotors(int16_t mc);
void mixTable(void);

// Serial
void serialInit(uint32_t baudrate);
void serialCom(void);

// Config
void parseRcChannels(const char *input);
void readEEPROM(void);
void writeParams(void);
void checkFirstTime(bool reset);
bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);
bool feature(uint32_t mask);
void featureSet(uint32_t mask);
void featureClear(uint32_t mask);
void featureClearAll(void);
uint32_t featureMask(void);

// spektrum
void spektrumInit(void);
bool spektrumFrameComplete(void);

// cli
void cliProcess(void);

// gps
void gpsInit(uint32_t baudrate);
