#pragma once

/* for VBAT monitoring frequency */
#define VBATFREQ 6        // to read battery voltage - nth number of loop iterations
#define BARO_TAB_SIZE_MAX   48

#define  VERSION  220

#define LAT  0
#define LON  1

// Serial GPS only variables
// navigation mode
typedef enum NavigationMode
{
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
    NAV_MODE_WP
} NavigationMode;

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
    MULTITYPE_OCTOX8 = 11,          // Java GUI is same for the next 3 configs
    MULTITYPE_OCTOFLATP = 12,       // MultiWinGui shows this differently
    MULTITYPE_OCTOFLATX = 13,       // MultiWinGui shows this differently
    MULTITYPE_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG = 16,
    MULTITYPE_VTAIL4 = 17,
    MULTITYPE_CUSTOM = 18,          // no current GUI displays this
    MULTITYPE_LAST = 19
} MultiType;

typedef enum GimbalFlags {
    GIMBAL_NORMAL = 1 << 0,
    GIMBAL_TILTONLY = 1 << 1,
    GIMBAL_DISABLEAUX34 = 1 << 2,
    GIMBAL_FORWARDAUX = 1 << 3,
    GIMBAL_MIXTILT = 1 << 4,
} GimbalFlags;

/*********** RC alias *****************/
enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PIDITEMS
};

enum {
    BOXARM = 0,
    BOXANGLE,
    BOXHORIZON,
    BOXBARO,
    BOXVARIO,
    BOXMAG,
    BOXHEADFREE,
    BOXHEADADJ,
    BOXCAMSTAB,
    BOXCAMTRIG,
    BOXGPSHOME,
    BOXGPSHOLD,
    BOXPASSTHRU,
    BOXBEEPERON,
    BOXLEDMAX,
    BOXLEDLOW,
    BOXLLIGHTS,
    BOXCALIB,
    BOXGOV,
    BOXOSD,
    CHECKBOXITEMS
};

#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))

#define SERVO_NORMAL    (1)
#define SERVO_REVERSE   (-1)

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

// Custom mixer configuration
typedef struct mixer_t {
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

typedef struct servoParam_t {
    int8_t direction;                   // servo direction
    uint16_t middle;                    // servo middle
    uint16_t min;                       // servo min
    uint16_t max;                       // servo max
} servoParam_t;

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

typedef struct config_t {
    uint8_t P8[PIDITEMS];
    uint8_t I8[PIDITEMS];
    uint8_t D8[PIDITEMS];

    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;

    uint8_t rollPitchRate;
    uint8_t yawRate;

    uint8_t dynThrPID;
    int16_t mag_declination;                // Get your magnetic decliniation from here : http://magnetic-declination.com/
    int16_t angleTrim[2];                   // accelerometer trim

    // sensor-related stuff
    uint8_t acc_lpf_factor;                 // Set the Low Pass Filter factor for ACC. Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter
    uint8_t accz_deadband;                  // ??
    uint8_t baro_tab_size;                  // size of baro filter array
    float baro_noise_lpf;                   // additional LPF to reduce baro noise
    float baro_cf;                          // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)

    uint16_t activate[CHECKBOXITEMS];       // activate switches

    // Radio/ESC-related configuration
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yawdeadband;                    // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t alt_hold_throttle_neutral;      // defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
    uint8_t alt_hold_fast_change;           // when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_throttle_neutral; when enabled, altitude changes slowly proportional to stick movement

    // Failsafe related configuration
    uint8_t failsafe_delay;                 // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t failsafe_off_delay;             // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
    uint16_t failsafe_throttle;             // Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
    uint16_t failsafe_detect_threshold;     // Update controls channel only if pulse is above failsafe_detect_threshold. below this trigger failsafe.

    // mixer-related configuration
    int8_t yaw_direction;
    uint16_t tri_yaw_middle;                // tail servo center pos. - use this for initial trim
    uint16_t tri_yaw_min;                   // tail servo min
    uint16_t tri_yaw_max;                   // tail servo max

    // flying wing related configuration
    uint16_t wing_left_min;                 // min/mid/max servo travel
    uint16_t wing_left_mid;
    uint16_t wing_left_max;
    uint16_t wing_right_min;
    uint16_t wing_right_mid;
    uint16_t wing_right_max;

    int8_t pitch_direction_l;               // left servo - pitch orientation
    int8_t pitch_direction_r;               // right servo - pitch orientation (opposite sign to pitch_direction_l if servos are mounted mirrored)
    int8_t roll_direction_l;                // left servo - roll orientation
    int8_t roll_direction_r;                // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)

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

    // gps-related stuff
    uint16_t gps_wp_radius;                 // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
    uint8_t gps_lpf;                        // Low pass filter cut frequency for derivative calculation (default 20Hz)
    uint8_t nav_slew_rate;                  // Adds a rate control to nav output, will smoothen out nav angle spikes
    uint8_t nav_controls_heading;           // copter faces toward the navigation point, maghold must be enabled for it
    uint16_t nav_speed_min;                 // cm/sec
    uint16_t nav_speed_max;                 // cm/sec
    uint16_t ap_mode;                       // Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks, creating a deadspan for GPS
} config_t;

// System-wide 
typedef struct master_t {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE

    uint8_t mixerConfiguration;
    uint32_t enabledFeatures;
    uint16_t looptime;                      // imu loop time in us
    motorMixer_t customMixer[MAX_MOTORS];   // custom mixtable

    // motor/esc/servo related stuff
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t motor_pwm_rate;                // The update rate of motor outputs (50-498Hz)
    uint16_t servo_pwm_rate;                // The update rate of servo outputs (50-498Hz)

    // global sensor-related stuff
    int8_t align[3][3];                     // acc, gyro, mag alignment (ex: with sensor output of X, Y, Z, align of 1 -3 2 would return X, -Z, Y)
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    uint16_t gyro_lpf;                      // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint16_t gyro_cmpf_factor;              // Set the Gyro Weight for Gyro/Acc complementary filter. Increasing this value would reduce and delay Acc influence on the output of the filter.
    uint16_t gyro_cmpfm_factor;             // Set the Gyro Weight for Gyro/Magnetometer complementary filter. Increasing this value would reduce and delay Magnetometer influence on the output of the filter
    uint32_t gyro_smoothing_factor;         // How much to smoothen with per axis (32bit value with Roll, Pitch, Yaw in bits 24, 16, 8 respectively
    uint8_t moron_threshold;                // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t mpu6050_scale;                  // es/non-es variance between MPU6050 sensors, half my boards are mpu6000ES, need this to be dynamic. automatically set by mpu6050 driver.
    int16_t accZero[3];
    int16_t magZero[3];

    // Battery/ADC stuff
    uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
    uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V)
    uint8_t power_adc_channel;              // which channel is used for current sensor. Right now, only 2 places are supported: RC_CH2 (unused when in CPPM mode, = 1), RC_CH8 (last channel in PWM mode, = 9)

    // Radio/ESC-related configuration
    uint8_t rcmap[8];                       // mapping of radio channels to internal RPYTA+ order
    uint8_t spektrum_hires;                 // spektrum high-resolution y/n (1024/2048bit)
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end
    uint8_t retarded_arm;                   // allow disarsm/arm on throttle down + roll left/right

    // gps-related stuff
    uint8_t gps_type;                       // Type of GPS hardware. 0: NMEA 1: UBX 2+ ??
    uint32_t gps_baudrate;                  // GPS baudrate

    // serial(uart1) baudrate
    uint32_t serial_baudrate;

    config_t profile[3];                    // 3 separate profiles
    uint8_t current_profile;                // currently loaded profile

    uint8_t magic_ef;                       // magic number, should be 0xEF
    uint8_t chk;                            // XOR checksum
} master_t;

typedef struct flags_t {
    uint8_t OK_TO_ARM;
    uint8_t ARMED;
    uint8_t ACC_CALIBRATED;
    uint8_t ANGLE_MODE;
    uint8_t HORIZON_MODE;
    uint8_t MAG_MODE;
    uint8_t BARO_MODE;
    uint8_t GPS_HOME_MODE;
    uint8_t GPS_HOLD_MODE;
    uint8_t HEADFREE_MODE;
    uint8_t PASSTHRU_MODE;
    uint8_t GPS_FIX;
    uint8_t GPS_FIX_HOME;
    uint8_t SMALL_ANGLES_25;
    uint8_t CALIBRATE_MAG;
    uint8_t VARIO_MODE;
} flags_t;

extern int16_t gyroZero[3];
extern int16_t gyroData[3];
extern int16_t angle[2];
extern int16_t axisPID[3];
extern int16_t rcCommand[4];
extern uint8_t rcOptions[CHECKBOXITEMS];
extern int16_t failsafeCnt;

extern int16_t debug[4];
extern int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
extern uint16_t acc_1G;
extern uint32_t currentTime;
extern uint32_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingB;
extern uint16_t calibratingG;
extern int16_t heading;
extern int16_t annex650_overrun_count;
extern int32_t baroPressure;
extern int32_t baroTemperature;
extern int32_t baroPressureSum;
extern int32_t BaroAlt;
extern int16_t sonarAlt;
extern int32_t EstAlt;
extern int32_t AltHold;
extern int16_t errorAltitudeI;
extern int16_t BaroPID;
extern int16_t vario;
extern int16_t headFreeModeHold;
extern int16_t zVelocity;
extern int16_t heading, magHold;
extern int16_t motor[MAX_MOTORS];
extern int16_t servo[8];
extern int16_t rcData[8];
extern uint16_t rssi;                  // range: [0;1023]
extern uint8_t vbat;
extern int16_t telemTemperature1;      // gyro sensor temperature
extern int16_t lookupPitchRollRC[6];   // lookup table for expo & RC rate PITCH+ROLL
extern int16_t lookupThrottleRC[11];   // lookup table for expo & mid THROTTLE
extern uint8_t toggleBeep;

// GPS stuff
extern int32_t  GPS_coord[2];
extern int32_t  GPS_home[2];
extern int32_t  GPS_hold[2];
extern uint8_t  GPS_numSat;
extern uint16_t GPS_distanceToHome;                          // distance to home or hold point in meters
extern int16_t  GPS_directionToHome;                         // direction to home or hol point in degrees
extern uint16_t GPS_altitude,GPS_speed;                      // altitude in 0.1m and speed in 0.1m/s
extern uint8_t  GPS_update;                                  // it's a binary toogle to distinct a GPS position update
extern int16_t  GPS_angle[2];                                // it's the angles that must be applied for GPS correction
extern uint16_t GPS_ground_course;                           // degrees*10
extern uint8_t  GPS_Present;                                 // Checksum from Gps serial
extern uint8_t  GPS_Enable;
extern int16_t  nav[2];
extern int8_t   nav_mode;                                    // Navigation mode
extern int16_t  nav_rated[2];                                // Adding a rate controller to the navigation to make it smoother
extern uint8_t  GPS_numCh;                                   // Number of channels
extern uint8_t  GPS_svinfo_chn[16];                          // Channel number
extern uint8_t  GPS_svinfo_svid[16];                         // Satellite ID
extern uint8_t  GPS_svinfo_quality[16];                      // Bitfield Qualtity
extern uint8_t  GPS_svinfo_cno[16];                          // Carrier to Noise Ratio (Signal Strength)

extern master_t mcfg;
extern config_t cfg;
extern flags_t f;
extern sensor_t acc;
extern sensor_t gyro;
extern baro_t baro;

// main
void loop(void);

// IMU
void imuInit(void);
void annexCode(void);
void computeIMU(void);
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
int getEstimatedAltitude(void);

// Sensors
void sensorsAutodetect(void);
void batteryInit(void);
uint16_t batteryAdcToVoltage(uint16_t src);
void ACC_getADC(void);
int Baro_update(void);
void Gyro_getADC(void);
void Mag_init(void);
int Mag_getADC(void);
void Sonar_init(void);
void Sonar_update(void);

// Output
void mixerInit(void);
void mixerLoadMix(int index);
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
void writeEEPROM(uint8_t b, uint8_t updateProfile);
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

// buzzer
void buzzer(uint8_t warn_vbat);

// cli
void cliProcess(void);

// gps
void gpsInit(uint32_t baudrate);
void GPS_reset_home_position(void);
void GPS_reset_nav(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
int32_t wrap_18000(int32_t error);

// telemetry
void initTelemetry(bool State);
void sendTelemetry(void);
