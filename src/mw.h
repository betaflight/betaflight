#pragma once

#include "runtime_config.h"
#include "flight_common.h"

/* for VBAT monitoring frequency */
#define VBATFREQ 6        // to read battery voltage - nth number of loop iterations
#define BARO_TAB_SIZE_MAX   48

#define  VERSION  230

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
    MULTITYPE_HEX6H = 18,
    MULTITYPE_PPM_TO_SERVO = 19,    // PPM -> servo relay 
    MULTITYPE_DUALCOPTER = 20,
    MULTITYPE_SINGLECOPTER = 21,
    MULTITYPE_CUSTOM = 22,          // no current GUI displays this
    MULTITYPE_LAST = 23
} MultiType;

typedef enum GimbalFlags {
    GIMBAL_NORMAL = 1 << 0,
    GIMBAL_MIXTILT = 1 << 1,
    GIMBAL_FORWARDAUX = 1 << 2,
} GimbalFlags;

#include "flight_mixer.h"

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400
#define CALIBRATING_BARO_CYCLES             200

#include "rx_common.h"
#include "config.h"
#include "config_storage.h"

extern int16_t axisPID[3];
extern int16_t rcCommand[4];

extern int16_t debug[4];
extern uint16_t acc_1G;
extern uint32_t accTimeSum;
extern int accSumCount;
extern uint32_t currentTime;
extern uint32_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingB;
extern uint16_t calibratingG;
extern int16_t heading;
extern int32_t baroPressure;
extern int32_t baroTemperature;
extern uint32_t baroPressureSum;
extern int32_t BaroAlt;
extern int32_t sonarAlt;
extern int32_t EstAlt;
extern int32_t AltHold;
extern int32_t errorAltitudeI;
extern int32_t BaroPID;
extern int32_t vario;
extern int16_t throttleAngleCorrection;
extern int16_t headFreeModeHold;
extern int16_t heading, magHold;
extern int16_t motor[MAX_MOTORS];
extern int16_t servo[MAX_SERVOS];
extern uint16_t rssi;                  // range: [0;1023]
extern uint8_t vbat;
extern int16_t telemTemperature1;      // gyro sensor temperature
extern uint8_t toggleBeep;

#define PITCH_LOOKUP_LENGTH 7
#define THROTTLE_LOOKUP_LENGTH 12
extern int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];   // lookup table for expo & RC rate PITCH+ROLL
extern int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE

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
extern int16_t  nav[2];
extern int8_t   nav_mode;                                    // Navigation mode
extern int16_t  nav_rated[2];                                // Adding a rate controller to the navigation to make it smoother
extern uint8_t  GPS_numCh;                                   // Number of channels
extern uint8_t  GPS_svinfo_chn[16];                          // Channel number
extern uint8_t  GPS_svinfo_svid[16];                         // Satellite ID
extern uint8_t  GPS_svinfo_quality[16];                      // Bitfield Qualtity
extern uint8_t  GPS_svinfo_cno[16];                          // Carrier to Noise Ratio (Signal Strength)

extern flags_t f;
extern sensor_t acc;
extern sensor_t gyro;
extern baro_t baro;

// main
void setPIDController(int type);
void loop(void);

// IMU
void imuInit(void);
void annexCode(void);
void computeIMU(void);
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
int getEstimatedAltitude(void);

// Sensors
void sensorsAutodetect(void);
void ACC_getADC(void);
int Baro_update(void);
void Gyro_getADC(void);
void Mag_init(void);
int Mag_getADC(void);
void Sonar_init(void);
void Sonar_update(void);

// Output
void mixerInit(void);
void mixerResetMotors(void);
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

// buzzer
void buzzer(bool warn_vbat);
void systemBeep(bool onoff);

// cli
void cliProcess(void);

// gps
void gpsInit(uint8_t baudrate);
void gpsThread(void);
void gpsSetPIDs(void);
int8_t gpsSetPassthrough(void);
void GPS_reset_home_position(void);
void GPS_reset_nav(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
int32_t wrap_18000(int32_t error);

