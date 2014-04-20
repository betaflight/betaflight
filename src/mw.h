#pragma once

#include "runtime_config.h"
#include "flight_common.h"
#include "failsafe.h"

/* for VBAT monitoring frequency */
#define VBATFREQ 6        // to read battery voltage - nth number of loop iterations
#define BARO_TAB_SIZE_MAX   48

#define  VERSION  230

// Serial GPS only variables
// navigation mode
typedef enum NavigationMode
{
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
    NAV_MODE_WP
} NavigationMode;

#include "gimbal.h"
#include "flight_mixer.h"

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400
#define CALIBRATING_BARO_CYCLES             200

#include "serial_common.h"
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

// main
void setPIDController(int type);
void loop(void);

// IMU
void imuInit(void);
void annexCode(void);
void computeIMU(void);
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

// buzzer
void buzzer(bool warn_vbat);
void systemBeep(bool onoff);

// cli
void cliProcess(void);
