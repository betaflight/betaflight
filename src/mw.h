#pragma once

#include "rc_controls.h"
#include "runtime_config.h"
#include "flight_common.h"
#include "failsafe.h"

/* for VBAT monitoring frequency */
#define VBATFREQ 6        // to read battery voltage - nth number of loop iterations
#define BARO_TAB_SIZE_MAX   48

#define  VERSION  230

#include "gimbal.h"
#include "flight_mixer.h"

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

#include "serial_common.h"
#include "rc_controls.h"
#include "rx_common.h"
#include "gps_common.h"
#include "config.h"
#include "config_profile.h"
#include "config_master.h"

extern int16_t debug[4];
extern uint16_t acc_1G;
extern uint32_t accTimeSum;
extern int accSumCount;
extern uint32_t currentTime;
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
extern int16_t motor[MAX_MOTORS];
extern int16_t servo[MAX_SERVOS];
extern uint16_t rssi;                  // range: [0;1023]
extern uint8_t vbat;
extern int16_t telemTemperature1;      // gyro sensor temperature
extern uint8_t toggleBeep;

extern flags_t f;

// IMU
void annexCode(void);
void computeIMU(void);
int getEstimatedAltitude(void);

// Sensors
void ACC_getADC(void);
int Baro_update(void);
void Gyro_getADC(void);
void Mag_init(void);
int Mag_getADC(void);

// Output
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
