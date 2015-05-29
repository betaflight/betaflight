/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <string.h>

#include "platform.h"
#include "version.h"

#ifdef BLACKBOX

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/encoding.h"
#include "common/utils.h"

#include "drivers/gpio.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/compass.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "blackbox.h"
#include "blackbox_io.h"

#define BLACKBOX_I_INTERVAL 32
#define BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS 200

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

#define STATIC_ASSERT(condition, name ) \
    typedef char assert_failed_ ## name [(condition) ? 1 : -1 ]

// Some macros to make writing FLIGHT_LOG_FIELD_* constants shorter:

#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED

static const char blackboxHeader[] =
    "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
    "H Blackbox version:1\n"
    "H Data version:2\n"
    "H I interval:" STR(BLACKBOX_I_INTERVAL) "\n";

static const char* const blackboxMainHeaderNames[] = {
    "I name",
    "I signed",
    "I predictor",
    "I encoding",
    "P predictor",
    "P encoding"
};

#ifdef GPS
static const char* const blackboxGPSGHeaderNames[] = {
    "G name",
    "G signed",
    "G predictor",
    "G encoding"
};

static const char* const blackboxGPSHHeaderNames[] = {
    "H name",
    "H signed",
    "H predictor",
    "H encoding"
};
#endif

/* All field definition structs should look like this (but with longer arrs): */
typedef struct blackboxFieldDefinition_t {
    const char *name;
    // If the field name has a number to be included in square brackets [1] afterwards, set it here, or -1 for no brackets:
    int8_t fieldNameIndex;

    // Each member of this array will be the value to print for this field for the given header index
    uint8_t arr[1];
} blackboxFieldDefinition_t;

typedef struct blackboxMainFieldDefinition_t {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxMainFieldDefinition_t;

typedef struct blackboxGPSFieldDefinition_t {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxGPSFieldDefinition_t;

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static const blackboxMainFieldDefinition_t blackboxMainFields[] = {
    /* loopIteration doesn't appear in P frames since it always increments */
    {"loopIteration",-1, UNSIGNED, .Ipredict = PREDICT(0),     .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(INC),           .Pencode = FLIGHT_LOG_FIELD_ENCODING_NULL, CONDITION(ALWAYS)},
    /* Time advances pretty steadily so the P-frame prediction is a straight line */
    {"time",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(STRAIGHT_LINE), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    /* I terms get special packed encoding in P frames: */
    {"axisI",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisI",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisI",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisD",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_0)},
    {"axisD",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_1)},
    {"axisD",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_2)},
    /* rcCommands are encoded together as a group in P-frames: */
    {"rcCommand",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"rcCommand",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"rcCommand",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    /* Throttle is always in the range [minthrottle..maxthrottle]: */
    {"rcCommand",   3, UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},

    {"vbatLatest",    -1, UNSIGNED, .Ipredict = PREDICT(VBATREF),  .Iencode = ENCODING(NEG_14BIT),   .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_VBAT},
    {"amperageLatest",-1, UNSIGNED, .Ipredict = PREDICT(0),        .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC},

#ifdef MAG
    {"magADC",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
    {"magADC",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
    {"magADC",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
#endif
#ifdef BARO
    {"BaroAlt",    -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_BARO},
#endif
#ifdef SONAR
    {"sonarRaw",   -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_SONAR},
#endif

    /* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
    {"gyroADC",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroADC",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroADC",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth",  0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth",  1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth",  2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    /* Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones): */
    {"motor",      0, UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_1)},
    /* Subsequent motors base their I-frame values on the first one, P-frame values on the average of last two frames: */
    {"motor",      1, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_2)},
    {"motor",      2, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_3)},
    {"motor",      3, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_4)},
    {"motor",      4, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_5)},
    {"motor",      5, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_6)},
    {"motor",      6, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_7)},
    {"motor",      7, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_8)},

    /* Tricopter tail servo */
    {"servo",      5, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(TRICOPTER)}
};

#ifdef GPS
// GPS position/vel frame
static const blackboxGPSFieldDefinition_t blackboxGpsGFields[] = {
    {"time",              -1, UNSIGNED, PREDICT(LAST_MAIN_FRAME_TIME), ENCODING(UNSIGNED_VB), CONDITION(NOT_LOGGING_EVERY_FRAME)},
    {"GPS_numSat",        -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_coord",          0, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_coord",          1, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_altitude",      -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_speed",         -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_ground_course", -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)}
};

// GPS home frame
static const blackboxGPSFieldDefinition_t blackboxGpsHFields[] = {
    {"GPS_home",           0, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_home",           1, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB),   CONDITION(ALWAYS)}
};
#endif

typedef enum BlackboxState {
    BLACKBOX_STATE_DISABLED = 0,
    BLACKBOX_STATE_STOPPED,
    BLACKBOX_STATE_SEND_HEADER,
    BLACKBOX_STATE_SEND_FIELDINFO,
    BLACKBOX_STATE_SEND_GPS_H_HEADERS,
    BLACKBOX_STATE_SEND_GPS_G_HEADERS,
    BLACKBOX_STATE_SEND_SYSINFO,
    BLACKBOX_STATE_RUNNING,
    BLACKBOX_STATE_SHUTTING_DOWN
} BlackboxState;

typedef struct gpsState_t {
    int32_t GPS_home[2], GPS_coord[2];
    uint8_t GPS_numSat;
} gpsState_t;

//From mixer.c:
extern uint8_t motorCount;

//From mw.c:
extern uint32_t currentTime;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;

static uint32_t blackboxLastArmingBeep = 0;

static struct {
    uint32_t headerIndex;

    /* Since these fields are used during different blackbox states (never simultaneously) we can
     * overlap them to save on RAM
     */
    union {
        int fieldIndex;
        int serialBudget;
        uint32_t startTime;
    } u;
} xmitState;

// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
static uint32_t blackboxConditionCache;

STATIC_ASSERT((sizeof(blackboxConditionCache) * 8) >= FLIGHT_LOG_FIELD_CONDITION_NEVER, too_many_flight_log_conditions);

static uint32_t blackboxIteration;
static uint32_t blackboxPFrameIndex, blackboxIFrameIndex;

/*
 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs
 * to encode:
 */
static uint16_t vbatReference;

static gpsState_t gpsHistory;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxValues_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxValues_t* blackboxHistory[3];

static bool testBlackboxConditionUncached(FlightLogFieldCondition condition)
{
    switch (condition) {
        case FLIGHT_LOG_FIELD_CONDITION_ALWAYS:
            return true;

        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8:
            return motorCount >= condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 + 1;
        
        case FLIGHT_LOG_FIELD_CONDITION_TRICOPTER:
            return masterConfig.mixerMode == MIXER_TRI;

        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2:
            if (IS_PID_CONTROLLER_FP_BASED(currentProfile->pidProfile.pidController)) {
                return currentProfile->pidProfile.D_f[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0] != 0;
            } else {
                return currentProfile->pidProfile.D8[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0] != 0;
            }

        case FLIGHT_LOG_FIELD_CONDITION_MAG:
#ifdef MAG
            return sensors(SENSOR_MAG);
#else
            return false;
#endif

        case FLIGHT_LOG_FIELD_CONDITION_BARO:
#ifdef BARO
            return sensors(SENSOR_BARO);
#else
            return false;
#endif

        case FLIGHT_LOG_FIELD_CONDITION_VBAT:
            return feature(FEATURE_VBAT);

        case FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC:
            return feature(FEATURE_CURRENT_METER) && masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC;

        case FLIGHT_LOG_FIELD_CONDITION_SONAR:
#ifdef SONAR
            return feature(FEATURE_SONAR);
#else
            return false;
#endif

        case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
            return masterConfig.blackbox_rate_num < masterConfig.blackbox_rate_denom;

        case FLIGHT_LOG_FIELD_CONDITION_NEVER:
            return false;
        default:
            return false;
    }
}

static void blackboxBuildConditionCache()
{
    FlightLogFieldCondition cond;

    blackboxConditionCache = 0;

    for (cond = FLIGHT_LOG_FIELD_CONDITION_FIRST; cond <= FLIGHT_LOG_FIELD_CONDITION_LAST; cond++) {
        if (testBlackboxConditionUncached(cond)) {
            blackboxConditionCache |= 1 << cond;
        }
    }
}

static bool testBlackboxCondition(FlightLogFieldCondition condition)
{
    return (blackboxConditionCache & (1 << condition)) != 0;
}

static void blackboxSetState(BlackboxState newState)
{
    //Perform initial setup required for the new state
    switch (newState) {
        case BLACKBOX_STATE_SEND_HEADER:
            xmitState.headerIndex = 0;
            xmitState.u.startTime = millis();
        break;
        case BLACKBOX_STATE_SEND_FIELDINFO:
        case BLACKBOX_STATE_SEND_GPS_G_HEADERS:
        case BLACKBOX_STATE_SEND_GPS_H_HEADERS:
            xmitState.headerIndex = 0;
            xmitState.u.fieldIndex = -1;
        break;
        case BLACKBOX_STATE_SEND_SYSINFO:
            xmitState.headerIndex = 0;
        break;
        case BLACKBOX_STATE_RUNNING:
            blackboxIteration = 0;
            blackboxPFrameIndex = 0;
            blackboxIFrameIndex = 0;
        break;
        case BLACKBOX_STATE_SHUTTING_DOWN:
            xmitState.u.startTime = millis();
            blackboxDeviceFlush();
        break;
        default:
            ;
    }
    blackboxState = newState;
}

static void writeIntraframe(void)
{
    blackboxValues_t *blackboxCurrent = blackboxHistory[0];
    int x;

    blackboxWrite('I');

    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(blackboxCurrent->time);

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->axisPID_P[x]);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->axisPID_I[x]);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
            blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x]);
        }
    }

    for (x = 0; x < 3; x++) {
        blackboxWriteSignedVB(blackboxCurrent->rcCommand[x]);
    }

    blackboxWriteUnsignedVB(blackboxCurrent->rcCommand[3] - masterConfig.escAndServoConfig.minthrottle); //Throttle lies in range [minthrottle..maxthrottle]

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
        /*
         * Our voltage is expected to decrease over the course of the flight, so store our difference from
         * the reference:
         *
         * Write 14 bits even if the number is negative (which would otherwise result in 32 bits)
         */
        blackboxWriteUnsignedVB((vbatReference - blackboxCurrent->vbatLatest) & 0x3FFF);
    }

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC)) {
        // 12bit value directly from ADC
        blackboxWriteUnsignedVB(blackboxCurrent->amperageLatest);
    }

#ifdef MAG
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
            for (x = 0; x < XYZ_AXIS_COUNT; x++) {
                blackboxWriteSignedVB(blackboxCurrent->magADC[x]);
            }
        }
#endif

#ifdef BARO
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO)) {
            blackboxWriteSignedVB(blackboxCurrent->BaroAlt);
        }
#endif

#ifdef SONAR
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SONAR)) {
            blackboxWriteSignedVB(blackboxCurrent->sonarRaw);
        }
#endif

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->gyroADC[x]);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->accSmooth[x]);
    }

    //Motors can be below minthrottle when disarmed, but that doesn't happen much
    blackboxWriteUnsignedVB(blackboxCurrent->motor[0] - masterConfig.escAndServoConfig.minthrottle);

    //Motors tend to be similar to each other
    for (x = 1; x < motorCount; x++) {
        blackboxWriteSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);
    }

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
        blackboxWriteSignedVB(blackboxHistory[0]->servo[5] - 1500);
    }

    //Rotate our history buffers:

    //The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    //And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    //And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

static void writeInterframe(void)
{
    int x;
    int32_t deltas[7];

    blackboxValues_t *blackboxCurrent = blackboxHistory[0];
    blackboxValues_t *blackboxLast = blackboxHistory[1];

    blackboxWrite('P');

    //No need to store iteration count since its delta is always 1

    /*
     * Since the difference between the difference between successive times will be nearly zero (due to consistent
     * looptime spacing), use second-order differences.
     */
    blackboxWriteSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->axisPID_P[x] - blackboxLast->axisPID_P[x]);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        deltas[x] = blackboxCurrent->axisPID_I[x] - blackboxLast->axisPID_I[x];
    }

    /* 
     * The PID I field changes very slowly, most of the time +-2, so use an encoding
     * that can pack all three fields into one byte in that situation.
     */
    blackboxWriteTag2_3S32(deltas);
    
    /*
     * The PID D term is frequently set to zero for yaw, which makes the result from the calculation
     * always zero. So don't bother recording D results when PID D terms are zero.
     */
    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
            blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x] - blackboxLast->axisPID_D[x]);
        }
    }

    /*
     * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
     * can pack multiple values per byte:
     */
    for (x = 0; x < 4; x++) {
        deltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];
    }

    blackboxWriteTag8_4S16(deltas);

    //Check for sensors that are updated periodically (so deltas are normally zero)
    int optionalFieldCount = 0;

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
        deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->vbatLatest - blackboxLast->vbatLatest;
    }

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC)) {
        deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->amperageLatest - blackboxLast->amperageLatest;
    }

#ifdef MAG
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
        for (x = 0; x < XYZ_AXIS_COUNT; x++) {
            deltas[optionalFieldCount++] = blackboxCurrent->magADC[x] - blackboxLast->magADC[x];
        }
    }
#endif

#ifdef BARO
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO)) {
        deltas[optionalFieldCount++] = blackboxCurrent->BaroAlt - blackboxLast->BaroAlt;
    }
#endif

#ifdef SONAR
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SONAR)) {
        deltas[optionalFieldCount++] = blackboxCurrent->sonarRaw - blackboxLast->sonarRaw;
    }
#endif

    blackboxWriteTag8_8SVB(deltas, optionalFieldCount);

    //Since gyros, accs and motors are noisy, base the prediction on the average of the history:
    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxHistory[0]->gyroADC[x] - (blackboxHistory[1]->gyroADC[x] + blackboxHistory[2]->gyroADC[x]) / 2);
    }

    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxHistory[0]->accSmooth[x] - (blackboxHistory[1]->accSmooth[x] + blackboxHistory[2]->accSmooth[x]) / 2);
    }

    for (x = 0; x < motorCount; x++) {
        blackboxWriteSignedVB(blackboxHistory[0]->motor[x] - (blackboxHistory[1]->motor[x] + blackboxHistory[2]->motor[x]) / 2);
    }

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
        blackboxWriteSignedVB(blackboxCurrent->servo[5] - blackboxLast->servo[5]);
    }

    //Rotate our history buffers
    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

static int gcd(int num, int denom)
{
    if (denom == 0) {
        return num;
    }

    return gcd(denom, num % denom);
}

static void validateBlackboxConfig()
{
    int div;

    if (masterConfig.blackbox_rate_num == 0 || masterConfig.blackbox_rate_denom == 0
            || masterConfig.blackbox_rate_num >= masterConfig.blackbox_rate_denom) {
        masterConfig.blackbox_rate_num = 1;
        masterConfig.blackbox_rate_denom = 1;
    } else {
        /* Reduce the fraction the user entered as much as possible (makes the recorded/skipped frame pattern repeat
         * itself more frequently)
         */
        div = gcd(masterConfig.blackbox_rate_num, masterConfig.blackbox_rate_denom);

        masterConfig.blackbox_rate_num /= div;
        masterConfig.blackbox_rate_denom /= div;
    }

    if (masterConfig.blackbox_device >= BLACKBOX_DEVICE_END) {
        masterConfig.blackbox_device = BLACKBOX_DEVICE_SERIAL;
    }
}

/**
 * Start Blackbox logging if it is not already running. Intended to be called upon arming.
 */
void startBlackbox(void)
{
    if (blackboxState == BLACKBOX_STATE_STOPPED) {
        validateBlackboxConfig();

        if (!blackboxDeviceOpen()) {
            blackboxSetState(BLACKBOX_STATE_DISABLED);
            return;
        }

        memset(&gpsHistory, 0, sizeof(gpsHistory));

        blackboxHistory[0] = &blackboxHistoryRing[0];
        blackboxHistory[1] = &blackboxHistoryRing[1];
        blackboxHistory[2] = &blackboxHistoryRing[2];

        vbatReference = vbatLatestADC;

        //No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it

        /*
         * We use conditional tests to decide whether or not certain fields should be logged. Since our headers
         * must always agree with the logged data, the results of these tests must not change during logging. So
         * cache those now.
         */
        blackboxBuildConditionCache();

        /*
         * Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
         * it finally plays the beep for this arming event.
         */
        blackboxLastArmingBeep = getArmingBeepTimeMicros();

        blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
    }
}

/**
 * Begin Blackbox shutdown.
 */
void finishBlackbox(void)
{
    if (blackboxState == BLACKBOX_STATE_RUNNING) {
        blackboxLogEvent(FLIGHT_LOG_EVENT_LOG_END, NULL);

        blackboxSetState(BLACKBOX_STATE_SHUTTING_DOWN);
    } else if (blackboxState != BLACKBOX_STATE_DISABLED && blackboxState != BLACKBOX_STATE_STOPPED
            && blackboxState != BLACKBOX_STATE_SHUTTING_DOWN) {
        /*
         * We're shutting down in the middle of transmitting headers, so we can't log a "log completed" event.
         * Just give the port back and stop immediately.
         */
        blackboxDeviceClose();
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    }
}

#ifdef GPS
static void writeGPSHomeFrame()
{
    blackboxWrite('H');

    blackboxWriteSignedVB(GPS_home[0]);
    blackboxWriteSignedVB(GPS_home[1]);
    //TODO it'd be great if we could grab the GPS current time and write that too

    gpsHistory.GPS_home[0] = GPS_home[0];
    gpsHistory.GPS_home[1] = GPS_home[1];
}

static void writeGPSFrame()
{
    blackboxWrite('G');

    /*
     * If we're logging every frame, then a GPS frame always appears just after a frame with the
     * currentTime timestamp in the log, so the reader can just use that timestamp for the GPS frame.
     *
     * If we're not logging every frame, we need to store the time of this GPS frame.
     */
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME)) {
        // Predict the time of the last frame in the main log
        blackboxWriteUnsignedVB(currentTime - blackboxHistory[1]->time);
    }

    blackboxWriteUnsignedVB(GPS_numSat);
    blackboxWriteSignedVB(GPS_coord[0] - gpsHistory.GPS_home[0]);
    blackboxWriteSignedVB(GPS_coord[1] - gpsHistory.GPS_home[1]);
    blackboxWriteUnsignedVB(GPS_altitude);
    blackboxWriteUnsignedVB(GPS_speed);
    blackboxWriteUnsignedVB(GPS_ground_course);

    gpsHistory.GPS_numSat = GPS_numSat;
    gpsHistory.GPS_coord[0] = GPS_coord[0];
    gpsHistory.GPS_coord[1] = GPS_coord[1];
}
#endif

/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
static void loadBlackboxState(void)
{
    blackboxValues_t *blackboxCurrent = blackboxHistory[0];
    int i;

    blackboxCurrent->time = currentTime;

    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_P[i] = axisPID_P[i];
    }
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_I[i] = axisPID_I[i];
    }
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_D[i] = axisPID_D[i];
    }

    for (i = 0; i < 4; i++) {
        blackboxCurrent->rcCommand[i] = rcCommand[i];
    }

    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->gyroADC[i] = gyroADC[i];
    }

    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->accSmooth[i] = accSmooth[i];
    }

    for (i = 0; i < motorCount; i++) {
        blackboxCurrent->motor[i] = motor[i];
    }

    blackboxCurrent->vbatLatest = vbatLatestADC;
    blackboxCurrent->amperageLatest = amperageLatestADC;

#ifdef MAG
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->magADC[i] = magADC[i];
    }
#endif

#ifdef BARO
    blackboxCurrent->BaroAlt = BaroAlt;
#endif

#ifdef SONAR
    // Store the raw sonar value without applying tilt correction
    blackboxCurrent->sonarRaw = sonarRead();
#endif

#ifdef USE_SERVOS
    //Tail servo for tricopters
    blackboxCurrent->servo[5] = servo[5];
#endif
}

/**
 * Transmit the header information for the given field definitions. Transmitted header lines look like:
 *
 * H Field I name:a,b,c
 * H Field I predictor:0,1,2
 *
 * Provide an array 'conditions' of FlightLogFieldCondition enums if you want these conditions to decide whether a field
 * should be included or not. Otherwise provide NULL for this parameter and NULL for secondCondition.
 *
 * Set xmitState.headerIndex to 0 and xmitState.u.fieldIndex to -1 before calling for the first time.
 *
 * secondFieldDefinition and secondCondition element pointers need to be provided in order to compute the stride of the
 * fieldDefinition and secondCondition arrays.
 *
 * Returns true if there is still header left to transmit (so call again to continue transmission).
 */
static bool sendFieldDefinition(const char * const *headerNames, unsigned int headerCount, const void *fieldDefinitions,
        const void *secondFieldDefinition, int fieldCount, const uint8_t *conditions, const uint8_t *secondCondition)
{
    const blackboxFieldDefinition_t *def;
    int charsWritten;
    static bool needComma = false;
    size_t definitionStride = (char*) secondFieldDefinition - (char*) fieldDefinitions;
    size_t conditionsStride = (char*) secondCondition - (char*) conditions;

    /*
     * We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
     * the whole header.
     */
    if (xmitState.u.fieldIndex == -1) {
        if (xmitState.headerIndex >= headerCount) {
            return false; //Someone probably called us again after we had already completed transmission
        }

        charsWritten = blackboxPrint("H Field ");
        charsWritten += blackboxPrint(headerNames[xmitState.headerIndex]);
        blackboxWrite(':');
        charsWritten++;

        xmitState.u.fieldIndex++;
        needComma = false;
    } else
        charsWritten = 0;

    for (; xmitState.u.fieldIndex < fieldCount && charsWritten < blackboxWriteChunkSize; xmitState.u.fieldIndex++) {
        def = (const blackboxFieldDefinition_t*) ((const char*)fieldDefinitions + definitionStride * xmitState.u.fieldIndex);

        if (!conditions || testBlackboxCondition(conditions[conditionsStride * xmitState.u.fieldIndex])) {
            if (needComma) {
                blackboxWrite(',');
                charsWritten++;
            } else {
                needComma = true;
            }

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                charsWritten += blackboxPrint(def->name);

                // Do we need to print an index in brackets after the name?
                if (def->fieldNameIndex != -1) {
                    charsWritten += blackboxPrintf("[%d]", def->fieldNameIndex);
                }
            } else {
                //The other headers are integers
                charsWritten += blackboxPrintf("%d", def->arr[xmitState.headerIndex - 1]);
            }
        }
    }

    // Did we complete this line?
    if (xmitState.u.fieldIndex == fieldCount) {
        blackboxWrite('\n');
        xmitState.headerIndex++;
        xmitState.u.fieldIndex = -1;
    }

    return xmitState.headerIndex < headerCount;
}

/**
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
static bool blackboxWriteSysinfo()
{
    if (xmitState.headerIndex == 0) {
        xmitState.u.serialBudget = 0;
        xmitState.headerIndex = 1;
    }

    // How many bytes can we afford to transmit this loop?
    xmitState.u.serialBudget = MIN(xmitState.u.serialBudget + blackboxWriteChunkSize, 64);

    // Most headers will consume at least 20 bytes so wait until we've built up that much link budget
    if (xmitState.u.serialBudget < 20) {
        return false;
    }

    switch (xmitState.headerIndex) {
        case 0:
            //Shouldn't ever get here
        break;
        case 1:
            xmitState.u.serialBudget -= blackboxPrint("H Firmware type:Cleanflight\n");
        break;
        case 2:
            xmitState.u.serialBudget -= blackboxPrintf("H Firmware revision:%s\n", shortGitRevision);
        break;
        case 3:
            xmitState.u.serialBudget -= blackboxPrintf("H Firmware date:%s %s\n", buildDate, buildTime);
        break;
        case 4:
            xmitState.u.serialBudget -= blackboxPrintf("H P interval:%d/%d\n", masterConfig.blackbox_rate_num, masterConfig.blackbox_rate_denom);
        break;
        case 5:
            xmitState.u.serialBudget -= blackboxPrintf("H rcRate:%d\n", masterConfig.controlRateProfiles[masterConfig.current_profile_index].rcRate8);
        break;
        case 6:
            xmitState.u.serialBudget -= blackboxPrintf("H minthrottle:%d\n", masterConfig.escAndServoConfig.minthrottle);
        break;
        case 7:
            xmitState.u.serialBudget -= blackboxPrintf("H maxthrottle:%d\n", masterConfig.escAndServoConfig.maxthrottle);
        break;
        case 8:
            xmitState.u.serialBudget -= blackboxPrintf("H gyro.scale:0x%x\n", castFloatBytesToInt(gyro.scale));
        break;
        case 9:
            xmitState.u.serialBudget -= blackboxPrintf("H acc_1G:%u\n", acc_1G);
        break;
        case 10:
            if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
                xmitState.u.serialBudget -= blackboxPrintf("H vbatscale:%u\n", masterConfig.batteryConfig.vbatscale);
            } else {
                xmitState.headerIndex += 2; // Skip the next two vbat fields too
            }
        break;
        case 11:
            xmitState.u.serialBudget -= blackboxPrintf("H vbatcellvoltage:%u,%u,%u\n", masterConfig.batteryConfig.vbatmincellvoltage,
                masterConfig.batteryConfig.vbatwarningcellvoltage, masterConfig.batteryConfig.vbatmaxcellvoltage);
        break;
        case 12:
            xmitState.u.serialBudget -= blackboxPrintf("H vbatref:%u\n", vbatReference);
        break;
        case 13:
            //Note: Log even if this is a virtual current meter, since the virtual meter uses these parameters too:
            if (feature(FEATURE_CURRENT_METER)) {
                xmitState.u.serialBudget -= blackboxPrintf("H currentMeter:%d,%d\n", masterConfig.batteryConfig.currentMeterOffset, masterConfig.batteryConfig.currentMeterScale);
            }
        break;
        default:
            return true;
    }

    xmitState.headerIndex++;
    return false;
}

/**
 * Write the given event to the log immediately
 */
void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t *data)
{
    if (blackboxState != BLACKBOX_STATE_RUNNING) {
        return;
    }

    //Shared header for event frames
    blackboxWrite('E');
    blackboxWrite(event);

    //Now serialize the data for this specific frame type
    switch (event) {
        case FLIGHT_LOG_EVENT_SYNC_BEEP:
            blackboxWriteUnsignedVB(data->syncBeep.time);
        break;
        case FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_START:
            blackboxWrite(data->autotuneCycleStart.phase);
            blackboxWrite(data->autotuneCycleStart.cycle | (data->autotuneCycleStart.rising ? 0x80 : 0));
            blackboxWrite(data->autotuneCycleStart.p);
            blackboxWrite(data->autotuneCycleStart.i);
            blackboxWrite(data->autotuneCycleStart.d);
        break;
        case FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_RESULT:
            blackboxWrite(data->autotuneCycleResult.flags);
            blackboxWrite(data->autotuneCycleStart.p);
            blackboxWrite(data->autotuneCycleStart.i);
            blackboxWrite(data->autotuneCycleStart.d);
        break;
        case FLIGHT_LOG_EVENT_AUTOTUNE_TARGETS:
            blackboxWriteS16(data->autotuneTargets.currentAngle);
            blackboxWrite((uint8_t) data->autotuneTargets.targetAngle);
            blackboxWrite((uint8_t) data->autotuneTargets.targetAngleAtPeak);
            blackboxWriteS16(data->autotuneTargets.firstPeakAngle);
            blackboxWriteS16(data->autotuneTargets.secondPeakAngle);
        break;
        case FLIGHT_LOG_EVENT_LOG_END:
            blackboxPrint("End of log");
            blackboxWrite(0);
        break;
    }
}

/* If an arming beep has played since it was last logged, write the time of the arming beep to the log as a synchronization point */
static void blackboxCheckAndLogArmingBeep()
{
    flightLogEvent_syncBeep_t eventData;

    // Use != so that we can still detect a change if the counter wraps
    if (getArmingBeepTimeMicros() != blackboxLastArmingBeep) {
        blackboxLastArmingBeep = getArmingBeepTimeMicros();

        eventData.time = blackboxLastArmingBeep;

        blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP, (flightLogEventData_t *) &eventData);
    }
}

/**
 * Call each flight loop iteration to perform blackbox logging.
 */
void handleBlackbox(void)
{
    int i;

    switch (blackboxState) {
        case BLACKBOX_STATE_SEND_HEADER:
            //On entry of this state, xmitState.headerIndex is 0 and startTime is intialised

            /*
             * Once the UART has had time to init, transmit the header in chunks so we don't overflow our transmit
             * buffer.
             */
            if (millis() > xmitState.u.startTime + 100) {
                for (i = 0; i < blackboxWriteChunkSize && blackboxHeader[xmitState.headerIndex] != '\0'; i++, xmitState.headerIndex++) {
                    blackboxWrite(blackboxHeader[xmitState.headerIndex]);
                }

                if (blackboxHeader[xmitState.headerIndex] == '\0') {
                    blackboxSetState(BLACKBOX_STATE_SEND_FIELDINFO);
                }
            }
        break;
        case BLACKBOX_STATE_SEND_FIELDINFO:
            //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
            if (!sendFieldDefinition(blackboxMainHeaderNames, ARRAY_LENGTH(blackboxMainHeaderNames), blackboxMainFields, blackboxMainFields + 1,
                    ARRAY_LENGTH(blackboxMainFields), &blackboxMainFields[0].condition, &blackboxMainFields[1].condition)) {
#ifdef GPS
                if (feature(FEATURE_GPS)) {
                    blackboxSetState(BLACKBOX_STATE_SEND_GPS_H_HEADERS);
                } else
#endif
                    blackboxSetState(BLACKBOX_STATE_SEND_SYSINFO);
            }
        break;
#ifdef GPS
        case BLACKBOX_STATE_SEND_GPS_H_HEADERS:
            //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
            if (!sendFieldDefinition(blackboxGPSHHeaderNames, ARRAY_LENGTH(blackboxGPSHHeaderNames), blackboxGpsHFields, blackboxGpsHFields + 1,
                    ARRAY_LENGTH(blackboxGpsHFields), &blackboxGpsHFields[0].condition, &blackboxGpsHFields[1].condition)) {
                blackboxSetState(BLACKBOX_STATE_SEND_GPS_G_HEADERS);
            }
        break;
        case BLACKBOX_STATE_SEND_GPS_G_HEADERS:
            //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
            if (!sendFieldDefinition(blackboxGPSGHeaderNames, ARRAY_LENGTH(blackboxGPSGHeaderNames), blackboxGpsGFields, blackboxGpsGFields + 1,
                    ARRAY_LENGTH(blackboxGpsGFields), &blackboxGpsGFields[0].condition, &blackboxGpsGFields[1].condition)) {
                blackboxSetState(BLACKBOX_STATE_SEND_SYSINFO);
            }
        break;
#endif
        case BLACKBOX_STATE_SEND_SYSINFO:
            //On entry of this state, xmitState.headerIndex is 0

            //Keep writing chunks of the system info headers until it returns true to signal completion
            if (blackboxWriteSysinfo()) {
                blackboxSetState(BLACKBOX_STATE_RUNNING);
            }
        break;
        case BLACKBOX_STATE_RUNNING:
            // On entry to this state, blackboxIteration, blackboxPFrameIndex and blackboxIFrameIndex are reset to 0

            // Write a keyframe every BLACKBOX_I_INTERVAL frames so we can resynchronise upon missing frames
            if (blackboxPFrameIndex == 0) {
                // Copy current system values into the blackbox
                loadBlackboxState();
                writeIntraframe();
            } else {
                blackboxCheckAndLogArmingBeep();

                /* Adding a magic shift of "masterConfig.blackbox_rate_num - 1" in here creates a better spread of
                 * recorded / skipped frames when the I frame's position is considered:
                 */
                if ((blackboxPFrameIndex + masterConfig.blackbox_rate_num - 1) % masterConfig.blackbox_rate_denom < masterConfig.blackbox_rate_num) {
                    loadBlackboxState();
                    writeInterframe();
                }
#ifdef GPS
                if (feature(FEATURE_GPS)) {
                    /*
                     * If the GPS home point has been updated, or every 128 intraframes (~10 seconds), write the
                     * GPS home position.
                     *
                     * We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
                     * still be interpreted correctly.
                     */
                    if (GPS_home[0] != gpsHistory.GPS_home[0] || GPS_home[1] != gpsHistory.GPS_home[1]
                        || (blackboxPFrameIndex == BLACKBOX_I_INTERVAL / 2 && blackboxIFrameIndex % 128 == 0)) {

                        writeGPSHomeFrame();
                        writeGPSFrame();
                    } else if (GPS_numSat != gpsHistory.GPS_numSat || GPS_coord[0] != gpsHistory.GPS_coord[0]
                            || GPS_coord[1] != gpsHistory.GPS_coord[1]) {
                        //We could check for velocity changes as well but I doubt it changes independent of position
                        writeGPSFrame();
                    }
                }
#endif
            }

            blackboxIteration++;
            blackboxPFrameIndex++;
            
            if (blackboxPFrameIndex == BLACKBOX_I_INTERVAL) {
                blackboxPFrameIndex = 0;
                blackboxIFrameIndex++;
            }
        break;
        case BLACKBOX_STATE_SHUTTING_DOWN:
            //On entry of this state, startTime is set and a flush is performed

            /*
             * Wait for the log we've transmitted to make its way to the logger before we release the serial port,
             * since releasing the port clears the Tx buffer.
             *
             * Don't wait longer than it could possibly take if something funky happens.
             */
            if (millis() > xmitState.u.startTime + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS || blackboxDeviceFlush()) {
                blackboxDeviceClose();
                blackboxSetState(BLACKBOX_STATE_STOPPED);
            }
        break;
        default:
        break;
    }

    // Did we run out of room on the device? Stop!
    if (isBlackboxDeviceFull()) {
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    }
}

static bool canUseBlackboxWithCurrentConfiguration(void)
{
    return feature(FEATURE_BLACKBOX);
}

/**
 * Call during system startup to initialize the blackbox.
 */
void initBlackbox(void)
{
    if (canUseBlackboxWithCurrentConfiguration()) {
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    } else {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
    }
}

#endif
