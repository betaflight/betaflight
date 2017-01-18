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
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef BLACKBOX

#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/encoding.h"
#include "common/utils.h"

#include "blackbox.h"
#include "blackbox_io.h"

#include "drivers/sensor.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "io/beeper.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/sonar.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#define BLACKBOX_I_INTERVAL 32
#define BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS 200
#define SLOW_FRAME_INTERVAL 4096

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
    "H Data version:2\n"
    "H I interval:" STR(BLACKBOX_I_INTERVAL) "\n";

static const char* const blackboxFieldHeaderNames[] = {
    "name",
    "signed",
    "predictor",
    "encoding",
    "predictor",
    "encoding"
};

/* All field definition structs should look like this (but with longer arrs): */
typedef struct blackboxFieldDefinition_s {
    const char *name;
    // If the field name has a number to be included in square brackets [1] afterwards, set it here, or -1 for no brackets:
    int8_t fieldNameIndex;

    // Each member of this array will be the value to print for this field for the given header index
    uint8_t arr[1];
} blackboxFieldDefinition_t;

#define BLACKBOX_DELTA_FIELD_HEADER_COUNT       ARRAY_LENGTH(blackboxFieldHeaderNames)
#define BLACKBOX_SIMPLE_FIELD_HEADER_COUNT      (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)
#define BLACKBOX_CONDITIONAL_FIELD_HEADER_COUNT (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)

typedef struct blackboxSimpleFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
} blackboxSimpleFieldDefinition_t;

typedef struct blackboxConditionalFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxConditionalFieldDefinition_t;

typedef struct blackboxDeltaFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxDeltaFieldDefinition_t;

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static const blackboxDeltaFieldDefinition_t blackboxMainFields[] = {
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
    {"rssi",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_RSSI},

    /* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
    {"gyroADC",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroADC",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroADC",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth",  0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth",  1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth",  2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"debug",    0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"debug",    1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"debug",    2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"debug",    3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    /* Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones): */
    {"motor",      0, UNSIGNED, .Ipredict = PREDICT(MINMOTOR), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_1)},
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
static const blackboxConditionalFieldDefinition_t blackboxGpsGFields[] = {
    {"time",              -1, UNSIGNED, PREDICT(LAST_MAIN_FRAME_TIME), ENCODING(UNSIGNED_VB), CONDITION(NOT_LOGGING_EVERY_FRAME)},
    {"GPS_numSat",        -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_coord",          0, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_coord",          1, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_altitude",      -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_speed",         -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_ground_course", -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)}
};

// GPS home frame
static const blackboxSimpleFieldDefinition_t blackboxGpsHFields[] = {
    {"GPS_home",           0, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)},
    {"GPS_home",           1, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)}
};
#endif

// Rarely-updated fields
static const blackboxSimpleFieldDefinition_t blackboxSlowFields[] = {
    {"flightModeFlags",       -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},
    {"stateFlags",            -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},

    {"failsafePhase",         -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxSignalReceived",      -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxFlightChannelsValid", -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)}
};

typedef enum BlackboxState {
    BLACKBOX_STATE_DISABLED = 0,
    BLACKBOX_STATE_STOPPED,
    BLACKBOX_STATE_PREPARE_LOG_FILE,
    BLACKBOX_STATE_SEND_HEADER,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER,
    BLACKBOX_STATE_SEND_GPS_H_HEADER,
    BLACKBOX_STATE_SEND_GPS_G_HEADER,
    BLACKBOX_STATE_SEND_SLOW_HEADER,
    BLACKBOX_STATE_SEND_SYSINFO,
    BLACKBOX_STATE_PAUSED,
    BLACKBOX_STATE_RUNNING,
    BLACKBOX_STATE_SHUTTING_DOWN
} BlackboxState;

#define BLACKBOX_FIRST_HEADER_SENDING_STATE BLACKBOX_STATE_SEND_HEADER
#define BLACKBOX_LAST_HEADER_SENDING_STATE BLACKBOX_STATE_SEND_SYSINFO

typedef struct blackboxMainState_s {
    uint32_t time;

    int32_t axisPID_P[XYZ_AXIS_COUNT], axisPID_I[XYZ_AXIS_COUNT], axisPID_D[XYZ_AXIS_COUNT];

    int16_t rcCommand[4];
    int16_t gyroADC[XYZ_AXIS_COUNT];
    int16_t accSmooth[XYZ_AXIS_COUNT];
    int16_t debug[4];
    int16_t motor[MAX_SUPPORTED_MOTORS];
    int16_t servo[MAX_SUPPORTED_SERVOS];

    uint16_t vbatLatest;
    uint16_t amperageLatest;

#ifdef BARO
    int32_t BaroAlt;
#endif
#ifdef MAG
    int16_t magADC[XYZ_AXIS_COUNT];
#endif
#ifdef SONAR
    int32_t sonarRaw;
#endif
    uint16_t rssi;
} blackboxMainState_t;

typedef struct blackboxGpsState_s {
    int32_t GPS_home[2], GPS_coord[2];
    uint8_t GPS_numSat;
} blackboxGpsState_t;

// This data is updated really infrequently:
typedef struct blackboxSlowState_s {
    uint32_t flightModeFlags; // extend this data size (from uint16_t)
    uint8_t stateFlags;
    uint8_t failsafePhase;
    bool rxSignalReceived;
    bool rxFlightChannelsValid;
} __attribute__((__packed__)) blackboxSlowState_t; // We pack this struct so that padding doesn't interfere with memcmp()

//From mixer.c:
extern uint16_t motorOutputHigh, motorOutputLow;

//From rc_controls.c
extern uint32_t rcModeActivationMask;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;

static uint32_t blackboxLastArmingBeep = 0;
static uint32_t blackboxLastFlightModeFlags = 0; // New event tracking of flight modes


static struct {
    uint32_t headerIndex;

    /* Since these fields are used during different blackbox states (never simultaneously) we can
     * overlap them to save on RAM
     */
    union {
        int fieldIndex;
        uint32_t startTime;
    } u;
} xmitState;

// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
static uint32_t blackboxConditionCache;

STATIC_ASSERT((sizeof(blackboxConditionCache) * 8) >= FLIGHT_LOG_FIELD_CONDITION_NEVER, too_many_flight_log_conditions);

static uint32_t blackboxIteration;
static uint16_t blackboxPFrameIndex, blackboxIFrameIndex;
static uint16_t blackboxSlowFrameIterationTimer;
static bool blackboxLoggedAnyFrames;

/*
 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs
 * to encode:
 */
static uint16_t vbatReference;

static blackboxGpsState_t gpsHistory;
static blackboxSlowState_t slowHistory;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxMainState_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxMainState_t* blackboxHistory[3];

static bool blackboxModeActivationConditionPresent = false;

/**
 * Return true if it is safe to edit the Blackbox configuration in the emasterConfig.
 */
bool blackboxMayEditConfig()
{
    return blackboxState <= BLACKBOX_STATE_STOPPED;
}

static bool blackboxIsOnlyLoggingIntraframes() {
    return blackboxConfig()->rate_num == 1 && blackboxConfig()->rate_denom == 32;
}

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
            return getMotorCount() >= condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 + 1;

        case FLIGHT_LOG_FIELD_CONDITION_TRICOPTER:
            return mixerConfig()->mixerMode == MIXER_TRI || mixerConfig()->mixerMode == MIXER_CUSTOM_TRI;

        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2:
            return currentProfile->pidProfile.D8[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0] != 0;

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
            return feature(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC;

        case FLIGHT_LOG_FIELD_CONDITION_SONAR:
#ifdef SONAR
            return feature(FEATURE_SONAR);
#else
            return false;
#endif

        case FLIGHT_LOG_FIELD_CONDITION_RSSI:
            return rxConfig()->rssi_channel > 0 || feature(FEATURE_RSSI_ADC);

        case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
            return blackboxConfig()->rate_num < blackboxConfig()->rate_denom;

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
        case BLACKBOX_STATE_PREPARE_LOG_FILE:
            blackboxLoggedAnyFrames = false;
        break;
        case BLACKBOX_STATE_SEND_HEADER:
            blackboxHeaderBudget = 0;
            xmitState.headerIndex = 0;
            xmitState.u.startTime = millis();
        break;
        case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
        case BLACKBOX_STATE_SEND_GPS_G_HEADER:
        case BLACKBOX_STATE_SEND_GPS_H_HEADER:
        case BLACKBOX_STATE_SEND_SLOW_HEADER:
            xmitState.headerIndex = 0;
            xmitState.u.fieldIndex = -1;
        break;
        case BLACKBOX_STATE_SEND_SYSINFO:
            xmitState.headerIndex = 0;
        break;
        case BLACKBOX_STATE_RUNNING:
            blackboxSlowFrameIterationTimer = SLOW_FRAME_INTERVAL; //Force a slow frame to be written on the first iteration
        break;
        case BLACKBOX_STATE_SHUTTING_DOWN:
            xmitState.u.startTime = millis();
        break;
        default:
            ;
    }
    blackboxState = newState;
}

static void writeIntraframe(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    blackboxWrite('I');

    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(blackboxCurrent->time);

    blackboxWriteSignedVBArray(blackboxCurrent->axisPID_P, XYZ_AXIS_COUNT);
    blackboxWriteSignedVBArray(blackboxCurrent->axisPID_I, XYZ_AXIS_COUNT);

    // Don't bother writing the current D term if the corresponding PID setting is zero
    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
            blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x]);
        }
    }

    // Write roll, pitch and yaw first:
    blackboxWriteSigned16VBArray(blackboxCurrent->rcCommand, 3);

    /*
     * Write the throttle separately from the rest of the RC data so we can apply a predictor to it.
     * Throttle lies in range [minthrottle..maxthrottle]:
     */
    blackboxWriteUnsignedVB(blackboxCurrent->rcCommand[THROTTLE] - motorConfig()->minthrottle);

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
            blackboxWriteSigned16VBArray(blackboxCurrent->magADC, XYZ_AXIS_COUNT);
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

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        blackboxWriteUnsignedVB(blackboxCurrent->rssi);
    }

    blackboxWriteSigned16VBArray(blackboxCurrent->gyroADC, XYZ_AXIS_COUNT);
    blackboxWriteSigned16VBArray(blackboxCurrent->accSmooth, XYZ_AXIS_COUNT);
    blackboxWriteSigned16VBArray(blackboxCurrent->debug, 4);

    //Motors can be below minimum output when disarmed, but that doesn't happen much
    blackboxWriteUnsignedVB(blackboxCurrent->motor[0] - motorOutputLow);

    //Motors tend to be similar to each other so use the first motor's value as a predictor of the others
    const int motorCount = getMotorCount();
    for (int x = 1; x < motorCount; x++) {
        blackboxWriteSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);
    }

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
        //Assume the tail spends most of its time around the center
        blackboxWriteSignedVB(blackboxCurrent->servo[5] - 1500);
    }

    //Rotate our history buffers:

    //The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    //And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    //And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

    blackboxLoggedAnyFrames = true;
}

static void blackboxWriteMainStateArrayUsingAveragePredictor(int arrOffsetInHistory, int count)
{
    int16_t *curr  = (int16_t*) ((char*) (blackboxHistory[0]) + arrOffsetInHistory);
    int16_t *prev1 = (int16_t*) ((char*) (blackboxHistory[1]) + arrOffsetInHistory);
    int16_t *prev2 = (int16_t*) ((char*) (blackboxHistory[2]) + arrOffsetInHistory);

    for (int i = 0; i < count; i++) {
        // Predictor is the average of the previous two history states
        int32_t predictor = (prev1[i] + prev2[i]) / 2;

        blackboxWriteSignedVB(curr[i] - predictor);
    }
}

static void writeInterframe(void)
{
    int x;
    int32_t deltas[8];

    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    blackboxMainState_t *blackboxLast = blackboxHistory[1];

    blackboxWrite('P');

    //No need to store iteration count since its delta is always 1

    /*
     * Since the difference between the difference between successive times will be nearly zero (due to consistent
     * looptime spacing), use second-order differences.
     */
    blackboxWriteSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    arraySubInt32(deltas, blackboxCurrent->axisPID_P, blackboxLast->axisPID_P, XYZ_AXIS_COUNT);
    blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);

    /*
     * The PID I field changes very slowly, most of the time +-2, so use an encoding
     * that can pack all three fields into one byte in that situation.
     */
    arraySubInt32(deltas, blackboxCurrent->axisPID_I, blackboxLast->axisPID_I, XYZ_AXIS_COUNT);
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

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->rssi - blackboxLast->rssi;
    }

    blackboxWriteTag8_8SVB(deltas, optionalFieldCount);

    //Since gyros, accs and motors are noisy, base their predictions on the average of the history:
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC),   XYZ_AXIS_COUNT);
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accSmooth), XYZ_AXIS_COUNT);
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, debug), 4);
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, motor),     getMotorCount());

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
        blackboxWriteSignedVB(blackboxCurrent->servo[5] - blackboxLast->servo[5]);
    }

    //Rotate our history buffers
    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

    blackboxLoggedAnyFrames = true;
}

/* Write the contents of the global "slowHistory" to the log as an "S" frame. Because this data is logged so
 * infrequently, delta updates are not reasonable, so we log independent frames. */
static void writeSlowFrame(void)
{
    int32_t values[3];

    blackboxWrite('S');

    blackboxWriteUnsignedVB(slowHistory.flightModeFlags);
    blackboxWriteUnsignedVB(slowHistory.stateFlags);

    /*
     * Most of the time these three values will be able to pack into one byte for us:
     */
    values[0] = slowHistory.failsafePhase;
    values[1] = slowHistory.rxSignalReceived ? 1 : 0;
    values[2] = slowHistory.rxFlightChannelsValid ? 1 : 0;
    blackboxWriteTag2_3S32(values);

    blackboxSlowFrameIterationTimer = 0;
}

/**
 * Load rarely-changing values from the FC into the given structure
 */
static void loadSlowState(blackboxSlowState_t *slow)
{
    slow->flightModeFlags = rcModeActivationMask; //was flightModeFlags;
    slow->stateFlags = stateFlags;
    slow->failsafePhase = failsafePhase();
    slow->rxSignalReceived = rxIsReceivingSignal();
    slow->rxFlightChannelsValid = rxAreFlightChannelsValid();
}

/**
 * If the data in the slow frame has changed, log a slow frame.
 *
 * If allowPeriodicWrite is true, the frame is also logged if it has been more than SLOW_FRAME_INTERVAL logging iterations
 * since the field was last logged.
 */
static void writeSlowFrameIfNeeded(bool allowPeriodicWrite)
{
    // Write the slow frame peridocially so it can be recovered if we ever lose sync
    bool shouldWrite = allowPeriodicWrite && blackboxSlowFrameIterationTimer >= SLOW_FRAME_INTERVAL;

    if (shouldWrite) {
        loadSlowState(&slowHistory);
    } else {
        blackboxSlowState_t newSlowState;

        loadSlowState(&newSlowState);

        // Only write a slow frame if it was different from the previous state
        if (memcmp(&newSlowState, &slowHistory, sizeof(slowHistory)) != 0) {
            // Use the new state as our new history
            memcpy(&slowHistory, &newSlowState, sizeof(slowHistory));
            shouldWrite = true;
        }
    }

    if (shouldWrite) {
        writeSlowFrame();
    }
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

    if (blackboxConfig()->rate_num == 0 || blackboxConfig()->rate_denom == 0
            || blackboxConfig()->rate_num >= blackboxConfig()->rate_denom) {
        blackboxConfig()->rate_num = 1;
        blackboxConfig()->rate_denom = 1;
    } else {
        /* Reduce the fraction the user entered as much as possible (makes the recorded/skipped frame pattern repeat
         * itself more frequently)
         */
        div = gcd(blackboxConfig()->rate_num, blackboxConfig()->rate_denom);

        blackboxConfig()->rate_num /= div;
        blackboxConfig()->rate_denom /= div;
    }

    // If we've chosen an unsupported device, change the device to serial
    switch (blackboxConfig()->device) {
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
#endif
#ifdef USE_SDCARD
        case BLACKBOX_DEVICE_SDCARD:
#endif
        case BLACKBOX_DEVICE_SERIAL:
            // Device supported, leave the setting alone
        break;

        default:
            blackboxConfig()->device = BLACKBOX_DEVICE_SERIAL;
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

        vbatReference = vbatLatest;

        //No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it

        /*
         * We use conditional tests to decide whether or not certain fields should be logged. Since our headers
         * must always agree with the logged data, the results of these tests must not change during logging. So
         * cache those now.
         */
        blackboxBuildConditionCache();

        blackboxModeActivationConditionPresent = isModeActivationConditionPresent(modeActivationProfile()->modeActivationConditions, BOXBLACKBOX);

        blackboxIteration = 0;
        blackboxPFrameIndex = 0;
        blackboxIFrameIndex = 0;

        /*
         * Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
         * it finally plays the beep for this arming event.
         */
        blackboxLastArmingBeep = getArmingBeepTimeMicros();
        blackboxLastFlightModeFlags = rcModeActivationMask; // record startup status


        blackboxSetState(BLACKBOX_STATE_PREPARE_LOG_FILE);
    }
}

/**
 * Begin Blackbox shutdown.
 */
void finishBlackbox(void)
{
    switch (blackboxState) {
        case BLACKBOX_STATE_DISABLED:
        case BLACKBOX_STATE_STOPPED:
        case BLACKBOX_STATE_SHUTTING_DOWN:
            // We're already stopped/shutting down
        break;

        case BLACKBOX_STATE_RUNNING:
        case BLACKBOX_STATE_PAUSED:
            blackboxLogEvent(FLIGHT_LOG_EVENT_LOG_END, NULL);

            // Fall through
        default:
            blackboxSetState(BLACKBOX_STATE_SHUTTING_DOWN);
    }
}

/**
 * Test Motors Blackbox Logging
 */
bool startedLoggingInTestMode = false;

void startInTestMode(void)
{
    if(!startedLoggingInTestMode) {
        if (blackboxConfig()->device == BLACKBOX_DEVICE_SERIAL) {
            serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
            if (sharedBlackboxAndMspPort) {
                return; // When in test mode, we cannot share the MSP and serial logger port!
            }
        }
        startBlackbox();
        startedLoggingInTestMode = true;
    }
}
void stopInTestMode(void)
{
    if(startedLoggingInTestMode) {
        finishBlackbox();
        startedLoggingInTestMode = false;
    }
}
/**
 * We are going to monitor the MSP_SET_MOTOR target variables motor_disarmed[] for values other than minthrottle
 * on reading a value (i.e. the user is testing the motors), then we enable test mode logging;
 * we monitor when the values return to minthrottle and start a delay timer (5 seconds); if
 * the test motors are left at minimum throttle for this delay timer, then we assume we are done testing and
 * shutdown the logger.
 *
 * Of course, after the 5 seconds and shutdown of the logger, the system will be re-enabled to allow the
 * test mode to trigger again; its just that the data will be in a second, third, fourth etc log file.
 */
bool inMotorTestMode(void) {
    static uint32_t resetTime = 0;
    uint16_t inactiveMotorCommand;
    if (feature(FEATURE_3D)) {
       inactiveMotorCommand = flight3DConfig()->neutral3d;
#ifdef USE_DSHOT
    } else if (isMotorProtocolDshot()) {
       inactiveMotorCommand = DSHOT_DISARM_COMMAND;
#endif
    } else {
       inactiveMotorCommand = motorConfig()->mincommand;
    }

    int i;
    bool atLeastOneMotorActivated = false;

    // set disarmed motor values
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        atLeastOneMotorActivated |= (motor_disarmed[i] != inactiveMotorCommand);

    if(atLeastOneMotorActivated) {
        resetTime = millis() + 5000; // add 5 seconds
        return true;
    } else {
        // Monitor the duration at minimum
        return (millis() < resetTime);
    }
    return false;
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

static void writeGPSFrame(timeUs_t currentTimeUs)
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
        blackboxWriteUnsignedVB(currentTimeUs - blackboxHistory[1]->time);
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
static void loadMainState(timeUs_t currentTimeUs)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    int i;

    blackboxCurrent->time = currentTimeUs;

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
        blackboxCurrent->gyroADC[i] = lrintf(gyro.gyroADCf[i]);
    }

    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->accSmooth[i] = acc.accSmooth[i];
    }

    for (i = 0; i < 4; i++) {
        blackboxCurrent->debug[i] = debug[i];
    }

    const int motorCount = getMotorCount();
    for (i = 0; i < motorCount; i++) {
        blackboxCurrent->motor[i] = motor[i];
    }

    blackboxCurrent->vbatLatest = vbatLatest;
    blackboxCurrent->amperageLatest = amperageLatest;

#ifdef MAG
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->magADC[i] = mag.magADC[i];
    }
#endif

#ifdef BARO
    blackboxCurrent->BaroAlt = baro.BaroAlt;
#endif

#ifdef SONAR
    // Store the raw sonar value without applying tilt correction
    blackboxCurrent->sonarRaw = sonarRead();
#endif

    blackboxCurrent->rssi = rssi;

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
 * For all header types, provide a "mainFrameChar" which is the name for the field and will be used to refer to it in the
 * header (e.g. P, I etc). For blackboxDeltaField_t fields, also provide deltaFrameChar, otherwise set this to zero.
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
static bool sendFieldDefinition(char mainFrameChar, char deltaFrameChar, const void *fieldDefinitions,
        const void *secondFieldDefinition, int fieldCount, const uint8_t *conditions, const uint8_t *secondCondition)
{
    const blackboxFieldDefinition_t *def;
    unsigned int headerCount;
    static bool needComma = false;
    size_t definitionStride = (char*) secondFieldDefinition - (char*) fieldDefinitions;
    size_t conditionsStride = (char*) secondCondition - (char*) conditions;

    if (deltaFrameChar) {
        headerCount = BLACKBOX_DELTA_FIELD_HEADER_COUNT;
    } else {
        headerCount = BLACKBOX_SIMPLE_FIELD_HEADER_COUNT;
    }

    /*
     * We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
     * the whole header.
     */

    // On our first call we need to print the name of the header and a colon
    if (xmitState.u.fieldIndex == -1) {
        if (xmitState.headerIndex >= headerCount) {
            return false; //Someone probably called us again after we had already completed transmission
        }

        uint32_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[xmitState.headerIndex]);

        if (blackboxDeviceReserveBufferSpace(charsToBeWritten) != BLACKBOX_RESERVE_SUCCESS) {
            return true; // Try again later
        }

        blackboxHeaderBudget -= blackboxPrintf("H Field %c %s:", xmitState.headerIndex >= BLACKBOX_SIMPLE_FIELD_HEADER_COUNT ? deltaFrameChar : mainFrameChar, blackboxFieldHeaderNames[xmitState.headerIndex]);

        xmitState.u.fieldIndex++;
        needComma = false;
    }

    // The longest we expect an integer to be as a string:
    const uint32_t LONGEST_INTEGER_STRLEN = 2;

    for (; xmitState.u.fieldIndex < fieldCount; xmitState.u.fieldIndex++) {
        def = (const blackboxFieldDefinition_t*) ((const char*)fieldDefinitions + definitionStride * xmitState.u.fieldIndex);

        if (!conditions || testBlackboxCondition(conditions[conditionsStride * xmitState.u.fieldIndex])) {
            // First (over)estimate the length of the string we want to print

            int32_t bytesToWrite = 1; // Leading comma

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                bytesToWrite += strlen(def->name) + strlen("[]") + LONGEST_INTEGER_STRLEN;
            } else {
                //The other headers are integers
                bytesToWrite += LONGEST_INTEGER_STRLEN;
            }

            // Now perform the write if the buffer is large enough
            if (blackboxDeviceReserveBufferSpace(bytesToWrite) != BLACKBOX_RESERVE_SUCCESS) {
                // Ran out of space!
                return true;
            }

            blackboxHeaderBudget -= bytesToWrite;

            if (needComma) {
                blackboxWrite(',');
            } else {
                needComma = true;
            }

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                blackboxPrint(def->name);

                // Do we need to print an index in brackets after the name?
                if (def->fieldNameIndex != -1) {
                    blackboxPrintf("[%d]", def->fieldNameIndex);
                }
            } else {
                //The other headers are integers
                blackboxPrintf("%d", def->arr[xmitState.headerIndex - 1]);
            }
        }
    }

    // Did we complete this line?
    if (xmitState.u.fieldIndex == fieldCount && blackboxDeviceReserveBufferSpace(1) == BLACKBOX_RESERVE_SUCCESS) {
        blackboxHeaderBudget--;
        blackboxWrite('\n');
        xmitState.headerIndex++;
        xmitState.u.fieldIndex = -1;
    }

    return xmitState.headerIndex < headerCount;
}

#ifndef BLACKBOX_PRINT_HEADER_LINE
#define BLACKBOX_PRINT_HEADER_LINE(x, ...) case __COUNTER__: \
                                                blackboxPrintfHeaderLine(x, __VA_ARGS__); \
                                                break;
#define BLACKBOX_PRINT_HEADER_LINE_CUSTOM(...) case __COUNTER__: \
                                                    {__VA_ARGS__}; \
                                               break;
#endif

/**
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
static bool blackboxWriteSysinfo()
{
    // Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
    if (blackboxDeviceReserveBufferSpace(64) != BLACKBOX_RESERVE_SUCCESS) {
        return false;
    }

    const profile_t *currentProfile = &masterConfig.profile[masterConfig.current_profile_index];
    const controlRateConfig_t *currentControlRateProfile = &currentProfile->controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile];
    switch (xmitState.headerIndex) {
        BLACKBOX_PRINT_HEADER_LINE("Firmware type:%s",                    "Cleanflight");
        BLACKBOX_PRINT_HEADER_LINE("Firmware revision:%s %s (%s) %s", FC_FIRMWARE_NAME, FC_VERSION_STRING, shortGitRevision, targetName);
        BLACKBOX_PRINT_HEADER_LINE("Firmware date:%s %s",                 buildDate, buildTime);
        BLACKBOX_PRINT_HEADER_LINE("Craft name:%s",                       masterConfig.name);
        BLACKBOX_PRINT_HEADER_LINE("P interval:%d/%d",                    blackboxConfig()->rate_num, blackboxConfig()->rate_denom);
        BLACKBOX_PRINT_HEADER_LINE("minthrottle:%d",                      motorConfig()->minthrottle);
        BLACKBOX_PRINT_HEADER_LINE("maxthrottle:%d",                      motorConfig()->maxthrottle);
        BLACKBOX_PRINT_HEADER_LINE("gyro_scale:0x%x",                     castFloatBytesToInt(1.0f));
        BLACKBOX_PRINT_HEADER_LINE("motorOutput:%d,%d",                   motorOutputLow,motorOutputHigh);
        BLACKBOX_PRINT_HEADER_LINE("acc_1G:%u",                           acc.dev.acc_1G);

        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
                blackboxPrintfHeaderLine("vbatscale:%u", batteryConfig()->vbatscale);
            } else {
                xmitState.headerIndex += 2; // Skip the next two vbat fields too
            }
            );

        BLACKBOX_PRINT_HEADER_LINE("vbatcellvoltage:%u,%u,%u",            batteryConfig()->vbatmincellvoltage,
                                                                          batteryConfig()->vbatwarningcellvoltage,
                                                                          batteryConfig()->vbatmaxcellvoltage);
        BLACKBOX_PRINT_HEADER_LINE("vbatref:%u",                          vbatReference);

        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            //Note: Log even if this is a virtual current meter, since the virtual meter uses these parameters too:
            if (feature(FEATURE_CURRENT_METER)) {
                blackboxPrintfHeaderLine("currentMeter:%d,%d", batteryConfig()->currentMeterOffset, batteryConfig()->currentMeterScale);
            }
            );

        BLACKBOX_PRINT_HEADER_LINE("looptime:%d",                         gyro.targetLooptime);
        BLACKBOX_PRINT_HEADER_LINE("gyro_sync_denom:%d",                  gyroConfig()->gyro_sync_denom);
        BLACKBOX_PRINT_HEADER_LINE("pid_process_denom:%d",                pidConfig()->pid_process_denom);
        BLACKBOX_PRINT_HEADER_LINE("rcRate:%d",                           currentControlRateProfile->rcRate8);
        BLACKBOX_PRINT_HEADER_LINE("rcExpo:%d",                           currentControlRateProfile->rcExpo8);
        BLACKBOX_PRINT_HEADER_LINE("rcYawRate:%d",                        currentControlRateProfile->rcYawRate8);
        BLACKBOX_PRINT_HEADER_LINE("rcYawExpo:%d",                        currentControlRateProfile->rcYawExpo8);
        BLACKBOX_PRINT_HEADER_LINE("thrMid:%d",                           currentControlRateProfile->thrMid8);
        BLACKBOX_PRINT_HEADER_LINE("thrExpo:%d",                          currentControlRateProfile->thrExpo8);
        BLACKBOX_PRINT_HEADER_LINE("dynThrPID:%d",                        currentControlRateProfile->dynThrPID);
        BLACKBOX_PRINT_HEADER_LINE("tpa_breakpoint:%d",                   currentControlRateProfile->tpa_breakpoint);
        BLACKBOX_PRINT_HEADER_LINE("rates:%d,%d,%d",                      currentControlRateProfile->rates[ROLL],
                                                                          currentControlRateProfile->rates[PITCH],
                                                                          currentControlRateProfile->rates[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rollPID:%d,%d,%d",                    currentProfile->pidProfile.P8[ROLL],
                                                                          currentProfile->pidProfile.I8[ROLL],
                                                                          currentProfile->pidProfile.D8[ROLL]);
        BLACKBOX_PRINT_HEADER_LINE("pitchPID:%d,%d,%d",                   currentProfile->pidProfile.P8[PITCH],
                                                                          currentProfile->pidProfile.I8[PITCH],
                                                                          currentProfile->pidProfile.D8[PITCH]);
        BLACKBOX_PRINT_HEADER_LINE("yawPID:%d,%d,%d",                     currentProfile->pidProfile.P8[YAW],
                                                                          currentProfile->pidProfile.I8[YAW],
                                                                          currentProfile->pidProfile.D8[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("altPID:%d,%d,%d",                     currentProfile->pidProfile.P8[PIDALT],
                                                                          currentProfile->pidProfile.I8[PIDALT],
                                                                          currentProfile->pidProfile.D8[PIDALT]);
        BLACKBOX_PRINT_HEADER_LINE("posPID:%d,%d,%d",                     currentProfile->pidProfile.P8[PIDPOS],
                                                                          currentProfile->pidProfile.I8[PIDPOS],
                                                                          currentProfile->pidProfile.D8[PIDPOS]);
        BLACKBOX_PRINT_HEADER_LINE("posrPID:%d,%d,%d",                    currentProfile->pidProfile.P8[PIDPOSR],
                                                                          currentProfile->pidProfile.I8[PIDPOSR],
                                                                          currentProfile->pidProfile.D8[PIDPOSR]);
        BLACKBOX_PRINT_HEADER_LINE("navrPID:%d,%d,%d",                    currentProfile->pidProfile.P8[PIDNAVR],
                                                                          currentProfile->pidProfile.I8[PIDNAVR],
                                                                          currentProfile->pidProfile.D8[PIDNAVR]);
        BLACKBOX_PRINT_HEADER_LINE("levelPID:%d,%d,%d",                   currentProfile->pidProfile.P8[PIDLEVEL],
                                                                          currentProfile->pidProfile.I8[PIDLEVEL],
                                                                          currentProfile->pidProfile.D8[PIDLEVEL]);
        BLACKBOX_PRINT_HEADER_LINE("magPID:%d",                           currentProfile->pidProfile.P8[PIDMAG]);
        BLACKBOX_PRINT_HEADER_LINE("velPID:%d,%d,%d",                     currentProfile->pidProfile.P8[PIDVEL],
                                                                          currentProfile->pidProfile.I8[PIDVEL],
                                                                          currentProfile->pidProfile.D8[PIDVEL]);
        BLACKBOX_PRINT_HEADER_LINE("dterm_filter_type:%d",                currentProfile->pidProfile.dterm_filter_type);
        BLACKBOX_PRINT_HEADER_LINE("dterm_lpf_hz:%d",                     currentProfile->pidProfile.dterm_lpf_hz);
        BLACKBOX_PRINT_HEADER_LINE("yaw_lpf_hz:%d",                       currentProfile->pidProfile.yaw_lpf_hz);
        BLACKBOX_PRINT_HEADER_LINE("dterm_notch_hz:%d",                   currentProfile->pidProfile.dterm_notch_hz);
        BLACKBOX_PRINT_HEADER_LINE("dterm_notch_cutoff:%d",               currentProfile->pidProfile.dterm_notch_cutoff);
        BLACKBOX_PRINT_HEADER_LINE("rollPitchItermIgnoreRate:%d",         currentProfile->pidProfile.rollPitchItermIgnoreRate);
        BLACKBOX_PRINT_HEADER_LINE("yawItermIgnoreRate:%d",               currentProfile->pidProfile.yawItermIgnoreRate);
        BLACKBOX_PRINT_HEADER_LINE("yaw_p_limit:%d",                      currentProfile->pidProfile.yaw_p_limit);
        BLACKBOX_PRINT_HEADER_LINE("dterm_average_count:%d",              currentProfile->pidProfile.dterm_average_count);
        BLACKBOX_PRINT_HEADER_LINE("vbat_pid_compensation:%d",            currentProfile->pidProfile.vbatPidCompensation);
        BLACKBOX_PRINT_HEADER_LINE("pidAtMinThrottle:%d",                 currentProfile->pidProfile.pidAtMinThrottle);

        // Betaflight PID controller parameters. Outcommented is flash space saving
        //BLACKBOX_PRINT_HEADER_LINE("anti_gravity_threshold:%d",           currentProfile->pidProfile.itermThrottleThreshold);
        //BLACKBOX_PRINT_HEADER_LINE("anti_gravity_gain:%d",                castFloatBytesToInt(currentProfile->pidProfile.itermAcceleratorGain));
        BLACKBOX_PRINT_HEADER_LINE("setpointRelaxRatio:%d",               currentProfile->pidProfile.setpointRelaxRatio);
        BLACKBOX_PRINT_HEADER_LINE("dtermSetpointWeight:%d",              currentProfile->pidProfile.dtermSetpointWeight);
        //BLACKBOX_PRINT_HEADER_LINE("yawRateAccelLimit:%d",                castFloatBytesToInt(currentProfile->pidProfile.yawRateAccelLimit));
        //BLACKBOX_PRINT_HEADER_LINE("rateAccelLimit:%d",                   castFloatBytesToInt(currentProfile->pidProfile.rateAccelLimit));
        // End of Betaflight controller parameters

        BLACKBOX_PRINT_HEADER_LINE("deadband:%d",                         rcControlsConfig()->deadband);
        BLACKBOX_PRINT_HEADER_LINE("yaw_deadband:%d",                     rcControlsConfig()->yaw_deadband);
        BLACKBOX_PRINT_HEADER_LINE("gyro_lpf:%d",                         gyroConfig()->gyro_lpf);
        BLACKBOX_PRINT_HEADER_LINE("gyro_soft_type:%d",                   gyroConfig()->gyro_soft_lpf_type);
        BLACKBOX_PRINT_HEADER_LINE("gyro_lowpass_hz:%d",                  gyroConfig()->gyro_soft_lpf_hz);
        BLACKBOX_PRINT_HEADER_LINE("gyro_notch_hz:%d,%d",                 gyroConfig()->gyro_soft_notch_hz_1,
                                                                          gyroConfig()->gyro_soft_notch_hz_2);
        BLACKBOX_PRINT_HEADER_LINE("gyro_notch_cutoff:%d,%d",             gyroConfig()->gyro_soft_notch_cutoff_1,
                                                                          gyroConfig()->gyro_soft_notch_cutoff_2);
        BLACKBOX_PRINT_HEADER_LINE("acc_lpf_hz:%d",                 (int)(accelerometerConfig()->acc_lpf_hz * 100.0f));
        BLACKBOX_PRINT_HEADER_LINE("acc_hardware:%d",                     accelerometerConfig()->acc_hardware);
        BLACKBOX_PRINT_HEADER_LINE("baro_hardware:%d",                    barometerConfig()->baro_hardware);
        BLACKBOX_PRINT_HEADER_LINE("mag_hardware:%d",                     compassConfig()->mag_hardware);
        //BLACKBOX_PRINT_HEADER_LINE("gyro_cal_on_first_arm:%d",            armingConfig()->gyro_cal_on_first_arm);
        BLACKBOX_PRINT_HEADER_LINE("rc_interpolation:%d",                 rxConfig()->rcInterpolation);
        BLACKBOX_PRINT_HEADER_LINE("rc_interpolation_interval:%d",        rxConfig()->rcInterpolationInterval);
        BLACKBOX_PRINT_HEADER_LINE("airmode_activate_throttle:%d",        rxConfig()->airModeActivateThreshold);
        BLACKBOX_PRINT_HEADER_LINE("serialrx_provider:%d",                rxConfig()->serialrx_provider);
        BLACKBOX_PRINT_HEADER_LINE("unsynced_fast_pwm:%d",                motorConfig()->useUnsyncedPwm);
        BLACKBOX_PRINT_HEADER_LINE("fast_pwm_protocol:%d",                motorConfig()->motorPwmProtocol);
        BLACKBOX_PRINT_HEADER_LINE("motor_pwm_rate:%d",                   motorConfig()->motorPwmRate);
        BLACKBOX_PRINT_HEADER_LINE("digitalIdleOffset:%d",          (int)(motorConfig()->digitalIdleOffsetPercent * 100.0f));
        BLACKBOX_PRINT_HEADER_LINE("debug_mode:%d",                       masterConfig.debug_mode);
        BLACKBOX_PRINT_HEADER_LINE("features:%d",                         masterConfig.enabledFeatures);

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
    // Only allow events to be logged after headers have been written
    if (!(blackboxState == BLACKBOX_STATE_RUNNING || blackboxState == BLACKBOX_STATE_PAUSED)) {
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
        case FLIGHT_LOG_EVENT_FLIGHTMODE: // New flightmode flags write
            blackboxWriteUnsignedVB(data->flightMode.flags);
            blackboxWriteUnsignedVB(data->flightMode.lastFlags);
        break;
        case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
            if (data->inflightAdjustment.floatFlag) {
                blackboxWrite(data->inflightAdjustment.adjustmentFunction + FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG);
                blackboxWriteFloat(data->inflightAdjustment.newFloatValue);
            } else {
                blackboxWrite(data->inflightAdjustment.adjustmentFunction);
                blackboxWriteSignedVB(data->inflightAdjustment.newValue);
            }
        break;
        case FLIGHT_LOG_EVENT_LOGGING_RESUME:
            blackboxWriteUnsignedVB(data->loggingResume.logIteration);
            blackboxWriteUnsignedVB(data->loggingResume.currentTime);
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

/* monitor the flight mode event status and trigger an event record if the state changes */
static void blackboxCheckAndLogFlightMode()
{
    flightLogEvent_flightMode_t eventData; // Add new data for current flight mode flags

    // Use != so that we can still detect a change if the counter wraps
    if (rcModeActivationMask != blackboxLastFlightModeFlags) {
        eventData.lastFlags = blackboxLastFlightModeFlags;
        blackboxLastFlightModeFlags = rcModeActivationMask;
        eventData.flags = rcModeActivationMask;

        blackboxLogEvent(FLIGHT_LOG_EVENT_FLIGHTMODE, (flightLogEventData_t *) &eventData);
    }
}

/*
 * Use the user's num/denom settings to decide if the P-frame of the given index should be logged, allowing the user to control
 * the portion of logged loop iterations.
 */
static bool blackboxShouldLogPFrame(uint32_t pFrameIndex)
{
    /* Adding a magic shift of "blackboxConfig()->rate_num - 1" in here creates a better spread of
     * recorded / skipped frames when the I frame's position is considered:
     */
    return (pFrameIndex + blackboxConfig()->rate_num - 1) % blackboxConfig()->rate_denom < blackboxConfig()->rate_num;
}

static bool blackboxShouldLogIFrame() {
    return blackboxPFrameIndex == 0;
}

// Called once every FC loop in order to keep track of how many FC loop iterations have passed
static void blackboxAdvanceIterationTimers()
{
    blackboxSlowFrameIterationTimer++;
    blackboxIteration++;
    blackboxPFrameIndex++;

    if (blackboxPFrameIndex == BLACKBOX_I_INTERVAL) {
        blackboxPFrameIndex = 0;
        blackboxIFrameIndex++;
    }
}

// Called once every FC loop in order to log the current state
static void blackboxLogIteration(timeUs_t currentTimeUs)
{
    // Write a keyframe every BLACKBOX_I_INTERVAL frames so we can resynchronise upon missing frames
    if (blackboxShouldLogIFrame()) {
        /*
         * Don't log a slow frame if the slow data didn't change ("I" frames are already large enough without adding
         * an additional item to write at the same time). Unless we're *only* logging "I" frames, then we have no choice.
         */
        writeSlowFrameIfNeeded(blackboxIsOnlyLoggingIntraframes());

        loadMainState(currentTimeUs);
        writeIntraframe();
    } else {
        blackboxCheckAndLogArmingBeep();
        blackboxCheckAndLogFlightMode(); // Check for FlightMode status change event

        if (blackboxShouldLogPFrame(blackboxPFrameIndex)) {
            /*
             * We assume that slow frames are only interesting in that they aid the interpretation of the main data stream.
             * So only log slow frames during loop iterations where we log a main frame.
             */
            writeSlowFrameIfNeeded(true);

            loadMainState(currentTimeUs);
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
                writeGPSFrame(currentTimeUs);
            } else if (GPS_numSat != gpsHistory.GPS_numSat || GPS_coord[0] != gpsHistory.GPS_coord[0]
                    || GPS_coord[1] != gpsHistory.GPS_coord[1]) {
                //We could check for velocity changes as well but I doubt it changes independent of position
                writeGPSFrame(currentTimeUs);
            }
        }
#endif
    }

    //Flush every iteration so that our runtime variance is minimized
    blackboxDeviceFlush();
}

/**
 * Call each flight loop iteration to perform blackbox logging.
 */
void handleBlackbox(timeUs_t currentTimeUs)
{
    int i;

    if (blackboxState >= BLACKBOX_FIRST_HEADER_SENDING_STATE && blackboxState <= BLACKBOX_LAST_HEADER_SENDING_STATE) {
        blackboxReplenishHeaderBudget();
    }

    switch (blackboxState) {
        case BLACKBOX_STATE_PREPARE_LOG_FILE:
            if (blackboxDeviceBeginLog()) {
                blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
            }
        break;
        case BLACKBOX_STATE_SEND_HEADER:
            //On entry of this state, xmitState.headerIndex is 0 and startTime is intialised

            /*
             * Once the UART has had time to init, transmit the header in chunks so we don't overflow its transmit
             * buffer, overflow the OpenLog's buffer, or keep the main loop busy for too long.
             */
            if (millis() > xmitState.u.startTime + 100) {
                if (blackboxDeviceReserveBufferSpace(BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION) == BLACKBOX_RESERVE_SUCCESS) {
                    for (i = 0; i < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION && blackboxHeader[xmitState.headerIndex] != '\0'; i++, xmitState.headerIndex++) {
                        blackboxWrite(blackboxHeader[xmitState.headerIndex]);
                        blackboxHeaderBudget--;
                    }

                    if (blackboxHeader[xmitState.headerIndex] == '\0') {
                        blackboxSetState(BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER);
                    }
                }
            }
        break;
        case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
            //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
            if (!sendFieldDefinition('I', 'P', blackboxMainFields, blackboxMainFields + 1, ARRAY_LENGTH(blackboxMainFields),
                    &blackboxMainFields[0].condition, &blackboxMainFields[1].condition)) {
#ifdef GPS
                if (feature(FEATURE_GPS)) {
                    blackboxSetState(BLACKBOX_STATE_SEND_GPS_H_HEADER);
                } else
#endif
                    blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
            }
        break;
#ifdef GPS
        case BLACKBOX_STATE_SEND_GPS_H_HEADER:
            //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
            if (!sendFieldDefinition('H', 0, blackboxGpsHFields, blackboxGpsHFields + 1, ARRAY_LENGTH(blackboxGpsHFields),
                    NULL, NULL)) {
                blackboxSetState(BLACKBOX_STATE_SEND_GPS_G_HEADER);
            }
        break;
        case BLACKBOX_STATE_SEND_GPS_G_HEADER:
            //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
            if (!sendFieldDefinition('G', 0, blackboxGpsGFields, blackboxGpsGFields + 1, ARRAY_LENGTH(blackboxGpsGFields),
                    &blackboxGpsGFields[0].condition, &blackboxGpsGFields[1].condition)) {
                blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
            }
        break;
#endif
        case BLACKBOX_STATE_SEND_SLOW_HEADER:
            //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
            if (!sendFieldDefinition('S', 0, blackboxSlowFields, blackboxSlowFields + 1, ARRAY_LENGTH(blackboxSlowFields),
                    NULL, NULL)) {
                blackboxSetState(BLACKBOX_STATE_SEND_SYSINFO);
            }
        break;
        case BLACKBOX_STATE_SEND_SYSINFO:
            //On entry of this state, xmitState.headerIndex is 0

            //Keep writing chunks of the system info headers until it returns true to signal completion
            if (blackboxWriteSysinfo()) {

                /*
                 * Wait for header buffers to drain completely before data logging begins to ensure reliable header delivery
                 * (overflowing circular buffers causes all data to be discarded, so the first few logged iterations
                 * could wipe out the end of the header if we weren't careful)
                 */
                if (blackboxDeviceFlushForce()) {
                    blackboxSetState(BLACKBOX_STATE_RUNNING);
                }
            }
        break;
        case BLACKBOX_STATE_PAUSED:
            // Only allow resume to occur during an I-frame iteration, so that we have an "I" base to work from
            if (IS_RC_MODE_ACTIVE(BOXBLACKBOX) && blackboxShouldLogIFrame()) {
                // Write a log entry so the decoder is aware that our large time/iteration skip is intended
                flightLogEvent_loggingResume_t resume;

                resume.logIteration = blackboxIteration;
                resume.currentTime = currentTimeUs;

                blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME, (flightLogEventData_t *) &resume);
                blackboxSetState(BLACKBOX_STATE_RUNNING);

                blackboxLogIteration(currentTimeUs);
            }

            // Keep the logging timers ticking so our log iteration continues to advance
            blackboxAdvanceIterationTimers();
        break;
        case BLACKBOX_STATE_RUNNING:
            // On entry to this state, blackboxIteration, blackboxPFrameIndex and blackboxIFrameIndex are reset to 0
            // Prevent the Pausing of the log on the mode switch if in Motor Test Mode
            if (blackboxModeActivationConditionPresent && !IS_RC_MODE_ACTIVE(BOXBLACKBOX) && !startedLoggingInTestMode) {
                blackboxSetState(BLACKBOX_STATE_PAUSED);
            } else {
                blackboxLogIteration(currentTimeUs);
            }

            blackboxAdvanceIterationTimers();
        break;
        case BLACKBOX_STATE_SHUTTING_DOWN:
            //On entry of this state, startTime is set

            /*
             * Wait for the log we've transmitted to make its way to the logger before we release the serial port,
             * since releasing the port clears the Tx buffer.
             *
             * Don't wait longer than it could possibly take if something funky happens.
             */
            if (blackboxDeviceEndLog(blackboxLoggedAnyFrames) && (millis() > xmitState.u.startTime + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS || blackboxDeviceFlushForce())) {
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
        // ensure we reset the test mode flag if we stop due to full memory card
        if (startedLoggingInTestMode) startedLoggingInTestMode = false;
    } else { // Only log in test mode if there is room!

        if(blackboxConfig()->on_motor_test) {
            // Handle Motor Test Mode
            if(inMotorTestMode()) {
                if(blackboxState==BLACKBOX_STATE_STOPPED)
                    startInTestMode();
            } else {
                if(blackboxState!=BLACKBOX_STATE_STOPPED)
                    stopInTestMode();
            }
        }
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
