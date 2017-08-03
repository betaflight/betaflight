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

#include "blackbox.h"
#include "blackbox_encoding.h"
#include "blackbox_io.h"

#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/encoding.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/gps.h"

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/diagnostics.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"


#if defined(ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#else
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SERIAL
#endif

#ifdef SDCARD_DETECT_INVERTED
#define BLACKBOX_INTERVED_CARD_DETECTION 1
#else
#define BLACKBOX_INTERVED_CARD_DETECTION 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);

PG_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig,
    .device = DEFAULT_BLACKBOX_DEVICE,
    .rate_num = 1,
    .rate_denom = 1,
    .invertedCardDetection = BLACKBOX_INTERVED_CARD_DETECTION,
);

#define BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS 200
static const int32_t blackboxSInterval = 4096;

// Some macros to make writing FLIGHT_LOG_FIELD_* constants shorter:

#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED

static const char blackboxHeader[] =
    "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
    "H Data version:2\n";

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

#define BLACKBOX_DELTA_FIELD_HEADER_COUNT       ARRAYLEN(blackboxFieldHeaderNames)
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
    {"axisRate",    0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisRate",    1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisRate",    2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
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
#ifdef PITOT
    {"AirSpeed",   -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_PITOT},
#endif
#ifdef USE_RANGEFINDER
    {"surfaceRaw",   -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_SURFACE},
#endif
    {"rssi",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_RSSI},

    /* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
    {"gyroADC",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroADC",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroADC",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"attitude",    0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"attitude",    1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"attitude",    2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"debug",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), FLIGHT_LOG_FIELD_CONDITION_DEBUG},
    {"debug",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), FLIGHT_LOG_FIELD_CONDITION_DEBUG},
    {"debug",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), FLIGHT_LOG_FIELD_CONDITION_DEBUG},
    {"debug",       3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), FLIGHT_LOG_FIELD_CONDITION_DEBUG},
    /* Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones): */
    {"motor",       0, UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_1)},
    /* Subsequent motors base their I-frame values on the first one, P-frame values on the average of last two frames: */
    {"motor",       1, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_2)},
    {"motor",       2, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_3)},
    {"motor",       3, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_4)},
    {"motor",       4, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_5)},
    {"motor",       5, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_6)},
    {"motor",       6, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_7)},
    {"motor",       7, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_8)},

    /* Tricopter tail servo */
    {"servo",       5, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(TRICOPTER)},

#ifdef NAV_BLACKBOX
    {"navState",  -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navFlags",  -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navEPH",    -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navEPV",    -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navPos",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navPos",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navPos",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navVel",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navVel",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navVel",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navAcc",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navAcc",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navAcc",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navTgtVel",  0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navTgtVel",  1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navTgtVel",  2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navTgtPos",  0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navTgtPos",  1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navTgtPos",  2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navSurf",    0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"navTgtSurf", 0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
#endif
};

#ifdef GPS
// GPS position/vel frame
static const blackboxConditionalFieldDefinition_t blackboxGpsGFields[] = {
    {"time",              -1, UNSIGNED, PREDICT(LAST_MAIN_FRAME_TIME), ENCODING(UNSIGNED_VB), CONDITION(NOT_LOGGING_EVERY_FRAME)},
    {"GPS_fixType",       -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_numSat",        -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_coord",          0, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_coord",          1, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_altitude",      -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_speed",         -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_ground_course", -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_hdop",          -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_eph",           -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_epv",           -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)}
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
    {"rxFlightChannelsValid", -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},

    {"hwHealthStatus",        -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},
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

    int32_t axisPID_P[XYZ_AXIS_COUNT];
    int32_t axisPID_I[XYZ_AXIS_COUNT];
    int32_t axisPID_D[XYZ_AXIS_COUNT];
    int32_t axisPID_Setpoint[XYZ_AXIS_COUNT];

    int16_t rcCommand[4];
    int16_t gyroADC[XYZ_AXIS_COUNT];
    int16_t accADC[XYZ_AXIS_COUNT];
    int16_t attitude[XYZ_AXIS_COUNT];
    int16_t debug[DEBUG16_VALUE_COUNT];
    int16_t motor[MAX_SUPPORTED_MOTORS];
    int16_t servo[MAX_SUPPORTED_SERVOS];

    uint16_t vbatLatest;
    uint16_t amperageLatest;

#ifdef BARO
    int32_t BaroAlt;
#endif
#ifdef PITOT
    int32_t airSpeed;
#endif
#ifdef MAG
    int16_t magADC[XYZ_AXIS_COUNT];
#endif
#ifdef USE_RANGEFINDER
    int32_t surfaceRaw;
#endif
    uint16_t rssi;
#ifdef NAV_BLACKBOX
    int16_t navState;
    uint16_t navFlags;
    uint16_t navEPH;
    uint16_t navEPV;
    int32_t navPos[XYZ_AXIS_COUNT];
    int16_t navRealVel[XYZ_AXIS_COUNT];
    int16_t navAccNEU[XYZ_AXIS_COUNT];
    int16_t navTargetVel[XYZ_AXIS_COUNT];
    int16_t navTargetPos[XYZ_AXIS_COUNT];
    int16_t navHeading;
    int16_t navTargetHeading;
    int16_t navSurface;
    int16_t navTargetSurface;
#endif
} blackboxMainState_t;

typedef struct blackboxGpsState_s {
    int32_t GPS_home[2];
    int32_t GPS_coord[2];
    uint8_t GPS_numSat;
} blackboxGpsState_t;

// This data is updated really infrequently:
typedef struct blackboxSlowState_s {
    uint32_t flightModeFlags; // extend this data size (from uint16_t)
    uint8_t stateFlags;
    uint8_t failsafePhase;
    bool rxSignalReceived;
    bool rxFlightChannelsValid;
    int32_t hwHealthStatus;
} __attribute__((__packed__)) blackboxSlowState_t; // We pack this struct so that padding doesn't interfere with memcmp()

//From rc_controls.c
extern boxBitmask_t rcModeActivationMask;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;

static uint32_t blackboxLastArmingBeep = 0;
static uint32_t blackboxLastFlightModeFlags = 0;

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

STATIC_ASSERT((sizeof(blackboxConditionCache) * 8) >= FLIGHT_LOG_FIELD_CONDITION_LAST, too_many_flight_log_conditions);

static uint32_t blackboxIFrameInterval;
static uint32_t blackboxIteration;
static uint16_t blackboxPFrameIndex;
static uint16_t blackboxIFrameIndex;
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
 * Return true if it is safe to edit the Blackbox configuration.
 */
bool blackboxMayEditConfig(void)
{
    return blackboxState <= BLACKBOX_STATE_STOPPED;
}

static bool blackboxIsOnlyLoggingIntraframes(void)
{
    return blackboxConfig()->rate_num == 1 && blackboxConfig()->rate_denom == blackboxIFrameInterval;
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
        return pidBank()->pid[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0].D != 0;

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

    case FLIGHT_LOG_FIELD_CONDITION_PITOT:
#ifdef PITOT
        return sensors(SENSOR_PITOT);
#else
        return false;
#endif

    case FLIGHT_LOG_FIELD_CONDITION_VBAT:
        return feature(FEATURE_VBAT);

    case FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC:
        return feature(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC;

    case FLIGHT_LOG_FIELD_CONDITION_SURFACE:
#ifdef USE_RANGEFINDER
        return sensors(SENSOR_RANGEFINDER);
#else
        return false;
#endif

    case FLIGHT_LOG_FIELD_CONDITION_RSSI:
        return rxConfig()->rssi_channel > 0 || feature(FEATURE_RSSI_ADC);

    case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
        return blackboxConfig()->rate_num < blackboxConfig()->rate_denom;

    case FLIGHT_LOG_FIELD_CONDITION_DEBUG:
        return debugMode != DEBUG_NONE;

    case FLIGHT_LOG_FIELD_CONDITION_NEVER:
        return false;

    default:
        return false;
    }
}

static void blackboxBuildConditionCache(void)
{
    blackboxConditionCache = 0;
    for (FlightLogFieldCondition cond = FLIGHT_LOG_FIELD_CONDITION_FIRST; cond <= FLIGHT_LOG_FIELD_CONDITION_LAST; cond++) {
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
        blackboxSlowFrameIterationTimer = blackboxSInterval; //Force a slow frame to be written on the first iteration
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

    blackboxWriteSignedVBArray(blackboxCurrent->axisPID_Setpoint, XYZ_AXIS_COUNT);
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

#ifdef PITOT
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_PITOT)) {
        blackboxWriteSignedVB(blackboxCurrent->airSpeed);
    }
#endif

#ifdef USE_RANGEFINDER
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SURFACE)) {
        blackboxWriteSignedVB(blackboxCurrent->surfaceRaw);
    }
#endif

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        blackboxWriteUnsignedVB(blackboxCurrent->rssi);
    }

    blackboxWriteSigned16VBArray(blackboxCurrent->gyroADC, XYZ_AXIS_COUNT);
    blackboxWriteSigned16VBArray(blackboxCurrent->accADC, XYZ_AXIS_COUNT);
    blackboxWriteSigned16VBArray(blackboxCurrent->attitude, XYZ_AXIS_COUNT);

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG)) {
        blackboxWriteSigned16VBArray(blackboxCurrent->debug, DEBUG16_VALUE_COUNT);
    }

    //Motors can be below minthrottle when disarmed, but that doesn't happen much
    blackboxWriteUnsignedVB(blackboxCurrent->motor[0] - motorConfig()->minthrottle);

    //Motors tend to be similar to each other so use the first motor's value as a predictor of the others
    const int motorCount = getMotorCount();
    for (int x = 1; x < motorCount; x++) {
        blackboxWriteSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);
    }

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
        //Assume the tail spends most of its time around the center
        blackboxWriteSignedVB(blackboxCurrent->servo[5] - 1500);
    }

#ifdef NAV_BLACKBOX
    blackboxWriteSignedVB(blackboxCurrent->navState);

    blackboxWriteSignedVB(blackboxCurrent->navFlags);
    blackboxWriteSignedVB(blackboxCurrent->navEPH);
    blackboxWriteSignedVB(blackboxCurrent->navEPV);

    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->navPos[x]);
    }

    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->navRealVel[x]);
    }

    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->navAccNEU[x]);
    }

    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->navTargetVel[x]);
    }

    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->navTargetPos[x]);
    }

    blackboxWriteSignedVB(blackboxCurrent->navSurface);
    blackboxWriteSignedVB(blackboxCurrent->navTargetSurface);
#endif

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
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    blackboxMainState_t *blackboxLast = blackboxHistory[1];

    blackboxWrite('P');

    //No need to store iteration count since its delta is always 1

    /*
     * Since the difference between the difference between successive times will be nearly zero (due to consistent
     * looptime spacing), use second-order differences.
     */
    blackboxWriteSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    int32_t deltas[8];
    arraySubInt32(deltas, blackboxCurrent->axisPID_Setpoint, blackboxLast->axisPID_Setpoint, XYZ_AXIS_COUNT);
    blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);

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
    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
            blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x] - blackboxLast->axisPID_D[x]);
        }
    }

    /*
     * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
     * can pack multiple values per byte:
     */
    for (int x = 0; x < 4; x++) {
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
        for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
            deltas[optionalFieldCount++] = blackboxCurrent->magADC[x] - blackboxLast->magADC[x];
        }
    }
#endif

#ifdef BARO
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO)) {
        deltas[optionalFieldCount++] = blackboxCurrent->BaroAlt - blackboxLast->BaroAlt;
    }
#endif

#ifdef PITOT
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_PITOT)) {
        deltas[optionalFieldCount++] = blackboxCurrent->airSpeed - blackboxLast->airSpeed;
    }
#endif

#ifdef USE_RANGEFINDER
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SURFACE)) {
        deltas[optionalFieldCount++] = blackboxCurrent->surfaceRaw - blackboxLast->surfaceRaw;
    }
#endif

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->rssi - blackboxLast->rssi;
    }

    blackboxWriteTag8_8SVB(deltas, optionalFieldCount);

    //Since gyros, accs and motors are noisy, base their predictions on the average of the history:
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC),   XYZ_AXIS_COUNT);
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accADC), XYZ_AXIS_COUNT);
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, attitude), XYZ_AXIS_COUNT);
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG)) {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, debug), DEBUG16_VALUE_COUNT);
    }
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, motor),     getMotorCount());

    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
        blackboxWriteSignedVB(blackboxCurrent->servo[5] - blackboxLast->servo[5]);
    }

#ifdef NAV_BLACKBOX
    blackboxWriteSignedVB(blackboxCurrent->navState - blackboxLast->navState);

    blackboxWriteSignedVB(blackboxCurrent->navFlags - blackboxLast->navFlags);
    blackboxWriteSignedVB(blackboxCurrent->navEPH - blackboxLast->navEPH);
    blackboxWriteSignedVB(blackboxCurrent->navEPV - blackboxLast->navEPV);

    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxCurrent->navPos[x] - blackboxLast->navPos[x]);
    }

    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxHistory[0]->navRealVel[x] - (blackboxHistory[1]->navRealVel[x] + blackboxHistory[2]->navRealVel[x]) / 2);
    }

    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxHistory[0]->navAccNEU[x] - (blackboxHistory[1]->navAccNEU[x] + blackboxHistory[2]->navAccNEU[x]) / 2);
    }

    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxHistory[0]->navTargetVel[x] - (blackboxHistory[1]->navTargetVel[x] + blackboxHistory[2]->navTargetVel[x]) / 2);
    }

    for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
        blackboxWriteSignedVB(blackboxHistory[0]->navTargetPos[x] - (blackboxHistory[1]->navTargetPos[x] + blackboxHistory[2]->navTargetPos[x]) / 2);
    }

    blackboxWriteSignedVB(blackboxCurrent->navSurface - blackboxLast->navSurface);
    blackboxWriteSignedVB(blackboxCurrent->navTargetSurface - blackboxLast->navTargetSurface);
#endif

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

    blackboxWriteUnsignedVB(slowHistory.hwHealthStatus);

    blackboxSlowFrameIterationTimer = 0;
}

/**
 * Load rarely-changing values from the FC into the given structure
 */
static void loadSlowState(blackboxSlowState_t *slow)
{
    memcpy(&slow->flightModeFlags, &rcModeActivationMask, sizeof(slow->flightModeFlags)); //was flightModeFlags;
    slow->stateFlags = stateFlags;
    slow->failsafePhase = failsafePhase();
    slow->rxSignalReceived = rxIsReceivingSignal();
    slow->rxFlightChannelsValid = rxAreFlightChannelsValid();
    slow->hwHealthStatus = (getHwGyroStatus()           << 2 * 0) |     // Pack hardware health status into a bit field.
                           (getHwAccelerometerStatus()  << 2 * 1) |     // Use raw hardwareSensorStatus_e values and pack them using 2 bits per value
                           (getHwCompassStatus()        << 2 * 2) |     // Report GYRO in 2 lowest bits, then ACC, COMPASS, BARO, GPS, RANGEFINDER and PITOT
                           (getHwBarometerStatus()      << 2 * 3) |
                           (getHwGPSStatus()            << 2 * 4) |
                           (getHwRangefinderStatus()    << 2 * 5) |
                           (getHwPitotmeterStatus()     << 2 * 6);
}

/**
 * If the data in the slow frame has changed, log a slow frame.
 *
 * If allowPeriodicWrite is true, the frame is also logged if it has been more than blackboxSInterval logging iterations
 * since the field was last logged.
 */
static bool writeSlowFrameIfNeeded(bool allowPeriodicWrite)
{
    // Write the slow frame peridocially so it can be recovered if we ever lose sync
    bool shouldWrite = allowPeriodicWrite && blackboxSlowFrameIterationTimer >= blackboxSInterval;

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
    return shouldWrite;
}

static void blackboxValidateConfig(void)
{
    if (blackboxConfig()->rate_num == 0 || blackboxConfig()->rate_denom == 0
            || blackboxConfig()->rate_num >= blackboxConfig()->rate_denom) {
        blackboxConfigMutable()->rate_num = 1;
        blackboxConfigMutable()->rate_denom = 1;
    } else {
        /* Reduce the fraction the user entered as much as possible (makes the recorded/skipped frame pattern repeat
         * itself more frequently)
         */
        const int div = gcd(blackboxConfig()->rate_num, blackboxConfig()->rate_denom);

        blackboxConfigMutable()->rate_num /= div;
        blackboxConfigMutable()->rate_denom /= div;
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
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_SERIAL;
    }
}

static void blackboxResetIterationTimers(void)
{
    blackboxIteration = 0;
    blackboxPFrameIndex = 0;
    blackboxIFrameIndex = 0;
}

/**
 * Start Blackbox logging if it is not already running. Intended to be called upon arming.
 */
void blackboxStart(void)
{
    if (blackboxState != BLACKBOX_STATE_STOPPED) {
        return;
    }

    blackboxValidateConfig();

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

    blackboxModeActivationConditionPresent = isModeActivationConditionPresent(BOXBLACKBOX);

    blackboxResetIterationTimers();

    /*
     * Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
     * it finally plays the beep for this arming event.
     */
    blackboxLastArmingBeep = getArmingBeepTimeMicros();
    memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask, sizeof(blackboxLastFlightModeFlags)); // record startup status

    blackboxSetState(BLACKBOX_STATE_PREPARE_LOG_FILE);
}

/**
 * Begin Blackbox shutdown.
 */
void blackboxFinish(void)
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

#ifdef GPS
static void writeGPSHomeFrame(void)
{
    blackboxWrite('H');

    blackboxWriteSignedVB(GPS_home.lat);
    blackboxWriteSignedVB(GPS_home.lon);
    //TODO it'd be great if we could grab the GPS current time and write that too

    gpsHistory.GPS_home[0] = GPS_home.lat;
    gpsHistory.GPS_home[1] = GPS_home.lon;
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

    blackboxWriteUnsignedVB(gpsSol.fixType);
    blackboxWriteUnsignedVB(gpsSol.numSat);
    blackboxWriteSignedVB(gpsSol.llh.lat - gpsHistory.GPS_home[0]);
    blackboxWriteSignedVB(gpsSol.llh.lon - gpsHistory.GPS_home[1]);
    blackboxWriteUnsignedVB(gpsSol.llh.alt / 100); // meters
    blackboxWriteUnsignedVB(gpsSol.groundSpeed);
    blackboxWriteUnsignedVB(gpsSol.groundCourse);
    blackboxWriteUnsignedVB(gpsSol.hdop);
    blackboxWriteUnsignedVB(gpsSol.eph);
    blackboxWriteUnsignedVB(gpsSol.epv);

    gpsHistory.GPS_numSat = gpsSol.numSat;
    gpsHistory.GPS_coord[0] = gpsSol.llh.lat;
    gpsHistory.GPS_coord[1] = gpsSol.llh.lon;
}
#endif

/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
static void loadMainState(timeUs_t currentTimeUs)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

    blackboxCurrent->time = currentTimeUs;

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_Setpoint[i] = axisPID_Setpoint[i];
        blackboxCurrent->axisPID_P[i] = axisPID_P[i];
        blackboxCurrent->axisPID_I[i] = axisPID_I[i];
        blackboxCurrent->axisPID_D[i] = axisPID_D[i];
        blackboxCurrent->gyroADC[i] = lrintf(gyro.gyroADCf[i]);
        blackboxCurrent->accADC[i] = acc.accADC[i];
#ifdef MAG
        blackboxCurrent->magADC[i] = mag.magADC[i];
#endif
    }

    for (int i = 0; i < 4; i++) {
        blackboxCurrent->rcCommand[i] = rcCommand[i];
    }

    blackboxCurrent->attitude[0] = attitude.values.roll;
    blackboxCurrent->attitude[1] = attitude.values.pitch;
    blackboxCurrent->attitude[2] = attitude.values.yaw;

    for (int i = 0; i < DEBUG16_VALUE_COUNT; i++) {
        blackboxCurrent->debug[i] = debug[i];
    }

    const int motorCount = getMotorCount();
    for (int i = 0; i < motorCount; i++) {
        blackboxCurrent->motor[i] = motor[i];
    }

    blackboxCurrent->vbatLatest = vbatLatestADC;
    blackboxCurrent->amperageLatest = amperageLatestADC;

#ifdef BARO
    blackboxCurrent->BaroAlt = baro.BaroAlt;
#endif

#ifdef PITOT
    blackboxCurrent->airSpeed = pitot.airSpeed;
#endif

#ifdef USE_RANGEFINDER
    // Store the raw rangefinder surface readout without applying tilt correction
    blackboxCurrent->surfaceRaw = rangefinderGetLatestRawAltitude();
#endif

    blackboxCurrent->rssi = rssi;

#ifdef USE_SERVOS
    //Tail servo for tricopters
    blackboxCurrent->servo[5] = servo[5];
#endif

#ifdef NAV_BLACKBOX
    blackboxCurrent->navState = navCurrentState;
    blackboxCurrent->navFlags = navFlags;
    blackboxCurrent->navEPH = navEPH;
    blackboxCurrent->navEPV = navEPV;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->navPos[i] = navLatestActualPosition[i];
        blackboxCurrent->navRealVel[i] = navActualVelocity[i];
        blackboxCurrent->navAccNEU[i] = navAccNEU[i];
        blackboxCurrent->navTargetVel[i] = navDesiredVelocity[i];
        blackboxCurrent->navTargetPos[i] = navTargetPosition[i];
    }
    blackboxCurrent->navSurface = navActualSurface;
    blackboxCurrent->navTargetSurface = navTargetSurface;
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
#define BLACKBOX_PRINT_HEADER_LINE(name, format, ...) case __COUNTER__: \
                                                blackboxPrintfHeaderLine(name, format, __VA_ARGS__); \
                                                break;
#define BLACKBOX_PRINT_HEADER_LINE_CUSTOM(...) case __COUNTER__: \
                                                    {__VA_ARGS__}; \
                                               break;
#endif

/**
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
static bool blackboxWriteSysinfo(void)
{
    // Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
    if (blackboxDeviceReserveBufferSpace(64) != BLACKBOX_RESERVE_SUCCESS) {
        return false;
    }

    switch (xmitState.headerIndex) {
        BLACKBOX_PRINT_HEADER_LINE("Firmware type", "%s",                   "Cleanflight");
        BLACKBOX_PRINT_HEADER_LINE("Firmware revision", "INAV %s (%s) %s",  FC_VERSION_STRING, shortGitRevision, targetName);
        BLACKBOX_PRINT_HEADER_LINE("Firmware date", "%s %s",                buildDate, buildTime);
        BLACKBOX_PRINT_HEADER_LINE("Log start datetime", "%s",              blackboxGetStartDateTime());
        BLACKBOX_PRINT_HEADER_LINE("Craft name", "%s",                      systemConfig()->name);
        BLACKBOX_PRINT_HEADER_LINE("P interval", "%d/%d",                   blackboxConfig()->rate_num, blackboxConfig()->rate_denom);
        BLACKBOX_PRINT_HEADER_LINE("minthrottle", "%d",                     motorConfig()->minthrottle);
        BLACKBOX_PRINT_HEADER_LINE("maxthrottle", "%d",                     motorConfig()->maxthrottle);
        BLACKBOX_PRINT_HEADER_LINE("gyro_scale", "0x%x",                    castFloatBytesToInt(1.0f));
        BLACKBOX_PRINT_HEADER_LINE("motorOutput", "%d,%d",                  motorConfig()->minthrottle,motorConfig()->maxthrottle);
        BLACKBOX_PRINT_HEADER_LINE("acc_1G", "%u",                          acc.dev.acc_1G);

        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
                blackboxPrintfHeaderLine("vbat_scale", "%u", batteryConfig()->vbatscale);
            } else {
                xmitState.headerIndex += 2; // Skip the next two vbat fields too
            }
            );
        BLACKBOX_PRINT_HEADER_LINE("vbatcellvoltage", "%u,%u,%u",           batteryConfig()->vbatmincellvoltage,
                                                                            batteryConfig()->vbatwarningcellvoltage,
                                                                            batteryConfig()->vbatmaxcellvoltage);
        BLACKBOX_PRINT_HEADER_LINE("vbatref", "%u",                         vbatReference);

        BLACKBOX_PRINT_HEADER_LINE_CUSTOM(
            //Note: Log even if this is a virtual current meter, since the virtual meter uses these parameters too:
            if (feature(FEATURE_CURRENT_METER)) {
                blackboxPrintfHeaderLine("currentMeter", "%d,%d",           batteryConfig()->currentMeterOffset,
                                                                            batteryConfig()->currentMeterScale);
            }
            );

        BLACKBOX_PRINT_HEADER_LINE("looptime", "%d",                        getPidUpdateRate());
        BLACKBOX_PRINT_HEADER_LINE("gyro_sync_denom", "%d",                 gyroConfig()->gyroSyncDenominator);
        BLACKBOX_PRINT_HEADER_LINE("rc_rate", "%d",                         100); //For compatibility reasons write rc_rate 100
        BLACKBOX_PRINT_HEADER_LINE("rc_expo", "%d",                         currentControlRateProfile->rcExpo8);
        BLACKBOX_PRINT_HEADER_LINE("rc_yaw_expo", "%d",                     currentControlRateProfile->rcYawExpo8);
        BLACKBOX_PRINT_HEADER_LINE("thr_mid", "%d",                         currentControlRateProfile->thrMid8);
        BLACKBOX_PRINT_HEADER_LINE("thr_expo", "%d",                        currentControlRateProfile->thrExpo8);
        BLACKBOX_PRINT_HEADER_LINE("tpa_rate", "%d",                        currentControlRateProfile->dynThrPID);
        BLACKBOX_PRINT_HEADER_LINE("tpa_breakpoint", "%d",                  currentControlRateProfile->tpa_breakpoint);
        BLACKBOX_PRINT_HEADER_LINE("rates", "%d,%d,%d",                     currentControlRateProfile->rates[ROLL],
                                                                            currentControlRateProfile->rates[PITCH],
                                                                            currentControlRateProfile->rates[YAW]);
        BLACKBOX_PRINT_HEADER_LINE("rollPID", "%d,%d,%d",                   pidBank()->pid[PID_ROLL].P,
                                                                            pidBank()->pid[PID_ROLL].I,
                                                                            pidBank()->pid[PID_ROLL].D);
        BLACKBOX_PRINT_HEADER_LINE("pitchPID", "%d,%d,%d",                  pidBank()->pid[PID_PITCH].P,
                                                                            pidBank()->pid[PID_PITCH].I,
                                                                            pidBank()->pid[PID_PITCH].D);
        BLACKBOX_PRINT_HEADER_LINE("yawPID", "%d,%d,%d",                    pidBank()->pid[PID_YAW].P,
                                                                            pidBank()->pid[PID_YAW].I,
                                                                            pidBank()->pid[PID_YAW].D);
        BLACKBOX_PRINT_HEADER_LINE("altPID", "%d,%d,%d",                    pidBank()->pid[PID_POS_Z].P,
                                                                            pidBank()->pid[PID_POS_Z].I,
                                                                            pidBank()->pid[PID_POS_Z].D);
        BLACKBOX_PRINT_HEADER_LINE("posPID", "%d,%d,%d",                    pidBank()->pid[PID_POS_XY].P,
                                                                            pidBank()->pid[PID_POS_XY].I,
                                                                            pidBank()->pid[PID_POS_XY].D);
        BLACKBOX_PRINT_HEADER_LINE("posrPID", "%d,%d,%d",                   pidBank()->pid[PID_VEL_XY].P,
                                                                            pidBank()->pid[PID_VEL_XY].I,
                                                                            pidBank()->pid[PID_VEL_XY].D);
        BLACKBOX_PRINT_HEADER_LINE("levelPID", "%d,%d,%d",                  pidBank()->pid[PID_LEVEL].P,
                                                                            pidBank()->pid[PID_LEVEL].I,
                                                                            pidBank()->pid[PID_LEVEL].D);
        BLACKBOX_PRINT_HEADER_LINE("magPID", "%d",                          pidBank()->pid[PID_HEADING].P);
        BLACKBOX_PRINT_HEADER_LINE("velPID", "%d,%d,%d",                    pidBank()->pid[PID_VEL_Z].P,
                                                                            pidBank()->pid[PID_VEL_Z].I,
                                                                            pidBank()->pid[PID_VEL_Z].D);
        BLACKBOX_PRINT_HEADER_LINE("yaw_p_limit", "%d",                     pidProfile()->yaw_p_limit);
        BLACKBOX_PRINT_HEADER_LINE("yaw_lpf_hz", "%d",                      pidProfile()->yaw_lpf_hz);
        BLACKBOX_PRINT_HEADER_LINE("dterm_lpf_hz", "%d",                    pidProfile()->dterm_lpf_hz);
        BLACKBOX_PRINT_HEADER_LINE("dterm_notch_hz", "%d",                  pidProfile()->dterm_soft_notch_hz);
        BLACKBOX_PRINT_HEADER_LINE("dterm_notch_cutoff", "%d",              pidProfile()->dterm_soft_notch_cutoff);
        BLACKBOX_PRINT_HEADER_LINE("deadband", "%d",                        rcControlsConfig()->deadband);
        BLACKBOX_PRINT_HEADER_LINE("yaw_deadband", "%d",                    rcControlsConfig()->yaw_deadband);
        BLACKBOX_PRINT_HEADER_LINE("gyro_lpf", "%d",                        gyroConfig()->gyro_lpf);
        BLACKBOX_PRINT_HEADER_LINE("gyro_lpf_hz", "%d",                     gyroConfig()->gyro_soft_lpf_hz);
        BLACKBOX_PRINT_HEADER_LINE("gyro_notch_hz", "%d,%d",                gyroConfig()->gyro_soft_notch_hz_1,
                                                                            gyroConfig()->gyro_soft_notch_hz_2);
        BLACKBOX_PRINT_HEADER_LINE("gyro_notch_cutoff", "%d,%d",            gyroConfig()->gyro_soft_notch_cutoff_1,
                                                                            gyroConfig()->gyro_soft_notch_cutoff_2);
        BLACKBOX_PRINT_HEADER_LINE("acc_lpf_hz", "%d",                      pidProfile()->acc_soft_lpf_hz);
        BLACKBOX_PRINT_HEADER_LINE("acc_hardware", "%d",                    accelerometerConfig()->acc_hardware);
        BLACKBOX_PRINT_HEADER_LINE("baro_hardware", "%d",                   barometerConfig()->baro_hardware);
        BLACKBOX_PRINT_HEADER_LINE("mag_hardware", "%d",                    compassConfig()->mag_hardware);
        BLACKBOX_PRINT_HEADER_LINE("serialrx_provider", "%d",               rxConfig()->serialrx_provider);
        BLACKBOX_PRINT_HEADER_LINE("motor_pwm_protocol", "%d",              motorConfig()->motorPwmProtocol);
        BLACKBOX_PRINT_HEADER_LINE("motor_pwm_rate", "%d",                  motorConfig()->motorPwmRate);
        BLACKBOX_PRINT_HEADER_LINE("debug_mode", "%d",                      systemConfig()->debug_mode);
        BLACKBOX_PRINT_HEADER_LINE("features", "%d",                        featureConfig()->enabledFeatures);

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
        blackboxWriteUnsignedVB(data->loggingResume.currentTimeUs);
        break;
    case FLIGHT_LOG_EVENT_IMU_FAILURE:
        blackboxWrite(0);
        break;
    case FLIGHT_LOG_EVENT_LOG_END:
        blackboxPrintf("End of log (disarm reason:%d)", getDisarmReason());
        blackboxWrite(0);
        break;
    }
}

/* If an arming beep has played since it was last logged, write the time of the arming beep to the log as a synchronization point */
static void blackboxCheckAndLogArmingBeep(void)
{
    // Use != so that we can still detect a change if the counter wraps
    if (getArmingBeepTimeMicros() != blackboxLastArmingBeep) {
        blackboxLastArmingBeep = getArmingBeepTimeMicros();
        flightLogEvent_syncBeep_t eventData;
        eventData.time = blackboxLastArmingBeep;
        blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP, (flightLogEventData_t *) &eventData);
    }
}

/* monitor the flight mode event status and trigger an event record if the state changes */
static void blackboxCheckAndLogFlightMode(void)
{
    // Use != so that we can still detect a change if the counter wraps
    if (memcmp(&rcModeActivationMask, &blackboxLastFlightModeFlags, sizeof(blackboxLastFlightModeFlags))) {
        flightLogEvent_flightMode_t eventData; // Add new data for current flight mode flags
        eventData.lastFlags = blackboxLastFlightModeFlags;
        memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask, sizeof(blackboxLastFlightModeFlags));
        memcpy(&eventData.flags, &rcModeActivationMask, sizeof(eventData.flags));
        blackboxLogEvent(FLIGHT_LOG_EVENT_FLIGHTMODE, (flightLogEventData_t *)&eventData);
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

static bool blackboxShouldLogIFrame(void)
{
    return blackboxPFrameIndex == 0;
}

// Called once every FC loop in order to keep track of how many FC loop iterations have passed
static void blackboxAdvanceIterationTimers(void)
{
    blackboxSlowFrameIterationTimer++;
    blackboxIteration++;
    blackboxPFrameIndex++;

    if (blackboxPFrameIndex == blackboxIFrameInterval) {
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
        blackboxCheckAndLogFlightMode();

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
            if (GPS_home.lat != gpsHistory.GPS_home[0] || GPS_home.lon != gpsHistory.GPS_home[1]
                || (blackboxPFrameIndex == (blackboxIFrameInterval / 2) && blackboxIFrameIndex % 128 == 0)) {

                writeGPSHomeFrame();
                writeGPSFrame(currentTimeUs);
            } else if (gpsSol.numSat != gpsHistory.GPS_numSat || gpsSol.llh.lat != gpsHistory.GPS_coord[0]
                    || gpsSol.llh.lon != gpsHistory.GPS_coord[1]) {
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
void blackboxUpdate(timeUs_t currentTimeUs)
{
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
                for (int i = 0; i < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION && blackboxHeader[xmitState.headerIndex] != '\0'; i++, xmitState.headerIndex++) {
                    blackboxWrite(blackboxHeader[xmitState.headerIndex]);
                    blackboxHeaderBudget--;
                }

                if (blackboxHeader[xmitState.headerIndex] == '\0') {
                    blackboxPrintfHeaderLine("I interval", "%d", blackboxIFrameInterval);
                    blackboxSetState(BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER);
                }
            }
        }
        break;
    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('I', 'P', blackboxMainFields, blackboxMainFields + 1, ARRAYLEN(blackboxMainFields),
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
        if (!sendFieldDefinition('H', 0, blackboxGpsHFields, blackboxGpsHFields + 1, ARRAYLEN(blackboxGpsHFields),
                NULL, NULL)) {
            blackboxSetState(BLACKBOX_STATE_SEND_GPS_G_HEADER);
        }
        break;
    case BLACKBOX_STATE_SEND_GPS_G_HEADER:
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('G', 0, blackboxGpsGFields, blackboxGpsGFields + 1, ARRAYLEN(blackboxGpsGFields),
                &blackboxGpsGFields[0].condition, &blackboxGpsGFields[1].condition)) {
            blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
        }
        break;
#endif
    case BLACKBOX_STATE_SEND_SLOW_HEADER:
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('S', 0, blackboxSlowFields, blackboxSlowFields + 1, ARRAYLEN(blackboxSlowFields),
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
            resume.currentTimeUs = currentTimeUs;

            blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME, (flightLogEventData_t *) &resume);
            blackboxSetState(BLACKBOX_STATE_RUNNING);

            blackboxLogIteration(currentTimeUs);
        }
        // Keep the logging timers ticking so our log iteration continues to advance
        blackboxAdvanceIterationTimers();
        break;
    case BLACKBOX_STATE_RUNNING:
        // On entry to this state, blackboxIteration, blackboxPFrameIndex and blackboxIFrameIndex are reset to 0
        if (blackboxModeActivationConditionPresent && !IS_RC_MODE_ACTIVE(BOXBLACKBOX)) {
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
    }
}

/*
 * Returns start time in ISO 8601 format, YYYY-MM-DDThh:mm:ss
 * Year value of "0000" indicates time not set
 */
static char startDateTime[20] = "0000-01-01T00:00:00";
const char *blackboxGetStartDateTime(void)
{
    return startDateTime;
}

void blackboxSetStartDateTime(const char *dateTime, timeMs_t timeNowMs)
{
    (void)dateTime;
    (void)timeNowMs;
}

static bool canUseBlackboxWithCurrentConfiguration(void)
{
    return feature(FEATURE_BLACKBOX);
}

/**
 * Call during system startup to initialize the blackbox.
 */
void blackboxInit(void)
{
    if (canUseBlackboxWithCurrentConfiguration()) {
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    } else {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
    }

    /* Decide on how ofter are we going to log I-frames*/
    if (blackboxConfig()->rate_denom <= 32) {
        blackboxIFrameInterval = 32;
    }
    else if (blackboxConfig()->rate_denom <= 64) {
        blackboxIFrameInterval = 64;
    }
    else if (blackboxConfig()->rate_denom <= 128) {
        blackboxIFrameInterval = 128;
    }
    else {
        blackboxIFrameInterval = 256;
    }
}
#endif
