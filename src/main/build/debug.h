/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#define DEBUG16_VALUE_COUNT 8
extern int16_t debug[DEBUG16_VALUE_COUNT];
extern uint8_t debugMode;

/*
 * Debug field annotations
 *
 * `debugType_e` below fixes what a debug mode *is*, and the DEBUG_SET() call
 * sites fix which `debug[n]` each mode writes. Neither records what a field
 * *means*, so every tool that displays debug data - configurator, blackbox log
 * viewer, documentation - has kept its own copy of the field labels, written by
 * reading firmware diffs and drifting silently whenever a mode is reworked.
 *
 * Record the meaning at the call site instead, in a trailing `//!<` comment, so
 * it is updated by the same diff that changes what the field holds. It is a
 * comment, so it costs no flash:
 *
 *     DEBUG_SET(DEBUG_CYCLETIME, 0, getTaskDeltaTimeUs(TASK_SELF));  //!< Cycle Time [unit:us]
 *     DEBUG_SET(DEBUG_CYCLETIME, 1, getAverageSystemLoadPercent());  //!< CPU Load [unit:%]
 *
 * Grammar, on the line the call ends on. Every bracket names what it is, so
 * nothing about an annotation depends on where it sits:
 *
 *     //!< [index:<indices>] <label> [<shape>]
 *
 *   index:     Which debug[n] this call writes. Omit it when the index argument
 *              is a compile-time constant; give it when the index is computed at
 *              run time (`axis`, `2 * axis + 1`, `motorIndex`), because tooling
 *              cannot evaluate those: `[index:2]`, `[index:0..2]`,
 *              `[index:0,2,4]`.
 *   <label>    What the value is, in the words a pilot reads rather than the name
 *              of the variable holding it. The one part that is prose, and the
 *              only one without a key. No `[` or `]`. One `{a|b|c}` group may
 *              spell out one label per index, in index order, for a call that
 *              writes a range: `Gyro Filtered {roll|pitch|yaw}`.
 *   <shape>    What kind of value the field holds. Tooling derives the sample
 *              format and the graph axis from the shape alone, so a field is one
 *              of exactly four:
 *
 *     unit:    A quantity, given as the unit of one LSB of the stored value. An
 *              optional decimal factor precedes the symbol, so
 *              `lrintf(angleDeg * 10)` is `[unit:0.1deg]` and `pressurePa / 100`
 *              is `[unit:100Pa]`. The factor may be negative, for a field that
 *              stores the magnitude of a negative quantity: a CRSF RSSI is
 *              `[unit:-1dBm]`. The symbol may be left out for a value that is
 *              scaled but dimensionless: `lrintf(ratio * 1000)` is
 *              `[unit:0.001]`.
 *
 *              Symbols: s ms us Hz kHz MHz kbit/s rad rad/s deg dps dps2 m cm
 *              m/s cm/s g g/s V A mAh degC Pa hPa rpm % dB dBm bytes ticks, plus
 *              the device-native units the firmware stores raw, which only the
 *              FC's own configuration can convert: gyroADC (gyro ADC counts),
 *              accADC and accADC/s (accelerometer ADC counts), rcCommand
 *              (throttle in rcCommand units) and eRPM (Dshot eRPM).
 *
 *     enum:    An enumerator, named by its type - `[enum:failsafePhase_e]`.
 *              Tooling reads the enumerator names from it, so the enum must be
 *              visible in the file or in a header the file includes.
 *
 *     flags:   Bit flags, named lowest bit first, with `-` for a bit the field
 *              does not use: `[flags:Channel 17|Channel 18|Signal Loss|Failsafe]`.
 *              The names are given here rather than read from the source because
 *              flag bits are `#define`s, not an enum. Tooling shows which bits
 *              are set instead of the number they add up to.
 *
 *     (none)   A plain integer: a count, or a state with no enum to name it.
 *              Tooling shows the number and fits the graph to the logged data.
 *
 * A field that packs two values into one index - `state * 100 + position` - or
 * that writes a sentinel is a plain integer to every consumer, and stays
 * unreadable however it is annotated; split it across two indices instead.
 *
 * An index written from more than one place has to mean one thing in a given
 * build, so annotations that share an index must agree - a disagreement is a bug
 * in the debug mode, not in the annotation.
 */
#define DEBUG_SET(mode, index, value) do { if (debugMode == (mode)) { debug[(index)] = (value); } } while (0)

typedef enum {
    DEBUG_NONE,
    DEBUG_CYCLETIME,
    DEBUG_BATTERY,
    DEBUG_GYRO_FILTERED,
    DEBUG_ACCELEROMETER,
    DEBUG_PIDLOOP,
    DEBUG_RC_INTERPOLATION,
    DEBUG_ANGLERATE,
    DEBUG_ESC_SENSOR,
    DEBUG_SCHEDULER,
    DEBUG_STACK,
    DEBUG_ESC_SENSOR_RPM,
    DEBUG_ESC_SENSOR_TMP,
    DEBUG_ALTITUDE,
    DEBUG_FFT,
    DEBUG_FFT_TIME,
    DEBUG_FFT_FREQ,
    DEBUG_RX_FRSKY_SPI,
    DEBUG_RX_SFHSS_SPI,
    DEBUG_GYRO_RAW,
    DEBUG_MULTI_GYRO_RAW,
    DEBUG_MULTI_GYRO_DIFF,
    DEBUG_MAX7456_SIGNAL,
    DEBUG_MAX7456_SPICLOCK,
    DEBUG_SBUS,
    DEBUG_FPORT,
    DEBUG_RANGEFINDER,
    DEBUG_RANGEFINDER_QUALITY,
    DEBUG_OPTICALFLOW,
    DEBUG_LIDAR_TF,
    DEBUG_ADC_INTERNAL,
    DEBUG_RUNAWAY_TAKEOFF,
    DEBUG_SDIO,
    DEBUG_CURRENT_SENSOR,
    DEBUG_USB,
    DEBUG_SMARTAUDIO,
    DEBUG_RTH,
    DEBUG_ITERM_RELAX,
    DEBUG_ACRO_TRAINER,
    DEBUG_RC_SMOOTHING,
    DEBUG_RX_SIGNAL_LOSS,
    DEBUG_RC_SMOOTHING_RATE,
    DEBUG_ANTI_GRAVITY,
    DEBUG_DYN_LPF,
    DEBUG_RX_SPEKTRUM_SPI,
    DEBUG_DSHOT_RPM_TELEMETRY,
    DEBUG_RPM_FILTER,
    DEBUG_D_MAX,
    DEBUG_AC_CORRECTION,
    DEBUG_AC_ERROR,
    DEBUG_MULTI_GYRO_SCALED,
    DEBUG_DSHOT_RPM_ERRORS,
    DEBUG_CRSF_LINK_STATISTICS_UPLINK,
    DEBUG_CRSF_LINK_STATISTICS_PWR,
    DEBUG_CRSF_LINK_STATISTICS_DOWN,
    DEBUG_BARO,
    DEBUG_AUTOPILOT_ALTITUDE,
    DEBUG_DYN_IDLE,
    DEBUG_FEEDFORWARD_LIMIT,
    DEBUG_FEEDFORWARD,
    DEBUG_BLACKBOX_OUTPUT,
    DEBUG_GYRO_SAMPLE,
    DEBUG_RX_TIMING,
    DEBUG_D_LPF,
    DEBUG_VTX_TRAMP,
    DEBUG_GHST,
    DEBUG_GHST_MSP,
    DEBUG_SCHEDULER_DETERMINISM,
    DEBUG_TIMING_ACCURACY,
    DEBUG_RX_EXPRESSLRS_SPI,
    DEBUG_RX_EXPRESSLRS_PHASELOCK,
    DEBUG_RX_STATE_TIME,
    DEBUG_GPS_RESCUE_VELOCITY,
    DEBUG_GPS_RESCUE_HEADING,
    DEBUG_GPS_RESCUE_TRACKING,
    DEBUG_GPS_CONNECTION,
    DEBUG_ATTITUDE,
    DEBUG_VTX_MSP,
    DEBUG_GPS_DOP,
    DEBUG_FAILSAFE,
    DEBUG_GYRO_CALIBRATION,
    DEBUG_ANGLE_MODE,
    DEBUG_ANGLE_TARGET,
    DEBUG_CURRENT_ANGLE,
    DEBUG_DSHOT_TELEMETRY_COUNTS,
    DEBUG_RPM_LIMIT,
    DEBUG_RC_STATS,
    DEBUG_MAG_CALIB,
    DEBUG_MAG_TASK_RATE,
    DEBUG_EZLANDING,
    DEBUG_TPA,
    DEBUG_S_TERM,
    DEBUG_SPA,
    DEBUG_TASK,
    DEBUG_GIMBAL,
    DEBUG_WING_SETPOINT,
    DEBUG_CHIRP,
    DEBUG_FLASH_TEST_PRBS,
    DEBUG_MAVLINK_TELEMETRY,
    DEBUG_AUTOPILOT_PID,
    DEBUG_POSITION_NAV,
    DEBUG_AUTOPILOT_STOP,
    DEBUG_PITOT,
    DEBUG_POSITION_EST,
    DEBUG_COUNT
} debugType_e;

extern const char * const debugModeNames[DEBUG_COUNT];

void debugInit(void);
