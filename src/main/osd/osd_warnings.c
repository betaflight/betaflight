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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#ifdef USE_OSD

#include "config/config.h"
#include "config/feature.h"

#include "common/maths.h"
#include "common/printf.h"

#include "drivers/display.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"
#include "drivers/dshot.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/pid.h"
#include "flight/pos_hold.h"

#include "io/beeper.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"
#include "osd/osd_warnings.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"
#include "sensors/sensors.h"

const char CRASHFLIP_WARNING[] = ">CRASH FLIP<";

#if defined(USE_ESC_SENSOR) || (defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY))
// ESC alarm character constants
#define ESC_ALARM_CURRENT   'C'
#define ESC_ALARM_TEMP      'T'
#define ESC_ALARM_RPM       'R'

/**
 * Return true if the specified motor is considered active (its commanded output exceeds the disarm threshold).
 *
 * @param motorIndex Index of the motor to check.
 * @return true if motor[motorIndex] > mixerRuntime.disarmMotorOutput, false otherwise.
 */
static inline bool isMotorActive(uint8_t motorIndex) {
    return (motor[motorIndex] > mixerRuntime.disarmMotorOutput);
}

/**
 * Populate an escSensorData_t structure from DShot telemetry for a motor.
 *
 * If DShot telemetry is active for the specified motor, fills dest with
 * available telemetry fields: rpm, temperature (if extended telemetry provides it),
 * current (if provided), voltage (if provided). consumption is set to 0 and
 * dataAge is set to 0 to indicate fresh data.
 *
 * @param dest Destination escSensorData_t to populate.
 * @param motorIndex Index of the motor to read telemetry for.
 * @return true if DShot telemetry was active for motorIndex and dest was populated; false otherwise.
 */
bool getDshotSensorData(escSensorData_t *dest, int motorIndex) {
    // Check if DShot telemetry is active for this motor
    if (!isDshotMotorTelemetryActive(motorIndex)) {
        return false;
    }

    const dshotTelemetryMotorState_t *motorState = &dshotTelemetryState.motorState[motorIndex];
    
    dest->rpm = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_eRPM];

    const bool edt = (motorState->telemetryTypes & DSHOT_EXTENDED_TELEMETRY_MASK) != 0;

    // Extract telemetry data if available
    dest->temperature = edt && (motorState->telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE)) ? 
        motorState->telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE] : 0;

    dest->current = edt && (motorState->telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_CURRENT)) ? 
        motorState->telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT] : 0;

    dest->voltage = edt && (motorState->telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_VOLTAGE)) ? 
        motorState->telemetryData[DSHOT_TELEMETRY_TYPE_VOLTAGE] : 0;

    dest->consumption = 0; // DShot doesn't typically provide consumption data
    dest->dataAge = 0; // Data is fresh

    return true;
}

/**
 * Evaluate ESC sensor/DShot telemetry values for a single motor and record alarm codes.
 *
 * Writes a null-terminated string of alarm characters into `buffer` for the given motor.
 * Possible characters appended are: ESC_ALARM_RPM (RPM alarm), ESC_ALARM_CURRENT (current alarm),
 * and ESC_ALARM_TEMP (temperature alarm). RPM is checked only if the motor is considered active;
 * current and temperature are checked regardless of motor activity. Thresholds are taken from
 * runtime configuration.
 *
 * @param data Pointer to the motor's escSensorData_t (rpm, current, temperature). Must not be NULL.
 * @param motorIndex Index of the motor whose activity state is used to gate RPM checks.
 * @param buffer Destination buffer to receive alarm characters; function appends a terminating '\0'.
 * @return true if one or more alarms were recorded (buffer is non-empty), false otherwise.
 */
static bool checkEscAlarmConditions(escSensorData_t *data, uint8_t motorIndex, char *buffer)
{
    uint8_t alarmPos = 0;

    // Check RPM alarm (only when motor is active)
    if (isMotorActive(motorIndex)) {
        if (data->rpm && osdConfig()->esc_rpm_alarm != ESC_RPM_ALARM_OFF && erpmToRpm(data->rpm) <= osdConfig()->esc_rpm_alarm) {
            buffer[alarmPos++] = ESC_ALARM_RPM;
        }
    }

    // Check current alarm (regardless of motor spinning state)
    if (data->current && osdConfig()->esc_current_alarm != ESC_CURRENT_ALARM_OFF && data->current >= osdConfig()->esc_current_alarm) {
        buffer[alarmPos++] = ESC_ALARM_CURRENT;
    }

    // Check temperature alarm (regardless of motor spinning state)
    if (data->temperature && osdConfig()->esc_temp_alarm != ESC_TEMP_ALARM_OFF && data->temperature >= osdConfig()->esc_temp_alarm) {
        buffer[alarmPos++] = ESC_ALARM_TEMP;
    }

    buffer[alarmPos] = '\0';

    return alarmPos;
}

/**
 * Build an ESC warning string (from ESC sensor data or DShot telemetry) into the provided buffer.
 *
 * Constructs a message prefixed with "ESC" and appends per-motor alarm indicators (e.g. " 1R", "2CT")
 * for any motor whose ESC/DShot data reports RPM, current, or temperature alarms. If the resulting
 * message is shorter than the preferred width, it is centered within the buffer for better visual
 * presentation. If the message would exceed the configured maximum size, the buffer is overwritten
 * with " FAULT OVERLOAD!" and processing stops.
 *
 * @param warningText Buffer where the constructed warning string will be written. Must be at least
 *                    OSD_WARNINGS_MAX_SIZE bytes long. On success contains the warning text;
 *                    on no-alarm result it will be set to an empty string.
 * @param isDshot    true to source ESC data from DShot telemetry (via getDshotSensorData),
 *                    false to source from ESC sensor data (via getEscSensorData).
 * @return true if one or more ESC alarms were found and written into warningText, false if no
 *         alarms were found (warningText will be cleared).
 */
static bool buildEscWarningMessage(char *warningText, bool isDshot) {
    unsigned escErrorLength = 0;
    bool escWarning = false;

    // Write 'ESC' prefix
    escErrorLength += tfp_sprintf(warningText + escErrorLength, "ESC");

    for (unsigned i = 0; i < getMotorCount(); i++) {
        escSensorData_t *escData = NULL;
        escSensorData_t escDataBuffer;
        // Get sensor data based on type
        if (isDshot) {
            if (getDshotSensorData(&escDataBuffer, i)) {
                escData = &escDataBuffer;
            }
        } else {
            escData = getEscSensorData(i);
        }

        if (escData) {
            char alarmChars[4]; // Buffer for alarm characters (C, T, R + '\0')
            // Only show motor if it has alarms (problems only approach)
            if (checkEscAlarmConditions(escData, i, alarmChars)) {
                // compute space needed: " " + digits(motor) + strlen(alarmChars)
                const unsigned digits = (i + 1 >= 10) ? 2 : 1;
                const unsigned needed = 1 + digits + (unsigned)strlen(alarmChars);
                if (escErrorLength + needed >= OSD_WARNINGS_MAX_SIZE) {
                    tfp_sprintf(warningText, " FAULT OVERLOAD!");
                    escWarning = true;
                    break;
                }
                escErrorLength += tfp_sprintf(warningText + escErrorLength, " %d%s", i + 1, alarmChars);
                escWarning = true;
            }
        }
    }

    if (escWarning) {
        // Center message if it's short for better visual presentation
        const int msgLen = strlen(warningText);
        const int minMsgLen = OSD_WARNINGS_PREFFERED_SIZE;
        if (msgLen < minMsgLen - 1) {
            const int capacity = OSD_WARNINGS_MAX_SIZE - 1; // leave room for '\0'
            int offset = (minMsgLen - msgLen) / 2;
            if (offset > capacity - msgLen) {
                offset = capacity - msgLen; // clamp
            }
            if (offset > 0) {
                memmove(warningText + offset, warningText, msgLen + 1);
                memset(warningText, ' ', offset);
            }
        }
        return true;
    }

    warningText[0] = '\0';
    return false;
}
#endif

/**
 * Render the current on-screen warning message and its display attributes.
 *
 * Selects the highest-priority active warning (arming-disabled reasons, arming countdown/beacon,
 * failsafe, crashflip, launch control, RSSI/link quality, battery, ESC/telemetry alarms, visual
 * beeper, chirp completion, etc.), writes a null-terminated warning string into `warningText`,
 * sets the severity in `*displayAttr`, and sets `*blinking` if the message should blink.
 *
 * The function reads various runtime subsystems (arming state, RC modes, battery and telemetry
 * state, ESC/DShot telemetry when available, GPS/rescue, link quality, sensors, config flags,
 * and other feature-specific state) to determine which warning to show. It returns immediately
 * after selecting the first applicable (highest-priority) warning; if no warning applies,
 * `warningText` will be left empty and `*displayAttr` is set to normal with `*blinking == false`.
 *
 * Parameters:
 * @param warningText Pointer to a writable buffer that will receive the null-terminated warning text.
 * @param blinking Pointer to a bool that will be set to true if the displayed warning should blink.
 * @param displayAttr Pointer to a uint8_t that will be set to the display severity code for the warning.
 *
 * Side effects:
 * - May clear the visual beeper state (when the visual beeper warning is shown).
 * - Reads many global subsystem states and configuration values to decide the warning.
 */
void renderOsdWarning(char *warningText, bool *blinking, uint8_t *displayAttr)
{
    const batteryState_e batteryState = getBatteryState();
    const timeUs_t currentTimeUs = micros();

    static timeUs_t armingDisabledUpdateTimeUs;
    static armingDisableFlags_e armingDisabledDisplayFlag = 0;

    warningText[0] = '\0';
    *displayAttr = DISPLAYPORT_SEVERITY_NORMAL;
    *blinking = false;

    // Cycle through the arming disabled reasons
    if (osdWarnGetState(OSD_WARNING_ARMING_DISABLE)) {
        if (IS_RC_MODE_ACTIVE(BOXARM) && isArmingDisabled()) {
            armingDisableFlags_e flags = getArmingDisableFlags();

            // Remove the ARMSWITCH flag unless it's the only one
            if (flags != ARMING_DISABLED_ARM_SWITCH) {
                flags &= ~ARMING_DISABLED_ARM_SWITCH;
            }

            // Rotate to the next arming disabled reason after a 0.5 second time delay
            // or if the current flag is no longer set or if just starting
            if (cmpTimeUs(currentTimeUs, armingDisabledUpdateTimeUs) > 500000
                || (flags & armingDisabledDisplayFlag) == 0) {
                armingDisabledUpdateTimeUs = currentTimeUs;

                armingDisableFlags_e flag = armingDisabledDisplayFlag << 1;         // next bit to try or 0
                armingDisableFlags_e flagsRemaining = flags & ~(flag - 1);          // clear all bits <= flag; clear all bits when flag == 0
                flag = flagsRemaining & -flagsRemaining;                            // LSB in remaining bits
                if (!flag) {
                    // no bit is set above flag (or flag was 0), try again with all bits
                    flag = flags & -flags;
                }
                armingDisabledDisplayFlag = flag;                                   // store for next iteration
            }

            tfp_sprintf(warningText, "%s", getArmingDisableFlagName(armingDisabledDisplayFlag));
            *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
            return;
        } else {
            armingDisabledDisplayFlag = 0;                                          // start from LSB next time
        }
    }

#ifdef USE_DSHOT
    if (isTryingToArm() && !ARMING_FLAG(ARMED)) {
        const int beaconGuard = cmpTimeUs(currentTimeUs, getLastDshotBeaconCommandTimeUs());
        const int armingDelayTime = MAX(DSHOT_BEACON_GUARD_DELAY_US - beaconGuard, 0) / 100000;  // time remaining until BEACON_GUARD_DELAY, in tenths of second
        if (beaconGuard < 500 * 1000) {   // first 0.5s since beacon
            tfp_sprintf(warningText, " BEACON ON");
        } else {
            tfp_sprintf(warningText, "ARM IN %d.%d", armingDelayTime / 10, armingDelayTime % 10);
        }
        *displayAttr = DISPLAYPORT_SEVERITY_INFO;
        return;
    }
#endif // USE_DSHOT
    if (osdWarnGetState(OSD_WARNING_FAIL_SAFE) && failsafeIsActive()) {
        tfp_sprintf(warningText, "FAIL SAFE");
        *displayAttr = DISPLAYPORT_SEVERITY_CRITICAL;
        *blinking = true;
        return;
    }

    // Warn when in flip over after crash mode
    if (osdWarnGetState(OSD_WARNING_CRASHFLIP) && IS_RC_MODE_ACTIVE(BOXCRASHFLIP)) {
        if (isCrashFlipModeActive()) { // if was armed in crashflip mode
            tfp_sprintf(warningText, CRASHFLIP_WARNING);
            *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
            return;
        } else if (!ARMING_FLAG(ARMED)) { // if disarmed, but crashflip mode is activated (not allowed / can't happen)
            tfp_sprintf(warningText, "CRASHFLIP SW");
            *displayAttr = DISPLAYPORT_SEVERITY_INFO;
            return;
        }
    }

#ifdef USE_LAUNCH_CONTROL
    // Warn when in launch control mode
    if (osdWarnGetState(OSD_WARNING_LAUNCH_CONTROL) && isLaunchControlActive()) {
#ifdef USE_ACC
        if (sensors(SENSOR_ACC)) {
            const int pitchAngle = constrain((attitude.raw[FD_PITCH] - accelerometerConfig()->accelerometerTrims.raw[FD_PITCH]) / 10, -90, 90);
            tfp_sprintf(warningText, "LAUNCH %d", pitchAngle);
        } else
#endif // USE_ACC
        {
            tfp_sprintf(warningText, "LAUNCH");
        }

        // Blink the message if the throttle is within 10% of the launch setting
        if ( calculateThrottlePercent() >= MAX(currentPidProfile->launchControlThrottlePercent - 10, 0)) {
            *blinking = true;
        }

        *displayAttr = DISPLAYPORT_SEVERITY_INFO;
        return;
    }
#endif // USE_LAUNCH_CONTROL

    // RSSI
    if (osdWarnGetState(OSD_WARNING_RSSI) && (getRssiPercent() < osdConfig()->rssi_alarm)) {
        tfp_sprintf(warningText, "RSSI LOW");
        *displayAttr = DISPLAYPORT_SEVERITY_CRITICAL;
        *blinking = true;
        return;
    }
#ifdef USE_RX_RSSI_DBM
    // rssi dbm
    if (osdWarnGetState(OSD_WARNING_RSSI_DBM) && (getRssiDbm() < osdConfig()->rssi_dbm_alarm)) {
        tfp_sprintf(warningText, "RSSI DBM");
        *displayAttr = DISPLAYPORT_SEVERITY_CRITICAL;
        *blinking = true;
        return;
    }
#endif // USE_RX_RSSI_DBM
#ifdef USE_RX_RSNR
    // rsnr
    if (osdWarnGetState(OSD_WARNING_RSNR) && (getRsnr() < osdConfig()->rsnr_alarm)) {
        tfp_sprintf(warningText, "RSNR LOW");
        *displayAttr = DISPLAYPORT_SEVERITY_CRITICAL;
        *blinking = true;
        return;
    }
#endif // USE_RX_RSNR

#ifdef USE_RX_LINK_QUALITY_INFO
    // Link Quality
    if (osdWarnGetState(OSD_WARNING_LINK_QUALITY) && (rxGetLinkQualityPercent() < osdConfig()->link_quality_alarm)) {
        tfp_sprintf(warningText, "LINK QUALITY");
        *displayAttr = DISPLAYPORT_SEVERITY_CRITICAL;
        *blinking = true;
        return;
    }
#endif // USE_RX_LINK_QUALITY_INFO

    if (osdWarnGetState(OSD_WARNING_BATTERY_CRITICAL) && batteryState == BATTERY_CRITICAL) {
        tfp_sprintf(warningText, " LAND NOW");
        *displayAttr = DISPLAYPORT_SEVERITY_CRITICAL;
        *blinking = true;
        return;
    }

    if (osdWarnGetState(OSD_WARNING_LOAD) && (getArmingDisableFlags() & ARMING_DISABLED_LOAD)) {
        tfp_sprintf(warningText, "CPU OVERLOAD");
        *displayAttr = DISPLAYPORT_SEVERITY_CRITICAL;
        *blinking = true;
        return;
    }

#ifdef USE_GPS_RESCUE
    if (osdWarnGetState(OSD_WARNING_GPS_RESCUE_UNAVAILABLE) &&
       ARMING_FLAG(ARMED) &&
       gpsRescueIsConfigured() &&
       !gpsRescueIsDisabled() &&
       !gpsRescueIsAvailable()) {
        tfp_sprintf(warningText, "RESCUE N/A");
        *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
        *blinking = true;
        return;
    }

    if (osdWarnGetState(OSD_WARNING_GPS_RESCUE_DISABLED) &&
       ARMING_FLAG(ARMED) &&
       gpsRescueIsConfigured() &&
       gpsRescueIsDisabled()) {

        statistic_t *stats = osdGetStats();
        if (cmpTimeUs(stats->armed_time, OSD_GPS_RESCUE_DISABLED_WARNING_DURATION_US) < 0) {
            tfp_sprintf(warningText, "RESCUE OFF");
            *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
            *blinking = true;
            return;
        }
    }

#endif // USE_GPS_RESCUE

#ifdef USE_POSITION_HOLD
    if (osdWarnGetState(OSD_WARNING_POSHOLD_FAILED) && posHoldFailure()) {
        tfp_sprintf(warningText, "POSHOLD FAIL");
        *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
        *blinking = true;
        return;
    }
#endif

    // Show warning if in HEADFREE flight mode
    if (FLIGHT_MODE(HEADFREE_MODE)) {
        tfp_sprintf(warningText, "HEADFREE");
        *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
        *blinking = true;
        return;
    }

#ifdef USE_ADC_INTERNAL
    const int16_t coreTemperature = getCoreTemperatureCelsius();
    if (osdWarnGetState(OSD_WARNING_CORE_TEMPERATURE) && coreTemperature >= osdConfig()->core_temp_alarm) {
        tfp_sprintf(warningText, "CORE %c: %3d%c", SYM_TEMPERATURE, osdConvertTemperatureToSelectedUnit(coreTemperature), osdGetTemperatureSymbolForSelectedUnit());
        *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
        *blinking = true;
        return;
    }
#endif // USE_ADC_INTERNAL

#ifdef USE_ESC_SENSOR
    // Show warning if we lose motor output, the ESC is overheating or excessive current draw
    if (featureIsEnabled(FEATURE_ESC_SENSOR) && osdWarnGetState(OSD_WARNING_ESC_FAIL) && ARMING_FLAG(ARMED)) {
        if (buildEscWarningMessage(warningText, false)) {
            *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
            *blinking = true;
            return;
        }
    }
#endif // USE_ESC_SENSOR

#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
    // Show esc error
    if (motorConfig()->dev.useDshotTelemetry && osdWarnGetState(OSD_WARNING_ESC_FAIL) && ARMING_FLAG(ARMED)) {
        if (buildEscWarningMessage(warningText, true)) {
            *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
            *blinking = true;
            return;
        }
    }
#endif

    if (osdWarnGetState(OSD_WARNING_BATTERY_WARNING) && batteryState == BATTERY_WARNING) {
        tfp_sprintf(warningText, "LOW BATTERY");
        *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
        *blinking = true;
        return;
    }

    // Show warning if mah consumed is over the configured limit
    if (osdWarnGetState(OSD_WARNING_OVER_CAP) && ARMING_FLAG(ARMED) && osdConfig()->cap_alarm > 0 && getMAhDrawn() >= osdConfig()->cap_alarm) {
        tfp_sprintf(warningText, "OVER CAP");
        *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
        *blinking = true;
        return;
    }

#ifdef USE_BATTERY_CONTINUE
    // Show warning if battery is not fresh and battery continue is active
    if (hasUsedMAh()) {
        tfp_sprintf(warningText, "BATTERY CONT");
        *displayAttr = DISPLAYPORT_SEVERITY_INFO;
        return;
    }
#endif // USE_BATTERY_CONTINUE

    // Show warning if battery is not fresh
    if (osdWarnGetState(OSD_WARNING_BATTERY_NOT_FULL) && !(ARMING_FLAG(ARMED) || ARMING_FLAG(WAS_EVER_ARMED)) && (getBatteryState() == BATTERY_OK)
          && getBatteryAverageCellVoltage() < batteryConfig()->vbatfullcellvoltage) {
        tfp_sprintf(warningText, "BATT < FULL");
        *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
        return;
    }

    // Visual beeper
    if (osdWarnGetState(OSD_WARNING_VISUAL_BEEPER) && osdGetVisualBeeperState()) {
        tfp_sprintf(warningText, "  * * * *");
        *displayAttr = DISPLAYPORT_SEVERITY_INFO;
        osdSetVisualBeeperState(false);
        return;
    }

#ifdef USE_CHIRP
    // Visual info that chirp excitation is finished
    if (pidChirpIsFinished()) {
        tfp_sprintf(warningText, "CHIRP EXC FINISHED");
        *displayAttr = DISPLAYPORT_SEVERITY_INFO;
        *blinking = true;
        return;
    }
#endif // USE_CHIRP

}

#endif // USE_OSD
