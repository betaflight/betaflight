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
#include "sensors/sensors.h"

const char CRASHFLIP_WARNING[] = ">CRASH FLIP<";

// ESC alarm character constants
#define ESC_ALARM_CURRENT   'C'
#define ESC_ALARM_TEMP      'T'
#define ESC_ALARM_RPM       'R'

// Check if character represents an alarm condition
#define IS_ESC_ALARM(c) ((c) == ESC_ALARM_CURRENT || (c) == ESC_ALARM_TEMP || (c) == ESC_ALARM_RPM)

#if defined(USE_ESC_SENSOR) || (defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY))
static char checkEscAlarmConditions(uint8_t motorIndex, uint16_t rpm, uint16_t temperature, uint16_t current, bool rpmAvailable, bool tempAvailable, bool currentAvailable)
{
    // Check current alarm (regardless of motor spinning state)
    if (currentAvailable && osdConfig()->esc_current_alarm != ESC_CURRENT_ALARM_OFF && current >= osdConfig()->esc_current_alarm) {
        return ESC_ALARM_CURRENT;
    }

    // Check temperature alarm (regardless of motor spinning state)
    if (tempAvailable && osdConfig()->esc_temp_alarm != ESC_TEMP_ALARM_OFF && temperature >= osdConfig()->esc_temp_alarm) {
        return ESC_ALARM_TEMP;
    }

    // Check RPM alarm (only when motor is spinning)
    if (motorIndex < getMotorCount() && motor[motorIndex] > mixerRuntime.disarmMotorOutput) {
        if (rpmAvailable && osdConfig()->esc_rpm_alarm != ESC_RPM_ALARM_OFF && rpm <= osdConfig()->esc_rpm_alarm) {
            return ESC_ALARM_RPM;
        }
    }

    // No alarm, display motor number
    return '0' + (motorIndex + 1) % 10;
}
#endif

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
        char* p = warningText;
        *p++ = 'E';
        *p++ = 'S';
        *p++ = 'C';

        bool escWarning = false;
        for (unsigned i = 0; i < getMotorCount() && p < warningText + OSD_WARNINGS_MAX_SIZE - 1; i++) {
            escSensorData_t *escData = getEscSensorData(i);

            char alarmChar = checkEscAlarmConditions(i,
                erpmToRpm(escData->rpm), 
                escData->temperature, 
                escData->current, 
                escData->rpm > 0, 
                escData->temperature > 0, 
                escData->current >= 0);

            escWarning |= IS_ESC_ALARM(alarmChar);
            *p++ = alarmChar;
        }

        *p = 0;  // terminate string

        if (escWarning) {
            const int msgLen = p - warningText;  // Calculate length from pointer difference
            const int minMsgLen = OSD_WARNINGS_PREFFERED_SIZE;           // intended minimum width
            if (msgLen < minMsgLen - 1) {
                // message is short, center it within minMsgLen
                const int offset = (minMsgLen - msgLen) / 2;
                memmove(warningText + offset, warningText, msgLen + 1);  // copy including '\0'
                memset(warningText, ' ', offset);                        // left padding with spaces
            }
            *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
            *blinking = true;
            return;
        } else {
            // no warning, erase generated message and continue
            warningText[0] = '\0';
        }
    }
#endif // USE_ESC_SENSOR

#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
    // Show esc error
    if (motorConfig()->dev.useDshotTelemetry && osdWarnGetState(OSD_WARNING_ESC_FAIL) && ARMING_FLAG(ARMED)) {
        uint8_t dshotEscErrorLength = 0;
        bool escWarning = false;

        // Write 'ESC'
        warningText[dshotEscErrorLength++] = 'E';
        warningText[dshotEscErrorLength++] = 'S';
        warningText[dshotEscErrorLength++] = 'C';

        // Cache motor pole count before loop
        const uint8_t motorPoleCount = motorConfig()->motorPoleCount;

        for (uint8_t k = 0; k < getMotorCount(); k++) {
            uint8_t dshotEscErrorLengthMotorBegin = dshotEscErrorLength;

            // Write ESC nr
            warningText[dshotEscErrorLength++] = ' ';
            warningText[dshotEscErrorLength++] = '0' + k + 1;

            if (isDshotMotorTelemetryActive(k)) {
                // Cache motor state for efficient access
                const dshotTelemetryMotorState_t *motorState = &dshotTelemetryState.motorState[k];
                
                // Calculate RPM from eRPM with cached pole count
                uint16_t rpm = motorState->telemetryData[DSHOT_TELEMETRY_TYPE_eRPM] * 200 / motorPoleCount;

                // Direct bit checking for extended telemetry
                bool edt = (motorState->telemetryTypes & DSHOT_EXTENDED_TELEMETRY_MASK) != 0;
                
                // Optimize telemetry availability and data extraction
                uint16_t temperature = edt && (motorState->telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE)) ? 
                    motorState->telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE] : 0;

                uint16_t current = edt && (motorState->telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_CURRENT)) ? 
                    motorState->telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT] : 0;

                char alarmChar = checkEscAlarmConditions(k, rpm, temperature, current, 
                    rpm > 0, 
                    temperature > 0, 
                    current > 0);

                escWarning |= IS_ESC_ALARM(alarmChar);
                warningText[dshotEscErrorLength++] = alarmChar;
            }

            // If no esc warning data undo esc nr (esc telemetry data types depends on the esc hw/sw)
            if (dshotEscErrorLengthMotorBegin + 2 == dshotEscErrorLength) {
                dshotEscErrorLength = dshotEscErrorLengthMotorBegin;
            }
        }

        // If warning exists then notify, otherwise clear warning message
        if (escWarning) {
            warningText[dshotEscErrorLength] = 0;  // terminate string
            *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
            *blinking = true;
        } else {
            warningText[0] = 0;
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
