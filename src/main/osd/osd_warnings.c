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
#include "flight/pid.h"

#include "io/beeper.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"
#include "osd/osd_warnings.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"

const char CRASHFLIP_WARNING[] = "> CRASH FLIP <";

void renderOsdWarning(char *warningText, bool *blinking, uint8_t *displayAttr)
{
    const batteryState_e batteryState = getBatteryState();
    const timeUs_t currentTimeUs = micros();

    static timeUs_t armingDisabledUpdateTimeUs;
    static unsigned armingDisabledDisplayIndex;

    warningText[0] = '\0';
    *displayAttr = DISPLAYPORT_SEVERITY_NORMAL;
    *blinking = false;

    // Cycle through the arming disabled reasons
    if (osdWarnGetState(OSD_WARNING_ARMING_DISABLE)) {
        if (IS_RC_MODE_ACTIVE(BOXARM) && isArmingDisabled()) {
            const armingDisableFlags_e armSwitchOnlyFlag = 1 << (ARMING_DISABLE_FLAGS_COUNT - 1);
            armingDisableFlags_e flags = getArmingDisableFlags();

            // Remove the ARMSWITCH flag unless it's the only one
            if ((flags & armSwitchOnlyFlag) && (flags != armSwitchOnlyFlag)) {
                flags -= armSwitchOnlyFlag;
            }

            // Rotate to the next arming disabled reason after a 0.5 second time delay
            // or if the current flag is no longer set
            if ((currentTimeUs - armingDisabledUpdateTimeUs > 5e5) || !(flags & (1 << armingDisabledDisplayIndex))) {
                if (armingDisabledUpdateTimeUs == 0) {
                    armingDisabledDisplayIndex = ARMING_DISABLE_FLAGS_COUNT - 1;
                }
                armingDisabledUpdateTimeUs = currentTimeUs;

                do {
                    if (++armingDisabledDisplayIndex >= ARMING_DISABLE_FLAGS_COUNT) {
                        armingDisabledDisplayIndex = 0;
                    }
                } while (!(flags & (1 << armingDisabledDisplayIndex)));
            }

            tfp_sprintf(warningText, "%s", armingDisableFlagNames[armingDisabledDisplayIndex]);
            *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
            return;
        } else {
            armingDisabledUpdateTimeUs = 0;
        }
    }

#ifdef USE_DSHOT
    if (isTryingToArm() && !ARMING_FLAG(ARMED)) {
        int armingDelayTime = (getLastDshotBeaconCommandTimeUs() + DSHOT_BEACON_GUARD_DELAY_US - currentTimeUs) / 1e5;
        if (armingDelayTime < 0) {
            armingDelayTime = 0;
        }
        if (armingDelayTime >= (DSHOT_BEACON_GUARD_DELAY_US / 1e5 - 5)) {
            tfp_sprintf(warningText, " BEACON ON"); // Display this message for the first 0.5 seconds
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
            *displayAttr = DISPLAYPORT_SEVERITY_INFO;
            return;
        } else if (!ARMING_FLAG(ARMED)) { // if disarmed, but crashflip mode is activated (not allowed / can't happen)
            tfp_sprintf(warningText, "CRASH FLIP SWITCH");
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
    if (featureIsEnabled(FEATURE_ESC_SENSOR) && osdWarnGetState(OSD_WARNING_ESC_FAIL)) {
        char escWarningMsg[OSD_FORMAT_MESSAGE_BUFFER_SIZE];
        unsigned pos = 0;

        const char *title = "ESC";

        // center justify message
        while (pos < (OSD_WARNINGS_MAX_SIZE - (strlen(title) + getMotorCount())) / 2) {
            escWarningMsg[pos++] = ' ';
        }

        strcpy(escWarningMsg + pos, title);
        pos += strlen(title);

        unsigned i = 0;
        unsigned escWarningCount = 0;
        while (i < getMotorCount() && pos < OSD_FORMAT_MESSAGE_BUFFER_SIZE - 1) {
            escSensorData_t *escData = getEscSensorData(i);
            const char motorNumber = '1' + i;
            // if everything is OK just display motor number else R, T or C
            char warnFlag = motorNumber;
            if (ARMING_FLAG(ARMED) && osdConfig()->esc_rpm_alarm != ESC_RPM_ALARM_OFF && (uint32_t)erpmToRpm(escData->rpm) <= (uint32_t)osdConfig()->esc_rpm_alarm) {
                warnFlag = 'R';
            }
            if (osdConfig()->esc_temp_alarm != ESC_TEMP_ALARM_OFF && escData->temperature >= osdConfig()->esc_temp_alarm) {
                warnFlag = 'T';
            }
            if (ARMING_FLAG(ARMED) && osdConfig()->esc_current_alarm != ESC_CURRENT_ALARM_OFF && escData->current >= osdConfig()->esc_current_alarm) {
                warnFlag = 'C';
            }

            escWarningMsg[pos++] = warnFlag;

            if (warnFlag != motorNumber) {
                escWarningCount++;
            }

            i++;
        }

        escWarningMsg[pos] = '\0';

        if (escWarningCount > 0) {
            tfp_sprintf(warningText, "%s", escWarningMsg);
            *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
            *blinking = true;
            return;
        }
    }
#endif // USE_ESC_SENSOR

#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
    // Show esc error
    if (osdWarnGetState(OSD_WARNING_ESC_FAIL)) {
        uint32_t dshotEscErrorLengthMotorBegin;
        uint32_t dshotEscErrorLength = 0;

        // Write 'ESC'
        warningText[dshotEscErrorLength++] = 'E';
        warningText[dshotEscErrorLength++] = 'S';
        warningText[dshotEscErrorLength++] = 'C';

        for (uint8_t k = 0; k < getMotorCount(); k++) {
            // Skip if no extended telemetry at all
            if ((dshotTelemetryState.motorState[k].telemetryTypes & DSHOT_EXTENDED_TELEMETRY_MASK) == 0) {
                continue;
            }

            // Remember text index before writing warnings
            dshotEscErrorLengthMotorBegin = dshotEscErrorLength;

            // Write ESC nr
            warningText[dshotEscErrorLength++] = ' ';
            warningText[dshotEscErrorLength++] = '0' + k + 1;

            // Add esc warnings
            if (ARMING_FLAG(ARMED) && osdConfig()->esc_rpm_alarm != ESC_RPM_ALARM_OFF
                    && isDshotMotorTelemetryActive(k)
                    && (dshotTelemetryState.motorState[k].telemetryData[DSHOT_TELEMETRY_TYPE_eRPM] * 100 * 2 / motorConfig()->motorPoleCount) <= osdConfig()->esc_rpm_alarm) {
                warningText[dshotEscErrorLength++] = 'R';
            }
            if (osdConfig()->esc_temp_alarm != ESC_TEMP_ALARM_OFF
                    && (dshotTelemetryState.motorState[k].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE)) != 0
                    && dshotTelemetryState.motorState[k].telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE] >= osdConfig()->esc_temp_alarm) {
                warningText[dshotEscErrorLength++] = 'T';
            }
            if (ARMING_FLAG(ARMED) && osdConfig()->esc_current_alarm != ESC_CURRENT_ALARM_OFF
                    && (dshotTelemetryState.motorState[k].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_CURRENT)) != 0
                    && dshotTelemetryState.motorState[k].telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT] >= osdConfig()->esc_current_alarm) {
                warningText[dshotEscErrorLength++] = 'C';
            }

            // If no esc warning data undo esc nr (esc telemetry data types depends on the esc hw/sw)
            if (dshotEscErrorLengthMotorBegin + 2 == dshotEscErrorLength)
                dshotEscErrorLength = dshotEscErrorLengthMotorBegin;
        }

        // If warning exists then notify, otherwise clear warning message
        if (dshotEscErrorLength > 3) {
            warningText[dshotEscErrorLength] = 0;        // End string
            *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
            *blinking = true;
            return;
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

#ifdef USE_RC_SMOOTHING_FILTER
    // Show warning if rc smoothing hasn't initialized the filters
    if (osdWarnGetState(OSD_WARNING_RC_SMOOTHING) && ARMING_FLAG(ARMED) && !rcSmoothingInitializationComplete() && rxConfig()->rc_smoothing_mode) {
        tfp_sprintf(warningText, "RCSMOOTHING");
        *displayAttr = DISPLAYPORT_SEVERITY_WARNING;
        *blinking = true;
        return;
    }
#endif // USE_RC_SMOOTHING_FILTER

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
        tfp_sprintf(warningText, "BATTERY CONTINUE");
        *displayAttr = DISPLAYPORT_SEVERITY_INFO;
        return;
    }
#endif // USE_BATTERY_CONTINUE

    // Show warning if battery is not fresh
    if (osdWarnGetState(OSD_WARNING_BATTERY_NOT_FULL) && !(ARMING_FLAG(ARMED) || ARMING_FLAG(WAS_EVER_ARMED)) && (getBatteryState() == BATTERY_OK)
          && getBatteryAverageCellVoltage() < batteryConfig()->vbatfullcellvoltage) {
        tfp_sprintf(warningText, "BATT < FULL");
        *displayAttr = DISPLAYPORT_SEVERITY_INFO;
        return;
    }

    // Visual beeper
    if (osdWarnGetState(OSD_WARNING_VISUAL_BEEPER) && osdGetVisualBeeperState()) {
        tfp_sprintf(warningText, "  * * * *");
        *displayAttr = DISPLAYPORT_SEVERITY_INFO;
        osdSetVisualBeeperState(false);
        return;
    }

}

#endif // USE_OSD
