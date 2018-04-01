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

#include <platform.h>

#ifdef USE_TARGET_CONFIG

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/light_led.h"
#include "drivers/pwm_esc_detect.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/vcd.h"

#include "rx/rx.h"

#include "io/serial.h"
#include "io/osd.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#if !defined(BEEBRAIN_V2D)
#define BBV2_FRSKY_RSSI_CH_IDX     9
#endif

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 32000           // 32kHz

void targetConfiguration(void)
{
    if (hardwareMotorType == MOTOR_BRUSHED) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1030;
        pidConfigMutable()->pid_process_denom = 1;
    }

    for (uint8_t pidProfileIndex = 0; pidProfileIndex < MAX_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[PID_ROLL].P  = 86;
        pidProfile->pid[PID_ROLL].I  = 50;
        pidProfile->pid[PID_ROLL].D  = 60;
        pidProfile->pid[PID_PITCH].P = 90;
        pidProfile->pid[PID_PITCH].I = 55;
        pidProfile->pid[PID_PITCH].D = 60;
        pidProfile->pid[PID_YAW].P   = 123;
        pidProfile->pid[PID_YAW].I   = 75;
    }

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        controlRateConfig->rcRates[FD_YAW] = 120;
        controlRateConfig->rcExpo[FD_ROLL] = 15;
        controlRateConfig->rcExpo[FD_PITCH] = 15;
        controlRateConfig->rcExpo[FD_YAW]  = 15;
        controlRateConfig->rates[FD_ROLL]  = 85;
        controlRateConfig->rates[FD_PITCH] = 85;
    }

    batteryConfigMutable()->batteryCapacity = 250;
    batteryConfigMutable()->vbatmincellvoltage = 28;
    batteryConfigMutable()->vbatwarningcellvoltage = 33;

    *customMotorMixerMutable(0) = (motorMixer_t){ 1.0f, -0.414178f,  1.0f, -1.0f };    // REAR_R
    *customMotorMixerMutable(1) = (motorMixer_t){ 1.0f, -0.414178f, -1.0f,  1.0f };    // FRONT_R
    *customMotorMixerMutable(2) = (motorMixer_t){ 1.0f,  0.414178f,  1.0f,  1.0f };    // REAR_L
    *customMotorMixerMutable(3) = (motorMixer_t){ 1.0f,  0.414178f, -1.0f, -1.0f };    // FRONT_L

    vcdProfileMutable()->video_system = VIDEO_SYSTEM_NTSC;
    strcpy(pilotConfigMutable()->name, "BeeBrain V2");
    osdConfigMutable()->cap_alarm  = 250;
    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(9, 11)  | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(23, 10) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(2, 10)  | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_FLYMODE]           = OSD_POS(17, 10) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(10, 10) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]         &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_1]       &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_THROTTLE_POS]       &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_CROSSHAIRS]         &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_HORIZON_SIDEBARS]   &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ARTIFICIAL_HORIZON] &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_CURRENT_DRAW]       &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_MAH_DRAWN]          &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_GPS_SPEED]          &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_GPS_LON]            &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_GPS_LAT]            &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_GPS_SATS]           &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_HOME_DIR]           &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_HOME_DIST]          &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_COMPASS_BAR]        &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ALTITUDE]           &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ROLL_PIDS]          &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_PITCH_PIDS]         &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_YAW_PIDS]           &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_DEBUG]              &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_POWER]              &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_PIDRATE_PROFILE]    &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]           &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_AVG_CELL_VOLTAGE]   &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_PITCH_ANGLE]        &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ROLL_ANGLE]         &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_USAGE]    &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_DISARMED]           &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_NUMERICAL_HEADING]  &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_NUMERICAL_VARIO]    &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ESC_TMP]            &= ~VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ESC_RPM]            &= ~VISIBLE_FLAG;

    modeActivationConditionsMutable(0)->modeId           = BOXANGLE;
    modeActivationConditionsMutable(0)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(0)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);

#if defined(BEEBRAIN_V2D)
    // DSM version
    for (uint8_t rxRangeIndex = 0; rxRangeIndex < NON_AUX_CHANNEL_COUNT; rxRangeIndex++) {
        rxChannelRangeConfig_t *channelRangeConfig = rxChannelRangeConfigsMutable(rxRangeIndex);

        channelRangeConfig->min = 1160;
        channelRangeConfig->max = 1840;
    }
#else
    // Frsky version
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIALRX_UART)].functionMask = FUNCTION_TELEMETRY_FRSKY_HUB | FUNCTION_RX_SERIAL;
    rxConfigMutable()->rssi_channel = BBV2_FRSKY_RSSI_CH_IDX;
    rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigsMutable(BBV2_FRSKY_RSSI_CH_IDX - 1);
    channelFailsafeConfig->mode = RX_FAILSAFE_MODE_SET;
    channelFailsafeConfig->step = CHANNEL_VALUE_TO_RXFAIL_STEP(1000);
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]        = OSD_POS(2, 11)  | VISIBLE_FLAG;
#endif
}
#endif
