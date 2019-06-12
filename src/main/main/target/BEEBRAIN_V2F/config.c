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
#include <string.h>

#include "platform.h"

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
#include "pg/rx.h"

#include "rx/rx.h"

#include "io/serial.h"

#include "osd/osd.h"

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
    if (getDetectedMotorType() == MOTOR_BRUSHED) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1030;
        pidConfigMutable()->pid_process_denom = 1;
    }

    for (uint8_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
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
    batteryConfigMutable()->vbatmincellvoltage = 280;
    batteryConfigMutable()->vbatwarningcellvoltage = 330;

    *customMotorMixerMutable(0) = (motorMixer_t){ 1.0f, -0.414178f,  1.0f, -1.0f };    // REAR_R
    *customMotorMixerMutable(1) = (motorMixer_t){ 1.0f, -0.414178f, -1.0f,  1.0f };    // FRONT_R
    *customMotorMixerMutable(2) = (motorMixer_t){ 1.0f,  0.414178f,  1.0f,  1.0f };    // REAR_L
    *customMotorMixerMutable(3) = (motorMixer_t){ 1.0f,  0.414178f, -1.0f, -1.0f };    // FRONT_L

    vcdProfileMutable()->video_system = VIDEO_SYSTEM_NTSC;
#if defined(BEESTORM)
    strcpy(pilotConfigMutable()->name, "BeeStorm");
#else
    strcpy(pilotConfigMutable()->name, "BeeBrain V2");
#endif
    osdConfigMutable()->cap_alarm  = 250;
    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(9, 11)  | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(23, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(2, 10)  | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_FLYMODE]           = OSD_POS(17, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(10, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]         &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_1]       &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_THROTTLE_POS]       &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_CROSSHAIRS]         &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_HORIZON_SIDEBARS]   &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ARTIFICIAL_HORIZON] &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_CURRENT_DRAW]       &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_MAH_DRAWN]          &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_GPS_SPEED]          &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_GPS_LON]            &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_GPS_LAT]            &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_GPS_SATS]           &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_HOME_DIR]           &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_HOME_DIST]          &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_COMPASS_BAR]        &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ALTITUDE]           &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ROLL_PIDS]          &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_PITCH_PIDS]         &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_YAW_PIDS]           &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_DEBUG]              &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_POWER]              &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_PIDRATE_PROFILE]    &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]           &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_AVG_CELL_VOLTAGE]   &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_PITCH_ANGLE]        &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ROLL_ANGLE]         &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_USAGE]    &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_DISARMED]           &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_NUMERICAL_HEADING]  &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_NUMERICAL_VARIO]    &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ESC_TMP]            &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ESC_RPM]            &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_G_FORCE]            &= ~OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_FLIP_ARROW]         &= ~OSD_PROFILE_1_FLAG;

    modeActivationConditionsMutable(0)->modeId           = BOXANGLE;
    modeActivationConditionsMutable(0)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(0)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);

    analyzeModeActivationConditions();

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
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]        = OSD_POS(2, 11)  | OSD_PROFILE_1_FLAG;
#endif
}
#endif
