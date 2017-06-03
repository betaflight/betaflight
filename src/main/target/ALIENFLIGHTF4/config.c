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

#include <platform.h>

#include "common/axis.h"

#include "config/config_master.h"
#include "config/feature.h"

#include "drivers/sensor.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "hardware_revision.h"

#include "rx/rx.h"

#include "io/serial.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "telemetry/telemetry.h"

#define CURRENTOFFSET 2500                      // ACS712/714-30A - 0A = 2.5V
#define CURRENTSCALE -667                       // ACS712/714-30A - 66.666 mV/A inverted mode

#define BRUSHED_MOTORS_PWM_RATE 32000           // 32kHz

// alternative defaults settings for AlienFlight targets
void targetConfiguration(void)
{
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    batteryConfigMutable()->currentMeterOffset = CURRENTOFFSET;
    batteryConfigMutable()->currentMeterScale = CURRENTSCALE;

    if (hardwareMotorType == MOTOR_BRUSHED) {
        motorConfigMutable()->motorPwmProtocol = PWM_TYPE_BRUSHED;
        motorConfigMutable()->motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1000;
    }
    if (hardwareRevision == AFF4_REV_1) {
        rxConfigMutable()->serialrx_provider = SERIALRX_SPEKTRUM2048;
        rxConfigMutable()->spektrum_sat_bind = 5;
        rxConfigMutable()->spektrum_sat_bind_autoreset = 1;
    } else {
        rxConfigMutable()->serialrx_provider = SERIALRX_SBUS;
        rxConfigMutable()->sbus_inversion = 0;
        serialConfigMutable()->portConfigs[3].functionMask = FUNCTION_TELEMETRY_FRSKY;
        telemetryConfigMutable()->telemetry_inversion = 0;
        featureConfigMutable()->enabledFeatures |= (FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TELEMETRY);
    }

    pidProfileMutable()->bank_mc.pid[ROLL].P = 53;
    pidProfileMutable()->bank_mc.pid[ROLL].I = 45;
    pidProfileMutable()->bank_mc.pid[ROLL].D = 52;
    pidProfileMutable()->bank_mc.pid[PITCH].P = 53;
    pidProfileMutable()->bank_mc.pid[PITCH].I = 45;
    pidProfileMutable()->bank_mc.pid[PITCH].D = 52;
    pidProfileMutable()->bank_mc.pid[YAW].P = 64;
    pidProfileMutable()->bank_mc.pid[YAW].D = 18;

    *customMotorMixerMutable(0) = (motorMixer_t){ 1.0f, -0.414178f,  1.0f, -1.0f };    // REAR_R
    *customMotorMixerMutable(1) = (motorMixer_t){ 1.0f, -0.414178f, -1.0f,  1.0f };    // FRONT_R
    *customMotorMixerMutable(2) = (motorMixer_t){ 1.0f,  0.414178f,  1.0f,  1.0f };    // REAR_L
    *customMotorMixerMutable(3) = (motorMixer_t){ 1.0f,  0.414178f, -1.0f, -1.0f };    // FRONT_L
    *customMotorMixerMutable(4) = (motorMixer_t){ 1.0f, -1.0f, -0.414178f, -1.0f };    // MIDFRONT_R
    *customMotorMixerMutable(5) = (motorMixer_t){ 1.0f,  1.0f, -0.414178f,  1.0f };    // MIDFRONT_L
    *customMotorMixerMutable(6) = (motorMixer_t){ 1.0f, -1.0f,  0.414178f,  1.0f };    // MIDREAR_R
    *customMotorMixerMutable(7) = (motorMixer_t){ 1.0f,  1.0f,  0.414178f, -1.0f };    // MIDREAR_L
}
