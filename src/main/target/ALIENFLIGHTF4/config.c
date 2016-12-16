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

#include "drivers/sensor.h"
#include "drivers/compass.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "io/serial.h"

#include "telemetry/telemetry.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#include "hardware_revision.h"

#define CURRENTOFFSET 2500                      // ACS712/714-30A - 0A = 2.5V
#define CURRENTSCALE -667                       // ACS712/714-30A - 66.666 mV/A inverted mode

#define BRUSHED_MOTORS_PWM_RATE 32000           // 32kHz

// alternative defaults settings for AlienFlight targets
void targetConfiguration(master_t *config)
{
    config->batteryConfig.currentMeterOffset = CURRENTOFFSET;
    config->batteryConfig.currentMeterScale = CURRENTSCALE;
    config->compassConfig.mag_hardware = MAG_NONE;            // disabled by default

    if (hardwareMotorType == MOTOR_BRUSHED) {
        config->motorConfig.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        config->pidConfig.pid_process_denom = 1;
    }

    if (hardwareRevision == AFF4_REV_1) {
        config->rxConfig.serialrx_provider = SERIALRX_SPEKTRUM2048;
        config->rxConfig.spektrum_sat_bind = 5;
        config->rxConfig.spektrum_sat_bind_autoreset = 1;
    } else {
        config->rxConfig.serialrx_provider = SERIALRX_SBUS;
        config->rxConfig.sbus_inversion = 0;
        config->serialConfig.portConfigs[findSerialPortIndexByIdentifier(TELEMETRY_UART)].functionMask = FUNCTION_TELEMETRY_FRSKY;
        config->telemetryConfig.telemetry_inversion = 0;
        intFeatureSet(FEATURE_CURRENT_METER | FEATURE_VBAT | FEATURE_TELEMETRY, &config->enabledFeatures);
    }

    config->profile[0].pidProfile.P8[ROLL] = 53;
    config->profile[0].pidProfile.I8[ROLL] = 45;
    config->profile[0].pidProfile.D8[ROLL] = 52;
    config->profile[0].pidProfile.P8[PITCH] = 53;
    config->profile[0].pidProfile.I8[PITCH] = 45;
    config->profile[0].pidProfile.D8[PITCH] = 52;
    config->profile[0].pidProfile.P8[YAW] = 64;
    config->profile[0].pidProfile.D8[YAW] = 18;

    config->customMotorMixer[0] = (motorMixer_t){ 1.0f, -0.414178f,  1.0f, -1.0f };    // REAR_R
    config->customMotorMixer[1] = (motorMixer_t){ 1.0f, -0.414178f, -1.0f,  1.0f };    // FRONT_R
    config->customMotorMixer[2] = (motorMixer_t){ 1.0f,  0.414178f,  1.0f,  1.0f };    // REAR_L
    config->customMotorMixer[3] = (motorMixer_t){ 1.0f,  0.414178f, -1.0f, -1.0f };    // FRONT_L
    config->customMotorMixer[4] = (motorMixer_t){ 1.0f, -1.0f, -0.414178f, -1.0f };    // MIDFRONT_R
    config->customMotorMixer[5] = (motorMixer_t){ 1.0f,  1.0f, -0.414178f,  1.0f };    // MIDFRONT_L
    config->customMotorMixer[6] = (motorMixer_t){ 1.0f, -1.0f,  0.414178f,  1.0f };    // MIDREAR_R
    config->customMotorMixer[7] = (motorMixer_t){ 1.0f,  1.0f,  0.414178f, -1.0f };    // MIDREAR_L
}
