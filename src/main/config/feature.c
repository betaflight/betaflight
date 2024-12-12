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

#include "platform.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(featureConfig_t, featureConfig, PG_FEATURE_CONFIG, 1);

PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
    .enabledFeatures = DEFAULT_FEATURES | DEFAULT_RX_FEATURE | FEATURE_ANTI_GRAVITY | FEATURE_AIRMODE,
);

// bitmask of features that are supported in current build configuration
uint32_t featuresSupportedByBuild =
    0
#ifdef USE_RX_PPM
    | FEATURE_RX_PPM
#endif
    | FEATURE_INFLIGHT_ACC_CAL // always available
#ifdef USE_SERIALRX
    | FEATURE_RX_SERIAL
#endif
    | FEATURE_MOTOR_STOP // always available
#ifdef USE_SERVOS
    | FEATURE_SERVO_TILT
#endif
#ifdef USE_SOFTSERIAL
    | FEATURE_SOFTSERIAL
#endif
#ifdef USE_GPS
    | FEATURE_GPS
#endif
#ifdef USE_RANGEFINDER
    | FEATURE_RANGEFINDER
#endif
#ifdef USE_OPTICALFLOW
    | FEATURE_OPTICALFLOW
#endif
#ifdef USE_TELEMETRY
    | FEATURE_TELEMETRY
#endif
    | FEATURE_3D // always available
#ifdef USE_PWM
    | FEATURE_RX_PARALLEL_PWM
#endif
#ifdef USE_RX_MSP
    | FEATURE_RX_MSP
#endif
#ifdef USE_ADC
    | FEATURE_RSSI_ADC
#endif
#ifdef USE_LED_STRIP  // but cms will try to use it
    | FEATURE_LED_STRIP
#endif
#ifdef USE_DASHBOARD
    | FEATURE_DASHBOARD
#endif
#ifdef USE_OSD
    | FEATURE_OSD
#endif
#ifdef USE_SERVOS
    | FEATURE_CHANNEL_FORWARDING
#endif
#ifdef USE_TRANSPONDER
    | FEATURE_TRANSPONDER
#endif
    | FEATURE_AIRMODE // always available
#ifdef USE_RX_SPI
    | FEATURE_RX_SPI
#endif
#ifdef USE_ESC_SENSOR
    | FEATURE_ESC_SENSOR
#endif
    | FEATURE_ANTI_GRAVITY // always available
    ;

static uint32_t runtimeFeatureMask;

void featureInit(void)
{
    runtimeFeatureMask = featureConfig()->enabledFeatures;
}

static void featureSet(const uint32_t mask, uint32_t *features)
{
    *features |= mask;
}

static void featureClear(const uint32_t mask, uint32_t *features)
{
    *features &= ~(mask);
}

// Determines if the feature is enabled (active) in the runtime state.
// This is the primary funciton used by code that wants to know if a
// feature is available.
bool featureIsEnabled(const uint32_t mask)
{
    return runtimeFeatureMask & mask;
}

// Determines if the feature is configured (set in the configuration). Doesn't mean the
// feature is active in the runtime. This function *SHOULD ONLY* be used in the config check
// performed at startup and when writing to EEPROM.
bool featureIsConfigured(const uint32_t mask)
{
    return featureConfig()->enabledFeatures & mask;
}

// Updates the configuration *AND* runtime state of a feature.
// Used *ONLY* by the config check process that runs at startup and EEPROM save.
void featureEnableImmediate(const uint32_t mask)
{
    featureSet(mask, &featureConfigMutable()->enabledFeatures);
    featureSet(mask, &runtimeFeatureMask);
}

// Updates the configuration *AND* runtime state of a feature.
// Used *ONLY* by the config check process that runs at startup and EEPROM save.
void featureDisableImmediate(const uint32_t mask)
{
    featureClear(mask, &featureConfigMutable()->enabledFeatures);
    featureClear(mask, &runtimeFeatureMask);
}

// Sets the configuration state of the feature and *DOES NOT* change the runtime state.
// For example, used by the CLI "feature" command. Use this function if you want to
// enable a feature that will be active after save/reboot.
void featureConfigSet(const uint32_t mask)
{
    featureSet(mask, &featureConfigMutable()->enabledFeatures);
}

// Sets the configuration state of the feature and *DOES NOT* change the runtime state.
// For example, used by the CLI "feature" command. Use this function if you want to
// disable a feature after a save/reboot.
void featureConfigClear(const uint32_t mask)
{
    featureClear(mask, &featureConfigMutable()->enabledFeatures);
}

// Sets the configuration state of all features and *DOES NOT* change the runtime state.
// For example, used by MSP to update all the configured features in one go.
void featureConfigReplace(const uint32_t mask)
{
    featureConfigMutable()->enabledFeatures = mask;
}
