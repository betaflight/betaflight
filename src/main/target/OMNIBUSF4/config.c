/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "config/config_master.h"
#include "config/feature.h"

#include "blackbox/blackbox_io.h"

#include "drivers/timer.h"

#include "hardware_revision.h"

// called on reset
void targetConfiguration(master_t *config)
{
    // default to serialrx / sbus
    config->rxConfig.serialrx_provider = SERIALRX_SBUS;
    // RX is on uart1 by default
    //config->serialConfig.portConfigs[0].functionMask = FUNCTION_RX_SERIAL;

    if (hardwareRevision == OMNIBUSF4V1) {
        // set the right PWM/PPM inputs
        config->ppmConfig.ioTag = timerHardware[0].tag;
        config->pwmConfig.ioTags[0] = timerHardware[0].tag;
        config->pwmConfig.ioTags[1] = timerHardware[1].tag;
        // SDCARD is not present on the V1
        intFeatureClear(FEATURE_SDCARD, &config->enabledFeatures);
        config->blackboxConfig.device = BLACKBOX_DEVICE_FLASH;
    } else {
        // set the right PWM/PPM inputs
        config->ppmConfig.ioTag = timerHardware[2].tag;
        config->pwmConfig.ioTags[0] = timerHardware[2].tag;
        config->pwmConfig.ioTags[1] = timerHardware[3].tag;
        // use the SDCARD on the V2
        config->blackboxConfig.device = BLACKBOX_DEVICE_SDCARD;
        // 0 for baro -> AUTO
        config->sensorSelectionConfig.baro_hardware = 0;
    }
    // all the rest of the PWM inputs are the same, set those
    int inputIndex = 2;
    for (int i = 4; i < USABLE_TIMER_CHANNEL_COUNT - 4 && inputIndex < PWM_INPUT_PORT_COUNT - 2; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_PWM) {
            config->pwmConfig.ioTags[inputIndex] = timerHardware[i].tag;
            inputIndex++;
        }
    }
}

// called every startup
void targetValidateConfiguration(master_t *config)
{
    if (hardwareRevision == OMNIBUSF4V1) {
        // make sure the SDCARD cannot be turned on for the v1
        intFeatureClear(FEATURE_SDCARD, &config->enabledFeatures);
        if (config->blackboxConfig.device == BLACKBOX_DEVICE_SDCARD) {
            config->blackboxConfig.device = BLACKBOX_DEVICE_FLASH;
        }
    }
}
