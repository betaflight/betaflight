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
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_power.h"

#include "config/feature.h"

#include "sensors/battery.h"
#include "sensors/current.h"
#include "sensors/voltage.h"

#include "fc/config.h"

voltageMeterSource_e batteryConfig_voltageMeterSource;
currentMeterSource_e batteryConfig_currentMeterSource;

uint16_t batteryConfig_vbatmaxcellvoltage;

uint8_t voltageSensorADCConfig_vbatscale;

int16_t currentSensorADCConfig_scale;
int16_t currentSensorADCConfig_offset;

#ifdef USE_VIRTUAL_CURRENT_METER
int16_t currentSensorVirtualConfig_scale;
int16_t currentSensorVirtualConfig_offset;
#endif

static long cmsx_Power_onEnter(void)
{
    batteryConfig_voltageMeterSource = batteryConfig()->voltageMeterSource;
    batteryConfig_currentMeterSource = batteryConfig()->currentMeterSource;

    batteryConfig_vbatmaxcellvoltage = batteryConfig()->vbatmaxcellvoltage;

    voltageSensorADCConfig_vbatscale = voltageSensorADCConfig(0)->vbatscale;

    currentSensorADCConfig_scale = currentSensorADCConfig()->scale;
    currentSensorADCConfig_offset = currentSensorADCConfig()->offset;

#ifdef USE_VIRTUAL_CURRENT_METER
    currentSensorVirtualConfig_scale = currentSensorVirtualConfig()->scale;
    currentSensorVirtualConfig_offset = currentSensorVirtualConfig()->offset;
#endif

    return 0;
}

static long cmsx_Power_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    batteryConfigMutable()->voltageMeterSource = batteryConfig_voltageMeterSource;
    batteryConfigMutable()->currentMeterSource = batteryConfig_currentMeterSource;

    batteryConfigMutable()->vbatmaxcellvoltage = batteryConfig_vbatmaxcellvoltage;

    voltageSensorADCConfigMutable(0)->vbatscale = voltageSensorADCConfig_vbatscale;

    currentSensorADCConfigMutable()->scale = currentSensorADCConfig_scale;
    currentSensorADCConfigMutable()->offset = currentSensorADCConfig_offset;

#ifdef USE_VIRTUAL_CURRENT_METER
    currentSensorVirtualConfigMutable()->scale = currentSensorVirtualConfig_scale;
    currentSensorVirtualConfigMutable()->offset = currentSensorVirtualConfig_offset;
#endif

    return 0;
}

static const OSD_Entry cmsx_menuPowerEntries[] =
{
    { "-- POWER --", OME_Label, NULL, NULL, 0},

    { "V METER", OME_TAB, NULL, &(OSD_TAB_t){ &batteryConfig_voltageMeterSource, VOLTAGE_METER_COUNT - 1, voltageMeterSourceNames }, 0 },
    { "I METER", OME_TAB, NULL, &(OSD_TAB_t){ &batteryConfig_currentMeterSource, CURRENT_METER_COUNT - 1, currentMeterSourceNames }, 0 },

    { "VBAT CLMAX", OME_UINT16, NULL, &(OSD_UINT16_t) { &batteryConfig_vbatmaxcellvoltage, VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX, 1 }, 0 },

    { "VBAT SCALE", OME_UINT8, NULL, &(OSD_UINT8_t){ &voltageSensorADCConfig_vbatscale, VBAT_SCALE_MIN, VBAT_SCALE_MAX, 1 }, 0 },

    { "IBAT SCALE", OME_INT16, NULL, &(OSD_INT16_t){ &currentSensorADCConfig_scale, -16000, 16000, 5 }, 0 },
    { "IBAT OFFSET", OME_INT16, NULL, &(OSD_INT16_t){ &currentSensorADCConfig_offset, -16000, 16000, 5 }, 0 },

#ifdef USE_VIRTUAL_CURRENT_METER
    { "IBAT VIRT SCALE", OME_INT16, NULL, &(OSD_INT16_t){ &currentSensorVirtualConfig_scale, -16000, 16000, 5 }, 0 },
    { "IBAT VIRT OFFSET", OME_INT16, NULL, &(OSD_INT16_t){ &currentSensorVirtualConfig_offset, -16000, 16000, 5 }, 0 },
#endif

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuPower = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUPWR",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Power_onEnter,
    .onExit = cmsx_Power_onExit,
    .entries = cmsx_menuPowerEntries
};

#endif
