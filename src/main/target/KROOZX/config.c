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

#include "platform.h"

#include "common/axis.h"
#include "drivers/io.h"
#include "osd/osd.h"
#include "pg/pinio.h"
#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"

#define VBAT_SCALE       113
#define OSD_CH_SWITCH           PC5

#ifdef USE_TARGET_CONFIG
void targetConfiguration(void)
{
    pinioConfigMutable()->ioTag[0] = IO_TAG(OSD_CH_SWITCH);
    pinioConfigMutable()->config[0] = PINIO_CONFIG_MODE_OUT_PP; // Default state is LOW

    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale = VBAT_SCALE;
    barometerConfigMutable()->baro_hardware = 0;
    compassConfigMutable()->mag_hardware = 0;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(12, 1) | OSD_PROFILE_1_FLAG;
}
#endif
