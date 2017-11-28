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

#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"

#include "io/osd.h"

#define VBAT_SCALE       113

#ifdef USE_TARGET_CONFIG
void targetConfiguration(void)
{
    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale = VBAT_SCALE;
    barometerConfigMutable()->baro_hardware = 0;
    compassConfigMutable()->mag_hardware = 0;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(12, 1) | VISIBLE_FLAG;
}
#endif
