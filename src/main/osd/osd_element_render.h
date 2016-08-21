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

#define OSD_FLIGHT_MODE_ACRO    (1 << 0)
#define OSD_FLIGHT_MODE_HORIZON (1 << 1)
#define OSD_FLIGHT_MODE_ANGLE   (1 << 2)

typedef struct voltageAndName_s {
    char *name;
    int16_t voltage;
} voltageAndName_t;

void osdElementRender_duration(const element_t *element, elementDataProviderFn dataFn);
void osdElementRender_mahDrawn(const element_t *element, elementDataProviderFn dataFn);
void osdElementRender_amperage(const element_t *element, elementDataProviderFn dataFn);
void osdElementRender_voltage(const element_t *element, elementDataProviderFn dataFn);
void osdElementRender_flightMode(const element_t *element, elementDataProviderFn dataFn);
void osdElementRender_indicatorMag(const element_t *element, elementDataProviderFn dataFn);
void osdElementRender_indicatorBaro(const element_t *element, elementDataProviderFn dataFn);
void osdElementRender_rssi(const element_t *element, elementDataProviderFn dataFn);
