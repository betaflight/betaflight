/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform/adc_impl.h"

#undef TEMPSENSOR_CAL1_ADDR
#undef TEMPSENSOR_CAL2_ADDR
#define TEMPSENSOR_CAL1_ADDR (&mockTemperatureCalibration[0])
#define TEMPSENSOR_CAL2_ADDR (&mockTemperatureCalibration[1])

#include "../../platform/UM32/adc_um324xx.c"
