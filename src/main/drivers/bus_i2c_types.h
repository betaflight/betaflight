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
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// i2cResource_t is an opaque data type which represents an I2C
// peripheral, implemented differently across MCU families.
// Code in src/main references I2C peripherals through i2cResource_t pointers;
// platform code casts these to the native MCU type (e.g. I2C_TypeDef*).
typedef struct i2cResource_s i2cResource_t;

// Opaque HAL handle type for I2C.
typedef struct i2cHalHandle_s i2cHalHandle_t;
