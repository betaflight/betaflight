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

// spiResource_t is an opaque data type which represents an SPI
// peripheral, implemented differently across MCU families.
// Code in src/main references SPI peripherals through spiResource_t pointers;
// platform code casts these to the native MCU type (e.g. SPI_TypeDef*).
typedef struct spiResource_s spiResource_t;

// Opaque HAL handle type for SPI.
typedef struct spiHalHandle_s spiHalHandle_t;
