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

/*
 * Minimal sdkconfig.h for bare-metal Betaflight ESP32-S3 build.
 * ESP-IDF headers require this file but we run without FreeRTOS.
 * Only defines needed to satisfy header-level #ifdef checks.
 */

#pragma once

/* No FreeRTOS - disable all RTOS features */
/* #undef CONFIG_FREERTOS_USE_LIST_DATA_INTEGRITY_CHECK_BYTES */
/* #undef CONFIG_FREERTOS_USE_TRACE_FACILITY */

/* Disable compiler optimization assertion tweaks */
/* #undef CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_SILENT */

/* ESP32-S3 target */
#define CONFIG_IDF_TARGET_ESP32S3  1
