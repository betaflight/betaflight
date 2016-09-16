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

#pragma once

#define I2C1_OVERCLOCK true
#define I2C2_OVERCLOCK true


/* STM32F4 specific settings that apply to all F4 targets */
#ifdef STM32F4

#define MAX_AUX_CHANNELS 99
#define TASK_GYROPID_DESIRED_PERIOD 125
#define SCHEDULER_DELAY_LIMIT 10
#define USE_SLOW_SERIAL_CLI
#define I2C3_OVERCLOCK true

#else /* when not an F4 */

#define MAX_AUX_CHANNELS 6
#define TASK_GYROPID_DESIRED_PERIOD 1000
#define SCHEDULER_DELAY_LIMIT 100

#endif

#ifdef STM32F1
// Using RX DMA disables the use of receive callbacks
#define USE_UART1_RX_DMA

#endif

#define SERIAL_RX
#define USE_CLI

#if (FLASH_SIZE > 64)
#define BLACKBOX
#define GPS
#define TELEMETRY
#define USE_SERVOS
#endif

#if (FLASH_SIZE > 128)
#define DISPLAY
#else
#define SKIP_CLI_COMMAND_HELP
#define SKIP_RX_MSP
#endif
