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

#define USE_PARAMETER_GROUPS
// type conversion warnings.
// -Wconversion can be turned on to enable the process of eliminating these warnings
//#pragma GCC diagnostic warning "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
// -Wpadded can be turned on to check padding of structs
//#pragma GCC diagnostic warning "-Wpadded"

//#define SCHEDULER_DEBUG // define this to use scheduler debug[] values. Undefined by default for performance reasons
#define DEBUG_MODE DEBUG_NONE // change this to change initial debug mode

#define I2C1_OVERCLOCK true
#define I2C2_OVERCLOCK true

#define MAX_PROFILE_COUNT 0

#ifdef STM32F1
#define MINIMAL_CLI
#endif

#ifdef STM32F3
#define MINIMAL_CLI
#endif

#ifdef STM32F4
#define I2C3_OVERCLOCK true
#endif

#ifdef STM32F7
#define I2C3_OVERCLOCK true
#define I2C4_OVERCLOCK true
#endif

#if defined(STM32F4) || defined(STM32F7)
#define SCHEDULER_DELAY_LIMIT           10
#else
#define SCHEDULER_DELAY_LIMIT           100
#endif

#define FAST_CODE
#define FAST_RAM

//CLI needs FC dependencies removed before we can compile it, disabling for now
//#define USE_CLI
