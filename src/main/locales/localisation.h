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

#if defined(USE_HD_EXTENDED) 
  #define TR2(x, y) y
#else
  #define TR2(x, y) x
#endif

#include "bf_locale.h"                                      // located in locales/xx, default locales/en, managed by make

extern const char STR_LOCALE[];
<<<<<<< HEAD
=======

// defines to be untranslated
#define STR_COMMA	              ","
#define STR_PERIOD	            "."

#define STR_VISUAL_BEEP         "  * * * *"

#define STR_MSP_API_NAME        "MSP API:"
>>>>>>> cd1cda2fb (Report locale in cli command - status)
