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

#pragma once

#if defined(USE_OSD_HD)                         // HD screen
  #define TR2(x, y) y
#elif defined(USE_OSD) || defined(USE_OSD_SD)   // PAL or NTCS screen
  #define TR2(x, y) x
#else
  #define TR2(x, y) x
#endif

#include "bf_locale.h"                          // located in locales/xx, default locales/en, managed by make

// defines to be untranslated
#define STR_COMMA	              ","
#define STR_PERIOD	            "."

#define STR_VISUAL_BEEP         "  * * * *"

#define STR_MSP_API_NAME        "MSP API:"
