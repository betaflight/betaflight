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

#include "stdbool.h"
#include "stdint.h"

#include "platform.h"

#include "build_config.h"

#ifdef STM32F1
#warning STM32F1 based targets are unsupported as of Betaflight 3.3.
#endif

#ifdef STM32F3
#warning STM32F3 based targets are unsupported as of Betaflight 4.1.
#endif

#ifdef USE_CLI_DEBUG_PRINT
#warning Do not use USE_CLI_DEBUG_PRINT for production builds.
#endif
