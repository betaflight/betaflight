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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build_config.h"

#ifdef USE_CLI_DEBUG_PRINT
#warning Do not use USE_CLI_DEBUG_PRINT for production builds.
#endif

mcuTypeId_e getMcuTypeId(void)
{
    const mcuTypeInfo_t *mcuTypeInfo = getMcuTypeInfo();
    if (mcuTypeInfo) {
        return mcuTypeInfo->id;
    }
    return MCU_TYPE_UNKNOWN;
}

const char *getMcuTypeName(void)
{
    const mcuTypeInfo_t *mcuTypeInfo = getMcuTypeInfo();
    if (mcuTypeInfo) {
        return mcuTypeInfo->name;
    }
    return "Unknown";
}
