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

#include "config_helper.h"

#ifdef USE_TARGET_CONFIG

void targetSerialPortFunctionConfig(targetSerialPortFunction_t *config, size_t count)
{
    for (size_t i = 0 ; i < count ; i++) {
        int index = findSerialPortIndexByIdentifier(config[i].identifier);
        if (index >= 0) {
            serialConfigMutable()->portConfigs[index].functionMask = config[i].function;
        }
    }
}
#endif
