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

#include <stdint.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "io/serial.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "target.h"

#define BLUETOOTH_MSP_UART          SERIAL_PORT_UART5
#define BLUETOOTH_MSP_BAUDRATE      BAUD_19200

void targetConfiguration(void)
{
    pinioConfigMutable()->config[0] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioBoxConfigMutable()->permanentId[0] = BOXARM;

    serialPortConfig_t *bluetoothMspUART = serialFindPortConfiguration(BLUETOOTH_MSP_UART);
    if (bluetoothMspUART) {
        bluetoothMspUART->functionMask = FUNCTION_MSP;
        bluetoothMspUART->msp_baudrateIndex = BLUETOOTH_MSP_BAUDRATE;
    }
}

#endif
