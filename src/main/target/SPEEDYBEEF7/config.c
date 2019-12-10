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

#include "flight/mixer.h"
#include "osd/osd.h"
#include "io/serial.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "pg/motor.h"
#include "target.h"

#include "config_helper.h"

#define BLUETOOTH_MSP_UART          SERIAL_PORT_USART3
#define BLUETOOTH_MSP_BAUDRATE      BAUD_19200
#define TELEMETRY_UART              SERIAL_PORT_SOFTSERIAL1

static targetSerialPortFunction_t targetSerialPortFunction[] = {
    { TELEMETRY_UART, FUNCTION_TELEMETRY_SMARTPORT },
};

void targetConfiguration(void)
{
    pinioConfigMutable()->config[0] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioBoxConfigMutable()->permanentId[0] = BOXARM;
    
    serialPortConfig_t *bluetoothMspUART = serialFindPortConfigurationMutable(BLUETOOTH_MSP_UART);
    if (bluetoothMspUART) {
        bluetoothMspUART->functionMask = FUNCTION_MSP;
        bluetoothMspUART->msp_baudrateIndex = BLUETOOTH_MSP_BAUDRATE;
    }

    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));

    osdElementConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(1, 12) | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_ALTITUDE] = OSD_POS(1, 11) | OSD_PROFILE_1_FLAG;

    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
}

#endif
