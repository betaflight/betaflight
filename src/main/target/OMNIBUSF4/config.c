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

#ifdef USE_TARGET_CONFIG

#include "config_helper.h"

#include "io/serial.h"

#include "pg/max7456.h"
#include "pg/pg.h"

#ifdef SYNERGYF4
#include "io/vtx.h"
#include "io/ledstrip.h"
#include "fc/config.h"
#include "pg/piniobox.h"
#include "common/axis.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "drivers/pwm_output.h"
static targetSerialPortFunction_t targetSerialPortFunction[] = {
    { SERIAL_PORT_USART1, FUNCTION_RX_SERIAL },
    { SERIAL_PORT_USART3,  FUNCTION_VTX_SMARTAUDIO },
};
#endif
#ifdef EXUAVF4PRO
static targetSerialPortFunction_t targetSerialPortFunction[] = {
    { SERIAL_PORT_USART1, FUNCTION_TELEMETRY_SMARTPORT },
    { SERIAL_PORT_USART3, FUNCTION_VTX_TRAMP },
    { SERIAL_PORT_UART4,  FUNCTION_RCDEVICE },
    { SERIAL_PORT_USART6, FUNCTION_RX_SERIAL },
};
#endif

void targetConfiguration(void)
{
#ifdef OMNIBUSF4BASE
    // OMNIBUS F4 AIO (1st gen) has a AB7456 chip that is detected as MAX7456
    max7456ConfigMutable()->clockConfig = MAX7456_CLOCK_CONFIG_FULL;
#endif

#ifdef EXUAVF4PRO
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));
#endif
#ifdef SYNERGYF4
    pinioBoxConfigMutable()->permanentId[0] = 39;
    vtxSettingsConfigMutable()->pitModeFreq = 0;
    ledStripStatusModeConfigMutable()->ledConfigs[0] = DEFINE_LED(0, 0, 0, 0, LF(COLOR), LO(VTX), 0);
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
    gyroConfigMutable()->gyro_sync_denom = 1;  // 8kHz gyro
    pidConfigMutable()->pid_process_denom = 1; // 8kHz PID
#endif
}
#endif
