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

#include "blackbox/blackbox.h"
#include "config/feature.h"
#include "drivers/sdcard.h"
#include "io/serial.h"
#include "pg/pg.h"
#include "pg/rx.h"
#include "sensors/battery.h"
#include "sensors/voltage.h"

#include "config_helper.h"

void targetMigrationConfiguration(void)
{
    // #define SDCARD_DETECT_INVERTED
    // %# set sdcard_card_detect_inverted = ON # Not available yet
    sdcardConfigMutable()->cardDetectInverted = 1;

    // #define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
    // %set battery_meter = ADC
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;

    // #define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
    // %set blackbox_device = SDCARD
    blackboxConfigMutable()->device = BLACKBOX_DEVICE_SDCARD;

    // #define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
    // %feature RX_SERIAL
    featureConfigMutable()->enabledFeatures |= FEATURE_RX_SERIAL;

    // #define SERIALRX_PROVIDER       SERIALRX_SBUS
    // %set serialrx_provider = SBUS
    rxConfigMutable()->serialrx_provider = 2;

    // #define SERIALRX_UART           SERIAL_PORT_USART6
    // %serial 5 64 115200 0 115200 115200
    static targetSerialPortFunction_t targetMigrationSerialPortFunction[] = {
        { SERIAL_PORT_USART6, FUNCTION_RX_SERIAL },
    };
    targetSerialPortFunctionConfig(targetMigrationSerialPortFunction, ARRAYLEN(targetMigrationSerialPortFunction));
}
