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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/serial.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "inverter.h"

#ifdef USE_INVERTER

#ifdef INVERTER_PIN_UART1
#define INVERTER_IOTAG_UART1 IO_TAG(INVERTER_PIN_UART1)
#else
#define INVERTER_IOTAG_UART1 IO_TAG_NONE
#endif

#ifdef INVERTER_PIN_UART2
#define INVERTER_IOTAG_UART2 IO_TAG(INVERTER_PIN_UART2)
#else
#define INVERTER_IOTAG_UART2 IO_TAG_NONE
#endif

#ifdef INVERTER_PIN_UART3
#define INVERTER_IOTAG_UART3 IO_TAG(INVERTER_PIN_UART3)
#else
#define INVERTER_IOTAG_UART3 IO_TAG_NONE
#endif

#ifdef INVERTER_PIN_UART4
#define INVERTER_IOTAG_UART4 IO_TAG(INVERTER_PIN_UART4)
#else
#define INVERTER_IOTAG_UART4 IO_TAG_NONE
#endif

#ifdef INVERTER_PIN_UART5
#define INVERTER_IOTAG_UART5 IO_TAG(INVERTER_PIN_UART5)
#else
#define INVERTER_IOTAG_UART5 IO_TAG_NONE
#endif

#ifdef INVERTER_PIN_UART6
#define INVERTER_IOTAG_UART6 IO_TAG(INVERTER_PIN_UART6)
#else
#define INVERTER_IOTAG_UART6 IO_TAG_NONE
#endif

const inverterConfig_t *pInverterConfig;

PG_REGISTER_WITH_RESET_TEMPLATE(inverterConfig_t, inverterConfig, PG_INVERTER_CONFIG, 0);

PG_RESET_TEMPLATE(inverterConfig_t, inverterConfig,
    .ioTag = {
        INVERTER_IOTAG_UART1,
        INVERTER_IOTAG_UART2,
        INVERTER_IOTAG_UART3,
        INVERTER_IOTAG_UART4,
        INVERTER_IOTAG_UART5,
        INVERTER_IOTAG_UART6,
    },
);

static void inverterSet(IO_t pin, bool on)
{
    IOWrite(pin, on);
}

static void initInverter(ioTag_t ioTag)
{
    IO_t pin = IOGetByTag(ioTag);
    IOInit(pin, OWNER_INVERTER, 1);
    IOConfigGPIO(pin, IOCFG_OUT_PP);

    inverterSet(pin, false);
}
#endif

void initInverters(const inverterConfig_t *inverterConfigToUse)
{
#ifdef USE_INVERTER
    pInverterConfig = inverterConfigToUse;

#ifdef INVERTER_PIN_UART1
    initInverter(pInverterConfig->ioTag[0]);
#endif

#ifdef INVERTER_PIN_UART2
    initInverter(pInverterConfig->ioTag[1]);
#endif

#ifdef INVERTER_PIN_UART3
    initInverter(pInverterConfig->ioTag[2]);
#endif

#ifdef INVERTER_PIN_UART4
    initInverter(pInverterConfig->ioTag[3]);
#endif

#ifdef INVERTER_PIN_UART5
    initInverter(pInverterConfig->ioTag[4]);
#endif

#ifdef INVERTER_PIN_UART6
    initInverter(pInverterConfig->ioTag[5]);
#endif
#else
    UNUSED(inverterConfigToUse);
#endif
}

void enableInverter(USART_TypeDef *USARTx, bool on)
{
#ifdef USE_INVERTER
    IO_t pin = IO_NONE;

#ifdef INVERTER_PIN_UART1
    if (USARTx == USART1) {
        pin = IOGetByTag(pInverterConfig->ioTag[0]);
    }
#endif

#ifdef INVERTER_PIN_UART2
    if (USARTx == USART2) {
        pin = IOGetByTag(pInverterConfig->ioTag[1]);
    }
#endif

#ifdef INVERTER_PIN_UART3
    if (USARTx == USART3) {
        pin = IOGetByTag(pInverterConfig->ioTag[2]);
    }
#endif

#ifdef INVERTER_PIN_UART4
    if (USARTx == UART4) {
        pin = IOGetByTag(pInverterConfig->ioTag[3]);
    }
#endif

#ifdef INVERTER_PIN_UART5
    if (USARTx == UART5) {
        pin = IOGetByTag(pInverterConfig->ioTag[4]);
    }
#endif

#ifdef INVERTER_PIN_UART6
    if (USARTx == USART6) {
        pin = IOGetByTag(pInverterConfig->ioTag[5]);
    }
#endif

    inverterSet(pin, on);
#else
    UNUSED(USARTx);
    UNUSED(on);
#endif
}
