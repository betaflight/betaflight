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

#ifdef USE_INVERTER

#include "io/serial.h"
#include "drivers/io.h"
#include "drivers/serial.h"
#include "drivers/serial_impl.h"


#include "inverter.h"

static const serialPinConfig_t *pSerialPinConfig;

static IO_t findInverterPin(serialPortIdentifier_e identifier)
{
    const int resourceIndex = serialResourceIndex(identifier);
    if (resourceIndex >= 0 && resourceIndex < (int)ARRAYLEN(pSerialPinConfig->ioTagInverter)) {
        return IOGetByTag(pSerialPinConfig->ioTagInverter[resourceIndex]);
    } else {
        return NULL;
    }
}

static void inverterSet(IO_t pin,  bool on)
{
    IOWrite(pin, on);
}

static void initInverter(serialPortIdentifier_e identifier)
{
    IO_t pin = findInverterPin(identifier);
    if (pin) {
        const int ownerIndex = serialOwnerIndex(identifier);
        // only UART supports inverter, so OWNER_INVERTER+ownerIndex does work
        IOInit(pin, OWNER_INVERTER, ownerIndex);
        IOConfigGPIO(pin, IOCFG_OUT_PP);
        inverterSet(pin, false);
    }
}

void initInverters(const serialPinConfig_t *serialPinConfigToUse)
{
    pSerialPinConfig = serialPinConfigToUse;

    for (unsigned i = 0; i < ARRAYLEN(serialPortIdentifiers); i++) {
        // it is safe to pass port without inverter
        initInverter(serialPortIdentifiers[i]);
    }
}

void enableInverter(serialPortIdentifier_e identifier, bool on)
{
    const IO_t pin = findInverterPin(identifier);
    if (pin) {
        inverterSet(pin, on);
    }
}

#endif // USE_INVERTER
