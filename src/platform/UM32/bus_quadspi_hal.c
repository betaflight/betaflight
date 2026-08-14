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
 *
 * Author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_QUADSPI

#include "drivers/bus_quadspi.h"
#include "drivers/bus_quadspi_impl.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"

#include "pg/bus_quadspi.h"
#include "drivers/bus_quadspi_impl.h"
// Provide platform-specific hardware table for STM32
const quadSpiHardware_t quadSpiHardware[QUADSPIDEV_COUNT] = {

};

// Provide platform-specific pin configuration for STM32
void quadSpiPinConfigure(const quadSpiConfig_t *pConfig)
{

}

static void Error_Handler(void) { while (1); }

void quadSpiInitDevice(quadSpiDevice_e device)
{

}

static const uint32_t quadSpi_addressSizeMap[] = {

};

static uint32_t quadSpi_addressSizeFromValue(uint8_t addressSize)
{
   
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
LOCAL_UNUSED_FUNCTION static bool quadSpiIsBusBusy(QUADSPI_TypeDef *instance)
{

    return false;
}

#define QUADSPI_DEFAULT_TIMEOUT 10

static void quadSpiSelectDevice(QUADSPI_TypeDef *instance)
{

}

static void quadSpiDeselectDevice(QUADSPI_TypeDef *instance)
{

}

bool quadSpiTransmit1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{


    return true;
}

bool quadSpiReceive1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{


    return true;
}

bool quadSpiReceive4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{


    return true;
}

bool quadSpiReceiveWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{


    return true;
}

bool quadSpiReceiveWithAddress4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{


    return true;
}
bool quadSpiTransmitWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{


    return true;
}

bool quadSpiTransmitWithAddress4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{

    return true;
}

bool quadSpiInstructionWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize)
{


    return true;
}

bool quadSpiInstructionWithData1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{


    return true;
}

void quadSpiSetDivisor(QUADSPI_TypeDef *instance, uint16_t divisor)
{

}
#endif
