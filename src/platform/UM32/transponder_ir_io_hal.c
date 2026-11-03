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
#include <string.h>

#include "platform.h"

#ifdef USE_TRANSPONDER

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/timer.h"
#include "drivers/transponder_ir_arcitimer.h"
#include "drivers/transponder_ir_erlt.h"
#include "drivers/transponder_ir_ilap.h"

#include "drivers/transponder_ir.h"

volatile uint8_t transponderIrDataTransferInProgress = 0;

#if(0)
static IO_t transponderIO = IO_NONE;
static TIM_HandleTypeDef TimHandle;
static uint16_t timerChannel = 0;
static uint8_t output;
static uint8_t alternateFunction;
#endif

#if !(defined(UM324xF))
#error "Transponder (via HAL) not supported on this MCU."
#endif

DMA_RAM_W transponder_t transponder;
bool transponderInitialised = false;

#if(0)
FAST_IRQ_HANDLER static void TRANSPONDER_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    UNUSED(descriptor);
    transponderIrDataTransferInProgress = 0;
}
#endif

bool transponderIrHardwareInit(ioTag_t ioTag, transponder_t *transponder)
{
    UNUSED(transponder);
    if (!ioTag) {
        return false;
    }

    transponderInitialised = true;
    return true;
}

bool transponderIrInit(const ioTag_t ioTag, const transponderProvider_e provider)
{
    UNUSED(provider);

    if (!ioTag) {
        return false;
    }

    return true;
}

bool transponderIrIsReady(void)
{
    return !transponderIrDataTransferInProgress;
}

void transponderIrWaitForTransmitComplete(void)
{
#ifdef DEBUG
    static uint32_t waitCounter = 0;
#endif

    while (transponderIrDataTransferInProgress) {
#ifdef DEBUG
        waitCounter++;
#endif
    }
}

void transponderIrUpdateData(const uint8_t* transponderData)
{
    UNUSED(transponderData);
}

void transponderIrDMAEnable(transponder_t *transponder)
{
    UNUSED(transponder);

    if (!transponderInitialised) {
        return;
    }
}

void transponderIrDisable(void)
{
    if (!transponderInitialised) {
        return;
    }
}

void transponderIrTransmit(void)
{

}
#endif
