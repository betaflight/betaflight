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
#include <string.h>

#include <platform.h>

#include "dma.h"
#include "nvic.h"
#include "io.h"
#include "timer.h"
#include "transponder_ir.h"

/*
 * Implementation note:
 * Using around over 700 bytes for a transponder DMA buffer is a little excessive, likely an alternative implementation that uses a fast
 * ISR to generate the output signal dynamically based on state would be more memory efficient and would likely be more appropriate for
 * other targets.  However this approach requires very little CPU time and is just fire-and-forget.
 *
 * On an STM32F303CC 720 bytes is currently fine and that is the target for which this code was designed for.
 */
uint8_t transponderIrDMABuffer[TRANSPONDER_DMA_BUFFER_SIZE];

volatile uint8_t transponderIrDataTransferInProgress = 0;

bool transponderIrInit(void)
{
    memset(&transponderIrDMABuffer, 0, TRANSPONDER_DMA_BUFFER_SIZE);

    ioTag_t ioTag = IO_TAG_NONE;
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_TRANSPONDER) {
            ioTag = timerHardware[i].tag;
            break;
        }
    }

    if (!ioTag) {
        return false;
    }


    transponderIrHardwareInit(ioTag);
    return true;
}

bool isTransponderIrReady(void)
{
    return !transponderIrDataTransferInProgress;
}

static uint16_t dmaBufferOffset;

void updateTransponderDMABuffer(const uint8_t* transponderData)
{
    uint8_t byteIndex;
    uint8_t bitIndex;
    uint8_t toggleIndex;

    for (byteIndex = 0; byteIndex < TRANSPONDER_DATA_LENGTH; byteIndex++) {

        uint8_t byteToSend = *transponderData;
        transponderData++;
        for (bitIndex = 0; bitIndex < TRANSPONDER_BITS_PER_BYTE; bitIndex++)
        {
            bool doToggles = false;
            if (bitIndex == 0) {
                doToggles = true;
            } else if (bitIndex == TRANSPONDER_BITS_PER_BYTE - 1) {
                doToggles = false;
            } else {
                doToggles = byteToSend & (1 << (bitIndex - 1));
            }

            for (toggleIndex = 0; toggleIndex < TRANSPONDER_TOGGLES_PER_BIT; toggleIndex++)
            {
                if (doToggles) {
                    transponderIrDMABuffer[dmaBufferOffset] = BIT_TOGGLE_1;
                } else {
                    transponderIrDMABuffer[dmaBufferOffset] = BIT_TOGGLE_0;
                }
                dmaBufferOffset++;
            }
            transponderIrDMABuffer[dmaBufferOffset] = BIT_TOGGLE_0;
            dmaBufferOffset++;
        }
    }
}

void transponderIrWaitForTransmitComplete(void)
{
    static uint32_t waitCounter = 0;

    while(transponderIrDataTransferInProgress) {
        waitCounter++;
    }
}

void transponderIrUpdateData(const uint8_t* transponderData)
{
    transponderIrWaitForTransmitComplete();

    updateTransponderDMABuffer(transponderData);
}


void transponderIrTransmit(void)
{
    transponderIrWaitForTransmitComplete();

    dmaBufferOffset = 0;

    transponderIrDataTransferInProgress = 1;
    transponderIrDMAEnable();
}
