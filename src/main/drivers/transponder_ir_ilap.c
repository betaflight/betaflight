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

#include "drivers/transponder_ir.h"
#include "drivers/transponder_ir_ilap.h"

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(UNIT_TEST)

static uint16_t dmaBufferOffset;
extern const struct transponderVTable ilapTansponderVTable;

void transponderIrInitIlap(transponder_t *transponder)
{
    // from drivers/transponder_ir.h
    transponder->gap_toggles        = TRANSPONDER_GAP_TOGGLES_ILAP;
    transponder->dma_buffer_size    = TRANSPONDER_DMA_BUFFER_SIZE_ILAP;
    transponder->vTable             = &ilapTansponderVTable;
    transponder->timer_hz           = TRANSPONDER_TIMER_MHZ_ILAP;
    transponder->timer_carrier_hz   = TRANSPONDER_CARRIER_HZ_ILAP;
    memset(&(transponder->transponderIrDMABuffer.ilap), 0, sizeof(transponder->transponderIrDMABuffer.ilap));

}

void updateTransponderDMABufferIlap(transponder_t *transponder, const uint8_t* transponderData)
{
        uint8_t byteIndex;
        uint8_t bitIndex;
        uint8_t toggleIndex;
        for (byteIndex = 0; byteIndex < TRANSPONDER_DATA_LENGTH_ILAP; byteIndex++) {
            uint8_t byteToSend = *transponderData;
            transponderData++;
            for (bitIndex = 0; bitIndex < TRANSPONDER_BITS_PER_BYTE_ILAP; bitIndex++)
            {
                bool doToggles = false;
                if (bitIndex == 0) {
                    doToggles = true;
                }
                else if (bitIndex == TRANSPONDER_BITS_PER_BYTE_ILAP - 1) {
                    doToggles = false;
                }
                else {
                    doToggles = byteToSend & (1 << (bitIndex - 1));
                }
                for (toggleIndex = 0; toggleIndex < TRANSPONDER_TOGGLES_PER_BIT_ILAP; toggleIndex++)
                {
                    if (doToggles) {
                        transponder->transponderIrDMABuffer.ilap[dmaBufferOffset] = transponder->bitToggleOne;
                    }
                    else {
                        transponder->transponderIrDMABuffer.ilap[dmaBufferOffset] = 0;
                    }
                    dmaBufferOffset++;
                }
                transponder->transponderIrDMABuffer.ilap[dmaBufferOffset] = 0;
                dmaBufferOffset++;
            }
        }
        dmaBufferOffset = 0;
}

const struct transponderVTable ilapTansponderVTable = {
     updateTransponderDMABufferIlap,
};

#endif
#endif
