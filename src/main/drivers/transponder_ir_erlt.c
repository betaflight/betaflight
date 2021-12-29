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
#include "drivers/transponder_ir_erlt.h"

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(UNIT_TEST)

static uint16_t dmaBufferOffset;
extern const struct transponderVTable erltTansponderVTable;

void transponderIrInitERLT(transponder_t *transponder)
{
    transponder->dma_buffer_size    = TRANSPONDER_DMA_BUFFER_SIZE_ERLT;
    transponder->vTable             = &erltTansponderVTable;
    transponder->timer_hz           = TRANSPONDER_TIMER_MHZ_ERLT;
    transponder->timer_carrier_hz   = TRANSPONDER_CARRIER_HZ_ERLT;
    memset(&(transponder->transponderIrDMABuffer.erlt), 0, sizeof(transponder->transponderIrDMABuffer.erlt));
}

void addBitToBuffer(transponder_t *transponder, uint8_t cycles, uint8_t pulsewidth)
{
    for (int i = 0; i < cycles; i++) {
        transponder->transponderIrDMABuffer.erlt[dmaBufferOffset++] = pulsewidth;
    }
}

void updateTransponderDMABufferERLT(transponder_t *transponder, const uint8_t* transponderData)
{
    uint8_t byteToSend = ~(*transponderData); //transponderData is stored inverted, so invert before using
    uint8_t paritysum = 0; //sum of one bits

    dmaBufferOffset = 0; //reset buffer count

    //start bit 1, always pulsed, bit value = 0
    addBitToBuffer(transponder, ERLTCyclesForZeroBit, transponder->bitToggleOne);

    //start bit 2, always not pulsed, bit value = 0
    addBitToBuffer(transponder, ERLTCyclesForZeroBit, ERLTBitQuiet);

    //add data bits, only the 6 LSB
    for (int i = 5; i >= 0; i--)
    {
        uint8_t bv = (byteToSend >> i) & 0x01;
        paritysum += bv;
        addBitToBuffer(transponder, (bv ? ERLTCyclesForOneBit : ERLTCyclesForZeroBit), ((i % 2) ? transponder->bitToggleOne : ERLTBitQuiet));
    }

    //parity bit, always pulsed, bit value is zero if sum is even, one if odd
    addBitToBuffer(transponder, ((paritysum % 2) ?  ERLTCyclesForOneBit : ERLTCyclesForZeroBit), transponder->bitToggleOne);

    //add final zero after the pulsed parity bit to stop pulses until the next update
    transponder->transponderIrDMABuffer.erlt[dmaBufferOffset++] = ERLTBitQuiet;

    //reset buffer size to that required by this ERLT id
    transponder->dma_buffer_size = dmaBufferOffset;
}

const struct transponderVTable erltTansponderVTable = {
     updateTransponderDMABufferERLT,
};

#endif
#endif
