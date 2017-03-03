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

#pragma once

#include "io_types.h"

/*** ARCITIMER ***/
#define TRANSPONDER_BITS_PER_BYTE_ARCITIMER 8
#define TRANSPONDER_DATA_LENGTH_ARCITIMER 9
#define TRANSPONDER_TOGGLES_PER_BIT_ARCITIMER 4
#define TRANSPONDER_GAP_TOGGLES_ARCITIMER 0
#define TRANSPONDER_TOGGLES_ARCITIMER (TRANSPONDER_TOGGLES_PER_BIT_ARCITIMER + TRANSPONDER_GAP_TOGGLES_ARCITIMER)
#define TRANSPONDER_DMA_BUFFER_SIZE_ARCITIMER 155 * TRANSPONDER_TOGGLES_PER_BIT_ARCITIMER // 620
#define TRANSPONDER_TIMER_MHZ_ARCITIMER       24
#define TRANSPONDER_CARRIER_HZ_ARCITIMER      41886
/*** ******** ***/


/*** ILAP ***/
#define TRANSPONDER_BITS_PER_BYTE 10 // start + 8 data + stop
#define TRANSPONDER_DATA_LENGTH 6
#define TRANSPONDER_TOGGLES_PER_BIT 11
#define TRANSPONDER_GAP_TOGGLES 1
#define TRANSPONDER_TOGGLES (TRANSPONDER_TOGGLES_PER_BIT + TRANSPONDER_GAP_TOGGLES)
#define TRANSPONDER_DMA_BUFFER_SIZE ((TRANSPONDER_TOGGLES_PER_BIT + 1) * TRANSPONDER_BITS_PER_BYTE * TRANSPONDER_DATA_LENGTH) //720
#define TRANSPONDER_TIMER_MHZ       24
#define TRANSPONDER_CARRIER_HZ      460750
/*** ******** ***/

/*
 * Implementation note:
 * Using around over 700 bytes for a transponder DMA buffer is a little excessive, likely an alternative implementation that uses a fast
 * ISR to generate the output signal dynamically based on state would be more memory efficient and would likely be more appropriate for
 * other targets.  However this approach requires very little CPU time and is just fire-and-forget.
 *
 * On an STM32F303CC 720 bytes is currently fine and that is the target for which this code was designed for.
 */
#if defined(STM32F3) || defined(UNIT_TEST)

    typedef union transponderIrDMABuffer_s {
        uint8_t arcitimer[TRANSPONDER_DMA_BUFFER_SIZE_ARCITIMER]; // 620
        uint8_t ilap[TRANSPONDER_DMA_BUFFER_SIZE]; // 720
    } transponderIrDMABuffer_t;

#elif defined(STM32F4)

    typedef union transponderIrDMABuffer_s {
        uint32_t arcitimer[TRANSPONDER_DMA_BUFFER_SIZE_ARCITIMER]; // 620
        uint32_t ilap[TRANSPONDER_DMA_BUFFER_SIZE]; // 720
    } transponderIrDMABuffer_t;

#else
    #error "Transponder not supported on this MCU."
#endif

typedef struct transponder_s {
    uint8_t gap_toggles;
    uint32_t timer_hz;
    uint32_t timer_carrier_hz;
    uint16_t bitToggleOne;
    uint32_t dma_buffer_size;
    transponderIrDMABuffer_t transponderIrDMABuffer;
    const struct transponderVTable *vTable;
} transponder_t;

typedef enum TransponderProvider{
    ARCITIMER,
    ILAP
} TransponderProvider;

struct transponderVTable {
    void (*updateTransponderDMABuffer)(transponder_t* transponder, const uint8_t* transponderData);
};

bool transponderIrInit(const TransponderProvider* transponderProvider);
void transponderIrDisable(void);

void transponderIrHardwareInit(ioTag_t ioTag, transponder_t* transponder_instance);
void transponderIrDMAEnable(transponder_t* transponder_instance);

void transponderIrWaitForTransmitComplete(void);

void transponderIrUpdateData(const uint8_t* transponderData);
void transponderIrTransmit(void);

bool isTransponderIrReady(void);

extern volatile uint8_t transponderIrDataTransferInProgress;
