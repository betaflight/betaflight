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

#pragma once

#include "drivers/io_types.h"

/*** ARCITIMER ***/
#define TRANSPONDER_BITS_PER_BYTE_ARCITIMER 8
#define TRANSPONDER_DATA_LENGTH_ARCITIMER 9
#define TRANSPONDER_TOGGLES_PER_BIT_ARCITIMER 4
#define TRANSPONDER_GAP_TOGGLES_ARCITIMER 0
#define TRANSPONDER_TOGGLES_ARCITIMER (TRANSPONDER_TOGGLES_PER_BIT_ARCITIMER + TRANSPONDER_GAP_TOGGLES_ARCITIMER)
#define TRANSPONDER_DMA_BUFFER_SIZE_ARCITIMER 155 * TRANSPONDER_TOGGLES_PER_BIT_ARCITIMER // 620
#define TRANSPONDER_TIMER_MHZ_ARCITIMER       24
#define TRANSPONDER_CARRIER_HZ_ARCITIMER      41886
#define TRANSPONDER_TRANSMIT_DELAY_ARCITIMER  4500
#define TRANSPONDER_TRANSMIT_JITTER_ARCITIMER 10000
/*** ******** ***/


/*** ILAP ***/
#define TRANSPONDER_BITS_PER_BYTE_ILAP 10 // start + 8 data + stop
#define TRANSPONDER_DATA_LENGTH_ILAP 6
#define TRANSPONDER_TOGGLES_PER_BIT_ILAP 11
#define TRANSPONDER_GAP_TOGGLES_ILAP 1
#define TRANSPONDER_TOGGLES_ILAP (TRANSPONDER_TOGGLES_PER_BIT_ILAP + TRANSPONDER_GAP_TOGGLES_ILAP)
#define TRANSPONDER_DMA_BUFFER_SIZE_ILAP ((TRANSPONDER_TOGGLES_PER_BIT_ILAP + 1) * TRANSPONDER_BITS_PER_BYTE_ILAP * TRANSPONDER_DATA_LENGTH_ILAP) //720
#define TRANSPONDER_TIMER_MHZ_ILAP       24
#define TRANSPONDER_CARRIER_HZ_ILAP      460750
#define TRANSPONDER_TRANSMIT_DELAY_ILAP   4500
#define TRANSPONDER_TRANSMIT_JITTER_ILAP  10000
/*** ******** ***/


/*** ERLT ***/
#define TRANSPONDER_DATA_LENGTH_ERLT        1

#define ERLTBitQuiet                        0
#define ERLTCyclesForOneBit                 25
#define ERLTCyclesForZeroBit                10
#define TRANSPONDER_DMA_BUFFER_SIZE_ERLT    200 // actually ERLT is variable length 91-196 depending on the ERLT id
#define TRANSPONDER_TIMER_MHZ_ERLT          18
#define TRANSPONDER_CARRIER_HZ_ERLT         38000
#define TRANSPONDER_TRANSMIT_DELAY_ERLT     22500
#define TRANSPONDER_TRANSMIT_JITTER_ERLT    5000
/*** ******** ***/


/*
 * Implementation note:
 * Using around over 700 bytes for a transponder DMA buffer is a little excessive, likely an alternative implementation that uses a fast
 * ISR to generate the output signal dynamically based on state would be more memory efficient and would likely be more appropriate for
 * other targets.  However this approach requires very little CPU time and is just fire-and-forget.
 */
#if defined(UNIT_TEST)

    typedef union transponderIrDMABuffer_s {
        uint8_t arcitimer[TRANSPONDER_DMA_BUFFER_SIZE_ARCITIMER]; // 620
        uint8_t ilap[TRANSPONDER_DMA_BUFFER_SIZE_ILAP]; // 720
        uint8_t erlt[TRANSPONDER_DMA_BUFFER_SIZE_ERLT]; // 91-200
    } transponderIrDMABuffer_t;

#elif defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)

    typedef union transponderIrDMABuffer_s {
        uint32_t arcitimer[TRANSPONDER_DMA_BUFFER_SIZE_ARCITIMER]; // 620
        uint32_t ilap[TRANSPONDER_DMA_BUFFER_SIZE_ILAP]; // 720
        uint32_t erlt[TRANSPONDER_DMA_BUFFER_SIZE_ERLT]; // 91-200
    } transponderIrDMABuffer_t;
#endif

typedef struct transponder_s {
    uint8_t gap_toggles;
    uint32_t timer_hz;
    uint32_t timer_carrier_hz;
    uint16_t bitToggleOne;
    uint32_t dma_buffer_size;

    #if defined(STM32F4)|| defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(UNIT_TEST)
        transponderIrDMABuffer_t transponderIrDMABuffer;
    #endif

    const struct transponderVTable *vTable;
} transponder_t;

typedef enum {
    TRANSPONDER_NONE = 0,
    TRANSPONDER_ILAP,
    TRANSPONDER_ARCITIMER,
    TRANSPONDER_ERLT
} transponderProvider_e;

#define TRANSPONDER_PROVIDER_COUNT 3

struct transponderVTable {
    void (*updateTransponderDMABuffer)(transponder_t *transponder, const uint8_t* transponderData);
};

bool transponderIrInit(const ioTag_t ioTag, const transponderProvider_e provider);
void transponderIrDisable(void);

void transponderIrHardwareInit(ioTag_t ioTag, transponder_t *transponder);
void transponderIrDMAEnable(transponder_t *transponder);

void transponderIrWaitForTransmitComplete(void);

void transponderIrUpdateData(const uint8_t* transponderData);
void transponderIrTransmit(void);

bool isTransponderIrReady(void);

extern volatile uint8_t transponderIrDataTransferInProgress;
