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

#ifdef USE_LED_STRIP

#include "common/color.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "platform/timer.h"

#include "drivers/light_ws2811strip.h"
#include "platform/light_ws2811strip_stm32.h"

static IO_t ws2811IO = IO_NONE;

static TIM_HandleTypeDef TimHandle;
static uint16_t timerChannel = 0;

static FAST_IRQ_HANDLER void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    HAL_DMA_IRQHandler(TimHandle.hdma[descriptor->userParam]);
#if defined(STM32H5) || defined(STM32N6)
    // DMA request is the update event (UDE) on GPDMA — see ws2811LedStripHardwareInit().
    __HAL_TIM_DISABLE_DMA(&TimHandle, TIM_DMA_UPDATE);
    // On the other platforms TIM_DMACmd() returns the handle to READY; without
    // this reset DMA_SetCurrDataCounter() returns HAL_BUSY from the second
    // frame on and the strip freezes on the first frame.
    TimHandle.State = HAL_TIM_STATE_READY;
#else
    TIM_DMACmd(&TimHandle, timerChannel, DISABLE);
#endif
    ws2811LedDataTransferInProgress = false;
}

bool ws2811LedStripHardwareInit(void)
{
    if (!ledStripIoTag) {
        return false;
    }

    const timerHardware_t *timerHardware = timerAllocate(ledStripIoTag, OWNER_LED_STRIP, 0);

    if (timerHardware == NULL) {
        return false;
    }

    TIM_TypeDef *timer = (TIM_TypeDef *)timerHardware->tim;
    timerChannel = timerHardware->channel;

    dmaResource_t *dmaRef;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec == NULL) {
        return false;
    }

    dmaRef = dmaSpec->ref;
    uint32_t dmaChannel = dmaSpec->channel;
#else
    dmaRef = timerHardware->dmaRef;
    uint32_t dmaChannel = timerHardware->dmaChannel;
#endif

    if (dmaRef == NULL || !dmaAllocate(dmaGetIdentifier(dmaRef), OWNER_LED_STRIP, 0)) {
        return false;
    }
    TimHandle.Instance = timer;

    /* Compute the prescaler value */
    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timerHardware->tim, WS2811_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timerHardware->tim, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    TimHandle.Init.Prescaler = prescaler;
    TimHandle.Init.Period = period; // 800kHz
    TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
        /* Initialization Error */
        return false;
    }

    static DMA_HandleTypeDef hdma_tim;

    ws2811IO = IOGetByTag(ledStripIoTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

#if defined(STM32H5)
    __HAL_RCC_GPDMA1_CLK_ENABLE();
    __HAL_RCC_GPDMA2_CLK_ENABLE();
#elif defined(STM32N6)
    __HAL_RCC_GPDMA1_CLK_ENABLE();
    __HAL_RCC_HPDMA1_CLK_ENABLE();
#else
    __DMA1_CLK_ENABLE();
    __DMA2_CLK_ENABLE();
#endif

    /* Set the parameters to be configured */
#if defined(STM32H5) || defined(STM32N6)
    // N6/H5 use HPDMA/GPDMA with a different HAL DMA init structure.
    //
    // Drive the channel from the timer UPDATE request (UDE), not the
    // per-channel compare request (CCxDE). Empirically, a CCxDE-driven GPDMA
    // channel re-fires continuously and races through the whole WS2811 bit
    // buffer in microseconds, so no valid waveform reaches the pin — even
    // though plain PWM (no DMA) on the same pin works. RM0481 documents no
    // semantic difference between the CC and UP DMA requests; UP is the
    // configuration proven to work, gives one CCRx write per timer period,
    // and mirrors the DShot GPDMA path (see pwm_output_dshot_hal.c).
    // timerHardware->dmaTimUPChannel is the per-timer GPDMA request id
    // (e.g. LL_GPDMA1_REQUEST_TIM1_UP).
    hdma_tim.Init.Request = timerHardware->dmaTimUPChannel;
    UNUSED(dmaChannel);
    hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim.Init.SrcInc = DMA_SINC_INCREMENTED;
    hdma_tim.Init.DestInc = DMA_DINC_FIXED;
    hdma_tim.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
    hdma_tim.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
    hdma_tim.Init.Priority = DMA_HIGH_PRIORITY;
    hdma_tim.Init.Mode = DMA_NORMAL;
    hdma_tim.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    hdma_tim.Init.SrcBurstLength = 1;
    hdma_tim.Init.DestBurstLength = 1;
    // On H5 both GPDMA master ports reach all memories and peripherals
    // (RM0481 fig. 1); splitting the transfer across ports picks each port's
    // zero-latency fast path: PORT1 for the source buffer in SRAM, PORT0 for
    // the APB bridge to the timer CCRx. On N6 the split is mandatory: PORT1
    // is the AXI port and the only one that reaches AXISRAM.
    hdma_tim.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT1 | DMA_DEST_ALLOCATED_PORT0;
    hdma_tim.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
#elif defined(STM32H7) || defined(STM32G4)
    hdma_tim.Init.Request = dmaChannel;
    hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim.Init.Mode = DMA_NORMAL;
    hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
#if !defined(STM32G4)
    hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;
#endif
#else
    hdma_tim.Init.Channel = dmaChannel;
    hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim.Init.Mode = DMA_NORMAL;
    hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;
#endif

    /* Set hdma_tim instance */
    hdma_tim.Instance = (DMA_ARCH_TYPE *)dmaRef;

    uint16_t dmaIndex = timerDmaIndex(timerChannel);

    /* Link hdma_tim to hdma[x] (channelx) */
    __HAL_LINKDMA(&TimHandle, hdma[dmaIndex], hdma_tim);

    dmaEnable(dmaGetIdentifier(dmaRef));
    dmaSetHandler(dmaGetIdentifier(dmaRef), WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, dmaIndex);

    /* Initialize TIMx DMA handle */
    if (HAL_DMA_Init(TimHandle.hdma[dmaIndex]) != HAL_OK) {
        /* Initialization Error */
        return false;
    }

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    TIM_OCInitStructure.OCNPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.Pulse = 0;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &TIM_OCInitStructure, timerChannel) != HAL_OK) {
        /* Configuration Error */
        return false;
    }
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        if (HAL_TIMEx_PWMN_Start(&TimHandle, timerChannel) != HAL_OK) {
            /* Starting PWM generation Error */
            return false;
        }
    } else {
        if (HAL_TIM_PWM_Start(&TimHandle, timerChannel) != HAL_OK) {
            /* Starting PWM generation Error */
            return false;
        }
    }

    return true;
}

void ws2811LedStripStartTransfer(void)
{
#ifdef USE_LED_STRIP_CACHE_MGMT
    SCB_CleanDCache_by_Addr(ledStripDMABuffer, WS2811_DMA_BUF_CACHE_ALIGN_BYTES);
#endif

#if defined(STM32H5) || defined(STM32N6)
    // GPDMA sizes the transfer in BYTES (CBR1.BNDT), not in transfer items:
    // HAL_DMA_Start_IT() writes the length straight into BNDT. Each item is a
    // 32-bit word (one CCRx write per WS2811 bit), so the byte count is the
    // word count times 4. Passing the word count would program only a quarter
    // of the buffer, clocking out ~1/4 of the bitstream (and cutting the reset
    // latch) so the strip shows nothing/garbage. Classic DMA (F4/F7/H7/G4)
    // counts transfer items, so it keeps the word count.
    const uint16_t ws2811DmaLength = WS2811_DMA_BUFFER_SIZE * sizeof(uint32_t);
#else
    const uint16_t ws2811DmaLength = WS2811_DMA_BUFFER_SIZE;
#endif

    if (DMA_SetCurrDataCounter(&TimHandle, timerChannel, ledStripDMABuffer, ws2811DmaLength) != HAL_OK) {
        /* DMA set error */
        ws2811LedDataTransferInProgress = false;
        return;
    }
    /* Reset timer counter */
    __HAL_TIM_SET_COUNTER(&TimHandle,0);
    /* Enable DMA requests */
#if defined(STM32H5) || defined(STM32N6)
    // Edge-triggered update event on GPDMA — see ws2811LedStripHardwareInit().
    __HAL_TIM_ENABLE_DMA(&TimHandle, TIM_DMA_UPDATE);
#else
    TIM_DMACmd(&TimHandle,timerChannel,ENABLE);
#endif
}
#endif
