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

#ifdef USE_LED_STRIP

#include "build/debug.h"

#include "common/color.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/timer.h"
<<<<<<< HEAD
<<<<<<< HEAD
#include "platform/timer.h"

#include "platform/light_ws2811strip_stm32.h"
=======
=======
#include "platform/timer.h"
>>>>>>> cdcdf52d6 (update CH32 drivers for rebase)

#include "drivers/light_ws2811strip.h"
<<<<<<< HEAD
=======
#include "platform/light_ws2811strip_stm32.h"

static IO_t ws2811IO = IO_NONE;
static dmaResource_t *dmaRef = NULL;
>>>>>>> ab93a2999 (WIP CH32 platform updates)
static TIM_TypeDef *timer = NULL;

static void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
#if defined(USE_WS2811_SINGLE_COLOUR)
    static uint32_t counter = 0;
#endif

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
#if defined(USE_WS2811_SINGLE_COLOUR)
        counter++;
        if (counter == WS2811_LED_STRIP_LENGTH) {
            // Output low for 50us delay
            memset(ledStripDMABuffer, 0, sizeof(ledStripDMABuffer));
        } else if (counter == (WS2811_LED_STRIP_LENGTH + WS2811_DELAY_ITERATIONS)) {
            counter = 0;
            ws2811LedDataTransferInProgress = false;
            xDMA_Cmd(descriptor->ref, DISABLE);
        }
#else
        ws2811LedDataTransferInProgress = false;
        xDMA_Cmd(descriptor->ref, DISABLE);
#endif

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

<<<<<<< HEAD
<<<<<<< HEAD
bool ws2811LedStripHardwareInit(void)
{
    if (!ledStripIoTag) {
=======
bool ws2811LedStripHardwareInit(ioTag_t ioTag)
{
    if (!ioTag) {
>>>>>>> 1b9653a92 (dshot 8K,uart4 MSP+DisplayPort function is OK)
=======
bool ws2811LedStripHardwareInit(void)
{
    if (!ledStripIoTag) {
>>>>>>> ab93a2999 (WIP CH32 platform updates)
        return false;
    }

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

<<<<<<< HEAD
<<<<<<< HEAD
    const timerHardware_t *timerHardware = timerAllocate(ledStripIoTag, OWNER_LED_STRIP, 0);
=======
    const timerHardware_t *timerHardware = timerAllocate(ioTag, OWNER_LED_STRIP, 0);
>>>>>>> 1b9653a92 (dshot 8K,uart4 MSP+DisplayPort function is OK)
=======
    const timerHardware_t *timerHardware = timerAllocate(ledStripIoTag, OWNER_LED_STRIP, 0);
>>>>>>> ab93a2999 (WIP CH32 platform updates)

    if (timerHardware == NULL) {
        return false;
    }

    timer = timerHardware->tim;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec == NULL) {
        return false;
    }

    dmaRef = dmaSpec->ref;

// #if defined(STM32F4)
//     uint32_t dmaChannel = dmaSpec->channel;
// #endif

#else
    dmaRef = timerHardware->dmaRef;

//     #if defined(STM32F4)
//     uint32_t dmaChannel = timerHardware->dmaChannel;
// #endif

#endif

    if (dmaRef == NULL || !dmaAllocate(dmaGetIdentifier(dmaRef), OWNER_LED_STRIP, 0)) {
        return false;
    }

<<<<<<< HEAD
<<<<<<< HEAD
    ws2811IO = IOGetByTag(ledStripIoTag);
=======
    ws2811IO = IOGetByTag(ioTag);
>>>>>>> 1b9653a92 (dshot 8K,uart4 MSP+DisplayPort function is OK)
=======
    ws2811IO = IOGetByTag(ledStripIoTag);
>>>>>>> ab93a2999 (WIP CH32 platform updates)
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);

    IOConfigGPIOAF(ws2811IO, IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP), timerHardware->alternateFunction);

    RCC_ClockCmd(timerRCC(timer), ENABLE);

    // Stop timer
    TIM_Cmd(timer, DISABLE);

    /* Compute the prescaler value */
    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, WS2811_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period; // 800kHz
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNPolarity =  (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity =  (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }

    TIM_OCInitStructure.TIM_Pulse = 0;

    timerOCInit(timer, timerHardware->channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(timer, ENABLE);
    TIM_ARRPreloadConfig(timer, ENABLE);

    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_CCxNCmd(timer, timerHardware->channel, TIM_CCxN_Enable);
    } else {
        TIM_CCxCmd(timer, timerHardware->channel, TIM_CCx_Enable);
    }

    TIM_Cmd(timer, ENABLE);

    dmaEnable(dmaGetIdentifier(dmaRef));
    dmaSetHandler(dmaGetIdentifier(dmaRef), WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);

    xDMA_DeInit(dmaRef);

    /* configure DMA */
    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)timerCCR(timer, timerHardware->channel);
    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

#if defined(USE_WS2811_SINGLE_COLOUR)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
#else
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
#endif
    xDMA_Init(dmaRef, &DMA_InitStructure);
#if defined(USE_DMA_SPEC)
    dmaMuxEnable(dmaGetIdentifier(dmaRef), dmaSpec->dmaMuxId);
#else
    #warning "USE_DMA_SPEC DETECT FAIL IN LEDSTRIP CH32H41x NEED DMA MUX "
#endif
    TIM_DMACmd(timer, timerDmaSource(timerHardware->channel), ENABLE);
    xDMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);
    return true;
}

<<<<<<< HEAD
<<<<<<< HEAD
void ws2811LedStripStartTransfer(void)
{
    xDMA_Cmd(dmaRef, DISABLE);
=======
void ws2811LedStripDMAEnable(void)
{
>>>>>>> 1b9653a92 (dshot 8K,uart4 MSP+DisplayPort function is OK)
=======
void ws2811LedStripStartTransfer(void)
{
<<<<<<< HEAD
    xDMA_Cmd(dmaRef, FALSE);
>>>>>>> ab93a2999 (WIP CH32 platform updates)
=======
    xDMA_Cmd(dmaRef, DISABLE);
>>>>>>> e00d53f2f (fix timer and led strip drivers bugs)
    xDMA_SetCurrDataCounter(dmaRef, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(timer, 0);
    TIM_Cmd(timer, ENABLE);
    xDMA_Cmd(dmaRef, ENABLE);
}
#endif
