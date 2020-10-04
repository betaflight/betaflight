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

#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT_BITBANG

#include "build/atomic.h"
#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/dshot.h"
#include "drivers/dshot_bitbang_impl.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h" // XXX for pwmOutputPort_t motors[]; should go away with refactoring
#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

// Setup GPIO_MODER and GPIO_ODR register manipulation values

void bbGpioSetup(bbMotor_t *bbMotor)
{
    bbPort_t *bbPort = bbMotor->bbPort;
    int pinIndex = bbMotor->pinIndex;

#ifdef STM32H7
    bbPort->gpioModeMask |= (GPIO_MODER_MODE0 << (pinIndex * 2)); // A minor name change in H7 CMSIS
#else
    bbPort->gpioModeMask |= (GPIO_MODER_MODER0 << (pinIndex * 2));
#endif
    bbPort->gpioModeInput |= (LL_GPIO_MODE_INPUT << (pinIndex * 2));
    bbPort->gpioModeOutput |= (LL_GPIO_MODE_OUTPUT << (pinIndex * 2));

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        bbPort->gpioIdleBSRR |= (1 << pinIndex);         // BS (lower half)
    } else
#endif
    {
        bbPort->gpioIdleBSRR |= (1 << (pinIndex + 16));  // BR (higher half)
    }

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        IOWrite(bbMotor->io, 1);
    } else
#endif
    {
        IOWrite(bbMotor->io, 0);
    }

#if defined(STM32F7) || defined(STM32G4) || defined(STM32H7)
    IOConfigGPIO(bbMotor->io, IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, bbPuPdMode));
#else
#error MCU dependent code required
#endif
}

void bbTimerChannelInit(bbPort_t *bbPort)
{
    const timerHardware_t *timhw = bbPort->timhw;

    switch (bbPort->timhw->channel) {
    case TIM_CHANNEL_1: bbPort->llChannel = LL_TIM_CHANNEL_CH1; break;
    case TIM_CHANNEL_2: bbPort->llChannel = LL_TIM_CHANNEL_CH2; break;
    case TIM_CHANNEL_3: bbPort->llChannel = LL_TIM_CHANNEL_CH3; break;
    case TIM_CHANNEL_4: bbPort->llChannel = LL_TIM_CHANNEL_CH4; break;
    }

    LL_TIM_OC_InitTypeDef ocInit;
    LL_TIM_OC_StructInit(&ocInit);
    ocInit.OCMode = LL_TIM_OCMODE_PWM1;
    ocInit.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
    ocInit.OCState = LL_TIM_OCSTATE_ENABLE;
    ocInit.OCPolarity = LL_TIM_OCPOLARITY_LOW;
    ocInit.CompareValue = 10; // Duty doesn't matter, but too value small would make monitor output invalid

    //TIM_Cmd(bbPort->timhw->tim, DISABLE);
    LL_TIM_DisableCounter(bbPort->timhw->tim);

    //timerOCInit(timhw->tim, timhw->channel, &TIM_OCStruct);
    LL_TIM_OC_Init(timhw->tim, bbPort->llChannel, &ocInit);

    //timerOCPreloadConfig(timhw->tim, timhw->channel, TIM_OCPreload_Enable);
    LL_TIM_OC_EnablePreload(timhw->tim, bbPort->llChannel);

#ifdef DEBUG_MONITOR_PACER
    if (timhw->tag) {
        IO_t io = IOGetByTag(timhw->tag);
        IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
        IOInit(io, OWNER_DSHOT_BITBANG, 0);
        //TIM_CtrlPWMOutputs(timhw->tim, ENABLE);
        LL_TIM_EnableAllOutputs(timhw->tim);
    }
#endif

    // Enable and keep it running

    //TIM_Cmd(bbPort->timhw->tim, ENABLE);
    LL_TIM_EnableCounter(bbPort->timhw->tim);
}

#ifdef USE_DMA_REGISTER_CACHE
void bbLoadDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
#if defined(STM32F7) || defined(STM32H7)
    ((DMA_ARCH_TYPE *)dmaResource)->CR = dmaRegCache->CR;
    ((DMA_ARCH_TYPE *)dmaResource)->FCR = dmaRegCache->FCR;
    ((DMA_ARCH_TYPE *)dmaResource)->NDTR = dmaRegCache->NDTR;
    ((DMA_ARCH_TYPE *)dmaResource)->PAR = dmaRegCache->PAR;
    ((DMA_ARCH_TYPE *)dmaResource)->M0AR = dmaRegCache->M0AR;
#elif defined(STM32G4)
    ((DMA_ARCH_TYPE *)dmaResource)->CCR = dmaRegCache->CCR;
    ((DMA_ARCH_TYPE *)dmaResource)->CNDTR = dmaRegCache->CNDTR;
    ((DMA_ARCH_TYPE *)dmaResource)->CPAR = dmaRegCache->CPAR;
    ((DMA_ARCH_TYPE *)dmaResource)->CMAR = dmaRegCache->CMAR;
#else
#error MCU dependent code required
#endif
}

static void bbSaveDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
#if defined(STM32F7) || defined(STM32H7)
    dmaRegCache->CR = ((DMA_ARCH_TYPE *)dmaResource)->CR;
    dmaRegCache->FCR = ((DMA_ARCH_TYPE *)dmaResource)->FCR;
    dmaRegCache->NDTR = ((DMA_ARCH_TYPE *)dmaResource)->NDTR;
    dmaRegCache->PAR = ((DMA_ARCH_TYPE *)dmaResource)->PAR;
    dmaRegCache->M0AR = ((DMA_ARCH_TYPE *)dmaResource)->M0AR;
#elif defined(STM32G4)
    ((DMA_ARCH_TYPE *)dmaResource)->CCR = dmaRegCache->CCR;
    ((DMA_ARCH_TYPE *)dmaResource)->CNDTR = dmaRegCache->CNDTR;
    ((DMA_ARCH_TYPE *)dmaResource)->CPAR = dmaRegCache->CPAR;
    ((DMA_ARCH_TYPE *)dmaResource)->CMAR = dmaRegCache->CMAR;
#else
#error MCU dependent code required
#endif
}
#endif

void bbSwitchToOutput(bbPort_t * bbPort)
{
    // Output idle level before switching to output
    // Use BSRR register for this
    // Normal: Use BR (higher half)
    // Inverted: Use BS (lower half)

    WRITE_REG(bbPort->gpio->BSRR, bbPort->gpioIdleBSRR);

    // Set GPIO to output

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->MODER, bbPort->gpioModeMask, bbPort->gpioModeOutput);
    }

    // Reinitialize port group DMA for output

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbDMA_Cmd(bbPort, DISABLE);
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegOutput);
#else
    //xDMA_DeInit(dmaResource);
    xLL_EX_DMA_Deinit(dmaResource);
    //xDMA_Init(dmaResource, &bbPort->outputDmaInit);
    xLL_EX_DMA_Init(dmaResource, &bbPort->outputDmaInit);
    // Needs this, as it is DeInit'ed above...
    //xDMA_ITConfig(dmaResource, DMA_IT_TC, ENABLE);
    xLL_EX_DMA_EnableIT_TC(dmaResource);
#endif

    // Reinitialize pacer timer for output

    bbPort->timhw->tim->ARR = bbPort->outputARR;

    bbPort->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;
}

#ifdef USE_DSHOT_TELEMETRY
void bbSwitchToInput(bbPort_t *bbPort)
{
    // Set GPIO to input

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->MODER, bbPort->gpioModeMask, bbPort->gpioModeInput);
    }

    // Reinitialize port group DMA for input

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegInput);
#else
    //xDMA_DeInit(dmaResource);
    xLL_EX_DMA_Deinit(dmaResource);
    //xDMA_Init(dmaResource, &bbPort->inputDmaInit);
    xLL_EX_DMA_Init(dmaResource, &bbPort->inputDmaInit);

    // Needs this, as it is DeInit'ed above...
    //xDMA_ITConfig(dmaResource, DMA_IT_TC, ENABLE);
    xLL_EX_DMA_EnableIT_TC(dmaResource);
#endif

    // Reinitialize pacer timer for input

    bbPort->timhw->tim->ARR = bbPort->inputARR;

    bbDMA_Cmd(bbPort, ENABLE);

    bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;
}
#endif

void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
{
    LL_DMA_InitTypeDef *dmainit = (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;

    LL_DMA_StructInit(dmainit);

    dmainit->Mode = LL_DMA_MODE_NORMAL;
#if defined(STM32G4) || defined(STM32H7)
    dmainit->PeriphRequest = bbPort->dmaChannel;
#else
    dmainit->Channel = bbPort->dmaChannel;
    dmainit->FIFOMode = LL_DMA_FIFOMODE_ENABLE ;
#endif

    dmainit->PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    dmainit->MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;

    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        dmainit->Priority = LL_DMA_PRIORITY_VERYHIGH;
        dmainit->Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
        dmainit->NbData = bbPort->portOutputCount;
        dmainit->PeriphOrM2MSrcAddress = (uint32_t)&bbPort->gpio->BSRR;
        dmainit->PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
        dmainit->MemoryOrM2MDstAddress = (uint32_t)bbPort->portOutputBuffer;
        dmainit->MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;

#ifdef USE_DMA_REGISTER_CACHE
        xLL_EX_DMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
#endif
    } else {
        dmainit->Priority = LL_DMA_PRIORITY_HIGH;
        dmainit->Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
        dmainit->NbData = bbPort->portInputCount;

        dmainit->PeriphOrM2MSrcAddress = (uint32_t)&bbPort->gpio->IDR;
        dmainit->PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
        dmainit->MemoryOrM2MDstAddress = (uint32_t)bbPort->portInputBuffer;

#ifdef STM32G4
        // XXX G4 seems to require 16-bit transfer
        dmainit->MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
#else
        dmainit->MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
#endif

#ifdef USE_DMA_REGISTER_CACHE
        xLL_EX_DMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegInput);
#endif
    }
}

void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period)
{
    LL_TIM_InitTypeDef *init = &bbPort->timeBaseInit;

    init->Prescaler = 0; // Feed raw timerClock
    init->ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    init->CounterMode = LL_TIM_COUNTERMODE_UP;
    init->Autoreload = period;
    //TIM_TimeBaseInit(bbPort->timhw->tim, &bbPort->timeBaseInit);
    LL_TIM_Init(bbPort->timhw->tim, init);
    MODIFY_REG(bbPort->timhw->tim->CR1, TIM_CR1_ARPE, TIM_AUTORELOAD_PRELOAD_ENABLE);
}

void bbTIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
{
    //TIM_DMACmd(TIMx, TIM_DMASource, NewState);
    if (NewState == ENABLE) {
        SET_BIT(TIMx->DIER, TIM_DMASource);
    } else {
        CLEAR_BIT(TIMx->DIER, TIM_DMASource);
    }
}

void bbDMA_ITConfig(bbPort_t *bbPort)
{
    //xDMA_ITConfig(bbPort->dmaResource, DMA_IT_TC, ENABLE);

    xLL_EX_DMA_EnableIT_TC(bbPort->dmaResource);

#if defined(STM32G4)
    SET_BIT(((DMA_Channel_TypeDef *)(bbPort->dmaResource))->CCR, DMA_CCR_TCIE|DMA_CCR_TEIE);
#else
    SET_BIT(((DMA_Stream_TypeDef *)(bbPort->dmaResource))->CR, DMA_SxCR_TCIE|DMA_SxCR_TEIE);
#endif
}

void bbDMA_Cmd(bbPort_t *bbPort, FunctionalState NewState)
{
    //xDMA_Cmd(bbPort->dmaResource, NewState);

    if (NewState == ENABLE) {
        xLL_EX_DMA_EnableResource(bbPort->dmaResource);
    } else {
        xLL_EX_DMA_DisableResource(bbPort->dmaResource);
    }
}

int bbDMA_Count(bbPort_t *bbPort)
{
    return xLL_EX_DMA_GetDataLength(bbPort->dmaResource);
}
#endif // USE_DSHOT_BITBANG
