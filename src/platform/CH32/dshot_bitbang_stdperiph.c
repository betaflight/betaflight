/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
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
#include "build/debug_pin.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/dma.h"
#include "platform/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/dshot.h"
#include "dshot_bitbang_impl.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "platform/timer.h"

#include "pg/motor.h"

void bbGpioSetup(bbMotor_t *bbMotor)
{
    bbPort_t *bbPort = bbMotor->bbPort;
    int pinIndex = bbMotor->pinIndex;

    bbPort->gpioModeMask |= (0x03u << (pinIndex * 2));
    bbPort->gpioModeInput |= ((uint32_t)DIR_IN << (pinIndex * 2));
    bbPort->gpioModeOutput |= ((uint32_t)DIR_OUT << (pinIndex * 2));

    const uint32_t pull = IO_CONFIG_GET_PULL(bbMotor->iocfg);
    const uint32_t input = pull == GPIO_PULL_NONE ? 0x4u : 0x8u;
    const unsigned shift = (pinIndex & 7) * 4;
    if (pinIndex < 8) {
        bbPort->gpioCfgLrMask |= 0xfu << shift;
        bbPort->gpioCfgLrOutput |= 0x3u << shift;
        bbPort->gpioCfgLrInput |= input << shift;
    } else {
        bbPort->gpioCfgHrMask |= 0xfu << shift;
        bbPort->gpioCfgHrOutput |= 0x3u << shift;
        bbPort->gpioCfgHrInput |= input << shift;
    }
    if (pull == GPIO_PULL_UP) {
        bbPort->gpioInputBSRR |= 1u << pinIndex;
    } else if (pull == GPIO_PULL_DOWN) {
        bbPort->gpioInputBSRR |= 1u << (pinIndex + 16);
    }

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        bbPort->gpioIdleBSRR |= (1u << pinIndex);        // BS (lower half)
    } else
#endif
    {
        bbPort->gpioIdleBSRR |= (1u << (pinIndex + 16)); // BR (higher half)
    }

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        IOWrite(bbMotor->io, 1);
    } else
#endif
    {
        IOWrite(bbMotor->io, 0);
    }
    // is this needed here?
}

void bbTimerChannelInit(bbPort_t *bbPort)
{
    const timerHardware_t *timhw = bbPort->timhw;

    TIM_OCInitTypeDef TIM_OCStruct;

    TIM_OCStructInit(&TIM_OCStruct);
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OCStruct.TIM_Pulse = 10; // Duty doesn't matter, but too value small would make monitor output invalid

    TIM_Cmd((TIM_TypeDef *)bbPort->timhw->tim, DISABLE);

    timerOCInit((TIM_TypeDef *)timhw->tim, timhw->channel, &TIM_OCStruct);

#ifdef DEBUG_MONITOR_PACER
    if (timhw->tag) {
        IO_t io = IOGetByTag(timhw->tag);
        IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
        IOInit(io, OWNER_DSHOT_BITBANG, 0);
        TIM_CtrlPWMOutputs((TIM_TypeDef *)timhw->tim, ENABLE);
    }
#endif

    // Enable and keep it running

    TIM_Cmd((TIM_TypeDef *)bbPort->timhw->tim, ENABLE);
}

#ifdef USE_DMA_REGISTER_CACHE

FAST_CODE
static void bbLoadDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    ((DMA_ARCH_TYPE *)dmaResource)->CFGR = dmaRegCache->CFGR;
    ((DMA_ARCH_TYPE *)dmaResource)->CNTR = dmaRegCache->CNTR;
    ((DMA_ARCH_TYPE *)dmaResource)->PADDR = dmaRegCache->PADDR;
    ((DMA_ARCH_TYPE *)dmaResource)->MADDR = dmaRegCache->MADDR;
    ((DMA_ARCH_TYPE *)dmaResource)->M1ADDR = dmaRegCache->M1ADDR;
}

static void bbSaveDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    dmaRegCache->CFGR = ((DMA_ARCH_TYPE *)dmaResource)->CFGR;
    dmaRegCache->CNTR = ((DMA_ARCH_TYPE *)dmaResource)->CNTR;
    dmaRegCache->PADDR = ((DMA_ARCH_TYPE *)dmaResource)->PADDR;
    dmaRegCache->MADDR = ((DMA_ARCH_TYPE *)dmaResource)->MADDR;
    dmaRegCache->M1ADDR = ((DMA_ARCH_TYPE *)dmaResource)->M1ADDR;
}
#endif


FAST_CODE
bool bbSwitchToOutput(bbPort_t *bbPort)
{
    dmaResource_t *dmaResource = bbPort->dmaResource;
    bbDMA_Cmd(bbPort, DISABLE);
    if (!bbDMAWaitStopped(bbPort)) {
        return false;
    }
    if (bbPort->dmaDescriptor) {
        DMA_CLEAR_FLAG(bbPort->dmaDescriptor, DMA_IT_GLOB);
    }
    dbgPinHi(1);
    // Output idle level before switching to output
    // Use BSRR register for this
    // Normal: Use BR (higher half)
    // Inverted: Use BS (lower half)

    WRITE_REG(bbPort->gpio->BSHR, bbPort->gpioIdleBSRR);

    // Set GPIO to output
    ATOMIC_BLOCK(NVIC_PRIO_TIMER)
    {
        if (bbPort->gpioCfgLrMask) {
            MODIFY_REG(bbPort->gpio->CFGLR, bbPort->gpioCfgLrMask, bbPort->gpioCfgLrOutput);
        }
        if (bbPort->gpioCfgHrMask) {
            MODIFY_REG(bbPort->gpio->CFGHR, bbPort->gpioCfgHrMask, bbPort->gpioCfgHrOutput);
        }
    }

    // Reinitialize port group DMA for output

#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegOutput);
#else
    xDMA_Init(dmaResource, &bbPort->outputDmaInit);
    xDMA_ITConfig(dmaResource, DMA_IT_TC | DMA_IT_TE, ENABLE);
#endif

    // Reinitialize pacer timer for output

    ((TIM_TypeDef *)bbPort->timhw->tim)->ATRLR = bbPort->outputARR;

    bbPort->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;

    dbgPinLo(1);
    return true;
}

#ifdef USE_DSHOT_TELEMETRY
FAST_CODE
void bbSwitchToInput(bbPort_t *bbPort)
{
    dbgPinHi(1);

    // Set the pull direction before enabling the input configuration.
    WRITE_REG(bbPort->gpio->BSHR, bbPort->gpioInputBSRR);
    ATOMIC_BLOCK(NVIC_PRIO_TIMER)
    {
        if (bbPort->gpioCfgLrMask) {
            MODIFY_REG(bbPort->gpio->CFGLR, bbPort->gpioCfgLrMask, bbPort->gpioCfgLrInput);
        }
        if (bbPort->gpioCfgHrMask) {
            MODIFY_REG(bbPort->gpio->CFGHR, bbPort->gpioCfgHrMask, bbPort->gpioCfgHrInput);
        }
    }

    // Reinitialize port group DMA for input

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegInput);
#else
    xDMA_Init(dmaResource, &bbPort->inputDmaInit);
    xDMA_ITConfig(dmaResource, DMA_IT_TC | DMA_IT_TE, ENABLE);
#endif

    bbDMA_Cmd(bbPort, ENABLE);

    bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;

    dbgPinLo(1);
}
#endif

void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
{
    DMA_InitTypeDef *dmainit =
        (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;

    DMA_StructInit(dmainit);

    dmainit->DMA_Mode = DMA_Mode_Normal;
    dmainit->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmainit->DMA_MemoryInc = DMA_MemoryInc_Enable;

    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        dmainit->DMA_Priority = DMA_Priority_VeryHigh;
        dmainit->DMA_DIR = DMA_DIR_PeripheralDST;
        dmainit->DMA_BufferSize = bbPort->portOutputCount;
        dmainit->DMA_PeripheralBaseAddr = (uint32_t)&bbPort->gpio->BSHR;
        dmainit->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        dmainit->DMA_Memory0BaseAddr = (uint32_t)bbPort->portOutputBuffer;
        dmainit->DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        dmainit->DMA_M2M = DMA_M2M_Disable;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
#endif
    } else {
        dmainit->DMA_Priority = DMA_Priority_VeryHigh;
        dmainit->DMA_DIR = DMA_DIR_PeripheralSRC;
        dmainit->DMA_BufferSize = bbPort->portInputCount;

        dmainit->DMA_PeripheralBaseAddr = (uint32_t)&bbPort->gpio->INDR;

        dmainit->DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        dmainit->DMA_Memory0BaseAddr = (uint32_t)bbPort->portInputBuffer;
        dmainit->DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        dmainit->DMA_M2M = DMA_M2M_Disable;
#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegInput);
#endif
    }
}

void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period)
{
    TIM_TimeBaseInitTypeDef *init = &bbPort->timeBaseInit;

    RCC_ClockCmd(timerRCC(bbPort->timhw->tim), ENABLE);

    init->TIM_Prescaler = 0; // Feed raw timerClock
    init->TIM_ClockDivision = TIM_CKD_DIV1;
    init->TIM_CounterMode = TIM_CounterMode_Up;
    init->TIM_Period = period;
    init->TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit((TIM_TypeDef *)bbPort->timhw->tim, init);
    TIM_ARRPreloadConfig((TIM_TypeDef *)bbPort->timhw->tim, ENABLE);
}

FAST_CODE
void bbTIM_ReloadPeriod(void *TIMx, uint16_t period)
{
    TIM_TypeDef *tim = (TIM_TypeDef *)TIMx;

    tim->ATRLR = period;
    tim->SWEVGR = TIM_PSCReloadMode_Immediate;
    tim->INTFR = (uint16_t)~TIM_FLAG_Update;
}

FAST_CODE
void bbTIM_DMACmd(void *TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
{
    ch32DmaBarrier();
    TIM_DMACmd((TIM_TypeDef *)TIMx, TIM_DMASource, NewState);
}

void bbDMA_ITConfig(bbPort_t *bbPort)
{
    xDMA_ITConfig(bbPort->dmaResource, DMA_IT_TC | DMA_IT_TE, ENABLE);
}

FAST_CODE
void bbDMA_Cmd(bbPort_t *bbPort, FunctionalState NewState)
{
    ch32DmaBarrier();
    xDMA_Cmd(bbPort->dmaResource, NewState);
    ch32DmaBarrier();
}

int bbDMA_Count(bbPort_t *bbPort)
{
    return xDMA_GetCurrDataCounter(bbPort->dmaResource);
}
#endif // USE_DSHOT_BITBANG
