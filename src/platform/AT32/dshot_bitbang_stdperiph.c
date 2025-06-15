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
#include "drivers/dma_reqmap.h"
#include "drivers/dshot.h"
#include "dshot_bitbang_impl.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

void bbGpioSetup(bbMotor_t *bbMotor)
{
    bbPort_t *bbPort = bbMotor->bbPort;
    int pinIndex = bbMotor->pinIndex;

    bbPort->gpioModeMask |= (0x03 << (pinIndex * 2));
    bbPort->gpioModeInput |= (GPIO_MODE_INPUT << (pinIndex * 2));
    bbPort->gpioModeOutput |= (GPIO_MODE_OUTPUT << (pinIndex * 2));

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

    // is this needed here?
    IOConfigGPIO(bbMotor->io, IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, bbPuPdMode));
}

void bbTimerChannelInit(bbPort_t *bbPort)
{
    const timerHardware_t *timhw = bbPort->timhw;

    TIM_OCInitTypeDef TIM_OCStruct;

    TIM_OCStructInit(&TIM_OCStruct);

    TIM_OCStruct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    TIM_OCStruct.oc_idle_state = TRUE;
    TIM_OCStruct.oc_output_state = TRUE;
    TIM_OCStruct.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;

    // TIM_OCStruct.TIM_Pulse = 10; // Duty doesn't matter, but too value small would make monitor output invalid

    tmr_channel_value_set(timhw->tim, TIM_CH_TO_SELCHANNEL(timhw->channel), 10);

    TIM_Cmd(timhw->tim, FALSE);

    timerOCInit(timhw->tim, timhw->channel, &TIM_OCStruct);

    tmr_channel_enable(timhw->tim, TIM_CH_TO_SELCHANNEL(timhw->channel), TRUE);

    timerOCPreloadConfig(timhw->tim, timhw->channel, TRUE);

#ifdef DEBUG_MONITOR_PACER
    if (timhw->tag) {
        IO_t io = IOGetByTag(timhw->tag);
        IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
        IOInit(io, OWNER_DSHOT_BITBANG, 0);
        TIM_CtrlPWMOutputs(timhw->tim, TRUE);
    }
#endif

    // Enable and keep it running

    TIM_Cmd(timhw->tim, TRUE);
}

#ifdef USE_DMA_REGISTER_CACHE

static void bbLoadDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    ((DMA_ARCH_TYPE *)dmaResource)->ctrl = dmaRegCache->CCR;
    ((DMA_ARCH_TYPE *)dmaResource)->dtcnt = dmaRegCache->CNDTR;
    ((DMA_ARCH_TYPE *)dmaResource)->paddr = dmaRegCache->CPAR;
    ((DMA_ARCH_TYPE *)dmaResource)->maddr = dmaRegCache->CMAR;
}

static void bbSaveDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    dmaRegCache->CCR = ((DMA_ARCH_TYPE *)dmaResource)->ctrl;
	dmaRegCache->CNDTR = ((DMA_ARCH_TYPE *)dmaResource)->dtcnt;
	dmaRegCache->CPAR = ((DMA_ARCH_TYPE *)dmaResource)->paddr ;
	dmaRegCache->CMAR = ((DMA_ARCH_TYPE *)dmaResource)->maddr ;
}
#endif

void bbSwitchToOutput(bbPort_t * bbPort)
{
    dbgPinHi(1);
    // Output idle level before switching to output
    // Use BSRR register for this
    // Normal: Use BR (higher half)
    // Inverted: Use BS (lower half)

    WRITE_REG(bbPort->gpio->scr, bbPort->gpioIdleBSRR);

    // Set GPIO to output
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->cfgr, bbPort->gpioModeMask, bbPort->gpioModeOutput);
    }

    // Reinitialize port group DMA for output

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegOutput);
#else
    xDMA_DeInit(dmaResource);
    xDMA_Init(dmaResource, &bbPort->outputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDMA_ITConfig(dmaResource, DMA_IT_TC, ENABLE);
#endif

    // Reinitialize pacer timer for output

    bbPort->timhw->tim->pr = bbPort->outputARR;

    bbPort->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;

    dbgPinLo(1);
}

#ifdef USE_DSHOT_TELEMETRY
void bbSwitchToInput(bbPort_t *bbPort)
{
    dbgPinHi(1);

    // Set GPIO to input

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->cfgr, bbPort->gpioModeMask, bbPort->gpioModeInput);
    }

    // Reinitialize port group DMA for input

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegInput);
#else
    xDMA_DeInit(dmaResource);
    xDMA_Init(dmaResource, &bbPort->inputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDMA_ITConfig(dmaResource, DMA_IT_TC, ENABLE);
#endif

    // Reinitialize pacer timer for input

    bbPort->timhw->tim->cval = 0;
    bbPort->timhw->tim->pr = bbPort->inputARR;

    bbDMA_Cmd(bbPort, TRUE);

    bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;

    dbgPinLo(1);
}
#endif

void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
{
    DMA_InitTypeDef *dmainit = (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;

    dma_default_para_init(dmainit);

    dmainit->loop_mode_enable = FALSE;
    // dmainit->DMA_Channel = bbPort->dmaChannel;
    dmainit->peripheral_inc_enable = FALSE;
    dmainit->memory_inc_enable = TRUE;
    /* dmainit->DMA_FIFOMode = DMA_FIFOMode_Enable ;
    dmainit->DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    dmainit->DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    dmainit->DMA_PeripheralBurst = DMA_PeripheralBurst_Single; */

    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        dmainit->priority = DMA_PRIORITY_VERY_HIGH;
        dmainit->direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        dmainit->buffer_size = bbPort->portOutputCount;
        dmainit->peripheral_base_addr = (uint32_t)&bbPort->gpio->scr;
        dmainit->peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
        dmainit->memory_base_addr = (uint32_t)bbPort->portOutputBuffer;
        dmainit->memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
#endif
    } else {
        dmainit->priority = DMA_PRIORITY_VERY_HIGH;
        dmainit->direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
        dmainit->buffer_size = bbPort->portInputCount;

        dmainit->peripheral_base_addr = (uint32_t)&bbPort->gpio->idt;
        dmainit->peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
        dmainit->memory_base_addr = (uint32_t)bbPort->portInputBuffer;
        dmainit->memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegInput);
#endif
    }
}

void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period)
{
    /*TIM_TimeBaseInitTypeDef *init = &bbPort->timeBaseInit;

    init->TIM_Prescaler = 0; // Feed raw timerClock
    init->TIM_ClockDivision = TMR_CLOCK_DIV1;
    init->TIM_CounterMode = TMR_COUNT_UP;
    init->TIM_Period = period; */

    tmr_base_init(bbPort->timhw->tim, period, 0);
    tmr_clock_source_div_set(bbPort->timhw->tim, TMR_CLOCK_DIV1);
    tmr_cnt_dir_set(bbPort->timhw->tim, TMR_COUNT_UP);
    tmr_period_buffer_enable(bbPort->timhw->tim, TRUE);

    //TIM_TimeBaseInit(bbPort->timhw->tim, init, DISABLE);

    tmr_period_buffer_enable(bbPort->timhw->tim, TRUE);
}

void bbTIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, confirm_state NewState)
{
    // TIM_DMACmd(TIMx, TIM_DMASource, NewState);
    tmr_dma_request_enable(TIMx, TIM_DMASource, NewState);
}

void bbDMA_ITConfig(bbPort_t *bbPort)
{
    xDMA_ITConfig(bbPort->dmaResource, DMA_IT_TCIF, TRUE);
}

void bbDMA_Cmd(bbPort_t *bbPort, confirm_state NewState)
{
    xDMA_Cmd(bbPort->dmaResource, NewState);
}

int bbDMA_Count(bbPort_t *bbPort)
{
    return xDMA_GetCurrDataCounter(bbPort->dmaResource);
}

#endif // USE_DSHOT_BB
