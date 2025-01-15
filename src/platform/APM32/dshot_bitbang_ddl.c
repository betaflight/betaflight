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

    bbPort->gpioModeMask |= (GPIO_MODE_MODE0 << (pinIndex * 2));
    bbPort->gpioModeInput |= (DDL_GPIO_MODE_INPUT << (pinIndex * 2));
    bbPort->gpioModeOutput |= (DDL_GPIO_MODE_OUTPUT << (pinIndex * 2));

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
}

void bbTimerChannelInit(bbPort_t *bbPort)
{
    const timerHardware_t *timhw = bbPort->timhw;

    switch (bbPort->timhw->channel) {
    case TMR_CHANNEL_1: bbPort->llChannel = DDL_TMR_CHANNEL_CH1; break;
    case TMR_CHANNEL_2: bbPort->llChannel = DDL_TMR_CHANNEL_CH2; break;
    case TMR_CHANNEL_3: bbPort->llChannel = DDL_TMR_CHANNEL_CH3; break;
    case TMR_CHANNEL_4: bbPort->llChannel = DDL_TMR_CHANNEL_CH4; break;
    }

    DDL_TMR_OC_InitTypeDef ocInit;
    DDL_TMR_OC_StructInit(&ocInit);
    ocInit.OCMode = DDL_TMR_OCMODE_PWM1;
    ocInit.OCIdleState = DDL_TMR_OCIDLESTATE_HIGH;
    ocInit.OCState = DDL_TMR_OCSTATE_ENABLE;
    ocInit.OCPolarity = DDL_TMR_OCPOLARITY_LOW;
    ocInit.CompareValue = 10; // Duty doesn't matter, but too value small would make monitor output invalid

    DDL_TMR_DisableCounter(bbPort->timhw->tim);

    DDL_TMR_OC_Init(timhw->tim, bbPort->llChannel, &ocInit);

    DDL_TMR_OC_EnablePreload(timhw->tim, bbPort->llChannel);

#ifdef DEBUG_MONITOR_PACER
    if (timhw->tag) {
        IO_t io = IOGetByTag(timhw->tag);
        IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
        IOInit(io, OWNER_DSHOT_BITBANG, 0);
        DDL_TMR_EnableAllOutputs(timhw->tim);
    }
#endif

    // Enable and keep it running
    DDL_TMR_EnableCounter(bbPort->timhw->tim);
}

#ifdef USE_DMA_REGISTER_CACHE
void bbLoadDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    ((DMA_ARCH_TYPE *)dmaResource)->SCFG = dmaRegCache->SCFG;
    ((DMA_ARCH_TYPE *)dmaResource)->FCTRL = dmaRegCache->FCTRL;
    ((DMA_ARCH_TYPE *)dmaResource)->NDATA = dmaRegCache->NDATA;
    ((DMA_ARCH_TYPE *)dmaResource)->PADDR = dmaRegCache->PADDR;
    ((DMA_ARCH_TYPE *)dmaResource)->M0ADDR = dmaRegCache->M0ADDR;
}

static void bbSaveDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    dmaRegCache->SCFG = ((DMA_ARCH_TYPE *)dmaResource)->SCFG;
    dmaRegCache->FCTRL = ((DMA_ARCH_TYPE *)dmaResource)->FCTRL;
    dmaRegCache->NDATA = ((DMA_ARCH_TYPE *)dmaResource)->NDATA;
    dmaRegCache->PADDR = ((DMA_ARCH_TYPE *)dmaResource)->PADDR;
    dmaRegCache->M0ADDR = ((DMA_ARCH_TYPE *)dmaResource)->M0ADDR;
}
#endif

void bbSwitchToOutput(bbPort_t * bbPort)
{
    // Output idle level before switching to output
    // Use BSC register for this
    // Normal: Use BC (higher half)
    // Inverted: Use BS (lower half)

    WRITE_REG(bbPort->gpio->BSC, bbPort->gpioIdleBSRR);

    // Set GPIO to output

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->MODE, bbPort->gpioModeMask, bbPort->gpioModeOutput);
    }

    // Reinitialize port group DMA for output

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbDMA_Cmd(bbPort, DISABLE);
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegOutput);
#else
    xDDL_EX_DMA_Deinit(dmaResource);
    xDDL_EX_DMA_Init(dmaResource, &bbPort->outputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDDL_EX_DMA_EnableIT_TC(dmaResource);
#endif

    // Reinitialize pacer timer for output

    bbPort->timhw->tim->AUTORLD = bbPort->outputARR;

    bbPort->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;
}

#ifdef USE_DSHOT_TELEMETRY
void bbSwitchToInput(bbPort_t *bbPort)
{
    // Set GPIO to input

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->MODE, bbPort->gpioModeMask, bbPort->gpioModeInput);
    }

    // Reinitialize port group DMA for input

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegInput);
#else
    xDDL_EX_DMA_Deinit(dmaResource);
    xDDL_EX_DMA_Init(dmaResource, &bbPort->inputDmaInit);

    // Needs this, as it is DeInit'ed above...
    xDDL_EX_DMA_EnableIT_TC(dmaResource);
#endif

    // Reinitialize pacer timer for input

    bbPort->timhw->tim->AUTORLD = bbPort->inputARR;

    bbDMA_Cmd(bbPort, ENABLE);

    bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;
}
#endif

void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
{
    DDL_DMA_InitTypeDef *dmainit = (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;

    DDL_DMA_StructInit(dmainit);

    dmainit->Mode = DDL_DMA_MODE_NORMAL;
    dmainit->Channel = bbPort->dmaChannel;
    dmainit->FIFOMode = DDL_DMA_FIFOMODE_ENABLE ;

    dmainit->PeriphOrM2MSrcIncMode = DDL_DMA_PERIPH_NOINCREMENT;
    dmainit->MemoryOrM2MDstIncMode = DDL_DMA_MEMORY_INCREMENT;

    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        dmainit->Priority = DDL_DMA_PRIORITY_VERYHIGH;
        dmainit->Direction = DDL_DMA_DIRECTION_MEMORY_TO_PERIPH;
        dmainit->NbData = bbPort->portOutputCount;
        dmainit->PeriphOrM2MSrcAddress = (uint32_t)&bbPort->gpio->BSC;
        dmainit->PeriphOrM2MSrcDataSize = DDL_DMA_PDATAALIGN_WORD;
        dmainit->MemoryOrM2MDstAddress = (uint32_t)bbPort->portOutputBuffer;
        dmainit->MemoryOrM2MDstDataSize = DDL_DMA_MDATAALIGN_WORD;

#ifdef USE_DMA_REGISTER_CACHE
        xDDL_EX_DMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
#endif
    } else {
        dmainit->Priority = DDL_DMA_PRIORITY_HIGH;
        dmainit->Direction = DDL_DMA_DIRECTION_PERIPH_TO_MEMORY;
        dmainit->NbData = bbPort->portInputCount;

        dmainit->PeriphOrM2MSrcAddress = (uint32_t)&bbPort->gpio->IDATA;
        dmainit->PeriphOrM2MSrcDataSize = DDL_DMA_PDATAALIGN_HALFWORD;
        dmainit->MemoryOrM2MDstAddress = (uint32_t)bbPort->portInputBuffer;
        dmainit->MemoryOrM2MDstDataSize = DDL_DMA_MDATAALIGN_WORD;

#ifdef USE_DMA_REGISTER_CACHE
        xDDL_EX_DMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegInput);
#endif
    }
}

void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period)
{
    DDL_TMR_InitTypeDef *init = &bbPort->timeBaseInit;

    init->Prescaler = 0; // Feed raw timerClock
    init->ClockDivision = DDL_TMR_CLOCKDIVISION_DIV1;
    init->CounterMode = DDL_TMR_COUNTERMODE_UP;
    init->Autoreload = period;
    //TIM_TimeBaseInit(bbPort->timhw->tim, &bbPort->timeBaseInit);
    DDL_TMR_Init(bbPort->timhw->tim, init);
    MODIFY_REG(bbPort->timhw->tim->CTRL1, TMR_CTRL1_ARPEN, TMR_AUTORELOAD_PRELOAD_ENABLE);
}

void bbTIM_DMACmd(TMR_TypeDef* TMRx, uint16_t TIM_DMASource, FunctionalState NewState)
{
    if (NewState == ENABLE) {
        SET_BIT(TMRx->DIEN, TIM_DMASource);
    } else {
        CLEAR_BIT(TMRx->DIEN, TIM_DMASource);
    }
}

void bbDMA_ITConfig(bbPort_t *bbPort)
{
    xDDL_EX_DMA_EnableIT_TC(bbPort->dmaResource);

    SET_BIT(((DMA_Stream_TypeDef *)(bbPort->dmaResource))->SCFG, DMA_SCFGx_TXCIEN|DMA_SCFGx_TXEIEN);
}

void bbDMA_Cmd(bbPort_t *bbPort, FunctionalState NewState)
{
    if (NewState == ENABLE) {
        xDDL_EX_DMA_EnableResource(bbPort->dmaResource);
    } else {
        xDDL_EX_DMA_DisableResource(bbPort->dmaResource);
    }
}

int bbDMA_Count(bbPort_t *bbPort)
{
    return xDDL_EX_DMA_GetDataLength(bbPort->dmaResource);
}
#endif // USE_DSHOT_BITBANG
