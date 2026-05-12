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
#include "build/debug_pin.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/dshot.h"
#include "dshot_bitbang_impl.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "platform/timer.h"

#include "pg/motor.h"

void bbGpioSetup(bbMotor_t *bbMotor)
{
    bbPort_t *bbPort = bbMotor->bbPort;
    int pinIndex = bbMotor->pinIndex;

    bbPort->gpioModeMask |= (3 << (pinIndex * 2));
    bbPort->gpioModeInput |= (GPIO_PMODE_INPUT << (pinIndex * 2));
    bbPort->gpioModeOutput |= (GPIO_PMODE_OUTPUT << (pinIndex * 2));

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

    TIM_OCInitTypeDef TIM_OCStruct;

    TIM_OCStructInit(&TIM_OCStruct);
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OCStruct.TIM_Pulse = 10; // Duty doesn't matter, but too value small would make monitor output invalid

    TIM_Cmd((TIM_TypeDef *)bbPort->timhw->tim, DISABLE);

    timerOCInit((TIM_TypeDef *)timhw->tim, timhw->channel, &TIM_OCStruct);
    // timerOCPreloadConfig(timhw->tim, timhw->channel, TIM_OCPreload_Enable);

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


void bbTimerChannelInit(bbPort_t *bbPort)
{
    const timerHardware_t *timhw = bbPort->timhw;

    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_InitOcStruct(&TIM_OCInitStructure);
    TIM_OCInitStructure.OCMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.OCIdleState = TIM_OC_IDLE_STATE_SET;
    TIM_OCInitStructure.Pulse       = 10;
    TIM_OCInitStructure.OCPolarity  = TIM_OC_POLARITY_LOW;

    TIM_OCInitStructure.TIM_Pulse = 10; // Duty doesn't matter, but too value small would make monitor output invalid

    TIM_Cmd((TIM_TypeDef *)bbPort->timhw->tim, DISABLE);

    timerOCInit((TIM_TypeDef *)timhw->tim, timhw->channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(timhw->tim, timhw->channel, TIM_OC_PRE_LOAD_ENABLE);

#ifdef DEBUG_MONITOR_PACER
    if (timhw->tag) {
        IO_t io = IOGetByTag(timhw->tag);
        IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
        IOInit(io, OWNER_DSHOT_BITBANG, 0);
        TIM_EnableCtrlPwmOutputs((TIM_TypeDef *)timhw->tim, ENABLE);
    }
#endif

    // Enable and keep it running

    TIM_Cmd((TIM_TypeDef *)bbPort->timhw->tim, ENABLE);
}

#ifdef USE_DMA_REGISTER_CACHE

static void bbLoadDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    ((DMA_Stream_TypeDef *)dmaResource)->SA = dmaRegCache->SA;
    ((DMA_Stream_TypeDef *)dmaResource)->DA = dmaRegCache->DA;
    ((DMA_Stream_TypeDef *)dmaResource)->LLP = dmaRegCache->LLP;
    ((DMA_Stream_TypeDef *)dmaResource)->CTRL = dmaRegCache->CTRL;
    ((DMA_Stream_TypeDef *)dmaResource)->CFG = dmaRegCache->CFG;
    ((DMA_Stream_TypeDef *)dmaResource)->SG = dmaRegCache->SG;
    ((DMA_Stream_TypeDef *)dmaResource)->DS = dmaRegCache->DS;
}

static void bbSaveDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    dmaRegCache->SA = ((DMA_Stream_TypeDef *)dmaResource)->SA;
    dmaRegCache->DA = ((DMA_Stream_TypeDef *)dmaResource)->DA;
    dmaRegCache->LLP = ((DMA_Stream_TypeDef *)dmaResource)->LLP;
    dmaRegCache->CTRL = ((DMA_Stream_TypeDef *)dmaResource)->CTRL;
    dmaRegCache->CFG = ((DMA_Stream_TypeDef *)dmaResource)->CFG;
    dmaRegCache->SG = ((DMA_Stream_TypeDef *)dmaResource)->SG;
    dmaRegCache->DS = ((DMA_Stream_TypeDef *)dmaResource)->DS;
}
#endif

void bbSwitchToOutput(bbPort_t * bbPort)
{
    dbgPinHi(1);
    // Output idle level before switching to output
    // Use BSRR register for this
    // Normal: Use BR (higher half)
    // Inverted: Use BS (lower half)

    WRITE_REG(bbPort->gpio->PBSC, bbPort->gpioIdleBSRR);

    // Set GPIO to output
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->PMODE, bbPort->gpioModeMask, bbPort->gpioModeOutput);
    }

    // Reinitialize port group DMA for output

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegOutput);
#else
    xDMA_DeInit(dmaResource);
    xDMA_Init(dmaResource, &bbPort->outputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDMA_ITConfig(dmaResource, DMA_CH_EVENT_TRANSFER_COMPLETE, ENABLE);
#endif

    // Reinitialize pacer timer for output

    ((TIM_TypeDef *)bbPort->timhw->tim)->AR = bbPort->outputARR;

    bbPort->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;

    dbgPinLo(1);
}

#ifdef USE_DSHOT_TELEMETRY
void bbSwitchToInput(bbPort_t *bbPort)
{
    dbgPinHi(1);

    // Set GPIO to input

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->PMODE, bbPort->gpioModeMask, bbPort->gpioModeInput);
    }

    // Reinitialize port group DMA for input

    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegInput);
#else
    xDMA_DeInit(dmaResource);
    xDMA_Init(dmaResource, &bbPort->inputDmaInit);
    // Needs this, as it is DeInit'ed above...
    xDMA_ITConfig(dmaResource, DMA_CH_EVENT_TRANSFER_COMPLETE, ENABLE);
#endif

    // Reinitialize pacer timer for input

    ((TIM_TypeDef *)bbPort->timhw->tim)->CNT = 0;
    ((TIM_TypeDef *)bbPort->timhw->tim)->AR = bbPort->inputARR;

    bbDMA_Cmd(bbPort, ENABLE);

    bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;

    dbgPinLo(1);
}
#endif

void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
{
    DMA_InitTypeDef *DMA_InitStructure = (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;

    DMA_ControllerCmd(DMA1, ENABLE);
    DMA_ChannelStructInit(DMA_InitStructure);
    DMA_InitStructure->IntEn              = 0x1U;
    DMA_InitStructure->DstTfrWidth        = DMA_CH_TRANSFER_WIDTH_32;
    DMA_InitStructure->SrcTfrWidth        = DMA_CH_TRANSFER_WIDTH_32;
    DMA_InitStructure->DstBurstLen        = DMA_CH_BURST_LENGTH_1;
    DMA_InitStructure->SrcBurstLen        = DMA_CH_BURST_LENGTH_1;
    DMA_InitStructure->TfrType            = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
    DMA_InitStructure->ChannelPriority    = DMA_CH_PRIORITY_0;
    DMA_InitStructure->SrcHandshaking     = DMA_CH_SRC_HANDSHAKING_HARDWARE;
    DMA_InitStructure->SrcHsInterface     = DMA_CH_HARDWARE_HANDSHAKING_IF_0;
    DMA_InitStructure->SrcHsInterfacePol  = DMA_CH_HANDSHAKING_IF_POL_H;
    DMA_ChannelInit(bbPort->dmaResource, &DMA_InitStructure, bbPort->dmaChannel);

    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        DMA_InitStructure->ChannelPriority = DMA_CH_PRIORITY_0;
        DMA_InitStructure->DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure->TfrTypeFlowCtrl    = DMA_CH_TRANSFER_FLOW_M2P_DMA;
        DMA_InitStructure->BlkTfrSize         = bbPort->portOutputCount;
        DMA_InitStructure->SrcAddr            = (uint32_t)bbPort->portOutputBuffer;
        DMA_InitStructure->DstAddr            = (uint32_t)&bbPort->gpio->PBSC;
        DMA_InitStructure->DstAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        DMA_InitStructure->SrcAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, dmainit);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
#endif
    } else {
        DMA_InitStructure->ChannelPriority = DMA_CH_PRIORITY_1;
        DMA_InitStructure->TfrTypeFlowCtrl    = DMA_CH_TRANSFER_FLOW_P2M_DMA;
        DMA_InitStructure->BlkTfrSize         = bbPort->portInputCount;
        DMA_InitStructure->SrcAddr            = (uint32_t)&bbPort->gpio->PID;
        DMA_InitStructure->DstAddr            = (uint32_t)bbPort->portInputBuffer;
        DMA_InitStructure->DstAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
        DMA_InitStructure->SrcAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        DMA_InitStructure->SrcTfrWidth        = DMA_CH_TRANSFER_WIDTH_16;

#ifdef USE_DMA_REGISTER_CACHE
        xDMA_Init(bbPort->dmaResource, DMA_InitStructure);
        bbSaveDMARegs(bbPort->dmaResource, &bbPort->dmaRegInput);
#endif
    }
}

void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period)
{
    TIM_TimeBaseInitTypeDef *TIM_TimeBaseStructure = &bbPort->timeBaseInit;

    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period    = period;
    TIM_TimeBaseStructure.Prescaler = 0;
    TIM_TimeBaseStructure.ClkDiv    = TIM_CLK_DIV1;
    TIM_TimeBaseStructure.CounterMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase((TIM_TypeDef *)bbPort->timhw->tim, &TIM_TimeBaseStructure);

    TIM_ConfigArPreload((TIM_TypeDef *)bbPort->timhw->tim, ENABLE); 
}

void bbTIM_DMACmd(void *TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
{
    TIM_EnableDma((TIM_TypeDef *)TIMx, TIM_DMASource, NewState);
}

void bbDMA_ITConfig(bbPort_t *bbPort)
{
    xDMA_ITConfig(bbPort->dmaResource, DMA_CH_EVENT_TRANSFER_COMPLETE, ENABLE);
}

void bbDMA_Cmd(bbPort_t *bbPort, FunctionalState NewState)
{
    xDMA_Cmd(bbPort->dmaResource, NewState);
}

int bbDMA_Count(bbPort_t *bbPort)
{
    return xDMA_GetCurrDataCounter(bbPort->dmaResource);
}

#endif // USE_DSHOT_BB
