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
#include "platform/dma.h"
#include "drivers/dshot.h"
#include "dshot_bitbang_impl.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "pwm_output_dshot_shared.h"

#include "pg/motor.h"

#ifdef USE_DMA_REGISTER_CACHE
static void bbLoadDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache);
static void bbSaveDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache);
#endif

// Setup GPIO mode register manipulation values for bbPort

void bbGpioSetup(bbMotor_t *bbMotor)
{
    bbPort_t *bbPort = bbMotor->bbPort;
    int pinIndex = bbMotor->pinIndex;
    
    // MODE register mask (2 bits per pin) for direct register manipulation
    bbPort->gpioModeMask  |= (GPIO_MODE << (pinIndex * 2));
    // MODE register value: 00 = input
    bbPort->gpioModeInput  |= (MODE_INPUT << (pinIndex * 2));
    // MODE register value: 01 = output
    bbPort->gpioModeOutput |= (MODE_OUTPUT << (pinIndex * 2));

    // Idle value pin mask (used with SET/CLR register, not BSRR)
    bbPort->gpioIdleBSRR |= (1 << pinIndex);

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
    ocInit.CompareValue = 10;

    LL_TIM_DisableCounter((TIM_TypeDef *)bbPort->timhw->tim);

    LL_TIM_OC_Init((TIM_TypeDef *)timhw->tim, bbPort->llChannel, &ocInit);

    LL_TIM_OC_EnablePreload((TIM_TypeDef *)timhw->tim, bbPort->llChannel);

    // Explicitly enable the CC channel (required for DMA request generation on UM324)
    LL_TIM_CC_EnableChannel((TIM_TypeDef *)timhw->tim, bbPort->llChannel);

#ifdef DEBUG_MONITOR_PACER
    if (timhw->tag) {
        IO_t io = IOGetByTag(timhw->tag);
        IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
        IOInit(io, OWNER_DSHOT_BITBANG, 0);
    }
#endif
    LL_TIM_EnableAllOutputs((TIM_TypeDef *)timhw->tim);

    // Enable and keep it running
    LL_TIM_EnableCounter((TIM_TypeDef *)bbPort->timhw->tim);
}

void bbSwitchToOutput(bbPort_t * bbPort)
{
    // Set idle output level via SET/CLR register (affects only motor pins)
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        bbPort->gpio->SET = bbPort->gpioIdleBSRR;   // idle HIGH
    } else
#endif
    {
        bbPort->gpio->CLR = bbPort->gpioIdleBSRR;   // idle LOW
    }

    // Switch pins to OUTPUT mode
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->MODE, bbPort->gpioModeMask, bbPort->gpioModeOutput);
    }

    // Reinitialize port group DMA for output
    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbDMA_Cmd(bbPort, DISABLE);
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegOutput);
#else
    xLL_EX_DMA_DeInit(dmaResource);
    xLL_EX_DMA_Init(dmaResource, &bbPort->outputDmaInit);
    xLL_EX_DMA_EnableIT_TC(dmaResource);
#endif

    // Reinitialize pacer timer for output
    ((TIM_TypeDef *)bbPort->timhw->tim)->ARR = bbPort->outputARR;

    bbPort->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;
}

#ifdef USE_DMA_REGISTER_CACHE
static void bbLoadDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    ((DMA_ARCH_TYPE *)dmaResource)->SAR  = dmaRegCache->SAR;
    ((DMA_ARCH_TYPE *)dmaResource)->DAR  = dmaRegCache->DAR;
    ((DMA_ARCH_TYPE *)dmaResource)->CTLL = dmaRegCache->CTLL;
    ((DMA_ARCH_TYPE *)dmaResource)->CTLH = dmaRegCache->CTLH;
    ((DMA_ARCH_TYPE *)dmaResource)->CFGL = dmaRegCache->CFGL;
    ((DMA_ARCH_TYPE *)dmaResource)->CFGH = dmaRegCache->CFGH;
}

static void bbSaveDMARegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
{
    dmaRegCache->SAR = ((DMA_ARCH_TYPE *)dmaResource)->SAR;
    dmaRegCache->DAR = ((DMA_ARCH_TYPE *)dmaResource)->DAR;
    dmaRegCache->CTLL = ((DMA_ARCH_TYPE *)dmaResource)->CTLL;
    dmaRegCache->CTLH = ((DMA_ARCH_TYPE *)dmaResource)->CTLH;
    dmaRegCache->CFGL = ((DMA_ARCH_TYPE *)dmaResource)->CFGL;
    dmaRegCache->CFGH = ((DMA_ARCH_TYPE *)dmaResource)->CFGH;
}
#endif

#ifdef USE_DSHOT_TELEMETRY
void bbSwitchToInput(bbPort_t *bbPort)
{
    // Switch pins to INPUT mode (MODE_INPUT value = 0)
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(bbPort->gpio->MODE, bbPort->gpioModeMask, bbPort->gpioModeInput);
    }

    // Reinitialize port group DMA for input
    dmaResource_t *dmaResource = bbPort->dmaResource;
#ifdef USE_DMA_REGISTER_CACHE
    bbLoadDMARegs(dmaResource, &bbPort->dmaRegInput);
#else
    xLL_EX_DMA_DeInit(dmaResource);
    xLL_EX_DMA_Init(dmaResource, &bbPort->inputDmaInit);
    xLL_EX_DMA_EnableIT_TC(dmaResource);
#endif

    // Reinitialize pacer timer for input
    ((TIM_TypeDef *)bbPort->timhw->tim)->ARR = bbPort->inputARR;

    bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;

    bbDMA_Cmd(bbPort, ENABLE);
}
#endif

void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
{
    LL_DMA_InitTypeDef *dmainit = (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimerValue(bbPort->timhw->tim, bbPort->timhw->channel, dmaGetOptionByTimer(bbPort->timhw));

    // Common settings
    dmainit->Channel = bbPort->dmaChannel;
    dmainit->SrcMSize = LL_DMA_BURST_SRC_NUM_1;
    dmainit->DstMSize = LL_DMA_BURST_DST_NUM_1;
    dmainit->FIFOMode = LL_DMA_FIFOMODE_DISABLE;
    dmainit->FCMode = LL_DMA_FCMODE_DISABLE;
    dmainit->Priority = LL_DMA_PRIORITY_6;
    dmainit->SrcHsSel = LL_DMA_SRC_HS_HW;
    dmainit->DstHsSel = LL_DMA_DST_HS_HW;
    dmainit->SrcReload = LL_DMA_SRC_RELOAD_DISABLE;
    dmainit->DstReload = LL_DMA_DST_RELOAD_DISABLE;

    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        dmainit->SrcAddress = (uint32_t)bbPort->portOutputBuffer;
        dmainit->DstAddress = (uint32_t)&bbPort->gpio->ODATA;
        dmainit->Direction = LL_DMA_MEMORY_TO_PERIPH;
        dmainit->SrcInc = LL_DMA_SRCINC_INC;
        dmainit->DstInc = LL_DMA_DSTINC_NOC;
        dmainit->SrcDataAlignment = LL_DMA_SRCDATAALIGN_WORD;
        dmainit->DstDataAlignment = LL_DMA_DSTDATAALIGN_WORD;
        dmainit->SrcPer = DMA_SRC_HANDSHAKING(0xF);
        dmainit->DstPer = DMA_DST_HANDSHAKING(dmaSpec->code);
        dmainit->NbData = bbPort->portOutputCount;
    } else {
        dmainit->SrcAddress = (uint32_t)&bbPort->gpio->IDATA;
        dmainit->DstAddress = (uint32_t)bbPort->portInputBuffer;
        dmainit->Direction = LL_DMA_PERIPH_TO_MEMORY;
        dmainit->SrcInc = LL_DMA_SRCINC_NOC;
        dmainit->DstInc = LL_DMA_DSTINC_INC;
        dmainit->SrcDataAlignment = LL_DMA_SRCDATAALIGN_HALFWORD;
        dmainit->DstDataAlignment = LL_DMA_DSTDATAALIGN_HALFWORD;
        dmainit->SrcPer = DMA_SRC_HANDSHAKING(dmaSpec->code);
        dmainit->DstPer = DMA_DST_HANDSHAKING(0xF);
        dmainit->NbData = bbPort->portInputCount;
    }

#ifdef USE_DMA_REGISTER_CACHE
    // Init DMA with this config, then capture register state into cache
    xLL_EX_DMA_Init(bbPort->dmaResource, dmainit);
    bbSaveDMARegs(bbPort->dmaResource,
                  direction == DSHOT_BITBANG_DIRECTION_OUTPUT
                      ? &bbPort->dmaRegOutput
                      : &bbPort->dmaRegInput);
#endif
}

void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period)
{
    LL_TIM_InitTypeDef *init = &bbPort->timeBaseInit;

    init->Prescaler = 0; // Feed raw timerClock
    init->ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    init->CounterMode = LL_TIM_COUNTERMODE_UP;
    init->Autoreload = period;
    LL_TIM_Init((TIM_TypeDef *)bbPort->timhw->tim, init);
    MODIFY_REG(((TIM_TypeDef *)bbPort->timhw->tim)->CR1, TIM_CR1_ARPE, TIM_AUTORELOAD_PRELOAD_ENABLE);
}

void bbTIM_DMACmd(void *TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
{
    if (NewState == ENABLE) {
        SET_BIT(((TIM_TypeDef *)TIMx)->DIER, TIM_DMASource);
    } else {
        CLEAR_BIT(((TIM_TypeDef *)TIMx)->DIER, TIM_DMASource);
    }
}

void bbDMA_ITConfig(bbPort_t *bbPort)
{
    xLL_EX_DMA_EnableIT_TC(bbPort->dmaResource);
}

void bbDMA_Cmd(bbPort_t *bbPort, FunctionalState NewState)
{
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
