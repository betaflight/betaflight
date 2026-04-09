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
#include <math.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/debug.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "platform/timer.h"
#include "drivers/system.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot.h"
#include "dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "pwm_output_dshot_shared.h"
#include "platform/io_impl.h"

#ifdef USE_DSHOT_TELEMETRY

void dshotEnableChannels(unsigned motorCount)
{
    for (unsigned i = 0; i < motorCount; i++) {
        // TIM_SelectOCxM(dmaMotors[i].timerHardware->tim,dmaMotors[i].timerHardware->channel,TIM_ForcedAction_InActive);  //output idle
        if (dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL) {
            TIM_CCxNCmd((TIM_TypeDef *)dmaMotors[i].timerHardware->tim, dmaMotors[i].timerHardware->channel, TIM_CCxN_Enable);
        } else {
            TIM_CCxCmd((TIM_TypeDef *)dmaMotors[i].timerHardware->tim, dmaMotors[i].timerHardware->channel, TIM_CCx_Enable);
        }
    }
}

#endif

FAST_CODE void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor
#ifndef USE_DSHOT_TELEMETRY
    ,TIM_OCInitTypeDef *pOcInit, DMA_InitTypeDef* pDmaInit
#endif
)
{
#ifdef USE_DSHOT_TELEMETRY
    TIM_OCInitTypeDef* pOcInit = &motor->ocInitStruct;
    DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;
#endif

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = (TIM_TypeDef *)timerHardware->tim;

    dmaResource_t *dmaRef = motor->dmaRef;

#if defined(USE_DSHOT_DMAR) && !defined(USE_DSHOT_TELEMETRY)
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
    }
#endif

    // xDMA_DeInit(dmaRef);
    // dmaRef->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
    ((DMA_ARCH_TYPE *)(dmaRef))->CFGR &= (uint16_t)(~DMA_CFGR1_EN);

#ifdef USE_DSHOT_TELEMETRY
    motor->isInput = false;
#endif

    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Disable);

    timerOCInit(timer, timerHardware->channel, pOcInit); 

    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);

    pDmaInit->DMA_DIR = DMA_DIR_PeripheralDST;

    xDMA_Init(dmaRef, pDmaInit);
    xDMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);

#ifdef USE_DSHOT_TELEMETRY
    //for BiDSHOT
    
    {

    // const IO_t motorIO = IOGetByTag(timerHardware->tag);
    // motor->iocfg = IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE);
    // // IOConfigGPIO(motorIO,IOCFG_OUT_PP);
    // // IOHi(motorIO);
    // IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

    const IO_t motorIO = IOGetByTag(timerHardware->tag);
    GPIO_TypeDef* portx = IO_GPIO(motorIO);
    uint16_t  gpio_pin = IO_Pin(motorIO);

    if(gpio_pin > 0x80)
    {   
       gpio_pin = gpio_pin >> 8; 
       uint32_t mulh =  gpio_pin*gpio_pin*gpio_pin*gpio_pin;
       portx->CFGHR  &= ~(0xF*mulh);
       portx->CFGHR |= (0xB*mulh);
    }
    else
    {
       uint32_t mull =  gpio_pin*gpio_pin*gpio_pin*gpio_pin;
       portx->CFGLR  &= ~(0xF*mull);
       portx->CFGLR |= (0xB*mull);
    }

    }

#endif

}

#ifdef USE_DSHOT_TELEMETRY
FAST_CODE
static void pwmDshotSetDirectionInput(
    motorDmaOutput_t * const motor
)
{
    DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;

    const timerHardware_t * const timerHardware = motor->timerHardware;
{
    //debug for BiDSHOT
    // const IO_t motorIO = IOGetByTag(timerHardware->tag);
    // motor->iocfg = IOCFG_IPU;
    // // IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);
    // IOConfigGPIO(motorIO, motor->iocfg);

    const IO_t motorIO = IOGetByTag(timerHardware->tag);
    GPIO_TypeDef* portx = IO_GPIO(motorIO);
    uint16_t  gpio_pin = IO_Pin(motorIO);
  
    portx->BSHR = gpio_pin;
    if(gpio_pin > 0x80)
    {   
       gpio_pin = gpio_pin >> 8; 
       uint32_t mulh =  gpio_pin*gpio_pin*gpio_pin*gpio_pin;
       portx->CFGHR  &= ~(0xF*mulh);
       portx->CFGHR |= (0x8*mulh);
    }
    else
    {
       uint32_t mull =  gpio_pin*gpio_pin*gpio_pin*gpio_pin;
       portx->CFGLR  &= ~(0xF*mull);
       portx->CFGLR |= (0x8*mull);
    }

}

    TIM_TypeDef *timer = (TIM_TypeDef *)timerHardware->tim;

    dmaResource_t *dmaRef = motor->dmaRef;

    // xDMA_DeInit(dmaRef);
    ((DMA_ARCH_TYPE *)(dmaRef))->CFGR &= (uint16_t)(~DMA_CFGR1_EN);

    motor->isInput = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }
    // TIM_ARRPreloadConfig(timer, DISABLE);
    timer->CTLR1 &= ~(TIM_ARPE);
    timer->ATRLR = 0xffff;
    
    TIM_ICInit(timer, &motor->icInitStruct);

    //biDshot botheageIC
    
    // TIM_SelectInputTrigger( timer, TIM_TS_TI1F_ED );	
    // TIM_IC2BothEdge_Cmd(timer, ENABLE);
	// TIM_IC3BothEdge_Cmd(timer, ENABLE);
	// TIM_IC4BothEdge_Cmd(timer, ENABLE);
    timer->SMCFGR &= (~((uint16_t)TIM_TS));
    timer->SMCFGR |= TIM_TS_TI1F_ED;
    timer->AUX |= TIM_AUX_CAPCH234_ED;

    pDmaInit->DMA_DIR = DMA_DIR_PeripheralSRC;
    xDMA_Init(dmaRef, pDmaInit);
}
#endif

void pwmCompleteDshotMotorUpdate(void)
{
    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(pwmMotorCount)) {
            return;
        }
    }


    for (int i = 0; i < dmaMotorTimerCount; i++) {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, ENABLE);
            TIM_DMAConfig(dmaMotorTimers[i].timer, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
            TIM_DMACmd(dmaMotorTimers[i].timer, TIM_DMA_Update, ENABLE);
        } else
#endif
        {
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, DISABLE);
            #ifdef USE_DSHOT_TELEMETRY
            ((TIM_TypeDef *)dmaMotorTimers[i].timer)->ATRLR = dmaMotorTimers[i].outputPeriod;
            #endif
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, ENABLE);
            TIM_SetCounter(dmaMotorTimers[i].timer, 0);
            
            TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}

FAST_CODE static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
#ifdef USE_DSHOT_TELEMETRY
        dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
#endif
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_Cmd(motor->timerHardware->dmaTimUPRef, DISABLE);
            TIM_DMACmd((TIM_TypeDef *)motor->timerHardware->tim, TIM_DMA_Update, DISABLE);
        } else
#endif
        {
            // TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, DISABLE);
            ((TIM_TypeDef *)motor->timerHardware->tim)->DMAINTENR &= ~motor->timerDmaSource;
            // xDMA_Cmd(motor->dmaRef, DISABLE);
             ((DMA_ARCH_TYPE *)(motor->dmaRef))->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
        }

#ifdef USE_DSHOT_TELEMETRY
        if (useDshotTelemetry) {
            pwmDshotSetDirectionInput(motor);

            // xDMA_SetCurrDataCounter(motor->dmaRef, GCR_TELEMETRY_INPUT_LEN);
            ((DMA_ARCH_TYPE *)(motor->dmaRef))->CNTR = GCR_TELEMETRY_INPUT_LEN;

            // xDMA_Cmd(motor->dmaRef, ENABLE);
            ((DMA_ARCH_TYPE *)(motor->dmaRef))->CFGR |= DMA_CFGR1_EN;

            // TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, ENABLE);
            ((TIM_TypeDef *)motor->timerHardware->tim)->DMAINTENR |= motor->timerDmaSource;

            dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
        }
#endif
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorProtocolTypes_e pwmProtocolType, uint8_t output)
{
#ifdef USE_DSHOT_TELEMETRY
#define OCINIT motor->ocInitStruct
#define DMAINIT motor->dmaInitStruct
#else
    TIM_OCInitTypeDef ocInitStruct;
    DMA_InitTypeDef   dmaInitStruct;
#define OCINIT ocInitStruct
#define DMAINIT dmaInitStruct
#endif

    dmaResource_t *dmaRef = NULL;
    uint32_t dmaMuxId = 0;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
        dmaMuxId = dmaSpec->dmaMuxId;
    }
#else //not defined USE_DMA_SPEC
    dmaRef = timerHardware->dmaRef; 
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
    }
#endif

    if (dmaRef == NULL) {
        return false;
    }

    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaRef);

    bool dmaIsConfigured = false;
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        const resourceOwner_t *owner = dmaGetOwner(dmaIdentifier);
        if (owner->owner == OWNER_TIMUP && owner->resourceIndex == timerGetTIMNumber(timerHardware->tim)) {
            dmaIsConfigured = true;
        } else if (!dmaAllocate(dmaIdentifier, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim))) {
            return false;
        }
    } else
#endif
    {
        if (!dmaAllocate(dmaIdentifier, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
            return false;
        }
    }

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    TIM_TypeDef *timer = timerHardware->tim;

    // Boolean configureTimer is always true when different channels of the same timer are processed in sequence,
    // causing the timer and the associated DMA initialized more than once.
    // To fix this, getTimerIndex must be expanded to return if a new timer has been requested.
    // However, since the initialization is idempotent, it is left as is in a favor of flash space (for now).
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount-1);

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;
    motor->timerHardware = timerHardware;

    const IO_t motorIO = IOGetByTag(timerHardware->tag);

    uint8_t pupMode = 0;
    pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PULL_DOWN : GPIO_PULL_UP;

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
    }
#endif

    motor->iocfg = IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, pupMode);
    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

    if (configureTimer) {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

        RCC_ClockCmd(timerRCC(timer), ENABLE);
        TIM_Cmd(timer, DISABLE);

        TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        TIM_TimeBaseStructure.TIM_Period = (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
    }

    TIM_OCStructInit(&OCINIT);
    OCINIT.TIM_OCMode = TIM_OCMode_PWM1;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        OCINIT.TIM_OutputNState = TIM_OutputNState_Enable;  //TIM_OutputNState_Enable
        OCINIT.TIM_OCNIdleState = TIM_OCNIdleState_Reset;      //TIM_OCNIdleState_Reset
        OCINIT.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        OCINIT.TIM_OutputState = TIM_OutputState_Enable; //TIM_OutputState_Enable
        OCINIT.TIM_OCIdleState = TIM_OCIdleState_Set;    //TIM_OCIdleState_Set
        OCINIT.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    OCINIT.TIM_Pulse = 0;

#ifdef USE_DSHOT_TELEMETRY
    TIM_ICStructInit(&motor->icInitStruct);
    motor->icInitStruct.TIM_ICSelection = TIM_ICSelection_TRC;
    motor->icInitStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    motor->icInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    motor->icInitStruct.TIM_Channel = timerHardware->channel;
    motor->icInitStruct.TIM_ICFilter = 2;
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
    } else
#endif
    {
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
    }

    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);

    if (!dmaIsConfigured) {
        dmaMuxEnable(dmaIdentifier, dmaMuxId);
        dmaEnable(dmaIdentifier);
    }

    DMA_StructInit(&DMAINIT);
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];

        // DMAINIT.DMA_Channel = timerHardware->dmaTimUPChannel;
        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->timer->dmaBurstBuffer;
        DMAINIT.DMA_DIR = DMA_DIR_PeripheralDST;
        // DMAINIT.DMA_FIFOMode = DMA_FIFOMode_Enable;
        // DMAINIT.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        // DMAINIT.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        // DMAINIT.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)&timerHardware->tim->DMAADR;
        DMAINIT.DMA_BufferSize = (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000) ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE; // XXX
        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMAINIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMAINIT.DMA_Mode = DMA_Mode_Normal;
        DMAINIT.DMA_Priority = DMA_Priority_High;
        DMAINIT.DMA_M2M = DMA_M2M_Disable;
    } else //unused
#endif
    {
        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];

        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)timerChCCR(timerHardware);
        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->dmaBuffer;
        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMAINIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMAINIT.DMA_Mode = DMA_Mode_Normal;
        DMAINIT.DMA_Priority = DMA_Priority_High;
        DMAINIT.DMA_M2M = DMA_M2M_Disable;
        DMAINIT.DMA_BufferSize = DSHOT_DMA_BUFFER_SIZE;
    }

    // XXX Consolidate common settings in the next refactor

    motor->dmaRef = dmaRef;

#ifdef USE_DSHOT_TELEMETRY
    motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
        (16 * MOTOR_BITLENGTH) / getDshotHz(pwmProtocolType);
    motor->timer->outputPeriod = (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
    pwmDshotSetDirectionOutput(motor);
#else
    pwmDshotSetDirectionOutput(motor, &OCINIT, &DMAINIT);
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        if (!dmaIsConfigured) {
            dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
        }
    } else
#endif
    {
        dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    }

    TIM_Cmd(timer, ENABLE);
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_CCxNCmd(timer, timerHardware->channel, TIM_CCxN_Enable);
    } else {
        TIM_CCxCmd(timer, timerHardware->channel, TIM_CCx_Enable);
    }
    if (configureTimer) {
        TIM_ARRPreloadConfig(timer, ENABLE);
        TIM_CtrlPWMOutputs(timer, ENABLE);
        TIM_Cmd(timer, ENABLE);
    }
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        // avoid high line during startup to prevent bootloader activation
        *timerChCCR(timerHardware) = 0xffff;
    }
#endif
    motor->configured = true;

    return true;
}

#endif
