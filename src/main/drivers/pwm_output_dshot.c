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
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#if defined(STM32F4)
#include "stm32f4xx.h"
#elif defined(STM32F3)
#include "stm32f30x.h"
#endif

#include "pwm_output.h"

#include "pwm_output_dshot_shared.h"

#ifdef USE_DSHOT_TELEMETRY

static void processInputIrq(motorDmaOutput_t * const motor)
{
    motor->hasTelemetry = true;
    DMA_Cmd(motor->dmaRef, DISABLE);
    TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, DISABLE);
    readDoneCount++;
}

void dshotEnableChannels(uint8_t motorCount)
{
    for (int i = 0; i < motorCount; i++) {
        if (dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL) {
            TIM_CCxNCmd(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerHardware->channel, TIM_CCxN_Enable);
        } else {
            TIM_CCxCmd(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerHardware->channel, TIM_CCx_Enable);
        }
    }
}

#endif

static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor);

FAST_CODE void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor, bool output
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
    TIM_TypeDef *timer = timerHardware->tim;

    dmaStream_t *dmaRef = motor->dmaRef;

#if defined(USE_DSHOT_DMAR) && !defined(USE_DSHOT_TELEMETRY)
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
    }
#endif

    DMA_DeInit(dmaRef);

#ifdef USE_DSHOT_TELEMETRY
    if (!output) {
        motor->isInput = true;
        motor->timer->inputDirectionStampUs = micros();
        TIM_ICInit(timer, &motor->icInitStruct);

#if defined(STM32F3)
        motor->dmaInitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
        motor->dmaInitStruct.DMA_M2M = DMA_M2M_Disable;
#elif defined(STM32F4)
        motor->dmaInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
#endif
    } else
#else
    UNUSED(output);
#endif
    {
#ifdef USE_DSHOT_TELEMETRY
        motor->isInput = false;
#endif
        timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Disable);
        timerOCInit(timer, timerHardware->channel, pOcInit);
        timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);

#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
#if defined(STM32F3)
            pDmaInit->DMA_DIR = DMA_DIR_PeripheralDST;
#else
            pDmaInit->DMA_DIR = DMA_DIR_MemoryToPeripheral;
#endif
        } else
#endif
        {
#if defined(STM32F3)
            pDmaInit->DMA_DIR = DMA_DIR_PeripheralDST;
            pDmaInit->DMA_M2M = DMA_M2M_Disable;
#elif defined(STM32F4)
            pDmaInit->DMA_DIR = DMA_DIR_MemoryToPeripheral;
#endif
        }
    }

    DMA_Init(dmaRef, pDmaInit);
    DMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);
}


void pwmCompleteDshotMotorUpdate(uint8_t motorCount)
{
    UNUSED(motorCount);

    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if (pwmDshotCommandIsQueued()) {
        if (!pwmDshotCommandOutputIsEnabled(motorCount)) {
            return;
        }
    }

    for (int i = 0; i < dmaMotorTimerCount; i++) {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            DMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            DMA_Cmd(dmaMotorTimers[i].dmaBurstRef, ENABLE);
            TIM_DMAConfig(dmaMotorTimers[i].timer, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
            TIM_DMACmd(dmaMotorTimers[i].timer, TIM_DMA_Update, ENABLE);
        } else
#endif
        {
            TIM_SetCounter(dmaMotorTimers[i].timer, 0);
            TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}

static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
#ifdef USE_DSHOT_TELEMETRY
        uint32_t irqStart = micros();
        if (motor->isInput) {
            processInputIrq(motor);
        } else
#endif
        {
#ifdef USE_DSHOT_DMAR
            if (useBurstDshot) {
                DMA_Cmd(motor->timerHardware->dmaTimUPRef, DISABLE);
                TIM_DMACmd(motor->timerHardware->tim, TIM_DMA_Update, DISABLE);
            } else
#endif
            {
                DMA_Cmd(motor->dmaRef, DISABLE);
                TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, DISABLE);
            }

#ifdef USE_DSHOT_TELEMETRY
            if (useDshotTelemetry) {
                pwmDshotSetDirectionOutput(motor, false);
                DMA_SetCurrDataCounter(motor->dmaRef, motor->dmaInputLen);
                DMA_Cmd(motor->dmaRef, ENABLE);
                TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, ENABLE);
                setDirectionMicros = micros() - irqStart;
            }
#endif
        }
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

void pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
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

    dmaStream_t *dmaRef = NULL;
#if defined(STM32F4)
    uint32_t dmaChannel = 0;
#endif
#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
#if defined(STM32F4)
        dmaChannel = dmaSpec->channel;
#endif
    }
#else
    dmaRef = timerHardware->dmaRef;
#if defined(STM32F4)
    dmaChannel = timerHardware->dmaChannel;
#endif
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
#if defined(STM32F4)
        dmaChannel = timerHardware->dmaTimUPChannel;
#endif
    }
#endif

    if (dmaRef == NULL) {
        return;
    }

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
#ifdef USE_DSHOT_TELEMETRY
    motor->useProshot = (pwmProtocolType == PWM_TYPE_PROSHOT1000);
#endif    
    motor->timerHardware = timerHardware;

    TIM_TypeDef *timer = timerHardware->tim;
    const IO_t motorIO = IOGetByTag(timerHardware->tag);

    // Boolean configureTimer is always true when different channels of the same timer are processed in sequence,
    // causing the timer and the associated DMA initialized more than once.
    // To fix this, getTimerIndex must be expanded to return if a new timer has been requested.
    // However, since the initialization is idempotent, it is left as is in a favor of flash space (for now).
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount-1);

    uint8_t pupMode = 0;
#ifdef USE_DSHOT_TELEMETRY
    if (!useDshotTelemetry) {
        pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PuPd_DOWN : GPIO_PuPd_UP;
    } else
#endif
    {
        pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PuPd_UP : GPIO_PuPd_DOWN;
    }

    IOConfigGPIOAF(motorIO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, pupMode), timerHardware->alternateFunction);

    if (configureTimer) {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

        RCC_ClockCmd(timerRCC(timer), ENABLE);
        TIM_Cmd(timer, DISABLE);

        TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        TIM_TimeBaseStructure.TIM_Period = (pwmProtocolType == PWM_TYPE_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
    }

    TIM_OCStructInit(&OCINIT);
    OCINIT.TIM_OCMode = TIM_OCMode_PWM1;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        OCINIT.TIM_OutputNState = TIM_OutputNState_Enable;
        OCINIT.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        OCINIT.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        OCINIT.TIM_OutputState = TIM_OutputState_Enable;
        OCINIT.TIM_OCIdleState = TIM_OCIdleState_Set;
        OCINIT.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    OCINIT.TIM_Pulse = 0;

#ifdef USE_DSHOT_TELEMETRY
    TIM_ICStructInit(&motor->icInitStruct);
    motor->icInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    motor->icInitStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    motor->icInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    motor->icInitStruct.TIM_Channel = timerHardware->channel;
    motor->icInitStruct.TIM_ICFilter = 0; //2;
#endif

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
    } else
#endif
    {
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
    }

    DMA_Cmd(dmaRef, DISABLE);
    DMA_DeInit(dmaRef);
    DMA_StructInit(&DMAINIT);

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaInit(timerHardware->dmaTimUPIrqHandler, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim));

#if defined(STM32F3)
        DMAINIT.DMA_MemoryBaseAddr = (uint32_t)motor->timer->dmaBurstBuffer;
        DMAINIT.DMA_DIR = DMA_DIR_PeripheralDST;
#else
        DMAINIT.DMA_Channel = timerHardware->dmaTimUPChannel;
        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->timer->dmaBurstBuffer;
        DMAINIT.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMAINIT.DMA_FIFOMode = DMA_FIFOMode_Enable;
        DMAINIT.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        DMAINIT.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMAINIT.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
#endif
        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)&timerHardware->tim->DMAR;
        DMAINIT.DMA_BufferSize = (pwmProtocolType == PWM_TYPE_PROSHOT1000) ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE; // XXX
        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMAINIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMAINIT.DMA_Mode = DMA_Mode_Normal;
        DMAINIT.DMA_Priority = DMA_Priority_High;
    } else
#endif
    {
        dmaInit(dmaGetIdentifier(dmaRef), OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

#if defined(STM32F3)
        DMAINIT.DMA_MemoryBaseAddr = (uint32_t)motor->dmaBuffer;
        DMAINIT.DMA_DIR = DMA_DIR_PeripheralDST;
        DMAINIT.DMA_M2M = DMA_M2M_Disable;
#elif defined(STM32F4)
        DMAINIT.DMA_Channel = dmaChannel;
        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->dmaBuffer;
        DMAINIT.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMAINIT.DMA_FIFOMode = DMA_FIFOMode_Enable;
        DMAINIT.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMAINIT.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMAINIT.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
#endif
        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)timerChCCR(timerHardware);
        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMAINIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMAINIT.DMA_Mode = DMA_Mode_Normal;
        DMAINIT.DMA_Priority = DMA_Priority_High;
    }

    // XXX Consolidate common settings in the next refactor

    motor->dmaRef = dmaRef;

#ifdef USE_DSHOT_TELEMETRY
    motor->dmaInputLen = motor->useProshot ? PROSHOT_TELEMETRY_INPUT_LEN : DSHOT_TELEMETRY_INPUT_LEN;
    motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
        ( 2 + (motor->useProshot ? 4 * MOTOR_NIBBLE_LENGTH_PROSHOT : 16 * MOTOR_BITLENGTH))
        / getDshotHz(pwmProtocolType);
    pwmDshotSetDirectionOutput(motor, true);
#else
    pwmDshotSetDirectionOutput(motor, true, &OCINIT, &DMAINIT);
#endif
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaSetHandler(timerHardware->dmaTimUPIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(2, 1), motor->index);
    } else
#endif
    {
        dmaSetHandler(dmaGetIdentifier(dmaRef), motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(2, 1), motor->index);
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
    motor->configured = true;
}

#endif
