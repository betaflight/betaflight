/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "drivers/io.h"
#include "timer.h"
#if defined(STM32F4)
#include "timer_stm32f4xx.h"
#elif defined(STM32F3)
#include "timer_stm32f30x.h"
#endif
#include "pwm_output.h"
#include "drivers/nvic.h"
#include "dma.h"
#include "rcc.h"

static uint8_t dmaMotorTimerCount = 0;
static motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
static motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];

motorDmaOutput_t *getMotorDmaOutput(uint8_t index)
{
    return &dmaMotors[index];
}

uint8_t getTimerIndex(TIM_TypeDef *timer)
{
    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (dmaMotorTimers[i].timer == timer) {
            return i;
        }
    }
    dmaMotorTimers[dmaMotorTimerCount++].timer = timer;
    return dmaMotorTimerCount-1;
}

void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
    motorDmaOutput_t *const motor = &dmaMotors[index];

    if (!motor->timerHardware || !motor->timerHardware->dmaRef) {
        return;
    }

    uint16_t packet = prepareDshotPacket(motor, value);

    uint8_t bufferSize = loadDmaBuffer(motor, packet);

    motor->timer->timerDmaSources |= motor->timerDmaSource;
    DMA_SetCurrDataCounter(motor->timerHardware->dmaRef, bufferSize);
    DMA_Cmd(motor->timerHardware->dmaRef, ENABLE);
}

void pwmCompleteDshotMotorUpdate(uint8_t motorCount)
{
    UNUSED(motorCount);

    for (int i = 0; i < dmaMotorTimerCount; i++) {
        TIM_SetCounter(dmaMotorTimers[i].timer, 0);
        TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
        dmaMotorTimers[i].timerDmaSources = 0;
    }
}

static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
        DMA_Cmd(motor->timerHardware->dmaRef, DISABLE);
        TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

void pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    motor->timerHardware = timerHardware;

    TIM_TypeDef *timer = timerHardware->tim;
    const IO_t motorIO = IOGetByTag(timerHardware->tag);

    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount-1);

    IOInit(motorIO, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
    IOConfigGPIOAF(motorIO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP), timerHardware->alternateFunction);

    if (configureTimer) {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

        RCC_ClockCmd(timerRCC(timer), ENABLE);
        TIM_Cmd(timer, DISABLE);

        TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        TIM_TimeBaseStructure.TIM_Period = pwmProtocolType == PWM_TYPE_PROSHOT1000 ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
    }

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    TIM_OCInitStructure.TIM_Pulse = 0;

    timerOCInit(timer, timerHardware->channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);
    motor->timerDmaSource = timerDmaSource(timerHardware->channel);
    motor->timer = &dmaMotorTimers[timerIndex];
    motor->timer->timerDmaSources &= ~motor->timerDmaSource;

    TIM_CCxCmd(timer, timerHardware->channel, TIM_CCx_Enable);

    if (configureTimer) {
        TIM_CtrlPWMOutputs(timer, ENABLE);
        TIM_ARRPreloadConfig(timer, ENABLE);
        TIM_Cmd(timer, ENABLE);
    }

#if defined(STM32F3)
    DMA_Channel_TypeDef *dmaRef = timerHardware->dmaRef;
#elif defined(STM32F4)
    DMA_Stream_TypeDef *dmaRef = timerHardware->dmaRef;
#else
#error "No MCU specified in DSHOT"
#endif

    if (dmaRef == NULL) {
        return;
    }

    dmaInit(timerHardware->dmaIrqHandler, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
    dmaSetHandler(timerHardware->dmaIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);

    DMA_Cmd(dmaRef, DISABLE);
    DMA_DeInit(dmaRef);

    DMA_StructInit(&DMA_InitStructure);
#if defined(STM32F3)
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)motor->dmaBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
#elif defined(STM32F4)
    DMA_InitStructure.DMA_Channel = timerHardware->dmaChannel;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)motor->dmaBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
#endif
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)timerChCCR(timerHardware);
    DMA_InitStructure.DMA_BufferSize = pwmProtocolType == PWM_TYPE_PROSHOT1000 ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    DMA_Init(dmaRef, &DMA_InitStructure);
    DMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);
}

#endif
