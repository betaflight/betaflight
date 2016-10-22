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
#include <string.h>

#include "platform.h"

#include "io.h"
#include "timer.h"
#include "pwm_output.h"
#include "nvic.h"
#include "dma.h"
#include "system.h"
#include "rcc.h"

#ifdef USE_DSHOT

#define MAX_DMA_TIMERS 8

#define MOTOR_DSHOT600_MHZ    12
#define MOTOR_DSHOT150_MHZ    3

static uint8_t dmaMotorTimerCount = 0;
static motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];

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

void pwmDigitalRequestTelemetry(uint8_t motorIndex)
{
    for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motors[motorIndex].updateFlags |= (i == motorIndex) ? MOTOR_UPDATE_TELEMETRY : MOTOR_UPDATE_NONE;
    }
}

static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        pwmMotorOutput_t * const motor = &motors[descriptor->userParam];
        if (motor->updateFlags & MOTOR_UPDATE_VALUE || motor->updateFlags & MOTOR_UPDATE_TELEMETRY) {
            pwmWriteValueToDmaBuffer(motor->value, motor->dmaBuffer, motor->updateFlags);
            motor->updateFlags = 0; 
        }

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

void pwmDigitalMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    pwmMotorOutput_t * const motor = &motors[motorIndex];
    motor->timerHardware = timerHardware;
    
    memset(motor->dmaBuffer, 0, MOTOR_DMA_BUFFER_SIZE);
    
    TIM_TypeDef *timer = timerHardware->tim;
    const IO_t motorIO = IOGetByTag(timerHardware->tag);
    
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount-1);
    
    IOInit(motorIO, OWNER_MOTOR, RESOURCE_OUTPUT, 0);
    IOConfigGPIOAF(motorIO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP), timerHardware->alternateFunction);

    if (configureTimer) {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;    
    
        RCC_ClockCmd(timerRCC(timer), ENABLE);
        TIM_Cmd(timer, DISABLE);
        
        uint32_t hz = (pwmProtocolType == PWM_TYPE_DSHOT600 ? MOTOR_DSHOT600_MHZ : MOTOR_DSHOT150_MHZ) * 1000000;
        TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / timerClockDivisor(timer) / hz) - 1;
        TIM_TimeBaseStructure.TIM_Period = MOTOR_BITLENGTH;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);        
    }
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_Low;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    }
    TIM_OCInitStructure.TIM_Pulse = 0;

    uint32_t timerChannelAddress = 0;
    uint32_t dmaItFlag = 0;
    switch (timerHardware->channel) {
        case TIM_Channel_1:
            TIM_OC1Init(timer, &TIM_OCInitStructure);
            motor->timerDmaSource = TIM_DMA_CC1;
            timerChannelAddress = (uint32_t)(&timer->CCR1);
            TIM_OC1PreloadConfig(timer, TIM_OCPreload_Enable);
            dmaItFlag = DMA_IT_TCIF1;
            break;
        case TIM_Channel_2:
            TIM_OC2Init(timer, &TIM_OCInitStructure);
            motor->timerDmaSource = TIM_DMA_CC2;
            timerChannelAddress = (uint32_t)(&timer->CCR2);
            TIM_OC2PreloadConfig(timer, TIM_OCPreload_Enable);
            dmaItFlag = DMA_IT_TCIF2;
            break;
        case TIM_Channel_3:
            TIM_OC3Init(timer, &TIM_OCInitStructure);
            motor->timerDmaSource = TIM_DMA_CC3;
            timerChannelAddress = (uint32_t)(&timer->CCR3);
            TIM_OC3PreloadConfig(timer, TIM_OCPreload_Enable);
            dmaItFlag = DMA_IT_TCIF3;
            break;
        case TIM_Channel_4:
            TIM_OC4Init(timer, &TIM_OCInitStructure);
            motor->timerDmaSource = TIM_DMA_CC4;
            timerChannelAddress = (uint32_t)(&timer->CCR4);
            TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
            dmaItFlag = DMA_IT_TCIF4;
            break;
    }
    dmaMotorTimers[timerIndex].timerDmaSources |= motor->timerDmaSource;
    
    TIM_CCxCmd(timer, motor->timerHardware->channel, TIM_CCx_Enable);
    
    if (configureTimer) {
        TIM_CtrlPWMOutputs(timer, ENABLE);
        TIM_ARRPreloadConfig(timer, ENABLE); 
        TIM_Cmd(timer, ENABLE);       
    }

    DMA_Stream_TypeDef *stream = timerHardware->dmaStream;
    
    DMA_Cmd(stream, DISABLE);
    DMA_DeInit(stream);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = timerHardware->dmaChannel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = timerChannelAddress;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)motor->dmaBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = MOTOR_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(stream, &DMA_InitStructure);

    DMA_ITConfig(stream, DMA_IT_TC, ENABLE);
    DMA_ClearITPendingBit(stream, dmaItFlag);
    
    dmaSetHandler(timerHardware->dmaIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);
}

void pwmStartDigitalOutput(void)
{
    for (uint8_t i; i < MAX_SUPPORTED_MOTORS; i++) {
        if (motors[i].enabled && motors[i].timerHardware) {
            //DMA_SetCurrDataCounter(motor->timerHardware->dmaStream, MOTOR_DMA_BUFFER_SIZE);  
            DMA_Cmd(motors[i].timerHardware->dmaStream, ENABLE);
        }
    }
    
    for (uint8_t i = 0; i < dmaMotorTimerCount; i++) {
        TIM_SetCounter(dmaMotorTimers[i].timer, 0);
        TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE); 
    }
}

void pwmStopDigitalOutput(void)
{
    for (uint8_t i; i < MAX_SUPPORTED_MOTORS; i++) {
        if (motors[i].enabled && motors[i].timerHardware) {
            DMA_Cmd(motors[i].timerHardware->dmaStream, DISABLE);
            TIM_DMACmd(motors[i].timerHardware->tim, motors[i].timerDmaSource, DISABLE);
        }
    }
}

#endif
