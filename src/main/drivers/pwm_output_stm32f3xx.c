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

#include "io.h"
#include "timer.h"
#include "pwm_output.h"
#include "nvic.h"
#include "dma.h"
#include "system.h"
#include "rcc.h"

#ifdef USE_DSHOT

#define MAX_DMA_TIMERS 8

#define MOTOR_DSHOT600_MHZ    24
#define MOTOR_DSHOT300_MHZ    12
#define MOTOR_DSHOT150_MHZ    6

#define MOTOR_BIT_0     14
#define MOTOR_BIT_1     29
#define MOTOR_BITLENGTH 39

static uint8_t dmaMotorTimerCount = 0;
static motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
static motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];

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

void pwmWriteDigital(uint8_t index, uint16_t value)
{
    motorDmaOutput_t * const motor = &dmaMotors[index];

    uint16_t packet = (value << 1) | 0;                            // Here goes telemetry bit (false for now)
    // compute checksum
    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    csum &= 0xf;
    // append checksum
    packet = (packet << 4) | csum;
    // generate pulses for whole packet
    for (int i = 0; i < 16; i++) {
        motor->dmaBuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
        packet <<= 1;
    }

    DMA_SetCurrDataCounter(motor->timerHardware->dmaChannel, MOTOR_DMA_BUFFER_SIZE);  
    DMA_Cmd(motor->timerHardware->dmaChannel, ENABLE);
}

void pwmCompleteDigitalMotorUpdate(uint8_t motorCount)
{
    UNUSED(motorCount);
    
    for (uint8_t i = 0; i < dmaMotorTimerCount; i++) {
        TIM_SetCounter(dmaMotorTimers[i].timer, 0);
        TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE); 
    }
}

static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
        DMA_Cmd(descriptor->channel, DISABLE);
        TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

void pwmDigitalMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    motor->timerHardware = timerHardware;
        
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

	    uint32_t hz;
	    switch (pwmProtocolType) {
	        case(PWM_TYPE_DSHOT600):
		        hz = MOTOR_DSHOT600_MHZ * 1000000;
		        break;
	        case(PWM_TYPE_DSHOT300):
		        hz = MOTOR_DSHOT300_MHZ * 1000000;
		        break;
	        default:
	        case(PWM_TYPE_DSHOT150):
		        hz = MOTOR_DSHOT150_MHZ * 1000000;
	    }

        TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SystemCoreClock / timerClockDivisor(timer) / hz) - 1);
        TIM_TimeBaseStructure.TIM_Period = MOTOR_BITLENGTH;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
    }
    
    TIM_OCStructInit(&TIM_OCInitStructure);
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

    timerOCInit(timer, timerHardware->channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);
    motor->timerDmaSource = timerDmaSource(timerHardware->channel);
    dmaMotorTimers[timerIndex].timerDmaSources |= motor->timerDmaSource;

    TIM_CCxCmd(timer, timerHardware->channel, TIM_CCx_Enable);

    if (configureTimer) {
        TIM_CtrlPWMOutputs(timer, ENABLE);  
        TIM_ARRPreloadConfig(timer, ENABLE); 
        TIM_Cmd(timer, ENABLE);    
    }

    DMA_Channel_TypeDef *channel = timerHardware->dmaChannel;

    dmaSetHandler(timerHardware->dmaIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);
        
    DMA_Cmd(channel, DISABLE);
    DMA_DeInit(channel);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)timerChCCR(timerHardware);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)motor->dmaBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = MOTOR_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(channel, &DMA_InitStructure);

    DMA_ITConfig(channel, DMA_IT_TC, ENABLE);
}

#endif
