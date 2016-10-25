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

#define MOTOR_DSHOT600_MHZ    12
#define MOTOR_DSHOT300_MHZ    6
#define MOTOR_DSHOT150_MHZ    3

#define MOTOR_BIT_0     7
#define MOTOR_BIT_1     14
#define MOTOR_BITLENGTH 19

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

    if(  HAL_TIM_PWM_Start_DMA(&motor->TimHandle, motor->timerHardware->channel, motor->dmaBuffer, MOTOR_DMA_BUFFER_SIZE) != HAL_OK)
    {
      /* Starting PWM generation Error */
        return;
    }
}

void pwmCompleteDigitalMotorUpdate(uint8_t motorCount)
{
    UNUSED(motorCount);
    
    for (uint8_t i = 0; i < dmaMotorTimerCount; i++) {
        //TIM_SetCounter(dmaMotorTimers[i].timer, 0);
        //TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
    }
}


static void motor_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
    HAL_DMA_IRQHandler(motor->TimHandle.hdma[motor->timerDmaSource]);
}

/*static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
        DMA_Cmd(descriptor->stream, DISABLE);
        TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}*/

void pwmDigitalMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType)
{
    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    motor->timerHardware = timerHardware;
        
    TIM_TypeDef *timer = timerHardware->tim;
    const IO_t motorIO = IOGetByTag(timerHardware->tag);
    
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount-1);
    
    IOInit(motorIO, OWNER_MOTOR, RESOURCE_OUTPUT, 0);
    IOConfigGPIOAF(motorIO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP), timerHardware->alternateFunction);

    __DMA1_CLK_ENABLE();

    if (configureTimer) {
        RCC_ClockCmd(timerRCC(timer), ENABLE);
        
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

        motor->TimHandle.Instance = timerHardware->tim;
        motor->TimHandle.Init.Prescaler = (SystemCoreClock / timerClockDivisor(timer) / hz) - 1;;
        motor->TimHandle.Init.Period = MOTOR_BITLENGTH;
        motor->TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        motor->TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
        if(HAL_TIM_PWM_Init(&motor->TimHandle) != HAL_OK)
        {
            /* Initialization Error */
            return;
        }

    }
    else
    {
        motor->TimHandle = dmaMotors[timerIndex].TimHandle;
    }
    
    switch (timerHardware->channel) {
        case TIM_CHANNEL_1:
            motor->timerDmaSource = TIM_DMA_ID_CC1;
            break;
        case TIM_CHANNEL_2:
            motor->timerDmaSource = TIM_DMA_ID_CC2;
            break;
        case TIM_CHANNEL_3:
            motor->timerDmaSource = TIM_DMA_ID_CC3;
            break;
        case TIM_CHANNEL_4:
            motor->timerDmaSource = TIM_DMA_ID_CC4;
            break;
    }

    dmaMotorTimers[timerIndex].timerDmaSources |= motor->timerDmaSource;
    

    static DMA_HandleTypeDef  hdma_tim;
    
    /* Set the parameters to be configured */
    hdma_tim.Init.Channel  = timerHardware->dmaChannel;
    hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
    hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
    hdma_tim.Init.Mode = DMA_NORMAL;
    hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;

    /* Set hdma_tim instance */
    if(timerHardware->dmaStream == NULL)
    {
        /* Initialization Error */
        return;
    }
    hdma_tim.Instance = timerHardware->dmaStream;

    /* Link hdma_tim to hdma[x] (channelx) */
    __HAL_LINKDMA(&motor->TimHandle, hdma[motor->timerDmaSource], hdma_tim);

    dmaSetHandler(timerHardware->dmaIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);

    /* Initialize TIMx DMA handle */
    if(HAL_DMA_Init(motor->TimHandle.hdma[motor->timerDmaSource]) != HAL_OK)
    {
        /* Initialization Error */
        return;
    }
    
    TIM_OC_InitTypeDef TIM_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCNIdleState  = TIM_OCNIDLESTATE_RESET;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    TIM_OCInitStructure.Pulse = 0;

    if(HAL_TIM_PWM_ConfigChannel(&motor->TimHandle, &TIM_OCInitStructure, motor->timerHardware->channel) != HAL_OK)
    {
        /* Configuration Error */
        return;
    }
}

#endif
