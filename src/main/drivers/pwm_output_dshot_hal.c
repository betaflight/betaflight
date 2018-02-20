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
    return dmaMotorTimerCount - 1;
}

void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
    motorDmaOutput_t *const motor = &dmaMotors[index];

    if (!motor->configured) {
        return;
    }

    uint16_t packet = prepareDshotPacket(motor, value);

    uint8_t bufferSize;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        bufferSize = loadDmaBuffer(&motor->timer->dmaBurstBuffer[timerLookupChannelIndex(motor->timerHardware->channel)], 4, packet);
        if (HAL_DMA_STATE_READY == motor->TimHandle.hdma[motor->timerDmaIndex]->State) {
            HAL_DMA_Start_IT(motor->TimHandle.hdma[motor->timerDmaIndex], (uint32_t)motor->timer->dmaBurstBuffer, (uint32_t)&motor->TimHandle.Instance->DMAR, bufferSize * 4);
        }
    } else
#endif
    {    
        bufferSize = loadDmaBuffer(motor->dmaBuffer, 1, packet);

        if (DMA_SetCurrDataCounter(&motor->TimHandle, motor->timerHardware->channel, motor->dmaBuffer, bufferSize) != HAL_OK) {
            /* DMA set error */
            return;
        }
    }
}

void pwmCompleteDshotMotorUpdate(uint8_t motorCount)
{
    UNUSED(motorCount);
    for (int i = 0; i < dmaMotorTimerCount; i++) {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            /* configure the DMA Burst Mode */
            LL_TIM_ConfigDMABurst(dmaMotorTimers[i].timer, LL_TIM_DMABURST_BASEADDR_CCR1, LL_TIM_DMABURST_LENGTH_4TRANSFERS);
            /* Enable the TIM DMA Request */
            LL_TIM_EnableDMAReq_UPDATE(dmaMotorTimers[i].timer);
            /* Reset timer counter */
            LL_TIM_SetCounter(dmaMotorTimers[i].timer, 0);
            if(IS_TIM_ADVANCED_INSTANCE(dmaMotorTimers[i].timer) != RESET) {
                /* Enable the main output */
                LL_TIM_EnableAllOutputs(dmaMotorTimers[i].timer);
            }
            /* Enable the counter */
            LL_TIM_EnableCounter(dmaMotorTimers[i].timer);
        } else
#endif
        {
            /* Reset timer counter */
            LL_TIM_SetCounter(dmaMotorTimers[i].timer, 0);
            /* Enable channel DMA requests */
            dmaMotorTimers[i].timer->DIER |= dmaMotorTimers[i].timerDmaSources;
        }
    }
}

static void motor_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
    HAL_DMA_IRQHandler(motor->TimHandle.hdma[motor->timerDmaIndex]);
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        LL_TIM_DisableCounter(motor->timerHardware->tim);
        LL_TIM_DisableDMAReq_UPDATE(motor->timerHardware->tim);
    } else
#endif
    {
        __HAL_DMA_DISABLE(&motor->hdma_tim);
        TIM_DMACmd(&motor->TimHandle, motor->timerHardware->channel, DISABLE);
    }
}

void pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot && timerHardware->dmaTimUPRef == NULL) {
        return;
    } else
#endif
    if (timerHardware->dmaRef == NULL) {
        return;
    }

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    motor->timerHardware = timerHardware;

    TIM_TypeDef *timer = timerHardware->tim;
    const IO_t motorIO = IOGetByTag(timerHardware->tag);

    IOConfigGPIOAF(motorIO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

    __DMA1_CLK_ENABLE();

    RCC_ClockCmd(timerRCC(timer), ENABLE);

    motor->TimHandle.Instance = timerHardware->tim;
    motor->TimHandle.Init.Prescaler = lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1;
    motor->TimHandle.Init.Period = pwmProtocolType == PWM_TYPE_PROSHOT1000 ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH;
    motor->TimHandle.Init.RepetitionCounter = 0;
    motor->TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    motor->TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    motor->TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&motor->TimHandle) != HAL_OK) {
        /* Initialization Error */
        return;
    }

    // Note that a timer and an associated DMA are initialized more than once.
    // To fix it, getTimerIndex must be expanded to return if a new timer has been requested.
    // However, since the initialization is idempotent, it is left as is in a favor of flash space (for now).

    motor->timer = &dmaMotorTimers[getTimerIndex(timer)];

    /* Set the common dma handle parameters to be configured */
    motor->hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
    motor->hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
    motor->hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
    motor->hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    motor->hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    motor->hdma_tim.Init.Mode = DMA_NORMAL;
    motor->hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
    motor->hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    motor->hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
    motor->hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timerDmaIndex = TIM_DMA_ID_UPDATE;
        /* Set the DMAR specific dma handle parameters to be configured */
        motor->hdma_tim.Init.Channel = timerHardware->dmaTimUPChannel;
        motor->hdma_tim.Init.FIFOMode = DMA_FIFOMODE_ENABLE;

        /* Set hdma_tim instance */
        motor->hdma_tim.Instance = timerHardware->dmaTimUPRef;

        /* Link hdma_tim to hdma[x] (channelx) */
        __HAL_LINKDMA(&motor->TimHandle, hdma[motor->timerDmaIndex], motor->hdma_tim);

        dmaInit(timerHardware->dmaTimUPIrqHandler, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim));
        dmaSetHandler(timerHardware->dmaTimUPIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);
    } else
#endif
    {
        motor->timerDmaIndex = timerDmaIndex(timerHardware->channel);
        motor->timer->timerDmaSources |= timerDmaSource(timerHardware->channel);

        /* Set the non-DMAR specific dma handle parameters to be configured */
        motor->hdma_tim.Init.Channel = timerHardware->dmaChannel;
        motor->hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

        /* Set hdma_tim instance */
        motor->hdma_tim.Instance = timerHardware->dmaRef;

        /* Link hdma_tim to hdma[x] (channelx) */
        __HAL_LINKDMA(&motor->TimHandle, hdma[motor->timerDmaIndex], motor->hdma_tim);

        dmaInit(timerHardware->dmaIrqHandler, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        dmaSetHandler(timerHardware->dmaIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);
    }

    /* Initialize TIMx DMA handle */
    if (HAL_DMA_Init(motor->TimHandle.hdma[motor->timerDmaIndex]) != HAL_OK) {
        /* Initialization Error */
        return;
    }

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    TIM_OCInitStructure.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;
    TIM_OCInitStructure.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    TIM_OCInitStructure.Pulse = 0;

    if (HAL_TIM_PWM_ConfigChannel(&motor->TimHandle, &TIM_OCInitStructure, motor->timerHardware->channel) != HAL_OK) {
        /* Configuration Error */
        return;
    }

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        /* Enable the Output compare channel */
        uint32_t channels = 0;
        if(output & TIMER_OUTPUT_N_CHANNEL) {
            switch(motor->timerHardware->channel) {
            case TIM_CHANNEL_1:
                channels = LL_TIM_CHANNEL_CH1N;
                break;
            case TIM_CHANNEL_2:
                channels = LL_TIM_CHANNEL_CH2N;
                break;
            case TIM_CHANNEL_3:
                channels = LL_TIM_CHANNEL_CH3N;
                break;
            }
        } else {
            switch(motor->timerHardware->channel) {
            case TIM_CHANNEL_1:
                channels = LL_TIM_CHANNEL_CH1;
                break;
            case TIM_CHANNEL_2:
                channels = LL_TIM_CHANNEL_CH2;
                break;
            case TIM_CHANNEL_3:
                channels = LL_TIM_CHANNEL_CH3;
                break;
            case TIM_CHANNEL_4:
                channels = LL_TIM_CHANNEL_CH4;
                break;
            }
        }

        LL_TIM_CC_EnableChannel(motor->timerHardware->tim, channels);
    } else
#endif
    {
        if (output & TIMER_OUTPUT_N_CHANNEL) {
            if (HAL_TIMEx_PWMN_Start(&motor->TimHandle, motor->timerHardware->channel) != HAL_OK) {
                /* Starting PWM generation Error */
                return;
            }
        } else {
            if (HAL_TIM_PWM_Start(&motor->TimHandle, motor->timerHardware->channel) != HAL_OK) {
                /* Starting PWM generation Error */
                return;
            }
        }
    }

    motor->configured = true;
}
#endif
