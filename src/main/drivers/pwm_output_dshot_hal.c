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
        motor->timer->dmaBurstLength = bufferSize * 4;
    } else
#endif
    {    
        bufferSize = loadDmaBuffer(motor->dmaBuffer, 1, packet);
        motor->timer->timerDmaSources |= motor->timerDmaSource;
        // @todo LL_DMA_SetDataLength
        MODIFY_REG(motor->timerHardware->dmaRef->NDTR, DMA_SxNDT, bufferSize);
        // @todo LL_DMA_EnableStream
        SET_BIT(motor->timerHardware->dmaRef->CR, DMA_SxCR_EN);
    }
}

void pwmCompleteDshotMotorUpdate(uint8_t motorCount)
{
    UNUSED(motorCount);

    for (int i = 0; i < dmaMotorTimerCount; i++) {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            // @todo LL_DMA_SetDataLength
            MODIFY_REG(dmaMotorTimers[i].dmaBurstRef->NDTR, DMA_SxNDT, dmaMotorTimers[i].dmaBurstLength);
            // @todo LL_DMA_EnableStream
            SET_BIT(dmaMotorTimers[i].dmaBurstRef->CR, DMA_SxCR_EN);

            /* configure the DMA Burst Mode */
            LL_TIM_ConfigDMABurst(dmaMotorTimers[i].timer, LL_TIM_DMABURST_BASEADDR_CCR1, LL_TIM_DMABURST_LENGTH_4TRANSFERS);
            /* Enable the TIM DMA Request */
            LL_TIM_EnableDMAReq_UPDATE(dmaMotorTimers[i].timer);
        } else
#endif
        {
            /* Reset timer counter */
            LL_TIM_SetCounter(dmaMotorTimers[i].timer, 0);
            /* Enable channel DMA requests */
            SET_BIT(dmaMotorTimers[i].timer->DIER, dmaMotorTimers[i].timerDmaSources);
            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}

static void motor_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];

#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            // @todo LL_DMA_DisableStream
            CLEAR_BIT(motor->timerHardware->dmaTimUPRef->CR, DMA_SxCR_EN);
            LL_TIM_DisableDMAReq_UPDATE(motor->timerHardware->tim);
        } else
#endif
        {
            // @todo LL_DMA_DisableStream
            CLEAR_BIT(motor->timerHardware->dmaRef->CR, DMA_SxCR_EN);
            CLEAR_BIT(motor->timerHardware->tim->DIER, motor->timerDmaSource);
        }

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

void pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    DMA_Stream_TypeDef *dmaRef;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
    } else
#endif
    {
        dmaRef = timerHardware->dmaRef;
    }

    if (dmaRef == NULL) {
        return;
    }

    LL_TIM_OC_InitTypeDef oc_init;
    LL_DMA_InitTypeDef dma_init;

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    motor->timerHardware = timerHardware;

    TIM_TypeDef *timer = timerHardware->tim;
    const IO_t motorIO = IOGetByTag(timerHardware->tag);

    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount - 1);

    // @note original DSHOT for F7 has pulldown instead of pullup
    IOConfigGPIOAF(motorIO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

    if (configureTimer) {
        LL_TIM_InitTypeDef init;
        LL_TIM_StructInit(&init);

        RCC_ClockCmd(timerRCC(timer), ENABLE);
        LL_TIM_DisableCounter(timer);

        init.Prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        init.Autoreload = pwmProtocolType == PWM_TYPE_PROSHOT1000 ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH;
        init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
        init.RepetitionCounter = 0;
        init.CounterMode = LL_TIM_COUNTERMODE_UP;
        LL_TIM_Init(timer, &init);
    }

    LL_TIM_OC_StructInit(&oc_init);
    oc_init.OCMode = LL_TIM_OCMODE_PWM1;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        oc_init.OCNState = LL_TIM_OCSTATE_ENABLE;
        oc_init.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
        oc_init.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? LL_TIM_OCPOLARITY_LOW : LL_TIM_OCPOLARITY_HIGH;
    } else {
        oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
        oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
        oc_init.OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? LL_TIM_OCPOLARITY_LOW : LL_TIM_OCPOLARITY_HIGH;
    }
    oc_init.CompareValue = 0;

    uint32_t channel;
    switch (timerHardware->channel) {
    case TIM_CHANNEL_1: channel = LL_TIM_CHANNEL_CH1; break;
    case TIM_CHANNEL_2: channel = LL_TIM_CHANNEL_CH2; break;
    case TIM_CHANNEL_3: channel = LL_TIM_CHANNEL_CH3; break;
    case TIM_CHANNEL_4: channel = LL_TIM_CHANNEL_CH4; break;
    }
    LL_TIM_OC_Init(timer, channel, &oc_init);
    LL_TIM_OC_EnablePreload(timer, channel);
    // @note original DSHOT for F7
    LL_TIM_OC_DisableFast(timer, channel);

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        // @todo quick hack to get TIM_CCER_CCxNE from TIM_CCER_CCxE
        LL_TIM_CC_EnableChannel(timer, 4 * channel);
    } else {
        LL_TIM_CC_EnableChannel(timer, channel);
    }

    if (configureTimer) {
        LL_TIM_EnableAllOutputs(timer);
        LL_TIM_EnableARRPreload(timer);
        // @note original DSHOT for F7
        LL_TIM_EnableCounter(timer);
    }

    motor->timer = &dmaMotorTimers[timerIndex];

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;

        if (!configureTimer) {
            motor->configured = true;
            return;
        }
    } else
#endif
    {
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
    }

    DMA_TypeDef *dma;
    uint32_t stream;
    if (dmaRef == DMA1_Stream0) { dma = DMA1; stream = LL_DMA_STREAM_0; }
    else if (dmaRef == DMA1_Stream1) { dma = DMA1; stream = LL_DMA_STREAM_1; }
    else if (dmaRef == DMA1_Stream2) { dma = DMA1; stream = LL_DMA_STREAM_2; }
    else if (dmaRef == DMA1_Stream3) { dma = DMA1; stream = LL_DMA_STREAM_3; }
    else if (dmaRef == DMA1_Stream4) { dma = DMA1; stream = LL_DMA_STREAM_4; }
    else if (dmaRef == DMA1_Stream5) { dma = DMA1; stream = LL_DMA_STREAM_5; }
    else if (dmaRef == DMA1_Stream6) { dma = DMA1; stream = LL_DMA_STREAM_6; }
    else if (dmaRef == DMA1_Stream7) { dma = DMA1; stream = LL_DMA_STREAM_7; }
    else if (dmaRef == DMA2_Stream0) { dma = DMA2; stream = LL_DMA_STREAM_0; }
    else if (dmaRef == DMA2_Stream1) { dma = DMA2; stream = LL_DMA_STREAM_1; }
    else if (dmaRef == DMA2_Stream2) { dma = DMA2; stream = LL_DMA_STREAM_2; }
    else if (dmaRef == DMA2_Stream3) { dma = DMA2; stream = LL_DMA_STREAM_3; }
    else if (dmaRef == DMA2_Stream4) { dma = DMA2; stream = LL_DMA_STREAM_4; }
    else if (dmaRef == DMA2_Stream5) { dma = DMA2; stream = LL_DMA_STREAM_5; }
    else if (dmaRef == DMA2_Stream6) { dma = DMA2; stream = LL_DMA_STREAM_6; }
    else if (dmaRef == DMA2_Stream7) { dma = DMA2; stream = LL_DMA_STREAM_7; }

    LL_DMA_DisableStream(dma, stream);
    //CLEAR_BIT(dmaRef->CR, DMA_SxCR_EN);
    LL_DMA_DeInit(dma, stream);
    LL_DMA_StructInit(&dma_init);

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaInit(timerHardware->dmaTimUPIrqHandler, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim));
        dmaSetHandler(timerHardware->dmaTimUPIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);

        dma_init.Channel = timerHardware->dmaTimUPChannel;
        dma_init.MemoryOrM2MDstAddress = (uint32_t)motor->timer->dmaBurstBuffer;
        dma_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
        dma_init.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
        dma_init.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_FULL;
        dma_init.MemBurst = LL_DMA_MBURST_SINGLE;
        dma_init.PeriphBurst = LL_DMA_PBURST_SINGLE;
        dma_init.PeriphOrM2MSrcAddress = (uint32_t)&timerHardware->tim->DMAR;
        dma_init.NbData = (pwmProtocolType == PWM_TYPE_PROSHOT1000) ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE; // XXX
        dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
        dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
        dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
        dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
        dma_init.Mode = LL_DMA_MODE_NORMAL;
        dma_init.Priority = LL_DMA_PRIORITY_HIGH;
    } else
#endif
    {
        dmaInit(timerHardware->dmaIrqHandler, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        dmaSetHandler(timerHardware->dmaIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);

        dma_init.Channel = timerHardware->dmaChannel;
        dma_init.MemoryOrM2MDstAddress = (uint32_t)motor->dmaBuffer;
        dma_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
        // @note original DSHOT for F7 disabled FIFO for non-burst
        dma_init.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
        dma_init.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
        dma_init.MemBurst = LL_DMA_MBURST_SINGLE;
        dma_init.PeriphBurst = LL_DMA_PBURST_SINGLE;
        dma_init.PeriphOrM2MSrcAddress = (uint32_t)timerChCCR(timerHardware);
        dma_init.NbData = pwmProtocolType == PWM_TYPE_PROSHOT1000 ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE;
        dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
        dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
        dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
        dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
        dma_init.Mode = LL_DMA_MODE_NORMAL;
        dma_init.Priority = LL_DMA_PRIORITY_HIGH;
    }

    // XXX Consolidate common settings in the next refactor
    LL_DMA_Init(dma, stream, &dma_init);
    LL_DMA_EnableIT_TC(dma, stream);

    motor->configured = true;
}
#endif
