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

#include "drivers/io.h"
#include "timer.h"
#include "pwm_output.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "dma.h"
#include "rcc.h"

static FAST_RAM_ZERO_INIT uint8_t dmaMotorTimerCount = 0;
static FAST_RAM_ZERO_INIT motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
static FAST_RAM_ZERO_INIT motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];

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

FAST_CODE void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
    motorDmaOutput_t *const motor = &dmaMotors[index];

    if (!motor->configured) {
        return;
    }

    /*If there is a command ready to go overwrite the value and send that instead*/
    if (pwmDshotCommandIsProcessing()) {
        value = pwmGetDshotCommand(index);
        if (value) {
            motor->requestTelemetry = true;
        }
    }

    motor->value = value;

    uint16_t packet = prepareDshotPacket(motor);
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
        LL_EX_DMA_SetDataLength(motor->timerHardware->dmaRef, bufferSize);
        LL_EX_DMA_EnableStream(motor->timerHardware->dmaRef);
    }
}

FAST_CODE void pwmCompleteDshotMotorUpdate(uint8_t motorCount)
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
            LL_EX_DMA_SetDataLength(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            LL_EX_DMA_EnableStream(dmaMotorTimers[i].dmaBurstRef);

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
            LL_EX_TIM_EnableIT(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources);
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
            LL_EX_DMA_DisableStream(motor->timerHardware->dmaTimUPRef);
            LL_TIM_DisableDMAReq_UPDATE(motor->timerHardware->tim);
        } else
#endif
        {
            LL_EX_DMA_DisableStream(motor->timerHardware->dmaRef);
            LL_EX_TIM_DisableIT(motor->timerHardware->tim, motor->timerDmaSource);
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

    IOConfigGPIOAF(motorIO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

    if (configureTimer) {
        LL_TIM_InitTypeDef init;
        LL_TIM_StructInit(&init);

        RCC_ClockCmd(timerRCC(timer), ENABLE);
        LL_TIM_DisableCounter(timer);

        init.Prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        init.Autoreload = (pwmProtocolType == PWM_TYPE_PROSHOT1000 ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH) - 1;
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
    LL_TIM_OC_DisableFast(timer, channel);

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        LL_EX_TIM_CC_EnableNChannel(timer, channel);
    } else {
        LL_TIM_CC_EnableChannel(timer, channel);
    }

    if (configureTimer) {
        LL_TIM_EnableAllOutputs(timer);
        LL_TIM_EnableARRPreload(timer);
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

    LL_EX_DMA_DeInit(dmaRef);
    LL_DMA_StructInit(&dma_init);

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaInit(timerHardware->dmaTimUPIrqHandler, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim));
        dmaSetHandler(timerHardware->dmaTimUPIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);

        dma_init.Channel = timerHardware->dmaTimUPChannel;
        dma_init.MemoryOrM2MDstAddress = (uint32_t)motor->timer->dmaBurstBuffer;
        dma_init.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_FULL;
        dma_init.PeriphOrM2MSrcAddress = (uint32_t)&timerHardware->tim->DMAR;
    } else
#endif
    {
        dmaInit(timerHardware->dmaIrqHandler, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        dmaSetHandler(timerHardware->dmaIrqHandler, motor_DMA_IRQHandler, NVIC_BUILD_PRIORITY(1, 2), motorIndex);

        dma_init.Channel = timerHardware->dmaChannel;
        dma_init.MemoryOrM2MDstAddress = (uint32_t)motor->dmaBuffer;
        dma_init.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
        dma_init.PeriphOrM2MSrcAddress = (uint32_t)timerChCCR(timerHardware);
    }

    dma_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma_init.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
    dma_init.MemBurst = LL_DMA_MBURST_SINGLE;
    dma_init.PeriphBurst = LL_DMA_PBURST_SINGLE;
    dma_init.NbData = pwmProtocolType == PWM_TYPE_PROSHOT1000 ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE;
    dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
    dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
    dma_init.Mode = LL_DMA_MODE_NORMAL;
    dma_init.Priority = LL_DMA_PRIORITY_HIGH;

    LL_EX_DMA_Init(dmaRef, &dma_init);
    LL_EX_DMA_EnableIT_TC(dmaRef);

    motor->configured = true;
}
#endif
