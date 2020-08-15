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

#include "common/time.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/motor.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/system.h"

#include "pwm_output.h"

// TODO remove once debugging no longer needed
#ifdef USE_DSHOT_TELEMETRY
#include <string.h>
#endif

#include "pwm_output_dshot_shared.h"

#ifdef USE_DSHOT_TELEMETRY

void dshotEnableChannels(uint8_t motorCount)
{
    for (int i = 0; i < motorCount; i++) {
        if (dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL) {
            LL_EX_TIM_CC_EnableNChannel(dmaMotors[i].timerHardware->tim, dmaMotors[i].llChannel);
        } else {
            LL_TIM_CC_EnableChannel(dmaMotors[i].timerHardware->tim, dmaMotors[i].llChannel);
        }
    }
}

#endif

void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor
#ifndef USE_DSHOT_TELEMETRY
    , LL_TIM_OC_InitTypeDef* pOcInit, LL_DMA_InitTypeDef* pDmaInit
#endif
)
{
#ifdef USE_DSHOT_TELEMETRY
    LL_TIM_OC_InitTypeDef* pOcInit = &motor->ocInitStruct;
    LL_DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;
#endif

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = timerHardware->tim;

    xLL_EX_DMA_DeInit(motor->dmaRef);

#ifdef USE_DSHOT_TELEMETRY
    motor->isInput = false;
#endif
    LL_TIM_OC_DisablePreload(timer, motor->llChannel);
    LL_TIM_OC_Init(timer, motor->llChannel, pOcInit);
    LL_TIM_OC_EnablePreload(timer, motor->llChannel);

    motor->dmaInitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;

    xLL_EX_DMA_Init(motor->dmaRef, pDmaInit);
    xLL_EX_DMA_EnableIT_TC(motor->dmaRef);
}

#ifdef USE_DSHOT_TELEMETRY
FAST_CODE static void pwmDshotSetDirectionInput(
    motorDmaOutput_t * const motor
)
{
    LL_DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = timerHardware->tim;

    xLL_EX_DMA_DeInit(motor->dmaRef);

    motor->isInput = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }
    LL_TIM_EnableARRPreload(timer); // Only update the period once all channels are done
    timer->ARR = 0xffffffff;

#ifdef STM32H7
    IOConfigGPIO(motor->io, GPIO_MODE_OUTPUT_PP);
#endif

    LL_TIM_IC_Init(timer, motor->llChannel, &motor->icInitStruct);

#ifdef STM32H7
    IOConfigGPIOAF(motor->io, motor->iocfg, timerHardware->alternateFunction);
#endif

    motor->dmaInitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    xLL_EX_DMA_Init(motor->dmaRef, pDmaInit);
}
#endif


FAST_CODE void pwmCompleteDshotMotorUpdate(void)
{
    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if (!dshotCommandQueueEmpty() && !dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
        return;
    }

    for (int i = 0; i < dmaMotorTimerCount; i++) {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xLL_EX_DMA_SetDataLength(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            xLL_EX_DMA_EnableResource(dmaMotorTimers[i].dmaBurstRef);

            /* configure the DMA Burst Mode */
            LL_TIM_ConfigDMABurst(dmaMotorTimers[i].timer, LL_TIM_DMABURST_BASEADDR_CCR1, LL_TIM_DMABURST_LENGTH_4TRANSFERS);
            /* Enable the TIM DMA Request */
            LL_TIM_EnableDMAReq_UPDATE(dmaMotorTimers[i].timer);
        } else
#endif
        {
            LL_TIM_DisableARRPreload(dmaMotorTimers[i].timer);
            dmaMotorTimers[i].timer->ARR = dmaMotorTimers[i].outputPeriod;

            /* Reset timer counter */
            LL_TIM_SetCounter(dmaMotorTimers[i].timer, 0);
            /* Enable channel DMA requests */
            LL_EX_TIM_EnableIT(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources);
            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}

FAST_CODE static void motor_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
        if (!motor->isInput) {
#ifdef USE_DSHOT_TELEMETRY
            dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
#endif
#ifdef USE_DSHOT_DMAR
            if (useBurstDshot) {
                xLL_EX_DMA_DisableResource(motor->timerHardware->dmaTimUPRef);
                LL_TIM_DisableDMAReq_UPDATE(motor->timerHardware->tim);
            } else
#endif
            {
                xLL_EX_DMA_DisableResource(motor->dmaRef);
                LL_EX_TIM_DisableIT(motor->timerHardware->tim, motor->timerDmaSource);
            }

#ifdef USE_DSHOT_TELEMETRY
            if (useDshotTelemetry) {
                pwmDshotSetDirectionInput(motor);
                xLL_EX_DMA_SetDataLength(motor->dmaRef, GCR_TELEMETRY_INPUT_LEN);
                xLL_EX_DMA_EnableResource(motor->dmaRef);
                LL_EX_TIM_EnableIT(motor->timerHardware->tim, motor->timerDmaSource);
                dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
            }
#endif
        }
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
#ifdef USE_DSHOT_TELEMETRY
#define OCINIT motor->ocInitStruct
#define DMAINIT motor->dmaInitStruct
#else
    LL_TIM_OC_InitTypeDef ocInitStruct;
    LL_DMA_InitTypeDef   dmaInitStruct;
#define OCINIT ocInitStruct
#define DMAINIT dmaInitStruct
#endif

    dmaResource_t *dmaRef = NULL;
    uint32_t dmaChannel = 0;
#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
        dmaChannel = dmaSpec->channel;
    }
#else
    dmaRef = timerHardware->dmaRef;
    dmaChannel = timerHardware->dmaChannel;
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
        dmaChannel = timerHardware->dmaTimUPChannel;
    }
#endif

    if (dmaRef == NULL) {
        return false;
    }

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    motor->dmaRef = dmaRef;

    TIM_TypeDef *timer = timerHardware->tim;

    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount - 1);

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;

    const IO_t motorIO = IOGetByTag(timerHardware->tag);
    uint8_t pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PULLDOWN : GPIO_PULLUP;
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
#ifdef STM32H7
        if (output & TIMER_OUTPUT_INVERTED) {
            IOHi(motorIO);
        } else {
            IOLo(motorIO);
        }
#endif
    }
#endif
    motor->timerHardware = timerHardware;

    motor->iocfg = IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, pupMode);
#ifdef STM32H7
    motor->io = motorIO;
#endif
    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

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

    LL_TIM_OC_StructInit(&OCINIT);
    OCINIT.OCMode = LL_TIM_OCMODE_PWM1;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        OCINIT.OCNState = LL_TIM_OCSTATE_ENABLE;
        OCINIT.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
        OCINIT.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? LL_TIM_OCPOLARITY_LOW : LL_TIM_OCPOLARITY_HIGH;
    } else {
        OCINIT.OCState = LL_TIM_OCSTATE_ENABLE;
        OCINIT.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
        OCINIT.OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? LL_TIM_OCPOLARITY_LOW : LL_TIM_OCPOLARITY_HIGH;
    }
    OCINIT.CompareValue = 0;

#ifdef USE_DSHOT_TELEMETRY
    LL_TIM_IC_StructInit(&motor->icInitStruct);
    motor->icInitStruct.ICPolarity = LL_TIM_IC_POLARITY_BOTHEDGE;
    motor->icInitStruct.ICPrescaler = LL_TIM_ICPSC_DIV1;
    motor->icInitStruct.ICFilter = 2;
#endif

    uint32_t channel = 0;
    switch (timerHardware->channel) {
    case TIM_CHANNEL_1: channel = LL_TIM_CHANNEL_CH1; break;
    case TIM_CHANNEL_2: channel = LL_TIM_CHANNEL_CH2; break;
    case TIM_CHANNEL_3: channel = LL_TIM_CHANNEL_CH3; break;
    case TIM_CHANNEL_4: channel = LL_TIM_CHANNEL_CH4; break;
    }
    motor->llChannel = channel;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
#ifdef USE_DSHOT_TELEMETRY
        motor->dmaRef = dmaRef;
#endif
    } else
#endif
    {
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
    }

    xLL_EX_DMA_DisableResource(dmaRef);
    xLL_EX_DMA_DeInit(dmaRef);
    LL_DMA_StructInit(&DMAINIT);

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaInit(timerHardware->dmaTimUPIrqHandler, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim));

        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];

#if defined(STM32H7) || defined(STM32G4)
        DMAINIT.PeriphRequest = dmaChannel;
#else
        DMAINIT.Channel = dmaChannel;
#endif
        DMAINIT.MemoryOrM2MDstAddress = (uint32_t)motor->timer->dmaBurstBuffer;
#ifndef STM32G4
        DMAINIT.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_FULL;
#endif
        DMAINIT.PeriphOrM2MSrcAddress = (uint32_t)&timerHardware->tim->DMAR;
    } else
#endif
    {
        dmaInit(dmaGetIdentifier(dmaRef), OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));

        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];

#if defined(STM32H7) || defined(STM32G4)
        DMAINIT.PeriphRequest = dmaChannel;
#else
        DMAINIT.Channel = dmaChannel;
#endif
        DMAINIT.MemoryOrM2MDstAddress = (uint32_t)motor->dmaBuffer;
#ifndef STM32G4
        DMAINIT.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
#endif
        DMAINIT.PeriphOrM2MSrcAddress = (uint32_t)timerChCCR(timerHardware);
    }

    DMAINIT.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
#ifndef STM32G4
    DMAINIT.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
    DMAINIT.MemBurst = LL_DMA_MBURST_SINGLE;
    DMAINIT.PeriphBurst = LL_DMA_PBURST_SINGLE;
#endif
    DMAINIT.NbData = pwmProtocolType == PWM_TYPE_PROSHOT1000 ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE;
    DMAINIT.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMAINIT.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMAINIT.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
    DMAINIT.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
    DMAINIT.Mode = LL_DMA_MODE_NORMAL;
    DMAINIT.Priority = LL_DMA_PRIORITY_HIGH;

    xLL_EX_DMA_Init(dmaRef, &DMAINIT);
    xLL_EX_DMA_EnableIT_TC(dmaRef);
    motor->dmaRef = dmaRef;

#ifdef USE_DSHOT_TELEMETRY
    motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
        ( 16 * MOTOR_BITLENGTH) / getDshotHz(pwmProtocolType);
    motor->timer->outputPeriod = (pwmProtocolType == PWM_TYPE_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
    pwmDshotSetDirectionOutput(motor);
#else
    pwmDshotSetDirectionOutput(motor, &OCINIT, &DMAINIT);
#endif
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaSetHandler(timerHardware->dmaTimUPIrqHandler, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    } else
#endif
    {
        dmaSetHandler(dmaGetIdentifier(dmaRef), motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    }

    LL_TIM_OC_Init(timer, channel, &OCINIT);
    LL_TIM_OC_EnablePreload(timer, channel);
    LL_TIM_OC_DisableFast(timer, channel);

    LL_TIM_EnableCounter(timer);
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
