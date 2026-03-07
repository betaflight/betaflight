/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
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
#include "drivers/dshot.h"
#include "dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "pwm_output_dshot_shared.h"
#include "platform/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/system.h"

#ifdef USE_DSHOT_TELEMETRY

void dshotEnableChannels(unsigned motorCount)
{
    for (unsigned i = 0; i < motorCount; i++) {
        if (dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL) {
            DDL_TMR_CC_EnableChannel(dmaMotors[i].timerHardware->tim, dmaMotors[i].llChannel << 4);
        } else {
            DDL_TMR_CC_EnableChannel(dmaMotors[i].timerHardware->tim, dmaMotors[i].llChannel);
        }
    }
}

#endif

void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor
#ifndef USE_DSHOT_TELEMETRY
    , DDL_TMR_OC_InitTypeDef* pOcInit, DDL_DMA_InitTypeDef* pDmaInit
#endif
)
{
#ifdef USE_DSHOT_TELEMETRY
    DDL_TMR_OC_InitTypeDef* pOcInit = &motor->ocInitStruct;
    DDL_DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;
#endif

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TMR_TypeDef *timer = timerHardware->tim;

//     dmaResource_t *dmaRef = motor->dmaRef;

// #if defined(USE_DSHOT_DMAR) && !defined(USE_DSHOT_TELEMETRY)
//     if (useBurstDshot) {
//         dmaRef = timerHardware->dmaTimUPRef;
//     }
// #endif

    xDDL_EX_DMA_DeInit(motor->dmaRef);

#ifdef USE_DSHOT_TELEMETRY
    motor->isInput = false;
#endif
    DDL_TMR_OC_DisablePreload(timer, motor->llChannel);
    DDL_TMR_OC_Init(timer, motor->llChannel, pOcInit);
    DDL_TMR_OC_EnablePreload(timer, motor->llChannel);

    motor->dmaInitStruct.Direction = DDL_DMA_DIRECTION_MEMORY_TO_PERIPH;

    xDDL_EX_DMA_Init(motor->dmaRef, pDmaInit);
    xDDL_EX_DMA_EnableIT_TC(motor->dmaRef);
}

#ifdef USE_DSHOT_TELEMETRY
FAST_CODE static void pwmDshotSetDirectionInput(
    motorDmaOutput_t * const motor
)
{
    DDL_DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TMR_TypeDef *timer = timerHardware->tim;

    xDDL_EX_DMA_DeInit(motor->dmaRef);

    motor->isInput = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }
    DDL_TMR_EnableARRPreload(timer); // Only update the period once all channels are done
    timer->AUTORLD = 0xffffffff;

    DDL_TMR_IC_Init(timer, motor->llChannel, &motor->icInitStruct);

    motor->dmaInitStruct.Direction = DDL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    xDDL_EX_DMA_Init(motor->dmaRef, pDmaInit);
}
#endif

FAST_CODE void pwmCompleteDshotMotorUpdate(void)
{
    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if (!dshotCommandQueueEmpty() && !dshotCommandOutputIsEnabled(dshotMotorCount)) {
        return;
    }

    for (int i = 0; i < dmaMotorTimerCount; i++) {
        do {
#ifdef USE_DSHOT_DMAR
            if (useBurstDshot) {
                xDDL_EX_DMA_SetDataLength(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
                xDDL_EX_DMA_EnableResource(dmaMotorTimers[i].dmaBurstRef);

                /* configure the DMA Burst Mode */
                DDL_TMR_ConfigDMABurst(dmaMotorTimers[i].timer, DDL_TMR_DMABURST_BASEADDR_CC1, DDL_TMR_DMABURST_LENGTH_4TRANSFERS);
                /* Enable the TIM DMA Request */
                DDL_TMR_EnableDMAReq_UPDATE(dmaMotorTimers[i].timer);
                break;
            }
#endif
            DDL_TMR_DisableARRPreload(dmaMotorTimers[i].timer);
            dmaMotorTimers[i].timer->AUTORLD = dmaMotorTimers[i].outputPeriod;

            /* Reset timer counter */
            DDL_TMR_SetCounter(dmaMotorTimers[i].timer, 0);
            /* Enable channel DMA requests */
            DDL_EX_TMR_EnableIT(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources);
            dmaMotorTimers[i].timerDmaSources = 0;
        } while (false);
    }
}

FAST_CODE static void motor_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
#ifdef USE_DSHOT_TELEMETRY
        dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
#endif
        do {
#ifdef USE_DSHOT_DMAR
            if (useBurstDshot) {
                xDDL_EX_DMA_DisableResource(motor->timerHardware->dmaTimUPRef);
                DDL_TMR_DisableDMAReq_UPDATE(motor->timerHardware->tim);
                break;
            }
#endif
            xDDL_EX_DMA_DisableResource(motor->dmaRef);
            DDL_EX_TMR_DisableIT(motor->timerHardware->tim, motor->timerDmaSource);
        } while (false);

#ifdef USE_DSHOT_TELEMETRY
        if (useDshotTelemetry) {
            pwmDshotSetDirectionInput(motor);
            xDDL_EX_DMA_SetDataLength(motor->dmaRef, GCR_TELEMETRY_INPUT_LEN);
            xDDL_EX_DMA_EnableResource(motor->dmaRef);
            DDL_EX_TMR_EnableIT(motor->timerHardware->tim, motor->timerDmaSource);
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
    DDL_TMR_OC_InitTypeDef ocInitStruct;
    DDL_DMA_InitTypeDef   dmaInitStruct;
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

    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaRef);

    bool dmaIsConfigured = false;
    do {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            const resourceOwner_t *owner = dmaGetOwner(dmaIdentifier);
            if (owner->owner == OWNER_TIMUP && owner->index == timerGetTIMNumber(timerHardware->tim)) {
                dmaIsConfigured = true;
            } else if (!dmaAllocate(dmaIdentifier, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim))) {
                return false;
            }
            break;
        }
#endif
        if (!dmaAllocate(dmaIdentifier, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
            return false;
        }
    } while (false);

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    motor->dmaRef = dmaRef;

    TMR_TypeDef *timer = timerHardware->tim;

    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount - 1);

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;

    const IO_t motorIO = IOGetByTag(timerHardware->tag);
    uint8_t pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PULLDOWN : GPIO_PULLUP;
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
    }
#endif
    motor->timerHardware = timerHardware;

    motor->iocfg = IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_LOW, pupMode);

    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

    if (configureTimer) {
        DDL_TMR_InitTypeDef init;
        DDL_TMR_StructInit(&init);

        RCC_ClockCmd(timerRCC(timer), ENABLE);
        DDL_TMR_DisableCounter(timer);

        init.Prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        init.Autoreload = (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH) - 1;
        init.ClockDivision = DDL_TMR_CLOCKDIVISION_DIV1;
        init.RepetitionCounter = 0;
        init.CounterMode = DDL_TMR_COUNTERMODE_UP;
        DDL_TMR_Init(timer, &init);
    }

    DDL_TMR_OC_StructInit(&OCINIT);
    OCINIT.OCMode = DDL_TMR_OCMODE_PWM1;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        OCINIT.OCNState = DDL_TMR_OCSTATE_ENABLE;
        OCINIT.OCNIdleState = DDL_TMR_OCIDLESTATE_LOW;
        OCINIT.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? DDL_TMR_OCPOLARITY_LOW : DDL_TMR_OCPOLARITY_HIGH;
    } else {
        OCINIT.OCState = DDL_TMR_OCSTATE_ENABLE;
        OCINIT.OCIdleState = DDL_TMR_OCIDLESTATE_HIGH;
        OCINIT.OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? DDL_TMR_OCPOLARITY_LOW : DDL_TMR_OCPOLARITY_HIGH;
    }
    OCINIT.CompareValue = 0;

#ifdef USE_DSHOT_TELEMETRY
    DDL_TMR_IC_StructInit(&motor->icInitStruct);
    motor->icInitStruct.ICPolarity = DDL_TMR_IC_POLARITY_BOTHEDGE;
    motor->icInitStruct.ICPrescaler = DDL_TMR_ICPSC_DIV1;
    motor->icInitStruct.ICFilter = 2;
#endif

    uint32_t channel = 0;
    switch (timerHardware->channel) {
    case TMR_CHANNEL_1: channel = DDL_TMR_CHANNEL_CH1; break;
    case TMR_CHANNEL_2: channel = DDL_TMR_CHANNEL_CH2; break;
    case TMR_CHANNEL_3: channel = DDL_TMR_CHANNEL_CH3; break;
    case TMR_CHANNEL_4: channel = DDL_TMR_CHANNEL_CH4; break;
    }
    motor->llChannel = channel;

    do {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            motor->timer->dmaBurstRef = dmaRef;
#ifdef USE_DSHOT_TELEMETRY
            motor->dmaRef = dmaRef;
#endif
            break;
        }
#endif
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
    } while (false);

    if (!dmaIsConfigured) {
        xDDL_EX_DMA_DisableResource(dmaRef);
        xDDL_EX_DMA_DeInit(dmaRef);

        dmaEnable(dmaIdentifier);
    }

    DDL_DMA_StructInit(&DMAINIT);
    do {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];

            DMAINIT.Channel = dmaChannel;
            DMAINIT.MemoryOrM2MDstAddress = (uint32_t)motor->timer->dmaBurstBuffer;
            DMAINIT.FIFOThreshold = DDL_DMA_FIFOTHRESHOLD_FULL;
            DMAINIT.PeriphOrM2MSrcAddress = (uint32_t)&timerHardware->tim->DMAR;
            break;
        }
#endif
        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];

        DMAINIT.Channel = dmaChannel;
        DMAINIT.MemoryOrM2MDstAddress = (uint32_t)motor->dmaBuffer;
        DMAINIT.FIFOThreshold = DDL_DMA_FIFOTHRESHOLD_1_4;
        DMAINIT.PeriphOrM2MSrcAddress = (uint32_t)timerChCCR(timerHardware);
    } while (false);

    DMAINIT.Direction = DDL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMAINIT.FIFOMode = DDL_DMA_FIFOMODE_ENABLE;
    DMAINIT.MemBurst = DDL_DMA_MBURST_SINGLE;
    DMAINIT.PeriphBurst = DDL_DMA_PBURST_SINGLE;
    DMAINIT.NbData = pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE;
    DMAINIT.PeriphOrM2MSrcIncMode = DDL_DMA_PERIPH_NOINCREMENT;
    DMAINIT.MemoryOrM2MDstIncMode = DDL_DMA_MEMORY_INCREMENT;
    DMAINIT.PeriphOrM2MSrcDataSize = DDL_DMA_PDATAALIGN_WORD;
    DMAINIT.MemoryOrM2MDstDataSize = DDL_DMA_MDATAALIGN_WORD;
    DMAINIT.Mode = DDL_DMA_MODE_NORMAL;
    DMAINIT.Priority = DDL_DMA_PRIORITY_HIGH;

    if (!dmaIsConfigured) {
        xDDL_EX_DMA_Init(dmaRef, &DMAINIT);
        xDDL_EX_DMA_EnableIT_TC(dmaRef);
    }

    motor->dmaRef = dmaRef;

#ifdef USE_DSHOT_TELEMETRY
    motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
        ( 16 * MOTOR_BITLENGTH) / getDshotHz(pwmProtocolType);
    motor->timer->outputPeriod = (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
    pwmDshotSetDirectionOutput(motor);
#else
    pwmDshotSetDirectionOutput(motor, &OCINIT, &DMAINIT);
#endif

    do {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            if (!dmaIsConfigured) {
                dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
            }
            break;
        }
#endif
        dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    } while (false);

    DDL_TMR_OC_Init(timer, channel, &OCINIT);
    DDL_TMR_OC_EnablePreload(timer, channel);
    DDL_TMR_OC_DisableFast(timer, channel);

    DDL_TMR_EnableCounter(timer);
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        DDL_EX_TMR_CC_EnableNChannel(timer, channel);
    } else {
        DDL_TMR_CC_EnableChannel(timer, channel);
    }

    if (configureTimer) {
        DDL_TMR_EnableAllOutputs(timer);
        DDL_TMR_EnableARRPreload(timer);
        DDL_TMR_EnableCounter(timer);
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
