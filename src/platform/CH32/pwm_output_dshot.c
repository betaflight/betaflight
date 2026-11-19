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

#include "build/atomic.h"
#include "build/debug.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "platform/timer.h"
#include "drivers/system.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot.h"
#include "dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "pwm_output_dshot_shared.h"
#include "platform/io_impl.h"

static inline bool isTimer32Bit(const TIM_TypeDef *tim)
{
    return tim == TIM9 || tim == TIM10 || tim == TIM11 || tim == TIM12;
}

#ifdef USE_DSHOT_TELEMETRY

// CH32H41x single-channel dual-edge input capture uses TIM_ICSelection_TRC:
// - CH1 on all timers routes TI1F_ED via SMCFGR.TS = TIM_TS_TI1F_ED.
// - CH2..CH4 on TIM1..TIM5 and TIM8 route their own pin's dual-edge detector via
//   TIMx_AUX.CAPCHx_ED (RM 14.3.11, 15.3.9).
// - TIM9..TIM12 lack TIMx_AUX (RM 15.3.9, 15.4.25), reserve CCxNP as RO=0 so
//   DirectTI is single-edge only (RM 15.4.9), and only expose TI1F_ED on CH1
//   in SMCFGR.TS (RM 15.4.3). Use DShot bitbang for bidirectional DShot on
//   pins mapped to TIM9..TIM12 CH2..CH4.
static inline bool pwmDshotSupportsTelemetryCapture(const TIM_TypeDef *tim, uint8_t channel)
{
    return !isTimer32Bit(tim) || channel == TIM_Channel_1;
}

static inline void pwmDshotConfigGpioPin(GPIO_TypeDef *portx, uint16_t gpioPin, uint32_t modeNibble, bool pullHighBeforeInput)
{
    if (!portx || gpioPin == 0) {
        return;
    }

    const unsigned pinIndex = (unsigned)__builtin_ctz(gpioPin);
    const unsigned shift = (pinIndex & 7U) * 4U;
    const uint32_t clearMask = 0xFU << shift;
    const uint32_t setMask = (modeNibble & 0xFU) << shift;

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        if (pullHighBeforeInput) {
            portx->BSHR = gpioPin;
        }
        if (pinIndex >= 8U) {
            MODIFY_REG(portx->CFGHR, clearMask, setMask);
        } else {
            MODIFY_REG(portx->CFGLR, clearMask, setMask);
        }
    }
}

void dshotEnableChannels(unsigned motorCount)
{
    for (unsigned i = 0; i < motorCount; i++) {
        if (dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL) {
            TIM_CCxNCmd((TIM_TypeDef *)dmaMotors[i].timerHardware->tim, dmaMotors[i].timerHardware->channel,
                        TIM_CCxN_Enable);
        } else {
            TIM_CCxCmd((TIM_TypeDef *)dmaMotors[i].timerHardware->tim, dmaMotors[i].timerHardware->channel,
                       TIM_CCx_Enable);
        }
    }
}
#endif

FAST_CODE void pwmDshotSetDirectionOutput(
    motorDmaOutput_t *const motor
#ifndef USE_DSHOT_TELEMETRY
    , TIM_OCInitTypeDef *pOcInit, DMA_InitTypeDef *pDmaInit
#endif
    )
{
#ifdef USE_DSHOT_TELEMETRY
    TIM_OCInitTypeDef *pOcInit = &motor->ocInitStruct;
    DMA_InitTypeDef *pDmaInit = &motor->dmaInitStruct;
#endif

    const timerHardware_t *const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = (TIM_TypeDef *)timerHardware->tim;

    dmaResource_t *dmaRef = motor->dmaRef;

#if defined(USE_DSHOT_DMAR) && !defined(USE_DSHOT_TELEMETRY)
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
    }
#endif

    ((DMA_ARCH_TYPE *)(dmaRef))->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
    xDMA_ITConfig(dmaRef, DMA_IT_TC, DISABLE);
    ch32DmaBarrier();

#ifdef USE_DSHOT_TELEMETRY
    motor->isInput = false;
#endif

    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Disable);
    timerOCInit(timer, timerHardware->channel, pOcInit);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);

    pDmaInit->DMA_DIR = DMA_DIR_PeripheralDST;
    xDMA_Init(dmaRef, pDmaInit);

    dmaChannelDescriptor_t *descriptor = dmaGetDescriptorByIdentifier(dmaGetIdentifier(dmaRef));
    if (descriptor) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_GLOB | DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF);
        ch32DmaBarrier();
        NVIC_ClearPendingIRQ(descriptor->irqN);
    }

    xDMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);

#ifdef USE_DSHOT_TELEMETRY
    const IO_t motorIO = IOGetByTag(timerHardware->tag);
    pwmDshotConfigGpioPin(IO_GPIO(motorIO), IO_Pin(motorIO), 0xBU, false);
#endif
}

#ifdef USE_DSHOT_TELEMETRY
FAST_CODE static void pwmDshotSetDirectionInput(motorDmaOutput_t *const motor)
{
    DMA_InitTypeDef *pDmaInit = &motor->dmaInitStruct;
    const timerHardware_t *const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = (TIM_TypeDef *)timerHardware->tim;
    if (!pwmDshotSupportsTelemetryCapture(timer, timerHardware->channel)) {
        return;
    }
    const IO_t motorIO = IOGetByTag(timerHardware->tag);

    pwmDshotConfigGpioPin(IO_GPIO(motorIO), IO_Pin(motorIO), 0x8U, true);

    dmaResource_t *dmaRef = motor->dmaRef;

    ((DMA_ARCH_TYPE *)(dmaRef))->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
    xDMA_ITConfig(dmaRef, DMA_IT_TC, DISABLE);
    ch32DmaBarrier();

    motor->isInput = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }
    timer->CTLR1 &= (uint16_t)(~TIM_ARPE);
    TIM_SetAutoreload(timer, 0xffff);

    TIM_ICInit(timer, &motor->icInitStruct);

    if (timerHardware->channel == TIM_Channel_1) {
        MODIFY_REG(timer->SMCFGR, (uint16_t)TIM_TS, TIM_TS_TI1F_ED);
    } else if (!isTimer32Bit(timer)) {
        switch (timerHardware->channel) {
        case TIM_Channel_2:
            timer->AUX |= TIM_AUX_CAPCH2_ED;
            break;
        case TIM_Channel_3:
            timer->AUX |= TIM_AUX_CAPCH3_ED;
            break;
        case TIM_Channel_4:
            timer->AUX |= TIM_AUX_CAPCH4_ED;
            break;
        default:
            break;
        }
    }

    pDmaInit->DMA_DIR = DMA_DIR_PeripheralSRC;
    xDMA_Init(dmaRef, pDmaInit);
}
#endif

void pwmCompleteDshotMotorUpdate(void)
{
    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(pwmMotorCount)) {
            return;
        }
    }

    ch32DmaBarrier();

    for (int i = 0; i < dmaMotorTimerCount; i++) {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            ch32DmaBarrier();
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, ENABLE);
            TIM_DMAConfig(dmaMotorTimers[i].timer, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
            TIM_DMACmd(dmaMotorTimers[i].timer, TIM_DMA_Update, ENABLE);
        } else
#endif
        {
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, DISABLE);
#ifdef USE_DSHOT_TELEMETRY
            TIM_SetAutoreload((TIM_TypeDef *)dmaMotorTimers[i].timer, dmaMotorTimers[i].outputPeriod);
#endif
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, ENABLE);
            TIM_SetCounter(dmaMotorTimers[i].timer, 0);
            ch32DmaBarrier();

            TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}

FAST_CODE static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t *const motor = &dmaMotors[descriptor->userParam];
#ifdef USE_DSHOT_TELEMETRY
        dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
#endif
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_Cmd(motor->timerHardware->dmaTimUPRef, DISABLE);
            TIM_DMACmd((TIM_TypeDef *)motor->timerHardware->tim, TIM_DMA_Update, DISABLE);
        } else
#endif
        {
            ((TIM_TypeDef *)motor->timerHardware->tim)->DMAINTENR &= (uint16_t)~motor->timerDmaSource;
            ((DMA_ARCH_TYPE *)(motor->dmaRef))->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
        }

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        ch32DmaBarrier();

#ifdef USE_DSHOT_TELEMETRY
        if (useDshotTelemetry && !motor->isInput
            && pwmDshotSupportsTelemetryCapture((const TIM_TypeDef *)motor->timerHardware->tim, motor->timerHardware->channel)) {
            pwmDshotSetDirectionInput(motor);

            ((DMA_ARCH_TYPE *)(motor->dmaRef))->CNTR = GCR_TELEMETRY_INPUT_LEN;
            ch32DmaBarrier();
            ((DMA_ARCH_TYPE *)(motor->dmaRef))->CFGR |= DMA_CFGR1_EN;
            ((TIM_TypeDef *)motor->timerHardware->tim)->DMAINTENR |= motor->timerDmaSource;

            dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
        }
#endif
    }
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex,
                                 motorProtocolTypes_e pwmProtocolType, uint8_t output)
{
#ifdef USE_DSHOT_TELEMETRY
#define OCINIT motor->ocInitStruct
#define DMAINIT motor->dmaInitStruct
#else
    TIM_OCInitTypeDef ocInitStruct;
    DMA_InitTypeDef dmaInitStruct;
#define OCINIT ocInitStruct
#define DMAINIT dmaInitStruct
#endif

    dmaResource_t *dmaRef = NULL;
    uint32_t dmaMuxId = 0;

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry
        && !pwmDshotSupportsTelemetryCapture((const TIM_TypeDef *)timerHardware->tim, timerHardware->channel)) {
        return false;
    }
#endif

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
        dmaMuxId = dmaSpec->dmaMuxId;
    }
#else //not defined USE_DMA_SPEC
    dmaRef = timerHardware->dmaRef;
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
    }
#endif

    if (dmaRef == NULL) {
        return false;
    }

    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaRef);

    bool dmaIsConfigured = false;
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        const resourceOwner_t *owner = dmaGetOwner(dmaIdentifier);
        if (owner->owner == OWNER_TIMUP && owner->resourceIndex == timerGetTIMNumber(timerHardware->tim)) {
            dmaIsConfigured = true;
        } else if (!dmaAllocate(dmaIdentifier, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim))) {
            return false;
        }
    } else
#endif
    {
        if (!dmaAllocate(dmaIdentifier, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
            return false;
        }
    }

    motorDmaOutput_t *const motor = &dmaMotors[motorIndex];
    TIM_TypeDef *timer = (TIM_TypeDef *)timerHardware->tim;

    // Boolean configureTimer is always true when different channels of the same timer are processed in sequence,
    // causing the timer and the associated DMA initialized more than once.
    // To fix this, getTimerIndex must be expanded to return if a new timer has been requested.
    // However, since the initialization is idempotent, it is left as is in a favor of flash space (for now).
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount - 1);

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;
    motor->timerHardware = timerHardware;

    const IO_t motorIO = IOGetByTag(timerHardware->tag);

    uint8_t pupMode = 0;
    pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PULL_DOWN : GPIO_PULL_UP;

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
    }
#endif

    motor->iocfg = IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, pupMode);
    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

    if (configureTimer) {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

        RCC_ClockCmd(timerRCC(timerHardware->tim), ENABLE);
        TIM_Cmd(timer, DISABLE);

        TIM_TimeBaseStructure.TIM_Prescaler =
            (uint16_t)(lrintf((float)timerClockFromInstance(timerHardware->tim) / getDshotHz(pwmProtocolType) + 0.01f) -
                       1);
        TIM_TimeBaseStructure.TIM_Period =
            (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
    }

    TIM_OCStructInit(&OCINIT);
    OCINIT.TIM_OCMode = TIM_OCMode_PWM1;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        OCINIT.TIM_OutputNState = TIM_OutputNState_Enable;  //TIM_OutputNState_Enable
        OCINIT.TIM_OCNIdleState = TIM_OCNIdleState_Reset;      //TIM_OCNIdleState_Reset
        OCINIT.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        OCINIT.TIM_OutputState = TIM_OutputState_Enable; //TIM_OutputState_Enable
        OCINIT.TIM_OCIdleState = TIM_OCIdleState_Set;    //TIM_OCIdleState_Set
        OCINIT.TIM_OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    OCINIT.TIM_Pulse = 0;

#ifdef USE_DSHOT_TELEMETRY
    TIM_ICStructInit(&motor->icInitStruct);
    motor->icInitStruct.TIM_ICSelection = TIM_ICSelection_TRC;
    motor->icInitStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    motor->icInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    motor->icInitStruct.TIM_Channel = timerHardware->channel;
    motor->icInitStruct.TIM_ICFilter = 2;
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
    } else
#endif
    {
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
    }

    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);

    if (!dmaIsConfigured) {
        dmaMuxEnable(dmaIdentifier, dmaMuxId);
        dmaEnable(dmaIdentifier);
    }

    DMA_StructInit(&DMAINIT);
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];

        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->timer->dmaBurstBuffer;
        DMAINIT.DMA_DIR = DMA_DIR_PeripheralDST;
        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)&timerHardware->tim->DMAADR;
        DMAINIT.DMA_BufferSize =
            (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000) ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE;
        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMAINIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMAINIT.DMA_Mode = DMA_Mode_Normal;
        DMAINIT.DMA_Priority = DMA_Priority_High;
        DMAINIT.DMA_M2M = DMA_M2M_Disable;
    } else
#endif
    {
        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];

        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)timerChCCR(timerHardware);
        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->dmaBuffer;
        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMAINIT.DMA_PeripheralDataSize = isTimer32Bit(timer) ? DMA_PeripheralDataSize_Word : DMA_PeripheralDataSize_HalfWord;
        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMAINIT.DMA_Mode = DMA_Mode_Normal;
        DMAINIT.DMA_Priority = DMA_Priority_High;
        DMAINIT.DMA_M2M = DMA_M2M_Disable;
        DMAINIT.DMA_BufferSize = DSHOT_DMA_BUFFER_SIZE;
    }

    // XXX Consolidate common settings in the next refactor

    motor->dmaRef = dmaRef;

#ifdef USE_DSHOT_TELEMETRY
    motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
                                      (16 * MOTOR_BITLENGTH) / getDshotHz(pwmProtocolType);
    motor->timer->outputPeriod =
        (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
    pwmDshotSetDirectionOutput(motor);
#else
    pwmDshotSetDirectionOutput(motor, &OCINIT, &DMAINIT);
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        if (!dmaIsConfigured) {
            dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
        }
    } else
#endif
    {
        dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
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
