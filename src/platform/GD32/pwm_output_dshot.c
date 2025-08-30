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

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/system.h"

#if defined(GD32F4)
#include "gd32f4xx.h"
#endif

#include "drivers/pwm_output.h"
#include "drivers/dshot.h"
#include "dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "pwm_output_dshot_shared.h"
#include "drivers/motor.h"

static uint16_t timer_oc_modes[MAX_SUPPORTED_MOTORS];
static uint16_t timer_oc_pulses[MAX_SUPPORTED_MOTORS];

#ifdef USE_DSHOT_TELEMETRY

void dshotEnableChannels(unsigned motorCount)
{
    for (unsigned i = 0; i < motorCount; i++) {
        if (dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL) {
            timer_channel_complementary_output_state_config((uint32_t)(dmaMotors[i].timerHardware->tim), dmaMotors[i].timerHardware->channel, TIMER_CCXN_ENABLE);
        } else {
            timer_channel_output_state_config((uint32_t)(dmaMotors[i].timerHardware->tim), dmaMotors[i].timerHardware->channel, TIMER_CCX_ENABLE);
        }
    }
}

#endif

FAST_CODE void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor
#ifndef USE_DSHOT_TELEMETRY
    ,timer_oc_parameter_struct *pOcInit,  DMA_InitTypeDef* pGenerDmaInit
#endif
)
{
#ifdef USE_DSHOT_TELEMETRY
    timer_oc_parameter_struct* pOcInit = &motor->ocInitStruct;  
    DMA_InitTypeDef* pGenerDmaInit = &motor->dmaInitStruct;
#endif

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = timerHardware->tim;

    dmaResource_t *dmaRef = motor->dmaRef;

#if defined(USE_DSHOT_DMAR) && !defined(USE_DSHOT_TELEMETRY)
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
    }
#endif

    xDMA_DeInit(dmaRef);

#ifdef USE_DSHOT_TELEMETRY
    motor->isInput = false;
#endif

    timerOCPreloadConfig(timer, timerHardware->channel, TIMER_OC_SHADOW_DISABLE);
    timerOCInit(timer, timerHardware->channel, pOcInit);

    timerOCModeConfig(timer, timerHardware->channel, timer_oc_modes[motor->index]);
    timer_channel_output_pulse_value_config((uint32_t)timer, timerHardware->channel, timer_oc_pulses[motor->index]);
    timerOCPreloadConfig(timer, timerHardware->channel, TIMER_OC_SHADOW_ENABLE);  

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        pGenerDmaInit->config.init_struct_m.direction = DMA_MEMORY_TO_PERIPH;
    } else
#endif
    {
        pGenerDmaInit->config.init_struct_s.direction = DMA_MEMORY_TO_PERIPH;

    }

    xDMA_Init(dmaRef, pGenerDmaInit);

    xDMA_ITConfig(dmaRef, DMA_INT_FTF, ENABLE);
}

#ifdef USE_DSHOT_TELEMETRY
FAST_CODE
static void pwmDshotSetDirectionInput(
    motorDmaOutput_t * const motor
)
{
    DMA_InitTypeDef* pGenerDmaInit = &motor->dmaInitStruct;

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = timerHardware->tim;

    dmaResource_t *dmaRef = motor->dmaRef;

    xDMA_DeInit(dmaRef);

    motor->isInput = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }

    timer_auto_reload_shadow_enable((uint32_t)timer);
    TIMER_CAR((uint32_t)timer) = 0xffffffff;
  
    timer_input_capture_config((uint32_t)timer, timerHardware->channel, &motor->icInitStruct);  

    if (useBurstDshot) {
        pGenerDmaInit->config.init_struct_m.direction = DMA_PERIPH_TO_MEMORY;
    } else {
        pGenerDmaInit->config.init_struct_s.direction = DMA_PERIPH_TO_MEMORY;
    }

    xDMA_Init(dmaRef, pGenerDmaInit); 
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

#ifdef GD32F4
    if (!useBurstDshot) {
        for(uint8_t i = 0; i < motorDeviceCount(); i++) {
            memmove((uint8_t *)&dshotDmaBuffer[i][2], (uint8_t *)&dshotDmaBuffer[i][1], 60);
        }
    }
#endif

    for (int i = 0; i < dmaMotorTimerCount; i++) {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, ENABLE);
            timer_dma_transfer_config((uint32_t)(dmaMotorTimers[i].timer), TIMER_DMACFG_DMATA_CH0CV, TIMER_DMACFG_DMATC_4TRANSFER);
            timer_dma_enable((uint32_t)(dmaMotorTimers[i].timer), TIMER_DMA_UPD);
        } else
#endif
        {
            timer_counter_value_config((uint32_t)(dmaMotorTimers[i].timer), 0);
            timer_dma_enable((uint32_t)(dmaMotorTimers[i].timer), dmaMotorTimers[i].timerDmaSources);
            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}

FAST_CODE static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_INT_FLAG_FTF)) { 
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
#ifdef USE_DSHOT_TELEMETRY
        dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
#endif
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_Cmd(motor->timerHardware->dmaTimUPRef, DISABLE);
            timer_dma_disable((uint32_t)(motor->timerHardware->tim), TIMER_DMA_UPD);
        } else
#endif
        {
            xDMA_Cmd(motor->dmaRef, DISABLE);
            timer_dma_disable((uint32_t)(motor->timerHardware->tim), motor->timerDmaSource);
        }

#ifdef USE_DSHOT_TELEMETRY
        if (useDshotTelemetry) {
            pwmDshotSetDirectionInput(motor);
            xDMA_SetCurrDataCounter(motor->dmaRef, GCR_TELEMETRY_INPUT_LEN);
            xDMA_Cmd(motor->dmaRef, ENABLE);
            timer_dma_enable((uint32_t)(motor->timerHardware->tim), motor->timerDmaSource);
            dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
        }
#endif
        DMA_CLEAR_FLAG(descriptor, DMA_INT_FLAG_FTF); 
        DMA_CLEAR_FLAG(descriptor, DMA_INT_FLAG_HTF);         
    }
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorProtocolTypes_e pwmProtocolType, uint8_t output)
{
#ifdef USE_DSHOT_TELEMETRY
#define OCINIT  motor->ocInitStruct
#define DMAINIT motor->dmaInitStruct
#else
    timer_oc_parameter_struct ocInitStruct; 
    DMA_InitTypeDef           dmaInitStruct;
#define OCINIT  ocInitStruct
#define DMAINIT dmaInitStruct
#endif

    dmaResource_t *dmaRef = NULL;
#if defined(GD32F4)
    uint32_t dmaChannel = 0;
#endif

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
#if defined(GD32F4)
        dmaChannel = dmaSpec->channel;
#endif
    }
#else
    dmaRef = timerHardware->dmaRef;
#if defined(GD32F4) 
    dmaChannel = timerHardware->dmaChannel;
#endif
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
#if defined(GD32F4)
        dmaChannel = timerHardware->dmaTimUPChannel;
#endif
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
        if (owner->owner == OWNER_TIMUP && owner->index == timerGetTIMNumber(timerHardware->tim)) {
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

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];

    motor->dmaRef = dmaRef;

    TIM_TypeDef *timer = timerHardware->tim;

    // Boolean configureTimer is always true when different channels of the same timer are processed in sequence,
    // causing the timer and the associated DMA initialized more than once.
    // To fix this, getTimerIndex must be expanded to return if a new timer has been requested.
    // However, since the initialization is idempotent, it is left as is in a favor of flash space (for now).
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount-1);

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;
    motor->timerHardware = timerHardware;

    const IO_t motorIO = IOGetByTag(timerHardware->tag);

    uint8_t pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PUPD_PULLDOWN : GPIO_PUPD_PULLUP;    

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
    }
#endif

    motor->iocfg = IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, pupMode);    
    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

    if (configureTimer) {
        timer_parameter_struct timer_initpara;
        timer_struct_para_init(&timer_initpara);

        RCC_ClockCmd(timerRCC(timer), ENABLE);
        timer_disable((uint32_t)timer);

        timer_initpara.period            = (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
        timer_initpara.prescaler         = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
        timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
        timer_initpara.counterdirection  = TIMER_COUNTER_UP;
        timer_initpara.repetitioncounter = 0;
        timer_init((uint32_t)timer, &timer_initpara);
    }

    timer_channel_output_struct_para_init(&OCINIT);
    timer_channel_output_mode_config((uint32_t)timer, timerHardware->channel, TIMER_OC_MODE_PWM0);
    timer_oc_modes[motor->index] = TIMER_OC_MODE_PWM0;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        OCINIT.outputnstate = TIMER_CCXN_ENABLE;
        OCINIT.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
        OCINIT.ocnpolarity  = (output & TIMER_OUTPUT_INVERTED) ? TIMER_OCN_POLARITY_LOW : TIMER_OCN_POLARITY_HIGH;
    } else {
        OCINIT.outputstate = TIMER_CCX_ENABLE;
        OCINIT.ocidlestate = TIMER_OC_IDLE_STATE_HIGH;
        OCINIT.ocpolarity  = (output & TIMER_OUTPUT_INVERTED) ? TIMER_OC_POLARITY_LOW : TIMER_OC_POLARITY_HIGH;   
    }

    timer_oc_pulses[motor->index] = 0;
    timer_channel_output_pulse_value_config((uint32_t)timer, timerHardware->channel, 0);

#ifdef USE_DSHOT_TELEMETRY

    timer_channel_input_struct_para_init(&motor->icInitStruct);

    motor->icInitStruct.icselection = TIMER_IC_SELECTION_DIRECTTI;
    motor->icInitStruct.icpolarity  = TIMER_IC_POLARITY_BOTH_EDGE;
    motor->icInitStruct.icprescaler = TIMER_IC_PSC_DIV1;
    motor->icInitStruct.icfilter    = 2;
    timer_input_capture_config((uint32_t)timer, timerHardware->channel, &motor->icInitStruct);
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
        motor->dmaInitStruct.data_mode = DMA_DATA_MODE_MULTI;
    } else
#endif
    {
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
        motor->dmaInitStruct.data_mode = DMA_DATA_MODE_SINGLE;
    }

    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);

    if (!dmaIsConfigured) {
        dmaEnable(dmaIdentifier);
    }

    if (useBurstDshot) {
        dma_multi_data_para_struct_init(&DMAINIT.config.init_struct_m);
        DMAINIT.data_mode = DMA_DATA_MODE_MULTI;
    } else {
        dma_single_data_para_struct_init(&DMAINIT.config.init_struct_s);
        DMAINIT.data_mode = DMA_DATA_MODE_SINGLE;
    }

    uint32_t temp_dma_periph;
    int temp_dma_channel;

    motor->dmaInitStruct.sub_periph = dmaChannel;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];

        gd32_dma_chbase_parse((uint32_t)dmaRef, &temp_dma_periph, &temp_dma_channel);

        DMAINIT.config.init_struct_m.memory0_addr = (uint32_t)motor->timer->dmaBurstBuffer;
        DMAINIT.config.init_struct_m.direction = DMA_MEMORY_TO_PERIPH;
        DMAINIT.config.init_struct_m.memory_burst_width = DMA_MEMORY_BURST_SINGLE;
        DMAINIT.config.init_struct_m.periph_burst_width = DMA_PERIPH_BURST_SINGLE;
        DMAINIT.config.init_struct_m.periph_addr = (uint32_t)&TIMER_DMATB((uint32_t)timerHardware->tim);
        DMAINIT.config.init_struct_m.number = (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000) ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE;
        DMAINIT.config.init_struct_m.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
        DMAINIT.config.init_struct_m.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        DMAINIT.config.init_struct_m.periph_width = DMA_PERIPH_WIDTH_32BIT;
        DMAINIT.config.init_struct_m.memory_width = DMA_MEMORY_WIDTH_32BIT;
        DMAINIT.config.init_struct_m.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
        DMAINIT.config.init_struct_m.priority = DMA_PRIORITY_HIGH;

    } else
#endif
    {
        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];

        gd32_dma_chbase_parse((uint32_t)dmaRef, &temp_dma_periph, &temp_dma_channel);

        DMAINIT.config.init_struct_s.memory0_addr = (uint32_t)motor->dmaBuffer;
        DMAINIT.config.init_struct_s.direction = DMA_MEMORY_TO_PERIPH;
        DMAINIT.config.init_struct_s.periph_addr = (uint32_t)timerChCCR(timerHardware);   
        DMAINIT.config.init_struct_s.number = DSHOT_DMA_BUFFER_SIZE;
        DMAINIT.config.init_struct_s.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
        DMAINIT.config.init_struct_s.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        DMAINIT.config.init_struct_s.periph_memory_width = DMA_PERIPH_WIDTH_32BIT;
        DMAINIT.config.init_struct_s.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
        DMAINIT.config.init_struct_s.priority = DMA_PRIORITY_HIGH;

    }

    // XXX Consolidate common settings in the next refactor

    motor->dmaRef = dmaRef;

#ifdef USE_DSHOT_TELEMETRY
    motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
        (16 * MOTOR_BITLENGTH) / getDshotHz(pwmProtocolType);
    motor->timer->outputPeriod = (pwmProtocolType == MOTOR_PROTOCOL_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
    pwmDshotSetDirectionOutput(motor);
#else
    pwmDshotSetDirectionOutput(motor, &OCINIT, &DMAINIT);
#endif

    dma_channel_subperipheral_select(temp_dma_periph, temp_dma_channel, dmaChannel);

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

    timer_enable((uint32_t)timer);

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        timer_channel_complementary_output_state_config((uint32_t)timer, timerHardware->channel, TIMER_CCXN_ENABLE);
    } else {
        timer_channel_output_state_config((uint32_t)timer, timerHardware->channel, TIMER_CCX_ENABLE);
    }

    if (configureTimer) {
        timer_auto_reload_shadow_enable((uint32_t)timer);
        timer_primary_output_config((uint32_t)timer, ENABLE);
        timer_enable((uint32_t)timer);
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
