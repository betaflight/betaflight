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
 *
 *
 * 关于  USE_DSHOT_DMAR
 * 是否启用定时器 timer 的burst 传输，在定时器中断时，通过dma 想timer 一次性传输多个数据
 * 条件： TMRx.dmactrl  和 TMRx.dmadt 寄存器
 * 对应文件  stm32/pwm_out_dshot.c
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/debug.h"

#include "drivers/dma.h"
#include "dma_reqmap_mcu.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "drivers/system.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "drivers/pwm_output_dshot_shared.h"

#ifdef USE_DSHOT_TELEMETRY

void dshotEnableChannels(uint8_t motorCount)
{
    for (int i = 0; i < motorCount; i++) {
        if (dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL) {
            tmr_channel_enable(dmaMotors[i].timerHardware->tim, ((dmaMotors[i].timerHardware->channel-1)*2+1),TRUE);
        } else {
            tmr_channel_enable(dmaMotors[i].timerHardware->tim, ((dmaMotors[i].timerHardware->channel-1)*2),TRUE);
        }
    }
}

#endif

FAST_CODE void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor
#ifndef USE_DSHOT_TELEMETRY
    ,tmr_output_config_type *pOcInit, dma_init_type * pDmaInit
#endif
)
{
#ifdef USE_DSHOT_TELEMETRY
	tmr_output_config_type* pOcInit = &motor->ocInitStruct;
	dma_init_type* pDmaInit = &motor->dmaInitStruct;
#endif

    const timerHardware_t * const timerHardware = motor->timerHardware;
    tmr_type *timer = timerHardware->tim;

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
    timerOCPreloadConfig(timer, timerHardware->channel, FALSE);
    //DISABLE CHANNEL TO CHANGE CHANNEL FROM INPUT TO OUTPUT
    //this is a trick 
    tmr_channel_enable(timer,(timerHardware->channel-1)*2,FALSE);

  switch(timerHardware->channel){
  case 1:

    	timer->cm1=timer->cm1 & 0xff00;
    	break;
    case 2:
    	timer->cm1= timer->cm1 & 0x00ff;
    	break;
    case 3:
    	timer->cm2 =timer->cm2 & 0xff00;
    	break;
    case 4:
    	timer->cm2 =timer->cm2 & 0x00ff;
    	break;
  }

    timerOCInit(timer, timerHardware->channel, pOcInit);
    tmr_channel_enable(timer,(timerHardware->channel-1)*2,TRUE);

    timerOCPreloadConfig(timer, timerHardware->channel, TRUE);

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        pDmaInit->direction= DMA_DIR_MEMORY_TO_PERIPHERAL;
    } else
#endif
    {
        pDmaInit->direction= DMA_DIR_MEMORY_TO_PERIPHERAL;
    }

    xDMA_Init(dmaRef, pDmaInit);
    xDMA_ITConfig(dmaRef, DMA_IT_TCIF, TRUE);
}


#ifdef USE_DSHOT_TELEMETRY
FAST_CODE static void pwmDshotSetDirectionInput(
    motorDmaOutput_t * const motor
)
{
    dma_init_type * pDmaInit = &motor->dmaInitStruct;

    const timerHardware_t * const timerHardware = motor->timerHardware;
    tmr_type *timer = timerHardware->tim;

    dmaResource_t *dmaRef = motor->dmaRef;

    xDMA_DeInit(dmaRef);

    motor->isInput = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }

    tmr_period_buffer_enable(timer, TRUE);
    timer->pr = 0xffffffff;
//DISABLE CHANNEL TO CHANGE FROM INPUT TO OUTPUT
    tmr_channel_enable(timer,motor->icInitStruct.input_channel_select,FALSE);
//clear output config
    switch(motor->icInitStruct.input_channel_select){
    case TMR_SELECT_CHANNEL_1:
    case TMR_SELECT_CHANNEL_1C:
    	timer->cm1=timer->cm1 & 0xff00;//清低8位
    	break;
    case TMR_SELECT_CHANNEL_2:
    case TMR_SELECT_CHANNEL_2C:
    	timer->cm1= timer->cm1 & 0x00ff;//清高8位
    	break;
    case TMR_SELECT_CHANNEL_3:
    case TMR_SELECT_CHANNEL_3C:
    	timer->cm2 =timer->cm2 & 0xff00;
    	break;
    case TMR_SELECT_CHANNEL_4:
    	timer->cm2 =timer->cm2 & 0x00ff;
    	break;
    default:
    	break;
    }

    motor->dmaInitStruct.direction =DMA_DIR_PERIPHERAL_TO_MEMORY ;
    xDMA_Init(dmaRef, pDmaInit);
    tmr_input_channel_init(timer, &motor->icInitStruct,TMR_CHANNEL_INPUT_DIV_1);//默认会自动cxen=1

}
#endif


void pwmCompleteDshotMotorUpdate(void)
{
    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
            return;
        }
    }
    for (int i = 0; i < dmaMotorTimerCount; i++) {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            tmr_dma_control_config(dmaMotorTimers[i].timer,TMR_DMA_TRANSFER_4BYTES,TMR_C1DT_ADDRESS);
            tmr_dma_request_enable(dmaMotorTimers[i].timer, TMR_OVERFLOW_DMA_REQUEST,TRUE);
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, TRUE);
        } else
#endif
        {
        	tmr_period_buffer_enable(dmaMotorTimers[i].timer,FALSE);
            dmaMotorTimers[i].timer->pr = dmaMotorTimers[i].outputPeriod;

        	tmr_period_buffer_enable(dmaMotorTimers[i].timer,TRUE);
        	tmr_counter_value_set(dmaMotorTimers[i].timer,0);

        	tmr_dma_request_enable(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, TRUE);
            dmaMotorTimers[i].timerDmaSources = 0;//why 0 ?
        }
    }
}


FAST_CODE static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
#ifdef USE_DSHOT_TELEMETRY
        dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
#endif
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_Cmd(motor->timerHardware->dmaTimUPRef, FALSE);
            tmr_dma_request_enable(motor->timerHardware->tim, TMR_OVERFLOW_DMA_REQUEST,FALSE);

        } else
#endif
        {
            xDMA_Cmd(motor->dmaRef, FALSE);
            tmr_dma_request_enable(motor->timerHardware->tim, motor->timerDmaSource,FALSE);
        }

#ifdef USE_DSHOT_TELEMETRY
        if (useDshotTelemetry) {
            pwmDshotSetDirectionInput(motor);
            xDMA_SetCurrDataCounter(motor->dmaRef, GCR_TELEMETRY_INPUT_LEN);
            //clear IF RIF flag of timer
            tmr_dma_request_enable(motor->timerHardware->tim, motor->timerDmaSource,TRUE);
            xDMA_Cmd(motor->dmaRef, TRUE);

            dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();

        }
#endif
        
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
#ifdef USE_DSHOT_TELEMETRY
#define OCINIT motor->ocInitStruct
#define DMAINIT motor->dmaInitStruct
#else
	tmr_output_config_type ocInitStruct;
    dma_init_type   dmaInitStruct;
#define OCINIT ocInitStruct
#define DMAINIT dmaInitStruct
#endif

    dmaResource_t *dmaRef = NULL;
    //add dmamuxid for at32f43x
    uint32_t dmaMuxId =-1;
#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
        dmaMuxId = dmaSpec->dmaMuxId;
    }
#else
    dmaRef = timerHardware->dmaRef;
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
        dmaMuxId=timerHardware->dmaTimUPChannel; //time up dmamux id
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

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    tmr_type  *timer = timerHardware->tim;

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

    uint8_t pupMode = 0;
    pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PULL_DOWN : GPIO_PULL_UP;

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
    }
#endif

    motor->iocfg = IO_CONFIG(GPIO_MODE_MUX ,  GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, pupMode);
    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);


    if (configureTimer) {

    	RCC_ClockCmd(timerRCC(timer), ENABLE);
        tmr_counter_enable(timer, FALSE);

        uint32_t perscaler = (uint32_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        uint32_t period  = (pwmProtocolType == PWM_TYPE_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;

        tmr_base_init(timer,period,perscaler);
        //TMR_CLOCK_DIV1 = 0X00 NO DIV
        tmr_clock_source_div_set(timer,TMR_CLOCK_DIV1);
        //COUNT UP
        tmr_cnt_dir_set(timer,TMR_COUNT_UP);
    }

//    TIM_OCStructInit(&OCINIT);
    tmr_output_default_para_init(&OCINIT);

    OCINIT.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
    	OCINIT.occ_output_state = TRUE;
    	OCINIT.occ_idle_state = FALSE;
    	OCINIT.occ_polarity =  (output & TIMER_OUTPUT_INVERTED) ? TMR_OUTPUT_ACTIVE_LOW : TMR_OUTPUT_ACTIVE_HIGH;
    } else {
    	OCINIT.oc_output_state = TRUE;
    	OCINIT.oc_idle_state = TRUE;
    	OCINIT.oc_polarity =  (output & TIMER_OUTPUT_INVERTED) ? TMR_OUTPUT_ACTIVE_LOW : TMR_OUTPUT_ACTIVE_HIGH;
    }
    //set tim pulse =0 , need timer channel

#ifdef USE_DSHOT_TELEMETRY

	motor->icInitStruct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
	motor->icInitStruct.input_polarity_select = TMR_INPUT_BOTH_EDGE;
	motor->icInitStruct.input_channel_select = (timerHardware->channel-1)*2;//FIXME: BUGS ON N CHANNEL
	motor->icInitStruct.input_filter_value = 2; // << this is the rpmfilter fail reason

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

    xDMA_Cmd(dmaRef, FALSE);
    xDMA_DeInit(dmaRef);

    if (!dmaIsConfigured) {
        dmaEnable(dmaIdentifier);
        //ENABLE dma mux for at32f4
        dmaMuxEnable(dmaIdentifier,dmaMuxId);
    }

//    DMA_StructInit(&DMAINIT);
    dma_default_para_init(&DMAINIT);
#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];

        DMAINIT.memory_base_addr=(uint32_t)motor->timer->dmaBurstBuffer;
        DMAINIT.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        DMAINIT.buffer_size=(pwmProtocolType == PWM_TYPE_PROSHOT1000) ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE; // XXX
        DMAINIT.peripheral_base_addr=(uint32_t)&timerHardware->tim->dmadt;
        DMAINIT.peripheral_inc_enable=FALSE;
        DMAINIT.memory_inc_enable=TRUE;
        DMAINIT.peripheral_data_width =DMA_PERIPHERAL_DATA_WIDTH_WORD;
        DMAINIT.memory_data_width=DMA_MEMORY_DATA_WIDTH_WORD;
        DMAINIT.loop_mode_enable=FALSE;
        DMAINIT.priority=DMA_PRIORITY_HIGH;

    } else
#endif
    {
        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];

        DMAINIT.memory_base_addr= (uint32_t) motor->dmaBuffer;
        DMAINIT.direction= DMA_DIR_MEMORY_TO_PERIPHERAL;
        DMAINIT.peripheral_base_addr = (uint32_t)timerChCCR(timerHardware);
        DMAINIT.peripheral_inc_enable =FALSE;
        DMAINIT.memory_inc_enable = TRUE;
        DMAINIT.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
		DMAINIT.memory_data_width =DMA_MEMORY_DATA_WIDTH_WORD;
		DMAINIT.loop_mode_enable=FALSE;
        DMAINIT.priority = DMA_PRIORITY_HIGH;
    }

    // XXX Consolidate common settings in the next refactor

    motor->dmaRef = dmaRef;

#ifdef USE_DSHOT_TELEMETRY
    motor->dshotTelemetryDeadtimeUs = DSHOT_TELEMETRY_DEADTIME_US + 1000000 *
        (16 * MOTOR_BITLENGTH) / getDshotHz(pwmProtocolType);
    motor->timer->outputPeriod = (pwmProtocolType == PWM_TYPE_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
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

    tmr_counter_enable(timer,TRUE);

   if (output & TIMER_OUTPUT_N_CHANNEL) {
        tmr_channel_enable(timer, ((timerHardware->channel -1)*2+1),TRUE);
    } else {
        tmr_channel_enable(timer, ((timerHardware->channel-1)*2),TRUE);
    }

    if (configureTimer) {
    	tmr_period_buffer_enable(timer,TRUE);
    	tmr_output_enable(timer,TRUE);
    	tmr_counter_enable(timer,TRUE);
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
