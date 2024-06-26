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

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
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

/**
 *  Convert from BF channel to AT32 constants for timer output channels
 * 
 * The AT and ST apis take a different approach to naming channels, so just passing the bf
 * channel number to the AT calls doesn't work. This function maps between them.
 * 
 * @param bfChannel a channel number as used in timerHardware->channel (1 based)
 * @param useNChannel indicates that the desired channel should be the complementary output (only available for 1 through 3)
 * @return an AT32 tmr_channel_select_type constant
 * XXX what to return for invalid inputs? The tmr_channel_select_type enum doesn't have a suitable value
 * 
 * @see TIM_CH_TO_SELCHANNEL macro
*/
tmr_channel_select_type toCHSelectType(const uint8_t bfChannel, const bool useNChannel)
{
    tmr_channel_select_type result = TMR_SELECT_CHANNEL_1; // XXX I don't like using ch 1 as a default result, but what to do?
    if (useNChannel)
    {
        // Complementary channels only available for 1 through 3
        switch (bfChannel) {
            case 1:
              result = TMR_SELECT_CHANNEL_1C;
              break;
            case 2:
              result = TMR_SELECT_CHANNEL_2C;
              break;
            case 3:
              result = TMR_SELECT_CHANNEL_3C;
              break;
            default:
              // bad input
              #ifdef HANG_ON_ERRORS
              while(true) {}
              #endif
              break;
        }

    } else {
        switch (bfChannel) {
            case 1:
              result = TMR_SELECT_CHANNEL_1;
              break;
            case 2:
              result = TMR_SELECT_CHANNEL_2;
              break;
            case 3:
              result = TMR_SELECT_CHANNEL_3;
              break;
            case 4:
              result = TMR_SELECT_CHANNEL_4;
              break;
            default:
              // bad input
              #ifdef HANG_ON_ERRORS
              while(true) {}
              #endif
              break;
        }
    }

    return result;
}

#ifdef USE_DSHOT_TELEMETRY

/**
 * Enable the timer channels for all motors
 * 
 *   Called once for every dshot update if telemetry is being used (not just enabled by #def)
 *   Called from pwm_output_dshot_shared.c pwmTelemetryDecode
*/
void dshotEnableChannels(uint8_t motorCount)
{
    for (int i = 0; i < motorCount; i++) {
        tmr_primary_mode_select(dmaMotors[i].timerHardware->tim, TMR_PRIMARY_SEL_COMPARE);

        tmr_channel_select_type atCh = toCHSelectType(dmaMotors[i].timerHardware->channel, dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL);
        tmr_channel_enable(dmaMotors[i].timerHardware->tim, atCh, TRUE);
    }
}

#endif // USE_DSHOT_TELEMETRY

/**
 * Set the timer and dma of the specified motor for use as an output
 * 
 * Called from pwmDshotMotorHardwareConfig in this file and also from
 * pwmTelemetryDecode in src/main/drivers/pwm_output_dshot_shared.c
*/
FAST_CODE void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor
    #ifndef USE_DSHOT_TELEMETRY
    ,TIM_OCInitTypeDef *pOcInit, DMA_InitTypeDef* pDmaInit
    #endif
    )
{
    #ifdef USE_DSHOT_TELEMETRY
    TIM_OCInitTypeDef* pOcInit = &motor->ocInitStruct;
    DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;
    #endif

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = timerHardware->tim;
    const uint8_t channel = timerHardware->channel; // BF channel index (1 based)
    const bool useCompOut = (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) != 0;

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

    // Disable the preload buffer so that we can write to the comparison registers (CxDT) immediately
    // This call has the channel -> AT32 channel selector mapping built in
    timerOCPreloadConfig(timer, channel, FALSE);

    tmr_channel_enable(timer, toCHSelectType(channel, useCompOut), FALSE);

    // The at32 apis do everything except actually put the channel in output mode, so we have to do that ourselves
    // This is probably a bug in the at32 sdk, so the need for this code may go away in future releases
    const uint8_t CH_OUTPUT = 0;
    switch (channel) {
        case 1:
            timer->cm1_output_bit.c1c = CH_OUTPUT;
            break;
        case 2:
            timer->cm1_output_bit.c2c = CH_OUTPUT;
            break;
        case 3:
            timer->cm2_output_bit.c3c = CH_OUTPUT;
            break;
        case 4:
            timer->cm2_output_bit.c4c = CH_OUTPUT;
            break;
    }

    timerOCInit(timer, channel, pOcInit);

    // On ST mcu this would be part of the ocInit struct, but on AT we have to do it seperately
    { // local scope for variables
        const bool useNChannel = false; // tmr_channel_value_set only supports the normal channels
        const tmr_channel_select_type atChannel = toCHSelectType(timerHardware->channel, useNChannel);
        tmr_channel_value_set(timer, atChannel, 0);
    }

    tmr_channel_enable(timer, toCHSelectType(channel, useCompOut), TRUE);
    timerOCPreloadConfig(timer, channel, TRUE);

    pDmaInit->direction = DMA_DIR_MEMORY_TO_PERIPHERAL; 
    xDMA_Init(dmaRef, pDmaInit);
    
    // Generate an interrupt when the transfer is complete
    xDMA_ITConfig(dmaRef, DMA_FDT_INT, TRUE);

}


#ifdef USE_DSHOT_TELEMETRY
/**
 * Set the timer and dma of the specified motor for use as an input
*/
FAST_CODE
static void pwmDshotSetDirectionInput(motorDmaOutput_t * const motor)
{
    DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;
    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = timerHardware->tim;
    dmaResource_t *dmaRef = motor->dmaRef;

    xDMA_DeInit(dmaRef);

    motor->isInput = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }

    tmr_period_buffer_enable(timer, FALSE);

    timer->pr = 0xffffffff;

    tmr_input_channel_init(timer, &motor->icInitStruct, TMR_CHANNEL_INPUT_DIV_1);

    pDmaInit->direction = DMA_DIR_PERIPHERAL_TO_MEMORY;

    xDMA_Init(dmaRef, pDmaInit);
}
#endif // USE_DSHOT_TELEMETRY

/**
 * Start the timers and dma requests to send dshot data to all motors
 * 
 * Called after pwm_output_dshot_shared.c has finished setting up the buffers that represent the dshot packets.
 * Iterates over all the timers needed (note that there may be less timers than motors since a single timer can run
 * multiple motors) and enables each one.
*/
void pwmCompleteDshotMotorUpdate(void)
{
    // If there is a dshot command loaded up, time it correctly with motor update
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
            return;
        }
    }

    for (int i = 0; i < dmaMotorTimerCount; i++) { // dmaMotorTimerCount is a global declared in pwm_output_dshot_shared.c
        #ifdef USE_DSHOT_DMAR
        // NB burst mode not tested
        if (useBurstDshot) {
            xDMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, TRUE);

            // TIM_DMAConfig(dmaMotorTimers[i].timer, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
            tmr_dma_control_config(dmaMotorTimers[i].timer, TMR_DMA_TRANSFER_4BYTES, TMR_CTRL1_ADDRESS);

            // XXX TODO - what is the equivalent of TIM_DMA_Update in AT32?
            // TIM_DMACmd(dmaMotorTimers[i].timer, TIM_DMA_Update, ENABLE);
            // tmr_dma_request_enable(dmaMotorTimers[i].timer, )
        } else
        #endif // USE_DSHOT_DMAR
        {
            // I think the counter is reset here to ensure that the first pulse is the correct width,
            // and maybe also to reduce the chance of an interrupt firing prematurely
            tmr_counter_value_set(dmaMotorTimers[i].timer, 0);

            // Allows setting the period with immediate effect
            tmr_period_buffer_enable(dmaMotorTimers[i].timer, FALSE);

            #ifdef USE_DSHOT_TELEMETRY
            // NB outputPeriod isn't set when telemetry is not #def'd
            tmr_period_value_set(dmaMotorTimers[i].timer, dmaMotorTimers[i].outputPeriod);
            #endif

            tmr_period_buffer_enable(dmaMotorTimers[i].timer, TRUE);

            // Ensure that overflow events are enabled. This event may affect both period and duty cycle
            tmr_overflow_event_disable(dmaMotorTimers[i].timer, FALSE);

            // Generate requests from the timer to one or more DMA channels
            tmr_dma_request_enable(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, TRUE);

            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}

/**
 * Interrupt handler called at the end of each packet
 * 
 * Responsible for switching the dshot direction after sending a dshot command so that we 
 * can receive dshot telemetry. If telemetry is not enabled, disables the dma and request generation.
*/
FAST_CODE static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF))
    {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];

        #ifdef USE_DSHOT_TELEMETRY
        dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
        #endif

        // Disable timers and dma

        #ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_Cmd(motor->timerHardware->dmaTimUPRef, DISABLE);
            TIM_DMACmd(motor->timerHardware->tim, TIM_DMA_Update, DISABLE);
        } else
        #endif
        {   // block for the 'else' in the #ifdef above

            // Important - disable requests before disabling the dma channel, otherwise it's possible to
            // have a pending request that will fire the moment the dma channel is re-enabled, which
            // causes fake pulses to be sent at the start of the next packet.
            // This may be the problem described in the errata 1.10.1. The full work-around sounds a bit
            // heavyweight, but we should keep it in mind in case it's needed.
            //   ADVTM
            //   How to clear TMR-triggered DAM requests
            //   Description:
            //   TMR-induced DMA request cannot be cleared by resetting/setting the corresponding DMA
            //   request enable bit in the TMRx_IDEN register.
            //   Workaround:
            //   Before enabling DMA channel, reset TMR (reset CRM clock of TMR) and initialize TMR to
            //   clear pending DMA requests.

            // disable request generation
            tmr_dma_request_enable(motor->timerHardware->tim, motor->timerDmaSource, FALSE);

            // disable the dma channel, (gets re-enabled in pwm_output_dshot_shared.c from pwmWriteDshotInt)
            xDMA_Cmd(motor->dmaRef, FALSE);
        }

        // If we're expecting telem, flip direction and re-enable
        #ifdef USE_DSHOT_TELEMETRY
        if (useDshotTelemetry) {
            pwmDshotSetDirectionInput(motor);
            xDMA_SetCurrDataCounter(motor->dmaRef, GCR_TELEMETRY_INPUT_LEN);
            xDMA_Cmd(motor->dmaRef, TRUE);
            tmr_dma_request_enable(motor->timerHardware->tim, motor->timerDmaSource, TRUE);

            dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
        }
        #endif

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

    } // if DMA_IT_TCIF
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, 
                                 motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    #ifdef USE_DSHOT_TELEMETRY
    #define OCINIT motor->ocInitStruct
    #define DMAINIT motor->dmaInitStruct
    #else
    TIM_OCInitTypeDef ocInitStruct;
    DMA_InitTypeDef   dmaInitStruct;
    #define OCINIT ocInitStruct
    #define DMAINIT dmaInitStruct
    #endif

    dmaResource_t *dmaRef = NULL;
    uint32_t dmaMuxId = 0;

    #if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
        dmaMuxId = dmaSpec->dmaMuxId;
    }
    #else // not defined USE_DMA_SPEC
    dmaRef = timerHardware->dmaRef;
    #endif // USE_DMA_SPEC

    #ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
    }
    #endif // USE_DSHOT_DMAR

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
    { // 'else' block
        if (!dmaAllocate(dmaIdentifier, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex))) {
            return false;
        }
    }

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    TIM_TypeDef *timer = timerHardware->tim;

    // Boolean configureTimer is always true when different channels of the same timer are processed in sequence,
    // causing the timer and the associated DMA initialized more than once.
    // To fix this, getTimerIndex must be expanded to return if a new timer has been requested.
    // However, since the initialization is idempotent (can be applied multiple times without changing the outcome), 
    // it is left as is in a favor of flash space (for now).
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount-1);

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;
    motor->timerHardware = timerHardware;

    const IO_t motorIO = IOGetByTag(timerHardware->tag);

    uint8_t pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PULL_DOWN : GPIO_PULL_UP;

    #ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
    }
    #endif

    motor->iocfg = IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL, pupMode);

    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

    if (configureTimer) {
        RCC_ClockCmd(timerRCC(timer), ENABLE);

        tmr_counter_enable(timer, FALSE);

        uint32_t prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        uint32_t period = (pwmProtocolType == PWM_TYPE_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;

        tmr_clock_source_div_set(timer, TMR_CLOCK_DIV1);
        tmr_repetition_counter_set(timer, 0);
        tmr_cnt_dir_set(timer, TMR_COUNT_UP);
        tmr_base_init(timer, period, prescaler);
    }

    tmr_output_config_type * ocConfig = &OCINIT;
    tmr_output_default_para_init(ocConfig);

    ocConfig->oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        // XXX N channels not yet tested, comments are the stm32 code
        // OCINIT.TIM_OutputNState = TIM_OutputNState_Enable;
        ocConfig->occ_output_state = TRUE;
        // OCINIT.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        ocConfig->occ_idle_state = FALSE;
        // OCINIT.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
        ocConfig->occ_polarity = (output & TIMER_OUTPUT_INVERTED) ? TMR_OUTPUT_ACTIVE_LOW : TMR_OUTPUT_ACTIVE_HIGH;
    } else {
        ocConfig->oc_output_state = TRUE;
        ocConfig->oc_idle_state = FALSE;
        ocConfig->oc_polarity = (output & TIMER_OUTPUT_INVERTED) ? TMR_OUTPUT_ACTIVE_LOW : TMR_OUTPUT_ACTIVE_HIGH;
    }

    #ifdef USE_DSHOT_TELEMETRY
    tmr_input_config_type * icConfig = &motor->icInitStruct;
    tmr_input_default_para_init(icConfig);
    icConfig->input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    icConfig->input_polarity_select = TMR_INPUT_BOTH_EDGE;
    const bool useNChannel = output & TIMER_OUTPUT_N_CHANNEL;
    icConfig->input_channel_select = toCHSelectType(timerHardware->channel, useNChannel);
    icConfig->input_filter_value = 2;
    #endif // USE_DSHOT_TELEMETRY

    #ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
    } else
    #endif
    { // 'else' block
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);

        // clear that bit from timerDmaSources
        // timerDmaSources can have more than one source set in it if multiple motors share a common timer,
        motor->timer->timerDmaSources &= ~motor->timerDmaSource;
    }

    xDMA_Cmd(dmaRef, FALSE);
    xDMA_DeInit(dmaRef);

    if (!dmaIsConfigured) {
        dmaEnable(dmaIdentifier);
        dmaMuxEnable(dmaIdentifier, dmaMuxId);
    }

    dma_default_para_init(&DMAINIT);

    #ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];

        DMAINIT.DMA_Channel = timerHardware->dmaTimUPChannel;
        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->timer->dmaBurstBuffer;
        DMAINIT.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMAINIT.DMA_FIFOMode = DMA_FIFOMode_Enable;
        DMAINIT.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        DMAINIT.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMAINIT.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)&timerHardware->tim->DMAR;
        DMAINIT.DMA_BufferSize = (pwmProtocolType == PWM_TYPE_PROSHOT1000) ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE; // XXX
        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMAINIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMAINIT.DMA_Mode = DMA_Mode_Normal;
        DMAINIT.DMA_Priority = DMA_Priority_High;
    } else
    #endif // USE_DSHOT_DMAR
    { // 'else' block
        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];
        DMAINIT.memory_base_addr = (uint32_t)motor->dmaBuffer;
        DMAINIT.memory_inc_enable = TRUE;
        DMAINIT.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
        DMAINIT.loop_mode_enable = FALSE;
        DMAINIT.buffer_size = DSHOT_DMA_BUFFER_SIZE;
        DMAINIT.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        DMAINIT.peripheral_base_addr = (uint32_t)timerChCCR(timerHardware); // returns the address of CxDT for timerHardware->channel
        DMAINIT.peripheral_inc_enable = FALSE;
        DMAINIT.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
        DMAINIT.priority = DMA_PRIORITY_HIGH;
    }

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
    { // 'else' block
        dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    }

    { // local scope
        const tmr_channel_select_type chSel = toCHSelectType(timerHardware->channel, output & TIMER_OUTPUT_N_CHANNEL);
        tmr_channel_enable(timer, chSel, TRUE); 
    }

    if (configureTimer) {
        // Changes to the period register are deferred until the next counter overflow
        tmr_period_buffer_enable(timer, TRUE);
        tmr_output_enable(timer, TRUE);
        tmr_counter_enable(timer, TRUE);
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
