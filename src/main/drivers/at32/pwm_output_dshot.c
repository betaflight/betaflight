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
tmr_channel_select_type toCHSelectType(uint8_t bfChannel, bool useNChannel)
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
        }
    }

    return result;
}

#ifdef USE_DSHOT_TELEMETRY

void dshotEnableChannels(uint8_t motorCount)
{
    for (int i = 0; i < motorCount; i++) {
        tmr_channel_select_type atCh = toCHSelectType(dmaMotors[i].timerHardware->channel, dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL);

        // XXX TODO need to confirm that these AT api calls are equivalent to the commented out ST calls
        tmr_primary_mode_select(dmaMotors[i].timerHardware->tim, TMR_PRIMARY_SEL_COMPARE);

        // TODO these polarity set calls might not be needed if they are already the default settings
        if (dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL) {
            // TIM_CCxNCmd(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerHardware->channel, TIM_CCxN_Enable);
            tmr_output_channel_polarity_set(dmaMotors[i].timerHardware->tim, atCh, TMR_POLARITY_ACTIVE_LOW);
        } else {
            // TIM_CCxCmd(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerHardware->channel, TIM_CCx_Enable);
            tmr_output_channel_polarity_set(dmaMotors[i].timerHardware->tim, atCh, TMR_POLARITY_ACTIVE_HIGH);
        }

        tmr_channel_enable(dmaMotors[i].timerHardware->tim, atCh, TRUE);
    }
}

#endif // USE_DSHOT_TELEMETRY


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
    const uint8_t channel = timerHardware->channel;
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
    // which seems a bit odd as we don't write to CxDT in this section
    // This call has the channel -> AT32 channel selector mapping built in
    timerOCPreloadConfig(timer, channel, FALSE);
    tmr_channel_enable(timer, toCHSelectType(channel, useCompOut), FALSE);

    timerOCInit(timer, timerHardware->channel, pOcInit);

    tmr_channel_enable(timer, toCHSelectType(timerHardware->channel, useCompOut), TRUE);
    timerOCPreloadConfig(timer, timerHardware->channel, TRUE);

    pDmaInit->direction = DMA_DIR_MEMORY_TO_PERIPHERAL; 
    xDMA_Init(dmaRef, pDmaInit);
    
    // Generate an interrupt when the transfer is complete
    xDMA_ITConfig(dmaRef, DMA_FDT_INT, TRUE);

}


#ifdef USE_DSHOT_TELEMETRY
FAST_CODE
static void pwmDshotSetDirectionInput(
    motorDmaOutput_t * const motor
)
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
    // TIM_ARRPreloadConfig(timer, ENABLE); // Need to set the ovfif bit to enable preload
    tmr_overflow_event_disable(timer, TRUE); // method is badly named. I think TRUE enables overflow events based on the code
    // timer->ARR = 0xffffffff;    // arr = auto reload register, may be equivelent to the preload register on at32
    timer->pr = 0xffffffff;

    // TIM_ICInit(timer, &motor->icInitStruct);

    // XXX which input divider setting to use? Looks like it's the equivalent of ST prescaler, and the setting
    // would have been in the init struct for ST, but isn't in the AT equivalent. We'll need another way of getting it
    // Looks like it may have been hardcoded to div_1 anyway
    tmr_input_channel_init(timer, &motor->icInitStruct, TMR_CHANNEL_INPUT_DIV_1);

#if defined(STM32F4)
    motor->dmaInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
#endif

    xDMA_Init(dmaRef, pDmaInit);
}
#endif


void pwmCompleteDshotMotorUpdate(void)
{
    // If there is a dshot command loaded up, time it correctly with motor update
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
            return;
        }
    }

    for(int i=0; i<3; i++)
    {
        motorDmaOutput_t * motor = &dmaMotors[i];
        uint32_t *buffer = motor->dmaBuffer;
        UNUSED(buffer);
    }

    // XXX DEBUG
    gpio_bits_set(GPIOC, GPIO_PINS_10);

    for (int i = 0; i < dmaMotorTimerCount; i++) { // dmaMotorTimerCount is a global declared in pwm_output_dshot_shared.c
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xDMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, TRUE);

            // TIM_DMAConfig(dmaMotorTimers[i].timer, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
            // XXX it's not at all clear that the hardware will be compatible at this level
            tmr_dma_control_config(dmaMotorTimers[i].timer, TMR_DMA_TRANSFER_4BYTES, TMR_CTRL1_ADDRESS);

            // XXX TODO - there doesn't seem to be an equivalent of TIM_DMA_Update in AT32
            // TIM_DMACmd(dmaMotorTimers[i].timer, TIM_DMA_Update, ENABLE);
            // tmr_dma_request_enable(dmaMotorTimers[i].timer, )
        } else
#endif
        {
            // TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, DISABLE);

            // Allows setting the period with immediate effect
            tmr_period_buffer_enable(dmaMotorTimers[i].timer, FALSE);


            // dmaMotorTimers[i].timer->ARR = dmaMotorTimers[i].outputPeriod;

            // XXX This isn't set when telemetry is disabled. Do we come through here? Yes, and outputPeriod is 0
            // Will be needed for bidir, check if it needs to be conditional or can be enabled for both cases
            // use an api?
            // dmaMotorTimers[i].timer->pr = dmaMotorTimers[i].outputPeriod;


            // TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, ENABLE);
            tmr_period_buffer_enable(dmaMotorTimers[i].timer, TRUE);

            // Ensure that overflow events are enabled. This event may affect both period and duty cycle
            tmr_overflow_event_disable(dmaMotorTimers[i].timer, FALSE);

            // Generate requests from the timer to one or more DMA channels
            tmr_dma_request_enable(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, TRUE);

            dmaMotorTimers[i].timerDmaSources = 0;

            // I think the counter is reset here to ensure that the first pulse is the correct width
            tmr_counter_value_set(dmaMotorTimers[i].timer, 0);
        }
    }
    // XXX DEBUG
    gpio_bits_reset(GPIOC, GPIO_PINS_10);
}

/**
 * Interrupt handler called at the end of each packet
 * 
 * Responsible for switching the dshot direction after sending a dshot command so that we 
 * can receive dshot telemetry. If telemetry is not enabled, disables the dma and request generation.
*/

// XXX for testing
uint32_t lastDTVal;
bool mapGood = false;
bool dtBAD = false;

FAST_CODE static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {

        // XXX DEBUG
        gpio_bits_set(GPIOC, GPIO_PINS_11);

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
        // disable the dma channel
        xDMA_Cmd(motor->dmaRef, FALSE); // Gets re-enabled in pwm_output_dshot_shared.c from pwmWriteDshotInt

        // disable request generation
        tmr_dma_request_enable(motor->timerHardware->tim, motor->timerDmaSource, FALSE);


        // If we're expecting telem, flip direction and re-enable

#ifdef USE_DSHOT_TELEMETRY
        if (useDshotTelemetry) {
            pwmDshotSetDirectionInput(motor);
            xDMA_SetCurrDataCounter(motor->dmaRef, GCR_TELEMETRY_INPUT_LEN);
            xDMA_Cmd(motor->dmaRef, TRUE);
            // TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, ENABLE);
            // XXX check
            tmr_dma_request_enable(motor->timerHardware->tim, motor->timerDmaSource, TRUE);

            dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
        }
#endif
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

        // XXX DEBUG
        gpio_bits_reset(GPIOC, GPIO_PINS_11);

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

    // XXX testing - find a better place and only enable the mux(s) being used
    // merge into dmaMuxEnable?
    dmamux_enable(DMA1, TRUE);
    dmamux_enable(DMA2, TRUE);

    // If we use pinio config for debug pins then the init will probably be done for us, but not until after this function runs
    // grab a pin for debug, C10 will do for now
    gpio_init_type init = {
        .gpio_pins = GPIO_PINS_10,
        .gpio_mode = GPIO_MODE_OUTPUT, 
        .gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE,
        .gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
        .gpio_pull = GPIO_PULL_NONE
    };
    gpio_init(GPIOC, &init);


#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
        dmaMuxId = dmaSpec->dmaMuxId;
    }

#else // not defined USE_DMA_SPEC
    dmaRef = timerHardware->dmaRef;
#if defined(STM32F4)
    // XXX Fixme
    dmaChannel = timerHardware->dmaChannel;
#endif
#endif // USE_DMA_SPEC

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
#if defined(STM32F4)
        // XXX Fixme
        dmaChannel = timerHardware->dmaTimUPChannel;
#endif
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
    {
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

    // motor->iocfg = IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, pupMode);
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
    if (output & TIMER_OUTPUT_N_CHANNEL) {  // What's the benefit of using either the N or normal channels?
        // XXX N channels not yet tested
        // OCINIT.TIM_OutputNState = TIM_OutputNState_Enable;
        ocConfig->occ_output_state = TRUE;
        // OCINIT.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        ocConfig->occ_idle_state = FALSE;   // XXX is 'reset' equivalent to TRUE or FALSE???
        // OCINIT.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
        ocConfig->occ_polarity = (output & TIMER_OUTPUT_INVERTED) ? TMR_OUTPUT_ACTIVE_LOW : TMR_OUTPUT_ACTIVE_HIGH;
    } else {
        ocConfig->oc_output_state = TRUE;
        ocConfig->oc_idle_state = FALSE;
        ocConfig->oc_polarity = (output & TIMER_OUTPUT_INVERTED) ? TMR_OUTPUT_ACTIVE_LOW : TMR_OUTPUT_ACTIVE_HIGH;
    }

    // OCINIT.TIM_Pulse = 0;
    { // local scope for variable
        tmr_channel_select_type atChannel = toCHSelectType(timerHardware->channel, false);  // tmr_channel_value_set only supports the normal channels
        tmr_channel_value_set(timer, atChannel, 0);
        // tmr_channel_value_set(timer, atChannel, (motor->index + 1) * 2);
    }

#ifdef USE_DSHOT_TELEMETRY

    // typedef struct
    // {
    //   tmr_channel_select_type                input_channel_select;   /*!< tmr input channel select */
    //   tmr_input_polarity_type                input_polarity_select;  /*!< tmr input polarity select */
    //   tmr_input_direction_mapped_type        input_mapped_select;    /*!< tmr channel mapped direct or indirect */
    //   uint8_t                                input_filter_value;     /*!< tmr channel filter value */
    // } tmr_input_config_type;

    // TIM_ICStructInit(&motor->icInitStruct);
    tmr_input_config_type * icConfig = &motor->icInitStruct;
    tmr_input_default_para_init(icConfig);

    // motor->icInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    icConfig->input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;

    // motor->icInitStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    icConfig->input_polarity_select = TMR_INPUT_BOTH_EDGE;

    // motor->icInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1; //  XXX not part of at32 input config

    // motor->icInitStruct.TIM_Channel = timerHardware->channel;
    icConfig->input_channel_select = toCHSelectType(timerHardware->channel, false);

    // motor->icInitStruct.TIM_ICFilter = 2;
    icConfig->input_filter_value = 2;   // What are the units, is this valid?
#endif


#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
    } else
#endif
    {
        // returns 1 bit set to indicate one of TMR_C1_DMA_REQUEST, C2_, C3_, C4_
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);

        // clear that bit from timerDmaSources
        // timerDmaSources can have more than one source set in it if multiple motors share a common timer,
        // although at this stage I'm not sure why we can't just set it to 0
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
#endif
    {
        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];

        DMAINIT.memory_base_addr = (uint32_t)motor->dmaBuffer;
        DMAINIT.memory_inc_enable = TRUE;
        DMAINIT.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
        DMAINIT.loop_mode_enable = FALSE;

        DMAINIT.buffer_size = DSHOT_DMA_BUFFER_SIZE; // XXX shouldn't need to be set here, added for testing

        DMAINIT.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;

        // Nothing about fifo or burst in at32 init struct
        // DMAINIT.DMA_FIFOMode = DMA_FIFOMode_Enable;
        // DMAINIT.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;  // Are we relying on this for something?
        // DMAINIT.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        // DMAINIT.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

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
    {
        dmaSetHandler(dmaIdentifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    }

    // toplevel enable for the entire timer - starts the counter running
    tmr_counter_enable(timer, TRUE);

    { // local scope
        tmr_channel_select_type chSel = toCHSelectType(timerHardware->channel, output & TIMER_OUTPUT_N_CHANNEL);
        tmr_channel_enable(timer, chSel, TRUE); 
    }

    if (configureTimer) {
        // Changes to the period register are deferred until the next counter overflow
        tmr_period_buffer_enable(timer, TRUE);

        tmr_output_enable(timer, TRUE);

        // XXX We already did this above, why is it repeated here?
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
