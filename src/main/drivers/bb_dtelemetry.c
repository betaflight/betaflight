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

#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_BB_DSHOT

#include "build/debug.h"

#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/dma.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_counts.h"
#include "drivers/timer.h"

#include "common/time.h"
#include "drivers/time.h"


// XXX MOTOR_xSHOTyyyy_HZ is not usable as generic frequency for timers.
// XXX Trying to fiddle with constants here.

// Symbol rate [symbol/sec]
#define MOTOR_DSHOT1200_SYMBOL_RATE    (1200 * 1000)
#define MOTOR_DSHOT600_SYMBOL_RATE     (600 * 1000)
#define MOTOR_DSHOT300_SYMBOL_RATE     (300 * 1000)
#define MOTOR_DSHOT150_SYMBOL_RATE     (150 * 1000)
#define MOTOR_PROSHOT1000_SYMBOL_RATE  (250 * 1000)

#define MOTOR_DSHOT1200_SYMBOL_TIME    (1 / MOTOR_DSHOT1200_SYMBOL_RATE)   // 0.833us
#define MOTOR_DSHOT600_SYMBOL_TIME     (1 / MOTOR_DSHOT600_SYMBOL_RATE)    // 1.666us
#define MOTOR_DSHOT300_SYMBOL_TIME     (1 / MOTOR_DSHOT300_SYMBOL_RATE)    // 3.333us
#define MOTOR_DSHOT150_SYMBOL_TIME     (1 / MOTOR_DSHOT150_SYMBOL_RATE)    // 6.666us
#define MOTOR_PROSHOT1000_SYMBOL_TIME  (1 / MOTOR_PROSHOT1000_SYMBOL_RATE) // 4.000us

#define MOTOR_DSHOT_BIT_PER_SYMBOL         1
#define MOTOR_PROSHOT1000_BIT_PER_SYMBOL   4

#define MOTOR_DSHOT_STATE_PER_SYMBOL       3  // Initial high, 0/1, low
#define MOTOR_PROSHOT1000_STATE_PER_SYMBOL 32 // 8 preamble + 16 + 8 postamble

#define MOTOR_DSHOT_BUFFER_SIZE            ((16 / MOTOR_DSHOT_BIT_PER_SYMBOL) * MOTOR_DSHOT_STATE_PER_SYMBOL)
#define MOTOR_PROSHOT1000_BUFFER_SIZE      ((16 / MOTOR_PROSHOT1000_BIT_PER_SYMBOL) * MOTOR_PROSHOT1000_STATE_PER_SYMBOL)

// Receive strategy for DSHOT
// DMA starts at first EXTI interrupt on ANY of the motor signal lines.
// We need a EXTI to port group/DMA channel map as a single EXTI line may
// contain multiple motor signal lines.
// (Note that we may need to extend EXTI facility to handle multiple
// handerlers on the same line.)
//
// DMA length
// 3-State per bit
// Double sampling = 6 samples per bit.
// Receive window = 16 bits + channel skew (x2) = 32 bit time
// Receive buffer size = 32 * 6 = 192 halfword / port group

#define TLM_COUNT 192
//#define TLM_COUNT 4096
#define TLM_SLACK 16

volatile uint16_t tlm_buffer[TLM_COUNT + TLM_SLACK];
//uint16_t tlm_peek[TLM_COUNT + TLM_SLACK];
uint16_t tlm_bits[TLM_COUNT + TLM_SLACK];

static IO_t tlm_io;                // I/O port for a signal line
static GPIO_TypeDef *tlm_gpio;     // GPIO instance for actual motor signal
static uint16_t tlm_dmaSource;     // dmaSource bit for TIM_DMA
static const timerHardware_t *tlm_timhw; // Pacer timer channel
static uint32_t tlm_exti_mask = 0;

static void directDtelemetryDMAIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    debug[2]++;

    const timerHardware_t *timhw = (timerHardware_t *)descriptor->userParam;

    DMA_Cmd(timhw->dmaRef, DISABLE);
    TIM_DMACmd(timhw->tim, tlm_dmaSource, DISABLE);

    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

    for (int i = 0; i < TLM_COUNT; i++) {
        tlm_bits[i] = tlm_buffer[i] & 2;
    }

    // Experimental only; Start receiving next frame
    EXTIEnable(tlm_io, ENABLE);
}

// Return frequency of smallest change [state/sec]

extern uint32_t getDshotBaseFrequency(motorPwmProtocolTypes_e pwmProtocolType);

void directTelemetryTimebaseInit(const timerHardware_t *timhw, motorPwmProtocolTypes_e pwmProtocolType)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    // TIM_TimeBaseInitStruct.TIM_Prescaler = (uint16_t)(lrintf((float) timerClock(timhw->tim) / (3 * getDshotHz(pwmProtocolType)) + 0.01f) - 1);
    // TIM_TimeBaseInitStruct.TIM_Period = 20 - 1;

    TIM_TimeBaseInitStruct.TIM_Prescaler = 0; // Feed raw timerClock
    uint32_t basefreq = getDshotBaseFrequency(pwmProtocolType) * 2; // Double sampling
    uint32_t timerclock = timerClock(timhw->tim);
    uint32_t period = timerclock / basefreq;
    // TIM_TimeBaseInitStruct.TIM_Period = timerClock(timhw->tim) / getDshotBaseFrequency(pwmProtocolType);
    TIM_TimeBaseInitStruct.TIM_Period = period;

    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timhw->tim, &TIM_TimeBaseInitStruct);
    TIM_Cmd(timhw->tim, ENABLE);
}

// Initialize a telemetry pacer:
// A pacing timer channel and an associated DMA stream

void directTelemetryTimerChannelInit(const timerHardware_t *timhw)
{
    // const timerHardware_t *timhw = motorPort->timhw;

    TIM_OCInitTypeDef TIM_OCStruct;

    TIM_OCStructInit(&TIM_OCStruct);
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OCStruct.TIM_Pulse = 499;
    TIM_OC1Init(timhw->tim, &TIM_OCStruct);
    TIM_OC1PreloadConfig(timhw->tim, TIM_OCPreload_Enable);

    DMA_Stream_TypeDef *stream = timhw->dmaRef;

    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(stream);
    dmaInit(dmaIdentifier, OWNER_DTELEMETRY, 0);
    dmaSetHandler(dmaIdentifier, directDtelemetryDMAIrqHandler, NVIC_BUILD_PRIORITY(2, 1), (uint32_t)timhw);

    tlm_dmaSource = timerDmaSource(timhw->channel);

    memset((uint8_t *)tlm_buffer, 0, sizeof(uint16_t) * (TLM_COUNT + TLM_SLACK));

    DMA_InitTypeDef dmainit;

    DMA_StructInit(&dmainit);

    dmainit.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dmainit.DMA_Mode = DMA_Mode_Normal;
    dmainit.DMA_Channel = timhw->dmaChannel;
    dmainit.DMA_Priority = DMA_Priority_Medium;
    dmainit.DMA_BufferSize = TLM_COUNT; // XXX Magic for DSHOT

    dmainit.DMA_PeripheralBaseAddr = (uint32_t)&tlm_gpio->IDR;

    dmainit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmainit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dmainit.DMA_Memory0BaseAddr = (uint32_t)tlm_buffer;
    dmainit.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmainit.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dmainit.DMA_FIFOMode = DMA_FIFOMode_Disable ;
    dmainit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    dmainit.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    dmainit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_DeInit(stream);
    DMA_Init(stream, &dmainit);

    DMA_ITConfig(stream, DMA_IT_TC, ENABLE);
}

static extiCallbackRec_t telemetry_extiCallbackRec;

static void directDtelemetryEXTIHandler(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    // Start DMA for this port group

    DMA_Cmd(tlm_timhw->dmaRef, ENABLE);
    TIM_DMACmd(tlm_timhw->tim, tlm_dmaSource, ENABLE);

    // Stop EXTI for the port group the interrupted line belongs to

    EXTIEnableByLineMask(tlm_exti_mask, DISABLE);
    // EXTIEnable(tlm_io, DISABLE);
}

static void directDtelemetryIOInitInput(IO_t io)
{
    IOInit(io, OWNER_MOTOR, 0);
    IOLo(io);
    IOConfigGPIO(io, IOCFG_IN_FLOATING);

    EXTIHandlerInit(&telemetry_extiCallbackRec, directDtelemetryEXTIHandler);

    // XXX Be careful for full implementation; EXTIConfig does IOConfig.
    EXTIConfig(io, &telemetry_extiCallbackRec, NVIC_PRIO_TIMER, IOCFG_IN_FLOATING, EXTI_TRIGGER_RISING);
    // EXTIEnable(io, true);

    tlm_exti_mask |= IO_EXTI_Line(io);

    EXTIEnableByLineMask(tlm_exti_mask, DISABLE);

    // We might need to further extend EXTI to turn on/off multiple mask at once
    // EXTIEnableMulti(mask for lines)
    // EXTIDisableMulti(mask for lines)
}

void directTelemetryInit(motorPwmProtocolTypes_e pwmProtocolType)
{
    // Simulated motor signal pin (input only for now)

    tlm_io = IOGetByTag(IO_TAG(PA1)); // Test input pin for Omnibus F4 V3 (PWM5)

    // int pinIndex = IO_GPIOPinIdx(io);
    // int portIndex = IO_GPIOPortIdx(io);

    tlm_gpio = IO_GPIO(tlm_io); // Need this in directTelemetryTimerChannelInit

    const timerHardware_t *timhw;

    // XXX Should get TIM1, CH1, PA8 on Omnibus F4 V3 test platform
    timhw = timerGetByUsage(TIM_USE_BB_TELEMETRY, 0);

    if (!timhw) {
        return;
    }

    tlm_timhw = timhw;

    // Initialize pacer timer timebase
    directTelemetryTimebaseInit(timhw, pwmProtocolType);

    // Initialize pacer timer channel
    directTelemetryTimerChannelInit(timhw);

    directDtelemetryIOInitInput(tlm_io);
}

void directTelemetryStart(void)
{
    EXTIEnable(tlm_io, ENABLE);
}
#endif // USE_BB_DSHOT
