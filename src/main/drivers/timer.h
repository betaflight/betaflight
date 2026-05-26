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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/dma.h"
#include "drivers/io_types.h"
#include "drivers/timer_types.h"

#if PLATFORM_TRAIT_RCC
#include "platform/rcc_types.h"
#endif

#include "drivers/resource.h"

#ifdef USE_TIMER
#include "timer_def.h"
#endif

#include "pg/timerio.h"

#define CC_CHANNELS_PER_TIMER         4 // TIM_Channel_1..4
#ifndef CC_CHANNEL_FROM_INDEX
#define CC_CHANNEL_FROM_INDEX(x)      ((uint16_t)(x) << 2)
#define CC_INDEX_FROM_CHANNEL(x)      ((uint8_t)((x) >> 2))
#endif

#define TIM_CH_TO_SELCHANNEL(ch)  ((ch - 1) * 2)
#define TIM_N(n) (1 << (n))
#define TIMER_INDEX(i) BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

typedef uint16_t captureCompare_t;        // 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)


// use different types from capture and overflow - multiple overflow handlers are implemented as linked list
struct timerCCHandlerRec_s;
struct timerOvrHandlerRec_s;
typedef void timerCCHandlerCallback(struct timerCCHandlerRec_s* self, uint16_t capture);
typedef void timerOvrHandlerCallback(struct timerOvrHandlerRec_s* self, uint16_t capture);

typedef struct timerCCHandlerRec_s {
    timerCCHandlerCallback* fn;
} timerEdgeHandlerRec_t;

typedef struct timerOvrHandlerRec_s {
    timerOvrHandlerCallback* fn;
    struct timerOvrHandlerRec_s* next;
} timerOvrHandlerRec_t;

typedef struct timerHardware_s {
    timerResource_t *tim;
    ioTag_t tag;
    uint8_t channel;
    uint8_t output;
    uint8_t alternateFunction;

#if defined(USE_TIMER_DMA)
#if defined(USE_DMA_SPEC)
    dmaResource_t *dmaRefConfigured;
    uint32_t dmaChannelConfigured;
#else // USE_DMA_SPEC
    dmaResource_t *dmaRef;
    uint32_t dmaChannel; // XXX Can be much smaller (e.g. uint8_t)
#endif // USE_DMA_SPEC
    dmaResource_t *dmaTimUPRef;
    uint32_t dmaTimUPChannel;
    uint8_t dmaTimUPIrqHandler;
#endif
} timerHardware_t;

typedef enum {
    TIMER_OUTPUT_NONE      = 0,
    TIMER_OUTPUT_INVERTED  = (1 << 0),
    TIMER_OUTPUT_N_CHANNEL = (1 << 1),
} timerFlag_e;

#define MHZ_TO_HZ(x) ((x) * 1000000)

extern const timerHardware_t fullTimerHardware[];

#define TIMER_CHANNEL_COUNT FULL_TIMER_CHANNEL_COUNT
#define TIMER_HARDWARE fullTimerHardware

typedef enum {
    TYPE_FREE,
    TYPE_PWMINPUT,
    TYPE_PPMINPUT,
    TYPE_PWMOUTPUT_MOTOR,
    TYPE_PWMOUTPUT_FAST,
    TYPE_PWMOUTPUT_SERVO,
    TYPE_SOFTSERIAL_RX,
    TYPE_SOFTSERIAL_TX,
    TYPE_SOFTSERIAL_RXTX,        // bidirectional pin for softserial
    TYPE_SOFTSERIAL_AUXTIMER,    // timer channel is used for softserial. No IO function on pin
    TYPE_ADC,
    TYPE_SERIAL_RX,
    TYPE_SERIAL_TX,
    TYPE_SERIAL_RXTX,
    TYPE_TIMER
} channelType_t;

//
// Legacy API
//
void timerConfigure(const timerHardware_t *timHw, uint16_t period, uint32_t hz);

//
// Initialisation
//
void timerInit(void);

//
// per-timer
//

// once-upon-a-time all the timers were started on boot, now they are started when needed.
void timerStart(const timerHardware_t *timHw);

//
// per-channel
//

void timerChannelConfigInput(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterSamples);
void timerChannelConfigInputDual(const timerHardware_t* timHw, bool polarityRising, unsigned inputFilterSamples);
void timerChannelInputPolarity(const timerHardware_t *timHw, bool polarityRising);
void timerChannelConfigOutput(const timerHardware_t* timHw, bool outEnable, bool stateHigh);
void timerChannelConfigGPIO(const timerHardware_t* timHw, ioConfig_t mode);

void timerChannelEdgeHandlerInit(timerEdgeHandlerRec_t *self, timerCCHandlerCallback *fn);
void timerChannelOverflowHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn);
void timerChannelConfigCallbacks(const timerHardware_t *channel, timerEdgeHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);
void timerChannelConfigInterrupt(const timerHardware_t* timHw, FunctionalState newState);
void timerChannelClearFlag(const timerHardware_t *timHw);

void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq);

//
// per-timer
//

void timerConfigUpdateCallback(const timerHardware_t *timHw, timerOvrHandlerRec_t *updateCallback);

uint32_t timerClock(const timerHardware_t *timHw);

void timerReconfigureTimeBase(const timerHardware_t *timHw, uint16_t period, uint32_t hz);

uint8_t timerInputInterrupt(const timerHardware_t *timHw);

#if defined(USE_TIMER_MGMT)
struct timerIOConfig_s;

struct timerIOConfig_s *timerIoConfigByTag(ioTag_t ioTag);
const timerHardware_t *timerGetAllocatedByNumberAndChannel(int8_t timerNumber, uint16_t timerChannel);
const resourceOwner_t *timerGetOwner(const timerHardware_t *timer);
#endif
const timerHardware_t *timerGetConfiguredByTag(ioTag_t ioTag);
const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex);
const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex);

int8_t timerGetNumberByIndex(uint8_t index);
int8_t timerGetIndexByNumber(uint8_t number);
int8_t timerGetTIMNumber(const timerHardware_t *timHw);

void timerReset(const timerHardware_t *timHw);
void timerSetPeriod(const timerHardware_t *timHw, uint32_t period);
uint32_t timerGetPeriod(const timerHardware_t *timHw);
void timerSetCounter(const timerHardware_t *timHw, uint32_t counter);
void timerDisable(const timerHardware_t *timHw);
void timerEnable(const timerHardware_t *timHw);
void timerEnableInterrupt(const timerHardware_t *timHw);
uint32_t timerGetPrescaler(const timerHardware_t *timHw);
