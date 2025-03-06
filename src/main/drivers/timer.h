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
#include "drivers/rcc_types.h"
#include "drivers/resource.h"

#ifdef USE_TIMER
#include "timer_def.h"
#endif

#include "pg/timerio.h"

#define CC_CHANNELS_PER_TIMER         4 // TIM_Channel_1..4
#ifdef AT32F435
#define CC_INDEX_FROM_CHANNEL(x)      ((uint8_t)(x) - 1)
#define CC_CHANNEL_FROM_INDEX(x)      ((uint16_t)(x) + 1)
#else
#define CC_CHANNEL_FROM_INDEX(x)      ((uint16_t)(x) << 2)
#define CC_INDEX_FROM_CHANNEL(x)      ((uint8_t)((x) >> 2))
#endif

#define TIM_CH_TO_SELCHANNEL(ch)  ((ch - 1) * 2)

typedef uint16_t captureCompare_t;        // 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)

typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;

// use different types from capture and overflow - multiple overflow handlers are implemented as linked list
struct timerCCHandlerRec_s;
struct timerOvrHandlerRec_s;
typedef void timerCCHandlerCallback(struct timerCCHandlerRec_s* self, uint16_t capture);
typedef void timerOvrHandlerCallback(struct timerOvrHandlerRec_s* self, uint16_t capture);

typedef struct timerCCHandlerRec_s {
    timerCCHandlerCallback* fn;
} timerCCHandlerRec_t;

typedef struct timerOvrHandlerRec_s {
    timerOvrHandlerCallback* fn;
    struct timerOvrHandlerRec_s* next;
} timerOvrHandlerRec_t;

typedef struct timerDef_s {
    TIM_TypeDef *TIMx;
    rccPeriphTag_t rcc;
    uint8_t inputIrq;
} timerDef_t;

typedef struct timerHardware_s {
    TIM_TypeDef *tim;
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

extern const timerDef_t timerDefinitions[];

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
void timerStart(TIM_TypeDef *tim);

//
// per-channel
//

void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterSamples);
void timerChConfigICDual(const timerHardware_t* timHw, bool polarityRising, unsigned inputFilterSamples);
void timerChICPolarity(const timerHardware_t *timHw, bool polarityRising);
volatile timCCR_t* timerChCCR(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRLo(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRHi(const timerHardware_t* timHw);
void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool stateHigh);
void timerChConfigGPIO(const timerHardware_t* timHw, ioConfig_t mode);

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn);
void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn);
void timerChConfigCallbacks(const timerHardware_t *channel, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);
void timerChConfigCallbacksDual(const timerHardware_t *channel, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback);
void timerChITConfigDualLo(const timerHardware_t* timHw, FunctionalState newState);
void timerChITConfig(const timerHardware_t* timHw, FunctionalState newState);
void timerChClearCCFlag(const timerHardware_t* timHw);

void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq);

//
// per-timer
//

void timerForceOverflow(TIM_TypeDef *tim);

void timerConfigUpdateCallback(const TIM_TypeDef *tim, timerOvrHandlerRec_t *updateCallback);

uint32_t timerClock(const TIM_TypeDef *tim);

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz);  // TODO - just for migration
void timerReconfigureTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz);

rccPeriphTag_t timerRCC(const TIM_TypeDef *tim);
uint8_t timerInputIrq(const TIM_TypeDef *tim);

#if defined(USE_TIMER_MGMT)
extern const resourceOwner_t freeOwner;

struct timerIOConfig_s;

struct timerIOConfig_s *timerIoConfigByTag(ioTag_t ioTag);
const timerHardware_t *timerGetAllocatedByNumberAndChannel(int8_t timerNumber, uint16_t timerChannel);
const resourceOwner_t *timerGetOwner(const timerHardware_t *timer);
#endif
const timerHardware_t *timerGetConfiguredByTag(ioTag_t ioTag);
const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex);
const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex);

#if defined(USE_HAL_DRIVER)
TIM_HandleTypeDef* timerFindTimerHandle(TIM_TypeDef *tim);
HAL_StatusTypeDef TIM_DMACmd(TIM_HandleTypeDef *htim, uint32_t Channel, FunctionalState NewState);
HAL_StatusTypeDef DMA_SetCurrDataCounter(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
uint16_t timerDmaIndex(uint8_t channel);
#else
void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init);
void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t preload);
#endif

volatile timCCR_t *timerCCR(TIM_TypeDef *tim, uint8_t channel);
uint16_t timerDmaSource(uint8_t channel);

uint16_t timerGetPrescalerByDesiredHertz(TIM_TypeDef *tim, uint32_t hz);
uint16_t timerGetPrescalerByDesiredMhz(TIM_TypeDef *tim, uint16_t mhz);
uint16_t timerGetPeriodByPrescaler(TIM_TypeDef *tim, uint16_t prescaler, uint32_t hz);

int8_t timerGetNumberByIndex(uint8_t index);
int8_t timerGetTIMNumber(const TIM_TypeDef *tim);
uint8_t timerLookupChannelIndex(const uint16_t channel);

// TODO: replace TIM_TypeDef with alternate
void timerReset(TIM_TypeDef *timer);
void timerSetPeriod(TIM_TypeDef *timer, uint32_t period);
uint32_t timerGetPeriod(TIM_TypeDef *timer);
void timerSetCounter(TIM_TypeDef *timer, uint32_t counter);
void timerDisable(TIM_TypeDef *timer);
void timerEnable(TIM_TypeDef *timer);
void timerEnableInterrupt(TIM_TypeDef *timer);
