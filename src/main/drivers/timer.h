/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#if !defined(USABLE_TIMER_CHANNEL_COUNT)
#define USABLE_TIMER_CHANNEL_COUNT 14
#endif

typedef uint16_t captureCompare_t;        // 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)

#if defined(STM32F303)
typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;
#elif defined(STM32F10X)
typedef uint16_t timCCR_t;
typedef uint16_t timCCER_t;
typedef uint16_t timSR_t;
typedef uint16_t timCNT_t;
#elif defined(UNIT_TEST)
typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;
#else
# error "Unknown CPU defined"
#endif

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

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint16_t pin;
    uint8_t channel;
    uint8_t irq;
    uint8_t outputEnable;
    GPIO_Mode gpioInputMode;
#ifdef STM32F303
    uint8_t gpioPinSource;             // TODO - this can be removed and pinSource calculated from pin
    uint8_t alternateFunction;
#endif
} timerHardware_t;

extern const timerHardware_t timerHardware[];

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

void timerConfigure(const timerHardware_t *timHw, uint16_t period, uint8_t mhz);  // This interface should be replaced.

void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterSamples);
void timerChConfigICDual(const timerHardware_t* timHw, bool polarityRising, unsigned inputFilterSamples);
void timerChICPolarity(const timerHardware_t *timHw, bool polarityRising);
volatile timCCR_t* timerChCCR(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRLo(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRHi(const timerHardware_t* timHw);
void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool stateHigh);
void timerChConfigGPIO(const timerHardware_t* timHw, GPIO_Mode mode);

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn);
void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn);
void timerChConfigCallbacks(const timerHardware_t *channel, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);
void timerChConfigCallbacksDual(const timerHardware_t *channel, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback);
void timerChITConfigDualLo(const timerHardware_t* timHw, FunctionalState newState);
void timerChITConfig(const timerHardware_t* timHw, FunctionalState newState);
void timerChClearCCFlag(const timerHardware_t* timHw);

void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority);

void timerInit(void);
void timerStart(void);
void timerForceOverflow(TIM_TypeDef *tim);

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz);  // TODO - just for migration

