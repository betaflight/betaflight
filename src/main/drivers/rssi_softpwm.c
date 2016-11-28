/* Copyright spell goes here */

//
// Interrupt based FrSky PWM RSSI
//

#include <stdbool.h>
#include <stdint.h>

#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"

#ifdef USE_RSSI_SOFTPWM

#ifndef USE_EXTI
Configuration problem: USE_RSSI_SOFTPWM requires USE_EXTI
#endif

#include "system.h"
#include "exti.h"
#include "io.h"
#include "gpio.h"
#include "nvic.h"

#include "drivers/rssi_softpwm.h"

#include "config/config_profile.h"
#include "config/config_master.h"
//#include "config/feature.h"

static IO_t rssiSoftPwmIO;
static extiCallbackRec_t rssiSoftPwm_extiCallbackRec;

// Measurement array
static volatile uint8_t sampleLVL[4];     // Input levels
static volatile uint32_t sampleTS[4]; // Time stamps
static volatile int sampleCount;

static bool rssiSoftPwmInitialized = false;
static bool rssiSoftPwmActive = false;
static volatile bool rssiSoftPwmInProgress = false;

static volatile uint32_t rawWidth;
static volatile uint32_t pulseWidth;

static uint8_t rxtype;
#define RXTYPE_FRSKY_X4R    0
#define RXTYPE_FRSKY_TFR4   1

extern uint16_t rssi; // XXX Defined in rx.c, should not access it.

#define INTPWM_INPUT_MIN      1
#define INTPWM_INPUT_MAX    999
#define INTPWM_OUTPUT_MIN     0
#define INTPWM_OUTPUT_MAX  1023

static uint16_t pulseMin;
static uint16_t pulseMax;

static void rssiSoftPwmExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    static bool running = false;
    static uint32_t timeStart;

    if (!rssiSoftPwmInProgress)
        return;

    if (!running) {
        timeStart = micros();
        EXTIConfig(rssiSoftPwmIO, &rssiSoftPwm_extiCallbackRec, NVIC_PRIO_INTPWM_EXTI, EXTI_Trigger_Falling);
        running = true;
    } else {
        rawWidth = micros() - timeStart;
        EXTIEnable(rssiSoftPwmIO, false);
        rssiSoftPwmInProgress = false;
        running = false;
    }
}

static uint32_t tmin = 0xFFFFFFFF;
static uint32_t tmax = 0;

static void computePulse(void)
{
    debug[1] = rawWidth;

    switch (rxtype) {
    case RXTYPE_FRSKY_TFR4:
        if (rawWidth > pulseMax) {
            pulseWidth = pulseMax;
        } else if (rawWidth < pulseMin) {
            pulseWidth = pulseMin;
        } else {
            pulseWidth = rawWidth;
        }
        break;

    default:
        if (rawWidth >= 990) {
            // Must have missed the first falling edge
            if (rawWidth < 1000 + 200)
                pulseWidth = (rawWidth - 990) / 2;
            else
                pulseWidth = 0; // Fail
        } else {
            pulseWidth = rawWidth;
        }
        break;
    }

    debug[2] = pulseWidth;

    // Monitor pulseWidths for parameter decision.

    if (rawWidth > tmax)
        tmax = rawWidth;

    if (rawWidth < tmin)
        tmin = rawWidth;

    debug[0] = tmax;
}

bool rssiSoftPwmInit(void)
{
    // Nothing special, standard spell.

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#if defined(STM32F3) || defined(STM32F4)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

    rssiSoftPwmIO = IOGetByTag(IO_TAG(RSSI_SOFTPWM_PIN)); // XXX Config var?

    if (IOGetOwner(rssiSoftPwmIO) != OWNER_FREE)
        return false;

    IOInit(rssiSoftPwmIO, OWNER_INTPWM, 0);
    IOConfigGPIO(rssiSoftPwmIO, IOCFG_IN_FLOATING);

    EXTIHandlerInit(&rssiSoftPwm_extiCallbackRec, rssiSoftPwmExtiHandler);
    EXTIConfig(rssiSoftPwmIO, &rssiSoftPwm_extiCallbackRec, NVIC_PRIO_INTPWM_EXTI, EXTI_Trigger_Rising);

    rxtype = masterConfig.rssi_softpwm_device;

    switch(rxtype) {
    case RXTYPE_FRSKY_TFR4:
        pulseMin = 24;
        pulseMax = 1024;
        break;

    default:
        pulseMin = 1;
        pulseMax = 999;
        break;
    }

    rssiSoftPwmInitialized = true;
    rssiSoftPwmActive = false;

    return true;
}

// updateIntpwm(): Scheduler task

void rssiSoftPwmUpdate(uint32_t currentTime)
{
    UNUSED(currentTime);

    uint32_t value;

    if (!rssiSoftPwmInitialized) {
        return;
    }

    if (rssiSoftPwmInProgress) {
        // Measurement in progress
        // XXX Check for non-interrupting case!?
        return;
    }

    if (!rssiSoftPwmActive) {
        // Dodge the first call.
        rssiSoftPwmActive = true;
    } else {
        computePulse();

        if (pulseWidth >= pulseMin
          && pulseWidth <= pulseMax) {
            // Valid duration, compute the scaled value.

          value = scaleRange(pulseWidth,
                        pulseMin, pulseMax,   // Config var?
                        INTPWM_OUTPUT_MIN, INTPWM_OUTPUT_MAX); // Config var?
        } else {
            value = 0;
        }

        // In terms of layering, this assignment should be done
        // somewhere higher; e.g., RSSI task in sensors.
        rssi = value;

        if (value > 100) {
            debug[3] = rawWidth;
        }
    }

    // Start a new measurement
    rssiSoftPwmInProgress = true;
    EXTIConfig(rssiSoftPwmIO, &rssiSoftPwm_extiCallbackRec, NVIC_PRIO_INTPWM_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(rssiSoftPwmIO, true);
}

#endif // USE_RSSI_SOFTPWM
