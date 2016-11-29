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

static IO_t rspIO;
static extiCallbackRec_t rsp_extiCallbackRec;

static bool rspInitialized = false;
static bool rspActive = false;
static volatile bool rspInProgress = false;

static volatile uint32_t rawWidth;
static volatile uint32_t pulseWidth;

static uint8_t rxtype;

static uint16_t rspValue;

#define INTPWM_INPUT_MIN      1
#define INTPWM_INPUT_MAX    999
#define INTPWM_OUTPUT_MIN     0
#define INTPWM_OUTPUT_MAX  1023

static uint16_t pulseMin;
static uint16_t pulseMax;

static void rspExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    static bool running = false;
    static uint32_t timeStart;

    if (!rspInProgress)
        return;

    if (!running) {
        timeStart = micros();
        EXTIConfig(rspIO, &rsp_extiCallbackRec, NVIC_PRIO_INTPWM_EXTI, EXTI_Trigger_Falling);
        running = true;
    } else {
        rawWidth = micros() - timeStart;
        EXTIEnable(rspIO, false);
        rspInProgress = false;
        running = false;
    }
}

// Measurement for min and max pulse width.
// Should eventually be gone.
static uint32_t tmin = 0xFFFFFFFF;
static uint32_t tmax = 0;

static void rspComputePulse(void)
{
    debug[0] = rawWidth;

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

    debug[1] = pulseWidth;

    // Monitor pulseWidths for parameter decision.

    if (rawWidth > tmax)
        tmax = rawWidth;

    if (rawWidth < tmin)
        tmin = rawWidth;

    debug[4] = tmax;
}

static int32_t rspFilterArray[5];
static int rspFilterPos;

bool rssiSoftPwmInit(void)
{
    // Nothing special, standard spell.

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#if defined(STM32F3) || defined(STM32F4)
#if 0
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
#endif

    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

    ioTag_t tag = masterConfig.rssiSoftPwmConfig.ioTag;
    if (tag == IO_TAG_NONE)
        return false;

    rspIO = IOGetByTag(tag);

    IOInit(rspIO, OWNER_RSSIPWM, 0);
    IOConfigGPIO(rspIO, IOCFG_IN_FLOATING);

    EXTIHandlerInit(&rsp_extiCallbackRec, rspExtiHandler);
    EXTIConfig(rspIO, &rsp_extiCallbackRec, NVIC_PRIO_INTPWM_EXTI, EXTI_Trigger_Rising);

    rxtype = masterConfig.rssiSoftPwmConfig.device;

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

    rspInitialized = true;
    rspActive = false;

    for (int i = 0 ; i < 5 ; i++)
        rspFilterArray[i] = 0;

    rspFilterPos = 0;

    return true;
}

// rssiSoftPwmUpdate(): Scheduler task

void rssiSoftPwmUpdate(uint32_t currentTime)
{
    UNUSED(currentTime);

    uint32_t value;

    if (!rspInitialized) {
        return;
    }

    if (rspInProgress) {
        // Measurement in progress
        // XXX Check for non-interrupting case!?
        return;
    }

    if (!rspActive) {
        // Dodge the first call.
        rspActive = true;
    } else {
        rspComputePulse();

        if (pulseWidth >= pulseMin && pulseWidth <= pulseMax) {

          // Valid duration, compute the scaled value.

          value = scaleRange(pulseWidth, pulseMin, pulseMax,
                        INTPWM_OUTPUT_MIN, INTPWM_OUTPUT_MAX);
        } else {
            value = 0;
        }

#if 1
        if (value > 800) {
            debug[3] = rawWidth;
        }
#endif

        rspFilterArray[rspFilterPos] = value;
        rspFilterPos = (rspFilterPos + 1) % 5;
        rspValue = (uint16_t)quickMedianFilter5(rspFilterArray);

        debug[2] = rspValue;
    }

    // Start a new measurement

    rspInProgress = true;
    EXTIConfig(rspIO, &rsp_extiCallbackRec, NVIC_PRIO_INTPWM_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(rspIO, true);
}

uint16_t rssiSoftPwmRead(void)
{
    return rspValue;
}

#endif // USE_RSSI_SOFTPWM
