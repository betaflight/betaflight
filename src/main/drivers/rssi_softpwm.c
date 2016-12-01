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
#include "build/atomic.h"
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

#define RSSI_OUTPUT_MIN     0
#define RSSI_OUTPUT_MAX  1023

static uint16_t pulseMin;
static uint16_t pulseMax;

static bool timerRunning = false;

static void rspExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    static uint32_t timeStart;

    if (!rspInProgress)
        return;

    if (!timerRunning) {
        timeStart = micros();
        EXTIConfig(rspIO, &rsp_extiCallbackRec, NVIC_PRIO_SOFTPWM_EXTI, EXTI_Trigger_Falling);
        timerRunning = true;
    } else {
        rawWidth = micros() - timeStart;
        EXTIEnable(rspIO, false);
        rspInProgress = false;
        timerRunning = false;
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

    case RXTYPE_FRSKY_X4R: // Probably all X series

        // X4R's RSSI signal is evil; shortest pulse is less than 3usec,
        // which is beyond the limit of software assisted PWM input can
        // reliably measure (probably even with input capture).
        // Here, we admit that we can miss the falling edge of the pulse
        // we have started our measurement, and compensate for the extra
        // cycle (effectively measuring the next pulse provided cycle time
        // is stable).

        if (rawWidth >= 990) {
            // Must have missed the first falling edge
            if (rawWidth < 1000 + 100) {
                if (rawWidth < 1000)
                    pulseWidth = 0;
                else
                    pulseWidth = rawWidth - 1000;
            } else {
                // Something went wrong
                pulseWidth = 0; // Fail
            }
        } else {
            pulseWidth = rawWidth;
        }
        break;

    default:
        pulseWidth = 500; // Default marker
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
    // XXX Are these done in exti.c already?
#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#if defined(STM32F3) || defined(STM32F4)
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
    EXTIConfig(rspIO, &rsp_extiCallbackRec, NVIC_PRIO_SOFTPWM_EXTI, EXTI_Trigger_Rising);

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

// rssiSoftPwmUpdate: Scheduler task

void rssiSoftPwmUpdate(uint32_t currentTime)
{
    UNUSED(currentTime);

    uint32_t value;
    bool longPulse = false;
    uint8_t longValue;

    if (!rspInitialized) {
        return;
    }

    ATOMIC_BLOCK(NVIC_PRIO_SOFTPWM_EXTI) {
        if (rspInProgress) {
            // No falling edge has been detected for a scheduling interval.
            // Remember the pin level and disable interrupt.
            longValue = IORead(rspIO);
            longPulse = true;
            rspInProgress = false;
            EXTIEnable(rspIO, false);
            timerRunning = false;
        }
    }

    if (!rspActive) {
        // Dodge the first call.
        rspActive = true;
    } else {
        if (longPulse) {
            pulseWidth = longValue ? pulseMax : pulseMin;
        } else {
            rspComputePulse();
        }

        if (pulseWidth >= pulseMin && pulseWidth <= pulseMax) {

            // Valid duration, compute the scaled value.

          value = scaleRange(pulseWidth, pulseMin, pulseMax,
                            RSSI_OUTPUT_MIN, RSSI_OUTPUT_MAX);
        } else {
            value = 0;
        }

#if 1
        if (value > 800) {
            debug[3] = rawWidth;
        }
#endif

        // Apply 5 position median filter to filter out occasional spikes
        rspFilterArray[rspFilterPos] = value;
        rspFilterPos = (rspFilterPos + 1) % 5;
        rspValue = (uint16_t)quickMedianFilter5(rspFilterArray);

        debug[2] = rspValue;
    }

    // Start a new measurement

    rspInProgress = true;
    EXTIConfig(rspIO, &rsp_extiCallbackRec, NVIC_PRIO_SOFTPWM_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(rspIO, true);
}

uint16_t rssiSoftPwmRead(void)
{
    return rspValue;
}

#endif // USE_RSSI_SOFTPWM
