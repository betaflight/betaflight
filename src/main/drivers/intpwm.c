/* Copyright spell goes here */

//
// Interrupt based PWM input handler
//
// Don't use this for serious control input purposes.
//
// The interrupt based PWM reader is mainly intended for reading
// PWM based RSSI without an external RC filter nor a timer resource,
// and with least MCU cycles.
//
// Some difficulty with FrSky style PWM RSSI is that it translates RSSI
// into pulse width rather directly. With this scheme, We have to measure
// the pulse width as short as 20+usec.
//
// Another problem with the RSSI scheme is an ambiguity of the FrSky PWM
// RSSI specification. All FrSky manual states that the PWM RSSI is
// 'duty cycle = 1%~99%', but associated figures only shows the high period
// for the 'duty cycle', not showing where the period is.
// In reality that the cycle time seem to fluctuate between 998~1046usec. 
// 
// The short pulse problem is dealt with an exti handler that does
// absolute minimum job during measurement; it just reads the input and
// stores it. This is repeated for 4 times, and result is analyzed in
// rather pesimistic way in the sense that it just throws away the
// measurement result when it detect something has went wrong:
//
// (1) When triggered by updateIntpwm(), it accepts 4 consecutive
// interrupts triggered by rising and falling edges of the input PWM.
// (2) With each interrupt, input pin level and timestamp are recorded.
// (3) After the 4th interrupt, the level pattern is examined, which is
// expected to be 0101 or 1010. If the recorded pattern does not
// match with the expected patterns, the whole measurement is discarded,
// and the measurement does not provide any valid result.
// (4) If the pattern matches, then the timestamps are converted into
// intervals.
// (5) Intervals are checked, and the measurement is discarded if any
// invalid duration is detected.
// (6) Finally, an interval corresponding to the 010 pattern is
// extracted as the "high period".
//
// Second problem is ignored, and the period is assumed to be constant.
//

#include <stdbool.h>
#include <stdint.h>

#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build_config.h"
#include "debug.h"

#include "common/maths.h"

#ifdef USE_INTPWM

#ifndef USE_EXTI
Configuration problem: USE_INTPWM requires USE_EXTI
#endif

#include "system.h"
#include "exti.h"
#include "io.h"
#include "gpio.h"
#include "nvic.h"

#include "drivers/intpwm.h"

static IO_t intpwmIO;
static extiCallbackRec_t intpwm_extiCallbackRec;

// Measurement array
static volatile uint32_t tStamp[4]; // Time stamps
static volatile union {        // Signal levels
    uint8_t sb[4];
    uint32_t sigs;
} pwmRec;

static volatile int recCount;

// Valid value for pwmRec.sigs, corresponding to 1010b and 0101b
#define SIGPAT1 ((1 << (8*0)) | (0 << (8*1)) | (1 << (8*2)) | (0 << (8*3)))
#define SIGPAT2 ((0 << (8*0)) | (1 << (8*1)) | (0 << (8*2)) | (1 << (8*3)))

static bool intpwmInitialized = false;
static bool intpwmActive = false;
static volatile bool intpwmInProgress = false;
static volatile uint32_t pulseWidth;

extern uint16_t rssi; // Defined in rx.c

#define INTPWM_MAX_PULSE 2500 // To detect glitch in micros()

void intpwmExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    if (!intpwmInProgress)
        return;

    if (recCount < 4) {
        pwmRec.sb[recCount] = IORead(intpwmIO);
        tStamp[recCount++] = micros();
    }

    if (recCount < 4)
        return;

    // Disable interrupt
    EXTIEnable(intpwmIO, false);

    intpwmInProgress = false;
}

static uint32_t tmin = 0xFFFFFFFF;
static uint32_t tmax = 0;

void computePulse(void)
{
    if (pwmRec.sigs != SIGPAT1 && pwmRec.sigs != SIGPAT2)
        return;

    // Compute intervals

    int i;

    for (i = 0 ; i < 3 ; i++)
        tStamp[i] = tStamp[i+1] - tStamp[i];

    // After converting to intervals, check for invalid intervals,
    // discard the whole measurement if found.

    for (i = 0 ; i < 3 ; i++)


    for (i = 0 ; i < 3 ; i++) {
        // If the measurement contains out range values,
        // ignore this measurement. (Was counter-going-back-micros; keep it?)

        if (tStamp[i] > INTPWM_MAX_PULSE)
            return;

        // Monitor pulseWidths for parameter decision.

        if (pulseWidth > tmax)
            tmax = pulseWidth;

        if (pulseWidth < tmin)
            tmin = pulseWidth;

        debug[2] = tmin;
        debug[3] = tmax;

        if (tStamp[i] > 1023 || tStamp[i] < 20)
            return;
    }

    if (pwmRec.sb[0] == 0) {
        pulseWidth = tStamp[1];
    } else {
        pulseWidth = tStamp[2];
    }

    debug[1] = pulseWidth;
}

bool intpwmInit(void)
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

    intpwmIO = IOGetByTag(IO_TAG(INTPWM_PIN)); // XXX Config var?

    if (IOGetOwner(intpwmIO) != OWNER_FREE)
        return false;

    IOInit(intpwmIO, OWNER_INTPWM, RESOURCE_INPUT, 0);
    IOConfigGPIO(intpwmIO, IOCFG_IN_FLOATING);

    EXTIHandlerInit(&intpwm_extiCallbackRec, intpwmExtiHandler);
    EXTIConfig(intpwmIO, &intpwm_extiCallbackRec, NVIC_PRIO_INTPWM_EXTI, EXTI_Trigger_Rising_Falling);

    intpwmInitialized = true;
    intpwmActive = false;

    return true;
}

// updateIntpwm(): Scheduler task
//
// Uses last measurement to compute a new value, then triggers a new
// measurement.
// The measurement itself takes 5 edges, so it will take 3 cycles at
// the most. At 1000us cycle, this is 3ms. Therefore, the scheduling
// interval should be longer than 3ms (333Hz) in this case.
// As the interval gets longer, the measurement will be out dated, but
// would not be a problem for RSSI measurements; 50Hz (17ms old) or
// even 10Hz (97ms old) would suffice.

void updateIntpwm(void)
{
    uint32_t value;

    if (!intpwmInitialized) {
        return;
    }

    if (intpwmInProgress) {
        // Measurement in progress
        // XXX Check for non-interrupting case!?
        return;
    }

    if (!intpwmActive) {
        // Dodge the first call.
        intpwmActive = true;
    } else {
        computePulse();

        if (pulseWidth >= INTPWM_INPUT_MIN
          && pulseWidth <= INTPWM_INPUT_MAX) {
            // Valid duration, compute the scaled value.

          value = scaleRange(pulseWidth,
                        INTPWM_INPUT_MIN, INTPWM_INPUT_MAX,   // Config var?
                        INTPWM_OUTPUT_MIN, INTPWM_OUTPUT_MAX); // Config var?
        } else {
            value = 0; // Signify failure
        }

        // In terms of layering, this assignment should be done
        // somewhere higher; e.g., RSSI task in sensors.
        rssi = value;
    }

    // Start a new measurement
    recCount = 0;

    // Mark it busy
    intpwmInProgress = true;

    // Enable interrupt
    EXTIEnable(intpwmIO, true);
}

#endif // USE_INTPWM
