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

#include "build/build_config.h"
#include "build/debug.h"

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
static volatile uint8_t sampleLVL[4];     // Input levels
static volatile uint32_t sampleTS[4]; // Time stamps
static volatile int sampleCount;

static bool intpwmInitialized = false;
static bool intpwmActive = false;
static volatile bool intpwmInProgress = false;
static volatile uint32_t pulseWidth;

extern uint16_t rssi; // Defined in rx.c

void intpwmExtiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);

    if (!intpwmInProgress)
        return;

    if (sampleCount < 3) {
        sampleLVL[sampleCount] = IORead(intpwmIO);
        sampleTS[sampleCount] = micros();
    }

    if (++sampleCount == 3) {
        // Disable interrupt
        EXTIEnable(intpwmIO, false);
        intpwmInProgress = false;
    }
}

static uint32_t tmin = 0xFFFFFFFF;
static uint32_t tmax = 0;

void computePulse(void)
{
    uint8_t sigbits;

    pulseWidth = 0;
    sigbits = ((sampleLVL[0] << 2)|(sampleLVL[1] << 1)|(sampleLVL[2]));

    // Compute intervals

    for (int i = 0 ; i < 2 ; i++) {
        sampleTS[i] = sampleTS[i+1] - sampleTS[i];
        if (sampleTS[i] > 1200)
            return;
    }

#define PULSE_LIMIT    1100
#define PULSE_INTERVAL 1000
#define PULSE_MAX_SHORT  40
#define PULSE_MIN_LONG  960

    switch (sigbits) {
    case 0: // 000 Start with H, long H x 3
        pulseWidth = PULSE_INTERVAL - 1;
        break;

    case 1: // 001 Start with H, long H x 1
        pulseWidth = PULSE_INTERVAL - sampleTS[1];
        break;

    case 2: // 010 Okay
        pulseWidth = sampleTS[1];
        break;

    case 3: // 011 Start with H, short H x 2
        pulseWidth = PULSE_INTERVAL - sampleTS[0];
        break;

    case 4: // 100 Start with L, Long H x 2
        pulseWidth = sampleTS[0];
        break;

    case 5: // 101 Okay
        if (sampleTS[0] + sampleTS[1] < PULSE_LIMIT)
            pulseWidth = sampleTS[0];
        break;

    case 6: // 110 Start with L, short H x 2
        pulseWidth = sampleTS[1];
        break;

    case 7: // 111 Start with L, short H x 3
        pulseWidth = 0;
        break;
    }

    debug[0] = pulseWidth;

    if (pulseWidth > 100) {
        debug[1] = sigbits;
        debug[2] = sampleTS[0];
        debug[3] = sampleTS[1];
    }

    // Monitor pulseWidths for parameter decision.

    if (pulseWidth > tmax)
        tmax = pulseWidth;

    if (pulseWidth < tmin)
        tmin = pulseWidth;

#if 0
    debug[2] = tmin;
    debug[3] = tmax;
#endif
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

    IOInit(intpwmIO, OWNER_INTPWM, 0);
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

void updateIntpwm(uint32_t currentTime)
{
    UNUSED(currentTime);

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
    sampleCount = 0;

    // Mark it busy
    intpwmInProgress = true;

    // Enable interrupt
    EXTIEnable(intpwmIO, true);
}

#endif // USE_INTPWM
