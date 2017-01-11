#include <stdbool.h>
#include <stdint.h>

#include "drivers/io_pca9685.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "config/feature.h"

#define PWM_DRIVER_IMPLEMENTATION_COUNT 1
#define PWM_DRIVER_MAX_CYCLE 4

static bool driverEnabled = false;
static uint8_t driverImplementationIndex = 0;

typedef struct {
    bool (*initFunction)(void);
    void (*writeFunction)(uint8_t servoIndex, uint16_t off);
    void (*setFrequencyFunction)(uint16_t freq);
    void (*syncFunction)(uint8_t cycleIndex);
} pwmDriverDriver_t;

pwmDriverDriver_t pwmDrivers[PWM_DRIVER_IMPLEMENTATION_COUNT] = {
    [0] = {
        .initFunction = pca9685Initialize,
        .writeFunction = pca9685setServoPulse,
        .setFrequencyFunction = pca9685setPWMFreq,
        .syncFunction = pca9685sync
    }
};

bool isPwmDriverEnabled() {
    return driverEnabled;
}

void pwmDriverSetPulse(uint8_t servoIndex, uint16_t length) {
    (pwmDrivers[driverImplementationIndex].writeFunction)(servoIndex, length);
}

void pwmDriverInitialize(void) {
    driverEnabled = (pwmDrivers[driverImplementationIndex].initFunction)();

    if (driverEnabled) {
        ENABLE_STATE(PWM_DRIVER_AVAILABLE);
    } else {
        DISABLE_STATE(PWM_DRIVER_AVAILABLE);
    }

}

void pwmDriverSync(void) {
    static uint8_t cycle = 0;

    (pwmDrivers[driverImplementationIndex].syncFunction)(cycle);

    cycle++;
    if (cycle == PWM_DRIVER_MAX_CYCLE) {
        cycle = 0;
    }
}
