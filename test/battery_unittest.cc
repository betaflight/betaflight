#include <stdint.h>

#include <limits.h>
#include "battery.h"

#include "unittest_macros.h"
#include "gtest/gtest.h"

typedef struct batteryAdcToVoltageExpectation_s {
    uint16_t adcReading;
    uint16_t expectedVoltageInDeciVoltSteps;
} batteryAdcToVoltageExpectation_t;

#define ELEVEN_TO_ONE_VOLTAGE_DIVIDER 110 // (10k:1k) * 10 for 0.1V

TEST(BatteryTest, BatteryADCToVoltage)
{
    // given

    batteryConfig_t batteryConfig;
    batteryConfig.vbatscale = ELEVEN_TO_ONE_VOLTAGE_DIVIDER;

    batteryInit(&batteryConfig);

    batteryAdcToVoltageExpectation_t batteryAdcToVoltageExpectations[] = {
            {1420, 125},
            {1430, 126},
            {1440, 127},
            {1890, 167},
            {1900, 168},
            {1910, 169}
    };
    uint8_t testIterationCount = sizeof(batteryAdcToVoltageExpectations) / sizeof(batteryAdcToVoltageExpectation_t);

    // expect

    for (uint8_t index = 0; index < testIterationCount; index ++) {
        batteryAdcToVoltageExpectation_t *batteryAdcToVoltageExpectation = &batteryAdcToVoltageExpectations[index];
        printf("adcReading: %d\n", batteryAdcToVoltageExpectation->adcReading);

        uint16_t pointOneVoltSteps = batteryAdcToVoltage(batteryAdcToVoltageExpectation->adcReading);

        EXPECT_EQ(pointOneVoltSteps, batteryAdcToVoltageExpectation->expectedVoltageInDeciVoltSteps);
    }
}

// STUBS

uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    return 0;
}

void delay(uint32_t ms)
{
    UNUSED(ms);
    return;
}
