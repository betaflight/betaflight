#include <stdbool.h>
#include <stdint.h>

#include "runtime_config.h"

flags_t f;
uint8_t rcOptions[CHECKBOX_ITEM_COUNT];

static uint32_t enabledSensors = 0;

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}
