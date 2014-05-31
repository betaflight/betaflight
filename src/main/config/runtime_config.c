#include <stdbool.h>
#include <stdint.h>

#include "config/runtime_config.h"

flags_t f;

// each entry in the array is a bitmask, 3 bits per aux channel (only aux 1 to 4), aux1 is first, each bit corresponds to an rc channel reading
// bit 1 - stick LOW
// bit 2 - stick MIDDLE
// bit 3 - stick HIGH
// an option is enabled when ANY channel has an appropriate reading corresponding to the bit.
// an option is disabled when NO channel has an appropriate reading corresponding to the bit.
// example: 110000000001 - option is only enabled when AUX1 is LOW or AUX4 is MEDIUM or HIGH.
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
