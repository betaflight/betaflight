#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "build_config.h"

#include "gpio_common.h"
#include "system_common.h"

#include "bus_i2c.h"

#ifndef SOFT_I2C

void i2cInit(I2C_TypeDef *I2C)
{
    // FIXME implement
}

uint16_t i2cGetErrorCounter(void)
{
    // FIXME implement
    return 0;
}

bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data)
{
    // FIXME implement
    return false;
}

bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    // FIXME implement
    return false;
}

#endif
