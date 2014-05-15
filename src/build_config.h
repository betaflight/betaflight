#pragma once

#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

//#define SOFT_I2C // enable to test software i2c

#ifndef __CC_ARM
#define REQUIRE_CC_ARM_PRINTF_SUPPORT
#define REQUIRE_PRINTF_LONG_SUPPORT
#endif
