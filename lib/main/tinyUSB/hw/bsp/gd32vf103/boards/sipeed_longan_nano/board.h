#ifndef _NUCLEI_SDK_HAL_H
#define _NUCLEI_SDK_HAL_H

#include "gd32vf103c_longan_nano.h"

// 4 bits for interrupt level, 0 for priority.
// level 0 = lowest priority, level 15 = highest priority.
#define __ECLIC_INTCTLBITS  4

#define __SYSTEM_CLOCK      72000000
#define HXTAL_VALUE         ((uint32_t)8000000)

#define SOC_DEBUG_UART      GD32_COM0

#define DBG_KEY_UNLOCK      0x4B5A6978
#define DBG_CMD_RESET       0x1
#define DBG_KEY             REG32(DBG + 0x0C)
#define DBG_CMD             REG32(DBG + 0x08)

#endif
