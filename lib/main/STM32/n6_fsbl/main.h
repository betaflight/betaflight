/*
 * STM32N6 FSBL stub — public include hub.
 *
 * Mirrors ST's Template_FSBL_XIP/FSBL/Inc/main.h pattern: the HAL config
 * is the single user-visible header that pulls in the rest of the HAL via
 * stm32n6xx.h.
 */

#pragma once

#include "stm32n6xx_hal.h"

void Error_Handler(void);
