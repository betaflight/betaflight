/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/divider.h"

#if PICO_EMULATE_DIVIDER
divmod_result_t hw_divider_results[NUM_CORES];
#endif