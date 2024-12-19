/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/ticks.h"

void tick_start(tick_gen_num_t tick, uint cycles) {
    valid_params_if(HARDWARE_TICKS, tick < TICK_COUNT);
#if PICO_RP2040
    ((void)tick);
    cycles <<= WATCHDOG_TICK_CYCLES_LSB;
    valid_params_if(HARDWARE_TICKS, cycles <= WATCHDOG_TICK_CYCLES_BITS);
    // On RP2040, this also provides a tick reference to the timer and SysTick
    watchdog_hw->tick = cycles | WATCHDOG_TICK_ENABLE_BITS;
#else
    cycles <<= TICKS_WATCHDOG_CYCLES_LSB;
    valid_params_if(HARDWARE_TICKS, cycles <= TICKS_WATCHDOG_CYCLES_BITS);
    // On later hardware, separate tick generators for every tick destination.
    ticks_hw->ticks[tick].cycles = cycles;
    ticks_hw->ticks[tick].ctrl = TICKS_WATCHDOG_CTRL_ENABLE_BITS;
#endif
}

void tick_stop(tick_gen_num_t tick) {
    valid_params_if(HARDWARE_TICKS, tick < TICK_COUNT);
#if PICO_RP2040
    ((void)tick);
    hw_clear_bits(&watchdog_hw->tick, WATCHDOG_TICK_ENABLE_BITS);
#else
    hw_clear_bits(&ticks_hw->ticks[tick].ctrl, TICKS_WATCHDOG_CTRL_ENABLE_BITS);
#endif
}

bool tick_is_running(tick_gen_num_t tick) {
    valid_params_if(HARDWARE_TICKS, tick < TICK_COUNT);
#if PICO_RP2040
    ((void)tick);
    return watchdog_hw->tick & WATCHDOG_TICK_ENABLE_BITS;
#else
    // On later hardware, separate tick generators for every tick destination.
    return ticks_hw->ticks[tick].ctrl & TICKS_WATCHDOG_CTRL_RUNNING_BITS;
#endif
}