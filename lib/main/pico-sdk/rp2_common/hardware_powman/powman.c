/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>

#include "pico.h"

#include "hardware/gpio.h"
#include "hardware/powman.h"

#ifndef PICO_POWMAN_DEBUG
#define PICO_POWMAN_DEBUG 0
#endif

#if PICO_POWMAN_DEBUG
bool powman_debug_printf = false;
void powman_enable_debug_printf(void) {
    powman_debug_printf = true;
}
#define powman_debug(format, args...) if (powman_debug_printf) printf(format, ## args)
#else
#define powman_debug(...)
#endif

static inline void powman_write(volatile uint32_t *reg, uint32_t value) {
    // Write needs a password in top 16 bits
    invalid_params_if(HARDWARE_POWMAN, value >> 16);
    *reg = POWMAN_PASSWORD_BITS | value;
}

void powman_timer_set_ms(uint64_t time_ms) {
    bool was_running = powman_timer_is_running();
    if (was_running) powman_timer_stop();
    powman_write(&powman_hw->set_time_15to0, time_ms & 0xffff);
    powman_write(&powman_hw->set_time_31to16, (time_ms >> 16) & 0xffff);
    powman_write(&powman_hw->set_time_47to32, (time_ms >> 32) & 0xffff);
    powman_write(&powman_hw->set_time_63to48, (time_ms >> 48) & 0xffff);
    if (was_running) powman_timer_start();
}

uint64_t powman_timer_get_ms(void) {
    // Need to make sure that the upper 32 bits of the timer
    // don't change, so read that first
    uint32_t hi = powman_hw->read_time_upper;
    uint32_t lo;
    do {
        // Read the lower 32 bits
        lo = powman_hw->read_time_lower;
        // Now read the upper 32 bits again and
        // check that it hasn't incremented. If it has loop around
        // and read the lower 32 bits again to get an accurate value
        uint32_t next_hi = powman_hw->read_time_upper;
        if (hi == next_hi) break;
        hi = next_hi;
    } while (true);
    return ((uint64_t) hi << 32u) | lo;
}

void powman_timer_set_1khz_tick_source_lposc(void) {
    powman_timer_set_1khz_tick_source_lposc_with_hz(32768);
}

void powman_timer_set_1khz_tick_source_lposc_with_hz(uint32_t lposc_freq_hz) {
    bool was_running = powman_timer_is_running();
    if (was_running) powman_timer_stop();
    uint32_t lposc_freq_khz = lposc_freq_hz / 1000;
    uint32_t lposc_freq_khz_frac16 = (lposc_freq_khz % 1000) * 65536 / 1000;
    powman_write(&powman_hw->lposc_freq_khz_int, lposc_freq_khz);
    powman_write(&powman_hw->lposc_freq_khz_frac, lposc_freq_khz_frac16);
    powman_set_bits(&powman_hw->timer, POWMAN_TIMER_USE_LPOSC_BITS);
    if (was_running) {
        powman_timer_start();
        while(!(powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS));
    }
}

void powman_timer_set_1khz_tick_source_xosc(void) {
    powman_timer_set_1khz_tick_source_xosc_with_hz(XOSC_HZ);
}

void powman_timer_set_1khz_tick_source_xosc_with_hz(uint32_t xosc_freq_hz) {
    bool was_running = powman_timer_is_running();
    if (was_running) powman_timer_stop();
    uint32_t xosc_freq_khz = xosc_freq_hz / 1000;
    uint32_t xosc_freq_khz_frac16 = (xosc_freq_khz % 1000) * 65536 / 1000;
    powman_write(&powman_hw->xosc_freq_khz_int, xosc_freq_khz);
    powman_write(&powman_hw->xosc_freq_khz_frac, xosc_freq_khz_frac16);
    powman_set_bits(&powman_hw->timer, POWMAN_TIMER_USE_XOSC_BITS);
    if (was_running) {
        powman_timer_start();
        while(!(powman_hw->timer & POWMAN_TIMER_USING_XOSC_BITS));
    }
}

static void powman_timer_use_gpio(uint32_t gpio, uint32_t use, uint32_t using) {
    bool was_running = powman_timer_is_running();
    if (was_running) powman_timer_stop();
    invalid_params_if(HARDWARE_POWMAN, !((gpio == 12) || (gpio == 14) || (gpio == 20) || (gpio == 22)));
    gpio_set_input_enabled(gpio, true);
    powman_write(&powman_hw->ext_time_ref, gpio);
    powman_set_bits(&powman_hw->timer, use);
    if (was_running) {
        powman_timer_start();
        while(!(powman_hw->timer & using));
    }
}

void powman_timer_set_1khz_tick_source_gpio(uint32_t gpio) {
    // todo check if we're using the GPIO setup already?
    powman_timer_use_gpio(gpio, POWMAN_TIMER_USE_GPIO_1KHZ_BITS, POWMAN_TIMER_USING_GPIO_1KHZ_BITS);
}

void powman_timer_enable_gpio_1hz_sync(uint32_t gpio) {
    // todo check if we're using the GPIO setup already?
    powman_timer_use_gpio(gpio, POWMAN_TIMER_USE_GPIO_1HZ_BITS, POWMAN_TIMER_USING_GPIO_1HZ_BITS);
}

void powman_timer_disable_gpio_1hz_sync(void) {
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_USE_GPIO_1HZ_BITS);
}

powman_power_state powman_get_power_state(void) {
    uint32_t state_reg = powman_hw->state & POWMAN_STATE_CURRENT_BITS;
    // todo we should have hardware/regs/powman.h values for these
    static_assert(POWMAN_POWER_DOMAIN_SRAM_BANK1 == 0, "");
    static_assert(POWMAN_POWER_DOMAIN_SRAM_BANK0 == 1, "");
    static_assert(POWMAN_POWER_DOMAIN_XIP_CACHE == 2, "");
    static_assert(POWMAN_POWER_DOMAIN_SWITCHED_CORE == 3, "");
    static_assert(POWMAN_STATE_CURRENT_BITS == 0xf, "");
    return (powman_power_state) state_reg;
}

// TODO: Should this fail to go to sleep if there is no wakeup alarm
int powman_set_power_state(powman_power_state state) {
    // Clear req ignored in case it has been set
    powman_clear_bits(&powman_hw->state, POWMAN_STATE_REQ_IGNORED_BITS);
    powman_debug("powman: Requesting state %x\n", state);
    powman_write(&powman_hw->state, (~state << POWMAN_STATE_REQ_LSB) & POWMAN_STATE_REQ_BITS);

    // Has it been ignored?
    if (powman_hw->state & POWMAN_STATE_REQ_IGNORED_BITS) {
        powman_debug("State req ignored because of a pending pwrup req: %"PRIx32"\n", powman_hw->current_pwrup_req);
        return PICO_ERROR_PRECONDITION_NOT_MET;
    }

    bool state_valid = (powman_hw->state & POWMAN_STATE_BAD_SW_REQ_BITS) == 0;
    if (!state_valid) {
        powman_debug("powman: Requested state invalid\n");
        return PICO_ERROR_INVALID_ARG;
    } else {
        powman_debug("powman: Requested state valid\n");
    }
    if (!powman_power_state_is_domain_on(state, POWMAN_POWER_DOMAIN_SWITCHED_CORE)) {
        // If we are turning off switched core then POWMAN_STATE_WAITING_BITS will be
        // set because we are waiting for proc to go to sleep, so return ok and then the proc
        // can go to sleep

        // Note if the powerdown is being blocked by a pending pwrup request we will break out of this and return a failure

        // Clk pow is slow so can take a few clk_pow cycles for waiting to turn up
        for (int i = 0; i < 100; i++) {
            if (powman_hw->state & POWMAN_STATE_WAITING_BITS) {
                return PICO_OK;
            }
        }

        // If it hasn't turned up then false
        powman_debug("powman: STATE_WAITING hasn't turned up\n");
        return PICO_ERROR_TIMEOUT;
    }
    // Wait while the state is changing then return true as we will be in the new state
    powman_debug("powman: waiting for state change\n");
    while(powman_hw->state & POWMAN_STATE_CHANGING_BITS) tight_loop_contents();
    powman_debug("powman: state changed to %x\n", state);
    return PICO_OK;
}

bool powman_configure_wakeup_state(powman_power_state sleep_state, powman_power_state wakeup_state) {
    // When powman wakes up it can keep the state of the sram0 and sram1 banks. Note, it can't
    // explicitly
    bool valid = powman_power_state_is_domain_on(wakeup_state, POWMAN_POWER_DOMAIN_XIP_CACHE);
    valid &= powman_power_state_is_domain_on(wakeup_state, POWMAN_POWER_DOMAIN_SWITCHED_CORE);
    valid &= powman_power_state_is_domain_on(sleep_state, POWMAN_POWER_DOMAIN_SRAM_BANK0) ==
             powman_power_state_is_domain_on(wakeup_state, POWMAN_POWER_DOMAIN_SRAM_BANK0);
    valid &= powman_power_state_is_domain_on(sleep_state, POWMAN_POWER_DOMAIN_SRAM_BANK1) ==
             powman_power_state_is_domain_on(wakeup_state, POWMAN_POWER_DOMAIN_SRAM_BANK1);
    if (valid) {
        powman_clear_bits(&powman_hw->seq_cfg, POWMAN_SEQ_CFG_HW_PWRUP_SRAM0_BITS | POWMAN_SEQ_CFG_HW_PWRUP_SRAM1_BITS);
        uint32_t seq_cfg_set = 0;
        if (!powman_power_state_is_domain_on(sleep_state, POWMAN_POWER_DOMAIN_SRAM_BANK0)) seq_cfg_set |= POWMAN_SEQ_CFG_HW_PWRUP_SRAM0_BITS;
        if (!powman_power_state_is_domain_on(sleep_state, POWMAN_POWER_DOMAIN_SRAM_BANK1)) seq_cfg_set |= POWMAN_SEQ_CFG_HW_PWRUP_SRAM1_BITS;
        powman_set_bits(&powman_hw->seq_cfg, seq_cfg_set);
    }
    return valid;
}

void powman_timer_enable_alarm_at_ms(uint64_t alarm_time_ms) {
    powman_set_bits(&powman_hw->inte, POWMAN_INTE_TIMER_BITS);
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_ENAB_BITS);
    // Alarm must be disabled to set the alarm time
    powman_write(&powman_hw->alarm_time_15to0, alarm_time_ms & 0xffff);
    powman_write(&powman_hw->alarm_time_31to16, (alarm_time_ms >> 16) & 0xffff);
    powman_write(&powman_hw->alarm_time_47to32, (alarm_time_ms >> 32) & 0xffff);
    powman_write(&powman_hw->alarm_time_63to48, (alarm_time_ms >> 48) & 0xffff);
    powman_clear_alarm();
    // TODO: Assuming pwrup on alarm has no bad side effects if already powered up
    powman_set_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_ENAB_BITS);
}

void powman_timer_disable_alarm(void) {
    powman_clear_bits(&powman_hw->inte, POWMAN_INTE_TIMER_BITS);
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_ENAB_BITS);
}

void powman_enable_alarm_wakeup_at_ms(uint64_t alarm_time_ms) {
    powman_timer_enable_alarm_at_ms(alarm_time_ms);
    powman_set_bits(&powman_hw->timer, POWMAN_TIMER_PWRUP_ON_ALARM_BITS);
}

void powman_disable_alarm_wakeup(void) {
    powman_timer_disable_alarm();
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_PWRUP_ON_ALARM_BITS);
}

void powman_enable_gpio_wakeup(uint gpio_wakeup_num, uint32_t gpio, bool edge, bool high) {
    invalid_params_if(HARDWARE_POWMAN, gpio_wakeup_num >= count_of(powman_hw->pwrup));

    // Need to make sure pad is input enabled
    gpio_set_input_enabled(gpio, true);

    // Set up gpio hardware for what we want
    uint32_t pwrup = (edge ? POWMAN_PWRUP0_MODE_VALUE_EDGE : POWMAN_PWRUP0_MODE_VALUE_LEVEL) << POWMAN_PWRUP0_MODE_LSB;
    pwrup |= (high ? POWMAN_PWRUP0_DIRECTION_BITS : 0);
    pwrup |= gpio << POWMAN_PWRUP0_SOURCE_LSB;
    powman_write(&powman_hw->pwrup[gpio_wakeup_num], pwrup);

    // Clear the status bit in case an edge is already latched
    powman_clear_bits(&powman_hw->pwrup[gpio_wakeup_num], POWMAN_PWRUP0_STATUS_BITS);

    // Important to enable it separately to allow the gpio to change
    powman_set_bits(&powman_hw->pwrup[gpio_wakeup_num], POWMAN_PWRUP0_ENABLE_BITS);
}

void powman_disable_gpio_wakeup(uint gpio_wakeup_num) {
    invalid_params_if(HARDWARE_POWMAN, gpio_wakeup_num >= count_of(powman_hw->pwrup));
    powman_clear_bits(&powman_hw->pwrup[gpio_wakeup_num], POWMAN_PWRUP0_ENABLE_BITS);
}

void powman_disable_all_wakeups(void) {
    for (uint i = 0; i < count_of(powman_hw->pwrup); i++) {
        powman_disable_gpio_wakeup(i);
    }
    powman_disable_alarm_wakeup();
}
