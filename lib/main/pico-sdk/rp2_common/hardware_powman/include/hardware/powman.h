/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_POWMAN_H
#define _HARDWARE_POWMAN_H

#include "pico.h"
#include "hardware/structs/powman.h"

/** \file hardware/powman.h
 *  \defgroup hardware_powman hardware_powman
 *
 * \brief Power Management API
 *
 */

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_POWMAN, Enable/disable hardware_powman assertions, type=bool, default=0, group=hardware_powman
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_POWMAN
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_POWMAN 0
#endif

/*! \brief Use the ~32KHz low power oscillator as the powman timer source
 *  \ingroup hardware_powman
 */
void powman_timer_set_1khz_tick_source_lposc(void);

/*! \brief Use the low power oscillator (specifying frequency) as the powman timer source
 *  \ingroup hardware_powman
 *  \param lposc_freq_hz specify an exact lposc freq to trim it
 */
void powman_timer_set_1khz_tick_source_lposc_with_hz(uint32_t lposc_freq_hz);

/*! \brief Use the crystal oscillator as the powman timer source
 *  \ingroup hardware_powman
 */
void powman_timer_set_1khz_tick_source_xosc(void);

/*! \brief Use the crystal oscillator as the powman timer source
 *  \ingroup hardware_powman
 *  \param xosc_freq_hz specify a crystal frequency
 */
void powman_timer_set_1khz_tick_source_xosc_with_hz(uint32_t xosc_freq_hz);

/*! \brief Use a 1KHz external tick as the powman timer source
 *  \ingroup hardware_powman
 *  \param gpio the gpio to use. must be 12, 14, 20, 22
 */
void powman_timer_set_1khz_tick_source_gpio(uint32_t gpio);

/*! \brief Use a 1Hz external signal as the powman timer source for seconds only
 *  \ingroup hardware_powman
 *
 * Use a 1hz sync signal, such as from a gps for the seconds component of the timer.
 * The milliseconds will still come from another configured source such as xosc or lposc
 *
 * \param gpio the gpio to use. must be 12, 14, 20, 22
 */
void powman_timer_enable_gpio_1hz_sync(uint32_t gpio);

/*! \brief Stop using 1Hz external signal as the powman timer source for seconds
 *  \ingroup hardware_powman
 */
void powman_timer_disable_gpio_1hz_sync(void);

/*! \brief Returns current time in ms
 *  \ingroup hardware_powman
 */
uint64_t powman_timer_get_ms(void);

/*! \brief Set current time in ms
 *  \ingroup hardware_powman
 *
 * \param time_ms Current time in ms
 */
void powman_timer_set_ms(uint64_t time_ms);

/*! \brief Set an alarm at an absolute time in ms
 *  \ingroup hardware_powman
 *
 * Note, the timer is stopped and then restarted as part of this function. This only controls the alarm
 * if you want to use the alarm to wake up powman then you should use \ref powman_enable_alarm_wakeup_at_ms
 *
 * \param alarm_time_ms time at which the alarm will fire
 */
void powman_timer_enable_alarm_at_ms(uint64_t alarm_time_ms);

/*! \brief Disable the alarm
 *  \ingroup hardware_powman
 *
 * Once an alarm has fired it must be disabled to stop firing as the alarm
 * comparison is alarm = alarm_time >= current_time
 */
void powman_timer_disable_alarm(void);

/*! \brief hw_set_bits helper function
 *  \ingroup hardware_powman
 *
 * \param reg register to set
 * \param bits bits of register to set
 * Powman needs a password for writes, to prevent accidentally writing to it.
 * This function implements hw_set_bits with an appropriate password.
 */
static inline void powman_set_bits(volatile uint32_t *reg, uint32_t bits) {
    invalid_params_if(HARDWARE_POWMAN, bits >> 16);
    hw_set_bits(reg, POWMAN_PASSWORD_BITS | bits);
}

/*! \brief hw_clear_bits helper function
 *  \ingroup hardware_powman
 *
 * Powman needs a password for writes, to prevent accidentally writing to it.
 * This function implements hw_clear_bits with an appropriate password.
 *
 * \param reg register to clear
 * \param bits bits of register to clear
 */
static inline void powman_clear_bits(volatile uint32_t *reg, uint32_t bits) {
    invalid_params_if(HARDWARE_POWMAN, bits >> 16);
    hw_clear_bits(reg, POWMAN_PASSWORD_BITS | bits);
}

/*! \brief Determine if the powman timer is running
 *  \ingroup hardware_powman
 */
static inline bool powman_timer_is_running(void) {
    return powman_hw->timer & POWMAN_TIMER_RUN_BITS;
}

/*! \brief Stop the powman timer
 * \ingroup hardware_powman
 */
static inline void powman_timer_stop(void) {
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_RUN_BITS);
}

/*! \brief Start the powman timer
 * \ingroup hardware_powman
 */
static inline void powman_timer_start(void) {
    powman_set_bits(&powman_hw->timer, POWMAN_TIMER_RUN_BITS);
}

/*! \brief Clears the powman alarm
 * \ingroup hardware_powman
 *
 * Note, the alarm must be disabled (see \ref powman_timer_disable_alarm) before clearing the alarm, as the alarm fires if
 * the time is greater than equal to the target, so once the time has passed the alarm will always fire while enabled.
 */
static inline void powman_clear_alarm(void) {
    powman_clear_bits(&powman_hw->timer, POWMAN_TIMER_ALARM_BITS);
}

/*! \brief Power domains of powman
 *  \ingroup hardware_powman
 */
enum powman_power_domains {
    POWMAN_POWER_DOMAIN_SRAM_BANK1 = 0,    ///< bank1 includes the top 256K of sram plus sram 8 and 9 (scratch x and scratch y)
    POWMAN_POWER_DOMAIN_SRAM_BANK0 = 1,    ///< bank0 is bottom 256K of sSRAM
    POWMAN_POWER_DOMAIN_XIP_CACHE = 2,     ///< XIP cache is 2x8K instances
    POWMAN_POWER_DOMAIN_SWITCHED_CORE = 3, ///< Switched core logic (processors, busfabric, peris etc)
    POWMAN_POWER_DOMAIN_COUNT = 4,
};

typedef uint32_t powman_power_state;

/*! \brief Get the current power state
 *  \ingroup hardware_powman
 */
powman_power_state powman_get_power_state(void);

/*! \brief Set the power state
 * \ingroup hardware_powman
 *
 * Check the desired state is valid. Powman will go to the state if it is valid and there are no pending power up requests.
 *
 * Note that if you are turning off the switched core then this function will never return as the processor will have
 * been turned off at the end.
 *
 * \param state the power state to go to
 * \returns PICO_OK if the state is valid. Misc PICO_ERRORs are returned if not
 */
int powman_set_power_state(powman_power_state state);

#define POWMAN_POWER_STATE_NONE 0

/*! \brief Helper function modify a powman_power_state to turn a domain on
 * \ingroup hardware_powman
 * \param orig original state
 * \param domain domain to turn on
 */
static inline powman_power_state powman_power_state_with_domain_on(powman_power_state orig, enum powman_power_domains domain) {
    invalid_params_if(HARDWARE_POWMAN, domain >= POWMAN_POWER_DOMAIN_COUNT);
    return orig | (1u << domain);
}

/*! \brief Helper function modify a powman_power_state to turn a domain off
 * \ingroup hardware_powman
 * \param orig original state
 * \param domain domain to turn off
 */
static inline powman_power_state powman_power_state_with_domain_off(powman_power_state orig, enum powman_power_domains domain) {
    invalid_params_if(HARDWARE_POWMAN, domain >= POWMAN_POWER_DOMAIN_COUNT);
    return orig &= ~(1u << domain);
}

/*! \brief Helper function to check if a domain is on in a given powman_power_state
 * \ingroup hardware_powman
 * \param state powman_power_state
 * \param domain domain to check is on
 */
static inline bool powman_power_state_is_domain_on(powman_power_state state, enum powman_power_domains domain) {
    invalid_params_if(HARDWARE_POWMAN, domain >= POWMAN_POWER_DOMAIN_COUNT);
    return state & (1u << domain);
}

/*! \brief Wake up from an alarm at a given time
 * \ingroup hardware_powman
 * \param alarm_time_ms time to wake up in ms
 */
void powman_enable_alarm_wakeup_at_ms(uint64_t alarm_time_ms);

/*! \brief Wake up from a gpio
 * \ingroup hardware_powman
 * \param gpio_wakeup_num hardware wakeup instance to use (0-3)
 * \param gpio gpio to wake up from (0-47)
 * \param edge true for edge sensitive, false for level sensitive
 * \param high true for active high, false active low
 */
void powman_enable_gpio_wakeup(uint gpio_wakeup_num, uint32_t gpio, bool edge, bool high);

/*! \brief Disable waking up from alarm
 *  \ingroup hardware_powman
 */
void powman_disable_alarm_wakeup(void);

/*! \brief Disable wake up from a gpio
 * \ingroup hardware_powman
 * \param gpio_wakeup_num hardware wakeup instance to use (0-3)
 */
void powman_disable_gpio_wakeup(uint gpio_wakeup_num);

/*! \brief Disable all wakeup sources
 *  \ingroup hardware_powman
 */
void powman_disable_all_wakeups(void);

/*! \brief Configure sleep state and wakeup state
 * \ingroup hardware_powman
 * \param sleep_state power state powman will go to when sleeping, used to validate the wakeup state
 * \param wakeup_state power state powman will go to when waking up. Note switched core and xip always power up. SRAM bank0 and bank1 can be left powered off
 * \returns true if the state is valid, false if not
 */
bool powman_configure_wakeup_state(powman_power_state sleep_state, powman_power_state wakeup_state);

/*! \brief Ignore wake up when the debugger is attached
 *  \ingroup hardware_powman
 *
 * Typically, when a debugger is attached it will assert the pwrupreq signal. OpenOCD does not clear this signal, even when you quit.
 * This means once you have attached a debugger powman will never go to sleep. This function lets you ignore the debugger
 * pwrupreq which means you can go to sleep with a debugger attached. The debugger will error out if you go to turn off the switch core with it attached,
 * as the processors have been powered off.
 *
 * \param ignored should the debugger power up request be ignored
 */
static inline void powman_set_debug_power_request_ignored(bool ignored) {
    if (ignored)
        powman_set_bits(&powman_hw->dbg_pwrcfg, 1);
    else
        powman_clear_bits(&powman_hw->dbg_pwrcfg, 0);
}

#endif