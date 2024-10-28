/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/claim.h"

check_hw_layout(timer_hw_t, ints, TIMER_INTS_OFFSET);

static hardware_alarm_callback_t alarm_callbacks[NUM_GENERIC_TIMERS][NUM_ALARMS];
static uint32_t target_hi[NUM_GENERIC_TIMERS][NUM_ALARMS];
static uint8_t timer_callbacks_pending[NUM_GENERIC_TIMERS];

static_assert(NUM_ALARMS * NUM_GENERIC_TIMERS <= 8, "");
static uint8_t claimed[NUM_GENERIC_TIMERS];

void timer_hardware_alarm_claim(timer_hw_t *timer, uint alarm_num) {
    check_hardware_alarm_num_param(alarm_num);
    hw_claim_or_assert(&claimed[timer_get_index(timer)], alarm_num, "Hardware alarm %d already claimed");
}

void hardware_alarm_claim(uint alarm_num) {
    timer_hardware_alarm_claim(PICO_DEFAULT_TIMER_INSTANCE(), alarm_num);
}

void timer_hardware_alarm_unclaim(timer_hw_t *timer, uint alarm_num) {
    check_hardware_alarm_num_param(alarm_num);
    hw_claim_clear(&claimed[timer_get_index(timer)], alarm_num);
}

void hardware_alarm_unclaim(uint alarm_num) {
    timer_hardware_alarm_unclaim(PICO_DEFAULT_TIMER_INSTANCE(), alarm_num);
}

bool timer_hardware_alarm_is_claimed(timer_hw_t *timer, uint alarm_num) {
    check_hardware_alarm_num_param(alarm_num);
    return hw_is_claimed(&claimed[timer_get_index(timer)], alarm_num);
}

bool hardware_alarm_is_claimed(uint alarm_num) {
    return timer_hardware_alarm_is_claimed(PICO_DEFAULT_TIMER_INSTANCE(), alarm_num);
}

int timer_hardware_alarm_claim_unused(timer_hw_t *timer, bool required) {
    return hw_claim_unused_from_range(&claimed[timer_get_index(timer)], required, 0, NUM_ALARMS - 1, "No alarms available");
}

int hardware_alarm_claim_unused(bool required) {
    return timer_hardware_alarm_claim_unused(PICO_DEFAULT_TIMER_INSTANCE(), required);
}

/// tag::time_us_64[]
uint64_t timer_time_us_64(timer_hw_t *timer) {
    // Need to make sure that the upper 32 bits of the timer
    // don't change, so read that first
    uint32_t hi = timer->timerawh;
    uint32_t lo;
    do {
        // Read the lower 32 bits
        lo = timer->timerawl;
        // Now read the upper 32 bits again and
        // check that it hasn't incremented. If it has loop around
        // and read the lower 32 bits again to get an accurate value
        uint32_t next_hi = timer->timerawh;
        if (hi == next_hi) break;
        hi = next_hi;
    } while (true);
    return ((uint64_t) hi << 32u) | lo;
}
/// end::time_us_64[]

/// \tag::busy_wait[]
void timer_busy_wait_us_32(timer_hw_t *timer, uint32_t delay_us) {
    if (0 <= (int32_t)delay_us) {
        // we only allow 31 bits, otherwise we could have a race in the loop below with
        // values very close to 2^32
        uint32_t start = timer->timerawl;
        while (timer->timerawl - start < delay_us) {
            tight_loop_contents();
        }
    } else {
        busy_wait_us(delay_us);
    }
}

void timer_busy_wait_us(timer_hw_t *timer, uint64_t delay_us) {
    uint64_t base = timer_time_us_64(timer);
    uint64_t target = base + delay_us;
    if (target < base) {
        target = (uint64_t)-1;
    }
    absolute_time_t t;
    update_us_since_boot(&t, target);
    timer_busy_wait_until(timer, t);
}

void timer_busy_wait_ms(timer_hw_t *timer, uint32_t delay_ms)
{
    if (delay_ms <= 0x7fffffffu / 1000) {
        timer_busy_wait_us_32(timer, delay_ms * 1000);
    } else {
        timer_busy_wait_us(timer, delay_ms * 1000ull);
    }
}

void timer_busy_wait_until(timer_hw_t *timer, absolute_time_t t) {
    uint64_t target = to_us_since_boot(t);
    uint32_t hi_target = (uint32_t)(target >> 32u);
    uint32_t hi = timer->timerawh;
    while (hi < hi_target) {
        hi = timer->timerawh;
        tight_loop_contents();
    }
    while (hi == hi_target && timer->timerawl < (uint32_t) target) {
        hi = timer->timerawh;
        tight_loop_contents();
    }
}
/// \end::busy_wait[]

uint64_t time_us_64(void) {
    return timer_time_us_64(PICO_DEFAULT_TIMER_INSTANCE());
}

void busy_wait_us_32(uint32_t delay_us) {
    timer_busy_wait_us_32(PICO_DEFAULT_TIMER_INSTANCE(), delay_us);
}

void busy_wait_us(uint64_t delay_us) {
    timer_busy_wait_us(PICO_DEFAULT_TIMER_INSTANCE(), delay_us);
}

void busy_wait_ms(uint32_t delay_ms)
{
    timer_busy_wait_ms(PICO_DEFAULT_TIMER_INSTANCE(), delay_ms);
}

void busy_wait_until(absolute_time_t t) {
    timer_busy_wait_until(PICO_DEFAULT_TIMER_INSTANCE(), t);
}

static void hardware_alarm_irq_handler(void) {
    // Determine which timer this IRQ is for
    uint irq_num = __get_current_exception() - VTABLE_FIRST_IRQ;
    uint alarm_num = TIMER_ALARM_NUM_FROM_IRQ(irq_num);
    check_hardware_alarm_num_param(alarm_num);
    uint timer_num = TIMER_NUM_FROM_IRQ(irq_num);
    timer_hw_t *timer = timer_get_instance(timer_num);
    hardware_alarm_callback_t callback = NULL;

    spin_lock_t *lock = spin_lock_instance(PICO_SPINLOCK_ID_TIMER);
    uint32_t save = spin_lock_blocking(lock);

    // Clear the timer IRQ (inside lock, because we check whether we have handled the IRQ yet in alarm_set by looking at the interrupt status
    timer->intr = 1u << alarm_num;
    // Clear any forced IRQ
    hw_clear_bits(&timer->intf, 1u << alarm_num);

    // make sure the IRQ is still valid
    if (timer_callbacks_pending[timer_num] & (1u << alarm_num)) {
        // Now check whether we have a timer event to handle that isn't already obsolete (this could happen if we
        // were already in the IRQ handler before someone else changed the timer setup
        if (timer->timerawh >= target_hi[timer_num][alarm_num]) {
            // we have reached the right high word as well as low word value
            callback = alarm_callbacks[timer_num][alarm_num];
            timer_callbacks_pending[timer_num] &= (uint8_t)~(1u << alarm_num);
        } else {
            // try again in 2^32 us
            timer->alarm[alarm_num] = timer->alarm[alarm_num]; // re-arm the timer
        }
    }

    spin_unlock(lock, save);

    if (callback) {
        callback(alarm_num);
    }
}

void timer_hardware_alarm_set_callback(timer_hw_t *timer, uint alarm_num, hardware_alarm_callback_t callback) {
    // todo check current core owner
    //  note this should probably be subsumed by irq_set_exclusive_handler anyway, since that
    //  should disallow IRQ handlers on both cores
    check_hardware_alarm_num_param(alarm_num);
    uint timer_num = timer_get_index(timer);
    uint irq_num = TIMER_ALARM_IRQ_NUM(timer, alarm_num);
    spin_lock_t *lock = spin_lock_instance(PICO_SPINLOCK_ID_TIMER);
    uint32_t save = spin_lock_blocking(lock);
    if (callback) {
        if (hardware_alarm_irq_handler != irq_get_vtable_handler(irq_num)) {
            // note that set_exclusive will silently allow you to set the handler to the same thing
            // since it is idempotent, which means we don't need to worry about locking ourselves
            irq_set_exclusive_handler(irq_num, hardware_alarm_irq_handler);
            irq_set_enabled(irq_num, true);
            // Enable interrupt in block and at processor
            hw_set_bits(&timer->inte, 1u << alarm_num);
        }
        alarm_callbacks[timer_num][alarm_num] = callback;
    } else {
        alarm_callbacks[timer_num][alarm_num] = NULL;
        timer_callbacks_pending[timer_num] &= (uint8_t)~(1u << alarm_num);
        irq_remove_handler(irq_num, hardware_alarm_irq_handler);
        irq_set_enabled(irq_num, false);
    }
    spin_unlock(lock, save);
}

void hardware_alarm_set_callback(uint alarm_num, hardware_alarm_callback_t callback) {
    timer_hardware_alarm_set_callback(PICO_DEFAULT_TIMER_INSTANCE(), alarm_num, callback);
}

bool timer_hardware_alarm_set_target(timer_hw_t *timer, uint alarm_num, absolute_time_t target) {
    bool missed;
    uint64_t now = timer_time_us_64(timer);
    uint64_t t = to_us_since_boot(target);
    if (now >= t) {
        missed = true;
    } else {
        missed = false;
        uint timer_num = timer_get_index(timer);

        // 1) actually set the hardware timer
        spin_lock_t *lock = spin_lock_instance(PICO_SPINLOCK_ID_TIMER);
        uint32_t save = spin_lock_blocking(lock);
        uint8_t old_timer_callbacks_pending = timer_callbacks_pending[timer_num];
        timer_callbacks_pending[timer_num] |= (uint8_t)(1u << alarm_num);
        timer->intr = 1u << alarm_num; // clear any IRQ
        timer->alarm[alarm_num] = (uint32_t) t;
        // Set the alarm. Writing time should arm it
        target_hi[timer_num][alarm_num] = (uint32_t)(t >> 32u);

        // 2) check for races
        if (!(timer->armed & 1u << alarm_num)) {
            // not armed, so has already fired .. IRQ must be pending (we are still under lock)
            assert(timer->ints & 1u << alarm_num);
        } else {
            if (timer_time_us_64(timer) >= t) {
                // we are already at or past the right time; there is no point in us racing against the IRQ
                // we are about to generate. note however that, if there was already a timer pending before,
                // then we still let the IRQ fire, as whatever it was, is not handled by our setting missed=true here
                missed = true;
                if (timer_callbacks_pending[timer_num] != old_timer_callbacks_pending) {
                    // disarm the timer
                    timer->armed = 1u << alarm_num;
                    // clear the IRQ...
                    timer->intr = 1u << alarm_num;
                    // ... including anything pending on the processor - perhaps unnecessary, but
                    // our timer flag says we aren't expecting anything.
                    irq_clear(timer_hardware_alarm_get_irq_num(timer, alarm_num));
                    // and clear our flag so that if the IRQ handler is already active (because it is on
                    // the other core) it will also skip doing anything
                    timer_callbacks_pending[timer_num] = old_timer_callbacks_pending;
                }
            }
        }
        spin_unlock(lock, save);
        // note at this point any pending timer IRQ can likely run
    }
    return missed;
}

bool hardware_alarm_set_target(uint alarm_num, absolute_time_t t) {
    return timer_hardware_alarm_set_target(PICO_DEFAULT_TIMER_INSTANCE(), alarm_num, t);
}

void timer_hardware_alarm_cancel(timer_hw_t *timer, uint alarm_num) {
    check_hardware_alarm_num_param(alarm_num);

    spin_lock_t *lock = spin_lock_instance(PICO_SPINLOCK_ID_TIMER);
    uint32_t save = spin_lock_blocking(lock);
    timer->armed = 1u << alarm_num;
    timer_callbacks_pending[timer_get_index(timer)] &= (uint8_t)~(1u << alarm_num);
    spin_unlock(lock, save);
}

void hardware_alarm_cancel(uint alarm_num) {
    timer_hardware_alarm_cancel(PICO_DEFAULT_TIMER_INSTANCE(), alarm_num);
}

void timer_hardware_alarm_force_irq(timer_hw_t *timer, uint alarm_num) {
    check_hardware_alarm_num_param(alarm_num);
    spin_lock_t *lock = spin_lock_instance(PICO_SPINLOCK_ID_TIMER);
    uint32_t save = spin_lock_blocking(lock);
    timer_callbacks_pending[timer_get_index(timer)] |= (uint8_t)(1u << alarm_num);
    spin_unlock(lock, save);
    hw_set_bits(&timer->intf, 1u << alarm_num);
}

void hardware_alarm_force_irq(uint alarm_num) {
    timer_hardware_alarm_force_irq(PICO_DEFAULT_TIMER_INSTANCE(), alarm_num);
}
