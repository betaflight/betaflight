/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include "pico.h"
#include "pico/time.h"
#include "pico/sync.h"
#include "pico/runtime_init.h"

const absolute_time_t ABSOLUTE_TIME_INITIALIZED_VAR(nil_time, 0);
const absolute_time_t ABSOLUTE_TIME_INITIALIZED_VAR(at_the_end_of_time, INT64_MAX);

typedef struct alarm_pool_entry {
    // next entry link or -1
    int16_t next;
    // low 15 bits are a sequence number used in the low word of the alarm_id so that
    // the alarm_id for this entry only repeats every 32767 adds (note this value is never zero)
    // the top bit is a cancellation flag.
    volatile uint16_t sequence;
    int64_t target;
    alarm_callback_t callback;
    void *user_data;
} alarm_pool_entry_t;

struct alarm_pool {
    uint8_t timer_alarm_num;
    uint8_t core_num;
    // this is protected by the lock (threads allocate from it, and the IRQ handler adds back to it)
    int16_t free_head;
    // this is protected by the lock (threads add to it, the IRQ handler removes from it)
    volatile int16_t new_head;
    volatile bool has_pending_cancellations;

    // this is owned by the IRQ handler so doesn't need additional locking
    int16_t ordered_head;
    uint16_t num_entries;
    alarm_pool_timer_t *timer;
    spin_lock_t *lock;
    alarm_pool_entry_t *entries;
};

#if !PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
// To avoid bringing in calloc, we statically allocate the arrays and the heap
static alarm_pool_entry_t default_alarm_pool_entries[PICO_TIME_DEFAULT_ALARM_POOL_MAX_TIMERS];

static alarm_pool_t default_alarm_pool = {
        .entries = default_alarm_pool_entries,
};

static inline bool default_alarm_pool_initialized(void) {
    return default_alarm_pool.lock != NULL;
}

static lock_core_t sleep_notifier;
#endif

#include "pico/time_adapter.h"

static alarm_pool_t *pools[TA_NUM_TIMERS][TA_NUM_TIMER_ALARMS];

static void alarm_pool_post_alloc_init(alarm_pool_t *pool, alarm_pool_timer_t *timer, uint hardware_alarm_num, uint max_timers);

static inline int16_t alarm_index(alarm_id_t id) {
    return (int16_t)(id >> 16);
}

static inline uint16_t alarm_sequence(alarm_id_t id) {
    return (uint16_t)id;
}

static alarm_id_t make_alarm_id(int index, uint16_t counter) {
    return index << 16 | counter;
}

#if !PICO_RUNTIME_NO_INIT_DEFAULT_ALARM_POOL
void __weak runtime_init_default_alarm_pool(void) {
#if !PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
    // allow multiple calls for ease of use from host tests
    if (!default_alarm_pool_initialized()) {
        alarm_pool_timer_t *timer = alarm_pool_get_default_timer();
        ta_hardware_alarm_claim(timer, PICO_TIME_DEFAULT_ALARM_POOL_HARDWARE_ALARM_NUM);
        alarm_pool_post_alloc_init(&default_alarm_pool,
                                   timer,
                                   PICO_TIME_DEFAULT_ALARM_POOL_HARDWARE_ALARM_NUM,
                                   PICO_TIME_DEFAULT_ALARM_POOL_MAX_TIMERS);
    }
    lock_init(&sleep_notifier, PICO_SPINLOCK_ID_TIMER);
#endif
}
#endif

void alarm_pool_init_default(void) {
    runtime_init_default_alarm_pool();
}

#if !PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
alarm_pool_t *alarm_pool_get_default(void) {
    assert(default_alarm_pool_initialized());
    return &default_alarm_pool;
}

#if defined(PICO_RUNTIME_INIT_DEFAULT_ALARM_POOL) && !PICO_RUNTIME_SKIP_INIT_DEFAULT_ALARM_POOL
PICO_RUNTIME_INIT_FUNC_RUNTIME(runtime_init_default_alarm_pool, PICO_RUNTIME_INIT_DEFAULT_ALARM_POOL);
#endif
#endif

// note the timer is created with IRQs on this core
alarm_pool_t *alarm_pool_create_on_timer(alarm_pool_timer_t *timer, uint hardware_alarm_num, uint max_timers) {
    alarm_pool_t *pool = (alarm_pool_t *) malloc(sizeof(alarm_pool_t));
    if (pool) {
        pool->entries = (alarm_pool_entry_t *) calloc(max_timers, sizeof(alarm_pool_entry_t));
        ta_hardware_alarm_claim(timer, hardware_alarm_num);
        alarm_pool_post_alloc_init(pool, timer, hardware_alarm_num, max_timers);
    }
    return pool;
}

alarm_pool_t *alarm_pool_create_on_timer_with_unused_hardware_alarm(alarm_pool_timer_t *timer, uint max_timers) {
    alarm_pool_t *pool = (alarm_pool_t *) malloc(sizeof(alarm_pool_t));
    if (pool) {
        pool->entries = (alarm_pool_entry_t *) calloc(max_timers, sizeof(alarm_pool_entry_t));
        alarm_pool_post_alloc_init(pool, timer, (uint) ta_hardware_alarm_claim_unused(timer, true), max_timers);
    }
    return pool;
}

static void alarm_pool_irq_handler(void);

// marker which we can use in place of handler function to indicate we are a repeating timer

#define repeating_timer_marker ((alarm_callback_t)alarm_pool_irq_handler)
#include "hardware/gpio.h"
static void alarm_pool_irq_handler(void) {
    // This IRQ handler does the main work, as it always (assuming the IRQ hasn't been enabled on both cores
    // which is unsupported) run on the alarm pool's core, and can't be preempted by itself, meaning
    // that it doesn't need locks except to protect against linked list, or other state access.
    // This simplifies the code considerably, and makes it much faster in general, even though we are forced to take
    // two IRQs per alarm.
    uint timer_alarm_num;
    alarm_pool_timer_t *timer = ta_from_current_irq(&timer_alarm_num);
    uint timer_num = ta_timer_num(timer);
    alarm_pool_t *pool = pools[timer_num][timer_alarm_num];
    assert(pool->timer_alarm_num == timer_alarm_num);
    int64_t earliest_target;
    // 1. clear force bits if we were forced (do this outside the loop, as forcing is hopefully rare)
    ta_clear_force_irq(timer, timer_alarm_num);
    do {
        // 2. clear the IRQ if it was fired
        ta_clear_irq(timer, timer_alarm_num);
        // 3. we look at the earliest existing alarm first; the reasoning here is that we
        //    don't want to delay an existing callback because a later one is added, and
        //    if both are due now, then we have a race anyway (but we prefer to fire existing
        //    timers before new ones anyway.
        int16_t earliest_index = pool->ordered_head;
        // by default, we loop if there was any event pending (we will mark it false
        // later if there is no work to do)
        if (earliest_index >= 0) {
            alarm_pool_entry_t *earliest_entry = &pool->entries[earliest_index];
            earliest_target = earliest_entry->target;
            if (((int64_t)ta_time_us_64(timer) - earliest_target) >= 0) {
                // time to call the callback now (or in the past)
                // note that an entry->target of < 0 means the entry has been canceled (not this is set
                // by this function, in response to the entry having been queued by the cancel_alarm API
                // meaning that we don't need to worry about tearing of the 64 bit value)
                int64_t delta;
                if (earliest_target >= 0) {
                    // special case repeating timer without making another function call which adds overhead
                    if (earliest_entry->callback == repeating_timer_marker) {
                        repeating_timer_t *rpt = (repeating_timer_t *)earliest_entry->user_data;
                        delta = rpt->callback(rpt) ? rpt->delay_us : 0;
                    } else {
                        alarm_id_t id = make_alarm_id(pool->ordered_head, earliest_entry->sequence);
                        delta = earliest_entry->callback(id, earliest_entry->user_data);
                    }
                } else {
                    // negative target means cancel alarm
                    delta = 0;
                }
                if (delta) {
                    int64_t next_time;
                    if (delta < 0) {
                        // delta is (positive) delta from last fire time
                        next_time = earliest_target - delta;
                    } else {
                        // delta is relative to now
                        next_time = (int64_t) ta_time_us_64(timer) + delta;
                    }
                    earliest_entry->target = next_time;
                    // need to re-add, unless we are the only entry or already at the front
                    if (earliest_entry->next >= 0 && next_time - pool->entries[earliest_entry->next].target >= 0) {
                        // unlink this item
                        pool->ordered_head = earliest_entry->next;
                        int16_t *prev = &pool->ordered_head;
                        // find insertion point; note >= as if we add a new item for the same time as another, then it follows
                        while (*prev >= 0 && (next_time - pool->entries[*prev].target) >= 0) {
                            prev = &pool->entries[*prev].next;
                        }
                        earliest_entry->next = *prev;
                        *prev = earliest_index;
                    }
                } else {
                    // need to remove the item
                    pool->ordered_head = earliest_entry->next;
                    // and add it back to the free list (under lock)
                    uint32_t save = spin_lock_blocking(pool->lock);
                    earliest_entry->next = pool->free_head;
                    pool->free_head = earliest_index;
                    spin_unlock(pool->lock, save);
                }
            }
        }
        // if we have any new alarms, add them to the ordered list
        if (pool->new_head >= 0) {
            uint32_t save = spin_lock_blocking(pool->lock);
            // must re-read new head under lock
            int16_t new_index = pool->new_head;
            // clear the list
            pool->new_head = -1;
            spin_unlock(pool->lock, save);
            // insert each of the new items
            while (new_index >= 0) {
                alarm_pool_entry_t *new_entry = &pool->entries[new_index];
                int64_t new_entry_time = new_entry->target;
                int16_t *prev = &pool->ordered_head;
                // find insertion point; note >= as if we add a new item for the same time as another, then it follows
                while (*prev >= 0 && (new_entry_time - pool->entries[*prev].target) >= 0) {
                    prev = &pool->entries[*prev].next;
                }
                int16_t next = *prev;
                *prev = new_index;
                new_index = new_entry->next;
                new_entry->next = next;
            }
        }
        // if we have any canceled alarms, then mark them for removal by setting their due time to -1 (which will
        // cause them to be handled the next time round and removed)
        if (pool->has_pending_cancellations) {
            pool->has_pending_cancellations = false;
            __compiler_memory_barrier();
            int16_t *prev = &pool->ordered_head;
            // set target for canceled items to -1, and move to front of the list
            for(int16_t index = pool->ordered_head; index != -1; ) {
                alarm_pool_entry_t *entry = &pool->entries[index];
                int16_t next = entry->next;
                if ((int16_t)entry->sequence < 0) {
                    // mark for deletion
                    entry->target = -1;
                    if (index != pool->ordered_head) {
                        // move to start of queue
                        *prev = entry->next;
                        entry->next = pool->ordered_head;
                        pool->ordered_head = index;
                    }
                } else {
                    prev = &entry->next;
                }
                index = next;
            }
        }
        earliest_index = pool->ordered_head;
        if (earliest_index < 0) break;
        // need to wait
        alarm_pool_entry_t *earliest_entry = &pool->entries[earliest_index];
        earliest_target = earliest_entry->target;
        // we are leaving a timeout every 2^32 microseconds anyway if there is no valid target, so we can choose any value.
        // best_effort_wfe_or_timeout now relies on it being the last value set, and arguably this is the
        // best value anyway, as it is the furthest away from the last fire.
        if (earliest_target != -1) { // cancelled alarm has target of -1
            ta_set_timeout(timer, timer_alarm_num, earliest_target);
        }
        // check we haven't now passed the target time; if not we don't want to loop again
    } while ((earliest_target - (int64_t)ta_time_us_64(timer)) <= 0);
    // We always want the timer IRQ to wake a WFE so that best_effort_wfe_or_timeout() will wake up. It will wake
    // a WFE on its own core by nature of having taken an IRQ, but we do an explicit SEV so it wakes the other core
    __sev();
}

void alarm_pool_post_alloc_init(alarm_pool_t *pool, alarm_pool_timer_t *timer, uint hardware_alarm_num, uint max_timers) {
    pool->timer = timer;
    pool->lock = spin_lock_instance(next_striped_spin_lock_num());
    pool->timer_alarm_num = (uint8_t) hardware_alarm_num;
    invalid_params_if(PICO_TIME, max_timers > 65536);
    pool->num_entries = (uint16_t)max_timers;
    pool->core_num = (uint8_t) get_core_num();
    pool->new_head = pool->ordered_head = -1;
    pool->free_head = (int16_t)(max_timers - 1);
    for(uint i=0;i<max_timers;i++) {
        pool->entries[i].next = (int16_t)(i-1);
    }
    pools[ta_timer_num(timer)][hardware_alarm_num] = pool;

    ta_enable_irq_handler(timer, hardware_alarm_num, alarm_pool_irq_handler);
}

void alarm_pool_destroy(alarm_pool_t *pool) {
#if !PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
    if (pool == &default_alarm_pool) {
        assert(false); // attempt to delete default alarm pool
        return;
    }
#endif
    ta_disable_irq_handler(pool->timer, pool->timer_alarm_num, alarm_pool_irq_handler);
    assert(pools[ta_timer_num(pool->timer)][pool->timer_alarm_num] == pool);
    pools[ta_timer_num(pool->timer)][pool->timer_alarm_num] = NULL;
    free(pool->entries);
    free(pool);
}

alarm_id_t alarm_pool_add_alarm_at(alarm_pool_t *pool, absolute_time_t time, alarm_callback_t callback,
                                   void *user_data, bool fire_if_past) {
    if (!fire_if_past) {
        absolute_time_t t = get_absolute_time();
        if (absolute_time_diff_us(t, time) < 0) return 0;
    }
    return alarm_pool_add_alarm_at_force_in_context(pool, time, callback, user_data);
}

alarm_id_t alarm_pool_add_alarm_at_force_in_context(alarm_pool_t *pool, absolute_time_t time, alarm_callback_t callback,
                                                    void *user_data) {
    // ---- take a free pool entry
    uint32_t save = spin_lock_blocking(pool->lock);
    int16_t index = pool->free_head;
    alarm_pool_entry_t *entry = &pool->entries[index];
    if (index >= 0) {
        // remove from free list
        pool->free_head = entry->next;
    }
    spin_unlock(pool->lock, save);
    if (index < 0) return PICO_ERROR_GENERIC; // PICO_ERROR_INSUFFICIENT_RESOURCES - not using to preserve previous -1 return code

    // ---- initialize the pool entry
    entry->callback = callback;
    entry->user_data = user_data;
    entry->target = (int64_t)to_us_since_boot(time);
    uint16_t next_sequence = (entry->sequence + 1) & 0x7fff;
    if (!next_sequence) next_sequence = 1; // zero is not allowed
    entry->sequence = next_sequence;
    alarm_id_t id = make_alarm_id(index, next_sequence);

    // ---- and add it to the new list
    save = spin_lock_blocking(pool->lock);
    entry->next = pool->new_head;
    pool->new_head = index;
    spin_unlock(pool->lock, save);

    // force the IRQ
    ta_force_irq(pool->timer, pool->timer_alarm_num);
    return id;
}

bool alarm_pool_cancel_alarm(alarm_pool_t *pool, alarm_id_t alarm_id) {
    int16_t index = alarm_index(alarm_id);
    if (index >= pool->num_entries) return false;
    uint16_t sequence = alarm_sequence(alarm_id);
    bool canceled = false;
    alarm_pool_entry_t *entry = &pool->entries[index];
    uint32_t save = spin_lock_blocking(pool->lock);
    // note this will not be true if the entry is already canceled (as the entry->sequence
    // will have the top bit set)
    uint current_sequence = entry->sequence;
    if (sequence == current_sequence) {
        entry->sequence = (uint16_t)(current_sequence | 0x8000);
        __compiler_memory_barrier();
        pool->has_pending_cancellations = true;
        canceled = true;
    }
    spin_unlock(pool->lock, save);
    // force the IRQ if we need to clean up an alarm id
    if (canceled) ta_force_irq(pool->timer, pool->timer_alarm_num);
    return canceled;
}

uint alarm_pool_timer_alarm_num(alarm_pool_t *pool) {
    return pool->timer_alarm_num;
}

uint alarm_pool_core_num(alarm_pool_t *pool) {
    return pool->core_num;
}

#if !PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
static int64_t sleep_until_callback(__unused alarm_id_t id, __unused void *user_data) {
    uint32_t save = spin_lock_blocking(sleep_notifier.spin_lock);
    lock_internal_spin_unlock_with_notify(&sleep_notifier, save);
    return 0;
}
#endif

void sleep_until(absolute_time_t t) {
#if PICO_ON_DEVICE && !defined(NDEBUG)
    if (__get_current_exception()) {
        panic("Attempted to sleep inside of an exception handler; use busy_wait if you must");
    }
#endif
#if !PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
    uint64_t t_us = to_us_since_boot(t);
    uint64_t t_before_us = t_us - PICO_TIME_SLEEP_OVERHEAD_ADJUST_US;
    // needs to work in the first PICO_TIME_SLEEP_OVERHEAD_ADJUST_US of boot
    if (t_before_us > t_us) t_before_us = 0;
    absolute_time_t t_before;
    update_us_since_boot(&t_before, t_before_us);
    if (absolute_time_diff_us(get_absolute_time(), t_before) > 0) {
        if (add_alarm_at(t_before, sleep_until_callback, NULL, false) >= 0) {
            // able to add alarm for just before the time
            while (!time_reached(t_before)) {
                uint32_t save = spin_lock_blocking(sleep_notifier.spin_lock);
                lock_internal_spin_unlock_with_wait(&sleep_notifier, save);
            }
        }
    }
#else
    // hook in case we're in RTOS; note we assume using the alarm pool is better always if available.
    sync_internal_yield_until_before(t);
#endif
    // now wait until the exact time
    busy_wait_until(t);
}

void sleep_us(uint64_t us) {
#if !PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
    sleep_until(make_timeout_time_us(us));
#else
    if (us < PICO_TIME_SLEEP_OVERHEAD_ADJUST_US) {
        busy_wait_us(us);
    } else {
        // hook in case we're in RTOS; note we assume using the alarm pool is better always if available.
        absolute_time_t t = make_timeout_time_us(us - PICO_TIME_SLEEP_OVERHEAD_ADJUST_US);
        sync_internal_yield_until_before(t);

        // then wait the rest of the way
        busy_wait_until(t);
    }
#endif
}

void sleep_ms(uint32_t ms) {
    sleep_us(ms * 1000ull);
}

bool best_effort_wfe_or_timeout(absolute_time_t timeout_timestamp) {
#if !PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
    if (__get_current_exception()) {
        tight_loop_contents();
        return time_reached(timeout_timestamp);
    } else {
        alarm_id_t id;
        // note that as of SDK 2.0.0 calling add_alarm_at always causes a SEV. What we really
        // want to do is cause an IRQ at the specified time in the future if there is not
        // an IRQ already happening before then. The problem is that the IRQ may be happening on the
        // other core, so taking an IRQ is the only way to get the state protection.
        //
        // Therefore, we make a compromise; we will set the alarm, if we won't wake up before the right time
        // already. This means that repeated calls to this function with the same timeout will work correctly
        // after the first one! This is fine, because we ask callers to use a polling loop on another
        // event variable when using this function.
        //
        // For this to work, we require that once we have set an alarm, an SEV happens no later than that, even
        // if we cancel the alarm as we do below. Therefore, the IRQ handler (which is always enabled) will
        // never set its wakeup time to a later value, but instead wake up once and then wake up again.
        //
        // This overhead when canceling alarms is a small price to pay for the much simpler/faster/cleaner
        // implementation that relies on the IRQ handler (on a single core) being the only state accessor.
        //
        // Note also, that the use of software spin locks on RP2350 to access state would always cause a SEV
        // due to use of LDREX etc., so actually using spin locks to protect the state would be worse.
        if (ta_wakes_up_on_or_before(alarm_pool_get_default()->timer, alarm_pool_get_default()->timer_alarm_num,
                                     (int64_t)to_us_since_boot(timeout_timestamp))) {
            // we already are waking up at or before when we want to (possibly due to us having been called
            // before in a loop), so we can do an actual WFE. Note we rely on the fact that the alarm pool IRQ
            // handler always does an explicit SEV, since it may be on the other core.
            __wfe();
            return time_reached(timeout_timestamp);
        } else {
            id = add_alarm_at(timeout_timestamp, sleep_until_callback, NULL, false);
            if (id <= 0) {
                tight_loop_contents();
                return time_reached(timeout_timestamp);
            } else {
                if (!time_reached(timeout_timestamp)) {
                    // ^ at the point above the timer hadn't fired, so it is safe
                    // to wait; the event will happen due to IRQ at some point between
                    // then and the correct wakeup time
                    __wfe();
                }
                // we need to clean up if it wasn't us that caused the wfe; if it was this will be a noop.
                cancel_alarm(id);
                return time_reached(timeout_timestamp);
            }
        }
    }
#else
    tight_loop_contents();
    return time_reached(timeout_timestamp);
#endif
}

bool alarm_pool_add_repeating_timer_us(alarm_pool_t *pool, int64_t delay_us, repeating_timer_callback_t callback, void *user_data, repeating_timer_t *out) {
    if (!delay_us) delay_us = 1;
    out->pool = pool;
    out->callback = callback;
    out->delay_us = delay_us;
    out->user_data = user_data;
    out->alarm_id = alarm_pool_add_alarm_at(pool, make_timeout_time_us((uint64_t)(delay_us >= 0 ? delay_us : -delay_us)),
                                            repeating_timer_marker, out, true);
    return out->alarm_id > 0;
}

bool cancel_repeating_timer(repeating_timer_t *timer) {
    bool rc = false;
    if (timer->alarm_id) {
        rc = alarm_pool_cancel_alarm(timer->pool, timer->alarm_id);
        timer->alarm_id = 0;
    }
    return rc;
}

alarm_pool_timer_t *alarm_pool_timer_for_timer_num(uint timer_num) {
    return ta_timer_instance(timer_num);
}

alarm_pool_timer_t *alarm_pool_get_default_timer(void) {
    return ta_default_timer_instance();
}

int64_t alarm_pool_remaining_alarm_time_us(alarm_pool_t *pool, alarm_id_t alarm_id) {
    // note there is no point distinguishing between invalid alarm_id and timer passed,
    // since an alarm_id that has fired without being re-enabled becomes logically invalid after
    // that point anyway
    int64_t rc = -1;
    int16_t index = alarm_index(alarm_id);
    if ((uint16_t)index < pool->num_entries) {
        uint16_t sequence = alarm_sequence(alarm_id);
        alarm_pool_entry_t *entry = &pool->entries[index];
        if (entry->sequence == sequence) {
            uint32_t save = spin_lock_blocking(pool->lock);
            int16_t search_index = pool->ordered_head;
            while (search_index >= 0) {
                entry = &pool->entries[search_index];
                if (index == search_index) {
                    if (entry->sequence == sequence) {
                        rc = entry->target - (int64_t) ta_time_us_64(pool->timer);
                    }
                    break;
                }
                search_index = entry->next;
            }
            spin_unlock(pool->lock, save);
        }
    }
    return rc;
}

int32_t alarm_pool_remaining_alarm_time_ms(alarm_pool_t *pool, alarm_id_t alarm_id) {
    int64_t rc = alarm_pool_remaining_alarm_time_us(pool, alarm_id);
    if (rc >= 0) rc /= 1000;
    return rc >= INT32_MAX ? INT32_MAX : (int32_t) rc;
}

#if !PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
int64_t remaining_alarm_time_us(alarm_id_t alarm_id) {
    return alarm_pool_remaining_alarm_time_us(alarm_pool_get_default(), alarm_id);
}

int32_t remaining_alarm_time_ms(alarm_id_t alarm_id) {
    return alarm_pool_remaining_alarm_time_ms(alarm_pool_get_default(), alarm_id);
}
#endif