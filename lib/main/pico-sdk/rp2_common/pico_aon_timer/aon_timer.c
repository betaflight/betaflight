/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/aon_timer.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

static aon_timer_alarm_handler_t aon_timer_alarm_handler;

#if HAS_RP2040_RTC
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#elif HAS_POWMAN_TIMER
#include "hardware/powman.h"

static void powman_timer_irq_handler(void) {
    uint irq_num = aon_timer_get_irq_num();
    // we are one-shot, so remove ourselves
    irq_set_enabled(irq_num, false);
    irq_remove_handler(irq_num, powman_timer_irq_handler);
    if (aon_timer_alarm_handler) aon_timer_alarm_handler();
}
#endif

void aon_timer_set_time(const struct timespec *ts) {
#if HAS_RP2040_RTC
    datetime_t dt;
    bool ok = time_to_datetime(ts->tv_sec, &dt);
    assert(ok);
    if (ok) rtc_set_datetime(&dt);
#elif HAS_POWMAN_TIMER
    powman_timer_set_ms(timespec_to_ms(ts));
#else
    panic_unsupported();
#endif
}

void aon_timer_get_time(struct timespec *ts) {
#if HAS_RP2040_RTC
    datetime_t dt;
    rtc_get_datetime(&dt);
    time_t t;
    bool ok = datetime_to_time(&dt, &t);
    assert(ok);
    ts->tv_nsec = 0;
    if (ok) {
        ts->tv_sec = t;
    } else {
        ts->tv_sec = -1;
    }
#elif HAS_POWMAN_TIMER
    ms_to_timespec(powman_timer_get_ms(), ts);
#else
    panic_unsupported();
#endif
}

aon_timer_alarm_handler_t aon_timer_enable_alarm(const struct timespec *ts, aon_timer_alarm_handler_t handler, bool wakeup_from_low_power) {
    uint32_t save = save_and_disable_interrupts();
    aon_timer_alarm_handler_t old_handler = aon_timer_alarm_handler;
    struct timespec ts_adjusted = *ts;
#if HAS_RP2040_RTC
    ((void)wakeup_from_low_power); // don't have a choice
    datetime_t dt;
    // adjust to after the target time
    if (ts_adjusted.tv_nsec) ts_adjusted.tv_sec++;
    bool ok = time_to_datetime(ts_adjusted.tv_sec, &dt);
    assert(ok);
    if (ok) {
        rtc_set_alarm(&dt, handler);
    }
#elif HAS_POWMAN_TIMER
    uint irq_num = aon_timer_get_irq_num();
    powman_timer_disable_alarm();
    // adjust to after the target time
    ts_adjusted.tv_nsec += 999999;
    if (ts_adjusted.tv_nsec > 1000000000) {
       ts_adjusted.tv_nsec -= 1000000000;
       ts_adjusted.tv_sec++;
    }
    if (ts_adjusted.tv_nsec) ts_adjusted.tv_sec++;
    if (wakeup_from_low_power) {
        powman_enable_alarm_wakeup_at_ms(timespec_to_ms(ts));
    } else {
        powman_disable_alarm_wakeup();
        powman_timer_enable_alarm_at_ms(timespec_to_ms(ts));
    }
    if (handler) {
        irq_set_exclusive_handler(irq_num, powman_timer_irq_handler);
        irq_set_enabled(irq_num, true);
    }
#else
    panic_unsupported();
#endif
    aon_timer_alarm_handler = handler;
    restore_interrupts_from_disabled(save);
    return old_handler;
}

void aon_timer_disable_alarm(void) {
    irq_set_enabled(aon_timer_get_irq_num(), false);
#if HAS_RP2040_RTC
    rtc_disable_alarm();
#elif HAS_POWMAN_TIMER
    powman_timer_disable_alarm();
#else
    panic_unsupported();
#endif
}

void aon_timer_start_with_timeofday(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct timespec ts;
    ts.tv_sec = tv.tv_sec;
    ts.tv_nsec = tv.tv_usec * 1000;
    aon_timer_start(&ts);
}

void aon_timer_start(const struct timespec *ts) {
#if HAS_RP2040_RTC
    rtc_init();
    aon_timer_set_time(ts);
#elif HAS_POWMAN_TIMER
    // todo how best to allow different configurations; this should just be the default
    powman_timer_set_1khz_tick_source_xosc();
    powman_timer_set_ms(timespec_to_ms(ts));
    powman_timer_start();
#else
    panic_unsupported();
#endif
}

void aon_timer_stop(void) {
#if HAS_RP2040_RTC
    hw_clear_bits(&rtc_hw->ctrl, RTC_CTRL_RTC_ENABLE_BITS);
#elif HAS_POWMAN_TIMER
    powman_timer_stop();
#else
    panic_unsupported();
#endif
}

void aon_timer_get_resolution(struct timespec *ts) {
#if HAS_RP2040_RTC
    ts->tv_sec = 1;
    ts->tv_nsec = 0;
#elif HAS_POWMAN_TIMER
    ts->tv_sec = 0;
    ts->tv_nsec = 1000000000 / 1000;
#else
    panic_unsupported();
#endif
}

bool aon_timer_is_running(void) {
#if HAS_RP2040_RTC
    return rtc_running();
#elif HAS_POWMAN_TIMER
    return powman_timer_is_running();
#else
    panic_unsupported();
#endif
}
