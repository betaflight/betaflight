/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_TIMER_H
#define _HARDWARE_TIMER_H

#include "pico.h"

#ifdef __cplusplus
extern "C" {
#endif

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_TIMER, Enable/disable assertions in the hardware_timer module, type=bool, default=0, group=hardware_timer
#ifndef PARAM_ASSERTIONS_ENABLED_HARDWARE_TIMER
#ifdef PARAM_ASSERTIONS_ENABLED_TIMER // backwards compatibility with SDK < 2.0.0
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_TIMER PARAM_ASSERTIONS_ENABLED_TIMER
#else
#define PARAM_ASSERTIONS_ENABLED_HARDWARE_TIMER 0
#endif
#endif

static inline void check_hardware_alarm_num_param(uint alarm_num) {
    invalid_params_if(HARDWARE_TIMER, alarm_num >= NUM_ALARMS);
}

uint32_t time_us_32();
uint64_t time_us_64();
void busy_wait_us_32(uint32_t delay_us);
void busy_wait_us(uint64_t delay_us);
void busy_wait_ms(uint32_t delay_m);
void busy_wait_until(absolute_time_t t);
bool time_reached(absolute_time_t t);
typedef void (*hardware_alarm_callback_t)(uint alarm_num);
void hardware_alarm_claim(uint alarm_num);
void hardware_alarm_unclaim(uint alarm_num);
int hardware_alarm_claim_unused(bool required);
void hardware_alarm_set_callback(uint alarm_num, hardware_alarm_callback_t callback);
bool hardware_alarm_set_target(uint alarm_num, absolute_time_t t);
void hardware_alarm_cancel(uint alarm_num);
void hardware_alarm_force_irq(uint alarm_num);
#ifdef __cplusplus
}
#endif

#endif
