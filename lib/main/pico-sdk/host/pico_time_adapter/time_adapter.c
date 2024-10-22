/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/time.h"
#include "pico/time_adapter.h"

PICO_WEAK_FUNCTION_DEF(ta_clear_force_irq)
void PICO_WEAK_FUNCTION_IMPL_NAME(ta_clear_force_irq)(alarm_pool_timer_t *timer, uint hardware_alarm_num) {
    panic_unsupported();
}
PICO_WEAK_FUNCTION_DEF(ta_clear_irq)
void PICO_WEAK_FUNCTION_IMPL_NAME(ta_clear_irq)(alarm_pool_timer_t *timer, uint hardware_alarm_num) {
    panic_unsupported();
}
PICO_WEAK_FUNCTION_DEF(ta_force_irq)
void PICO_WEAK_FUNCTION_IMPL_NAME(ta_force_irq)(alarm_pool_timer_t *timer, uint hardware_alarm_num) {
    panic_unsupported();
}
PICO_WEAK_FUNCTION_DEF(ta_get_handler_hardware_alarm_num)
int PICO_WEAK_FUNCTION_IMPL_NAME(ta_get_handler_hardware_alarm_num)() {
    panic_unsupported();
}
PICO_WEAK_FUNCTION_DEF(ta_set_timeout)
void PICO_WEAK_FUNCTION_IMPL_NAME(ta_set_timeout)(alarm_pool_timer_t *timer, uint hardware_alarm_num, int64_t target) {
    panic_unsupported();
}
PICO_WEAK_FUNCTION_DEF(ta_wakes_up_on_or_before)
void PICO_WEAK_FUNCTION_IMPL_NAME(ta_wakes_up_on_or_before)(alarm_pool_timer_t *timer, uint hardware_alarm_num, int64_t target) {
    panic_unsupported();
}
PICO_WEAK_FUNCTION_DEF(ta_enable_irq_handler)
void PICO_WEAK_FUNCTION_IMPL_NAME(ta_enable_irq_handler)(alarm_pool_timer_t *timer, uint hardware_alarm_num, void (*irq_handler)(void)) {
    panic_unsupported();
}
PICO_WEAK_FUNCTION_DEF(ta_disable_irq_handler)
void PICO_WEAK_FUNCTION_IMPL_NAME(ta_disable_irq_handler)(alarm_pool_timer_t *timer, uint hardware_alarm_num, void (*irq_handler)(void)) {
    panic_unsupported();
}
PICO_WEAK_FUNCTION_DEF(ta_hardware_alarm_claim)
void PICO_WEAK_FUNCTION_IMPL_NAME(ta_hardware_alarm_claim)(alarm_pool_timer_t *timer, uint hardware_alaram_num) {
    panic_unsupported();
}
PICO_WEAK_FUNCTION_DEF(ta_hardware_alarm_claim_unused)
int PICO_WEAK_FUNCTION_IMPL_NAME(ta_hardware_alarm_claim_unused)(alarm_pool_timer_t *timer, bool required) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(ta_from_current_irq);
alarm_pool_timer_t *PICO_WEAK_FUNCTION_IMPL_NAME(ta_from_current_irq)(uint *alarm_num) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(ta_timer_num);
uint ta_timer_num(alarm_pool_timer_t *timer) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(ta_timer_instance);
alarm_pool_timer_t *ta_timer_instance(uint instance_num) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(ta_default_timer_instance);
alarm_pool_timer_t *ta_default_timer_instance(void) {
    panic_unsupported();

}

