/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/irq.h"
#include "hardware/claim.h"

// totally non-functional IRQ

#if PICO_VTABLE_PER_CORE
static uint8_t user_irq_claimed[NUM_CORES];
static inline uint8_t *user_irq_claimed_ptr(void) {
    return &user_irq_claimed[get_core_num()];
}
#else
static uint8_t user_irq_claimed;
static inline uint8_t *user_irq_claimed_ptr(void) {
    return &user_irq_claimed;
}
#endif

PICO_WEAK_FUNCTION_DEF(irq_set_enabled)
void PICO_WEAK_FUNCTION_IMPL_NAME(irq_set_enabled)(uint num, bool enabled) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_is_enabled)
bool PICO_WEAK_FUNCTION_IMPL_NAME(irq_is_enabled)(uint num) {
    return false;
}

PICO_WEAK_FUNCTION_DEF(irq_set_mask_enabled)
void PICO_WEAK_FUNCTION_IMPL_NAME(irq_set_mask_enabled)(uint32_t mask, bool enabled) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_set_mask_n_enabled)
void PICO_WEAK_FUNCTION_IMPL_NAME(irq_set_mask_n_enabled)(uint n, uint32_t mask, bool enabled) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_set_pending)
void PICO_WEAK_FUNCTION_IMPL_NAME(irq_set_pending)(uint num) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_has_shared_handler)
bool PICO_WEAK_FUNCTION_IMPL_NAME(irq_has_shared_handler)(uint irq_num) {
    return false;
}

PICO_WEAK_FUNCTION_DEF(irq_get_vtable_handler)
irq_handler_t PICO_WEAK_FUNCTION_IMPL_NAME(irq_get_vtable_handler)(uint num) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_set_exclusive_handler)
void PICO_WEAK_FUNCTION_IMPL_NAME(irq_set_exclusive_handler)(uint num, irq_handler_t handler) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_get_exclusive_handler)
irq_handler_t PICO_WEAK_FUNCTION_IMPL_NAME(irq_get_exclusive_handler)(uint num) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_add_shared_handler)
void PICO_WEAK_FUNCTION_IMPL_NAME(irq_add_shared_handler)(uint num, irq_handler_t handler, uint8_t order_priority) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_remove_handler)
void PICO_WEAK_FUNCTION_IMPL_NAME(irq_remove_handler)(uint num, irq_handler_t handler) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_set_priority)
void PICO_WEAK_FUNCTION_IMPL_NAME(irq_set_priority)(uint num, uint8_t hardware_priority) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_get_priority)
uint PICO_WEAK_FUNCTION_IMPL_NAME(irq_get_priority)(uint num) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_clear)
void PICO_WEAK_FUNCTION_IMPL_NAME(irq_clear)(uint int_num) {
    panic_unsupported();
}

PICO_WEAK_FUNCTION_DEF(irq_init_priorities)
void PICO_WEAK_FUNCTION_IMPL_NAME(irq_init_priorities)() {
}

static uint get_user_irq_claim_index(uint irq_num) {
    invalid_params_if(HARDWARE_IRQ, irq_num < FIRST_USER_IRQ || irq_num >= NUM_IRQS);
    // we count backwards from the last, to match the existing hard coded uses of user IRQs in the SDK which were previously using 31
    static_assert(NUM_IRQS - FIRST_USER_IRQ <= 8, ""); // we only use a single byte's worth of claim bits today.
    return NUM_IRQS - irq_num  - 1u;
}

PICO_WEAK_FUNCTION_DEF(user_irq_claim)
void PICO_WEAK_FUNCTION_IMPL_NAME(user_irq_claim)(uint irq_num) {
    hw_claim_or_assert(user_irq_claimed_ptr(), get_user_irq_claim_index(irq_num), "User IRQ is already claimed");
}

PICO_WEAK_FUNCTION_DEF(user_irq_unclaim)
void PICO_WEAK_FUNCTION_IMPL_NAME(user_irq_unclaim)(uint irq_num) {
    hw_claim_clear(user_irq_claimed_ptr(), get_user_irq_claim_index(irq_num));
}

PICO_WEAK_FUNCTION_DEF(user_irq_claim_unused)
int PICO_WEAK_FUNCTION_IMPL_NAME(user_irq_claim_unused)(bool required) {
    int bit = hw_claim_unused_from_range(user_irq_claimed_ptr(), required, 0, NUM_USER_IRQS - 1, "No user IRQs are available");
    if (bit >= 0) bit =  (int)NUM_IRQS - bit - 1;
    return bit;
}

PICO_WEAK_FUNCTION_DEF(user_irq_is_claimed)
bool PICO_WEAK_FUNCTION_IMPL_NAME(user_irq_is_claimed)(uint irq_num) {
    return hw_is_claimed(user_irq_claimed_ptr(), get_user_irq_claim_index(irq_num));
}
