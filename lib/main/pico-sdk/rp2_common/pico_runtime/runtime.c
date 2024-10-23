/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/runtime.h"
#include "pico/runtime_init.h"


void __weak hard_assertion_failure(void) {
    panic("Hard assert");
}

static void runtime_run_initializers_from(uintptr_t *from) {

    // Start and end points of the constructor list,
    // defined by the linker script.
    extern uintptr_t __preinit_array_end;

    // Call each function in the list, based on the mask
    // We have to take the address of the symbols, as __preinit_array_start *is*
    // the first function value, not the address of it.
    for (uintptr_t *p = from; p < &__preinit_array_end; p++) {
        uintptr_t val = *p;
        ((void (*)(void))val)();
    }
}

void runtime_run_initializers(void) {
    extern uintptr_t __preinit_array_start;
    runtime_run_initializers_from(&__preinit_array_start);
}

// We keep the per-core initializers in the standard __preinit_array so a standard C library
// initialization will force the core 0 initialization, however we also want to be able to find
// them after the fact so that we can run them on core 1. Per core initializers have sections
// __preinit_array.ZZZZZ.nnnnn i.e. the ZZZZZ sorts below all the standard __preinit_array.nnnnn
// values, and then we sort within the ZZZZZ.
//
// We create a dummy initializer in __preinit_array.YYYYY (between the standard initializers
// and the per core initializers), so we find the first per core initializer. Whilst we could
// have done this via an entry in the linker script, we want to preserve backwards compatibility
// with RP2040 custom linker scripts.
static void first_per_core_initializer(void) {}
PICO_RUNTIME_INIT_FUNC(first_per_core_initializer, "YYYYY");

void runtime_run_per_core_initializers(void) {
    runtime_run_initializers_from(&__pre_init_first_per_core_initializer);
}