/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "pico/runtime_init.h"
#ifdef __riscv
#include "hardware/riscv.h"
#else
#include "hardware/structs/scb.h"
#endif
#include "hardware/structs/sio.h"
#include "hardware/regs/psm.h"
#include "hardware/claim.h"

#if !PICO_RP2040
#ifndef __riscv
#include "hardware/structs/m33.h"
#endif
#endif

// note that these are not reset by core reset, however for now, I think people resetting cores
// and then relying on multicore_lockout for that core without re-initializing, is probably
// something we can live with breaking.
//
// whilst we could clear this in core 1 reset path, that doesn't necessarily catch all,
// and means pulling in this array even if multicore_lockout is not used.
static bool lockout_victim_initialized[NUM_CORES];

void multicore_fifo_push_blocking(uint32_t data) {
    multicore_fifo_push_blocking_inline(data);
}

bool multicore_fifo_push_timeout_us(uint32_t data, uint64_t timeout_us) {
    absolute_time_t end_time = make_timeout_time_us(timeout_us);
    // We wait for the fifo to have some space
    while (!multicore_fifo_wready()) {
        tight_loop_contents();
        if (time_reached(end_time)) return false;
    }

    sio_hw->fifo_wr = data;

    // Fire off an event to the other core
    __sev();
    return true;
}

uint32_t multicore_fifo_pop_blocking(void) {
    return multicore_fifo_pop_blocking_inline();
}

bool multicore_fifo_pop_timeout_us(uint64_t timeout_us, uint32_t *out) {
    absolute_time_t end_time = make_timeout_time_us(timeout_us);
    // If nothing there yet, we wait for an event first,
    // to try and avoid too much busy waiting
    while (!multicore_fifo_rvalid()) {
        if (best_effort_wfe_or_timeout(end_time)) return false;
    }

    *out = sio_hw->fifo_rd;
    return true;
}

// Default stack for core1 ... if multicore_launch_core1 is not included then .stack1 section will be garbage collected
static uint32_t __attribute__((section(".stack1"))) core1_stack[PICO_CORE1_STACK_SIZE / sizeof(uint32_t)];

static void __attribute__ ((naked)) core1_trampoline(void) {
#ifdef __riscv
    // Do not add function calls here, because we want to preserve the return
    // address pointing back to the bootrom launch routine.
    pico_default_asm(
        "lw a0, 0(sp)\n"
        "lw a1, 4(sp)\n"
        "lw a2, 8(sp)\n"
        "lw gp, 12(sp)\n"
        "addi sp, sp, 16\n"
        "jr a2\n"
    );
#else
    pico_default_asm("pop {r0, r1, pc}");
#endif
}

int core1_wrapper(int (*entry)(void), void *stack_base) {
#if !PICO_RUNTIME_SKIP_INIT_PER_CORE_INSTALL_STACK_GUARD
    // install core1 stack guard
    runtime_init_per_core_install_stack_guard(stack_base);
#else
    (void)stack_base;
#endif
    runtime_run_per_core_initializers();
    return (*entry)();
}

void multicore_reset_core1(void) {
    // Use atomic aliases just in case core 1 is also manipulating some PSM state
    io_rw_32 *power_off = (io_rw_32 *) (PSM_BASE + PSM_FRCE_OFF_OFFSET);
    io_rw_32 *power_off_set = hw_set_alias(power_off);
    io_rw_32 *power_off_clr = hw_clear_alias(power_off);

    // Hard-reset core 1.
    // Reading back confirms the core 1 reset is in the correct state, but also
    // forces APB IO bridges to fence on any internal store buffering
    *power_off_set = PSM_FRCE_OFF_PROC1_BITS;
    while (!(*power_off & PSM_FRCE_OFF_PROC1_BITS)) tight_loop_contents();

    // Allow for the fact that the caller may have already enabled the FIFO IRQ for their
    // own purposes (expecting FIFO content after core 1 is launched). We must disable
    // the IRQ during the handshake, then restore afterward
    uint irq_num = SIO_FIFO_IRQ_NUM(0);
    bool enabled = irq_is_enabled(irq_num);
    irq_set_enabled(irq_num, false);

    // Bring core 1 back out of reset. It will drain its own mailbox FIFO, then push
    // a 0 to our mailbox to tell us it has done this.
    *power_off_clr = PSM_FRCE_OFF_PROC1_BITS;

    // check the pushed value
    uint32_t value = multicore_fifo_pop_blocking();
    assert(value == 0);
    (void) value; // silence warning

    // restore interrupt state
    irq_set_enabled(irq_num, enabled);
}

void multicore_launch_core1_with_stack(void (*entry)(void), uint32_t *stack_bottom, size_t stack_size_bytes) {
    assert(!(stack_size_bytes & 3u));
    uint32_t *stack_ptr = stack_bottom + stack_size_bytes / sizeof(uint32_t);
    // Push values onto top of stack for core1_trampoline
#ifdef __riscv
    // On RISC-V we also need to initialise the global pointer
    stack_ptr -= 4;
    uint32_t vector_table = riscv_read_csr(mtvec);
    asm volatile ("mv %0, gp" : "=r"(stack_ptr[3]));
#else
    stack_ptr -= 3;
    uint32_t vector_table = scb_hw->vtor;
#endif
    stack_ptr[0] = (uintptr_t) entry;
    stack_ptr[1] = (uintptr_t) stack_bottom;
    stack_ptr[2] = (uintptr_t) core1_wrapper;
#if PICO_VTABLE_PER_CORE
    #warning PICO_VTABLE_PER_CORE==1 is not currently supported in pico_multicore
    panic_unsupported();
#endif
    multicore_launch_core1_raw(core1_trampoline, stack_ptr, vector_table);
}

void multicore_launch_core1(void (*entry)(void)) {
    extern uint32_t __StackOneBottom;
    uint32_t *stack_limit = (uint32_t *) &__StackOneBottom;
    // hack to reference core1_stack although that pointer is wrong.... core1_stack should always be <= stack_limit, if not boom!
    uint32_t *stack = core1_stack <= stack_limit ? stack_limit : (uint32_t *) -1;
    multicore_launch_core1_with_stack(entry, stack, sizeof(core1_stack));
}

void multicore_launch_core1_raw(void (*entry)(void), uint32_t *sp, uint32_t vector_table) {
    // Allow for the fact that the caller may have already enabled the FIFO IRQ for their
    // own purposes (expecting FIFO content after core 1 is launched). We must disable
    // the IRQ during the handshake, then restore afterwards.
    uint irq_num = SIO_FIFO_IRQ_NUM(0);
    bool enabled = irq_is_enabled(irq_num);
    irq_set_enabled(irq_num, false);

    // Values to be sent in order over the FIFO from core 0 to core 1
    //
    // vector_table is value for VTOR register
    // sp is initial stack pointer (SP)
    // entry is the initial program counter (PC) (don't forget to set the thumb bit!)
    const uint32_t cmd_sequence[] =
            {0, 0, 1, (uintptr_t) vector_table, (uintptr_t) sp, (uintptr_t) entry};

    uint seq = 0;
    do {
        uint cmd = cmd_sequence[seq];
        // Always drain the READ FIFO (from core 1) before sending a 0
        if (!cmd) {
            multicore_fifo_drain();
            // Execute a SEV as core 1 may be waiting for FIFO space via WFE
            __sev();
        }
        multicore_fifo_push_blocking(cmd);
        uint32_t response = multicore_fifo_pop_blocking();
        // Move to next state on correct response (echo-d value) otherwise start over
        seq = cmd == response ? seq + 1 : 0;
    } while (seq < count_of(cmd_sequence));

    irq_set_enabled(irq_num, enabled);
}

#define LOCKOUT_MAGIC_START 0x73a8831eu
#define LOCKOUT_MAGIC_END (~LOCKOUT_MAGIC_START)

static mutex_t lockout_mutex;
static bool lockout_in_progress;

// note this method is in RAM because lockout is used when writing to flash
// it only makes inline calls
static void __isr __not_in_flash_func(multicore_lockout_handler)(void) {
    multicore_fifo_clear_irq();
    while (multicore_fifo_rvalid()) {
        if (sio_hw->fifo_rd == LOCKOUT_MAGIC_START) {
            uint32_t save = save_and_disable_interrupts();
            multicore_fifo_push_blocking_inline(LOCKOUT_MAGIC_START);
            while (multicore_fifo_pop_blocking_inline() != LOCKOUT_MAGIC_END) {
                tight_loop_contents(); // not tight but endless potentially
            }
            restore_interrupts_from_disabled(save);
            multicore_fifo_push_blocking_inline(LOCKOUT_MAGIC_END);
        }
    }
}

static void check_lockout_mutex_init(void) {
    // use known available lock - we only need it briefly
    uint32_t save = hw_claim_lock();
    if (!mutex_is_initialized(&lockout_mutex)) {
        mutex_init(&lockout_mutex);
    }
    hw_claim_unlock(save);
}

void multicore_lockout_victim_init(void) {
    check_lockout_mutex_init();
    // On platforms other than RP2040, these are actually the same IRQ number
    // (each core only sees its own IRQ, always at the same IRQ number).
    uint core_num = get_core_num();
    uint fifo_irq_this_core = SIO_FIFO_IRQ_NUM(core_num);
    irq_set_exclusive_handler(fifo_irq_this_core, multicore_lockout_handler);
    irq_set_enabled(fifo_irq_this_core, true);
    lockout_victim_initialized[core_num] = true;
}

static bool multicore_lockout_handshake(uint32_t magic, absolute_time_t until) {
    uint irq_num = SIO_FIFO_IRQ_NUM(get_core_num());
    bool enabled = irq_is_enabled(irq_num);
    if (enabled) irq_set_enabled(irq_num, false);
    bool rc = false;
    do {
        int64_t next_timeout_us = absolute_time_diff_us(get_absolute_time(), until);
        if (next_timeout_us < 0) {
            break;
        }
        multicore_fifo_push_timeout_us(magic, (uint64_t)next_timeout_us);
        next_timeout_us = absolute_time_diff_us(get_absolute_time(), until);
        if (next_timeout_us < 0) {
            break;
        }
        uint32_t word = 0;
        if (!multicore_fifo_pop_timeout_us((uint64_t)next_timeout_us, &word)) {
            break;
        }
        if (word == magic) {
            rc = true;
        }
    } while (!rc);
    if (enabled) irq_set_enabled(irq_num, true);
    return rc;
}

static bool multicore_lockout_start_block_until(absolute_time_t until) {
    check_lockout_mutex_init();
    if (!mutex_enter_block_until(&lockout_mutex, until)) {
        return false;
    }
    hard_assert(!lockout_in_progress);
    bool rc = multicore_lockout_handshake(LOCKOUT_MAGIC_START, until);
    lockout_in_progress = rc;
    mutex_exit(&lockout_mutex);
    return rc;
}

bool multicore_lockout_start_timeout_us(uint64_t timeout_us) {
    return multicore_lockout_start_block_until(make_timeout_time_us(timeout_us));
}

void multicore_lockout_start_blocking(void) {
    multicore_lockout_start_block_until(at_the_end_of_time);
}

static bool multicore_lockout_end_block_until(absolute_time_t until) {
    assert(mutex_is_initialized(&lockout_mutex));
    if (!mutex_enter_block_until(&lockout_mutex, until)) {
        return false;
    }
    assert(lockout_in_progress);
    bool rc = multicore_lockout_handshake(LOCKOUT_MAGIC_END, until);
    if (rc) {
        lockout_in_progress = false;
    }
    mutex_exit(&lockout_mutex);
    return rc;
}

bool multicore_lockout_end_timeout_us(uint64_t timeout_us) {
    return multicore_lockout_end_block_until(make_timeout_time_us(timeout_us));
}

void multicore_lockout_end_blocking(void) {
    multicore_lockout_end_block_until(at_the_end_of_time);
}

bool multicore_lockout_victim_is_initialized(uint core_num) {
    return lockout_victim_initialized[core_num];
}

#if NUM_DOORBELLS

static uint8_t doorbell_claimed[NUM_CORES][(NUM_DOORBELLS + 7) >> 3];

static inline bool is_bit_claimed(const uint8_t *bits, uint bit_index) {
    return (bits[bit_index >> 3u] & (1u << (bit_index & 7u)));
}

static inline void set_claimed_bit(uint8_t *bits, uint bit_index) {
    bits[bit_index >> 3u] |= ( uint8_t ) ( 1u << ( bit_index & 7u ));
}

static inline void clear_claimed_bit(uint8_t *bits, uint bit_index) {
    bits[bit_index >> 3u] &= ( uint8_t ) ~( 1u << ( bit_index & 7u ));
}

static bool multicore_doorbell_claim_under_lock(uint doorbell_num, uint core_mask, bool required) {
    static_assert(NUM_CORES == 2, "");
    uint claimed_cores_for_doorbell = (uint) (is_bit_claimed(doorbell_claimed[0], doorbell_num) |
                                              (is_bit_claimed(doorbell_claimed[1], doorbell_num + 1u) << 1));
    if (claimed_cores_for_doorbell & core_mask) {
        if (required) {
            panic( "Multicoore doorbell %d already claimed on core mask 0x%x; requested core mask 0x%x\n",
                   claimed_cores_for_doorbell, core_mask);
        }
        return false;
    } else {
        for(uint i=0; i<NUM_CORES; i++) {
            if (core_mask & (1u << i)) {
                set_claimed_bit(doorbell_claimed[i], doorbell_num);
            }
        }
        return true;
    }
}

void multicore_doorbell_claim(uint doorbell_num, uint core_mask) {
    check_doorbell_num_param(doorbell_num);
    uint32_t save = hw_claim_lock();
    multicore_doorbell_claim_under_lock(doorbell_num, core_mask, true);
    hw_claim_unlock(save);
}

int multicore_doorbell_claim_unused(uint core_mask, bool required) {
    int rc = PICO_ERROR_INSUFFICIENT_RESOURCES;
    uint32_t save = hw_claim_lock();
    for(int i=NUM_DOORBELLS-1; i>=0; i--) {
        if (multicore_doorbell_claim_under_lock((uint) i, core_mask, false)) {
            rc = i;
            break;
        }
    }
    if (required && rc < 0) {
        panic("No free doorbells");
    }
    hw_claim_unlock(save);
    return rc;
}

void multicore_doorbell_unclaim(uint doorbell_num, uint core_mask) {
    check_doorbell_num_param(doorbell_num);
    uint32_t save = hw_claim_lock();
    for(uint i=0; i < NUM_CORES; i++) {
        if (core_mask & (1u << i)) {
            assert(is_bit_claimed(doorbell_claimed[i], doorbell_num));
            clear_claimed_bit(doorbell_claimed[i], doorbell_num);
        }
    }
    hw_claim_unlock(save);
}


#endif