/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico.h"
#include "hardware/irq.h"
#include "pico/asm_helper.S"

pico_default_asm_setup

#if !PICO_DISABLE_SHARED_IRQ_HANDLERS

.data
.align 2

.global irq_handler_chain_slots

.global irq_handler_chain_first_slot
.global irq_handler_chain_remove_tail

//
// These Slots make up the code and structure of the handler chains; the only external information are the VTABLE entries
// (obviously one set per core) and a free list head. Each individual handler chain starts with the VTABLE entry I
// pointing at the address of slot S (with thumb bit set). Thus each slot which is part of a chain is executable.
//
// The execution jumps (via branch instruction) from one slot to the other, then jumps to the end of chain handler.
// The entirety of the state needed to traverse the chain is contained within the slots of the chain, which is why
// a VTABLE entry is all that is needed per chain (rather than requiring a separarte set of head pointers)
//

irq_handler_chain_slots:
.set next_slot_number, 1
.rept PICO_MAX_SHARED_IRQ_HANDLERS
    // a slot is executable and is always 3 instructions long.
#ifndef __riscv
    .hword 0    // inst1 (either: ldr r0, [pc, #4]           or for the FIRST slot:  add r1, pc, #0                      )
    .hword 0    // inst2 (        blx r0                                             b irq_handler_chain_first_slot      )

    .hword 0    // inst3 (either: b next_slot                or for the LAST         pop {pc}                            )
#else
    .word  0    // inst1 (either: lui  ra, %hi(handler)      or for the FIRST slot:  jal t0, irq_handler_chain_first_slot)
    .word  0    // inst2 (either: jalr ra. %lo(handler)(ra)                          .word handler                       )

    .hword 0    // inst3 (either: j next_slot                or for the LAST slot:   cm.popret {ra}, 16                  )
#endif

    // next is a single byte index of next slot in chain (or -1 to end)
.if next_slot_number == PICO_MAX_SHARED_IRQ_HANDLERS
    .byte 0xff
.else
    .byte next_slot_number
.endif
    // next is the 8 bit unsigned priority
    .byte 0x00
1:
    // and finally the handler function pointer for Arm:
#ifndef __riscv
    .word 0x0000000
#endif
    .set next_slot_number, next_slot_number + 1
.endr

irq_handler_chain_first_slot:
#ifndef __riscv
    push {r0, lr}     // Save EXC_RETURN token, so `pop {r0, pc}` will return from interrupt
                      // Note that r0 does not NEED to be saved, however we must maintain
                      // an 8 byte stack alignment, and this is the cheapest way to do so
    ldr  r0, [r1, #4] // Get `handler` field of irq_handler_chain_slot
    adds r1, #1       // r1 points to `inst3` field of slot struct. Set Thumb bit on r1,
    mov  lr, r1       // and copy to lr, so `inst3` is executed on return from handler
    bx   r0           // Enter handler
#else
    .insn 0xb842      // cm.push {ra}, -16: Save ultimate return address
    add ra, t0, 4     // Set up function call to return to offset 8 of the slot
    lw t0, (t0)       // Load pointer from offset 4 of the slot
    jr t0             // Call it, with our calculated return address
#endif

irq_handler_chain_remove_tail:
#ifndef __riscv
    mov  r0, lr       // Get start of struct. This function was called by a bl at offset +4,
    subs r0, #9       // so lr points to offset +8. Note also lr has its Thumb bit set!
    ldr  r1, =irq_add_tail_to_free_list
    blx  r1
    pop  {r0, pc}     // Top of stack is EXC_RETURN
#else
    add a0, ra, -10   // Expect to be called with a 16-bit jal, at 8-byte offset in the slot.
    call irq_add_tail_to_free_list
    .insn 0xbe42      // cm.popret {ra}, 16
#endif

#endif
