#include "pico/asm_helper.S"

// Support for breaking out individual RISC-V exception causes to handlers
// implemented as normal C functions. Note the handler is still responsible
// for incrementing `mepc` before return, if it intends to return to the
// instruction after the one that caused the exception.

.macro decl_isr name
.weak \name
\name:
.endm

// must be in RAM due to branches from vector table
.section .time_critical.hardware_exception

.p2align 2
.global __riscv_exception_table
__riscv_exception_table:
.word isr_riscv_machine_instr_align_exception
.word isr_riscv_machine_instr_fault_exception
.word isr_riscv_machine_instr_illegal_exception
.word isr_riscv_machine_ebreak_exception
.word isr_riscv_machine_load_align_exception
.word isr_riscv_machine_load_fault_exception
.word isr_riscv_machine_store_align_exception
.word isr_riscv_machine_store_fault_exception
.word isr_riscv_machine_ecall_umode_exception
.word isr_riscv_machine_ecall_smode_exception
.word __unhandled_exception // reserved
.word isr_riscv_machine_ecall_mmode_exception

// mscratch = 0 outside of exception handler. mscratch = user ra during
// exception handler. Assume 0 is not a valid ra.
.global isr_riscv_machine_exception
// still allow override just in case hardware_exception is pulled in by a library
// note: that when LIX_HARDWARE_EXCEPTION=1, crt0_riscv.S does not define its own weak method
.weak isr_riscv_machine_exception
isr_riscv_machine_exception:
    csrrw ra, mscratch, ra
    bnez ra, __nested_exception
    // Exception handler runs on foreground stack: this may fault, but we will
    // catch the fault and go to __nested_exception.
    addi sp, sp, -64
    // Work downward, to ensure that after tripping a stack guard PMP region
    // we re-trip it before trashing memory below the guard.
    sw t6, 60(sp)
    sw t5, 56(sp)
    sw t4, 52(sp)
    sw t3, 48(sp)
    sw a7, 44(sp)
    sw a6, 40(sp)
    sw a5, 36(sp)
    sw a4, 32(sp)
    sw a3, 28(sp)
    sw a2, 24(sp)
    sw a1, 20(sp)
    sw a0, 16(sp)
    sw t2, 12(sp)
    sw t1,  8(sp)
    sw t0,  4(sp)
    // ra already saved

    // Using unsigned comparison for double-ended bounds check
    csrr ra, mcause
    li t6, 11 // XCAUSE_ECALL_M
    bltu t6, ra, __unhandled_exception

    // Enter exception through table
    la t6, __riscv_exception_table
    sh2add ra, ra, t6
    lw ra, (ra)
    jalr ra, ra

    // Restore saved registers
    lw t6, 60(sp)
    lw t5, 56(sp)
    lw t4, 52(sp)
    lw t3, 48(sp)
    lw a7, 44(sp)
    lw a6, 40(sp)
    lw a5, 36(sp)
    lw a4, 32(sp)
    lw a3, 28(sp)
    lw a2, 24(sp)
    lw a1, 20(sp)
    lw a0, 16(sp)
    lw t2, 12(sp)
    lw t1,  8(sp)
    lw t0,  4(sp)
    // ra restored from mscratch
    addi sp, sp, 64

    // Restore mscratch to 0 to avoid tripping next exception
    csrrw ra, mscratch, zero
    mret

decl_isr isr_riscv_machine_instr_align_exception
decl_isr isr_riscv_machine_instr_fault_exception
decl_isr isr_riscv_machine_instr_illegal_exception
decl_isr isr_riscv_machine_ebreak_exception
decl_isr isr_riscv_machine_load_align_exception
decl_isr isr_riscv_machine_load_fault_exception
decl_isr isr_riscv_machine_store_align_exception
decl_isr isr_riscv_machine_store_fault_exception
decl_isr isr_riscv_machine_ecall_umode_exception
decl_isr isr_riscv_machine_ecall_smode_exception
decl_isr isr_riscv_machine_ecall_mmode_exception
    // fall through

// Reach here when executing an exception that did not have a non-default
// handler assigned. Since a breakpoint will cause another exception if the
// debugger is not connected, we can't have an ebreak here. Just spin the
// core forever. You can check `mcause` and `mepc` to see what happened and
// where.
.global __unhandled_exception
__unhandled_exception:
    // Restore original registers and stack pointer so debugger can backtrace
    csrr ra, mscratch
    lw t6, 60(sp)
    addi sp, sp, -64
    // Second symbol here just to make it clearer in the debugger why you got
    // here; the entry point can appear labelled with the name of any one of the
    // unhandled exceptions, which is less clear.
.global __halt_on_unhandled_exception
__halt_on_unhandled_exception:
1:
    j 1b

// Detected an exception occurring whilst running an exception handler. State
// of original exception was trashed by new exception, so this is not
// recoverable. Best we can do is to halt now to avoid further trashing.
__nested_exception:
1:
    j 1b

