/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/asm_helper.S"

#if !HAS_DOUBLE_COPROCESSOR
#error attempt to compile double_aeabi_rp2350 when there is no DCP
#else

#include "hardware/dcp_instr.inc.S"
#include "hardware/dcp_canned.inc.S"

pico_default_asm_setup

.macro double_section name
#if PICO_DOUBLE_IN_RAM
.section RAM_SECTION_NAME(\name), "ax"
#else
.section SECTION_NAME(\name), "ax"
#endif
.endm

.macro double_wrapper_section func
double_section WRAPPER_FUNC_NAME(\func)
.endm

// ============== STATE SAVE AND RESTORE ===============

.macro saving_func type func
  // Note we are usually 32-bit aligned already at this point, as most of the
  // function bodies contain exactly two 16-bit instructions: bmi and bx lr.
  // We want the PCMP word-aligned.
.p2align 2
  // When the engaged flag is set, branch back here to invoke save routine and
  // hook lr with the restore routine, then fall back through to the entry
  // point. The engaged flag will be clear when checked a second time.
1:
  push {lr}              // 16-bit instruction
  bl generic_save_state  // 32-bit instruction
  b 1f                   // 16-bit instruction
  // This is the actual entry point:
\type\()_func \func
  PCMP apsr_nzcv
  bmi 1b
1:
.endm

.macro saving_func_return
  bx lr
.endm

double_section __rp2350_dcp_engaged_state_save_restore
.thumb_func
generic_save_state:
  sub sp, #24
  push {r0, r1}
  // do save here
  PXMD r0, r1
  strd r0, r1, [sp, #8 + 0]
  PYMD r0, r1
  strd r0, r1, [sp, #8 + 8]
  REFD r0, r1
  strd r0, r1, [sp, #8 + 16]
  pop {r0, r1}
  blx lr
  // <- wrapped function returns here
  // fall through into restore:
.thumb_func
generic_restore_state:
  // do restore here
  pop {r12, r14}
  WXMD r12, r14
  pop {r12, r14}
  WYMD r12, r14
  pop {r12, r14}
  WEFD r12, r14
  pop {pc}

// ============== ARITHMETIC FUNCTIONS ===============

double_wrapper_section __aeabi_dadd
saving_func wrapper __aeabi_dadd
  dcp_dadd_m r0,r1,r0,r1,r2,r3
  saving_func_return

double_wrapper_section __aeabi_dsub
saving_func wrapper __aeabi_dsub
  dcp_dsub_m r0,r1,r0,r1,r2,r3
  saving_func_return

double_wrapper_section __aeabi_drsub
saving_func wrapper __aeabi_drsub
  dcp_dsub_m r0,r1,r2,r3,r0,r1
  saving_func_return

double_wrapper_section __aeabi_dmul
saving_func wrapper __aeabi_dmul

 // todo optimize this based on final decision on saving_func_entry
  push {r4,r14}
  dcp_dmul_m r0,r1,r0,r1,r2,r3,r0,r1,r2,r3,r4,r12,r14
 // todo optimize this based on final decision on saving_func_entry
  pop {r4,lr}
  saving_func_return

double_section ddiv_fast
saving_func regular ddiv_fast
  dcp_ddiv_fast_m r0,r1,r0,r1,r2,r3,r0,r1,r2,r3,r12
  saving_func_return

double_wrapper_section __aeabi_ddiv
saving_func wrapper __aeabi_ddiv
@ with correct rounding
  dcp_ddiv_m r0,r1,r0,r1,r2,r3,r0,r1,r2,r3,r12
  saving_func_return

double_section sqrt_fast
saving_func regular sqrt_fast
  dcp_dsqrt_fast_m r0,r1,r0,r1,r0,r1,r2,r3,r12
  saving_func_return

double_wrapper_section sqrt
saving_func wrapper sqrt
@ with correct rounding
  dcp_dsqrt_m r0,r1,r0,r1,r0,r1,r2,r3,r12
  saving_func_return

// todo not a real thing
double_wrapper_section __aeabi_dclassify
saving_func wrapper __aeabi_dclassify
@ with correct rounding
  dcp_dclassify_m apsr_nzcv,r0,r1
  saving_func_return

// ============== CONVERSION FUNCTIONS ===============

double_wrapper_section __aeabi_d2f
saving_func wrapper __aeabi_d2f
@ with rounding
  dcp_double2float_m r0,r0,r1
  saving_func_return

double_wrapper_section __aeabi_i2d
saving_func wrapper __aeabi_i2d
  dcp_int2double_m r0,r1,r0
  saving_func_return

double_wrapper_section __aeabi_ui2d
saving_func wrapper __aeabi_ui2d
  dcp_uint2double_m r0,r1,r0
  saving_func_return

double_wrapper_section __aeabi_d2iz
saving_func wrapper __aeabi_d2iz
@ with truncation towards 0
  dcp_double2int_m r0,r0,r1
  saving_func_return

double_wrapper_section __aeabi_d2uiz
saving_func wrapper __aeabi_d2uiz
@ with truncation towards 0
  dcp_double2uint_m r0,r0,r1
  saving_func_return

// todo not a real thing
double_wrapper_section __aeabi_d2i_r
saving_func wrapper __aeabi_d2i_r
@ with rounding
  dcp_double2int_r_m r0,r0,r1
  saving_func_return

// todo not a real thing
double_wrapper_section __aeabi_d2ui_r
saving_func wrapper __aeabi_d2ui_r
@ with rounding
  dcp_double2uint_r_m r0,r0,r1
  saving_func_return

// ============== COMPARISON FUNCTIONS ===============

double_wrapper_section __aeabi_dcmpun
saving_func wrapper __aeabi_dcmpun
  dcp_dcmp_m r0,r0,r1,r2,r3
  // extract unordered bit
  ubfx r0, r0, #28, #1
  saving_func_return

double_wrapper_section __aeabi_dcmp

saving_func wrapper __aeabi_cdrcmple
  dcp_dcmp_m apsr_nzcv,r2,r3,r0,r1 // with arguments reversed
  bvs cmp_nan
  saving_func_return

// these next two can be the same function in the absence of exceptions
saving_func wrapper __aeabi_cdcmple
//wrapper_func __aeabi_dcmp
  dcp_dcmp_m apsr_nzcv,r0,r1,r2,r3
  bvs cmp_nan
  saving_func_return

// It is not clear from the ABI documentation whether cdcmpeq must set the C flag
// in the same way as cdcmple. If not, we could save the "bvs" below; but we
// err on the side of caution.
saving_func wrapper __aeabi_cdcmpeq
//wrapper_func __aeabi_dcmp
  dcp_dcmp_m apsr_nzcv,r0,r1,r2,r3
  bvs cmp_nan
  saving_func_return

// If the result of a flag-setting comparison is "unordered" then we need to set C and clear Z.
// We could conceivably just do lsrs r12,r14,#1, or even cmp r14,r14,lsr#1 as (a) r14 here is a
// return address and r14b0=1 for Thumb mode; (b) we are unlikely to be returning to address 0.
cmp_nan:
  movs r12, #3 // r12 does not need to be preserved by the flag-setting comparisons
  lsrs r12, #1 // set C, clear Z
  saving_func_return

// int FUNC_NAME(__aeabi_dcmpeq)(double, double)         result (1, 0) denotes (=, ?<>) [2], use for C == and !=
double_wrapper_section __aeabi_dcmpeq
saving_func wrapper __aeabi_dcmpeq
  dcp_dcmp_m r0,r0,r1,r2,r3
  // extract Z
  ubfx r0, r0, #30, #1
  saving_func_return

// int FUNC_NAME(__aeabi_dcmplt)(double, double)         result (1, 0) denotes (<, ?>=) [2], use for C <
double_wrapper_section __aeabi_dcmplt
saving_func wrapper __aeabi_dcmplt
  dcp_dcmp_m apsr_nzcv,r2,r3,r0,r1
  ite hi
  movhi r0,#1
  movls r0,#0
  saving_func_return

// int FUNC_NAME(__aeabi_dcmple)(double, double)         result (1, 0) denotes (<=, ?>) [2], use for C <=
double_wrapper_section __aeabi_dcmple
saving_func wrapper __aeabi_dcmple
  dcp_dcmp_m apsr_nzcv,r2,r3,r0,r1
  ite hs
  movhs r0,#1
  movlo r0,#0
  saving_func_return

// int FUNC_NAME(__aeabi_dcmpge)(double, double)         result (1, 0) denotes (>=, ?<) [2], use for C >=
double_wrapper_section __aeabi_dcmpge
saving_func wrapper __aeabi_dcmpge
  dcp_dcmp_m apsr_nzcv,r0,r1,r2,r3
  ite hs
  movhs r0,#1
  movlo r0,#0
  saving_func_return

// int FUNC_NAME(__aeabi_dcmpgt)(double, double)         result (1, 0) denotes (>, ?<=) [2], use for C >
double_wrapper_section __aeabi_dcmpgt
saving_func wrapper __aeabi_dcmpgt
  dcp_dcmp_m apsr_nzcv,r0,r1,r2,r3
  ite hi
  movhi r0,#1
  movls r0,#0
  saving_func_return

#endif
