/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/asm_helper.S"
#if HAS_DOUBLE_COPROCESSOR

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

double_wrapper_section conv_tod

@ convert int64 to double, rounding
wrapper_func __aeabi_l2d
regular_func int642double
 movs r2,#0       @ fall through
@ convert unsigned 64-bit fix to double, rounding; number of r0:r1 bits after point in r2
regular_func fix642double
 cmp r1,#0
 bge 10f @ positive? can use unsigned code
 rsbs r0,#0
 sbc r1,r1,r1,lsl#1 @ make positive
 cbz r1,7f @ high word is zero?
 clz r3,r1
 subs r3,#11
 bmi 2f
 rsbs r12,r3,#32
 lsrs r12,r0,r12
 lsls r0,r3
 lsls r1,r3
 orrs r1,r1,r12
 add r2,r2,r3
 rsbs r2,#0
 add r2,#0x3ff+19+32
 add r1,r1,r2,lsl#20 @ insert exponent
 orr r1,#0x80000000
 mov r3,0x7fe
 cmp r2,r3
 it lo @ over/underflow?
 bxlo r14
 b 3f
7:
 mov r1,r2
 b fix2double_neg
2:
 add r3,#33
 lsls r12,r0,r3 @ rounding bit in carry, sticky bits in Z
 sub r3,#1
 lsl r12,r1,r3
 rsb r3,#32
 lsr r0,r3
 lsr r1,r3
 orr r0,r0,r12
@ push {r14}
@ bl dumpreg
@ pop {r14}
 sub r2,r3,r2
 add r2,#0x3ff+19+32
 beq 4f @ potential rounding tie?
 adcs r0,r0,#0
5:
 adc r1,r1,r2,lsl#20 @ insert exponent, add rounding
 orr r1,#0x80000000
 mov r3,0x7fe
 cmp r2,r3
 it lo
 bxlo r14
@ over/underflow?
3:
 mov r1,#0
 it ge
 movtge r1,#0x7ff0 @ overflow
 mov r0,#0
 bx r14
1:
 movs r1,#0
 bx r14
4:
 bcc 5b @ not a rounding tie after all
 adcs r0,r0,#0
 bic r0,r0,#1 @ force to even
 b 5b

@ convert uint64 to double, rounding
wrapper_func __aeabi_ul2d
regular_func uint642double
 movs r2,#0       @ fall through
@ convert unsigned 64-bit fix to double, rounding; number of r0:r1 bits after point in r2
regular_func ufix642double
10:
 cbz r1,7f @ high word zero?
 clz r3,r1
 subs r3,#11
 bmi 2f
 rsbs r12,r3,#32
 lsrs r12,r0,r12
 lsls r0,r3
 lsls r1,r3
 orrs r1,r1,r12
 add r2,r2,r3
 rsbs r2,#0
 add r2,#0x3ff+19+32
 add r1,r1,r2,lsl#20 @ insert exponent
 mov r3,0x7fe
 cmp r2,r3
 it lo @ over/underflow?
 bxlo r14
 b 3f
7:
 mov r1,r2
 b ufix2double
2:
 add r3,#33
 lsls r12,r0,r3 @ rounding bit in carry, sticky bits in Z
 sub r3,#1
 lsl r12,r1,r3
 rsb r3,#32
 lsr r0,r3
 lsr r1,r3
 orr r0,r0,r12
@ push {r14}
@ bl dumpreg
@ pop {r14}
 sub r2,r3,r2
 add r2,#0x3ff+19+32
 beq 4f @ potential rounding tie?
 adcs r0,r0,#0
5:
 adc r1,r1,r2,lsl#20 @ insert exponent, add rounding
 mov r3,0x7fe
 cmp r2,r3
 it lo
 bxlo r14
@ over/underflow?
3:
 mov r1,#0
 it ge
 movtge r1,#0x7ff0 @ overflow
 mov r0,#0
 bx r14
1:
 movs r1,#0
 bx r14
4:
 bcc 5b @ not a rounding tie after all
 adcs r0,r0,#0
 bic r0,r0,#1 @ force to even
 b 5b

regular_func fix2double
 cmp r0,#0
 bge ufix2double @ positive? can use unsigned code
 rsbs r0,#0 @ make positive
fix2double_neg:
 clz r3,r0
 subs r3,#11
 bmi 2f
 lsls r0,r3
 add r2,r1,r3
 rsbs r2,#0
 add r2,#0x3ff+19
 add r1,r0,r2,lsl#20 @ insert exponent
 orr r1,#0x80000000
 mov r0,#0
 mov r3,0x7fe
 cmp r2,r3
 it lo @ over/underflow?
 bxlo r14
 b 3f
2:
 rsb r3,#0
 lsrs r12,r0,r3
 rsb r2,r3,#32
 lsls r0,r0,r2
@ push {r14}
@ bl dumpreg
@ pop {r14}
 sub r2,r3,r1
 add r2,#0x3ff+19
 add r1,r12,r2,lsl#20 @ insert exponent
 orr r1,#0x80000000
 mov r3,0x7fe
 cmp r2,r3
 it lo
 bxlo r14
@ over/underflow?
3:
 mov r1,#0x80000000
 it ge
 movtge r1,#0xfff0 @ overflow
 mov r0,#0
 bx r14
1:
 movs r1,#0
 bx r14

regular_func ufix2double
 cbz r0,1f @ zero? return it
 clz r3,r0
 subs r3,#11
 bmi 2f
 lsls r0,r3
 add r2,r1,r3
 rsbs r2,#0
 add r2,#0x3ff+19
 add r1,r0,r2,lsl#20 @ insert exponent
 mov r0,#0
 mov r3,0x7fe
 cmp r2,r3
 it lo @ over/underflow?
 bxlo r14
 b 3f
2:
 rsbs r3,#0
 lsrs r12,r0,r3
 rsb r2,r3,#32
 lsls r0,r0,r2
@ push {r14}
@ bl dumpreg
@ pop {r14}
 sub r2,r3,r1
 add r2,#0x3ff+19
 add r1,r12,r2,lsl#20 @ insert exponent
 mov r3,0x7fe
 cmp r2,r3
 it lo
 bxlo r14
@ over/underflow?
3:
 mov r1,#0
 it ge
 movtge r1,#0x7ff0 @ overflow
 mov r0,#0
 bx r14
1:
 movs r1,#0
 bx r14

double_wrapper_section conv_dtoi64

@ convert double to signed int64, rounding towards 0, clamping
wrapper_func __aeabi_d2lz
regular_func double2int64_z
 movs r2,#0      @ fall through
@ convert double in r0:r1 to signed fixed point in r0:r1, clamping
regular_func double2fix64_z
 sub r2,#0x3ff+52-1 @ remove exponent bias, compensate for mantissa length
 asrs r12,r1,#20 @ sign and exponent
 sub r3,r12,#1
 sub r1,r1,r3,lsl#20 @ install implied 1, clear exponent
 lsls r3,#21
@ push {r14}
@ bl dumpreg
@ pop {r14}
 cmp r3,#0xffc00000
 bhs 1f @ 0, ∞/NaN?
 adds r2,r2,r3,lsr#21 @ offset exponent by fix precision; r1 is now required left shift
 bmi 4f @ actually a right shift?
 cmp r2,#11 @ overflow?
 bge 5f
 lsls r1,r2
 rsbs r3,r2,#32
 lsrs r3,r0,r3
 orrs r1,r1,r3
 lsls r0,r2
 cmp r12,#0
 it ge
 bxge r14
 rsbs r0,#0
 sbc r1,r1,r1,lsl#1
 bx r14
4:
 adds r2,#32
 ble 6f @ result fits in low word?
 lsl r3,r1,r2
 rsbs r2,#32
 lsrs r1,r2
 lsrs r0,r2
 orrs r0,r0,r3
 cmp r12,#0
 it ge
 bxge r14
 rsbs r0,#0
 sbc r1,r1,r1,lsl#1
 bx r14
6:
 rsbs r2,#0
 usat r2,#5,r2 @ underflow to 0
 lsrs r0,r1,r2
 movs r1,#0
 cmp r12,#0
 it ge
 bxge r14
 rsbs r0,#0
 sbc r1,r1,r1,lsl#1
 bx r14
1:
 beq 3f @ ±∞/±NaN?
2:
 movs r0,#0 @ ±0: return 0
 movs r1,#0
 bx r14
3:
 orrs r1,r0,r1,lsl#12 @ mantissa field
 it ne @ NaN?
 movne r12,#0 @ treat NaNs as +∞
@ here original argument was ±Inf or we have under/overflow
5:
 mvn r1,#0x80000000
 add r1,r1,r12,lsr#31 @ so -Inf → 0x80000000, +Inf → 0x7fffffff
 mvn r0,r12,asr#31
 bx r14

double_wrapper_section conv_dtoui64

@ convert double to unsigned int64, rounding towards -Inf, clamping
wrapper_func __aeabi_d2ulz
regular_func double2uint64
regular_func double2uint64_z
 movs r2,#0      @ fall through
@ convert double in r0:r1 to unsigned fixed point in r0:r1, clamping
regular_func double2ufix64
regular_func double2ufix64_z
 subw r2,r2,#0x3ff+52-1 @ remove exponent bias, compensate for mantissa length
 asrs r3,r1,#20 @ sign and exponent
 sub r3,#1
 sub r1,r1,r3,lsl#20 @ install implied 1, clear exponent and sign
 bmi 7f @ argument negative?
 movw r12,#0x7fe
 cmp r3,r12
 bhs 1f @ 0, ∞/NaN?
 adds r2,r3 @ offset exponent by fix precision; r2 is now required left shift
 bmi 2f @ actually a right shift?
 cmp r2,#12 @ overflow?
 bge 4f
 lsls r1,r2
 rsbs r3,r2,#32
 lsrs r3,r0,r3
 lsls r0,r2
 orrs r1,r1,r3
 bx r14
2:
 adds r2,#32
 ble 5f @ result fits in low word?
 lsl r3,r1,r2
 rsbs r2,#32
 lsrs r1,r2
 lsrs r0,r2
 orrs r0,r0,r3
 bx r14
5:
 rsbs r2,#0
 usat r2,#5,r2 @ underflow to 0
 lsrs r0,r1,r2
 movs r1,#0
 bx r14
1:
 bhi 3f @ 0? return 0
4:
@ here overflow has occurred
 mvn r0,#0
 mvn r1,#0
 bx r14
7:
 cmp r3,#0xfffffffe
 bne 3f @ -0? return 0
 orrs r2,r0,r1,lsl#12 @ mantissa field
 bne 4b
3:
 movs r0,#0
 movs r1,#0
 bx r14

#endif
