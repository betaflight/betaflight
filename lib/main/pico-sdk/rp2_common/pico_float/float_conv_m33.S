/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if !PICO_RP2040
#include "pico/asm_helper.S"

pico_default_asm_setup

.macro float_section name
#if PICO_FLOAT_IN_RAM
.section RAM_SECTION_NAME(\name), "ax"
#else
.section SECTION_NAME(\name), "ax"
#endif
.endm

.macro float_wrapper_section func
float_section WRAPPER_FUNC_NAME(\func)
.endm

float_wrapper_section conv_tof

@ convert int64 to float, rounding
wrapper_func __aeabi_l2f
regular_func int642float
 movs r2,#0       @ fall through
@ convert signed 64-bit fix to float, rounding; number of r0:r1 bits after point in r2
regular_func fix642float
 cmp r1,#0
 bge 10f @ positive? use unsigned code
 rsbs r0,#0
 sbc r1,r1,r1,lsl#1 @ make positive
 cbz r1,7f @ high word is zero?
 clz r3,r1
 subs r3,#8
 bmi 2f
 lsls r1,r3
 lsls r12,r0,r3 @ bits that will be lost
 rsb r3,#32
 lsr r0,r3
 orr r0,r0,r1
 sub r2,r2,r3
 rsb r2,#149
 adds r12,r12,r12 @ rounding bit into carry
 adc r0,r0,r2,lsl#23 @ insert exponent, add rounding
 orr r0,r0,#0x80000000
 beq 4f @ potential rounding tie?
 cmp r2,#0xfe
 bhs 3f @ over/underflow?
 bx r14
2:
 add r3,#33
 lsls r12,r1,r3 @ rounding bit in carry, sticky bits in r12
 orrs r12,r12,r0 @ all of low word into sticky bits: affects Z but not C
 rsb r3,#33
 lsr r0,r1,r3
@ push {r14}
@ bl dumpreg
@ pop {r14}
 sub r2,r3,r2
 add r2,#22+127+32
 adc r0,r0,r2,lsl#23 @ insert exponent, add rounding
 orr r0,r0,#0x80000000
 beq 4f @ potential rounding tie?
 cmp r2,#0xfe
 it lo
 bxlo r14
@ over/underflow?
3:
 mov r0,#0x80000000 @ underflow
 it ge
 movtge r0,#0xff80 @ overflow
1:
 bx r14
7:
 mov r1,r2
 b fix2float_neg
4:
 it cs @ rounding tie?
 biccs r0,r0,#1 @ force to even if we rounded up
 cmp r2,#0xfe
 it lo
 bxlo r14
 b 3b

@ convert signed 32-bit fix to float, rounding; number of r0 bits after point in r1
.thumb_func
regular_func fix2float
 cmp r0,#0
 bge ufix2float @ positive? can use unsigned code
 rsbs r0,#0 @ make positive
fix2float_neg:
 clz r3,r0
 subs r3,#8
 bmi 2f
 lsls r0,r3
 add r2,r1,r3
 rsb r2,#149
 add r0,r0,r2,lsl#23 @ insert exponent
 orr r0,#0x80000000
 cmp r2,#0xfe
 it lo @ over/underflow?
 bxlo r14
 b 3f
2:
 add r3,#33
 lsls r12,r0,r3 @ rounding bit in carry, sticky bits in r12
 rsb r3,#33
 lsr r0,r3
@ push {r14}
@ bl dumpreg
@ pop {r14}
 sub r2,r3,r1
 add r2,#22+127
 adc r0,r0,r2,lsl#23 @ insert exponent
 orr r0,#0x80000000
 beq 4f @ potential rounding tie?
 cmp r2,#0xfe
 it lo
 bxlo r14
@ over/underflow?
3:
 mov r0,#0x80000000 @ underflow
 it ge
 orrge r0,#0x7f800000 @ overflow
1:
 bx r14
4:
 it cs @ rounding tie?
 biccs r0,r0,#1 @ force to even if we rounded up
 cmp r2,#0xfe
 it lo
 bxlo r14
 b 3b

@ convert unsigned 32-bit fix to float, rounding; number of r0 bits after point in r1
regular_func ufix2float
 cbz r0,1f @ zero? return it
 clz r3,r0
 subs r3,#8
 bmi 2f
 lsls r0,r3
 add r2,r1,r3
 rsb r2,#149
 add r0,r0,r2,lsl#23 @ insert exponent
@ push {r14}
@ bl dumpreg
@ pop {r14}
 cmp r2,#0xfe
 it lo @ over/underflow?
 bxlo r14
 b 3f
2:
 add r3,#33
 lsls r12,r0,r3 @ rounding bit in carry, sticky bits in r12
 rsb r3,#33
 lsr r0,r3
@ push {r14}
@ bl dumpreg
@ pop {r14}
 sub r2,r3,r1
 add r2,#22+127
 adc r0,r0,r2,lsl#23 @ insert exponent
 beq 4f @ potential rounding tie?
 cmp r2,#0xfe
 it lo
 bxlo r14
@ over/underflow?
3:
 ite ge
 movge r0,#0x7f800000 @ overflow
 movlt r0,#0x00000000 @ underflow
1:
 bx r14
4:
 it cs @ rounding tie?
 biccs r0,r0,#1 @ force to even if we rounded up
 cmp r2,#0xfe
 it lo
 bxlo r14
 b 3b

@ convert uint64 to float, rounding
wrapper_func __aeabi_ul2f
regular_func uint642float
 movs r2,#0       @ fall through
@ convert unsigned 64-bit fix to float, rounding; number of r0:r1 bits after point in r2
regular_func ufix642float
10:
 cbz r1,7f @ high word is zero?
 clz r3,r1
 subs r3,#8
 bmi 2f
 lsls r1,r3
 lsls r12,r0,r3 @ bits that will be lost
 rsb r3,#32
 lsr r0,r3
 orr r0,r0,r1
 sub r2,r2,r3
 rsb r2,#149
 adds r12,r12,r12 @ rounding bit into carry
 adc r0,r0,r2,lsl#23 @ insert exponent, add rounding
 beq 4f @ potential rounding tie?
 cmp r2,#0xfe
 bhs 3f @ over/underflow?
 bx r14
2:
 add r3,#33
 lsls r12,r1,r3 @ rounding bit in carry, sticky bits in r12
 orrs r12,r12,r0 @ all of low word into sticky bits: affects Z but not C
 rsb r3,#33
 lsr r0,r1,r3
@ push {r14}
@ bl dumpreg
@ pop {r14}
 sub r2,r3,r2
 add r2,#22+127+32
 adc r0,r0,r2,lsl#23 @ insert exponent, add rounding
 beq 4f @ potential rounding tie?
 cmp r2,#0xfe
 it lo
 bxlo r14
@ over/underflow?
3:
 ite ge
 movge r0,#0x7f800000 @ overflow
 movlt r0,#0x00000000 @ underflow
1:
 bx r14
7:
 mov r1,r2
 b ufix2float
4:
 it cs @ rounding tie?
 biccs r0,r0,#1 @ force to even if we rounded up
 cmp r2,#0xfe
 it lo
 bxlo r14
 b 3b

float_wrapper_section conv_ftoi64

@ convert float to signed int64, rounding towards 0, clamping
wrapper_func __aeabi_f2lz
regular_func float2int64_z
 movs r1,#0      @ fall through
@ convert float in r0 to signed fixed point in r0:r1, clamping
regular_func float2fix64_z
 subs r1,#0x95 @ remove exponent bias, compensate for mantissa length
 asrs r2,r0,#23 @ sign and exponent
 sub r3,r2,#1
 sub r0,r0,r3,lsl#23 @ install implied 1, clear exponent
 uxtb r3,r3
 cmp r3,#0xfe
 bhs 1f @ 0 or Inf/NaN?
 adds r1,r3 @ offset exponent by fix precision; r1 is now required left shift
 bmi 4f @ actually a right shift?
 subs r3,r1,#32 @ result fits in high 32 bits only?
 bge 8f
 subs r3,r1,#8 @ result fits in low 32 bits only?
 ble 7f
 lsls r0,#8
 rsbs r1,r3,#32
 lsrs r1,r0,r1
 lsls r0,r3
 cmp r2,#0
 it ge
 bxge r14
 rsbs r0,#0 @ negate if necessary
 sbcs r1,r1,r1,lsl#1
 bx r14
7:
 lsls r0,r0,r1
 movs r1,r2,asr#31 @ sign extend
 eors r0,r0,r1 @ negate if necessary
 subs r0,r0,r1
 bx r14
8:
 cmp r3,#8 @ overflow?
 bge 5f
 lsls r0,r0,r3
 eor r1,r0,r2,asr#31 @ negate if necessary
 add r1,r1,r2,lsr#31
 movs r0,#0
 bx r14
1:
 bhi 3f @ 0?
 lsls r1,r0,#9 @ mantissa field
 it ne @ NaN?
 movne r2,#0 @ treat NaNs as +∞
5:
 mvn r1,#0x80000000 @ = 0x7fffffff
 add r1,r1,r2,lsr#31 @ so -Inf → 0x80000000, +Inf → 0x7fffffff
 mvn r0,r2,asr#31
 bx r14
3:
 movs r0,#0
 movs r1,#0
 bx r14
4:
 rsbs r1,#0
 usat r1,#5,r1
 lsrs r0,r0,r1
 eors r0,r0,r2,asr#31 @ negate if necessary
 adds r0,r0,r2,lsr#31
 movs r1,r0,asr#31 @ sign extend
 bx r14

float_wrapper_section conv_ftoui64

@ convert float to unsigned int64, rounding towards -Inf, clamping
wrapper_func __aeabi_f2ulz
regular_func float2uint64
regular_func float2uint64_z
 movs r1,#0      @ fall through
@ convert float in r0 to unsigned fixed point in r0:r1, clamping
regular_func float2ufix64
//regular_func float2ufix64_z
 subs r1,#0x96 @ remove exponent bias, compensate for mantissa length
 asrs r2,r0,#23 @ sign and exponent
 sub r3,r2,#1
 cmp r3,#0xfe
 bhs 1f @ -ve, 0 or Inf/NaN?
 sub r0,r0,r3,lsl#23 @ install implied 1, clear exponent
 adds r1,r2 @ offset exponent by fix precision; r1 is now required left shift
 bmi 4f @ actually a right shift?
 subs r2,r1,#7
 ble 7f @ result (easily) fits in lo 32 bits?
 subs r3,r1,#32
 bge 8f @ results might fit in hi 32 bits?
 lsls r0,r0,#7
 rsbs r3,r2,#32
 lsrs r1,r0,r3
 lsls r0,r0,r2
 bx r14
7:
 lsls r0,r1
 movs r1,#0
 bx r14
8:
 cmp r1,#32+9 @ overflow?
 bge 5f
 lsls r1,r0,r3
 movs r0,#0
 bx r14
5:
 mvn r0,#0 @ = 0xffffffff
 mvn r1,#0 @ = 0xffffffff
 bx r14
4:
 rsbs r1,#0
 usat r1,#5,r1 @ if shift is long return 0
 lsrs r0,r0,r1
 movs r1,#0
 bx r14
1:
 cmp r0,#0xff800000
 bhi 5b @ -NaN, return 0xffffffff
 cmp r0,#0x00800000
 bgt 5b @ +Inf or +NaN, return 0xfffffff
2:
 movs r0,#0 @ return 0
 movs r1,#0
 bx r14

#endif
