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

.extern rtwopi
.extern logtab0
.extern exptab0
.extern exptab1
.extern exptab2
.extern trigtab

@ load a 32-bit constant n into register rx
.macro movlong rx,n
 movw \rx,#(\n)&0xffff
 movt \rx,#((\n)>>16)&0xffff
.endm

float_section frrcore

@ input:
@ r0 mantissa m Q23
@ r1 exponent e>=-32, typically offset by +9
@ output:
@ r0..r1 preserved
@ r6 range reduced result
@ r2,r3,r4,r5 trashed
frr_core:
 ldr r2,=rtwopi
 asrs r3,r1,#5               @ k=e/32, k<=5 for e offsets up to 9+32
 add r2,r2,r3,lsl#2 @ p
 and r3,r1,#31               @ s=e%32
 mov r4,#1
 lsls r4,r4,r3               @ 1<<s
 umull r3,r4,r4,r0
@ r2    p
@ r3:r4 u0:u1 = m<<(e%32); u1 is never more than 2<<23
 ldr r5,[r2,#12]             @ a0=p[3]
 umull r5,r6,r5,r4           @ r0=a0*u1 hi, discard lo
@ r6  r0
 ldr r5,[r2,#8]              @ a1=p[2]
 mla r6,r5,r4,r6             @ a1*u1 lo, discard hi
 umlal r5,r6,r5,r3           @ a1*u0 hi, discard lo
@ r6  r0
 ldr r5,[r2,#4]              @ a2=p[1]
 mla r6,r5,r3,r6             @ r0+=a2*u0
 bx r14


float_wrapper_section expf

2:
@ we could use fadd macro to calculate x+1 here
@ need to add quadratic term
 rsb r3,#1                   @ 1-(e+9)
 lsrs r0,r3                  @ Q31
 smmulr r1,r0,r0             @ Q30
 cmp r12,#0
 bne 1f
 add r0,r0,r1
 lsrs r0,#8
 adc r0,r0,#0x3f800000       @ result is just over 1
 bx r14
1:
 sub r0,r1,r0
 asrs r0,#7
 adc r0,r0,#0x3f800000       @ result is just under 1
 bx r14

7:
 movs r1,#1
 bfi r0,r1,#23,#9            @ implied 1, clear sign bit
 adds r3,#9
 bmi 2b                      @ <~2^-9? return 1+x+x²/2
 movlong r1,0xb17217f8       @ ln2 Q32
 lsls r0,r3                  @ Q32
 cmp r12,#0
 bne 8f
 cmp r0,r1
 blo 6f
 sub r0,r0,r1
 mov r12,#1
 b 6f

8:
 mvn r12,#0
 subs r0,r1,r0
 bhs 6f
 add r0,r0,r1
 mvn r12,#1
 b 6f

wrapper_func expf
 ands r12,r0,#1<<31          @ save sign
 ubfx r3,r0,#23,#8           @ get exponent
 subs r3,r3,#0x7f
 bmi 7b                      @ e<0?
 cmp r3,#7
 bge 20f                     @ overflow, Inf or NaN?
 movs r1,#1
 bfi r0,r1,#23,#9            @ implied 1, clear sign bit
 lsls r0,r3                  @ x Q23
 eors r0,r0,r12,asr#31
 adds r0,r0,r12,lsr#31       @ negate if sign bit set
 mov r1,#0xB8AA              @ 1/ln2 Q15, rounded down
 add r1,r1,r12,lsr#31        @ rounded up if we have a negative argument
 smull r1,r2,r1,r0           @ Q38
 movlong r1,0xb17217f8       @ ln2 Q32
 asr r12,r2,#6               @ Q0 exponent candidate
 lsls r0,#9                  @ Q32
 mls r0,r12,r1,r0            @ Q32-Q0*Q32=Q32

6:
 lsrs r1,r0,#28
 ldr r2,=exptab0
 add r2,r2,r1,lsl#3
 ldmia r2,{r2-r3}
 and r2,r2,#63
 orr r2,#64                  @ y=(t&0x3f)+0x40; Q6
 sub r0,r3

 lsrs r1,r0,#24
 ldr r3,=exptab1+4
 ldr r3,[r3,r1,lsl#3]
 add r1,#256
 muls r2,r2,r1 @ y Q14
 sub r0,r3

 lsrs r1,r0,#21
 ldr r3,=exptab2+4
 ldr r3,[r3,r1,lsl#3]
 add r1,r1,r1
 add r1,#4096
 mla r2,r2,r1,r2             @ y Q26
 sub r0,r3                   @ ε Q32
 smmlar r2,r2,r0,r2          @ y(1+ε) Q26

 cmp r2,#1<<27
 bhs 1f
2:
 cmp r12,#0x7f
 bgt 23f
 cmn r12,#0x7e
 blt 23f
 lsls r0,r12,#23
 adds r0,r0,r2,lsr#3
 adc r0,r0,#0x3f000000
 bx r14

@ mantissa calculation has overflowed
1:
 lsrs r2,#1
 adds r12,#1
 b 2b

20:
@ process Inf/NaN for fexp
 cmp r3,#0x80
 bne 22f
 lsls r2,r0,#9
 ite eq
 biceq r0,r0,r0,asr#31       @ +Inf -> + Inf; -Inf -> +0
 orrne r0,r0,#0x00400000
 bx r14

23:
 ands r12,r12,#1<<31
22:
 eors r0,r12,#1<<31
 subs r0,r0,r0,lsr#8         @ overflow: convert to +0 or +Inf
 bx r14




float_wrapper_section logf

1:
 movlong r0,0xff800000       @ return -Inf
 bx r14

4:
 lsls r1,r0,#9
 it ne
 orrne r0,r0,#0x00400000
 bx r14

@ here result may be close to zero
5:
 sub r1,r0,#0x3f800000
 bmi 7f
 cmp r1,#0x8000
 bge 6f                      @ |ε|>~2^-8?
@ here ε positive
 clz r12,r1
 sub r3,r12,#1
 b 8f

7:
 rsbs r2,r1,#0
 cmp r2,#0x8000
 bge 6f                      @ |ε|>~2^-8?
@ here ε negative
@ r1: x Q24
 clz r12,r2
 sub r3,r12,#2
8:
 sub r12,#10
 lsls r0,r1,r3               @ ε Q24+r3
 beq 10f                     @ ε=0?
 smmulr r1,r0,r0
 asrs r1,r1,r12
 sub r0,r0,r1,asr#1          @ - ε²/2
 smmulr r1,r1,r0
 asrs r2,r1,r12
 movs r3,#0x55
 mul r2,r2,r3
 adds r0,r0,r2,asr#8         @ + ~ ε³/3
 rsb r1,r12,#117
 b 9f

wrapper_func logf
 lsls r3,r0,#1
 bcs 1b                      @ x<0?
 lsrs r3,#24
 beq 1b                      @ x==0/denormal?
 sub r12,r3,#0x7e
 cmp r12,#0x81               @ +Inf/NaN?
 beq 4b
 push {r14}
 cmp r12,#1                  @ result will be near zero?
 bls 5b
6:
 movs r2,#1
 bfi r0,r2,#23,#9            @ set implied 1, clear exponent Q52

 lsls r0,#8

 lsrs r1,r0,#27
 ldr r2,=logtab0-16*8
 add r2,r2,r1,lsl#3
 ldmia r2,{r2-r3}
 lsls r2,#26
 umlal r2,r0,r2,r0

 mvn r1,r0,asr#24
 ldr r2,=exptab1+4
 ldr r2,[r2,r1,lsl#3]
 add r3,r3,r2
 lsls r1,#24
 umlal r1,r0,r1,r0

 ldr r2,=exptab2+4
 mvn r1,r0,asr#21
 ldr r2,[r2,r1,lsl#3]
 lsls r1,#21
 orr r1,#0x00100000
 umlal r1,r0,r1,r0
 add r3,r3,r2

@ r0: ε Q32
@ r3: log y
 smmulr r1,r0,r0             @ ε² Q32
 sub r3,r0
 add r3,r3,r1,lsr#1          @ log u - ε + ε²/2 Q32
 movlong r2,0xb17218         @ ln2 Q24, but accurate to 2^-29 or so
 cmp r12,#1                  @ result will be near zero?
 bls 2f
 mul r2,r2,r12
 mov r1,#125
 add r3,#255                 @ reduce systematic error
 subs r0,r2,r3,lsr#8
9:
 bmi 1f
 bl fpack_q
10:
 pop {r15}

2:
 mov r1,#117
 bne 3f @ r12=0?
@ here r12=1
 movlong r2,0xb17217f8
 sub r0,r2,r3
 bl fpack_q
 pop {r15}

3:
 mov r0,r3
 bl fpack_q
 orrs r0,r0,#1<<31
 pop {r15}

1:
 rsbs r0,#0
 bl fpack_q
 orrs r0,r0,#1<<31
 pop {r15}


float_wrapper_section atan2f

@ fatan small reduced angle case
@ r0 y/x in IEEE format, 0..2^-8
@ r2 e+6 <0
@ r3 #1
@ r6 flags
20:
 rsbs r4,r2,#0               @ -e-6 = shift down of mantissa required to get to Q29 >0
 cmp r4,#7                   @ -e-6 ≥ 7 [ e ≤ -13 ]
 bge 1f                      @ very small reduced angle? atn θ ~ θ
@ do a power series
 bfi r0,r3,#23,#9            @ fix up mantissa Q23-e
 lsls r0,#7                  @ Q30-e
 smmulr r1,r0,r0             @ θ² Q60-2e-Q32=Q28-2e
 lsrs r1,r4                  @ Q28-2e + (e+6) = Q34-e
 smmulr r1,r1,r0             @ θ³ Q34-e+Q30-e-Q32=Q32-2e
 mov r3,#0x55555555          @ 1/3 Q32
 lsrs r1,r4                  @ Q32-2e + e+6 = Q38-e
 smmulr r1,r1,r3             @ θ³/3 Q38-e
 sub r0,r0,r1,lsr#8          @ Q30-e
 cmp r6,#4                   @ at least one quadrant to add?
 bhs 2f
 add r1,r2,#113
 bl fpack_q
 eors r0,r0,r6,lsl#31        @ fix sign
 pop {r4-r6,r15}

@ here reduced angle is < 2^-12
1:
 cmp r6,#4
 bhs 3f                      @ at least one quadrant to add?
 eors r0,r0,r6,lsl#31        @ no: return y/x with the correct sign
 pop {r4-r6,r15}

3:
 bfi r0,r3,#23,#9            @ fix up mantissa
 lsrs r0,r4                  @ Q29
 lsls r0,#1                  @ Q30
 b 40f

2:
 lsrs r0,r4                  @ Q30-e + e+6 = Q36
 lsrs r0,#6                  @ Q30
 b 40f

@ case where reduced (x',y') has x' infinite
71:
 sbfx r4,r0,#23,#8
 movs r0,#0
 cmn r4,#1                   @ y' also infinite?
 bne 80f
 mov r0,#0x3f800000          @ both infinite: pretend ∞/∞=1
 b 80f

@ case where reduced (x',y') has y' zero
70:
 ubfx r4,r1,#23,#8
 movs r0,#0
 cbnz r4,80f                 @ x' also zero?
 tst r6,#4
 beq 80f                     @ already in quadrants 0/±2? then 0/0 result will be correct
 tst r6,#2
 ite eq
 addeq r6,#6
 subne r6,#6                 @ fix up when in quadrants ±0
 b 80f

90:
 movs r0,r1
91:
 orrs r0,r0,#0x00400000
 pop {r4-r6,r15}

wrapper_func atan2f
 push {r4-r6,r14}
 lsls r4,r1,#1
 cmp r4,#0xff000000
 bhi 90b                     @ y NaN?
 lsls r4,r0,#1
 cmp r4,#0xff000000
 bhi 91b                     @ x NaN?
 lsrs r6,r0,#31              @ b31..2: quadrant count; b1: sign to apply before addition; b0: sign to apply after addition
 bic r0,#1<<31
@ y now positive
 movs r1,r1
 bpl 1f

@ here y positive, x negative
 adds r6,#10
 bic r1,r1,#1<<31
 cmp r1,r0
 bhi 4f                      @ |x| > y: 3rd octant
@ |x| < y: 2nd octant
 subs r6,#6
 b 3f

1:
@ here x and y positive
 cmp r1,r0
 bhs 4f
@ x < y: 1st octant
 adds r6,#6
3:
 movs r2,r1                  @ exchange x and y
 movs r1,r0
 movs r0,r2
4:
@ here
@ r0 y'
@ r1 x'
@ where both x' and y' are positive, y'/x' < 1+δ, and the final result is
@ ± (Q.π/2 ± atn y/x) where 0≤Q≤2 is a quadrant count in r6b3..2, the inner negation
@ is given by r6b1 and the outer negation by r6b0. x' can be infinite, or both x' and
@ y' can be infinite, but not y' alone.

 sbfx r2,r1,#23,#8
 cmn r2,#1
 beq 71b                     @ x' infinite?
 ubfx r2,r0,#23,#8
 cmp r2,#0
 beq 70b                     @ y' zero?
 bl __aeabi_fdiv
80:
                             @ r0 y/x in IEEE format, 0..1
 lsr r2,r0,#23               @ exponent
 movs r3,#1
 subs r2,#0x7f-6
 bmi 20b
 bfi r0,r3,#23,#9            @ fix up mantissa
 lsls r0,r2
 lsls r0,#2
50:

@ from here atan2(y,1) where 1 implied
@ r0 y Q31 0≤y<1+δ

 lsrs r2,r0,#16
 mul r3,r2,r2                @ y^2
 movw r4,#0x895c
 muls r2,r2,r4               @ y*0x895c
 movw r5,#0x1227
 lsrs r3,#14
 mls r2,r3,r5,r2
 subs r2,#0x330000           @ Chebyshev approximation to atn(y)
 lsrs r2,#25
 ldr r3,=trigtab+4
 add r3,r3,r2,lsl#4
 ldmia r3,{r3-r5}
@ r0 y  Q30
@ r3 phi0  Q32
@ r4 sphi0 Q31
@ r5 cphi0 Q31
@ r6 flags
@ x0= cphi0 + sphi0*y
@ y0=-sphi0 + cphi0*y
 umull r12,r1,r0,r4          @ sphi0*y
 umull r12,r2,r0,r5          @ cphi0*y
 add r1,r1,r5,lsr#1          @ x0 Q30
 sub r2,r2,r4,lsr#1          @ y0 Q30
11:
@ r1 x0 Q30
@ r2 y0 Q30
@ r3 phi0 Q32
@ r6 flags
 mov r0,#0xffffffff
 lsrs r4,r1,#15
 udiv r12,r0,r4              @ rx0 Q17
 lsls r4,r2,#6               @ y0 Q36
 smmul r4,r4,r12             @ t2=y0*rx0 Q21
 lsls r5,r4,#11              @ t2 Q32
 smmlar r0,r5,r2,r1
 smmlsr r1,r5,r1,r2
@ r0 x1 Q30
@ r1 y1 Q30
@ r3 phi0
@ r4 r2 Q21
@ r12 rx0 Q17
 mul r5,r4,r4                @ t2_2 Q42
 smull r2,r5,r4,r5           @ t2_3 Q63
 add r3,r3,r4,lsl#11         @ Q32
 lsls r5,#1                  @ Q32
 mov r2,#0x55555555          @ 1/3
 smmlsr r3,r2,r5,r3          @ t2_3/3

@ r0 x1 Q30
@ r1 y1 Q30
@ r3 phi0+phi1 Q32
@ r12 rx0 Q17
 mul r0,r1,r12               @ y1*rx0 Q30+Q17=Q47
 add r3,r3,r0,asr#15
@ r3 phi0+phi1+phi2, result over reduced range Q32
@ r6 flags

 lsrs r0,r3,#2               @ Q30
@ adc r0,#0                  @ rounding
40:
 lsl r5,r6,#30               @ b1 -> b31
 eor r0,r0,r5,asr#31         @ negate if required
 movlong r1,0x6487ED51       @ π/2 Q30

 lsr r5,r6,#2                @ quadrants to add
 mla r0,r5,r1,r0
 mov r1,#0x80-9              @ for packing Q30
60:
 bl fpack_q
 eors r0,r0,r6,lsl#31
 pop {r4-r6,r15}

@=======================================

float_wrapper_section fpack

@ fnegpack: negate and pack
@ fpack_q31:
@ input
@ r0 Q31 result, must not be zero
@ r1 exponent offset [fpack_q only]
@ output
@ r0 IEEE single
@ trashes (r1,)r2
fnegpack_q31:
 rsbs r0,r0,#0
fpack_q31:
 mov r1,#0x7f-9              @ exponent
fpack_q:
 clz r2,r0
 rsbs r2,#8
 bmi 1f
 lsrs r0,r0,r2               @ save rounding bit in carry
 add r2,r2,r1
 adc r0,r0,r2,lsl#23         @ insert exponent
 bx r14

1:
 rsb r2,#0
 lsls r0,r0,r2
 sub r2,r1,r2
 add r0,r0,r2,lsl#23
 bx r14

float_wrapper_section tanf

wrapper_func tanf
 push {r14}
 bl sincosf_raw
 bl __aeabi_fdiv
 pop {r15}

float_wrapper_section fsin_fcos

10: @ process Inf/NaN for sinf and fcos
 lsls r1,r0,#9
 it eq
 orreq r0,#0x80000000
 orr r0,#0x00400000          @ turn Inf into NaN
 movs r1,r0                  @ copy result to cosine output
 bx r14

@ case where angle is very small (<2^-32) before reduction
32:
 adds r1,r1,#0x7f
 it eq
 moveq r0,#0                 @ flush denormal to zero

 movs r1,0x3f800000          @ return x for sine, 1 for cosine
 tst r12,#2
 it ne
 movne r0,r1                 @ calculating cosine? move to result registers
 bx r14

40:
@ case where range-reduced angle is very small
@ here
@ r0 mantissa
@ r1 exponent+9
@ r6 abs range-reduced angle / 2π < 2^-7 Q32
@ r12b31: dsincos flag
@ r12b30: original argument sign
@ r12b2..1: quadrant count
@ r12b0: sign of reduced angle
 push {r12}
 movs r12,#0
2:
 add r12,#2
 add r1,#2
 bl frr_core                 @ repeat range reduction with extra factor of 2^2 (, 2^4, 2^6, 2^8,...)
 eors r4,r6,r6,asr#31        @ we want to keep the sign in r6b31 for later
 cmp r4,#1<<26               @ ≥ 2^-6?
 bhs 1f                      @ loop until the result is big enough
 cmp r12,#32                 @ safety net
 bne 2b
1:
@ here r4 is the abs range-reduced angle Q32+r12, 2^-6..2^-4 in Q32 terms

@ 2π=6.487ED511 (0B4611A6...)
 movlong r5,0x487ED511       @ 2π Q64 high fractional word
 umull r2,r0,r4,r5
 movs r5,#6                  @ 2π integer part
 mla r0,r4,r5,r0

@ here
@ r0 θ, abs range reduced angle θ 0..π/4 Q32+r12, 2π * 2^-6..2^-4 in Q32 terms (so top bit is clear)
@ r6b31: sign of reduced angle
@ r12: excess exponent ≥2, multiple of 2
@ r0 / 2^r12 < ~ 2π * 2^-7 so for sin we need to go to term in x^3

 smmulr r1,r0,r0             @ θ² Q32+2*r12
 lsrs r1,r1,r12              @ θ² Q32+r12
 lsrs r1,r1,r12              @ θ² Q32
 movs r4,#0x55555555         @ 1/3 Q32
 smmulr r4,r1,r4             @ θ²/3 Q32
 smmulr r2,r4,r1             @ θ⁴/3 Q32
 rsb r3,r1,r2,lsr#2          @ -θ²+θ⁴/12) Q32
 asrs r3,#9                  @ -θ²/2+θ⁴/24 Q24
 adc r3,r3,#0x3f800000       @ IEEE single with rounding

 smmulr r2,r4,r0             @ θ³/3 Q32+r12
 sub r0,r0,r2,lsr#1          @ θ-θ³/6 Q32+r12
 rsb r1,r12,#117
 bl fpack_q
@ here
@ r0 packed sin
@ r3 packed cos
@ r6b31: sign of reduced angle
 pop {r12}                   @ get flags
 lsrs r6,#31
 bfi r12,r6,#0,#1

 asrs r2,r12,#1
 bmi 23f                     @ doing sincos?
 asrs r2,#1
 bcc 21f                     @ need sine?
@ need cosine:
 ands r12,#4
 orrs r0,r3,r12,lsl#29       @ insert sign
 pop {r4-r6,r15}

21:
 eors r12,r12,r12,lsr#2
 orrs r0,r0,r12,lsl#31       @ insert sign
 pop {r4-r6,r15}

23:
 ands r2,r12,#4
 orrs r3,r3,r2,lsl#29        @ insert sign
 push {r3}
@ drop into sincosf code below...
 b 20f

wrapper_func sincosf
 push {r1, r2, lr}
 bl sincosf_raw
 pop {r2, r3}
 str r0, [r2]
 str r1, [r3]
 pop {pc}

sincosf_raw:
 ands r12,r0,#1<<31
 lsrs r12,#1                 @ save argument sign in r12b30
 orrs r12,r12,#1<<31         @ flag we want both results in r12b31
 b 1f

wrapper_func sinf
 lsrs r12,r0,#29             @ negative argument -> +2 quadrants
 ands r12,#4
 b 1f

wrapper_func cosf
 movs r12,#2                 @ cos -> +1 quadrant
1:
 ubfx r1,r0,#23,#8           @ get exponent
 sub r1,r1,#0x7f
 cmp r1,#0x80
 beq 10b                     @ Inf or NaN?
 cmn r1,#32
 blt 32b                     @ very small argument?
 movs r3,#1
 bfi r0,r3,#23,#9            @ fix implied 1 in mantissa
 push {r4-r6,r14}
 add r1,#9                   @ e+9
 bl frr_core
@ r6 θ/2π 0..1 Q64
 lsrs r4,r6,#30              @ quadrant count
 adc r4,r4,#0                @ rounded
 sub r6,r6,r4,lsl#30         @ now -0.125≤r6<+0.125 Q32
 add r12,r12,r4,lsl#1
 orr r12,r12,r6,lsr#31       @ sign into r12b0
@ r12b2..1: quadrant count
@ r12b0: sign of reduced angle
 eors r6,r6,r6,asr#31        @ absolute value of reduced angle 0≤r7<0.125 Q32
 cmp r6,#1<<25               @ θ / 2π < 2^-7?
 blo 40b

@ 2π=6.487ED511 (0B4611A6...)
 movlong r5,0x487ED511       @ 2π Q64 high fractional word
 umull r2,r0,r6,r5
 movs r5,#6                  @ 2π integer part
 mla r0,r6,r5,r0
@ r0 range reduced angle θ 0..π/4 Q32
 lsrs r2,r0,#27
 ldr r3,=trigtab+4
 add r3,r3,r2,lsl#4
 ldmia r3,{r1-r3}
 subs r0,r1
@ r0:   ε     Q32 |ε| < 1.17*2^-6
@ r2:   sin φ Q31
@ r3:   cos φ Q31
 asrs r1,r12,#1
 bmi 5f                      @ doing sincosf?
 asrs r1,#1
 bcs 3f                      @ need cosine?
@ here need sine
 smmulr r1,r0,r0             @ ε² Q32
 mov r4,#0x55555555          @ 1/3 Q32
 asrs r1,#1
 smmlsr r2,r1,r2,r2          @ sin φ - ε²/2*sin φ ~ sin φ cos ε
 smmulr r1,r1,r0             @ ε³/2
 smmlsr r1,r1,r4,r0          @ ε - ε³/6

 smmlar r0,r1,r3,r2          @ sin φ cos ε + cos φ (ε - ε³/6) ~ sin (φ+ε)
@ the sign of the result is r12b0^r12b2
 bl fpack_q31
 eors r12,r12,r12,lsr#2
 orrs r0,r0,r12,lsl#31       @ insert sign
 pop {r4-r6,r15}

3:
@ here need cosine
 smmulr r1,r0,r0             @ ε² Q32
 mov r4,#0x55555555          @ 1/3 Q32
 asrs r1,#1
 smmlsr r3,r1,r3,r3          @ cos φ - ε²/2*cos φ ~ cos φ cos ε
 smmulr r1,r1,r0             @ ε³/2
 smmlsr r1,r1,r4,r0          @ ε - ε³/6

 smmlsr r0,r1,r2,r3 @ cos φ cos ε - sin φ (ε - ε³/6) ~ cos (φ+ε)
@ the sign of the result is r12b2
 bl fpack_q31
 ands r12,#4
 orrs r0,r0,r12,lsl#29       @ insert sign
 pop {r4-r6,r15}

5:
@ here doing sincosf
 smmulr r1,r0,r0             @ ε² Q32
 mov r6,#0x55555555          @ 1/3 Q32
 asrs r1,#1
 smmlsr r4,r1,r2,r2          @ sin φ - ε²/2*sin φ ~ sin φ cos ε
 smmlsr r5,r1,r3,r3          @ cos φ - ε²/2*cos φ ~ cos φ cos ε
 smmulr r1,r1,r0             @ ε³/2
 smmlsr r6,r1,r6,r0          @ ε - ε³/6
@ here
@ r2 sin φ
@ r3 cos φ
@ r4 sin φ cos ε
@ r5 cos φ cos ε
@ r6 ε - ε³/6 ~ sin ε
 smmlsr r0,r6,r2,r5          @ cos φ cos ε - sin φ (ε - ε³/6) ~ cos (φ+ε)
 bl fpack_q31
 ands r5,r12,#4
 eors r0,r0,r5,lsl#29        @ negate cosine in quadrants 2 and 3
 push {r0}
 smmlar r0,r6,r3,r4          @ sin φ cos ε + cos φ (ε - ε³/6) ~ sin (φ+ε)
 bl fpack_q31
20:
 eors r4,r12,r12,lsr#1
 eors r4,r4,r12,lsr#2
 ands r5,r12,#1<<30
 tst r12,#2                  @ exchange sine and cosine in odd quadrants
 beq 1f
 eors r1,r0,r4,lsl#31        @ negate sine on b0^b1^b2
 pop {r0}
 eors r0,r0,r5,lsl#1         @ negate sine result if argument was negative
 pop {r4-r6,r15}
1:
 pop {r1}
 eors r0,r0,r4,lsl#31        @ negate sine on b0^b1^b2
 eors r0,r0,r5,lsl#1         @ negate sine result if argument was negative
 pop {r4-r6,r15}

#endif
