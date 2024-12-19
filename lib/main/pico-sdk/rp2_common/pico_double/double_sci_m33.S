/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if !PICO_RP2040
#include "pico/asm_helper.S"

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

.global rtwopi
.global logtab0
.global exptab0
.global exptab1
.global exptab2
.global trigtab

@ load a 32-bit constant n into register rx
.macro movlong rx,n
 movw \rx,#(\n)&0xffff
 movt \rx,#((\n)>>16)&0xffff
.endm

double_section rtwopi

// 1/2π to plenty of accuracy, 256 bits per line
.long 0                      @ this allows values of e down to -32
rtwopi:
.long 0,0
.long 0x28BE60DB, 0x9391054A, 0x7F09D5F4, 0x7D4D3770, 0x36D8A566, 0x4F10E410, 0x7F9458EA, 0xF7AEF158
.long 0x6DC91B8E, 0x909374B8, 0x01924BBA, 0x82746487, 0x3F877AC7, 0x2C4A69CF, 0xBA208D7D, 0x4BAED121
.long 0x3A671C09, 0xAD17DF90, 0x4E64758E, 0x60D4CE7D, 0x272117E2, 0xEF7E4A0E, 0xC7FE25FF, 0xF7816603
.long 0xFBCBC462, 0xD6829B47, 0xDB4D9FB3, 0xC9F2C26D, 0xD3D18FD9, 0xA797FA8B, 0x5D49EEB1, 0xFAF97C5E
.long 0xCF41CE7D, 0xE294A4BA, 0x9AFED7EC, 0x47E35742, 0x1580CC11

double_section drrcore

@ input:
@ r0:r1 mantissa m Q52
@ r2 exponent e>=-32, typically offset by +12
@ r3 pointer to rtwopi table
@ output:
@ r0..r2 preserved
@ r7:r8 range reduced result
@ r3,r4,r5,r6,r9,r10 trashed
.thumb_func
drr_core:
 ldr r3,=rtwopi
 and r5,r2,#31               @ s=e%32
 mov r7,#1
 lsls r7,r7,r5               @ 1<<s
 asrs r8,r2,#5               @ k=e/32, k<=32 for double with e offset <32; with e offsets up to 12+64, k<=34
 umull r4,r5,r7,r0
 movs r6,#0
 umlal r5,r6,r7,r1
@ r2       e
@ r4:r5:r6 u0:u1:u2 = m<<(e%32); u2 is never more than 2<<20
@ r8       e/32
 add r3,r3,r8,lsl#2          @ p

 ldr r9,[r3,#16]             @ a0=p[4]
 umull r10,r7,r9,r6          @ r0=a0*u2 hi, discard lo
@ r7  r0
 ldr r9,[r3,#12]             @ a1=p[3]
 umull r10,r8,r9,r5          @ r1=a1*u1 hi, discard lo
 umaal r7,r8,r9,r6           @ r0:r1=r0+r1+a1*u2
@ r7:r8  r0:r1
 ldr r9,[r3,#8]              @ a2=p[2]
 mla r8,r9,r6,r8             @ r1+=a2*u2
 umlal r7,r8,r9,r5           @ r0:r1+=a2*u1
 umull r10,r6,r9,r4          @ r0x=a2*u0 hi, discard lo; u2 no longer needed
@ r7:r8 r0:r1
@ r6 r0x
 ldr r9,[r3,#4]              @ a3=p[1]
 mla r8,r9,r5,r8             @ r1+=a3*u1
 umlal r7,r8,r9,r4           @ r0:r1+=a3*u0
@ r7:r8 r0:r1
@ r6 r0x
 ldr r9,[r3,#0]              @ a4=p[0]
 mla r8,r9,r4,r8             @ r1+=a4*u0
@ r7:r8 r0:r1
@ r6 r0x
 adds r7,r6
 adc r8, r8, #0
 bx r14

double_wrapper_section tan
wrapper_func tan
 push {r14}
 bl sincos_raw
 bl __aeabi_ddiv
 pop {r15}

double_wrapper_section sin_cos

10:                          @ process Inf/NaN for dsin and dcos
 orrs r2,r0,r1,lsl#12
 it eq
 orreq r1,#0x80000000
 orr r1,#0x00080000
 movs r3,r1                  @ copy results to cosine output
 movs r2,r0
 bx r14

@ case where angle is very small (<2^-32) before reduction
32:
 cbnz r3,33f
 movs r0,#0                  @ flush denormal
 movs r1,#0
33:
 movs r2,#0                  @ return x for sine, 1 for cosine
 movlong r3,0x3ff00000
 tst r12,#2
 itt ne
 movne r0,r2                 @ calculating cosine? move to result registers
 movne r1,r3
 bx r14

@ case where angle is fairly small after reduction
30:
 movs r2,#0
 movs r3,#0
 movs r4,#0
 movs r5,#0x80000000
 b 31f

40:
@ case where range-reduced angle is very small
@ here
@ r0:r1 mantissa
@ r2 exponent+12
@ r7:r8 abs range-reduced angle < 2^-10 Q64
@ r12b31: dsincos flag
@ r12b30: original argument sign
@ r12b2..1: quadrant count
@ r12b0: sign of reduced angle
 push {r12}
 movs r12,#0
2:
 add r12,#4
 add r2,#4
 bl drr_core                 @ repeat range reduction with extra factor of 2^4 (, 2^8, 2^12, 2^16,...)
 eors r9,r8,r8,asr#31        @ we want to keep the sign in r8b31 for later
 cmp r9,#1<<24               @ ≥ 2^-8?
 bhs 1f                      @ loop until the result is big enough
 cmp r12,#64                 @ safety net
 bne 2b
1:
 eors r7,r7,r8,asr#31
@ here r7:r9 is the abs range-reduced angle Q64+r12, 2^-8..2^-4 in Q64 terms

@ 2π=6.487ED511 0B4611A6 (2633...)
 movlong r6,0x0B4611A6       @ 2π Q64 low fractional word
 umull r10,r0,r9,r6
 movlong r6,0x487ED511       @ 2π Q64 high fractional word
 umull r10,r1,r7,r6
 umaal r0,r1,r9,r6
 movs r6,#6                  @ 2π integer part
 umlal r0,r1,r7,r6
 mla r1,r9,r6,r1

@ here
@ r0:r1 θ, abs range reduced angle θ 0..π/4 Q64+r12
@ r8b31: sign of reduced angle
@ r12: excess exponent ≥4, multiple of 4
@ r0:r1 / 2^r12 < ~ 2π * 2^-10 so for sin we need to go to term in x^5
 rsbs r10,r12,#32
 bmi 1f                      @ very small result?

 lsr r3,r1,r12
 lsr r2,r0,r12
 lsl r9,r1,r10
 orr r2,r9
@ r2:r3 θ Q64 (with r12 bits of loss of precision)
 umull r9,r4,r2,r3
 umull r9,r5,r2,r3
 umaal r4,r5,r3,r3
@ r4:r5 θ² Q64 = θ²/2 Q65 < ~ 4π² * 2^-20
 umull r9,r6,r4,r5
 umull r9,r7,r4,r5
 umaal r6,r7,r5,r5
@ r6:r7 θ⁴ Q64 < ~ 16π⁴ * 2^-40 < 2^-29
 lsrs r6,#3
 orrs r6,r6,r7,lsl#29
@ r6 θ⁴ Q61
 mov r9,#0xaaaaaaaa          @ 2/3 Q32
 umull r6,r7,r6,r9
@ r7 θ⁴ * 2/3 Q61 = θ⁴/24 Q65
 rsbs r6,r4,r7
 sbcs r7,r5,r5,lsl#1
@ r6:r7  - θ²/2 + θ⁴/24 Q65, <=0
 asrs r9,r7,#12
 lsrs r6,#12
 adcs r6,r6,r7,lsl#20
 adcs r7,r9,#0x0ff00000
 adds r7,#0x30000000
 push {r6,r7}                @ packed cos θ

@ here
@ r0:r1 θ, abs range reduced angle θ 0..π/4 Q64+r12
@ r4:r5 θ² Q64 = θ²/2 Q65
 umull r9,r2,r0,r5
 umull r9,r3,r1,r4
 umaal r2,r3,r1,r5
@ r2:r3 θ³ Q64+r12
 umull r9,r6,r2,r5
 umull r9,r7,r3,r4
 umaal r6,r7,r3,r5
@ r6:r7 θ⁵ Q64+r12; in fact r7 is always 0
 mov r9,#0xaaaaaaaa          @ 2/3 Q32
 umull r10,r4,r2,r9
 movs r5,#0
 umlal r4,r5,r3,r9
 adds r4,r4,r5
 adcs r5,r5,#0
@ r4:r5 θ³*2/3 Q64+r12 = θ³/6 Q66+r12
 mov r9,#0x88888888          @ 8/15 Q32
 umull r2,r3,r6,r9
@ r3 θ⁵*8/15 Q64+r12 = θ⁵/120 Q70+r12
 subs r4,r4,r3,lsr#4
 sbc r5,r5,#0
@ r4:r5 θ³/6-θ⁵/120 Q66+r12
 subs r0,r0,r4,lsr#2
 sbc r1,r1,r5,asr#2
 subs r0,r0,r5,lsl#30
 sbc r1,r1,#0
@ r0:r1 θ-θ³/6+θ⁵/120 Q64+r12
 rsb r4,r12,#0x400
 sub r4,#14
 bl dpack_q
 pop {r6,r7,r12}             @ get cosine, flags
 b 2f

1:
@ case where range reduction result has excess exponent ≥ 32
@ so sin x=x, cos x=1
 rsb r4,r12,#0x400
 sub r4,#14
 bl dpack_q
 pop {r12}                   @ get flags
 movs r6,#0
 movlong r7,0x3ff00000
2:
@ here r0:r1 is packed sine, r6:r7 is packed cosine
 lsrs r8,#31
 bfi r12,r8,#0,#1

 asrs r9,r12,#1
 bmi 23f                     @ doing dsincos?
 asrs r9,#1
 bcc 21f                     @ need sine?
@ need cosine:
 ands r12,#4
 orrs r1,r7,r12,lsl#29       @ insert sign
 movs r0,r6
 pop {r4-r10,r15}

21:
 eors r12,r12,r12,lsr#2
 orrs r1,r1,r12,lsl#31       @ insert sign
 pop {r4-r10,r15}

23:
 ands r4,r12,#4
 eors r7,r7,r4,lsl#29        @ insert sign
 push {r6,r7}
 b 20f

@ sincos à la GNU with pointers to where to put results
wrapper_func sincos
 push {r2-r5, lr}
 bl sincos_raw
 pop {r4-r5}
 stmia r4!, {r0, r1}
 stmia r5!, {r2, r3}
 pop {r4, r5, pc}

@ sincos with results in r0:r1 and r2:r3
.thumb_func
sincos_raw:
 ands r12,r1,#1<<31
 lsrs r12,#1                 @ save argument sign in r12b30
 orrs r12,r12,#1<<31         @ flag we want both results in r12b31
 b 1f

wrapper_func sin
 lsrs r12,r1,#29             @ negative argument -> +2 quadrants
 ands r12,#4
 b 1f

wrapper_func cos
 movs r12,#2                 @ cos -> +1 quadrant
1:
 ubfx r3,r1,#20,#11          @ get exponent
 sub r2,r3,#0x3ff
 cmp r2,#0x400
 beq 10b                     @ Inf or NaN?
 cmn r2,#32
 blt 32b                     @ very small argument?
 movs r3,#1
 bfi r1,r3,#20,#12           @ fix implied 1 in mantissa
 push {r4-r10,r14}
 add r2,#12                  @ e+12
 bl drr_core
@ r7:r8 θ/2π 0..1 Q64
 lsrs r4,r8,#30              @ quadrant count
 adc r4,r4,#0                @ rounded
 sub r8,r8,r4,lsl#30         @ now -0.125≤r7:r8<+0.125 Q64
 add r12,r12,r4,lsl#1
 orr r12,r12,r8,lsr#31       @ sign into r12b0
@ r12b2..1: quadrant count
@ r12b0: sign of reduced angle
 eors r7,r7,r8,asr#31
 eors r8,r8,r8,asr#31        @ absolute value of reduced angle 0≤r7:r8<0.125 Q64
 cmp r8,#1<<22               @ < 2^-10?
 blo 40b

@ 2π=6.487ED511 0B4611A6 (2633...)
 movlong r9,0x0B4611A6       @ 2π Q64 low fractional word
 umull r10,r0,r8,r9
 movlong r9,0x487ED511       @ 2π Q64 high fractional word
 umull r10,r1,r7,r9
 umaal r0,r1,r8,r9
 movs r9,#6                  @ 2π integer part
 umlal r0,r1,r7,r9
 mla r1,r8,r9,r1
@ r0:r1 range reduced angle θ 0..π/4 Q64

 cmp r1,#1<<25               @ θ < 2^-7?
 blo 30b
 lsrs r2,r1,#27
 ldr r3,=trigtab
 add r3,r3,r2,lsl#4
 ldmia r3,{r2-r5}
31:
 subs r0,r0,r2
 sbcs r1,r1,r3               @ ε=Θ-φ Q64
 bmi 2f                      @ ε negative?

 asrs r6,r12,#1
 bmi 5f                      @ doing dsincos?
 asrs r6,#1
 bcs 3f                      @ need cosine?

@ here ε is positive, we need the sin of θ and the sign of the result is r12b0^r12b2
 bl dsc_h0
 bl dsc_h0s
 bl dpack_q63
21:
 eors r12,r12,r12,lsr#2
 orrs r1,r1,r12,lsl#31       @ insert sign
 pop {r4-r10,r15}

2:
 asrs r6,r12,#1
 bmi 6f                      @ doing dsincos?
 asrs r6,#1
 bcs 4f                      @ need cosine?

@ here ε is negative, we need the sin of θ and the sign of the result is r12b0^r12b2
 bl dsc_h1
 bl dsc_h1s
 bl dnegpack_q63
 eors r12,r12,r12,lsr#2
 orrs r1,r1,r12,lsl#31       @ insert sign
 pop {r4-r10,r15}

@ here ε is positive, we need the cos of θ and the sign of the result is r12b2
3:
 bl dsc_h0
 bl dsc_h0c
 bl dnegpack_q63
22:
 ands r12,#4
 orrs r1,r1,r12,lsl#29       @ insert sign
 pop {r4-r10,r15}

@ here ε is negative, we need the cos of θ and the sign of the result is r12b2
4:
 bl dsc_h1
 bl dsc_h1c
15:
 bl dpack_q63
 ands r12,#4
 orrs r1,r1,r12,lsl#29       @ insert sign
 pop {r4-r10,r15}

5:
@ dsincos, ε positive
 bl dsc_h0
 push {r2-r7}
 bl dsc_h0c
 bl dnegpack_q63
 ands r4,r12,#4
 eors r1,r1,r4,lsl#29        @ negate cosine in quadrants 2 and 3
 pop {r2-r7}
 push {r0,r1}
 bl dsc_h0s
 bl dpack_q63
20:
 eors r4,r12,r12,lsr#1
 eors r4,r4,r12,lsr#2
 eors r1,r1,r4,lsl#31        @ negate sine on b0^b1^b2
 tst r12,#2                  @ exchange sine and cosine in odd quadrants
 ittte ne
 movne r2,r0
 movne r3,r1
 popne {r0,r1}
 popeq {r2,r3}
 ands r4,r12,#1<<30
 eors r1,r1,r4,lsl#1         @ negate sine result if argument was negative
 pop {r4-r10,r15}

6:
@ dsincos, ε negative
 bl dsc_h1
 push {r2-r7}
 bl dsc_h1c
 bl dpack_q63
 ands r4,r12,#4
 eors r1,r1,r4,lsl#29        @ negate cosine in quadrants 2 and 3
 pop {r2-r7}
 push {r0,r1}
 bl dsc_h1s
 bl dnegpack_q63
 b 20b

@ sin/cos power series for negative ε
dsc_h1:
 rsbs r0,r0,#0
 sbc r1,r1,r1,lsl#1
@ drop into positive ε code

@ sin/cos power series for positive ε
dsc_h0:
@ r0:r1 ε Q64
@ r4:   sin φ Q32
@ r5:   cos φ Q32
 umull r6,r7,r1,r1
 umull r2,r3,r0,r1
 lsrs r7,#1
 rrx r6,r6
 adds r2,r6,r3
 adc r3,r7,#0
@ r2:r3 ε²/2 Q64
 umull r7,r6,r3,r0
 umlal r7,r6,r2,r1
 movs r7,#0
 umlal r6,r7,r3,r1
@ r6:r7 ε³/2 Q64
 mov r8,#0x55555555
 umull r9,r6,r6,r8
 mov r9,#0
 umlal r6,r9,r7,r8
 adds r6,r6,r9
 adcs r7,r9,#0
@ r6:r7 ε³/6 Q64
 subs r6,r0,r6
 sbcs r7,r1,r7
@ r6:r7 ε-ε³/6 Q64
 mov r0,r3,lsl#12            @ ε²/2 Q44
 orr r0,r0,r2,lsr#20
 umull r0,r9,r0,r0           @ r9: ε⁴/4 Q88-32=Q56 = ε⁴/32 Q59
 umlal r0,r9,r9,r8           @ r9: ε⁴/24 Q59

 umull r0,r8,r9,r1           @ ε⁵/24 Q59
 mov r0,#0x33333333
 umull r0,r8,r8,r0           @ r8: ε⁵/120 Q59
@ r6:r7 ε-ε³/6+ε⁵/120 Q64
 smmulr r10,r8,r1            @ r10: ε⁶/120 Q59
 movlong r0,0x2aaaaaaa
 smmlsr r9,r0,r10,r9         @ ε⁴/24-ε⁶/720
 subs r2,r2,r9,lsl#5
 sbcs r3,r3,r9,lsr#27
@ r2:r3 ε²/2-ε⁴/24+ε⁶/720 Q64
 smmulr r10,r10,r1           @ r10: ε⁷/120 Q59
 mov r0,#0x6180000           @ 1/42 Q64
 smmlsr r8,r10,r0,r8         @ r8: ε⁵/120-ε⁷/5040 Q59
 adds r6,r6,r8,lsl#5
 adcs r7,r7,r8,lsr#27
 bx r14

dsc_h0s:
@ postprocess for sine, positive ε
@ r2:r3 1-cos |ε| Q64
@ r4:   sin φ Q31
@ r5:   cos φ Q31
@ r6:r7 sin |ε| Q64
 umull r8,r0,r2,r4
 mov r1,#0
 umlal r0,r1,r3,r4
@ r0:r1 sin φ(1-cos ε)
 umull r8,r3,r6,r5
 umlal r3,r4,r7,r5
@ r3:r4 sin φ+cos φ.sin ε
 subs r0,r3,r0
 sbc r1,r4,r1
@ r0:r1 sin φ+cos φ.sin ε - sin φ(1-cos ε) = sin φ.cos ε + cos φ.sin ε = sin(φ+ε)
 bx r14

dsc_h1s:
@ postprocess for sine, negative ε
@ r2:r3 1-cos |ε| Q64
@ r4:   sin φ Q31
@ r5:   cos φ Q31
@ r6:r7 sin |ε| Q64
 umull r8,r0,r2,r4
 mov r1,#0
 umlal r0,r1,r3,r4
@ r0:r1 sin φ(1-cos ε)
 umull r8,r3,r6,r5
 rsbs r4,r4,#0
 umlal r3,r4,r7,r5
@ r3:r4 -sin φ-cos φ.sin ε
 adds r0,r3,r0
 adc r1,r4,r1
@ r0:r1 -sin φ-cos φ.sin ε + sin φ(1-cos ε) = -sin φ.cos ε - cos φ.sin ε = -sin(φ+ε)
 bx r14

dsc_h0c:
@ postprocess for cosine, positive ε
@ r2:r3 1-cos |ε| Q64
@ r4:   sin φ Q32
@ r5:   cos φ Q32
@ r6:r7 sin |ε| Q64
 umull r8,r0,r2,r5
 mov r1,#0
 umlal r0,r1,r3,r5
@ r0:r1 cos φ(1-cos ε)
 umull r8,r3,r6,r4
 rsbs r5,#0
 umlal r3,r5,r7,r4
@ r3:r5 -cos φ+sin φ.sin ε
 adds r0,r3,r0
 adc r1,r5,r1
@ r0:r1 -cos φ+sin φ.sin ε + cos φ(1-cos ε) = - cos φ.cos ε + sin φ.sin ε = -cos(φ+ε)
 bx r14

dsc_h1c:
@ postprocess for cosine, negative ε
@ r2:r3 1-cos |ε| Q64
@ r4:   sin φ Q32
@ r5:   cos φ Q32
@ r6:r7 sin |ε| Q64
 umull r8,r0,r2,r5
 mov r1,#0
 umlal r0,r1,r3,r5
@ r0:r1 cos φ(1-cos ε)
 umull r8,r3,r6,r4
 umlal r3,r5,r7,r4
@ r3:r5 cos φ-sin φ.sin ε
 subs r0,r3,r0
 sbc r1,r5,r1
@ r0:r1 cos φ-sin φ.sin ε - cos φ(1-cos ε) = cos φ.cos ε - sin φ.sin ε = cos(φ+ε)
 bx r14

double_section dpack

@ dnegpack: negate and pack
@ dpack_q63:
@ input
@ r0:r1 Q63 result, must not be zero
@ r4    exponent offset [dpack_q only]
@ output
@ r0:r1 IEEE double
@ trashes r2,r3,r4
.thumb_func
dnegpack_q63:
 rsbs r0,r0,#0
 sbc r1,r1,r1,lsl#1
.thumb_func
dpack_q63:
 mov r4,#0x3ff-12            @ exponent
.thumb_func
dpack_q:
 clz r2,r1
 cmp r2,#12
 bhs 1f
 adds r2,#21
 lsls r3,r1,r2
 rsb r2,#32
 lsrs r0,r0,r2               @ save rounding bit in carry
 lsr r1,r1,r2
 add r2,r2,r4
 adcs r0,r0,r3               @ with rounding
 adc r1,r1,r2,lsl#20         @ insert exponent
 bx r14

1:
 cbz r1,2f
 rsb r2,#43
 lsrs r3,r0,r2
 rsb r2,#32
 lsls r1,r1,r2
 lsls r0,r0,r2
 orrs r1,r3
 sub r2,r4,r2
 add r1,r1,r2,lsl#20
 bx r14

2:
 movs r1,r0
 mov r0,#0
 sub r4,#32
 bne dpack_q
 bx r14



double_section dreduce

@ input:
@ r0:r1 x, double (only mantissa bits used)
@ r2    quotient offset
@ r3    exponent e of x with 0x3ff bias removed, -32≤e<12 so 2^-32≤x<2^12
@ r4:r5 r Q64, 0.5≤r<1
@ r6    1/r underestimate Q31
@ output:
@ r0:r1 x mod r Q64 [possibly slightly > r?]
@ r2    quotient+offset
@ r4:r5 r preserved
@ trashes r7,r8
@ increases r2 by up to 2^13
@ this version only used by dexp
.thumb_func
dreduce:
 movs r7,#1
 bfi r1,r7,#20,#12           @ insert implied 1, clear exponent and sign
 lsls r8,r7,r3
 beq 1f                      @ e<0, x<1
 umull r0,r7,r0,r8
 mla r1,r1,r8,r7
@ r0:r1 x Q52
 umull r7,r8,r1,r6           @ Q83-32=Q51
 lsrs r6,r8,#19              @ q Q0
 adds r2,r2,r6
 umull r7,r8,r6,r4
 mla r8,r6,r5,r8             @ r7:r8 q*r Q64
 lsls r1,#12
 orrs r1,r1,r0,lsr#20
 rsbs r0,r7,r0,lsl#12
 sbc r1,r1,r8                @ x-qr Q64
@ check we never return slightly more than r
 cmp r1,r5                   @ quick comparison
 it lo
 bxlo r14
 b 2f

1:
 adds r3,#12
 movs r7,#1
 lsls r8,r7,r3
 beq 1f                      @ e<-12
 umull r0,r7,r0,r8
 mla r1,r1,r8,r7             @ x Q64
 cmp r1,r5                   @ quick comparison
 it lo
 bxlo r14
2:
 it eq
 cmpeq r0,r4
 it lo
 bxlo r14
 subs r0,r0,r4               @ subtract one r
 sbc r1,r1,r5
 adds r2,#1
 bx r14

1:
@ here e<-12, have to shift r0:r1 down by -r3 places
 add r3,#32
 lsls r6,r1,r3
 rsbs r3,#32
 lsrs r0,r0,r3
 lsrs r1,r1,r3
 orrs r0,r0,r6
 bx r14

double_section exptab

.align 2
exptab0:
.quad 0x0000000000000000,0x0f85186008b15304,0x1e27076e2af2e5c8,0x2f57120421b2120d
.quad 0x3f7230dabc7c5512,0x4e993155a517a717,0x5fabe0ee0abf0d9d,0x6fad36769c6defe3
.quad 0x7ebd623de3cc7b69,0x8f42faf3820681f0,0x9ec813538ab7d537,0xaf70154920b3ab7f
exptab1:
.quad 0x0000000000000000,0x00ff805515885e02,0x01fe02a6b1067890,0x02fb88ebf0214edc
.quad 0x03f815161f807c7a,0x04f3a910d1a95d3c,0x05ee46c1f56c46aa,0x06e7f009ebe465ff
.quad 0x07e0a6c39e0cc013,0x08d86cc491ecbfe1,0x09cf43dcff5eafd5,0x0ac52dd7e4726a46
.quad 0x0bba2c7b196e7e23,0x0cae41876471f5bf,0x0da16eb88cb8df61,0x0e93b5c56d85a909
.quad 0x0f85186008b15331,0x1075983598e47130
exptab2:
.quad 0x000fff8005551559,0x002ffb808febc309,0x004ff3829a0e91b1,0x006fe78722fde71f
.quad 0x008fd78f299aa0c3,0x00afc39bac66434f,0x00cfabada9832a41,0x00ef8fc61eb4b74f
.quad 0x010f6fe6095f81b6,0x012f4c0e66898567,0x014f244032da521a,0x016ef87c6a9b3a48
.quad 0x018ec8c409b781ff,0x01ae95180bbc8d9c,0x01ce5d796bda1070,0x01ee21e924e23b3a
logtab0:
.quad 0xa0ec7f4233957338,0x918986bdf5fa1431,0x8391f2e0e6fa026b,0x7751a813071282e6
.quad 0x6a73b26a68212621,0x5fabe0ee0abf0d9d,0x546ab61cb7e0b419,0x48a507ef3de59695
.quad 0x3c4e0edc55e5cbd1,0x32a4b539e8ad68ce,0x289a56d996fa3ccb,0x21aefcf9a11cb2c9
.quad 0x16f0d28ae56b4b86,0x0f85186008b15304,0x07e0a6c39e0cc002,0x0000000000000000

.align 2
exprrdata:
.quad 0xB17217F7D1CF79AC     @ ln2 Q64
.long 0xB8AA3B29             @ 1/ln2 Q31, rounded down

double_wrapper_section exp

2:
@ could use dadd macro to calculate x+1 here
 lsl r0,r1,#11
 orr r0,#0x80000000
 lsls r1,#1
 adc r3,r3,#32
 movlong r1,0x3ff00000
 rsb r3,#11
 lsr r0,r3
 it cc
 bxcc r14
 rsbs r0,#0
 sbc r1,r1,#0
 bx r14

wrapper_func exp
 movs r12,r1,lsr#31          @ save sign
 ubfx r3,r1,#20,#11          @ get exponent
 sub r3,r3,#0x3ff
 cmp r3,#12
 bge 20f                     @ overflow, Inf or NaN?
 cmn r3,#0x20
 ble 2b                      @ <2^-32? return x+1
 push {r4-r8,r14}
 ldr r4,=exprrdata
 ldmia r4,{r4-r6}
 mov r2,#0
 bl dreduce
 tst r12,#1
 beq 1f
 mvn r2,r2                   @ quotient is now signed
 subs r0,r4,r0
 sbc r1,r5,r1
1:
 add r12,r2,#0x3fe           @ exponent offset
 mov r3,#0x7fe
 cmp r12,r3
 bhs 1f                      @ under/overflow
 lsrs r2,r1,#28
 ldr r3,=exptab0
 add r3,r3,r2,lsl#3
 ldmia r3,{r2-r3}
 and r5,r2,#63
 orr r5,#64                  @ y=(t&0x3f)+0x40; Q6
 subs r0,r2
 sbcs r1,r3
 lsrs r2,r1,#24
 ldr r3,=exptab1
 add r3,r3,r2,lsl#3
 ldmia r3,{r3-r4}
 add r2,#256
 muls r5,r5,r2               @ y Q14
 subs r0,r3
 sbcs r1,r4
 lsrs r2,r1,#21
 ldr r3,=exptab2
 add r3,r3,r2,lsl#3
 ldmia r3,{r3-r4}
 add r2,r2,r2
 add r2,#4096
 mla r5,r5,r2,r5             @ y Q26
 subs r0,r3
 sbcs r1,r4

 movs r2,r1,lsl#10
 orrs r2,r2,r0,lsr#22
 adc r2,r2,#0                @ ε Q42, rounded
 smull r3,r4,r2,r2           @ ε² Q84-32=Q52
 lsrs r3,#21
 orrs r3,r3,r4,lsl#11
 adds r0,r0,r3
 adc r1,r1,r4,lsr#21         @ ε+ε²/2 Q64
 smull r3,r4,r4,r2           @ Q52*Q42=Q94; Q94-32=Q62
 mov r3,#0x55555555          @ 1/6 Q33
 smull r3,r4,r3,r4           @ ε³/6 Q63
 smmulr r3,r4,r1             @ ε⁴/6+ε⁵/12 Q63+Q32-32=Q63
 add r4,r4,r3,lsr#2
 adds r2,r0,r4,lsl#1
 adc r3,r1,r4,asr#31
 lsls r1,r5,#3               @ y Q29
 umull r4,r0,r1,r2           @ εlo * y Q61+32
 smlal r0,r1,r1,r3           @ εhi * y + y Q61
@ assert result is in range 1..2
 lsrs r0,#9
 adcs r0,r0,r1,lsl#23
 lsr r1,#9
 adcs r1,r1,r12,lsl#20
 pop {r4-r8,r15}

20:
@ process Inf/NaN for dexp
 cmp r3,#0x400
 bne 22f
 orrs r2,r0,r1,lsl#12
 ite eq
 biceq r1,r1,r1,asr#31       @ +Inf -> + Inf; -Inf -> +0
 orrne r1,r1,#0x00080000
 bx r14

22:
 movs r0,#0
 movs r1,#0
 tst r12,#1
 it eq
 movteq r1,0x7ff0
 bx r14

1:                           @ under/overflow
 mov r0,#0
 mov r1,#0
 it ge
 movtge r1,#0x7ff0
 pop {r4-r8,r15}




double_wrapper_section log

1:
 movlong r1,0xfff00000       @ return -Inf
 movs r0,#0
 bx r14

4:
 orrs r2,r0,r1,lsl#12
 it ne
 orrne r1,#0x00080000
 bx r14

10:
 mvns r5,r6,asr#22           @ very small argument?
 bne 10f
 mov r4,#4096
 b 11f

@ check for argument near 1: here
@ r1 : mantissa
@ r12: exponent, -1 or 0
12:
 eor r3,r12,r1,lsr#12
 lsls r3,r3,#24              @ check 8 bits of mantissa
 bne 12f                     @ not very close to 1
 cmp r12,#0
 bne 13f
@ argument is 1+ε, result will be positive
 lsls r1,#19
 orrs r1,r1,r0,lsr#13
 lsls r0,#19
@ r0:r1 ε Q71 0≤ε<2^-8
 clz r4,r1                   @ r4≥1
 cmp r4,#32
 bhs 14f
 movs r5,#1
 lsls r5,r4
 umull r2,r3,r0,r5
 mla r3,r1,r5,r3             @ r2:r3 ε Q71+r4
 umull r12,r5,r0,r3
 umull r12,r6,r1,r2
 umaal r5,r6,r1,r3           @ r5:r6 ε² Q142+r4-64 = Q78+r4

 subs r2,r2,r5,lsr#8
 sbc r3,r3,#0
 subs r2,r2,r6,lsl#24
 sbcs r3,r3,r6,lsr#8

 umull r12,r7,r0,r6
 umull r12,r8,r1,r5
 umaal r7,r8,r1,r6           @ r7:r8 ε³ Q149+r4-64 = Q85+r4: when ε is nearly 2^-8, r4=1 and Q86, so r8<0x40000000
 mov r5,#0x55555555          @ ~1/3 Q32

 umull r12,r6,r7,r5
 movs r12,#0
 umlal r6,r12,r8,r5
 adds r6,r6,r12
 adc r12,r12,#0              @ multiply by 0x5555555555555555

 adds r2,r2,r6,lsr#14
 adc r3,r3,#0
 adds r2,r2,r12,lsl#18
 adc r3,r3,r12,lsr#14

 smmulr r5,r8,r1             @ ε⁴ Q53+r4+Q71-Q64=Q60+r4 ~ 2^-32
 movs r7,#0x33333333         @ 1/5 Q32
 smmulr r6,r5,r1             @ ε⁵ Q60+r4+q71-Q64=Q67+r4 ~ 2^-40
 smmulr r8,r6,r7             @ ε⁵/5 Q67+r4 ~ 2^-42
 sub r7,r5,r8,lsr#5
 smmulr r5,r6,r1             @ ε⁶ Q67+r4+q71-Q64=Q74+r4 ~ 2^-48
 movt r6,#0x2a80             @ 1/6 Q32 fiddled slightly (PMC)
 smmulr r5,r6,r5             @ ε⁶/6 Q75+r4 ~ 2^-50
 add r7,r7,r5,lsr#12

 subs r0,r2,r7,lsl#9
 sbc r1,r3,r7,lsr#23

 rsb r4,#0x400
 sub r4,#0x15
 bl dpack_q
 pop {r4-r8,r15}

@ here we have positive ε sufficiently small we need (at most) a quadratic term
14:
@ here r0=ε Q71, 0≤ε<2^-40
 clz r4,r0
 lsls r1,r0,r4               @ ε Q71+r4
 umull r2,r3,r0,r1           @ ε² Q142+r4
 mov r0,#0
 subs r0,r0,r3,lsr#8
 sbc r1,r1,#0
 rsb r4,#0x400
 sub r4,#0x35
 bl dpack_q
 pop {r4-r8,r15}



13:                          @ argument is 1-ε, result will be negative
 movs r1,r1,lsl#18
 orrs r1,r1,r0,lsr#14
 movs r0,r0,lsl#18
 rsbs r0,#0
 sbc r1,r1,r1,lsl#1
@ r0:r1 -ε Q71 -2^-9≤ε<0
 clz r4,r1
 cmp r4,#32
 bhs 15f
 subs r4,#1                  @ 0≤r4<31
 movs r5,#1
 lsls r5,r4
 umull r2,r3,r0,r5
 mla r3,r1,r5,r3             @ r2:r3 ε Q71+r4
 umull r12,r5,r0,r3
 umull r12,r6,r1,r2
 umaal r5,r6,r1,r3           @ r5:r6 ε² Q142+r4-64 = Q78+r4

 adds r2,r2,r5,lsr#8
 adc r3,r3,#0
 adds r2,r2,r6,lsl#24
 adcs r3,r3,r6,lsr#8

 umull r12,r7,r0,r6
 umull r12,r8,r1,r5
 umaal r7,r8,r1,r6           @ r7:r8 ε³ Q149+r4-64 = Q85+r4: when ε is nearly 2^-8, r4=0 and Q85, so r8<0x20000000
 mov r5,#0x55555555          @ ~1/3 Q32

 umull r12,r6,r7,r5
 movs r12,#0
 umlal r6,r12,r8,r5
 adds r6,r6,r12
 adc r12,r12,#0              @ multiply by 0x5555555555555555

 adds r2,r2,r6,lsr#14
 adc r3,r3,#0
 adds r2,r2,r12,lsl#18
 adc r3,r3,r12,lsr#14

 smmulr r5,r8,r1             @ ε⁴ Q53+r4+Q71-Q64=Q60+r4 ~ 2^-32
 movs r7,#0x33333333         @ 1/5 Q32
 smmulr r6,r5,r1             @ ε⁵ Q60+r4+q71-Q64=Q67+r4 ~ 2^-40
 smmulr r8,r6,r7             @ ε⁵/5 Q67+r4 ~ 2^-42
 add r7,r5,r8,lsr#5
 smmulr r5,r6,r1             @ ε⁶ Q67+r4+q71-Q64=Q74+r4 ~ 2^-48
 movt r6,#0x2a80             @ 1/6 Q32 fiddled slightly (PMC)
 smmulr r5,r6,r5             @ ε⁶/6 Q75+r4 ~ 2^-50
 add r7,r7,r5,lsr#12

 adds r0,r2,r7,lsl#9
 adc r1,r3,r7,lsr#23

 rsb r4,#0x400
 sub r4,#0x15
 bl dpack_q
 orr r1,r1,#1<<31
 pop {r4-r8,r15}

@ here we have negative ε sufficiently small we need (at most) a quadratic term
@ here r0=ε Q71, |ε|<2^-41
15:
 clz r4,r0
 lsls r1,r0,r4               @ ε Q71+r4
 umull r2,r3,r0,r1           @ ε² Q142+r4
 mov r0,r3,lsr#8
 rsb r4,#0x400
 sub r4,#0x35
 bl dpack_q
 eors r1,r1,#0x80000000
 pop {r4-r8,r15}

wrapper_func log
 lsls r12,r1,#1
 bcs 1b                      @ x<0?
 lsrs r12,#21
 beq 1b                      @ x==0/denormal?
 sub r12,#0x3ff
 cmp r12,#0x400              @ +Inf/NaN?
 beq 4b
 movs r2,#1
 bfi r1,r2,#20,#12           @ set implied 1, clear exponent Q52
 push {r4-r8,r14}
 cmp r12,r12,asr#31          @ exponent = -1 or 0?
 beq 12b
12:
 lsrs r4,r1,#16
 ldr r5,=logtab0-16*8
 add r5,r5,r4,lsl#3
 ldmia r5,{r2-r3}
 and r5,r2,#63
 add r5,#64
 umull r0,r6,r5,r0
 mla r1,r5,r1,r6             @ Q59

 mvn r4,r1,asr#19
 and r4,#31
 ldr r5,=exptab1
 add r5,r5,r4,lsl#3
 ldmia r5,{r5-r6}
 adds r2,r2,r5
 adc r3,r3,r6
 add r4,#256
 umull r0,r6,r4,r0
 mla r6,r4,r1,r6             @ r0:r6 Q67

 mvns r4,r6,asr#24
 beq 10b                     @ small argument at this stage?
10:
 ldr r5,=exptab2
 add r5,r5,r4,lsl#3
 ldmia r5,{r1,r5}
 adds r2,r2,r1
 adc r3,r3,r5
 mov r5,#4097
 add r4,r5,r4,lsl#1          @ 4097+2k
11:
 lsls r4,#17
 umull r5,r0,r4,r0
 rsb r1,r4,r4,lsl#3
 umlal r0,r1,r4,r6           @ Q96=Q64

@ r0:r1 ε Q64
@ r2:r3 y Q64

 eor r4,r0,r1,asr#31
 eor r5,r1,r1,asr#31         @ r4:r5 |ε| Q64
 umull r6,r7,r5,r5
 umull r4,r8,r4,r5
 lsrs r7,#1
 rrx r6,r6
 adds r6,r6,r8
 adc r7,r7,#0                @ r6:r7 ε²/2 Q64

 movs r4,r1,lsl#10
 orrs r4,r4,r0,lsr#22
 adc r4,r4,#0                @ ε Q42, rounded

 subs r0,r0,r6
 sbc r1,r1,r7                @ r0:r1 ε-ε²/2 Q64

 smmulr r5,r4,r4             @ ε² Q42+42-32=Q52
 smmulr r6,r5,r4             @ ε³ Q52+42-32=Q62
 smmulr r7,r5,r5             @ ε⁴ Q52+52-32=Q72 ε⁴/4 Q74
 mov r4,#0x55555555          @ 1/3 Q32
 smmulr r6,r6,r4             @ ε³/3 Q62
 subs r6,r6,r7,lsr#12        @ ε³/3-ε⁴/4 Q62

 adds r0,r0,r6,lsl#2         @ Q64
 adc r1,r1,r6,asr#30

 subs r0,r2,r0
 sbc r1,r3,r1
 ldr r2,=exprrdata
 ldmia r2,{r2,r5}
 adds r12,#1
 ble 1f
@ positive result
 umull r2,r3,r2,r12
 movs r4,#0
 umlal r3,r4,r5,r12
 subs r2,r2,r0
 sbcs r3,r3,r1
 sbc r4,r4,#0
 movs r1,#0x40000000
 b 2f                        @ to pack result

@ negative result
1:
 rsbs r12,#0
 umull r2,r3,r2,r12
 movs r4,#0
 umlal r3,r4,r5,r12
 adds r2,r0,r2
 adcs r3,r1,r3
 adc r4,r4,#0
 movs r1,#0xc0000000
2:
 cbnz r4,2f
 movs r4,r3
 movs r3,r2
 movs r2,#0
 subs r1,#32<<20
1:
@ here r3:r4 is guaranteed nonzero
 cbnz r4,2f
 movs r4,r3
 movs r3,#0
 subs r1,#32<<20
2:
 clz r5,r4
 sub r6,r5,#0x1d
 sub r1,r1,r6,lsl#20
 lsls r4,r5
 lsls r0,r3,r5
 rsb r5,#32
 lsrs r3,r5
 orrs r4,r4,r3
 lsrs r2,r5
 orrs r0,r0,r2
@ now r0:r4 is normalised to Q63
 lsrs r0,#11
 adcs r0,r0,r4,lsl#21        @ with rounding
 adc r1,r1,r4,lsr#11
 pop {r4-r8,r15}

@===========================================

double_section trigtab
trigtab:
//     φ Q64 lo    φ Q64 hi    sin φ Q31   cos φ Q31
.long 0x25735c0b, 0x03f574a9, 0x01fab529, 0x7ffc1500
.long 0x00aa2feb, 0x0c44d954, 0x0621d2c8, 0x7fda601f
.long 0x42e86336, 0x13d9d3cf, 0x09ea5e3c, 0x7f9d88f0
.long 0x046dc42f, 0x1c0a86d9, 0x0dfe171f, 0x7f3b9ed0
.long 0xec509ba7, 0x23ebf53e, 0x11e6e7bc, 0x7ebdefc7
.long 0x039c1cd2, 0x2bf112b2, 0x15dcf546, 0x7e1e7749
.long 0x3d7a05ca, 0x33f293f4, 0x19cbc014, 0x7d5fac9c
.long 0x7aefa0a0, 0x3c19321d, 0x1dc6221d, 0x7c7d2f2f
.long 0x12fb0fb5, 0x4450ef70, 0x21c10c53, 0x7b782235
.long 0x476b8d5c, 0x4bf1a045, 0x256adc18, 0x7a68ad16
.long 0x576940c1, 0x53ead96d, 0x29361639, 0x792f2d3c
.long 0xd34eeadf, 0x5bb34289, 0x2ce039d6, 0x77e02625
.long 0x31ea5069, 0x641107d9, 0x30c4d3c0, 0x76586270
.long 0xf36756cb, 0x6bfda2b3, 0x34687e1c, 0x74c77a22
.long 0x2f7aed0e, 0x73e07531, 0x37fae7cc, 0x731c1127
.long 0xfcc48ec0, 0x7c14790e, 0x3ba3a70e, 0x7141cd8e
.long 0x3b04b713, 0x83547b8f, 0x3ed289d4, 0x6f85d8eb
.long 0x135b369a, 0x8c0b6fd9, 0x4294ead4, 0x6d51efa3
.long 0x4aca525f, 0x9439d326, 0x460a6e70, 0x6b230540
.long 0x41c44083, 0x9c0e0b34, 0x4948b03d, 0x68f1ef07
.long 0xa36d08bf, 0xa47961fd, 0x4cb1f285, 0x667a849d
.long 0xc8f81636, 0xabe547d2, 0x4fa2221e, 0x6436622f
.long 0xd0c8c9ad, 0xb3feade1, 0x52c37024, 0x61a4af86
.long 0x64730e73, 0xbbfe356a, 0x55c5f028, 0x5f02a3a3
.long 0xb08ca5c6, 0xc412b955, 0x58ba9208, 0x5c4194a3
.long 0x54530090, 0xcbb02d37, 0x5b6ef419, 0x59938ed8
.long 0xa18d7c36, 0xd3aa3c6e, 0x5e2e0225, 0x56af32fc
.long 0x48a2c7d0, 0xdbff19f6, 0x60f35332, 0x5392ed7e
.long 0x7aad9a94, 0xe422ade1, 0x638edfca, 0x50732bc7
.long 0x19ef15ff, 0xebfb85bd, 0x65fa18bb, 0x4d5c61c7
.long 0x7e0f96bd, 0xf44c4b49, 0x686f81f7, 0x4a0217e6
.long 0x1def09eb, 0xfc4dbdfd, 0x6ab2d2c4, 0x46b4e413
// maximum e=0.002617 = 0.167497*2^-6

double_wrapper_section atan2

@ datan small reduced angle case I
20:
@ r0:r1 has z=y/x in IEEE format, <2^-11
@ r2 e+11
 rsbs r12,r2,#0              @ shift down of mantissa required to get to Q63 >0
 subs r10,r12,#32
 bge 1f                      @ very small reduced angle?
 bfi r1,r3,#20,#12           @ fix up mantissa
 cmp r7,#4
 bhs 2f                      @ at least one quadrant to add? then don't need extreme accuracy
@ otherwise we need to do a power series for accuracy

 rsbs r10,#0
 lsr r3,r1,r12
 lsr r2,r0,r12
 lsl r9,r1,r10
 orr r2,r9
@ r2:r3 z Q63 (with r12 bits of loss of precision)
 lsls r1,#11
 orrs r1,r1,r0,lsr#21
 lsls r0,#11
@ r0:r1 z Q74+r12
 umull r9,r4,r2,r3
 umull r9,r5,r2,r3
 umaal r4,r5,r3,r3
@ r4:r5 z² Q62 < 2^-22
 umull r9,r2,r0,r5
 umull r9,r3,r1,r4
 umaal r2,r3,r1,r5
@ r2:r3 z³ Q72+r12 <2^-33
 umull r9,r6,r2,r5
 umull r9,r8,r3,r4
 umaal r6,r8,r3,r5
@ r6:r8 z⁵ Q70+r12 <2^-55; in fact r8 is always 0
 mov r9,#0xaaaaaaaa          @ 2/3 Q32
 umull r10,r4,r2,r9
 movs r5,#0
 umlal r4,r5,r3,r9
 adds r4,r4,r5
 adcs r5,r5,#0
@ r4:r5 z³*2/3 Q72+r12 = z³/3 Q73+r12
 mov r9,#0x33333333          @ 1/5 Q32
 umull r2,r3,r6,r9
@ r3 z⁵*1/5 Q70+r12
 subs r4,r4,r3,lsl#3
 sbc r5,r5,#0
@ r4:r5 z³/3-z⁵/5 Q73+r12
 subs r0,r0,r4,lsl#1
 sbc r1,r1,r5,lsl#1
 sub r1,r1,r4,lsr#31
@ r0:r1 z-z³/3+z⁵/5 Q74+r12
 rsb r4,r12,#0x400
 sub r4,#24
 b 60f                       @ pack and return

2:
 rsbs r10,#0
 lsls r4,r1,r10
 lsrs r0,r0,r12
 lsrs r1,r1,r12
 orrs r0,r0,r4               @ shift down r12 places
 b 50f


1:
 cmp r7,#4
 bhs 2f                      @ at least one quadrant to add?
 eors r1,r1,r7,lsl#31        @ no: return y/x with the correct sign
 pop {r4-r11,r15}

2:
 bfi r1,r3,#20,#12           @ fix up mantissa
 usat r10,#6,r10             @ saturate r10 to 63
 lsrs r0,r1,r10
 movs r1,#0                  @ shift down 32+r10 places
 b 40f

@ datan small reduced angle case II
10:
 lsrs r1,#1
 rrxs r0,r0
 movs r2,#0
 movs r3,#0
 movs r6,#0
 mov r7,#1<<30
 b 11f

@ case where reduced (x',y') has x' infinite
71:
 sbfx r4,r1,#20,#11
 movs r0,#0
 movs r1,#0
 cmn r4,#1                   @ y' also infinite?
 bne 80f
 movt r1,#0x3ff0             @ both infinite: pretend ∞/∞=1
 b 80f

@ case where reduced (x',y') has y' zero
70:
 ubfx r4,r3,#20,#11
 movs r0,#0
 movs r1,#0
 cbnz r4,80f                 @ x' also zero?
 tst r7,#4
 beq 80f                     @ already in quadrants 0/±2? then 0/0 result will be correct
 tst r7,#2
 ite eq
 addeq r7,#6
 subne r7,#6                 @ fix up when in quadrants ±0
 b 80f

90:
 movs r0,r2
 movs r1,r3
91:
 orrs r1,r1,#0x00080000
 bx r14

wrapper_func atan2
 cmp r2,#1                   @ set C if low word is non-zero
 adc r12,r3,r3
 cmn r12,#0x00200000         @ y NaN?
 bhi 90b
 cmp r0,#1                   @ set C if low word is non-zero
 adc r12,r1,r1
 cmn r12,#0x00200000         @ x NaN?
 bhi 91b
 push {r4-r11,r14}
 lsrs r7,r1,#31              @ b31..2: quadrant count; b1: sign to apply before addition; b0: sign to apply after addition
 bic r1,#1<<31
@ y now positive
 movs r3,r3
 bpl 1f

@ here y positive, x negative
 adds r7,#10
 bic r3,r3,#1<<31
 cmp r3,r1
 bhi 4f                      @ |x| > y: 3rd octant
@ |x| < y: 2nd octant
 subs r7,#6
 b 3f

1:
@ here x and y positive
 cmp r3,r1
 bhi 4f
@ x < y: 1st octant
 adds r7,#6
3:
 movs r4,r2                  @ exchange x and y
 movs r5,r3
 movs r2,r0
 movs r3,r1
 movs r0,r4
 movs r1,r5
4:

@ here
@ r0:r1 y'
@ r2:r3 x'
@ where both x' and y' are positive, y'/x' < 1+δ, and the final result is
@ ± (Q.π/2 ± atn y/x) where 0≤Q≤2 is a quadrant count in r7b3..2, the inner negation
@ is given by r7b1 and the outer negation by r7b0. x' can be infinite, or both x' and
@ y' can be infinite, but not y' alone.

 sbfx r4,r3,#20,#11
 cmn r4,#1
 beq 71b                     @ x' infinite?
 ubfx r4,r1,#20,#11
 cmp r4,#0
 beq 70b                     @ y' zero?
 bl __aeabi_ddiv
80:
@ r0:r1 y/x in IEEE format, 0..1
 lsr r2,r1,#20               @ exponent
 movs r3,#1
 subs r2,#0x3ff-11
 bmi 20b
 bfi r1,r3,#20,#12           @ fix up mantissa
 movs r3,#1
 lsl r3,r2
 umull r0,r4,r0,r3
 mla r1,r1,r3,r4
50:
 push {r7}                   @ save flags

@ from here atan2(y,1) where 1 implied
@ r0:r1 y Q63 0≤y<1+δ

 lsrs r2,r1,#16
 cmp r2,#0x100
 blo 10b                     @ small angle?
 mul r3,r2,r2                @ y^2
 movw r4,#0x895c
 muls r2,r2,r4               @ y*0x895c
 movw r5,#0x1227
 lsrs r3,#14
 mls r2,r3,r5,r2
 subs r2,#0x330000           @ Chebyshev approximation to atn(y)
 lsrs r2,#25
 ldr r3,=trigtab
 add r3,r3,r2,lsl#4
 ldmia r3,{r2-r5}
 lsrs r3,#1
 rrxs r2,r2
@ r2:r3 phi0 Q63
@ r4    sphi0
@ r5    cphi0
 umull r12,r6,r4,r0
 movs r7,#0
 umlal r6,r7,r4,r1
 adds r6,r6,r5,lsl#31
 adc r7,r7,r5,lsr#1          @ x0= ((i128)cphi0<<31)+(((i128)sphi0*(i128)y)>>32); // Q62
@ r6:r7 x0
 umull r12,r0,r5,r0
 movs r8,#0
 umlal r0,r8,r5,r1
 subs r0,r0,r4,lsl#31
 sbc r1,r8,r4,lsr#1          @ y0=-((i128)sphi0<<31)+(((i128)cphi0*(i128)y)>>32); // Q62
11:
@ r0:r1 y0
@ r2:r3 phi0
@ r6:r7 x0

 lsls r4,r1,#6
 orr r4,r4,r0,lsr#26
 lsrs r5,r7,#15
 sdiv r4,r4,r5               @  t2=(y0>>26)/(x0>>47); // Q62-26/Q62-47=Q21

 mul r5,r4,r4                @ t2_2 Q42
 add r3,r3,r4,lsl#10         @ phi0+t2
 smull r8,r9,r4,r5           @ t2_3 Q63
 mov r10,r9,lsl#16
 orr r10,r10,r8,lsr#16
 smmulr r10,r10,r5           @ t2_5 Q57
 mov r12,#0x66666666         @ 1/5 Q33
 smmulr r11,r10,r5           @ t2_7 Q67
 smmulr r10,r10,r12          @ t2_5/5 Q57+33-32=Q58

 movlong r12,0x124925        @ 1/7 Q23
 smmulr r11,r11,r12          @ t2_7/7 Q67+23-32=Q58
 mov r12,#0x55555555
 sub r11,r11,r11,asr#12      @ Q58 PMC correction
 sub r10,r10,r11

 adds r2,r2,r10,lsl#5
 adc r3,r3,r10,asr#27        @ Q63 phi0 + t_2 + t2_5/5 - t2_7/7 + t2_7/7/4096
 umull r5,r10,r8,r12
 mov r11,#0
 smlal r10,r11,r9,r12        @ t2_3 * 0x55555555
 adds r10,r10,r11
 adc r11,r11,r11,asr#31      @ t2_3/3 Q63
 subs r2,r2,r10
 sbc r3,r3,r11               @ Q63 phi0+phi1

 lsls r4,r4,#11              @ t2 Q32
 umull r5,r8,r4,r0           @ t2*y0l
 it mi
 submi r8,r8,r0              @ correction if t2 is negative
 mov r9,r8,asr#31            @ sign extend
 smlal r8,r9,r4,r1           @ t2*y0h
@ r8:r9 (t2*y0)<<11

 umull r5,r10,r4,r6          @ t2*x0l
 it mi
 submi r10,r10,r6            @ correction if t2 is negative
 mov r11,r10,asr#31          @ sign extend
 smlal r10,r11,r4,r7
@ r10:r11 (t2*x0)<<11

 adds r6,r8
 adc r7,r7,r9
 subs r0,r10
 sbc r1,r1,r11
@ r0:r1 y1=y0-t2*x0
@ r2:r3 phi0+phi1
@ r6:r7 x1=x0+t2*y0

 mov r4,#0xffffffff
 lsrs r5,r7,#14
 udiv r4,r4,r5               @ rx1 Q16
 lsrs r5,r0,#11
 orrs r5,r5,r1,lsl#21        @ N set according to y1, hence also t3
 smmul r5,r4,r5              @ t3=(y1>>11)*rx1 Q35
 lsr r6,r6,#3
 orr r6,r6,r7,lsl#29
 umull r11,r8,r5,r6          @ t3*x1l
 lsr r10,r7,#3
 it mi
 submi r8,r8,r6              @ correction if t3 is negative
 mla r8,r5,r10,r8
 adds r2,r2,r5,lsl#28
 adc r3,r3,r5,asr#4
 sub r0,r0,r8
@ r0: y2
@ r2:r3 phi0+phi1+phi2
@ r4: rx1
@ r5: t3

 smull r8,r9,r0,r4           @ y2*rx1
@ stall
 lsrs r8,#14
 orr r8,r8,r9,lsl#18         @ t4
 smmlsr r0,r8,r7,r0
 adds r2,r2,r8,asr#1
 adc r3,r3,r8,asr#31
@ r0: y3
@ r4: rx1
 mul r4,r4,r0
 adds r0,r2,r4,asr#15
 adc r1,r3,r4,asr#31
@ r0:r1 result over reduced range Q63
 pop {r7}                    @ restore flags
40:
 lsrs r1,#1
 rrxs r0,r0
@ r0:r1 result over reduced range Q62
 lsl r6,r7,#30               @ b1 -> b31
 eor r0,r0,r6,asr#31         @ negate if required
 eor r1,r1,r6,asr#31
 movlong r2,0x10B4611A       @ π/2 Q62 low word
 movlong r3,0x6487ED51       @ π/2 Q62 high word
 lsr r6,r7,#2                @ quadrants to add
 umlal r0,r1,r6,r2
 mla r1,r6,r3,r1
 mov r4,#0x400-12            @ for packing Q62
60:
 bl dpack_q
 eors r1,r1,r7,lsl#31
 pop {r4-r11,r15}

#endif
