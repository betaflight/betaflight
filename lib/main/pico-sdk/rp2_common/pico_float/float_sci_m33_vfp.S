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

@ load a 32-bit constant n into register rx
.macro movlong rx,n
 movw \rx,#(\n)&0xffff
 movt \rx,#((\n)>>16)&0xffff
.endm

float_section frrcore_v

// 1/2π to plenty of accuracy
.long 0                      @ this allows values of e down to -32
rtwopi:
.long 0,0
.long 0x28BE60DB, 0x9391054A, 0x7F09D5F4, 0x7D4D3770, 0x36D8A566, 0x4F10E410

@ input:
@ r0 mantissa m Q23
@ r1 exponent e>=-32, typically offset by +9
@ output:
@ r0..r1 preserved
@ r6 range reduced result in revolutions Q32
@ r2,r3,r4,r5 trashed
.thumb_func
frr_core:
 adr r2,rtwopi
 asrs r3,r1,#5               @ k=e/32, k<=5 for e offsets up to 9+32
 add r2,r2,r3,lsl#2          @ p
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

wrapper_func expf
@ soft float version, via 2^x
 asrs r1,r0,#23
 bmi 1f
 cmp r1,#0x85
 bge 3f
10:
 movs r2,#1
 bfi r0,r2,#23,#9
 subs r1,#0x7e
 bmi 2f
 lsl r0,r1                   @ x Q24
11:
 movlong r3,0x5c551d95       @ 1/log(2) Q30
 smull r0,r1,r0,r3           @ Q54
 adr r2,k_exp2
 vldmia r2,{s8-s10}
 lsrs r2,r0,#22
 bfi r2,r1,#10,#17           @ ε Q32
 vmov s0,r2
 vcvt.f32.u32 s0,s0,#32
 vmul.f32 s1,s0,s0
 ubfx r2,r1,#17,#5
 vmul.f32 s4,s0,s8
 adr r3,exptab3
 vmul.f32 s2,s0,s1
 ldr r2,[r3,r2,lsl#2]
 vmla.f32 s4,s1,s9
 asrs r1,#22
 vmla.f32 s4,s2,s10
 add r2,r2,r1,lsl#23
 vmov s0,r2
 vmla.f32 s0,s0,s4
 vmov r0,s0
 bx r14

2:                           @ x≤0.5
 rsbs r1,#0
 lsrs r0,r1
@ adc r0,#0                  @ rounding not needed
 b 11b

3:                           @ risk of overflow, Inf,NaN
 movlong r2,0x42B17218
 cmp r0,r2
 blo 10b                     @ in range after all
 cmp r0,#0x7f800000
 bls 4f                      @ not NaN?
 orrs r0,#0x00400000
 bx r14

4:
 movlong r0,0x7f800000       @ return +Inf
 bx r14

1:                           @ x<0, r1=0xffffffXX where XX is biased exponent
 cmn r1,#0x7b
 bge 5f                      @ risk of underflow, -Inf, -NaN?
13:
 movs r2,#1
 bfi r0,r2,#23,#9
 adds r1,#0x82
 bpl 6f
 rsbs r1,#0
 lsrs r0,r1
 adc r0,r0,#0                @ rounding
12:
 rsbs r0,#0
 b 11b

6:
 lsls r0,r1
 b 12b

5:
 movlong r2,0xC2AEAC4F
 cmp r0,r2
 bls 13b
 cmp r0,#0xff800000
 bls 14f
 orrs r0,#0x00400000
 bx r14
14:
 mov r0,#0
 bx r14

.align 3
k_exp2:
.float 0.693147181           @ log2
.float 0.240226507           @ log²2/2
.float 0.055504109           @ log³2/6

exptab3:                     @ pow(2,[0..31]/32)
.word 0x3f800000
.word 0x3f82cd87
.word 0x3f85aac3
.word 0x3f88980f
.word 0x3f8b95c2
.word 0x3f8ea43a
.word 0x3f91c3d3
.word 0x3f94f4f0
.word 0x3f9837f0
.word 0x3f9b8d3a
.word 0x3f9ef532
.word 0x3fa27043
.word 0x3fa5fed7
.word 0x3fa9a15b
.word 0x3fad583f
.word 0x3fb123f6
.word 0x3fb504f3
.word 0x3fb8fbaf
.word 0x3fbd08a4
.word 0x3fc12c4d
.word 0x3fc5672a
.word 0x3fc9b9be
.word 0x3fce248c
.word 0x3fd2a81e
.word 0x3fd744fd
.word 0x3fdbfbb8
.word 0x3fe0ccdf
.word 0x3fe5b907
.word 0x3feac0c7
.word 0x3fefe4ba
.word 0x3ff5257d
.word 0x3ffa83b3

float_wrapper_section logf

wrapper_func logf
 cmp r0,#0x7f800000          @ catch Inf, NaN, -ve
 bhs 1f
 asrs r1,r0,#23              @ get exponent; C from here is preserved...
 beq 2f                      @ ±0?
 mov r2,#1
 bfi r0,r2,#23,#9            @ fix implied 1
 it cc                       @ 50% ... to here...
 lslcc r0,#1                 @ this plus sbc below means we work relative to nearby power of 2
 adr r3,#k_log3
 vldmia r3,{s8-s10}
@ 0x00c00000 ≤ r0 < 0x017fffff
 adr r3,logtab3-24*8+4
 add r3,r3,r0,lsr#16         @ look up r0>>19 rounded, preserving flags
 bic r3,#7

 ldrd r2,r3,[r3]
 mul r0,r0,r2                @ ε
 vmov s0,s1,r3,r0            @ s0=-log u, s1=ε

 vcvt.f32.s32 s1,s1,#32
 vmul.f32 s2,s1,s1           @ power series in ε
 sbc r1,r1,#0x7e             @ ... and here
 vmul.f32 s3,s1,s2
 lsls r1,#23                 @ e Q23
 vmul.f32 s4,s2,s2           @ to ε⁴
@ movlong r2,0x58b90bfc      @ log 2 Q31, more accurate than we deserve
 movw r2,0x0bfc
 vmul.f32 s2,s2,s8
 movt r2,0x58b9
 vmul.f32 s3,s3,s9
 smmulr r1,r1,r2             @ Q22
 vmul.f32 s4,s4,s10
 vmov s7,r1
 vsub.f32 s3,s3,s4
 vcvt.f32.s32 s7,s7,#22
 vsub.f32 s2,s2,s3
 vsub.f32 s1,s1,s2
 vadd.f32 s0,s0,s1           @ log ε - log u
 vadd.f32 s0,s0,s7           @ e log 2 + log ε - log u
 vmov r0,s0
 bx r14

1:
 bgt 3f                      @ +NaN?
 beq 10f                     @ +Inf?
2:
 cmp r0,#0x80800000          @ -0?
 blo 11f
 cmp r0,#0xff800000          @ -NaN/-Inf?
3:
 orr r0,#0x00400000
 bhi 10f
 movlong r0,0xffc00000
10:
 bx r14
11:
 movlong r0,0xff800000
 bx r14

.align 3
k_log3:
.float 0.5
.float 0.333333333333333
.float 0.25
.float 0                     @ alignment

logtab3:
@ u=64/[48:2:96]; u Q8, -log u F32
.word 0x0155,0xbe92cb01      @ 00003e9b..00004145
.word 0x0148,0xbe7dc8c3      @ 00003ec8..00004158
.word 0x013b,0xbe545f68      @ 00003ec1..00004137
.word 0x012f,0xbe2c99c7      @ 00003ebb..00004119
.word 0x0125,0xbe0a3c2c      @ 00003ef3..0000413d
.word 0x011a,0xbdc61a2f      @ 00003eca..000040fe
.word 0x0111,0xbd83acc2      @ 00003eeb..0000410d
.word 0x0108,0xbcfc14d8      @ 00003ee8..000040f8
.word 0x0100,0x00000000      @ 00003f00..00004100
.word 0x00f8,0x3d020aec      @ 00003ef8..000040e8
.word 0x00f1,0x3d77518e      @ 00003f13..000040f5
.word 0x00ea,0x3db80698      @ 00003f12..000040e6
.word 0x00e4,0x3ded393b      @ 00003f3c..00004104
.word 0x00dd,0x3e168b08      @ 00003f05..000040bf
.word 0x00d8,0x3e2dfa03      @ 00003f48..000040f8
.word 0x00d2,0x3e4ad2d7      @ 00003f2a..000040ce
.word 0x00cd,0x3e637fde      @ 00003f43..000040dd
.word 0x00c8,0x3e7cc8e3      @ 00003f48..000040d8
.word 0x00c3,0x3e8b5ae6      @ 00003f39..000040bf
.word 0x00bf,0x3e95f784      @ 00003f6b..000040e9
.word 0x00ba,0x3ea38c6e      @ 00003f36..000040aa
.word 0x00b6,0x3eaeadef      @ 00003f46..000040b2
.word 0x00b2,0x3eba0ec4      @ 00003f46..000040aa
.word 0x00ae,0x3ec5b1cd      @ 00003f36..00004092
.word 0x00ab,0x3ece995f      @ 00003f75..000040cb

float_wrapper_section fsin_fcos

30:
 lsls r1,r0,#9
 bne 1f                      @ NaN? return it
 orrs r0,r0,#0x80000000      @ Inf: make a NaN
1:
 orrs r0,r0,#0x00400000      @ set top mantissa bit of NaN
 bx r14

@ heavy-duty range reduction
@ here x≥256, -e in r1
40:
 push {r4-r7,r14}
 movs r3,#1
 bfi r0,r3,#23,#9            @ insert implied 1 in mantissa, clear sign
 rsb r1,#9                   @ e+9
 mov r7,#0x7e                @ this will be the exponent of the reduced angle - 1
42:
 bl frr_core
@ here r6 is revolutions Q32
 lsrs r3,r6,#30              @ quadrant count
 adcs r3,r3,#0               @ rounded
 add r12,r12,r3
 subs r6,r6,r3,lsl#30        @ reduced angle/2π Q32 -.125≤x<+.125
@ comment out from here...
 lsls r2,r6,#2               @ Q34
 it cs
 rsbcs r2,r2,#0              @ absolute value
 cmp r2,#1<<28               @ big enough for accuracy?
 bhs 41f
@ ... to here for slightly better accuracy
43:
 adds r1,r1,#2               @ try again with increased exponent
 bl frr_core
 eors r2,r6,r6,asr#32        @ absolute value
 adc r2,r2,#0
 cmp r2,#1<<28               @ big enough yet?
 bhs 44f
 subs r7,r7,#2
 bpl 43b                     @ safety net
44:

41:
 ldr r4,=0xC90FDAA2          @ 2π Q29
 umull r2,r4,r2,r4           @ r4 has reduced angle Q34+Q29-Q32=Q31
@ add r4,r4,r2,lsr#31
 clz r2,r4                   @ normalise
 lsls r4,r4,r2
 lsrs r4,r4,#8
 sub r2,r7,r2
 adc r0,r4,r2,lsl#23         @ with rounding
 lsrs r1,r0,#23              @ re-extract exponent as there may have been a carry into it
 rsbs r1,r1,#0x7f            @ prepare exponent for re-entry
 lsrs r6,r6,#31
 add r3,r0,r6,lsl#31         @ apply sign of reduced angle
 pop {r4-r7,r14}
 b 5f                        @ re-enter with no risk of looping

.ltorg

@ light-duty range reduction
@ here argument ≥1
@ r0: argument
@ r1: -e
@ r12: quadrant count
@ required result is sin(r0+r12*π/2)
10:
 cmn r1,#0x80
 beq 30b                     @ Inf/NaN
 bics r2,r0,r12,lsl#31       @ negative argument,doing sin -> +2 quadrants
 it mi
 addmi r12,r12,#2
 bic r0,r0,#0x80000000       @ make positive: original sign is now captured in quadrant count in r12

@ this may not actually be faster than doing it in integer registers
 vmov s0,r0
 adr r2,k_sc4
 vldmia r2!,{s5-s7}
@ vmul.f32 s4,s4,s0          @ this accurate calculation of the quadrant count does not seem necessary
@ vfma.f32 s4,s5,s0
 vmul.f32 s4,s5,s0           @ this is BALGE
 cmn r1,#8                   @ ≥256?
 vrintn.f32.f32 s4,s4        @ round to quadrant count: x<256 so count≤163
 ble 40b                     @ then do heavy-duty range reduction
 vfms.f32 s0,s4,s7
 vfms.f32 s0,s4,s6 
 vmov r3,s0                  @ reduced angle
 vcvt.s32.f32 s3,s4
 ubfx r2,r3,#23,#8           @ get exponent
 cmp r2,#0x78
 blo 40b                     @ very small result? use heavy-duty reduction to get a more accurate answer
 rsbs r1,r2,#0x7f            @ ready for re-entry
 vmov r2,s3                  @ integer quadrant count
 add r12,r12,r2
@ prepare to re-enter with no risk of looping
 b 5f

k_sc4:
@ 2/π=0.A2F9836E4E441529FC...
.word 0x3f22f983             @ 2/π
@ π/2=1.921FB54442D1846989...
.word 0xb695777a,0x3fc91000  @ these two add up to π/2 with error ~1.6e-13

wrapper_func sincosf

 push {r0-r2,r14}
 ubfx r1,r0,#23,#8
 cmp r1,#0xff                @ Inf/NaN?
 beq 2f
 bl cosf_entry               @ this will exit via 1f or 2f...
 pop {r1-r2,r14}
 str r0,[r14]
@ here C is still set from lsrs r12,r12,#1
 bcs 1f
 mvns r1,r1
 eor r12,r12,r1,lsr#31
@ this is fsc_costail:
@ here calculate cos φ+ε = cosθ
 vmul.f32 s5,s7,s1           @ sinφ sinε
 vfma.f32 s5,s2,s6           @ sinφ sinε + cosφ(1-cosε)
 vsub.f32 s5,s6,s5           @ cosφ - (sinφ sinε + cosφ(1-cosε)) = cosφ cosε - sinφ sinε
 vmov.f32 r0,s5
 eor r0,r0,r12,lsl#31
 str r0,[r2]
 pop {r15}

1:
 eor r12,r12,r1,lsr#31
@ this is fsc_sintail:
@ here calculate sin φ+ε = sinθ
 vmul.f32 s4,s2,s7           @ sinφ(1-cosε)
 vfms.f32 s4,s6,s1           @ sinφ(1-cosε) - cosφ sinε
 eor r1,r12,r3,lsr#31        @ flip sign if (reduced) argument was negative
 vsub.f32 s4,s7,s4           @ cosφ sinε + sinφ cosε
 vmov.f32 r0,s4
 eor r0,r0,r1,lsl#31
 str r0,[r2]                 @ save cos result
 pop {r15}

@ sincos of Inf or NaN
2:
 lsls r1,r0,#9
 pop {r1-r3,r14}
 bne 1f                      @ NaN? return it
 orrs r0,r0,#0x80000000      @ Inf: make a NaN
1:
 orrs r0,r0,#0x00400000      @ set top mantissa bit of NaN
 str r0,[r2]                 @ both sin and cos results
 str r0,[r3]
 bx r14

wrapper_func sinf
@ r12b1..0: quadrant count
 movs r12,#0
 b 1f

wrapper_func cosf
.thumb_func
cosf_entry:
 movs r12,#1                 @ cos -> +1 quadrant
1:
 ubfx r1,r0,#23,#8           @ get exponent
 cbz r1,20f                  @ 0/denormal?
22:
 rsbs r1,r1,#0x7f
 bls 10b                     @ argument ≥1? needs reduction; also Inf/NaN handling
 bic r3,r0,r12,lsl#31        @ this would mess up NaNs so do it here
5:
@ here we have a quadrant count in r12 and a signed offset r0 from r12*π/2
 bic r0,r3,#0x80000000       @ this would mess up NaNs so do it here
 vmov s0,r0
 ubfx r0,r0,#18,#5           @ extract top of mantissa
 adds r0,r0,#32              @ insert implied 1
 lsrs r1,r0,r1               @ to fixed point Q5
 ldr r2,=k_sc3
 adcs r1,r1,#0               @ rounding
 vldmia r2!,{s8-s9}
 add r2,r2,r1,lsl#2          @ 12 bytes per entry
 add r2,r2,r1,lsl#3

 vldmia r2,{s5-s7}           @ φ, cosφ, sinφ
 vsub.f32 s1,s0,s5           @ ε
 vmul.f32 s2,s1,s1           @ ε²
 lsrs r12,r12,#1             @ computing cosine?
 vmul.f32 s3,s2,s1           @ ε³
 bcs 2f

 vmul.f32 s2,s2,s8           @ ε²/2! ~ 1-cosε
 vmul.f32 s3,s3,s9           @ ε³/3!
 vsub.f32 s1,s1,s3           @ ε-ε³/3! ~ sinε

@ here:
@ s1: sinε
@ s2: 1-cosε
@ s6: cosφ
@ s7: sinφ
@ r12: quadrant count
fsc_sintail:
@ here calculate sin φ+ε = sinθ
 vmul.f32 s4,s2,s7           @ sinφ(1-cosε)
 vfms.f32 s4,s6,s1           @ sinφ(1-cosε) - cosφ sinε
 eor r1,r12,r3,lsr#31        @ flip sign if (reduced) argument was negative
 vsub.f32 s4,s7,s4           @ cosφ sinε + sinφ cosε
 vmov.f32 r0,s4
 eor r0,r0,r1,lsl#31
 bx r14

20:
 and r0,r0,#0x80000000       @ make signed zero
 b 22b

.align 2
2:
 vmul.f32 s3,s3,s9           @ ε³/3!
 vsub.f32 s1,s1,s3           @ ε-ε³/3! ~ sinε
 vmul.f32 s2,s2,s8           @ ε²/2! ~ 1-cosε
fsc_costail:
@ here calculate cos φ+ε = cosθ
 vmul.f32 s5,s7,s1           @ sinφ sinε
 vfma.f32 s5,s2,s6           @ sinφ sinε + cosφ(1-cosε)
 vsub.f32 s5,s6,s5           @ cosφ - (sinφ sinε + cosφ(1-cosε)) = cosφ cosε - sinφ sinε
 vmov.f32 r0,s5
 eor r0,r0,r12,lsl#31
 bx r14

.align 3
k_sc3:
.word 0x3EFFFEC1             @ ~ 1/2! with PMC
.word 0x3e2aaa25             @ ~ 1/3! with PMC

trigtab2:
//      φ          cos φ      sin φ
.word 0x00000000,0x3f800000,0x00000000
.word 0x3cfcc961,0x3f7fe0cd,0x3cfcbf1c           @ φ=0.03085774 : cos φ=3feffc199ff28ef4 33.3b; sin φ=3f9f97e38006c678 39.2b
.word 0x3d810576,0x3f7f7dfe,0x3d80ef9e           @ φ=0.06299870 : cos φ=3fefefbfc00d6b6d 33.3b; sin φ=3fb01df3c000dfd5 40.2b
.word 0x3dbf0c09,0x3f7ee30f,0x3dbec522           @ φ=0.09328467 : cos φ=3fefdc61dff4f58e 33.5b; sin φ=3fb7d8a43ffdf9ac 39.0b
.word 0x3dff24b6,0x3f7e0414,0x3dfe7be2           @ φ=0.12458174 : cos φ=3fefc0827fdaf90f 31.8b; sin φ=3fbfcf7c3ff9dd0c 37.4b
.word 0x3e1f0713,0x3f7ceb48,0x3e1e63a0           @ φ=0.15530042 : cos φ=3fef9d68ffe680a0 32.3b; sin φ=3fc3cc73fffa6d09 36.5b
.word 0x3e40306d,0x3f7b811d,0x3e3f1015           @ φ=0.18768473 : cos φ=3fef70239fe32301 32.1b; sin φ=3fc7e2029ffdbc2c 37.8b
.word 0x3e60ada2,0x3f79dccf,0x3e5ee13e           @ φ=0.21941236 : cos φ=3fef3b99e023f5aa 31.8b; sin φ=3fcbdc27bffe216d 38.1b
.word 0x3e800d7b,0x3f7808fa,0x3e7d7196           @ φ=0.25010285 : cos φ=3fef011f401572a6 32.6b; sin φ=3fcfae32c00328bb 37.3b
.word 0x3e8f986e,0x3f75ff65,0x3e8db868           @ φ=0.28045982 : cos φ=3feebfeca0aaaf99 29.6b; sin φ=3fd1b70cfffc1468 36.0b
.word 0x3e9fe1f4,0x3f739e93,0x3e9d4bfd           @ φ=0.31227076 : cos φ=3fee73d25fbf733b 31.0b; sin φ=3fd3a97fa0002ced 40.5b
.word 0x3eb054c6,0x3f70f7ae,0x3eacddb3           @ φ=0.34439677 : cos φ=3fee1ef5bfcf70cb 31.4b; sin φ=3fd59bb65fff5c30 38.6b
.word 0x3ebf89c5,0x3f6e4b60,0x3ebb1a0a           @ φ=0.37409797 : cos φ=3fedc96bffdebb8a 31.9b; sin φ=3fd763414003344b 36.3b
.word 0x3ecfc426,0x3f6b35ca,0x3eca1c63           @ φ=0.40579337 : cos φ=3fed66b93fe27dc6 32.1b; sin φ=3fd9438c5ffe5d45 37.3b
.word 0x3ee054f2,0x3f67d166,0x3ed93907           @ φ=0.43814808 : cos φ=3fecfa2cbffc16e9 35.0b; sin φ=3fdb2720dffef5b6 37.9b
.word 0x3eeff0dd,0x3f64664b,0x3ee74116           @ φ=0.46863452 : cos φ=3fec8cc95f714272 29.8b; sin φ=3fdce822c00479ad 35.8b
.word 0x3f002b31,0x3f609488,0x3ef5c30f           @ φ=0.50065905 : cos φ=3fec1290ffc99208 31.2b; sin φ=3fdeb861dfff3932 38.4b
.word 0x3f07e407,0x3f5cc5a2,0x3f01992b           @ φ=0.53082317 : cos φ=3feb98b44034cd46 31.3b; sin φ=3fe033255ffff628 41.7b
.word 0x3f101fc5,0x3f587d8f,0x3f08a165           @ φ=0.56298476 : cos φ=3feb0fb1e0ceda6f 29.3b; sin φ=3fe1142c9ffd5ae4 35.6b
.word 0x3f17f68a,0x3f5434b5,0x3f0f31ca           @ φ=0.59360564 : cos φ=3fea8696a038a06f 31.2b; sin φ=3fe1e639400269fb 35.7b
.word 0x3f1fffe2,0x3f4f9b59,0x3f15c8d7           @ φ=0.62499821 : cos φ=3fe9f36b1f428363 29.4b; sin φ=3fe2b91ae001d55d 36.1b
.word 0x3f280646,0x3f4acf6b,0x3f1c37c4           @ φ=0.65634573 : cos φ=3fe959ed61449f08 28.7b; sin φ=3fe386f87ffd9617 35.7b
.word 0x3f303041,0x3f45b9e0,0x3f229ae4           @ φ=0.68823630 : cos φ=3fe8b73c0047ae7a 30.8b; sin φ=3fe4535c7ffdf1ac 36.0b
.word 0x3f381da7,0x3f4098ca,0x3f28a620           @ φ=0.71920246 : cos φ=3fe81319402ae6e1 31.6b; sin φ=3fe514c3ffff423c 37.4b
.word 0x3f3fc72f,0x3f3b76ac,0x3f2e564a           @ φ=0.74913305 : cos φ=3fe76ed5809d419f 29.7b; sin φ=3fe5cac93fffaf1d 38.7b
.word 0x3f4813db,0x3f35b6cc,0x3f34526b           @ φ=0.78155297 : cos φ=3fe6b6d9800e8b52 33.1b; sin φ=3fe68a4d5ffe89fc 36.5b
.word 0x3f4fc779,0x3f30352f,0x3f39b4d0           @ φ=0.81163746 : cos φ=3fe606a5dfdc2b5b 31.8b; sin φ=3fe73699fffd7fc8 35.7b
.word 0x3f57dd52,0x3f2a4170,0x3f3f2d91           @ φ=0.84322083 : cos φ=3fe5482e011ba752 28.9b; sin φ=3fe7e5b21ffcb223 35.3b
.word 0x3f5fce26,0x3f243e9f,0x3f445dc3           @ φ=0.87423933 : cos φ=3fe487d3e0b9864b 29.5b; sin φ=3fe88bb85ffde6d5 35.9b
.word 0x3f6825f1,0x3f1dc250,0x3f499d1c           @ φ=0.90682894 : cos φ=3fe3b849ffea9b8f 32.6b; sin φ=3fe933a38002730d 35.7b
.word 0x3f703be1,0x3f175041,0x3f4e7ebf           @ φ=0.93841368 : cos φ=3fe2ea0820791b4e 30.1b; sin φ=3fe9cfd7e0053e65 34.6b
.word 0x3f781078,0x3f10ed71,0x3f5306af           @ φ=0.96900129 : cos φ=3fe21dae1fdea23e 31.9b; sin φ=3fea60d5e001b90b 36.2b
.word 0x3f7ff4d4,0x3f0a5aa7,0x3f57649b           @ φ=0.99982953 : cos φ=3fe14b54deeaa407 28.9b; sin φ=3feaec9360012825 36.8b

float_wrapper_section tanf

wrapper_func tanf
 push {r0,r14}
 ubfx r1,r0,#23,#8
 cmp r1,#0xff                @ Inf/NaN?
 beq 2f
 bl cosf_entry               @ this will exit via sintail or costail...
 ldr r1,[sp,#0]
@ here C is still set from lsrs r12,r12,#1
 bcs 1f
@ we exited via sintail
@ this is fsc_costail:
@ here calculate cos φ+ε = cosθ
 vmul.f32 s5,s7,s1           @ sinφ sinε
 vfma.f32 s5,s2,s6           @ sinφ sinε + cosφ(1-cosε)
 eors r1,r1,r3
 vsub.f32 s5,s6,s5           @ cosφ - (sinφ sinε + cosφ(1-cosε)) = cosφ cosε - sinφ sinε
 vdiv.f32 s0,s5,s4
 vmov.f32 r0,s0
 it pl
 eorpl r0,r0,#0x80000000
 pop {r1,r15}

1:
@ we exited via costail
@ this is fsc_sintail:
@ here calculate sin φ+ε = sinθ
 vmul.f32 s4,s2,s7           @ sinφ(1-cosε)
 vfms.f32 s4,s6,s1           @ sinφ(1-cosε) - cosφ sinε
 eors r1,r1,r3
 vsub.f32 s4,s7,s4           @ cosφ sinε + sinφ cosε
 vdiv.f32 s0,s4,s5
 vmov.f32 r0,s0
 it mi
 eormi r0,r0,#0x80000000
 pop {r1,r15}

@ tan of Inf or NaN
2:
 lsls r1,r0,#9
 bne 1f                      @ NaN? return it
 orrs r0,r0,#0x80000000      @ Inf: make a NaN
1:
 orrs r0,r0,#0x00400000      @ set top mantissa bit of NaN
 pop {r3,r15}


float_wrapper_section atan2f

50:
60:
 orrs r0,r1,#0x00400000
 bx r14

51:
 bne 52f                     @ NaN?
 cmp r3,#0x7f800000          @ y an infinity; x an infinity too?
 bne 55f                     @ no: carry on
@ here x and y are both infinities
 b 66f

52:
62:
 orrs r0,r0,#0x00400000
 bx r14

61:
 bne 62b                     @ NaN?
 cmp r3,#0x7f800000          @ y an infinity; x an infinity too?
 bne 65f                     @ no: carry on
66:
@ here x and y are both infinities
 subs r0,r0,#1               @ make both finite (and equal) with same sign and retry
 subs r1,r1,#1
 b 86f

70:
 and r3,#0x80000000
 cmp r2,#0x00800000
 bhs 72f                     @ y 0 or denormal?
@ here both x and y are zeros
 b 85f
71:
 and r2,#0x80000000
72:
 vmov s0,s1,r2,r3
 vdiv.f32 s2,s0,s1           @ restart the division
 b 73f                       @ and go back and check for NaNs

80:
 and r3,#0x80000000
 cmp r2,#0x00800000
 bhs 82f                     @ y 0 or denormal?
85:
@ here both x and y are zeros
 orr r1,r1,0x3f800000        @ retry with x replaced by ~1 with appropriate sign
 b 86f
 
81:
 and r2,#0x80000000
82:
 vmov s0,s1,r2,r3
 vdiv.f32 s2,s1,s0           @ restart the division
 b 83f                       @ and go back and check for NaNs

wrapper_func atan2f
86:
 bic r2,r0,#0x80000000
 bic r3,r1,#0x80000000
 vmov s0,s1,r2,r3
 cmp r2,r3                   @ |y| vs. |x|
 bhi 1f
@ here |x|≥|y| so we need |y|/|x|; octant/xs/ys: 0++,3-+,4--,7+-
 vdiv.f32 s2,s0,s1           @ get this division started; result ≤1
 cmp r3,#0x00800000
 blo 70b                     @ x 0 or denormal?
 cmp r2,#0x00800000
 blo 71b                     @ y 0 or denormal?
73:
 cmp r3,#0x7f800000
 bhi 50b                     @ x NaN?
 cmp r2,#0x7f800000
 bhs 51b                     @ y Inf or NaN?
55:
 cmp r1,#0
 ite mi
 ldrmi r12,pi                @ if x<0, need two extra quadrants
 movpl r12,#0
                             @ inner negation is the sign of x
 b 2f

1:
@ here |x|<|y| so we need |x|/|y|; octant/xs/ys: 1++,2-+,5--,6+-
 vdiv.f32 s2,s1,s0           @ result <1
 cmp r3,#0x00800000
 blo 80b                     @ x 0 or denormal?
 cmp r2,#0x00800000
 blo 81b                     @ y 0 or denormal?
83:
 cmp r3,#0x7f800000
 bhi 60b                     @ x NaN?
 cmp r2,#0x7f800000
 bhs 61b                     @ y Inf or NaN?
65:
 ldr r12,piover2             @ always one extra quadrant in this path
 eors r1,r1,#0x80000000      @ inner negation is the complement of the sign of x

2:
@ here
@ r0 y
@ r1 ±x
@ r2 |y|
@ r3 |x|
@ s0,s1 = |x|,|y|
@ s2=s0/s1 or s1/s0, 0≤s2≤1
@ r12=quadrant count * π/2
@ where the final result is
@ ± (r12 ± atn s2) where the inner negation is given by r1b31 and the outer negation by r0b31

 adr r2,trigtab3
 vmov.f32 s3,s2
 vcvt.u32.f32 s3,s3,#6
 vmov.f32 r3,s3
 lsrs r3,r3,#1
 adcs r3,r3,#0               @ rounding; set Z if in φ==0 case
 add r2,r2,r3,lsl#3
 vldr s5,[r2,#4]             @ t=tanφ
 vmul.f32 s0,s5,s2           @ ty
 vsub.f32 s1,s2,s5           @ y-t
 vmov.f32 s5,#1.0
 vadd.f32 s0,s5,s0           @ 1+ty
 beq 9f                      @ did we look up zeroth table entry?

@ now (s0,s1) = (x,y)
 vdiv.f32 s0,s1,s0           @ ε
 ldr r2,[r2]                 @ φ Q29
@ result is now ±(r12±(r2+atn(s0))
 cmp r1,#0                   @ inner negation
 it mi
 rsbmi r2,r2,#0
 add r2,r12,r2               @ Q29
 cmp r0,#0                   @ outer negation
 it mi
 rsbmi r2,r2,#0
 cmp r2,#0
 bpl 1f
 rsbs r2,r2,#0
 clz r3,r2
 lsls r2,r2,r3
 beq 3f
 rsb r3,#0x180
 b 2f
1:
 clz r3,r2
 lsls r2,r2,r3
 beq 3f
 rsb r3,#0x80
2:
 lsrs r2,r2,#8               @ rounding bit to carry
 adc r2,r2,r3,lsl#23         @ with rounding
3:
 vmul.f32 s2,s0,s0           @ ε²
 vldr.f32 s3,onethird
 vmul.f32 s2,s2,s0           @ ε³
 teq r0,r1
 vmul.f32 s2,s2,s3           @ ε³/3
 vmov.f32 s4,r2
 vsub.f32 s0,s0,s2           @ ~atn(ε)
 ite pl
 vaddpl.f32 s0,s4,s0
 vsubmi.f32 s0,s4,s0
 vmov.f32 r0,s0
 bx r14

9:                           @ we looked up the zeroth table entry; we could generate slightly more accurate results here
@ now (s0,s1) = (x,y)
 vdiv.f32 s0,s1,s0           @ ε
@ result is now ±(r12±(0+atn(s0))
 mov r2,r12                  @ Q29; in fact r12 is only ±π/2 or ±π so can probably simplify this
 cmp r0,#0                   @ outer negation
 it mi
 rsbmi r2,r2,#0
 cmp r2,#0
 bpl 1f
 rsbs r2,r2,#0
 clz r3,r2
 lsls r2,r2,r3
 beq 3f
 rsb r3,#0x180
 b 2f
1:
 clz r3,r2
 lsls r2,r2,r3
 beq 3f
 rsb r3,#0x80
2:
 lsrs r2,r2,#8               @ rounding bit to carry
 adc r2,r2,r3,lsl#23         @ with rounding
3:
 vmul.f32 s2,s0,s0           @ ε²
 vldr.f32 s3,onethird
 vmul.f32 s2,s2,s0           @ ε³
 teq r0,r1
 vmul.f32 s2,s2,s3           @ ε³/3
 vmov.f32 s4,r2
 vsub.f32 s0,s0,s2           @ ~atn(ε)
 ite pl
 vaddpl.f32 s0,s4,s0
 vsubmi.f32 s0,s4,s0
 vmov.f32 r0,s0
 tst r0,#0x7f800000          @ about to return a denormal?
 it ne
 bxne r14
 and r0,r0,#0x80000000       @ make it zero
 bx r14

piover2:  .word 0x3243f6a9   @ Q29
pi:       .word 0x6487ed51   @ Q29
onethird: .float 0.33333333

trigtab3:
//      φ Q29      tan φ SP
.word 0x00000000,0x00000000
.word 0x00ffee23,0x3d0001bb  @ φ=0.03124148 : tan φ=3fa000375fffff9d 50.4b
.word 0x01fe88dc,0x3d7f992a  @ φ=0.06232112 : tan φ=3faff3253fffea1f 44.5b
.word 0x02fe0a70,0x3dc01203  @ φ=0.09351084 : tan φ=3fb8024060002522 42.8b
.word 0x03fad228,0x3e000368  @ φ=0.12436779 : tan φ=3fc0006cfffffc90 45.2b
.word 0x04f5ab70,0x3e1ffdea  @ φ=0.15498897 : tan φ=3fc3ffbd400014d5 42.6b
.word 0x05ed56f8,0x3e3fdddc  @ φ=0.18522213 : tan φ=3fc7fbbb80000beb 43.4b
.word 0x06e4cfa0,0x3e601425  @ φ=0.21543103 : tan φ=3fcc02849fffe817 42.4b
.word 0x07d8d3e0,0x3e80215d  @ φ=0.24521822 : tan φ=3fd0042b9ffff89f 43.1b
.word 0x08c60460,0x3e9000a5  @ φ=0.27417201 : tan φ=3fd20014a000182b 41.4b
.word 0x09b26770,0x3ea01492  @ φ=0.30302784 : tan φ=3fd402923ffff932 43.2b
.word 0x0a996d50,0x3eb01377  @ φ=0.33122888 : tan φ=3fd6026ee0001062 42.0b
.word 0x0b7a6d10,0x3ebff4a0  @ φ=0.35869458 : tan φ=3fd7fe93ffff8c38 39.1b
.word 0x0c593ce0,0x3ed0019f  @ φ=0.38589329 : tan φ=3fda0033e0001354 41.7b
.word 0x0d33ebd0,0x3ee01bbc  @ φ=0.41258803 : tan φ=3fdc0377800162a1 37.5b
.word 0x0e087ab0,0x3ef01fbd  @ φ=0.43853506 : tan φ=3fde03f79fffddf2 40.9b
.word 0x0ed56180,0x3effef98  @ φ=0.46354747 : tan φ=3fdffdf30000767d 39.1b
.word 0x0fa1de80,0x3f080ebf  @ φ=0.48850942 : tan φ=3fe101d7dfffb9fc 38.9b
.word 0x10639d00,0x3f0fec31  @ φ=0.51215982 : tan φ=3fe1fd862000aad5 37.6b
.word 0x112690e0,0x3f180cfd  @ φ=0.53595775 : tan φ=3fe3019fa00069ea 38.3b
.word 0x11e014c0,0x3f200065  @ φ=0.55860364 : tan φ=3fe4000ca00022e5 39.9b
.word 0x129651e0,0x3f2808be  @ φ=0.58084959 : tan φ=3fe50117c00015a7 40.6b
.word 0x1346d400,0x3f300a7d  @ φ=0.60239601 : tan φ=3fe6014f9fffa020 38.4b
.word 0x13efc7c0,0x3f37ee2f  @ φ=0.62302005 : tan φ=3fe6fdc5dfff98d7 38.3b
.word 0x14988960,0x3f400c32  @ φ=0.64362019 : tan φ=3fe801863fffff81 46.0b
.word 0x1537a8c0,0x3f47ef42  @ φ=0.66304433 : tan φ=3fe8fde8400062a4 38.4b
.word 0x15d4cc60,0x3f4ff630  @ φ=0.68222636 : tan φ=3fe9fec5ffff76e2 37.9b
.word 0x166ef280,0x3f581534  @ φ=0.70104337 : tan φ=3feb02a680004e91 38.7b
.word 0x16ff75c0,0x3f5fef1e  @ φ=0.71868408 : tan φ=3febfde3c0001404 40.7b
.word 0x179116a0,0x3f68184d  @ φ=0.73646098 : tan φ=3fed03099ffed6e5 36.8b
.word 0x181b5aa0,0x3f701722  @ φ=0.75333911 : tan φ=3fee02e43fffd351 39.5b
.word 0x18a10560,0x3f781071  @ φ=0.76965588 : tan φ=3fef020e20005c05 38.5b
.word 0x19214060,0x3f7ff451  @ φ=0.78530902 : tan φ=3feffe8a1fffe11b 40.1b

#endif
