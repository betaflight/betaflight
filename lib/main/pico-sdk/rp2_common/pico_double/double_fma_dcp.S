/*
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/asm_helper.S"

#if !HAS_DOUBLE_COPROCESSOR
#error attempt to compile double_fma_rp2350 when there is no DCP
#else

#include "hardware/dcp_instr.inc.S"
#include "hardware/dcp_canned.inc.S"

pico_default_asm_setup

// factor out save/restore (there is a copy in float code)

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

.macro saving_func_return
  bx lr
.endm

double_section __rp2350_dcp_engaged_state_save_restore_copy
.thumb_func
__dcp_save_state:
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
__dcp_restore_state:
  // do restore here
  pop {r12, r14}
  WXMD r12, r14
  pop {r12, r14}
  WYMD r12, r14
  pop {r12, r14}
  WEFD r12, r14
  pop {pc}

double_wrapper_section __dfma
@ cf saving_func macro: but here we need to record the SP before the state save possibly changes it
1:
 push {lr}              // 16-bit instruction
 bl __dcp_save_state    // 32-bit instruction
 b 1f                   // 16-bit instruction

@ compute mn+a with full intermediate precision
@ r0:r1 m
@ r2:r3 n
@ [r13,#0] a
wrapper_func fma
 mov r12,sp                  @ save the SP
 PCMP apsr_nzcv              @ test the engaged flag
 bmi 1b
1:
 push {r4-r8,r14}
 ldrd r4,r5,[r12,#0]         @ fetch a using original SP
 ubfx r7,r1,#20,#11          @ r7=em
 ubfx r8,r3,#20,#11          @ r8=en
 add r8,r7,r8                @ em+en

 eors r6,r1,r3               @ get sign of mn
 eors r6,r6,r5               @ set N if mn has opposite sign to a, i.e. if the operation is essentially a subtraction
 WXUP r4,r5                  @ write a to coprocessor to get its classification
 PEFD r14,r12                @ r14=fa
 WXUP r0,r1                  @ write m and n to coprocessor to get their classifications
 WYUP r2,r3
 PEFD r6,r12                 @ r6=fm, r12=fn, r14=fa
 orr r14,r14,r6
 orr r14,r14,r12             @ OR of all the classification flags, so we can check if any are zero/Inf/NaN

 RXMS r3,r6,0                @ we will almost always need the full product so compute it here (cf dmul macro)
 RYMS r7,r12,0
 umull r0,r1,r3,r7
 mov r2,#0                   @ seems to be no 16-bit instruction which zeros a register without affecting the flags
 umlal r1,r2,r3,r12
 umlal r1,r2,r6,r7
 mov r3,#0
 umlal r2,r3,r6,r12          @ r0:r1:r2:r3: full product mn Q124 1≤mn<4

 bmi 50f                     @ mn has opposite sign to a so operation is essentially a subtraction

@ ======================== ADDITION PATH ========================

 tst r14,#0x70000000         @ were any of the arguments zero/inf/NaN?
 bne 90f                     @ then use mla path which gives the correct result in all these cases
 ubfx r14,r5,#20,#11         @ r14=ea
@ here all operands are finite and non-zero
@ r0:r1:r2:r3: full product mn Q124 1≤mn<4
@ r4:r5 a IEEE packed
@ r8: em+en [biased +0x3ff*2]
@ r14: ea [biased +0x3ff]
 subw r7,r8,#0x3fd
 subs r7,r7,r14              @ em+en-ea+2 (debiased)
 blt 80f                     @ branch if |a| is big compared to |mn|, more precisely if ea-(em+en)≥3 so e.g. if ea=0 (hence 1≤a<2) then em+en≤-3 and mn<4.2¯³=1/2

@ ======================== ADDITION PATH, RESULT HAS COMPARABLE MAGNITUDE TO mn  ========================

@ here |mn| is big compared to |a|; e.g. if em+en=0 (so 1≤mn<4) then ea≤2 and a<8
 movs r8,#1
 bfi r5,r8,#20,#12           @ insert implied 1 in a
 rsbs r7,r7,#74              @ shift up ≤74 (can be negative) that will be required for a (Q52) to align with mn (Q124, ending in 20 zeros)
@ now add (shifted) a into mn, preserving flags
 and r8,r7,#0x1f             @ k=shift mod 32
 mov r12,#1
 lsl r12,r12,r8              @ 2^k
 umull r5,r6,r5,r12          @ shift up high word: r4:r5:r6 is now a_lo + 2^k a_hi
 sub r12,#1                  @ 2^k-1
 umlal r4,r5,r4,r12          @ shift up low word, adding in: r4:r5:r6 is now (a_lo + 2^k a_hi) + (2^k-1) a_lo = 2^k (a_lo + a_hi) = a shifted up by k
 bmi 91f                     @ use flags: will a be shifted down?
 cmp r7,#64                  @ shift up by two more words?
 bge 92f
 cmp r7,#32                  @ shift up by one more word?
 bge 93f
 adds r0,r0,r4               @ no more word shifts
 adcs r1,r1,r5
 adcs r2,r2,r6
 adcs r3,r3,#0               @ r0:r1:r2:r3: mn + a (cf dmul macro)
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD                        @ as dmul macro tail: exponent computed in coprocessor is correct
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
  pop {r4-r8,lr}
  saving_func_return

93:
 adds r1,r1,r4
 adcs r2,r2,r5
 adcs r3,r3,r6               @ r0:r1:r2:r3: mn + (a<<32)
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

92:
 adds r2,r2,r4
 adcs r3,r3,r5               @ r0:r1:r2:r3: mn + (a<<64); note this cannot overflow as total shift was at most 74 (see above)
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

91:                          @ case where a (Q52) is shifted down relative to mn (Q124); the mod 32 part of the shift of a has already been done
@ r0:r1:r2:r3: mn
@ r4:r5:r6: a
@ r7: alignment shift required (negative)
 cmn r7,#32                  @ shift down one word?
 bge 94f
 cmn r7,#64                  @ shift down two words?
 bge 95f
@ here a is shifted entirely below the bottom of m
 orr r0,r0,#1                @ a is non-zero so ensure we set the sticky bit
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

94:
 adds r0,r0,r5               @ one word shift down
 adcs r1,r1,r6
 adcs r2,r2,#0
 adcs r3,r3,#0
 orr r0,r0,r4                @ contribution from a to sticky bits
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

95:
 adds r0,r0,r6               @ two word shift down
 adcs r1,r1,#0
 adcs r2,r2,#0
 adcs r3,r3,#0
 orr r0,r0,r4                @ contribution from a to sticky bits
 orr r0,r0,r5
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

@ ======================== ADDITION PATH, RESULT HAS COMPARABLE MAGNITUDE TO a  ========================

80:
@ here |mn|<~|a|
@ r0:r1:r2:r3: mn Q124
@ r4:r5 a IEEE packed
@ r7: -(shift down required to align mn with a), guaranteed negative
@ r8: em+en [biased +0x3ff*2]
@ r14: ea [biased +0x3ff]
 tst r3,#0x20000000
 bne 1f @ 2≤mn<4?
 adds r2,r2,r2               @ normalise so mn is 2..4 Q124; note that the contents of r0 and r1 are always destined for the sticky bit in this path
 adcs r3,r3,r3
 subs r7,r7,#1               @ correction to alignment shift
1:
@ now we construct an IEEE packed value in r2:r3 such that adding it to r4:r5 gives the correct final result
@ observe that the exponent of this constructed value will be at least two less than that of a (by the "|a| is big compared to |mn|" test above)
@ so the alignment shift in the final addition will be by at least two places; thus we can use bit 0 of the constructed
@ value as a sticky bit, and we still have one bit in hand for rounding
 subs r7,r7,#2               @ now r7 < -2
 orr r0,r0,r2,lsl#23         @ shift r2:r3 down 9 places, ORing excess into sticky bits
 lsrs r2,r2,#9
 orr r2,r2,r3,lsl#23
 lsrs r3,r3,#9
 orrs r0,r0,r1
 it ne
 orrne r2,r2,#1              @ sticky bit from bottom 64 bits of mn as shifted
@ r2:r3 mn 2..4 Q51, i.e. 1..2 Q52
@ r2b0 holds sticky bit; note that for alignment with a in r4:r5, r2:r3 will be shifted down at least one place

 lsrs r6,r5,#31              @ get sign of a (which in this path is the same as the sign of mn, and of the result)
 orr r3,r3,r6,lsl#31         @ set sign in mn

 adds r14,r7,r14             @ get exponent for mn relative to a; note this can go negative
 add r3,r3,r14,lsl#20        @ note that "implied" 1 is present in r3, giving an offset of 1 in the exponent
 bmi 1f                      @ negative? then we have just constructed a denormal (or less) and the addition will give an incorrect result
 dcp_dadd_m r0,r1,r2,r3,r4,r5
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

1:
@ compare with similar code in subtraction path: here we cannot underflow
 cmn r7,#64                  @ if the alignment shift for mn is very large then the result is just a
 ble 82f
 add r3,r3,#0x40000000       @ ea cannot be very large (as adding r7 made it negative), so safe to add 1024 to exponents of both a and mn
 add r5,r5,#0x40000000
 dcp_dadd_m r0,r1,r2,r3,r4,r5
 sub r1,r1,#0x40000000       @ fix exponent
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

90:
@ dcp_dmul_m tail then dadd ("mla path")
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
 dcp_dadd_m r0,r1,r0,r1,r4,r5
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

82:                          @ |mn| is very small compared to |a|, so result is a
 RDDM r0,r1                  @ clear the engaged flag
 movs r0,r4
 movs r1,r5
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

@ ======================== SUBTRACTION PATH ========================

50:
 tst r14,#0x70000000         @ were any of the arguments zero/inf/NaN?
 bne 90b @ then use mla path which gives the correct result in all these cases
 ubfx r14,r5,#20,#11         @ r14=ea
@ now all operands are finite and non-zero
@ r0:r1:r2:r3: full product mn Q124 1≤mn<4
@ r4:r5 a IEEE packed (including sign bit; sign of mn is opposite as we are in the subtraction path)
@ r8: em+en [+0x3ff*2]
@ r14: ea [+0x3ff]
 subw r8,r8,#0x3fc           @ em+en+3
 subs r7,r8,r14              @ em+en-ea+3 (debiased)
 blt 80f                     @ branch if |a| is big compared to |mn|, more precisely if ea-(em+en)≥4 so e.g. if ea=0 then em+en≤-4 and mn<4.2^-4=1/4
 beq 94f                     @ branch if ea-(em+en)=3 e.g. if ea=0 then em+en=-3 and 1/8=2^-3≤mn<4.2^-3=1/2
@ in this branch, if e.g. em+en=0 (so 1≤mn<4) then ea≤2 and a<8
 rsbs r7,r7,#75              @ 75-(em+en-ea+3) = 72-(em+en-ea), shift up 0..74 that will be required for a (Q52) to align with mn (Q124, ending in 20 zeros)
 mvn r14,r5,lsr#31           @ save complement of sign of a
@ subtract (shifted) a from mn
 and r6,r7,#0x1f             @ k=shift mod 32
 mov r12,#1
 bfi r5,r12,#20,#12          @ insert implied 1 in a
 lsl r12,r12,r6              @ 2^k
 umull r5,r6,r5,r12
 sub r12,#1
 umlal r4,r5,r4,r12          @ shift a up by shift amount mod 32 (see comment in addition path)
@ r4:r5:r6: a shifted up by k=shift mod 32
 bmi 91f                     @ will a be shifted down?
 cmp r7,#64                  @ shift up by two more words?
 bge 92f
 cmp r7,#32                  @ shift up by one more word?
 bge 93f
 subs r0,r0,r4               @ no more word shifts; this cannot go negative or have bad cancellation
 sbcs r1,r1,r5
 sbcs r2,r2,r6
 sbcs r3,r3,#0               @ r0:r1:r2:r3: mn - a (cf dmul macro)
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD                        @ as dmul macro tail: exponent and sign computed in coprocessor is correct
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

94:
@ here if ea-(em+en)=3 e.g. if ea=0 then em+en=-3 and 1/8=2^-3≤mn<4.2^-3=1/2
@ r0:r1:r2:r3: full product mn Q124 1≤mn<4
@ r4:r5 a IEEE packed (including sign bit; sign of mn is opposite as we are in the subtraction path)
 lsls r5,r5,#11              @ convert a to mantissa Q63 in r4:r5
 orrs r5,r5,r4,lsr#21
 lsls r4,r4,#11
 orrs r5,r5,0x80000000       @ implied 1
 movs r6,#0
 subs r0,r6,r0               @ compute |a|-|mn|
 sbcs r6,r6,r1
 sbcs r4,r4,r2
 sbcs r5,r5,r3
 WXMS r0,r6                  @ write sticky bits
 WXMO r4,r5                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
 eor r1,r1,0x80000000        @ sign of result is opposite to that of product as yielded by coprocessor
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

93:
 subs r1,r1,r4               @ shifting a up by one word: this cannot go negative or have bad cancellation
 sbcs r2,r2,r5
 sbcs r3,r3,r6
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

92:
 subs r2,r2,r4               @ shifting a up by two words: this /can/ go negative or have bad cancellation
 sbcs r3,r3,r5
 cmp r3,#0x01000000          @ check we have at least 57 bits of product so that dmul tail will round correctly (this test is slightly conservative - 55 needed?)
 blt 1f                      @ also trap case where result is negative
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

@ heavy cancellation case
@ r0:r1:r2:r3: result Q124, signed
@ r8: em+en+3
@ r14b0: save complement of sign of a
1:
 sub r8,r8,#1                @ em+en+2
 RDDM r6,r7                  @ clear engaged flag
 blo 2f                      @ if result is negative...
 movs r6,#0                  @ ... negate it...
 subs r0,r6,r0
 sbcs r1,r6,r1
 sbcs r2,r6,r2
 sbcs r3,r6,r3
 eor r14,r14,#1              @ ... and flip saved sign
2:                           @ now normalise result
 orrs r6,r2,r3               @ shift up by 64 possible?
 bne 7f
 movs r3,r1                  @ do it
 movs r2,r0
 movs r1,#0
 movs r0,#0
 sub r8,r8,#64               @ fix exponent
7:
 cmp r3,#0                   @ shift up by 32 possible?
 bne 8f
 movs r3,r2                  @ do it
 movs r2,r1
 movs r1,r0
 movs r0,#0
 sub r8,r8,#32
8:
 cmp r3,#0                   @ is result zero? return it
 beq 9f
 clz r6,r3                   @ k=amount of final shift
 subs r8,r8,r6               @ final exponent
 movs r7,#1
 lsls r7,r7,r6               @ r7=2^k
 muls r3,r3,r7
 subs r7,r7,#1               @ 2^k-1
 umlal r2,r3,r2,r7
 umlal r1,r2,r1,r7
 umlal r0,r1,r0,r7           @ r0:r1:r2:r3: normalised result
 orrs r0,r0,r1               @ any sticky bits below top 64?
 it ne
 orrne r2,r2,#1              @ or into sticky bit
 lsrs r0,r2,#11              @ align to mantissa position for IEEE format
 lsrs r1,r3,#11
 orr r0,r0,r3,lsl#21
 lsls r2,r2,#22              @ rounding bit in C, sticky bit in ~Z
 bcc 10f                     @ no rounding?
 beq 11f                     @ rounding tie?
 adcs r0,r0,#0               @ round up (C is set)
 adcs r1,r1,#0
 adds r8,r8,r1,lsr#20        @ candidate for exponent field
 ble 12f                     @ underflow? overflow cannot occur here as the result is smaller in magnitude than a
 bfi r1,r8,#20,#11           @ insert exponent
 orr r1,r1,r14,lsl#31        @ or in sign
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

11:
 adcs r0,r0,#0               @ round up as above
 adcs r1,r1,#0
 bic r0,r0,#1                @ to even
 adds r8,r8,r1,lsr#20        @ candidate for exponent field
 ble 12f                     @ underflow?
 bfi r1,r8,#20,#11           @ insert exponent
 orr r1,r1,r14,lsl#31        @ or in sign
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

10:
 adds r8,r8,r1,lsr#20        @ candidate for exponent field
 ble 12f                     @ underflow?
 bfi r1,r8,#20,#11           @ insert exponent
 orr r1,r1,r14,lsl#31        @ or in sign
9:
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

12:
 mov r1,r14,lsl#31           @ underflow: return signed zero
 movs r0,#0
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

91:                          @ case where a (Q52) is shifted down relative to mn (Q124); the mod 32 part of the shift of a has already been done
@ r0:r1:r2:r3: mn
@ r4:r5:r6: a
@ r7: alignment shift required (negative)
 cmn r7,#32                  @ shift down one word?
 bge 94f
 cmn r7,#64                  @ shift down two words?
 bge 95f
@ here a is shifted entirely below the bottom of m
 subs r0,r0,#1               @ subtract an epsilon (a is non-zero)
 sbcs r1,r1,#0
 sbcs r2,r2,#0
 sbcs r3,r3,#0
 orr r0,r0,#1                @ ensure the sticky bit is set (a is non-zero)
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

94:
 rsbs r4,r4,#0               @ one word shift down
 sbcs r0,r0,r5
 sbcs r1,r1,r6
 sbcs r2,r2,#0
 sbcs r3,r3,#0
 orr r0,r0,r4                @ sticky bits
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

95:
 movs r7,#0                  @ two words shift down
 subs r4,r7,r4
 sbcs r5,r7,r5
 sbcs r0,r0,r6
 sbcs r1,r1,r7
 sbcs r2,r2,r7
 sbcs r3,r3,r7
 orrs r0,r0,r4               @ sticky bits
 orrs r0,r0,r5
 WXMS r0,r1                  @ write sticky bits
 WXMO r2,r3                  @ write sticky+result bits
 NRDD
 RDDM r0,r1
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

80:
@ here |a| is big compared to |mn|, more precisely ea-(em+en)≥4 so e.g. if ea=0 then em+en≤-4 and mn<4.2^-4=1/4
@ r0:r1:r2:r3: mn Q124
@ r4:r5: a IEEE packed
@ r7<0, em+en-ea+3 (debiased)
@ r14: ea [+0x3ff]
 lsrs r6,r3,#29
 bne 1f                      @ 2≤mn<4?
 adds r2,r2,r2               @ shift up one place
 adcs r3,r3,r3
 subs r7,r7,#1               @ fix exponent
1:                           @ now r2:r3 is mn Q61, sticky bits in r0:r1
 subs r7,r7,#3
@ r7=emn-ea <-3
 orr r0,r0,r2,lsl#23         @ gather sticky bits
 lsrs r2,r2,#9               @ adjust mn to Q52 ready to create packed IEEE version of mn
 orr r2,r2,r3,lsl#23
 lsrs r3,r3,#9
 orrs r0,r0,r1               @ or of all sticky bits
 it ne
 orrne r2,r2,#1              @ sticky bit from bottom 64 bits of mn

 mvn r6,r5,lsr#31            @ complement of sign of a
 orr r3,r3,r6,lsl#31         @ fix sign of mn so we do a subtraction

 adds r14,r7,r14             @ this can go negative; r14 is now at most ea[+0x3ff]-4
 add r3,r3,r14,lsl#20
@ the exponent field in r2:r3 (mn) is now at most ea[+0x3ff]-3
@ that means that in the dadd operation that follows, mn will be shifted down at least three places to align with a,
@ and a post-normalisation shift up of at most one place will be needed
@ therefore in the worst case r2b2 affects b0 of the result; r2b1 affects the rounding of the result; and r2b0 can be used as a sticky bit
 bmi 1f                      @ did exponent go negative?

 dcp_dadd_m r0,r1,r2,r3,r4,r5
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return

1:
 cmn r7,#64                  @ is mn being shifted well below the bottom of a?
 ble 82b                     @ then result is just a
 add r3,r3,#0x40000000       @ otherwise offset exponents by +1024
 add r5,r5,#0x40000000
 dcp_dadd_m r0,r1,r2,r3,r4,r5
 ubfx r2,r1,#20,#11          @ get exponent
 cmp r2,#0x400               @ too small?
 itte ls
 andls r1,r1,0x80000000      @ flush to signed zero
 movls r0,#0
 subhi r1,r1,#0x40000000     @ else fix exponent of result
// todo optimize this based on final decision on saving_func_entry
 pop {r4-r8,lr}
 saving_func_return


double_wrapper_section __dmla
@ cf saving_func macro: but here we need to record the SP before the state save possibly changes it
1:
 push {lr}              // 16-bit instruction
 bl __dcp_save_state    // 32-bit instruction
 b 1f                   // 16-bit instruction

@ r0:r1 m
@ r2:r3 n
@ [r13,#0] a
regular_func mla
 mov r12,sp                  @ save the SP
 PCMP apsr_nzcv              @ test the engaged flag
 bmi 1b
1:
 push {r4,r5,r14}
 dcp_dmul_m r0,r1,r0,r1,r2,r3,r0,r1,r2,r3,r4,r5,r14
 ldrd r2,r3,[r12,#0]        @ fetch a using original SP
 dcp_dadd_m r0,r1,r0,r1,r2,r3
// todo optimize this based on final decision on saving_func_entry
 pop {r4,r5,r14}
 saving_func_return

#endif
