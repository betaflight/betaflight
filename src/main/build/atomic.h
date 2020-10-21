/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#if !defined(UNIT_TEST)
// BASEPRI manipulation functions
// only set_BASEPRI is implemented in device library. It does always create memory barrier
// missing versions are implemented here

// set BASEPRI register, do not create memory barrier
__attribute__( ( always_inline ) ) static inline void __set_BASEPRI_nb(uint32_t basePri)
{
   __ASM volatile ("\tMSR basepri, %0\n" : : "r" (basePri) );
}

// set BASEPRI_MAX register, do not create memory barrier
__attribute__( ( always_inline ) ) static inline void __set_BASEPRI_MAX_nb(uint32_t basePri)
{
   __ASM volatile ("\tMSR basepri_max, %0\n" : : "r" (basePri) );
}

#endif

#if defined(UNIT_TEST)
// atomic related functions for unittest.

extern uint8_t atomic_BASEPRI;

static inline uint8_t __get_BASEPRI(void)
{
    return atomic_BASEPRI;
}

// restore BASEPRI (called as cleanup function), with global memory barrier
static inline void __basepriRestoreMem(uint8_t *val)
{
    atomic_BASEPRI = *val;
    asm volatile ("": : :"memory"); // compiler memory barrier
}

// increase BASEPRI, with global memory barrier, returns true
static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
    if(prio && (atomic_BASEPRI == 0 || atomic_BASEPRI > prio)) {
        atomic_BASEPRI = prio;
    }
    asm volatile ("": : :"memory"); // compiler memory barrier
    return 1;
}

// restore BASEPRI (called as cleanup function), no memory barrier
static inline void __basepriRestore(uint8_t *val)
{
    atomic_BASEPRI = *val;
}

// increase BASEPRI, no memory barrier, returns true
static inline uint8_t __basepriSetRetVal(uint8_t prio)
{
    if(prio && (atomic_BASEPRI == 0 || atomic_BASEPRI > prio)) {
        atomic_BASEPRI = prio;
    }
    return 1;
}

#else
// ARM BASEPRI manipulation

// restore BASEPRI (called as cleanup function), with global memory barrier
static inline void __basepriRestoreMem(uint8_t *val)
{
    __set_BASEPRI(*val);
}

// set BASEPRI_MAX, with global memory barrier, returns true
static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
    __set_BASEPRI_MAX(prio);
    return 1;
}

// restore BASEPRI (called as cleanup function), no memory barrier
static inline void __basepriRestore(uint8_t *val)
{
    __set_BASEPRI_nb(*val);
}

// set BASEPRI_MAX, no memory barrier, returns true
static inline uint8_t __basepriSetRetVal(uint8_t prio)
{
    __set_BASEPRI_MAX_nb(prio);
    return 1;
}

#endif

// Run block with elevated BASEPRI (using BASEPRI_MAX), restoring BASEPRI on exit.
// All exit paths are handled. Implemented as for loop, does intercept break and continue
// Full memory barrier is placed at start and at exit of block
// __unused__ attribute is used to supress CLang warning
#define ATOMIC_BLOCK(prio) for ( uint8_t __basepri_save __attribute__ ((__cleanup__ (__basepriRestoreMem), __unused__)) = __get_BASEPRI(), \
                                     __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __ToDo = 0 )

// Run block with elevated BASEPRI (using BASEPRI_MAX), but do not create memory barrier.
// Be careful when using this, you must use some method to prevent optimizer from breaking things
// - lto is used for Cleanflight compilation, so function call is not memory barrier
// - use ATOMIC_BARRIER or volatile to protect used variables
// - gcc 4.8.4 does write all values in registers to memory before 'asm volatile', so this optimization does not help much
// - gcc 5 and later works as intended, generating quite optimal code
#define ATOMIC_BLOCK_NB(prio) for ( uint8_t __basepri_save __attribute__ ((__cleanup__ (__basepriRestore), __unused__)) = __get_BASEPRI(), \
                                    __ToDo = __basepriSetRetVal(prio); __ToDo ; __ToDo = 0 ) \

// ATOMIC_BARRIER
// Create memory barrier
// - at the beginning of containing block (value of parameter must be reread from memory)
// - at exit of block (all exit paths) (parameter value if written into memory, but may be cached in register for subsequent use)
// On gcc 5 and higher, this protects only memory passed as parameter (any type can be used)
// this macro can be used only ONCE PER LINE, but multiple uses per block are fine

#ifndef __UNIQL
# define __UNIQL_CONCAT2(x,y) x ## y
# define __UNIQL_CONCAT(x,y) __UNIQL_CONCAT2(x,y)
# define __UNIQL(x) __UNIQL_CONCAT(x,__LINE__)
#endif

#define ATOMIC_BARRIER_ENTER(dataPtr, refStr)                              \
    __asm__ volatile ("\t# barrier (" refStr ") enter\n" : "+m" (*(dataPtr)))

#define ATOMIC_BARRIER_LEAVE(dataPtr, refStr)                              \
    __asm__ volatile ("\t# barrier (" refStr ") leave\n" : "m" (*(dataPtr)))

#if defined(__clang__)
// CLang version, using Objective C-style block
// based on https://stackoverflow.com/questions/24959440/rewrite-gcc-cleanup-macro-with-nested-function-for-clang
typedef void (^__cleanup_block)(void);
static inline void __do_cleanup(__cleanup_block * b) { (*b)(); }

#define ATOMIC_BARRIER(data)                                            \
    typeof(data) *__UNIQL(__barrier) = &data;                           \
    ATOMIC_BARRIER_ENTER(__UNIQL(__barrier), #data);                    \
    __cleanup_block __attribute__((cleanup(__do_cleanup), __unused__)) __UNIQL(__cleanup) = \
        ^{  ATOMIC_BARRIER_LEAVE(__UNIQL(__barrier), #data); };         \
    do {} while(0)                                                      \
/**/
#else
// gcc version, uses local function for cleanup.
#define ATOMIC_BARRIER(data)                                            \
    __extension__ void  __UNIQL(__barrierEnd)(typeof(data) **__d) {     \
         ATOMIC_BARRIER_LEAVE(*__d, #data);                             \
    }                                                                   \
    typeof(data) __attribute__((__cleanup__(__UNIQL(__barrierEnd)))) *__UNIQL(__barrier) = &data; \
    ATOMIC_BARRIER_ENTER(__UNIQL(__barrier), #data);                    \
    do {} while(0)                                                      \
/**/
#endif

// define these wrappers for atomic operations, using gcc builtins
#define ATOMIC_OR(ptr, val) __sync_fetch_and_or(ptr, val)
#define ATOMIC_AND(ptr, val) __sync_fetch_and_and(ptr, val)
