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

#include <stddef.h>
#include <stdint.h>

#define NOOP do {} while (0)

#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define ARRAYEND(x) (&(x)[ARRAYLEN(x)])

#define CONST_CAST(type, value) ((type)(value))

#define CONCAT_HELPER(x,y) x ## y
#define CONCAT(x,y) CONCAT_HELPER(x, y)
#define CONCAT2(_1,_2) CONCAT(_1, _2)
#define CONCAT3(_1,_2,_3)  CONCAT(CONCAT(_1, _2), _3)
#define CONCAT4(_1,_2,_3,_4)  CONCAT(CONCAT3(_1, _2, _3), _4)

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define EXPAND_I(x) x
#define EXPAND(x) EXPAND_I(x)

// expand to t if bit is 1, f when bit is 0. Other bit values are not supported
#define PP_IIF(bit, t, f) PP_IIF_I(bit, t, f)
#define PP_IIF_I(bit, t, f) PP_IIF_ ## bit(t, f)
#define PP_IIF_0(t, f) f
#define PP_IIF_1(t, f) t

// Expand all arguments and call macro with them. When expansion of some argument contains ',', it will be passed as multiple arguments
// #define TAKE3(_1,_2,_3) CONCAT3(_1,_2,_3)
// #define MULTI2 A,B
// PP_CALL(TAKE3, MULTI2, C) expands to ABC
#define PP_CALL(macro, ...) macro(__VA_ARGS__)

#if !defined(UNUSED)
#define UNUSED(x) (void)(x) // Variables and parameters that are not used
#endif

#define DISCARD(x) (void)(x) // To explicitly ignore result of x (usually an I/O register access).

#define STATIC_ASSERT(condition, name) _Static_assert((condition), #name)


#define BIT(x) (1 << (x))

/*
http://resnet.uoregon.edu/~gurney_j/jmpc/bitwise.html
*/
#define BITCOUNT(x) (((BX_(x)+(BX_(x)>>4)) & 0x0F0F0F0F) % 255)
#define BX_(x) ((x) - (((x)>>1)&0x77777777) - (((x)>>2)&0x33333333) - (((x)>>3)&0x11111111))


/*
 * https://groups.google.com/forum/?hl=en#!msg/comp.lang.c/attFnqwhvGk/sGBKXvIkY3AJ
 * Return (v ? floor(log2(v)) : 0) when 0 <= v < 1<<[8, 16, 32, 64].
 * Inefficient algorithm, intended for compile-time constants.
 */
#define LOG2_8BIT(v)  (8 - 90/(((v)/4+14)|1) - 2/((v)/2+1))
#define LOG2_16BIT(v) (8*((v)>255) + LOG2_8BIT((v) >>8*((v)>255)))
#define LOG2_32BIT(v) (16*((v)>65535L) + LOG2_16BIT((v)*1L >>16*((v)>65535L)))
#define LOG2_64BIT(v) \
    (32*((v)/2L>>31 > 0) \
     + LOG2_32BIT((v)*1L >>16*((v)/2L>>31 > 0) \
                         >>16*((v)/2L>>31 > 0)))
#define LOG2(v) LOG2_64BIT(v)

#if 0
// ISO C version, but no type checking
#define container_of(ptr, type, member) \
                      ((type *) ((char *)(ptr) - offsetof(type, member)))
#else
// non ISO variant from linux kernel; checks ptr type, but triggers 'ISO C forbids braced-groups within expressions [-Wpedantic]'
//  __extension__ is here to disable this warning
#define container_of(ptr, type, member)  ( __extension__ ({     \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );}))

static inline int16_t cmp16(uint16_t a, uint16_t b) { return (int16_t)(a-b); }
static inline int32_t cmp32(uint32_t a, uint32_t b) { return (int32_t)(a-b); }

static inline uint32_t llog2(uint32_t n) { return 31 - __builtin_clz(n | 1); }

// using memcpy_fn will force memcpy function call, instead of inlining it. In most cases function call takes fewer instructions
//  than inlined version (inlining is cheaper for very small moves < 8 bytes / 2 store instructions)
#if defined(UNIT_TEST) || defined(SIMULATOR_BUILD)
// Call memcpy when building unittest - this is easier that asm symbol name mangling (symbols start with _underscore on win32)
#include <string.h>
static inline void  memcpy_fn ( void * destination, const void * source, size_t num ) { memcpy(destination, source, num); }
#else
void * memcpy_fn ( void * destination, const void * source, size_t num ) asm("memcpy");
#endif


#endif

#if __GNUC__ > 6
#define FALLTHROUGH ;__attribute__ ((fallthrough))
#else
#define FALLTHROUGH do {} while(0)
#endif
