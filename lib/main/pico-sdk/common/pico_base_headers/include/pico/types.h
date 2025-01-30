/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_TYPES_H
#define _PICO_TYPES_H

#ifndef __ASSEMBLER__

#include "pico/assert.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef unsigned int uint;

// PICO_CONFIG: PICO_OPAQUE_ABSOLUTE_TIME_T, Enable opaque type for absolute_time_t to help catch inadvertent confusing uint64_t delays with absolute times, default=0, advanced=true, group=pico_base
#ifndef PICO_OPAQUE_ABSOLUTE_TIME_T
#define PICO_OPAQUE_ABSOLUTE_TIME_T 0
#endif

/*! \typedef absolute_time_t
    \brief An opaque 64 bit timestamp in microseconds

    The type is used instead of a raw uint64_t to prevent accidentally passing relative times or times in the wrong
    time units where an absolute time is required.

    note: As of SDK 2.0.0 this type defaults to being a uin64_t (i.e. no protection); it is enabled
    by setting PICO_OPAQUE_ABSOLUTE_TIME_T to 1

    \see to_us_since_boot()
    \see update_us_since_boot()
    \ingroup timestamp
*/
#if PICO_OPAQUE_ABSOLUTE_TIME_T
typedef struct {
    uint64_t _private_us_since_boot;
} absolute_time_t;
#else
typedef uint64_t absolute_time_t;
#endif

/*! fn to_us_since_boot
 * \brief convert an absolute_time_t into a number of microseconds since boot.
 * \param t the absolute time to convert
 * \return a number of microseconds since boot, equivalent to t
 * \ingroup timestamp
 */
static inline uint64_t to_us_since_boot(absolute_time_t t) {
#ifdef PICO_DEBUG_ABSOLUTE_TIME_T
    return t._private_us_since_boot;
#else
    return t;
#endif
}

/*! fn update_us_since_boot
 * \brief update an absolute_time_t value to represent a given number of microseconds since boot
 * \param t the absolute time value to update
 * \param us_since_boot the number of microseconds since boot to represent. Note this should be representable
 *                      as a signed 64 bit integer
 * \ingroup timestamp
 */
static inline void update_us_since_boot(absolute_time_t *t, uint64_t us_since_boot) {
#ifdef PICO_DEBUG_ABSOLUTE_TIME_T
    assert(us_since_boot <= INT64_MAX);
    t->_private_us_since_boot = us_since_boot;
#else
    *t = us_since_boot;
#endif
}

/*! fn from_us_since_boot
 * \brief convert a number of microseconds since boot to an absolute_time_t
 * \param us_since_boot number of microseconds since boot
 * \return an absolute time equivalent to us_since_boot
 * \ingroup timestamp
 */
static inline absolute_time_t from_us_since_boot(uint64_t us_since_boot) {
    absolute_time_t t;
    update_us_since_boot(&t, us_since_boot);
    return t;
}

#ifdef NDEBUG
#define ABSOLUTE_TIME_INITIALIZED_VAR(name, value) name = value
#else
#define ABSOLUTE_TIME_INITIALIZED_VAR(name, value) name = {value}
#endif

// PICO_CONFIG: PICO_INCLUDE_RTC_DATETIME, Whether to include the datetime_t type used with the RP2040 RTC hardware, default=1 on RP2040, group=util_datetime
#ifndef PICO_INCLUDE_RTC_DATETIME
#define PICO_INCLUDE_RTC_DATETIME PICO_RP2040
#endif

#if PICO_INCLUDE_RTC_DATETIME
/** \struct datetime_t
 *  \ingroup util_datetime
 *  \brief Structure containing date and time information
 *
 *    When setting an RTC alarm, set a field to -1 tells
 *    the RTC to not match on this field
 */
typedef struct {
    int16_t year;    ///< 0..4095
    int8_t month;    ///< 1..12, 1 is January
    int8_t day;      ///< 1..28,29,30,31 depending on month
    int8_t dotw;     ///< 0..6, 0 is Sunday
    int8_t hour;     ///< 0..23
    int8_t min;      ///< 0..59
    int8_t sec;      ///< 0..59
} datetime_t;
#endif

#define bool_to_bit(x) ((uint)!!(x))

#endif
#endif
