/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "build/build_config.h"

typedef uint16_t pgn_t;

// parameter group registry flags
typedef enum {
    PGRF_NONE = 0,
    PGRF_CLASSIFICATON_BIT = (1 << 0)
} pgRegistryFlags_e;

typedef enum {
    PGR_PGN_MASK =          0x0fff,
    PGR_PGN_VERSION_MASK =  0xf000,
    PGR_SIZE_MASK =         0x0fff,
    PGR_SIZE_SYSTEM_FLAG =  0x0000, // documentary
    PGR_SIZE_PROFILE_FLAG = 0x8000  // start using flags from the top bit down
} pgRegistryInternal_e;

// function that resets a single parameter group instance
typedef void (pgResetFunc)(void * /* base */, int /* size */);

typedef struct pgRegistry_s {
    pgn_t pgn;             // The parameter group number, the top 4 bits are reserved for version
    uint16_t size;         // Size of the group in RAM, the top 4 bits are reserved for flags
    uint8_t *address;      // Address of the group in RAM.
    uint8_t *copy;         // Address of the copy in RAM.
    uint8_t **ptr;         // The pointer to update after loading the record into ram.
    union {
        void *ptr;         // Pointer to init template
        pgResetFunc *fn;   // Popinter to pgResetFunc
    } reset;
} pgRegistry_t;

static inline uint16_t pgN(const pgRegistry_t* reg) {return reg->pgn & PGR_PGN_MASK;}
static inline uint8_t pgVersion(const pgRegistry_t* reg) {return (uint8_t)(reg->pgn >> 12);}
static inline uint16_t pgSize(const pgRegistry_t* reg) {return reg->size & PGR_SIZE_MASK;}
static inline uint16_t pgIsSystem(const pgRegistry_t* reg) {return (reg->size & PGR_SIZE_PROFILE_FLAG) == 0;}
static inline uint16_t pgIsProfile(const pgRegistry_t* reg) {return (reg->size & PGR_SIZE_PROFILE_FLAG) == PGR_SIZE_PROFILE_FLAG;}

#define PG_PACKED __attribute__((packed))

#ifdef __APPLE__
extern const pgRegistry_t __pg_registry_start[] __asm("section$start$__DATA$__pg_registry");
extern const pgRegistry_t __pg_registry_end[] __asm("section$end$__DATA$__pg_registry");
#define PG_REGISTER_ATTRIBUTES __attribute__ ((section("__DATA,__pg_registry"), used, aligned(4)))

extern const uint8_t __pg_resetdata_start[] __asm("section$start$__DATA$__pg_resetdata");
extern const uint8_t __pg_resetdata_end[] __asm("section$end$__DATA$__pg_resetdata");
#define PG_RESETDATA_ATTRIBUTES __attribute__ ((section("__DATA,__pg_resetdata"), used, aligned(2)))
#else
extern const pgRegistry_t __pg_registry_start[];
extern const pgRegistry_t __pg_registry_end[];
#define PG_REGISTER_ATTRIBUTES __attribute__ ((section(".pg_registry"), used, aligned(4)))

extern const uint8_t __pg_resetdata_start[];
extern const uint8_t __pg_resetdata_end[];
#define PG_RESETDATA_ATTRIBUTES __attribute__ ((section(".pg_resetdata"), used, aligned(2)))
#endif

#define PG_REGISTRY_SIZE (__pg_registry_end - __pg_registry_start)

// Helper to iterate over the PG register.  Cheaper than a visitor style callback.
#define PG_FOREACH(_name) \
    for (const pgRegistry_t *(_name) = __pg_registry_start; (_name) < __pg_registry_end; _name++)

#define PG_FOREACH_PROFILE(_name)                                    \
    PG_FOREACH(_name)                                                \
        if (pgIsSystem(_name)) \
            continue;                                                \
        else                                                         \
            /**/

// Reset configuration to default (by name)
// Only current profile is reset for profile based configs
#define PG_RESET_CURRENT(_name)                                         \
    do {                                                                \
        extern const pgRegistry_t _name ##_Registry;                    \
        pgResetCurrent(&_name ## _Registry);                            \
    } while (0)                                                          \
    /**/

// Declare system config
#define PG_DECLARE(_type, _name)                                        \
    extern _type _name ## _System;                                      \
    extern _type _name ## _Copy;                                        \
    static inline const _type* _name(void) { return &_name ## _System; }\
    static inline _type* _name ## Mutable(void) { return &_name ## _System; }\
    struct _dummy                                                       \
    /**/

// Declare system config array
#define PG_DECLARE_ARRAY(_type, _size, _name)                           \
    extern _type _name ## _SystemArray[_size];                          \
    extern _type _name ## _CopyArray[_size];                            \
    static inline const _type* _name(int _index) { return &_name ## _SystemArray[_index]; } \
    static inline _type* _name ## Mutable(int _index) { return &_name ## _SystemArray[_index]; } \
    static inline _type (* _name ## _array(void))[_size] { return &_name ## _SystemArray; } \
    struct _dummy                                                       \
    /**/

// Declare profile config
#define PG_DECLARE_PROFILE(_type, _name)                                \
    extern _type *_name ## _ProfileCurrent;                             \
    static inline const _type* _name(void) { return _name ## _ProfileCurrent; } \
    static inline _type* _name ## Mutable(void) { return _name ## _ProfileCurrent; } \
    struct _dummy                                                       \
    /**/


// Register system config
#define PG_REGISTER_I(_type, _name, _pgn, _version, _reset)             \
    _type _name ## _System;                                             \
    _type _name ## _Copy;                                               \
    /* Force external linkage for g++. Catch multi registration */      \
    extern const pgRegistry_t _name ## _Registry;                       \
    const pgRegistry_t _name ##_Registry PG_REGISTER_ATTRIBUTES = {     \
        .pgn = _pgn | (_version << 12),                                 \
        .size = sizeof(_type) | PGR_SIZE_SYSTEM_FLAG,                   \
        .address = (uint8_t*)&_name ## _System,                         \
        .copy = (uint8_t*)&_name ## _Copy,                              \
        .ptr = 0,                                                       \
        _reset,                                                         \
    }                                                                   \
    /**/

#define PG_REGISTER(_type, _name, _pgn, _version)                       \
    PG_REGISTER_I(_type, _name, _pgn, _version, .reset = {.ptr = 0})    \
    /**/

#define PG_REGISTER_WITH_RESET_FN(_type, _name, _pgn, _version)         \
    extern void pgResetFn_ ## _name(_type *);                           \
    PG_REGISTER_I(_type, _name, _pgn, _version, .reset = {.fn = (pgResetFunc*)&pgResetFn_ ## _name }) \
    /**/

#define PG_REGISTER_WITH_RESET_TEMPLATE(_type, _name, _pgn, _version)   \
    extern const _type pgResetTemplate_ ## _name;                       \
    PG_REGISTER_I(_type, _name, _pgn, _version, .reset = {.ptr = (void*)&pgResetTemplate_ ## _name}) \
    /**/

// Register system config array
#define PG_REGISTER_ARRAY_I(_type, _size, _name, _pgn, _version, _reset)  \
    _type _name ## _SystemArray[_size];                                 \
    _type _name ## _CopyArray[_size];                                   \
    extern const pgRegistry_t _name ##_Registry;                        \
    const pgRegistry_t _name ## _Registry PG_REGISTER_ATTRIBUTES = {    \
        .pgn = _pgn | (_version << 12),                                 \
        .size = (sizeof(_type) * _size) | PGR_SIZE_SYSTEM_FLAG,         \
        .address = (uint8_t*)&_name ## _SystemArray,                    \
        .copy = (uint8_t*)&_name ## _CopyArray,                         \
        .ptr = 0,                                                       \
        _reset,                                                         \
    }                                                                   \
    /**/

#define PG_REGISTER_ARRAY(_type, _size, _name, _pgn, _version)            \
    PG_REGISTER_ARRAY_I(_type, _size, _name, _pgn, _version, .reset = {.ptr = 0}) \
    /**/

#define PG_REGISTER_ARRAY_WITH_RESET_FN(_type, _size, _name, _pgn, _version) \
    extern void pgResetFn_ ## _name(_type *);    \
    PG_REGISTER_ARRAY_I(_type, _size, _name, _pgn, _version, .reset = {.fn = (pgResetFunc*)&pgResetFn_ ## _name}) \
    /**/

#if 0
// ARRAY reset mechanism is not implemented yet, only few places in code would benefit from it - See pgResetInstance
#define PG_REGISTER_ARRAY_WITH_RESET_TEMPLATE(_type, _size, _name, _pgn, _version) \
    extern const _type pgResetTemplate_ ## _name;                       \
    PG_REGISTER_ARRAY_I(_type, _size, _name, _pgn, _version, .reset = {.ptr = (void*)&pgResetTemplate_ ## _name}) \
    /**/
#endif

#ifdef UNIT_TEST
# define _PG_PROFILE_CURRENT_DECL(_type, _name)                  \
    _type *_name ## _ProfileCurrent = &_name ## _Storage[0];
#else
# define _PG_PROFILE_CURRENT_DECL(_type, _name)  \
    _type *_name ## _ProfileCurrent;
#endif

// register profile config
#define PG_REGISTER_PROFILE_I(_type, _name, _pgn, _version, _reset)     \
    STATIC_UNIT_TESTED _type _name ## _Storage[MAX_PROFILE_COUNT];      \
    STATIC_UNIT_TESTED _type _name ## _CopyStorage[MAX_PROFILE_COUNT];  \
    _PG_PROFILE_CURRENT_DECL(_type, _name)                              \
    extern const pgRegistry_t _name ## _Registry;                       \
    const pgRegistry_t _name ## _Registry PG_REGISTER_ATTRIBUTES = {    \
        .pgn = _pgn | (_version << 12),                                 \
        .size = sizeof(_type) | PGR_SIZE_PROFILE_FLAG,                  \
        .address = (uint8_t*)&_name ## _Storage,                        \
        .copy = (uint8_t*)&_name ## _CopyStorage,                       \
        .ptr = (uint8_t **)&_name ## _ProfileCurrent,                   \
        _reset,                                                         \
    }                                                                   \
    /**/

#define PG_REGISTER_PROFILE(_type, _name, _pgn, _version)               \
    PG_REGISTER_PROFILE_I(_type, _name, _pgn, _version, .reset = {.ptr = 0}) \
    /**/

#define PG_REGISTER_PROFILE_WITH_RESET_FN(_type, _name, _pgn, _version) \
    extern void pgResetFn_ ## _name(_type *);                           \
    PG_REGISTER_PROFILE_I(_type, _name, _pgn, _version, .reset = {.fn = (pgResetFunc*)&pgResetFn_ ## _name}) \
    /**/

#define PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(_type, _name, _pgn, _version) \
    extern const _type pgResetTemplate_ ## _name;                       \
    PG_REGISTER_PROFILE_I(_type, _name, _pgn, _version, .reset = {.ptr = (void*)&pgResetTemplate_ ## _name}) \
    /**/


// Emit reset defaults for config.
// Config must be registered with PG_REGISTER_<xxx>_WITH_RESET_TEMPLATE macro
#define PG_RESET_TEMPLATE(_type, _name, ...)                            \
    const _type pgResetTemplate_ ## _name PG_RESETDATA_ATTRIBUTES = {   \
        __VA_ARGS__                                                     \
    }                                                                   \
    /**/

const pgRegistry_t* pgFind(pgn_t pgn);

void pgLoad(const pgRegistry_t* reg, int profileIndex, const void *from, int size, int version);
int pgStore(const pgRegistry_t* reg, void *to, int size, uint8_t profileIndex);
void pgResetAll(int profileCount);
void pgResetCurrent(const pgRegistry_t *reg);
bool pgResetCopy(void *copy, pgn_t pgn);
void pgReset(const pgRegistry_t* reg, int profileIndex);
void pgActivateProfile(int profileIndex);
