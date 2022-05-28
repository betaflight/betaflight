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
    PGR_SIZE_SYSTEM_FLAG =  0x0000 // documentary
} pgRegistryInternal_e;

// function that resets a single parameter group instance
typedef void (pgResetFunc)(void * /* base */);

typedef struct pgRegistry_s {
    pgn_t pgn;             // The parameter group number, the top 4 bits are reserved for version
    uint8_t length;        // The number of elements in the group
    uint16_t size;         // Size of the group in RAM, the top 4 bits are reserved for flags
    uint8_t *address;      // Address of the group in RAM.
    uint8_t *copy;         // Address of the copy in RAM.
    uint8_t **ptr;         // The pointer to update after loading the record into ram.
    union {
        void *ptr;         // Pointer to init template
        pgResetFunc *fn;   // Popinter to pgResetFunc
    } reset;
    uint32_t *fnv_hash;    // Used to detect if config has changed prior to write
} pgRegistry_t;

static inline uint16_t pgN(const pgRegistry_t* reg) {return reg->pgn & PGR_PGN_MASK;}
static inline uint8_t pgVersion(const pgRegistry_t* reg) {return (uint8_t)(reg->pgn >> 12);}
static inline uint16_t pgSize(const pgRegistry_t* reg) {return reg->size & PGR_SIZE_MASK;}
static inline uint16_t pgElementSize(const pgRegistry_t* reg) {return (reg->size & PGR_SIZE_MASK) / reg->length;}

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

// Reset configuration to default (by name)
#define PG_RESET(_name)                                         \
    do {                                                                \
        extern const pgRegistry_t _name ##_Registry;                    \
        pgReset(&_name ## _Registry);                            \
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
#define PG_DECLARE_ARRAY(_type, _length, _name)                         \
    extern _type _name ## _SystemArray[_length];                        \
    extern _type _name ## _CopyArray[_length];                          \
    static inline const _type* _name(int _index) { return &_name ## _SystemArray[_index]; } \
    static inline _type* _name ## Mutable(int _index) { return &_name ## _SystemArray[_index]; } \
    static inline _type (* _name ## _array(void))[_length] { return &_name ## _SystemArray; } \
    struct _dummy                                                       \
    /**/

// Register system config
#define PG_REGISTER_I(_type, _name, _pgn, _version, _reset)             \
    _type _name ## _System;                                             \
    _type _name ## _Copy;                                               \
    uint32_t _name ## _fnv_hash;                                        \
    /* Force external linkage for g++. Catch multi registration */      \
    extern const pgRegistry_t _name ## _Registry;                       \
    const pgRegistry_t _name ##_Registry PG_REGISTER_ATTRIBUTES = {     \
        .pgn = _pgn | (_version << 12),                                 \
        .length = 1,                                                    \
        .size = sizeof(_type) | PGR_SIZE_SYSTEM_FLAG,                   \
        .address = (uint8_t*)&_name ## _System,                         \
        .fnv_hash = &_name ## _fnv_hash,                                \
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
#define PG_REGISTER_ARRAY_I(_type, _length, _name, _pgn, _version, _reset)  \
    _type _name ## _SystemArray[_length];                               \
    _type _name ## _CopyArray[_length];                                 \
    uint32_t _name ## _fnv_hash;                                        \
    extern const pgRegistry_t _name ##_Registry;                        \
    const pgRegistry_t _name ## _Registry PG_REGISTER_ATTRIBUTES = {    \
        .pgn = _pgn | (_version << 12),                                 \
        .length = _length,                                              \
        .size = (sizeof(_type) * _length) | PGR_SIZE_SYSTEM_FLAG,       \
        .address = (uint8_t*)&_name ## _SystemArray,                    \
        .fnv_hash = &_name ## _fnv_hash,                                \
        .copy = (uint8_t*)&_name ## _CopyArray,                         \
        .ptr = 0,                                                       \
        _reset,                                                         \
    }                                                                   \
    /**/

#define PG_REGISTER_ARRAY(_type, _length, _name, _pgn, _version)        \
    PG_REGISTER_ARRAY_I(_type, _length, _name, _pgn, _version, .reset = {.ptr = 0}) \
    /**/

#define PG_REGISTER_ARRAY_WITH_RESET_FN(_type, _length, _name, _pgn, _version) \
    extern void pgResetFn_ ## _name(_type *);    \
    PG_REGISTER_ARRAY_I(_type, _length, _name, _pgn, _version, .reset = {.fn = (pgResetFunc*)&pgResetFn_ ## _name}) \
    /**/

#if 0
// ARRAY reset mechanism is not implemented yet, only few places in code would benefit from it - See pgResetInstance
#define PG_REGISTER_ARRAY_WITH_RESET_TEMPLATE(_type, _length, _name, _pgn, _version) \
    extern const _type pgResetTemplate_ ## _name;                       \
    PG_REGISTER_ARRAY_I(_type, _length, _name, _pgn, _version, .reset = {.ptr = (void*)&pgResetTemplate_ ## _name}) \
    /**/
#endif

#define PG_ARRAY_ELEMENT_OFFSET(type, index, member) (index * sizeof(type) + offsetof(type, member))

// Emit reset defaults for config.
// Config must be registered with PG_REGISTER_<xxx>_WITH_RESET_TEMPLATE macro
#define PG_RESET_TEMPLATE(_type, _name, ...)                            \
    const _type pgResetTemplate_ ## _name PG_RESETDATA_ATTRIBUTES = {   \
        __VA_ARGS__                                                     \
    }                                                                   \
    /**/

#define CONVERT_PARAMETER_TO_FLOAT(param) (0.001f * param)
#define CONVERT_PARAMETER_TO_PERCENT(param) (0.01f * param)

const pgRegistry_t* pgFind(pgn_t pgn);

bool pgLoad(const pgRegistry_t* reg, const void *from, int size, int version);
int pgStore(const pgRegistry_t* reg, void *to, int size);
void pgResetAll(void);
void pgResetInstance(const pgRegistry_t *reg, uint8_t *base);
bool pgResetCopy(void *copy, pgn_t pgn);
void pgReset(const pgRegistry_t* reg);
