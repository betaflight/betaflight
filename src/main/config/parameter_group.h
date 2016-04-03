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

typedef uint16_t pgn_t;

// parameter group registry flags
typedef enum {
    PGRF_NONE = 0,
    PGRF_CLASSIFICATON_BIT = (1 << 0),
} pgRegistryFlags_e;

typedef enum {
    PGR_PGN_MASK =          0x0fff,
    PGR_PGN_VERSION_MASK =  0xf000,
    PGR_SIZE_MASK =         0x0fff,
    PGR_SIZE_SYSTEM_FLAG =  0x0000, // documentary
    PGR_SIZE_PROFILE_FLAG = 0x8000, // start using flags from the top bit down
} pgRegistryInternal_e;

typedef struct pgRegistry_s {
    pgn_t pgn;          // The parameter group number, the top 4 bits are reserved for version
    uint16_t size;      // Size of the group in RAM, the top 4 bits are reserved for flags
    uint8_t *address;   // Address of the group in RAM.
    uint8_t **ptr;      // The pointer to update after loading the record into ram.
} pgRegistry_t;

static inline uint16_t pgN(const pgRegistry_t* reg) {return reg->pgn & PGR_PGN_MASK;}
static inline uint8_t pgVersion(const pgRegistry_t* reg) {return reg->pgn >> 12;}
static inline uint16_t pgSize(const pgRegistry_t* reg) {return reg->size & PGR_SIZE_MASK;}
static inline uint16_t pgIsSystem(const pgRegistry_t* reg) {return (reg->size & PGR_SIZE_PROFILE_FLAG) == 0;}

#define PG_PACKED __attribute__((packed))

#ifdef __APPLE__
extern const pgRegistry_t __pg_registry_start[] __asm("section$start$__DATA$__pg_registry");
extern const pgRegistry_t __pg_registry_end[] __asm("section$end$__DATA$__pg_registry");
#define PG_REGISTER_ATTRIBUTES __attribute__ ((section("__DATA,__pg_registry"), used, aligned(4)))
#else
extern const pgRegistry_t __pg_registry_start[];
extern const pgRegistry_t __pg_registry_end[];
#define PG_REGISTER_ATTRIBUTES __attribute__ ((section(".pg_registry"), used, aligned(4)))
#endif

#define PG_REGISTRY_SIZE (__pg_registry_end - __pg_registry_start)

// Helper to iterate over the PG register.  Cheaper than a visitor style callback.
#define PG_FOREACH(_name) \
    for (const pgRegistry_t *(_name) = __pg_registry_start; (_name) < __pg_registry_end; _name++)

#define PG_FOREACH_PROFILE(_name)                                    \
    PG_FOREACH(_name)                                                \
        if(pgIsSystem(_name)) \
            continue;                                                \
        else                                                         \
            /**/

#define PG_DECLARE(_type, _name)                                        \
    extern _type _name ## _System;                                      \
    static inline _type* _name(void) { return &_name ## _System; }      \
    struct _dummy                                                        \
    /**/

#define PG_DECLARE_ARR(_type, _size, _name)                             \
    extern _type _name ## _SystemArray[_size];                          \
    static inline _type* _name(int _index) { return &_name ## _SystemArray[_index]; } \
    static inline _type (* _name ## _arr(void))[_size] { return &_name ## _SystemArray; } \
    struct _dummy                                                        \
    /**/

#define PG_DECLARE_PROFILE(_type, _name)                                \
    extern _type *_name ## _ProfileCurrent;                             \
    static inline _type* _name(void) { return _name ## _ProfileCurrent; } \
    struct _dummy                                                        \
    /**/

// Register config
#define PG_REGISTER(_type, _name, _pgn, _version)                       \
    _type _name ## _System;                                             \
    static const pgRegistry_t _name ##_Registry PG_REGISTER_ATTRIBUTES = { \
        .pgn = _pgn | (_version << 12),                                 \
        .size = sizeof(_type) | PGR_SIZE_SYSTEM_FLAG,                   \
        .address = (uint8_t*)&_name ## _System,                         \
        .ptr = 0,                                                       \
    }                                                                   \
    /**/

// Register config
#define PG_REGISTER_ARR(_type, _size, _name, _pgn, _version)            \
    _type _name ## _SystemArray[_size];                                 \
    static const pgRegistry_t _name ## _Registry PG_REGISTER_ATTRIBUTES = { \
        .pgn = _pgn | (_version << 12),                                 \
        .size = sizeof(_type) | PGR_SIZE_SYSTEM_FLAG,                   \
        .address = (uint8_t*)&_name ## _SystemArray,                    \
        .ptr = 0,                                                       \
    }                                                                   \
    /**/


#ifdef UNIT_TEST
# define PG_ASSIGN(_type, _name)                \
    _type *_name ## _ProfileCurrent = &_name ## _Storage[0];
#else
# define PG_ASSIGN(_type, _name)                \
    _type *_name ## _ProfileCurrent;
#endif

#define PG_REGISTER_PROFILE(_type, _name, _pgn, _version)               \
    STATIC_UNIT_TESTED _type _name ## _Storage[MAX_PROFILE_COUNT];      \
    PG_ASSIGN(_type, _name)                                             \
    static const pgRegistry_t _name ## _Registry PG_REGISTER_ATTRIBUTES = { \
        .pgn = _pgn | (_version << 12),                                 \
        .size = sizeof(_type) | PGR_SIZE_PROFILE_FLAG,                  \
        .address = (uint8_t*)&_name ## _Storage,                        \
        .ptr = (uint8_t **)&_name ## _ProfileCurrent,                   \
    }                                                                   \
    /**/

typedef uint8_t (*pgMatcherFuncPtr)(const pgRegistry_t *candidate, const void *criteria);

const pgRegistry_t* pgFind(pgn_t pgn);
const pgRegistry_t* pgMatcher(pgMatcherFuncPtr matcher, const void *criteria);
void pgLoad(const pgRegistry_t* reg, const void *from, int size);
void pgResetAll(uint8_t profileCount);
void pgActivateProfile(uint8_t profileIndexToActivate);
