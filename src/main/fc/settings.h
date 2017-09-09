#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config/parameter_group.h"

#include "fc/settings_generated.h"

typedef struct lookupTableEntry_s {
    const char * const *values;
    const uint8_t valueCount;
} lookupTableEntry_t;

extern const lookupTableEntry_t cliLookupTables[];

#define VALUE_TYPE_OFFSET 0
#define VALUE_SECTION_OFFSET 4
#define VALUE_MODE_OFFSET 6

typedef enum {
    // value type, bits 0-3
    VAR_UINT8 = (0 << VALUE_TYPE_OFFSET),
    VAR_INT8 = (1 << VALUE_TYPE_OFFSET),
    VAR_UINT16 = (2 << VALUE_TYPE_OFFSET),
    VAR_INT16 = (3 << VALUE_TYPE_OFFSET),
    VAR_UINT32 = (4 << VALUE_TYPE_OFFSET),
    VAR_FLOAT = (5 << VALUE_TYPE_OFFSET), // 0x05

    // value section, bits 4-5
    MASTER_VALUE = (0 << VALUE_SECTION_OFFSET),
    PROFILE_VALUE = (1 << VALUE_SECTION_OFFSET),
    CONTROL_RATE_VALUE = (2 << VALUE_SECTION_OFFSET), // 0x20
    // value mode, bits 6-7
    MODE_DIRECT = (0 << VALUE_MODE_OFFSET),
    MODE_LOOKUP = (1 << VALUE_MODE_OFFSET), // 0x40
} cliValueFlag_e;

#define VALUE_TYPE_MASK (0x0F)
#define VALUE_SECTION_MASK (0x30)
#define VALUE_MODE_MASK (0xC0)

typedef struct cliMinMaxConfig_s {
    const uint8_t indexes[CLIVALUE_MIN_MAX_INDEX_BYTES];
} cliMinMaxConfig_t;

typedef struct cliLookupTableConfig_s {
    const uint8_t tableIndex;
} cliLookupTableConfig_t;

typedef union {
    cliLookupTableConfig_t lookup;
    cliMinMaxConfig_t minmax;
} cliValueConfig_t;

typedef struct {
    const uint8_t encoded_name[CLIVALUE_ENCODED_NAME_MAX_BYTES];
    const uint8_t type; // see cliValueFlag_e
    const cliValueConfig_t config;
    clivalue_offset_t offset;

} __attribute__((packed)) clivalue_t;

extern const clivalue_t cliValueTable[];

void clivalue_get_name(const clivalue_t *val, char *buf);
bool clivalue_name_contains(const clivalue_t *val, char *buf, const char *cmdline);
bool clivalue_name_exact_match(const clivalue_t *val, char *buf, const char *cmdline, uint8_t var_name_length);
pgn_t clivalue_get_pgn(const clivalue_t *val);
// Returns the minimum valid value for the given clivalue_t. clivalue_min_t
// depends on the target and build options, but will always be a signed
// integer (e.g. intxx_t,)
clivalue_min_t clivalue_get_min(const clivalue_t *val);
// Returns the maximum valid value for the given clivalue_t. clivalue_max_t
// depends on the target and build options, but will always be an unsigned
// integer (e.g. uintxx_t,)
clivalue_max_t clivalue_get_max(const clivalue_t *val);