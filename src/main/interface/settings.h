#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pg/pg.h"

#include "interface/settings_generated.h"

typedef struct lookupTableEntry_s {
    const char * const *values;
    const uint8_t valueCount;
} lookupTableEntry_t;

extern const lookupTableEntry_t settingLookupTables[];

#define SETTING_TYPE_OFFSET 0
#define SETTING_SECTION_OFFSET 4
#define SETTING_MODE_OFFSET 6

typedef enum {
    // value type, bits 0-3
    VAR_UINT8 = (0 << SETTING_TYPE_OFFSET),
    VAR_INT8 = (1 << SETTING_TYPE_OFFSET),
    VAR_UINT16 = (2 << SETTING_TYPE_OFFSET),
    VAR_INT16 = (3 << SETTING_TYPE_OFFSET),
    VAR_UINT32 = (4 << SETTING_TYPE_OFFSET),
} setting_type_e;

typedef enum {
    // value section, bits 4-5
    MASTER_VALUE = (0 << SETTING_SECTION_OFFSET),
    PROFILE_VALUE = (1 << SETTING_SECTION_OFFSET),
    CONTROL_RATE_VALUE = (2 << SETTING_SECTION_OFFSET),
} setting_section_e;

typedef enum {
    // value mode, bits 6-7
    MODE_DIRECT = (0 << SETTING_MODE_OFFSET),
    MODE_LOOKUP = (1 << SETTING_MODE_OFFSET),
    MODE_ARRAY  = (2 << SETTING_MODE_OFFSET),
} setting_mode_e;

#define SETTING_TYPE_MASK (0x0F)
#define SETTING_SECTION_MASK (0x30)
#define SETTING_MODE_MASK (0xC0)

typedef struct settingMinMaxConfig_s {
    const uint8_t indexes[SETTING_MIN_MAX_INDEX_BYTES];
} settingMinMaxConfig_t;

typedef struct settingLookupTableConfig_s {
    const uint8_t tableIndex;
} settingLookupTableConfig_t;

typedef struct settingArrayLengthConfig_s {
    const uint8_t length;
} settingArrayLengthConfig_t;

typedef union {
    settingLookupTableConfig_t lookup;
    settingMinMaxConfig_t minmax;
    settingArrayLengthConfig_t array;
} settingConfig_t;

typedef struct {
    const uint8_t encoded_name[SETTING_ENCODED_NAME_MAX_BYTES];
    const uint8_t type; // see settingFlag_e
    const settingConfig_t config;
    const setting_offset_t offset;
} __attribute__((packed)) setting_t;

extern const setting_t settingsTable[];

static inline setting_type_e SETTING_TYPE(const setting_t *s) { return s->type & SETTING_TYPE_MASK; }
static inline setting_section_e SETTING_SECTION(const setting_t *s) { return s->type & SETTING_SECTION_MASK; }
static inline setting_mode_e SETTING_MODE(const setting_t *s) { return s->type & SETTING_MODE_MASK; }

void settingGetName(const setting_t *val, char *buf);
bool settingNameContains(const setting_t *val, char *buf, const char *cmdline);
bool settingNameIsExactMatch(const setting_t *val, char *buf, const char *cmdline, uint8_t var_name_length);
// Returns a setting_t with the exact name (case sensitive), or
// NULL if no setting with that name exists.
const setting_t *settingFind(const char *name);
// Returns the size in bytes of the setting value.
size_t settingGetValueSize(const setting_t *val);
pgn_t settingGetPgNumber(const setting_t *val);
uint16_t settingGetValueOffset(const setting_t *value);
// Returns a pointer to the actual value stored by
// the setting_t. The returned value might be modified.
void * settingGetValuePointer(const setting_t *val);
// Returns a pointer to the backed up copy of the value. Note that
// this will contain random garbage unless a copy of the parameter
// group for the value has been manually performed. Currently, this
// is only used by cli.c during config dumps.
const void * settingGetCopyValuePointer(const setting_t *val);
// Returns the minimum valid value for the given setting_t. setting_min_t
// depends on the target and build options, but will always be a signed
// integer (e.g. intxx_t,)
setting_min_t settingGetMin(const setting_t *val);
// Returns the maximum valid value for the given setting_t. setting_max_t
// depends on the target and build options, but will always be an unsigned
// integer (e.g. uintxx_t,)
setting_max_t settingGetMax(const setting_t *val);
uint8_t settingGetArrayLength(const setting_t *val);
