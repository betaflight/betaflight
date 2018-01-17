#include <string.h>
#include <stdint.h>

#include "common/utils.h"

#include "interface/settings_generated.h"
#include "interface/settings.h"

#include "interface/settings_generated.inc"

void settingGetName(const setting_t *val, char *buf)
{
    unsigned bpos = 0;
    unsigned n = 0;
#ifndef SETTING_ENCODED_NAME_USES_BYTE_INDEXING
    unsigned shift = 0;
#endif
    for (int ii = 0; ii < SETTING_ENCODED_NAME_MAX_BYTES; ii++) {
#ifdef SETTING_ENCODED_NAME_USES_BYTE_INDEXING
        n = val->encoded_name[ii];
#else
        // Decode a variable size uint
        unsigned b = val->encoded_name[ii];
        if (b >= 0x80) {
            // More bytes follow
            n |= (b&0x7f) << shift;
            shift += 7;
            continue;
        }
        // Final byte
        n |= b << shift;
#endif
        const char *word = settingNamesWords[n];
        if (!word) {
            // No more words
            break;
        }
        if (bpos > 0) {
            // Word separator
            buf[bpos++] = '_';
          }
        strcpy(&buf[bpos], word);
        bpos += strlen(word);
#ifndef SETTING_ENCODED_NAME_USES_BYTE_INDEXING
        // Reset shift and n
        shift = 0;
        n = 0;
#endif
    }
    buf[bpos] = '\0';
}

bool settingNameContains(const setting_t *val, char *buf, const char *cmdline)
{
    settingGetName(val, buf);
    return strstr(buf, cmdline) != NULL;
}

bool settingNameIsExactMatch(const setting_t *val, char *buf, const char *cmdline, uint8_t var_name_length)
{
    settingGetName(val, buf);
    return strncasecmp(cmdline, buf, strlen(buf)) == 0 && var_name_length == strlen(buf);
}

const setting_t *settingFind(const char *name)
{
    char buf[SETTING_MAX_NAME_LENGTH];
    for (int ii = 0; ii < SETTINGS_TABLE_COUNT; ii++) {
        const setting_t *setting = &settingsTable[ii];
        settingGetName(setting, buf);
        if (strcmp(buf, name) == 0) {
            return setting;
        }
    }
    return NULL;
}

size_t settingGetValueSize(const setting_t *val)
{
    switch (SETTING_TYPE(val)) {
    case VAR_UINT8:
    case VAR_INT8:
        return 1;
    case VAR_UINT16:
    case VAR_INT16:
        return 2;
    case VAR_UINT32:
        return 4;
    }
    return 0; // Unreachable
}

pgn_t settingGetPgNumber(const setting_t *val)
{
    unsigned pos = val - (const setting_t *)settingsTable;
    unsigned acc = 0;
    for (unsigned ii = 0; ii < SETTINGS_PGN_COUNT; ii++) {
        acc += settingsPgnCounts[ii];
        if (acc > pos) {
            return settingsPgn[ii];
        }
    }
    return -1;
}

uint16_t settingGetValueOffset(const setting_t *value)
{
    switch (SETTING_SECTION(value)) {
    case MASTER_VALUE:
        return value->offset;
    case PROFILE_VALUE:
        return value->offset + sizeof(pidProfile_t) * getCurrentPidProfileIndex();
    case CONTROL_RATE_VALUE:
        return value->offset + sizeof(controlRateConfig_t) * getCurrentControlRateProfileIndex();
    }
    return 0;
}

void *settingGetValuePointer(const setting_t *val)
{
    const pgRegistry_t *pg = pgFind(settingGetPgNumber(val));
    return pg->address + settingGetValueOffset(val);
}

const void * settingGetCopyValuePointer(const setting_t *val)
{
    const pgRegistry_t *pg = pgFind(settingGetPgNumber(val));
    return pg->copy + settingGetValueOffset(val);
}

setting_min_t settingGetMin(const setting_t *val)
{
    if (SETTING_MODE(val) == MODE_DIRECT) {
        return 0;
    }
    return settingMinMaxTable[SETTING_INDEXES_GET_MIN(val)];
}

setting_max_t settingGetMax(const setting_t *val)
{
    if (SETTING_MODE(val) == MODE_LOOKUP) {
        return settingLookupTables[val->config.lookup.tableIndex].valueCount - 1;
    }
    return settingMinMaxTable[SETTING_INDEXES_GET_MAX(val)];
}

uint8_t settingGetArrayLength(const setting_t *val)
{
    if(SETTING_MODE(val) != MODE_ARRAY) {
        return 0;
    }
    return val->config.array.length;
}
