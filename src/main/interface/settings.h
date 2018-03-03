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
#include "pg/pg.h"


typedef enum {
    TABLE_OFF_ON = 0,
    TABLE_UNIT,
    TABLE_ALIGNMENT,
#ifdef USE_GPS
    TABLE_GPS_PROVIDER,
    TABLE_GPS_SBAS_MODE,
#endif
#ifdef USE_BLACKBOX
    TABLE_BLACKBOX_DEVICE,
    TABLE_BLACKBOX_MODE,
#endif
    TABLE_CURRENT_METER,
    TABLE_VOLTAGE_METER,
#ifdef USE_SERVOS
    TABLE_GIMBAL_MODE,
#endif
#ifdef USE_SERIAL_RX
    TABLE_SERIAL_RX,
#endif
#ifdef USE_RX_SPI
    TABLE_RX_SPI,
#endif
    TABLE_GYRO_LPF,
    TABLE_ACC_HARDWARE,
#ifdef USE_BARO
    TABLE_BARO_HARDWARE,
#endif
#ifdef USE_MAG
    TABLE_MAG_HARDWARE,
#endif
    TABLE_DEBUG,
    TABLE_MOTOR_PWM_PROTOCOL,
    TABLE_RC_INTERPOLATION,
    TABLE_RC_INTERPOLATION_CHANNELS,
    TABLE_LOWPASS_TYPE,
    TABLE_FAILSAFE,
    TABLE_CRASH_RECOVERY,
#ifdef USE_CAMERA_CONTROL
    TABLE_CAMERA_CONTROL_MODE,
#endif
    TABLE_BUS_TYPE,
#ifdef USE_MAX7456
    TABLE_MAX7456_CLOCK,
#endif
#ifdef USE_RANGEFINDER
    TABLE_RANGEFINDER_HARDWARE,
#endif
#ifdef USE_GYRO_OVERFLOW_CHECK
    TABLE_GYRO_OVERFLOW_CHECK,
#endif
    TABLE_RATES_TYPE,
#ifdef USE_OVERCLOCK
    TABLE_OVERCLOCK,
#endif
#ifdef USE_DUAL_GYRO
    TABLE_GYRO,
#endif
    LOOKUP_TABLE_COUNT
} lookupTableIndex_e;

typedef struct lookupTableEntry_s {
    const char * const *values;
    const uint8_t valueCount;
} lookupTableEntry_t;


#define VALUE_TYPE_OFFSET 0
#define VALUE_SECTION_OFFSET 2
#define VALUE_MODE_OFFSET 4

typedef enum {
    // value type, bits 0-1
    VAR_UINT8 = (0 << VALUE_TYPE_OFFSET),
    VAR_INT8 = (1 << VALUE_TYPE_OFFSET),
    VAR_UINT16 = (2 << VALUE_TYPE_OFFSET),
    VAR_INT16 = (3 << VALUE_TYPE_OFFSET),

    // value section, bits 2-3
    MASTER_VALUE = (0 << VALUE_SECTION_OFFSET),
    PROFILE_VALUE = (1 << VALUE_SECTION_OFFSET),
    PROFILE_RATE_VALUE = (2 << VALUE_SECTION_OFFSET),

    // value mode, bits 4-5
    MODE_DIRECT = (0 << VALUE_MODE_OFFSET),
    MODE_LOOKUP = (1 << VALUE_MODE_OFFSET),
    MODE_ARRAY = (2 << VALUE_MODE_OFFSET)
} cliValueFlag_e;


#define VALUE_TYPE_MASK (0x03)
#define VALUE_SECTION_MASK (0x0c)
#define VALUE_MODE_MASK (0x30)

typedef struct cliMinMaxConfig_s {
    const int16_t min;
    const int16_t max;
} cliMinMaxConfig_t;

typedef struct cliLookupTableConfig_s {
    const lookupTableIndex_e tableIndex;
} cliLookupTableConfig_t;

typedef struct cliArrayLengthConfig_s {
    const uint8_t length;
} cliArrayLengthConfig_t;

typedef union {
    cliLookupTableConfig_t lookup;
    cliMinMaxConfig_t minmax;
    cliArrayLengthConfig_t array;
} cliValueConfig_t;

typedef struct clivalue_s {
    const char *name;
    const uint8_t type; // see cliValueFlag_e
    const cliValueConfig_t config;

    pgn_t pgn;
    uint16_t offset;
} __attribute__((packed)) clivalue_t;


extern const lookupTableEntry_t lookupTables[];
extern const uint16_t valueTableEntryCount;

extern const clivalue_t valueTable[];
//extern const uint8_t lookupTablesEntryCount;

extern const char * const lookupTableGyroHardware[];

extern const char * const lookupTableAccHardware[];
//extern const uint8_t lookupTableAccHardwareEntryCount;

extern const char * const lookupTableBaroHardware[];
//extern const uint8_t lookupTableBaroHardwareEntryCount;

extern const char * const lookupTableMagHardware[];
//extern const uint8_t lookupTableMagHardwareEntryCount;

extern const char * const lookupTableRangefinderHardware[];
