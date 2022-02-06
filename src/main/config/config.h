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

#include "pg/pg.h"

#define MAX_NAME_LENGTH 16u

typedef enum {
    CONFIGURATION_STATE_DEFAULTS_BARE = 0,
    CONFIGURATION_STATE_DEFAULTS_CUSTOM,
    CONFIGURATION_STATE_CONFIGURED,
} configurationState_e;

typedef struct pilotConfig_s {
    char craftName[MAX_NAME_LENGTH + 1];
    char pilotName[MAX_NAME_LENGTH + 1];
} pilotConfig_t;

PG_DECLARE(pilotConfig_t, pilotConfig);

typedef struct systemConfig_s {
    uint8_t pidProfileIndex;
    uint8_t activeRateProfile;
    uint8_t debug_mode;
    uint8_t task_statistics;
    uint8_t rateProfile6PosSwitch;
    uint8_t cpu_overclock;
    uint8_t powerOnArmingGraceTime; // in seconds
    char boardIdentifier[sizeof(TARGET_BOARD_IDENTIFIER) + 1];
    uint8_t hseMhz;                 // Only used for F4 and G4 targets
    uint8_t configurationState;     // The state of the configuration (defaults / configured)
    uint8_t enableStickArming; // boolean that determines whether stick arming can be used
} systemConfig_t;

PG_DECLARE(systemConfig_t, systemConfig);

struct pidProfile_s;
extern struct pidProfile_s *currentPidProfile;

void initEEPROM(void);
bool resetEEPROM(bool useCustomDefaults);
bool readEEPROM(void);
void writeEEPROM(void);
void writeUnmodifiedConfigToEEPROM(void);
void ensureEEPROMStructureIsValid(void);

void saveConfigAndNotify(void);
void validateAndFixGyroConfig(void);

void setConfigDirty(void);
bool isConfigDirty(void);

uint8_t getCurrentPidProfileIndex(void);
void changePidProfile(uint8_t pidProfileIndex);
void changePidProfileFromCellCount(uint8_t cellCount);

uint8_t getCurrentControlRateProfileIndex(void);
void changeControlRateProfile(uint8_t profileIndex);

bool canSoftwareSerialBeUsed(void);

uint16_t getCurrentMinthrottle(void);

void resetConfig(void);
void targetConfiguration(void);
void targetValidateConfiguration(void);

bool isSystemConfigured(void);
void setRebootRequired(void);
bool getRebootRequired(void);
bool isEepromWriteInProgress(void);
