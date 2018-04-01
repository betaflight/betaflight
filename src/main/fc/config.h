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

#define MAX_NAME_LENGTH 16u

typedef struct pilotConfig_s {
    char name[MAX_NAME_LENGTH + 1];
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
} systemConfig_t;

PG_DECLARE(systemConfig_t, systemConfig);

struct pidProfile_s;
extern struct pidProfile_s *currentPidProfile;

void beeperOffSet(uint32_t mask);
void beeperOffSetAll(uint8_t beeperCount);
void beeperOffClear(uint32_t mask);
void beeperOffClearAll(void);
uint32_t getBeeperOffMask(void);
void setBeeperOffMask(uint32_t mask);
uint32_t getPreferredBeeperOffMask(void);
void setPreferredBeeperOffMask(uint32_t mask);

void initEEPROM(void);
void resetEEPROM(void);
void readEEPROM(void);
void writeEEPROM(void);
void ensureEEPROMContainsValidData(void);

void saveConfigAndNotify(void);
void validateAndFixGyroConfig(void);
void activateConfig(void);

uint8_t getCurrentPidProfileIndex(void);
void changePidProfile(uint8_t pidProfileIndex);
struct pidProfile_s;
void resetPidProfile(struct pidProfile_s *profile);

uint8_t getCurrentControlRateProfileIndex(void);
void changeControlRateProfile(uint8_t profileIndex);

bool canSoftwareSerialBeUsed(void);

uint16_t getCurrentMinthrottle(void);

void resetConfigs(void);
void targetConfiguration(void);
void targetValidateConfiguration(void);
