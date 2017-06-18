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

#include "config/parameter_group.h"

#include "drivers/adc.h"
#include "drivers/flash.h"
#include "drivers/rx_pwm.h"
#include "drivers/sdcard.h"
#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/sound_beeper.h"
#include "drivers/vcd.h"

typedef enum {
    FEATURE_RX_PPM = 1 << 0,
    //FEATURE_VBAT = 1 << 1,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_RX_SERIAL = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_GPS = 1 << 7,
    FEATURE_FAILSAFE = 1 << 8,
    FEATURE_SONAR = 1 << 9,
    FEATURE_TELEMETRY = 1 << 10,
    //FEATURE_CURRENT_METER = 1 << 11,
    FEATURE_3D = 1 << 12,
    FEATURE_RX_PARALLEL_PWM = 1 << 13,
    FEATURE_RX_MSP = 1 << 14,
    FEATURE_RSSI_ADC = 1 << 15,
    FEATURE_LED_STRIP = 1 << 16,
    FEATURE_DASHBOARD = 1 << 17,
    FEATURE_OSD = 1 << 18,
    FEATURE_BLACKBOX_UNUSED = 1 << 19,
    FEATURE_CHANNEL_FORWARDING = 1 << 20,
    FEATURE_TRANSPONDER = 1 << 21,
    FEATURE_AIRMODE = 1 << 22,
    FEATURE_SDCARD_UNUSED = 1 << 23,
    //FEATURE_VTX = 1 << 24,
    FEATURE_RX_SPI = 1 << 25,
    FEATURE_SOFTSPI = 1 << 26,
    FEATURE_ESC_SENSOR = 1 << 27,
    FEATURE_ANTI_GRAVITY = 1 << 28,
    FEATURE_DYNAMIC_FILTER = 1 << 29,
} features_e;

#define MAX_NAME_LENGTH 16

#ifndef USE_OSD_SLAVE
typedef struct systemConfig_s {
    uint8_t pidProfileIndex;
    uint8_t activeRateProfile;
    uint8_t debug_mode;
    uint8_t task_statistics;
    char name[MAX_NAME_LENGTH + 1]; // FIXME misplaced, see PG_PILOT_CONFIG in CF v1.x
} systemConfig_t;
#endif

#ifdef USE_OSD_SLAVE
typedef struct systemConfig_s {
    uint8_t debug_mode;
    uint8_t task_statistics;
} systemConfig_t;
#endif

PG_DECLARE(systemConfig_t, systemConfig);
PG_DECLARE(adcConfig_t, adcConfig);
PG_DECLARE(beeperDevConfig_t, beeperDevConfig);
PG_DECLARE(flashConfig_t, flashConfig);
PG_DECLARE(ppmConfig_t, ppmConfig);
PG_DECLARE(pwmConfig_t, pwmConfig);
PG_DECLARE(vcdProfile_t, vcdProfile);
PG_DECLARE(sdcardConfig_t, sdcardConfig);
PG_DECLARE(serialPinConfig_t, serialPinConfig);

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
void writeEEPROM();
void ensureEEPROMContainsValidData(void);

void saveConfigAndNotify(void);
void validateAndFixConfig(void);
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
