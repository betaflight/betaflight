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
#include "common/time.h"
#include "config/parameter_group.h"
#include "drivers/adc.h"
#include "drivers/rx_pwm.h"
#include "fc/stats.h"

#define MAX_PROFILE_COUNT 3
#define ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS 1500
#define MAX_NAME_LENGTH 16

#define ACC_TASK_FREQUENCY_DEFAULT 500
#define ACC_TASK_FREQUENCY_MIN 100
#define ACC_TASK_FREQUENCY_MAX 1000
#define ATTITUDE_TASK_FREQUENCY_DEFAULT 250
#define ATTITUDE_TASK_FREQUENCY_MIN 100
#define ATTITUDE_TASK_FREQUENCY_MAX 1000

typedef enum {
    ASYNC_MODE_NONE,
    ASYNC_MODE_GYRO,
    ASYNC_MODE_ALL
} asyncMode_e;

typedef enum {
    FEATURE_RX_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_UNUSED_1 = 1 << 2,          // Unused in INAV
    FEATURE_RX_SERIAL = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_GPS = 1 << 7,
    FEATURE_UNUSED_3 = 1 << 8,          // was FEATURE_FAILSAFE
    FEATURE_UNUSED_4 = 1 << 9,          // was FEATURE_SONAR
    FEATURE_TELEMETRY = 1 << 10,
    FEATURE_CURRENT_METER = 1 << 11,
    FEATURE_3D = 1 << 12,
    FEATURE_RX_PARALLEL_PWM = 1 << 13,
    FEATURE_RX_MSP = 1 << 14,
    FEATURE_RSSI_ADC = 1 << 15,
    FEATURE_LED_STRIP = 1 << 16,
    FEATURE_DASHBOARD = 1 << 17,
    FEATURE_UNUSED_2 = 1 << 18,         // Unused in INAV
    FEATURE_BLACKBOX = 1 << 19,
    FEATURE_CHANNEL_FORWARDING = 1 << 20,
    FEATURE_TRANSPONDER = 1 << 21,
    FEATURE_AIRMODE = 1 << 22,
    FEATURE_SUPEREXPO_RATES = 1 << 23,
    FEATURE_VTX = 1 << 24,
    FEATURE_RX_SPI = 1 << 25,
    FEATURE_SOFTSPI = 1 << 26,
    FEATURE_PWM_SERVO_DRIVER = 1 << 27,
    FEATURE_PWM_OUTPUT_ENABLE = 1 << 28,
    FEATURE_OSD = 1 << 29,
} features_e;

typedef struct systemConfig_s {
    uint16_t accTaskFrequency;
    uint16_t attitudeTaskFrequency;
    uint8_t current_profile_index;
    uint8_t asyncMode;
    uint8_t debug_mode;
    uint8_t i2c_speed;
    uint8_t cpuUnderclock;
    uint8_t throttle_tilt_compensation_strength;      // the correction that will be applied at throttle_correction_angle.
    inputFilteringMode_e pwmRxInputFilteringMode;
    char name[MAX_NAME_LENGTH + 1];
} systemConfig_t;

PG_DECLARE(systemConfig_t, systemConfig);

typedef struct beeperConfig_s {
    uint32_t beeper_off_flags;
    uint32_t preferred_beeper_off_flags;
} beeperConfig_t;

PG_DECLARE(beeperConfig_t, beeperConfig);

typedef struct adcChannelConfig_s {
    uint8_t adcFunctionChannel[ADC_FUNCTION_COUNT];
} adcChannelConfig_t;

PG_DECLARE(adcChannelConfig_t, adcChannelConfig);


#ifdef STATS
PG_DECLARE(statsConfig_t, statsConfig);
#endif

void beeperOffSet(uint32_t mask);
void beeperOffSetAll(uint8_t beeperCount);
void beeperOffClear(uint32_t mask);
void beeperOffClearAll(void);
uint32_t getBeeperOffMask(void);
void setBeeperOffMask(uint32_t mask);
uint32_t getPreferredBeeperOffMask(void);
void setPreferredBeeperOffMask(uint32_t mask);

void copyCurrentProfileToProfileSlot(uint8_t profileSlotIndex);

void initEEPROM(void);
void resetEEPROM(void);
void readEEPROM(void);
void writeEEPROM();
void ensureEEPROMContainsValidData(void);

void saveConfigAndNotify(void);
void validateAndFixConfig(void);

uint8_t getConfigProfile(void);
bool setConfigProfile(uint8_t profileIndex);
void setConfigProfileAndWriteEEPROM(uint8_t profileIndex);

bool canSoftwareSerialBeUsed(void);
void applyAndSaveBoardAlignmentDelta(int16_t roll, int16_t pitch);

void createDefaultConfig(void);
void resetConfigs(void);
void targetConfiguration(void);

uint32_t getPidUpdateRate(void);
timeDelta_t getGyroUpdateRate(void);
uint16_t getAccUpdateRate(void);
#ifdef ASYNC_GYRO_PROCESSING
uint16_t getAttitudeUpdateRate(void);
uint8_t getAsyncMode(void);
#endif
