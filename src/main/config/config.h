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

#define MAX_PROFILE_COUNT 3
#define MAX_CONTROL_RATE_PROFILE_COUNT 3
#define ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS 1500
#define MAX_NAME_LENGTH 16

#define ACC_TASK_FREQUENCY_DEFAULT 500
#define ACC_TASK_FREQUENCY_MIN 100
#define ACC_TASK_FREQUENCY_MAX 1000
#define ATTITUDE_TASK_FREQUENCY_DEFAULT 250
#define ATTITUDE_TASK_FREQUENCY_MIN 100
#define ATTITUDE_TASK_FREQUENCY_MAX 1000

#ifdef ASYNC_GYRO_PROCESSING
typedef enum {
    ASYNC_MODE_NONE,
    ASYNC_MODE_GYRO,
    ASYNC_MODE_ALL
} asyncMode_e;
#endif

typedef enum {
    FEATURE_RX_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_UNUSED_1 = 1 << 2,          // Unused in INAV
    FEATURE_RX_SERIAL = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_GPS = 1 << 7,
    FEATURE_FAILSAFE = 1 << 8,
    FEATURE_SONAR = 1 << 9,
    FEATURE_TELEMETRY = 1 << 10,
    FEATURE_CURRENT_METER = 1 << 11,
    FEATURE_3D = 1 << 12,
    FEATURE_RX_PARALLEL_PWM = 1 << 13,
    FEATURE_RX_MSP = 1 << 14,
    FEATURE_RSSI_ADC = 1 << 15,
    FEATURE_LED_STRIP = 1 << 16,
    FEATURE_DASHBOARD= 1 << 17,
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

void beeperOffSet(uint32_t mask);
void beeperOffSetAll(uint8_t beeperCount);
void beeperOffClear(uint32_t mask);
void beeperOffClearAll(void);
uint32_t getBeeperOffMask(void);
void setBeeperOffMask(uint32_t mask);
uint32_t getPreferredBeeperOffMask(void);
void setPreferredBeeperOffMask(uint32_t mask);

void copyCurrentProfileToProfileSlot(uint8_t profileSlotIndex);

void resetEEPROM(void);
void readEEPROMAndNotify(void);
void ensureEEPROMContainsValidData(void);

void saveConfigAndNotify(void);
void validateAndFixConfig(void);
void activateConfig(void);

uint8_t getCurrentProfile(void);
void changeProfile(uint8_t profileIndex);

void setProfile(uint8_t profileIndex);
void setControlRateProfile(uint8_t profileIndex);
struct pidProfile_s;
void resetPidProfile(struct pidProfile_s *pidProfile);

uint8_t getCurrentControlRateProfile(void);
void changeControlRateProfile(uint8_t profileIndex);
bool canSoftwareSerialBeUsed(void);
void applyAndSaveBoardAlignmentDelta(int16_t roll, int16_t pitch);

uint16_t getCurrentMinthrottle(void);
struct master_s;
void targetConfiguration(struct master_s *config);

#ifdef ASYNC_GYRO_PROCESSING
uint32_t getPidUpdateRate(void);
uint32_t getGyroUpdateRate(void);
uint16_t getAccUpdateRate(void);
uint16_t getAttitudeUpdateRate(void);
uint8_t getAsyncMode(void);
#endif
