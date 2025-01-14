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
 *
 * Author: jflyper
 */

#pragma once

#include "common/time.h"

#include "pg/motor.h"

#include "drivers/motor_types.h"

void motorPostInitNull();
void motorWriteNull(uint8_t index, float value);
bool motorDecodeTelemetryNull(void);
void motorUpdateCompleteNull(void);

void motorPostInit();
void motorWriteAll(float *values);
void motorRequestTelemetry(uint8_t index);

void motorInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3DHigh, float *deadbandMotor3DLow);

float motorConvertFromExternal(uint16_t externalValue);
uint16_t motorConvertToExternal(float motorValue);

struct motorDevConfig_s; // XXX Shouldn't be needed once pwm_output* is really cleaned up.
void motorDevInit(const struct motorDevConfig_s *motorConfig, uint16_t idlePulse, uint8_t motorCount);
unsigned motorDeviceCount(void);
motorVTable_t *motorGetVTable(void);
bool checkMotorProtocolEnabled(const motorDevConfig_t *motorConfig, bool *protocolIsDshot);
bool isMotorProtocolDshot(void);
bool isMotorProtocolBidirDshot(void);
bool isMotorProtocolEnabled(void);

void motorDisable(void);
void motorEnable(void);
float motorEstimateMaxRpm(void);
bool motorIsEnabled(void);
bool motorIsMotorEnabled(uint8_t index);
bool motorIsMotorIdle(uint8_t index);
timeMs_t motorGetMotorEnableTimeMs(void);
void motorShutdown(void); // Replaces stopPwmAllMotors

#ifdef USE_DSHOT_BITBANG
struct motorDevConfig_s;
typedef struct motorDevConfig_s motorDevConfig_t;
bool isDshotBitbangActive(const motorDevConfig_t *motorConfig);
#endif
