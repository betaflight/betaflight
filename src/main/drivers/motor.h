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

void motorPostInit(void);
void motorWriteAll(float *values);
void motorRequestTelemetry(unsigned index);

void motorInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3DHigh, float *deadbandMotor3DLow);

float motorConvertFromExternal(uint16_t externalValue);
uint16_t motorConvertToExternal(float motorValue);

void motorDevInit(unsigned motorCount);
unsigned motorDeviceCount(void);
const motorVTable_t *motorGetVTable(void);
bool checkMotorProtocolEnabled(const motorDevConfig_t *motorConfig, bool *protocolIsDshot);

motorProtocolFamily_e motorGetProtocolFamily(void);

bool isMotorProtocolDshot(void);
bool isMotorProtocolBidirDshot(void);
bool isMotorProtocolEnabled(void);

void motorDisable(void);
void motorEnable(void);
float motorEstimateMaxRpm(void);
bool motorIsEnabled(void);
bool motorIsMotorEnabled(unsigned index);
bool motorIsMotorIdle(unsigned index);
timeMs_t motorGetMotorEnableTimeMs(void);
void motorShutdown(void); // Replaces stopPwmAllMotors
