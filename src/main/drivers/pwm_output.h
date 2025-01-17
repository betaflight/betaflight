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

#include "platform.h"

#include "common/time.h"

#include "drivers/dma.h"
#include "drivers/io_types.h"
#include "drivers/motor_types.h"
#include "drivers/timer.h"

#include "pg/motor.h"

#define PWM_TIMER_1MHZ        MHZ_TO_HZ(1)

// TODO: move the implementation defintions to impl header (platform)
struct timerHardware_s;

typedef struct {
    volatile timCCR_t *ccr;
    TIM_TypeDef       *tim;
} timerChannel_t;

typedef struct {
    timerChannel_t channel;
    float pulseScale;
    float pulseOffset;
    bool forceOverflow;
    bool enabled;
    IO_t io;
} pwmOutputPort_t;

extern FAST_DATA_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

void motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorDevConfig, uint16_t idlePulse);

typedef struct servoDevConfig_s {
    // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
    uint16_t servoCenterPulse;              // This is the value for servos when they should be in the middle. e.g. 1500.
    uint16_t servoPwmRate;                  // The update rate of servo outputs (50-498Hz)
    ioTag_t  ioTags[MAX_SUPPORTED_SERVOS];
} servoDevConfig_t;

void servoDevInit(const servoDevConfig_t *servoDevConfig);

void pwmServoConfig(const struct timerHardware_s *timerHardware, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse);

void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion);

void pwmWriteServo(uint8_t index, float value);

pwmOutputPort_t *pwmGetMotors(void);
bool pwmIsSynced(void);
void analogInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow);
