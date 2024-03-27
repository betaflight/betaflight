/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#ifdef USE_ALTHOLD_MODE
#include "common/time.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#define ALTHOLD_TASK_PERIOD 100         // hz
#define ALTHOLD_ENTER_PERIOD 50         // ms
#define ALTHOLD_MAX_EXIT_PERIOD 3000    // ms


typedef struct altholdConfig_s {
    uint8_t velPidP;
    uint8_t velPidD;

    uint8_t altPidP;
    uint8_t altPidI;

    uint8_t minThrottle;
    uint8_t maxThrottle;

    uint8_t angleLimit;
} altholdConfig_t;

PG_DECLARE(altholdConfig_t, altholdConfig);

typedef struct {
    float max;
    float min;
    float kp;
    float kd;
    float ki;
    float lastErr;
    float integral;
} simplePid_s;

typedef struct {
    simplePid_s altPid;
    simplePid_s velPid;
    float throttle;
    float throttleFactor;
    float targetAltitude;
    float measuredAltitude;
    float measuredAccel;
    float velocityEstimationAccel;  // based on acceleration
    float startVelocityEstimationAccel;
    bool altHoldEnabled;
    uint32_t enterTime;
    uint32_t exitTime;
    float smoothedAltitude;
} altHoldState_s;


void initAltHoldState(void);
void updateAltHoldState(timeUs_t currentTimeUs);
float getAltHoldThrottle(void);
float getAltHoldThrottleFactor(float currentThrottle);

#endif
