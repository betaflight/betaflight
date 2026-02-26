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

#include <stdbool.h>
#include <stdint.h>

#ifdef USE_HOVER_CALIBRATION

typedef enum {
    HOVER_CAL_STATUS_IDLE = 0,
    HOVER_CAL_STATUS_WAITING_STABLE,
    HOVER_CAL_STATUS_SAMPLING,
    HOVER_CAL_STATUS_COMPLETE,
    HOVER_CAL_STATUS_FAILED
} hoverCalibrationStatus_e;

typedef enum {
    HOVER_CAL_FAIL_NONE = 0,
    HOVER_CAL_FAIL_DISARMED,
    HOVER_CAL_FAIL_NO_ALTITUDE,
    HOVER_CAL_FAIL_TOO_LOW,
    HOVER_CAL_FAIL_NOT_LEVEL,
    HOVER_CAL_FAIL_MOVING,
    HOVER_CAL_FAIL_ALTHOLD_MODE,
    HOVER_CAL_FAIL_RESULT_RANGE
} hoverCalibrationFailReason_e;

void hoverCalibrationInit(void);
void hoverCalibrationUpdate(void);
void hoverCalibrationStart(void);
void hoverCalibrationAbort(void);

bool isHoverCalibrationActive(void);
hoverCalibrationStatus_e getHoverCalibrationStatus(void);
hoverCalibrationFailReason_e getHoverCalibrationFailReason(void);
uint8_t getHoverCalibrationProgress(void);  // 0-100%
uint16_t getHoverCalibrationResult(void);   // Last calibrated value

#endif // USE_HOVER_CALIBRATION
