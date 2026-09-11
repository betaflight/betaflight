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

// Shared state and helpers for the DShot motor output path: per-timer and
// per-motor DMA resources used by the DPWM and DShot output drivers.

#pragma once

#ifdef USE_DSHOT

#include "drivers/dshot.h"
#include "dshot_dpwm.h"

extern FAST_DATA_ZERO_INIT uint8_t dmaMotorTimerCount;

extern FAST_DATA_ZERO_INIT motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
extern FAST_DATA_ZERO_INIT motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
extern FAST_DATA_ZERO_INIT uint32_t dshotTelemetryBitWidth[MAX_SUPPORTED_MOTORS];

uint8_t getTimerIndex(void *timer);
motorDmaOutput_t *getMotorDmaOutput(unsigned index);
void dshotEnableChannels(unsigned motorCount);
void pwmDshotSetDirectionOutput(motorDmaOutput_t *const motor);
void pwmDshotReleaseAllMotors(void);
void pwmDshotRequestTelemetry(unsigned index);
bool pwmDshotIsMotorIdle(unsigned index);
bool pwmTelemetryDecode(void);

#endif
