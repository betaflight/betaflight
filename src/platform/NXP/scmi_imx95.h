/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// SCMI status codes, plus SCMI_TIMEOUT for the transport.
#define SCMI_SUCCESS            0
#define SCMI_NOT_SUPPORTED      (-1)
#define SCMI_INVALID_PARAMETERS (-2)
#define SCMI_DENIED             (-3)
#define SCMI_NOT_FOUND          (-4)
// Also the answer to a call from an exception handler, which is refused.
#define SCMI_BUSY               (-6)
#define SCMI_TIMEOUT            (-64)
// No posted sensor read to collect: none was posted, or a later call took its reply.
#define SCMI_NOTHING_PENDING    (-65)

// SCMI clock ids are the CCM root index plus this offset.
#define SCMI_CLOCK_ROOT(root)   ((root) + 41)

// Calls made since reset and their total time in getCycleCounter() units, for a debugger to read.
extern uint32_t scmiCallCount;
extern uint32_t scmiCallCycles;
// Posted sensor reads: time to post, replies collected and time to collect them.
extern uint32_t scmiSensorPostCycles;
extern uint32_t scmiSensorReplyCount;
extern uint32_t scmiSensorReplyCycles;

// Mux mode bit that keeps the input path on for a pad driven by a peripheral.
#define SCMI_PIN_SION           0x10

// Resets this core's logical machine (SYSTEM_POWER_STATE_SET, warm reset); other cores keep running.
int32_t scmiSystemReset(void);
int32_t scmiClockEnable(uint32_t clockId);
int32_t scmiClockRateGet(uint32_t clockId, uint64_t *rateHz);
int32_t scmiClockRateSet(uint32_t clockId, uint64_t rateHz);
int32_t scmiClockParentSet(uint32_t clockId, uint32_t parentId);
int32_t scmiClockParentGet(uint32_t clockId, uint32_t *parentId);

// Silicon name and revision from MISC_SI_INFO, such as "i.MX95 B0"; name is left alone on failure.
#define SCMI_SI_NAME_LENGTH     16
int32_t scmiSiNameGet(char name[SCMI_SI_NAME_LENGTH + 1]);

// A sensor answers SENSOR_READING_GET only after it is enabled.
int32_t scmiSensorEnable(uint32_t sensorId);
// SENSOR_READING_GET without waiting: post it, poll busy, then collect the reply; value is left alone on failure.
// A posted read gives up after half a second; a synchronous call in between takes its reply.
int32_t scmiSensorReadStart(uint32_t sensorId);
bool scmiSensorReadBusy(void);
int32_t scmiSensorReadResult(int64_t *value);

int32_t scmiPadConfigure(uint32_t configRegister, uint32_t configValue);

// Arguments one to five are an IOMUXC_PAD_<pad>__<signal> tuple from fsl_iomuxc.h.
int32_t scmiPinConfigure(uint32_t muxRegister, uint32_t muxMode, uint32_t inputRegister,
                         uint32_t inputDaisy, uint32_t configRegister, uint32_t configValue);
