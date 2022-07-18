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
#include "build/build_config.h"
#include "common/time.h"
#include "pg/pg.h"

typedef enum BlackboxDevice {
    BLACKBOX_DEVICE_NONE = 0,
    BLACKBOX_DEVICE_FLASH = 1,
    BLACKBOX_DEVICE_SDCARD = 2,
    BLACKBOX_DEVICE_SERIAL = 3
} BlackboxDevice_e;

typedef enum BlackboxMode {
    BLACKBOX_MODE_NORMAL = 0,
    BLACKBOX_MODE_MOTOR_TEST,
    BLACKBOX_MODE_ALWAYS_ON
} BlackboxMode;

typedef enum BlackboxSampleRate { // Sample rate is 1/(2^BlackboxSampleRate)
    BLACKBOX_RATE_ONE = 0,
    BLACKBOX_RATE_HALF,
    BLACKBOX_RATE_QUARTER,
    BLACKBOX_RATE_8TH,
    BLACKBOX_RATE_16TH
} BlackboxSampleRate_e;

typedef enum FlightLogEvent {
    FLIGHT_LOG_EVENT_SYNC_BEEP = 0,
    FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_START = 10,   // UNUSED
    FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_RESULT = 11,  // UNUSED
    FLIGHT_LOG_EVENT_AUTOTUNE_TARGETS = 12,       // UNUSED
    FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT = 13,
    FLIGHT_LOG_EVENT_LOGGING_RESUME = 14,
    FLIGHT_LOG_EVENT_DISARM = 15,
    FLIGHT_LOG_EVENT_FLIGHTMODE = 30, // Add new event type for flight mode status.
    FLIGHT_LOG_EVENT_LOG_END = 255
} FlightLogEvent;

typedef struct blackboxConfig_s {
    uint8_t sample_rate; // sample rate
    uint8_t device;
    uint32_t fields_disabled_mask;
    uint8_t mode;
} blackboxConfig_t;

PG_DECLARE(blackboxConfig_t, blackboxConfig);

union flightLogEventData_u;
void blackboxLogEvent(FlightLogEvent event, union flightLogEventData_u *data);

void blackboxInit(void);
void blackboxUpdate(timeUs_t currentTimeUs);
void blackboxSetStartDateTime(const char *dateTime, timeMs_t timeNowMs);
int blackboxCalculatePDenom(int rateNum, int rateDenom);
uint8_t blackboxGetRateDenom(void);
uint16_t blackboxGetPRatio(void);
uint8_t blackboxCalculateSampleRate(uint16_t pRatio);
void blackboxValidateConfig(void);
void blackboxFinish(void);
bool blackboxMayEditConfig(void);
#ifdef UNIT_TEST
STATIC_UNIT_TESTED void blackboxLogIteration(timeUs_t currentTimeUs);
STATIC_UNIT_TESTED bool blackboxShouldLogPFrame(void);
STATIC_UNIT_TESTED bool blackboxShouldLogIFrame(void);
STATIC_UNIT_TESTED bool blackboxShouldLogGpsHomeFrame(void);
STATIC_UNIT_TESTED bool writeSlowFrameIfNeeded(void);
// Called once every FC loop in order to keep track of how many FC loop iterations have passed
STATIC_UNIT_TESTED void blackboxAdvanceIterationTimers(void);
extern int32_t blackboxSInterval;
extern int32_t blackboxSlowFrameIterationTimer;
#endif
