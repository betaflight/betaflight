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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/logging.h"

#ifdef BOOTLOG

#include "drivers/time.h"

static bootLogEntry_t   events[MAX_BOOTLOG_ENTRIES];
static int              eventCount;
static bool             eventOverflow;

#ifdef BOOTLOG_DESCRIPTIONS
static const char *     eventDescription[BOOT_EVENT_CODE_COUNT] = {
    [BOOT_EVENT_CONFIG_LOADED]              = "CONFIG_LOADED",
    [BOOT_EVENT_SYSTEM_INIT_DONE]           = "SYSTEM_INIT_DONE",
    [BOOT_EVENT_PWM_INIT_DONE]              = "PWM_INIT_DONE",
    [BOOT_EVENT_EXTRA_BOOT_DELAY]           = "EXTRA_BOOT_DELAY",
    [BOOT_EVENT_SENSOR_INIT_DONE]           = "SENSOR_INIT_DONE",
    [BOOT_EVENT_GPS_INIT_DONE]              = "GPS_INIT_DONE",
    [BOOT_EVENT_LEDSTRIP_INIT_DONE]         = "LEDSTRIP_INIT_DONE",
    [BOOT_EVENT_TELEMETRY_INIT_DONE]        = "TELEMETRY_INIT_DONE",
    [BOOT_EVENT_SYSTEM_READY]               = "SYSTEM_READY",
    [BOOT_EVENT_GYRO_DETECTION]             = "GYRO_DETECTION",
    [BOOT_EVENT_ACC_DETECTION]              = "ACC_DETECTION",
    [BOOT_EVENT_BARO_DETECTION]             = "BARO_DETECTION",
    [BOOT_EVENT_MAG_DETECTION]              = "MAG_DETECTION",
    [BOOT_EVENT_RANGEFINDER_DETECTION]      = "RANGEFINDER_DETECTION",
    [BOOT_EVENT_MAG_INIT_FAILED]            = "MAG_INIT_FAILED",
    [BOOT_EVENT_HMC5883L_READ_OK_COUNT]     = "HMC5883L_READ_OK_COUNT",
    [BOOT_EVENT_HMC5883L_READ_FAILED]       = "HMC5883L_READ_FAILED",
    [BOOT_EVENT_HMC5883L_SATURATION]        = "HMC5883L_SATURATION",
    [BOOT_EVENT_TIMER_CH_SKIPPED]           = "TIMER_CHANNEL_SKIPPED",
    [BOOT_EVENT_TIMER_CH_MAPPED]            = "TIMER_CHANNEL_MAPPED",
    [BOOT_EVENT_PITOT_DETECTION]            = "PITOT_DETECTION",
    [BOOT_EVENT_HARDWARE_IO_CONFLICT]       = "HARDWARE_CONFLICT",
};

const char * getBootlogEventDescription(bootLogEventCode_e eventCode)
{
    if (eventCode < BOOT_EVENT_CODE_COUNT) {
        return eventDescription[eventCode];
    }
    else {
        return NULL;
    }
}
#endif

void initBootlog(void)
{
    memset(events, 0, sizeof(events));
    eventCount = 0;
    eventOverflow = false;
}

int getBootlogEventCount(void)
{
    return eventCount;
}

bootLogEntry_t * getBootlogEvent(int index)
{
    if (index <= eventCount) {
        return &events[index];
    }
    else {
        return 0;
    }
}

static void addBootlogEntry(bootLogEntry_t * event)
{
    if (eventCount >= MAX_BOOTLOG_ENTRIES) {
        eventOverflow = true;
        return;
    }

    memcpy(&events[eventCount], event, sizeof(bootLogEntry_t));
    events[eventCount].timestamp = millis();

    eventCount++;
}

void addBootlogEvent2(bootLogEventCode_e eventCode, uint16_t eventFlags)
{
    bootLogEntry_t  event;
    event.eventCode = eventCode;
    event.eventFlags = eventFlags;
    addBootlogEntry(&event);
}

void addBootlogEvent4(bootLogEventCode_e eventCode, uint16_t eventFlags, uint32_t param1, uint32_t param2)
{
    bootLogEntry_t  event;
    event.eventCode = eventCode;
    event.eventFlags = eventFlags | BOOT_EVENT_FLAGS_PARAM32;
    event.params.u32[0] = param1;
    event.params.u32[1] = param2;
    addBootlogEntry(&event);
}

void addBootlogEvent6(bootLogEventCode_e eventCode, uint16_t eventFlags, uint16_t param1, uint16_t param2, uint16_t param3, uint16_t param4)
{
    bootLogEntry_t  event;
    event.eventCode = eventCode;
    event.eventFlags = eventFlags | BOOT_EVENT_FLAGS_PARAM16;
    event.params.u16[0] = param1;
    event.params.u16[1] = param2;
    event.params.u16[2] = param3;
    event.params.u16[3] = param4;
    addBootlogEntry(&event);
}
#else
const char * getBootlogEventDescription(bootLogEventCode_e eventCode) {UNUSED(eventCode);return NULL;}
void initBootlog(void) {}
int getBootlogEventCount(void) {return 0;}
bootLogEntry_t * getBootlogEvent(int index) {UNUSED(index);return NULL;}
void addBootlogEvent2(bootLogEventCode_e eventCode, uint16_t eventFlags)
    {UNUSED(eventCode);UNUSED(eventFlags);}
void addBootlogEvent4(bootLogEventCode_e eventCode, uint16_t eventFlags, uint32_t param1, uint32_t param2)
    {UNUSED(eventCode);UNUSED(eventFlags);UNUSED(param1);UNUSED(param2);}
void addBootlogEvent6(bootLogEventCode_e eventCode, uint16_t eventFlags, uint16_t param1, uint16_t param2, uint16_t param3, uint16_t param4)
    {UNUSED(eventCode);UNUSED(eventFlags);UNUSED(param1);UNUSED(param2);UNUSED(param3);UNUSED(param4);}
#endif
