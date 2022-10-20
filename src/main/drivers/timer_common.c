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

#include "platform.h"

#ifdef USE_TIMER

#include "drivers/dshot_bitbang.h"
#include "drivers/io.h"
#include "timer.h"

#ifdef USE_TIMER_MGMT
#include "pg/timerio.h"

const resourceOwner_t freeOwner = { .owner = OWNER_FREE, .resourceIndex = 0 };

static resourceOwner_t timerOwners[MAX_TIMER_PINMAP_COUNT];

timerIOConfig_t *timerIoConfigByTag(ioTag_t ioTag)
{
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            return timerIOConfigMutable(i);
        }
    }

    return NULL;
}

const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex)
{

    if (!ioTag || !timerIndex) {
        return NULL;
    }

    uint8_t index = 1;
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        if (TIMER_HARDWARE[i].tag == ioTag) {
            if (index == timerIndex) {
                return &TIMER_HARDWARE[i];
            }
            ++index;
        }
    }

    return NULL;
}

const timerHardware_t *timerGetConfiguredByTag(ioTag_t ioTag)
{
    uint8_t timerIndex = 0;
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            timerIndex = timerIOConfig(i)->index;

            break;
        }
    }

    return timerGetByTagAndIndex(ioTag, timerIndex);
}

const timerHardware_t *timerGetAllocatedByNumberAndChannel(int8_t timerNumber, uint16_t timerChannel)
{
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        const timerHardware_t *timer = timerGetByTagAndIndex(timerIOConfig(i)->ioTag, timerIOConfig(i)->index);
        if (timer && timerGetTIMNumber(timer->tim) == timerNumber && timer->channel == timerChannel && timerOwners[i].owner) {
            return timer;
        }
    }

#if defined(USE_DSHOT_BITBANG)
    return dshotBitbangTimerGetAllocatedByNumberAndChannel(timerNumber, timerChannel);
#else
    return NULL;
#endif
}

const resourceOwner_t *timerGetOwner(const timerHardware_t *timer)
{
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        const timerHardware_t *assignedTimer = timerGetByTagAndIndex(timerIOConfig(i)->ioTag, timerIOConfig(i)->index);
        if (assignedTimer && assignedTimer == timer) {
            return &timerOwners[i];
        }
    }

#if defined(USE_DSHOT_BITBANG)
    return dshotBitbangTimerGetOwner(timer);
#else
    return &freeOwner;
#endif
}

const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex)
{
    if (!ioTag) {
        return NULL;
    }

    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            const timerHardware_t *timer = timerGetByTagAndIndex(ioTag, timerIOConfig(i)->index);

            if (timerGetOwner(timer)->owner) {
                return NULL;
            }

            timerOwners[i].owner = owner;
            timerOwners[i].resourceIndex = resourceIndex;

            return timer;
        }
    }

    return NULL;
}
#else

const timerHardware_t *timerGetConfiguredByTag(ioTag_t ioTag)
{
#if TIMER_CHANNEL_COUNT > 0
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        if (TIMER_HARDWARE[i].tag == ioTag) {
            return &TIMER_HARDWARE[i];
        }
    }
#else
    UNUSED(ioTag);
#endif
    return NULL;
}

const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex)
{
    UNUSED(owner);
    UNUSED(resourceIndex);

    return timerGetConfiguredByTag(ioTag);
}
#endif

ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index)
{
#if USABLE_TIMER_CHANNEL_COUNT > 0
    uint8_t currentIndex = 0;
    for (unsigned i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if ((timerHardware[i].usageFlags & usageFlag) == usageFlag) {
            if (currentIndex == index) {
                return timerHardware[i].tag;
            }
            currentIndex++;
        }
    }
#else
    UNUSED(usageFlag);
    UNUSED(index);
#endif
    return IO_TAG_NONE;
}
#endif
