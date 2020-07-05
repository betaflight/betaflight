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

static uint8_t timerIndexByTag(ioTag_t ioTag)
{
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            return timerIOConfig(i)->index;
        }
    }
    return 0;
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

const timerHardware_t *timerGetByTag(ioTag_t ioTag)
{
    uint8_t timerIndex = timerIndexByTag(ioTag);

    return timerGetByTagAndIndex(ioTag, timerIndex);
}

const resourceOwner_t *timerGetOwner(int8_t timerNumber, uint16_t timerChannel)
{
    const resourceOwner_t *timerOwner = &freeOwner;
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        const timerHardware_t *timer = timerGetByTagAndIndex(timerIOConfig(i)->ioTag, timerIOConfig(i)->index);
        if (timer && timerGetTIMNumber(timer->tim) == timerNumber && timer->channel == timerChannel) {
            timerOwner = &timerOwners[i];

            break;
        }
    }

#if defined(USE_DSHOT_BITBANG)
    if (!timerOwner->owner) {
        timerOwner = dshotBitbangTimerGetOwner(timerNumber, timerChannel);
    }
#endif

    return timerOwner;
}

const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex)
{
    if (!ioTag) {
        return NULL;
    }

    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            const timerHardware_t *timer = timerGetByTagAndIndex(ioTag, timerIOConfig(i)->index);

            if (timerGetOwner(timerGetTIMNumber(timer->tim), timer->channel)->owner) {
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

const timerHardware_t *timerGetByTag(ioTag_t ioTag)
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

    return timerGetByTag(ioTag);
}
#endif

ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index)
{
#if !defined(USE_UNIFIED_TARGET) && USABLE_TIMER_CHANNEL_COUNT > 0
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
