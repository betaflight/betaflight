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

// Persistent objects in the SNVS low power general purpose registers, which
// survive a reset; a magic value marks them valid.

#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/persistent.h"

#include "platform/nxp_clock.h"

#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24) | ('e' << 16) | ('f' << 8) | ('1' << 0))

#if defined(IMXRT11XX)

// Four SNVS registers on this family: the magic shares the first with the boot
// mode, and objects without a register read back zero.
#define PERSISTENT_MAGIC_11XX       0xbef1u
#define PERSISTENT_MAGIC_SHIFT      16
#define PERSISTENT_REASON_MASK      0xffffu

STATIC_ASSERT(RESET_BOOTLOADER_REQUEST_FLASH <= PERSISTENT_REASON_MASK,
              "a boot mode has to fit beside the magic in the first SNVS register");

static volatile uint32_t *persistentObjectSlot(persistentObjectId_e id)
{
    switch (id) {
    case PERSISTENT_OBJECT_RESET_REASON:
        return &SNVS->LPGPR[0];
    case PERSISTENT_OBJECT_RTC_HIGH:
        return &SNVS->LPGPR[1];
    case PERSISTENT_OBJECT_RTC_LOW:
        return &SNVS->LPGPR[2];
    case PERSISTENT_OBJECT_SERIALRX_BAUD:
        return &SNVS->LPGPR[3];
    default:
        return NULL;
    }
}

// Which bits of its register an object owns.
static uint32_t persistentObjectMask(persistentObjectId_e id)
{
    return (id == PERSISTENT_OBJECT_RESET_REASON) ? PERSISTENT_REASON_MASK : UINT32_MAX;
}

static bool persistentStoreIsMarked(void)
{
    return (SNVS->LPGPR[0] >> PERSISTENT_MAGIC_SHIFT) == PERSISTENT_MAGIC_11XX;
}

static void persistentStoreMark(void)
{
    SNVS->LPGPR[0] = (SNVS->LPGPR[0] & PERSISTENT_REASON_MASK)
                   | ((uint32_t)PERSISTENT_MAGIC_11XX << PERSISTENT_MAGIC_SHIFT);
}

#elif defined(IMXRT10XX)

STATIC_ASSERT(PERSISTENT_OBJECT_COUNT <= SNVS_LPGPR_COUNT,
              "too many persistent objects for the SNVS general purpose registers");

static volatile uint32_t *persistentObjectSlot(persistentObjectId_e id)
{
    return &SNVS->LPGPR[id];
}

static uint32_t persistentObjectMask(persistentObjectId_e id)
{
    UNUSED(id);
    return UINT32_MAX;
}

static bool persistentStoreIsMarked(void)
{
    return SNVS->LPGPR[PERSISTENT_OBJECT_MAGIC] == PERSISTENT_OBJECT_MAGIC_VALUE;
}

static void persistentStoreMark(void)
{
    SNVS->LPGPR[PERSISTENT_OBJECT_MAGIC] = PERSISTENT_OBJECT_MAGIC_VALUE;
}

#else
#error "Unsupported i.MXRT family: no persistent object store (add the family's backup registers here)"
#endif

static void persistentObjectValidate(void)
{
    if (persistentStoreIsMarked()) {
        return;
    }

    for (unsigned i = 0; i < PERSISTENT_OBJECT_COUNT; i++) {
        volatile uint32_t *slot = persistentObjectSlot((persistentObjectId_e)i);
        if (slot) {
            *slot = 0;
        }
    }

    persistentStoreMark();
}

void persistentObjectInit(void)
{
    nxpClockGateEnable(NXP_CLK_SNVS);

    // Clear the soft lock; a set hard lock in LPLR drops writes until power on reset.
    SNVS->HPLR &= ~SNVS_HPLR_GPR_SL_MASK;

    // Without GPR_Z_DIS a tamper event zeroes the registers.
    SNVS->LPCR |= SNVS_LPCR_GPR_Z_DIS_MASK;

    persistentObjectValidate();
}

uint32_t persistentObjectRead(persistentObjectId_e id)
{
    persistentObjectValidate();

    volatile uint32_t *slot = ((unsigned)id < PERSISTENT_OBJECT_COUNT) ? persistentObjectSlot(id) : NULL;

    return slot ? (*slot & persistentObjectMask(id)) : 0;
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    persistentObjectValidate();

    volatile uint32_t *slot = ((unsigned)id < PERSISTENT_OBJECT_COUNT) ? persistentObjectSlot(id) : NULL;

    if (slot) {
        const uint32_t mask = persistentObjectMask(id);
        *slot = (*slot & ~mask) | (value & mask);
    }
}
