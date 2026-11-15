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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(CONFIG_IN_FLASH)

#include "config/config_streamer.h"

static bool ch32ConfigAddressIsValid(uintptr_t address, uint32_t eraseSize)
{
    const uintptr_t start = (uintptr_t)&__config_start;
    const uintptr_t end = (uintptr_t)&__config_end;

    return start <= end && end - start >= CONFIG_STREAMER_BUFFER_SIZE &&
        start % eraseSize == 0 && end % eraseSize == 0 &&
        address % CONFIG_STREAMER_BUFFER_SIZE == 0 &&
        address >= start && address <= end - CONFIG_STREAMER_BUFFER_SIZE;
}

static configStreamerResult_e ch32FlashStatusToConfigResult(FLASH_Status status)
{
    switch (status) {
    case FLASH_COMPLETE:
        return CONFIG_RESULT_SUCCESS;
    case FLASH_ERROR_WRP:
        return CONFIG_RESULT_ADDRESS_INVALID;
    case FLASH_TIMEOUT:
        return CONFIG_RESULT_TIMEOUT;
    default:
        return CONFIG_RESULT_FAILURE;
    }
}

void configUnlock(void)
{
    FLASH_Unlock_Fast();
}

void configLock(void)
{
    FLASH_Lock_Fast();
    ch32InvalidateFlashCache();
}

void configClearFlags(void)
{
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
}

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    // Erase sectors are 8 KiB in dual-flash mode and 4 KiB in single mode.
    const uint32_t eraseSize = (*(volatile uint32_t *)FLASH_CFGR0_BASE & (1U << 28)) ? 0x2000U : 0x1000U;
    if (!ch32ConfigAddressIsValid(address, eraseSize)) {
        return CONFIG_RESULT_ADDRESS_INVALID;
    }

    if (address % eraseSize == 0) {
        const FLASH_Status eraseStatus = FLASH_ErasePage(address);
        ch32InvalidateFlashCache();
        if (eraseStatus != FLASH_COMPLETE) {
            return ch32FlashStatusToConfigResult(eraseStatus);
        }
    }

    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == sizeof(uint32_t) * 64, "CONFIG_STREAMER_BUFFER_SIZE does not match written size");
    const FLASH_Status progStatus = FLASH_ProgramPage_Fast(address, buffer);
    if (progStatus != FLASH_COMPLETE) {
        ch32InvalidateFlashCache();
        return ch32FlashStatusToConfigResult(progStatus);
    }

    const FLASH_Status status = FLASH_GetStatus();
    ch32InvalidateFlashCache();
    if (status != FLASH_COMPLETE) {
        return ch32FlashStatusToConfigResult(status);
    }

    // A locked fast-program controller can ignore writes without setting an error.
    if (memcmp((const void *)address, buffer, CONFIG_STREAMER_BUFFER_SIZE) != 0) {
        return CONFIG_RESULT_FAILURE;
    }

    return CONFIG_RESULT_SUCCESS;
}

#endif // CONFIG_IN_FLASH
