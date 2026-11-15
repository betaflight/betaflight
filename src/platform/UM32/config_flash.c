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

#include "platform.h"

#if defined(CONFIG_IN_FLASH)

#include "common/utils.h"
#include "config/config_streamer_impl.h"

#if !defined(FLASH_PAGE_SIZE)
#error "Flash page size not defined for UM32 target."
#endif

#define FLASH_CACHE_STATE_POLL_LIMIT 1000000U

static bool waitForFlashCacheState(const CACHE_TypeDef *cache, uint32_t state)
{
    for (unsigned attempt = 0; attempt < FLASH_CACHE_STATE_POLL_LIMIT; attempt++) {
        if ((cache->STATUS & CACHE_STATUS_CACHE_STATUS) == state) {
            return true;
        }
    }
    return false;
}

static bool restartFlashCache(CACHE_TypeDef *cache)
{
    if (!(cache->CTRL & CACHE_CTRL_CACHE_EN)) {
        return true;
    }

    CLEAR_BIT(cache->CTRL, CACHE_CTRL_CACHE_EN);
    const bool stopped = waitForFlashCacheState(cache, CACHE_STATUS_CLOSED_FLAG);
    // Restore the original enable bit even if the stop timed out.
    SET_BIT(cache->CTRL, CACHE_CTRL_CACHE_EN);
    const bool started = waitForFlashCacheState(cache, CACHE_STATUS_OPEND_FLAG);
    return stopped && started;
}

void configUnlock(void)
{
    HAL_FLASH_Unlock();
}

configStreamerResult_e configLock(void)
{
    HAL_FLASH_Lock();
    // UM324xF manual section 4.3.2.2: restart the internal Flash caches
    // after programming, before the config is read back for validation.
    __DSB();
    const bool instructionCacheReady = restartFlashCache(ICACHE);
    const bool dataCacheReady = restartFlashCache(DCACHE);
    __DSB();
    __ISB();
    if (!instructionCacheReady || !dataCacheReady) {
        return CONFIG_RESULT_TIMEOUT;
    }
    return CONFIG_RESULT_SUCCESS;
}

void configClearFlags(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPDS | FLASH_FLAG_PVFS | FLASH_FLAG_EVFS);
}

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    if (address % FLASH_PAGE_SIZE == 0) {
        const HAL_StatusTypeDef status = HAL_FLASH_Erase_Page(address);
        if (status != HAL_OK) {
            return CONFIG_RESULT_FAILURE;
        }
    }

    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == sizeof(uint32_t), "CONFIG_STREAMER_BUFFER_SIZE does not match written size");
    const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *buffer);
    if (status != HAL_OK) {
        return CONFIG_RESULT_ADDRESS_INVALID;
    }
    return CONFIG_RESULT_SUCCESS;
}

#endif // CONFIG_IN_FLASH
