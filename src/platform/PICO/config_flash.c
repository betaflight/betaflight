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

#include <string.h>

#include "platform.h"
#include "drivers/system.h"
#include "config/config_streamer.h"

#include "hardware/flash.h"
#include "hardware/sync.h"

#if defined(CONFIG_IN_FLASH)

static uint32_t interrupts;

void configFlashUnlock(void)
{
    // TODO: think this one through
    interrupts = save_and_disable_interrupts();
}

void configFlashLock(void)
{
    restore_interrupts(interrupts);
}

void configFlashClearFlag(void)
{
    // NOOP
}

configStreamerResult_e configFlashWriteWord(config_streamer_t *c, config_streamer_buffer_align_type_t *buffer)
{
    if (c->address % FLASH_PAGE_SIZE == 0) {
        // Erase the flash sector before writing
        const int status = flash_range_erase(c->address, FLASH_PAGE_SIZE);
        if (status != 0) {
            return CONFIG_RESULT_FAILURE;
        }
    }

    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == sizeof(uint32_t) * 1,  "CONFIG_STREAMER_BUFFER_SIZE does not match written size");

    // TODO: stream the entire buffer to flash here.
    // Write data to flash
    const int status = flash_range_program(c->address, (uint32_t)*buffer, sizeof(uint32_t));
    if (status != 0) {
        return CONFIG_RESULT_ADDRESS_INVALID;
    }
}

#endif // CONFIG_IN_FLASH
