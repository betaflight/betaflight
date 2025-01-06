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

void configUnlock(void)
{
    // NOOP
}

void configLock(void)
{
    // NOOP
}

void configFlashClearFlags(void)
{
    // NOOP
}

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
    uint32_t interrupts = save_and_disable_interrupts();

    if (address == __config_start) {
        // Erase the flash sector before writing
        flash_range_erase(address, FLASH_PAGE_SIZE);
    }

    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == sizeof(config_streamer_buffer_type_t) * CONFIG_STREAMER_BUFFER_SIZE,  "CONFIG_STREAMER_BUFFER_SIZE does not match written size");

    // Write data to flash
    // TODO: synchronise second core...

    flash_range_program(address, buffer, CONFIG_STREAMER_BUFFER_SIZE);

    restore_interrupts(interrupts);
    return CONFIG_RESULT_SUCCESS;
}

#endif // CONFIG_IN_FLASH
