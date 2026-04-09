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

/*
 * An implementation of persistent data storage utilizing RTC backup data register.
 * Retains values written across software resets and boot loader activities.
 */
/*
*  CH32H41x has no backup register, and the sampling SRAM simulates the operation. 
*  The data in the SRAM is not lost after a hot reset.
*/

#include <stdint.h>
#include "platform.h"

#include "drivers/persistent.h"
#include "drivers/system.h"

#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24)|('e' << 16)|('f' << 8)|('1' << 0))

#define BKP_DATA_ADDR    (0x2017FC00)   //resv  1KB


uint32_t simbkpdataread(uint32_t id)
{
    return *(volatile uint32_t *)(BKP_DATA_ADDR+id*4);
}

void simbkpdatawrite(uint32_t id, uint32_t value)
{
    *(volatile uint32_t *)(BKP_DATA_ADDR+id*4) = value;
}

uint32_t getsoftresetflag(void)
{
    return (((RCC->RSTSCKR) & (RCC_SFTRSTF)) ==  RCC_SFTRSTF); //have softreset
}

uint32_t persistentObjectRead(persistentObjectId_e id)
{
    uint32_t value = simbkpdataread((uint32_t)id);
    return value;
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    simbkpdatawrite((uint32_t)id, value);
}

// static void persistentObjectRTCEnable(void)
// {
//     return ;
// }

void persistentObjectInit(void)
{
    // persistentObjectRTCEnable();
    uint32_t wasSoftReset;

    wasSoftReset = getsoftresetflag();

    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
        for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
    }
}
