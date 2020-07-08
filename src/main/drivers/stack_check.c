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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "common/utils.h"

#include "drivers/stack_check.h"

#define STACK_FILL_CHAR 0xa5

extern char _estack; // end of stack, declared in .LD file
extern char _Min_Stack_Size; // declared in .LD file

/*
 * The ARM processor uses a full descending stack. This means the stack pointer holds the address
 * of the last stacked item in memory. When the processor pushes a new item onto the stack,
 * it decrements the stack pointer and then writes the item to the new memory location.
 *
 *
 * RAM layout is generally as below, although some targets vary
 *
 * F1 Boards
 * RAM is origin 0x20000000 length 20K that is:
 * 0x20000000 to 0x20005000
 *
 * F3 Boards
 * RAM is origin 0x20000000 length 40K that is:
 * 0x20000000 to 0x2000a000
 *
 * F4 Boards
 * RAM is origin 0x20000000 length 128K that is:
 * 0x20000000 to 0x20020000
 *
 * See the linker scripts for actual stack configuration.
 */

#ifdef USE_STACK_CHECK

static uint32_t usedStackSize;

void taskStackCheck(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    char * const stackHighMem = &_estack;
    const uint32_t stackSize = (uint32_t)&_Min_Stack_Size;
    char * const stackLowMem = stackHighMem - stackSize;
    const char * const stackCurrent = (char *)&stackLowMem;

    char *p;
    for (p = stackLowMem; p < stackCurrent; ++p) {
        if (*p != STACK_FILL_CHAR) {
            break;
        }
    }

    usedStackSize = (uint32_t)stackHighMem - (uint32_t)p;

    DEBUG_SET(DEBUG_STACK, 0, (uint32_t)stackHighMem & 0xffff);
    DEBUG_SET(DEBUG_STACK, 1, (uint32_t)stackLowMem & 0xffff);
    DEBUG_SET(DEBUG_STACK, 2, (uint32_t)stackCurrent & 0xffff);
    DEBUG_SET(DEBUG_STACK, 3, (uint32_t)p & 0xffff);
}

uint32_t stackUsedSize(void)
{
    return usedStackSize;
}
#endif

uint32_t stackTotalSize(void)
{
    return (uint32_t)(intptr_t)&_Min_Stack_Size;
}

uint32_t stackHighMem(void)
{
    return (uint32_t)(intptr_t)&_estack;
}
