/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software: you can redistribute
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

/*
 * Single owner of the HPM BGPR (battery-backed general purpose register)
 * word map.  Both supported targets provide 8 words.
 *
 *   word 0       reserved: HPM boot ROM DFU/ISP trigger.  The ROM inspects
 *                only word 0 for DFU_TRIGGER_MAGIC before entering the
 *                bootloader, so this assignment is fixed by the ROM protocol
 *                and cannot move (see DFU_TRIGGER_MAGIC in the HPM SDK).
 *   words 1..7   persistent objects (persistentObjectId_e shifted by
 *                BGPR_PERSISTENT_WORD_OFFSET).
 */

#pragma once

#include "common/utils.h"
#include "drivers/persistent.h"

#define BGPR_WORD_COUNT                 8
#define BGPR_ROM_DFU_TRIGGER_WORD       0
#define BGPR_ROM_DFU_TRIGGER_MAGIC      0x55464455UL
#define BGPR_PERSISTENT_WORD_OFFSET     1

/* The persistent-object range must fit in the words left after the ROM's
 * DFU trigger word. */
STATIC_ASSERT(PERSISTENT_OBJECT_COUNT + BGPR_PERSISTENT_WORD_OFFSET <= BGPR_WORD_COUNT,
              persistentObjectsFitInBgprWords);
