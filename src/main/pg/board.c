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
#include <string.h>

#include "platform.h"

#if defined(USE_BOARD_INFO)
#include "build/version.h"

#include "fc/board_info.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "board.h"

PG_REGISTER_WITH_RESET_FN(boardConfig_t, boardConfig, PG_BOARD_CONFIG, 0);

void pgResetFn_boardConfig(boardConfig_t *boardConfig)
{
    if (boardInformationIsSet()) {
        strncpy(boardConfig->manufacturerId, getManufacturerId(), MAX_MANUFACTURER_ID_LENGTH + 1);
        strncpy(boardConfig->boardName, getBoardName(), MAX_BOARD_NAME_LENGTH + 1);
        boardConfig->boardInformationSet = true;
    } else {
        boardConfig->boardInformationSet = false;
    }

#if defined(USE_SIGNATURE)
    if (signatureIsSet()) {
        memcpy(boardConfig->signature, getSignature(), SIGNATURE_LENGTH);
        boardConfig->signatureSet = true;
    } else {
        boardConfig->signatureSet = false;
    }
#endif
}
#endif // USE_BOARD_INFO:
