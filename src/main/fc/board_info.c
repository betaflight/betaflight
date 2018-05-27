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
#include "pg/board.h"

static bool boardInformationSet = false;
static char manufacturerId[MAX_MANUFACTURER_ID_LENGTH + 1];
static char boardName[MAX_BOARD_NAME_LENGTH + 1];

static bool signatureSet = false;
static uint8_t signature[SIGNATURE_LENGTH];

void initBoardInformation(void)
{
    boardInformationSet = boardConfig()->boardInformationSet;
    if (boardInformationSet) {
        strncpy(manufacturerId, boardConfig()->manufacturerId, MAX_MANUFACTURER_ID_LENGTH);
        strncpy(boardName, boardConfig()->boardName, MAX_BOARD_NAME_LENGTH);
    }

    signatureSet = boardConfig()->signatureSet;
    if (signatureSet) {
        memcpy(signature, boardConfig()->signature, SIGNATURE_LENGTH);
    }
}

const char *getManufacturerId(void)
{
    return manufacturerId;
}

const char *getBoardName(void)
{
    return boardName;
}

bool boardInformationIsSet(void)
{
    return boardInformationSet;
}

bool setManufacturerId(const char *newManufacturerId)
{
    if (!boardInformationSet) {
        strncpy(manufacturerId, newManufacturerId, MAX_MANUFACTURER_ID_LENGTH);

        return true;
    } else {
        return false;
    }
}

bool setBoardName(const char *newBoardName)
{
    if (!boardInformationSet) {
        strncpy(boardName, newBoardName, MAX_BOARD_NAME_LENGTH);

        return true;
    } else {
        return false;
    }
}

bool persistBoardInformation(void)
{
    if (!boardInformationSet) {
        strncpy(boardConfigMutable()->manufacturerId, manufacturerId, MAX_MANUFACTURER_ID_LENGTH);
        strncpy(boardConfigMutable()->boardName, boardName, MAX_BOARD_NAME_LENGTH);
        boardConfigMutable()->boardInformationSet = true;

        initBoardInformation();

        return true;
    } else {
        return false;
    }
}

#if defined(USE_SIGNATURE)
const uint8_t *getSignature(void)
{
    return signature;
}

bool signatureIsSet(void)
{
    return signatureSet;
}

bool setSignature(const uint8_t *newSignature)
{
    if (!signatureSet) {
        memcpy(signature, newSignature, SIGNATURE_LENGTH);

        return true;
    } else {
        return false;
    }
}

bool persistSignature(void)
{
    if (!signatureSet) {
        memcpy(boardConfigMutable()->signature, signature, SIGNATURE_LENGTH);
        boardConfigMutable()->signatureSet = true;

        initBoardInformation();

        return true;
    } else {
        return false;
    }
}
#endif
#endif // USE_BOARD_INFO
