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

#pragma once

void initBoardInformation(void);

const char *getBoardName(void);
const char *getManufacturerId(void);
bool boardInformationIsSet(void);

bool setBoardName(const char *newBoardName);
bool setManufacturerId(const char *newManufacturerId);
bool persistBoardInformation(void);

const uint8_t * getSignature(void);
bool signatureIsSet(void);

bool setSignature(const uint8_t *newSignature);
bool persistSignature(void);
