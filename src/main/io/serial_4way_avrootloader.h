/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 * Author: 4712
 * for info about Hagens AVRootloader:
 * http://www.mikrocontroller.net/topic/avr-bootloader-mit-verschluesselung
*/

#pragma once

void BL_SendBootInit(void);
uint8_t BL_ConnectEx(uint8_32_u *pDeviceInfo);
uint8_t BL_SendCMDKeepAlive(void);
uint8_t BL_PageErase(ioMem_t *pMem);
uint8_t BL_ReadEEprom(ioMem_t *pMem);
uint8_t BL_WriteEEprom(ioMem_t *pMem);
uint8_t BL_WriteFlash(ioMem_t *pMem);
uint8_t BL_ReadFlash(uint8_t interface_mode, ioMem_t *pMem);
void BL_SendCMDRunRestartBootloader(uint8_32_u *pDeviceInfo);
