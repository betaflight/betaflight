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
*/
#include <platform.h>
#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER

#pragma once

uint8_t Stk_SignOn(void);
uint8_t Stk_ConnectEx(void);
uint8_t Stk_ReadEEprom(void);
uint8_t Stk_WriteEEprom(void);
uint8_t Stk_ReadFlash(void);
uint8_t Stk_WriteFlash(void);
uint8_t Stk_Chip_Erase(void);

#endif
