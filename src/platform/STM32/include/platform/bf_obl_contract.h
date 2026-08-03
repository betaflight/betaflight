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

#pragma once

/*
 * Shared contract between the Betaflight OpenBootloader
 * (lib/main/STM32/n6_obl) and the BF application.
 *
 * OBL arms IWDG before jumping to BF and decides DFU vs retry on the
 * next boot from RCC->RSR (IWDGRSTF / LCKRSTF / WWDGRSTF set means BF
 * crashed). BF's only job is to refresh the IWDG from a periodic task,
 * or OBL's ~10 s reload trips on a healthy BF.
 */

/*
 * Direct register write avoids requiring HAL_IWDG_MODULE_ENABLED in BF.
 * 0xAAAA is the IWDG reload key; all the HAL refresh wrapper does.
 */
#define BF_OBL_IWDG_REFRESH() do { IWDG->KR = 0x0000AAAAU; } while (0)
