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
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SYSTEM_X32M7XX_H__
#define __SYSTEM_X32M7XX_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/** _System */

 /** Power supply source configuration **/
#define PWR_SUPPLY_MODE_MASK               (PWR_SYSCTRL4_MLDOEN | PWR_SYSCTRL4_DCDCEN | PWR_SYSCTRL4_VCORESRC | PWR_SYSCTRL4_DCDCFRCEN)    
#define PWR_LDO_SUPPLY                     (PWR_SYSCTRL4_MLDOEN)             /* Core domains are supplied from the LDO  */
#define PWR_DIRECT_SMPS_SUPPLY             (PWR_SYSCTRL4_DCDCEN)             /* Core domains are supplied from the SMPS */ 
#define PWR_EXTERNAL_SOURCE_SUPPLY         (PWR_SYSCTRL4_VCORESRC)           /* The SMPS and the LDO are Bypassed. The Core domains are supplied from an external source */

 /** NRST Analog and Digital Filter configuration **/
#define PWR_RST_AGFBPEN_MAST              (PWR_SYSCTRL1_AGF_ARSTOBP)
#define PWR_RST_DGFBPEN_MAST              (PWR_SYSCTRL1_NRST_DGFBP)
#define PWR_RST_DGF_CNT_MAST              (PWR_SYSCTRL1_NRST_DGFCNT)
#define PWR_RST_DGF_CNT_DEFAULT           ((uint32_t)0x200000U)

extern uint32_t SystemCoreClock; /* System Clock Frequency (Core Clock) */

extern void SystemInit(void);
#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_X32M7XX_H__ */
