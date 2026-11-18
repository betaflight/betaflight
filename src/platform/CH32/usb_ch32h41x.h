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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/usb_io.h"

#define CH32_USBHS_PLL_LOCK_TIMEOUT 100000u

static inline bool ch32UsbHsResolvePllConfig(uint32_t hseHz, uint32_t *pllRefer, uint32_t *pllInDiv)
{
    if ((hseHz % 25000000u) == 0 && hseHz != 0 && (hseHz / 25000000u) <= 32u) {
        *pllRefer = RCC_USBHSPLLRefer_25M;
        *pllInDiv = (uint32_t)(((hseHz / 25000000u) - 1u) << 8);
        return true;
    }
    if ((hseHz % 32000000u) == 0 && hseHz != 0 && (hseHz / 32000000u) <= 32u) {
        *pllRefer = RCC_USBHSPLLRefer_32M;
        *pllInDiv = (uint32_t)(((hseHz / 32000000u) - 1u) << 8);
        return true;
    }
    if ((hseHz % 24000000u) == 0 && hseHz != 0 && (hseHz / 24000000u) <= 32u) {
        *pllRefer = RCC_USBHSPLLRefer_24M;
        *pllInDiv = (uint32_t)(((hseHz / 24000000u) - 1u) << 8);
        return true;
    }
    if ((hseHz % 20000000u) == 0 && hseHz != 0 && (hseHz / 20000000u) <= 32u) {
        *pllRefer = RCC_USBHSPLLRefer_20M;
        *pllInDiv = (uint32_t)(((hseHz / 20000000u) - 1u) << 8);
        return true;
    }
    return false;
}

static inline bool ch32UsbHsPhyInit(void)
{
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    IOInit(IOGetByTag(IO_TAG(PB8)), OWNER_USB, 0);
    IOInit(IOGetByTag(IO_TAG(PB9)), OWNER_USB, 0);

#ifdef USE_VCP
    usbGenerateDisconnectPulse();
#endif

    if ((RCC->PLLCFGR & RCC_SYSPLL_SEL) != RCC_SYSPLL_USBHS) {
        uint32_t pllRefer = RCC_USBHSPLLRefer_25M;
        uint32_t pllInDiv = RCC_USBHSPLL_IN_Div1;
        if (!ch32UsbHsResolvePllConfig(HSE_VALUE, &pllRefer, &pllInDiv)) {
            return false;
        }
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig(RCC_USBHSPLLSource_HSE);
        RCC_USBHSPLLReferConfig(pllRefer);
        RCC_USBHSPLLClockSourceDivConfig(pllInDiv);
        RCC_USBHS_PLLCmd(ENABLE);
        for (volatile uint32_t wait = 0; RCC_GetFlagStatus(RCC_FLAG_USBHSPLLRDY) == RESET; wait++) {
            if (wait >= CH32_USBHS_PLL_LOCK_TIMEOUT) {
                return false;
            }
        }
    }

    RCC_UTMIcmd(ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, ENABLE);
    return true;
}

static inline const char *ch32UsbGetUidSerialString(char serialBuf[25])
{
    static const char hexDigits[16] = "0123456789ABCDEF";
    const uint32_t words[3] = { U_ID_0, U_ID_1, U_ID_2 };

    for (unsigned w = 0; w < 3U; w++) {
        const uint32_t val = words[w];
        for (unsigned nib = 0; nib < 8U; nib++) {
            serialBuf[w * 8U + nib] = hexDigits[(val >> (28U - nib * 4U)) & 0x0FU];
        }
    }
    serialBuf[24] = '\0';
    return serialBuf;
}
