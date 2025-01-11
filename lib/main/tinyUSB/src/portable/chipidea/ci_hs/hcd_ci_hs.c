/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

// Chipidea Highspeed USB IP implement EHCI for host functionality

#if CFG_TUH_ENABLED && defined(TUP_USBIP_EHCI)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "common/tusb_common.h"
#include "host/hcd.h"
#include "portable/ehci/ehci_api.h"
#include "ci_hs_type.h"

#if CFG_TUSB_MCU == OPT_MCU_MIMXRT1XXX

#include "ci_hs_imxrt.h"

#if CFG_TUH_MEM_DCACHE_ENABLE
bool hcd_dcache_clean(void const* addr, uint32_t data_size) {
  return imxrt_dcache_clean(addr, data_size);
}

bool hcd_dcache_invalidate(void const* addr, uint32_t data_size) {
  return imxrt_dcache_invalidate(addr, data_size);
}

bool hcd_dcache_clean_invalidate(void const* addr, uint32_t data_size) {
  return imxrt_dcache_clean_invalidate(addr, data_size);
}
#endif

#elif TU_CHECK_MCU(OPT_MCU_LPC18XX, OPT_MCU_LPC43XX)

#include "ci_hs_lpc18_43.h"

#else
#error "Unsupported MCUs"
#endif

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+

bool hcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  ci_hs_regs_t *hcd_reg = CI_HS_REG(rhport);

  // Reset controller
  hcd_reg->USBCMD |= USBCMD_RESET;
  while ( hcd_reg->USBCMD & USBCMD_RESET ) {}

  // Set mode to device, must be set immediately after reset
#if CFG_TUSB_MCU == OPT_MCU_LPC18XX || CFG_TUSB_MCU == OPT_MCU_LPC43XX
  // LPC18XX/43XX need to set VBUS Power Select to HIGH
  // RHPORT1 is fullspeed only (need external PHY for Highspeed)
  hcd_reg->USBMODE = USBMODE_CM_HOST | USBMODE_VBUS_POWER_SELECT;
  if (rhport == 1) {
    hcd_reg->PORTSC1 |= PORTSC1_FORCE_FULL_SPEED;
  }
#else
  hcd_reg->USBMODE = USBMODE_CM_HOST;
#endif

  // FIXME force full speed, still have issue with Highspeed enumeration
  // probably due to physical connection bouncing when plug/unplug
  // 1. Have issue when plug/unplug devices, maybe the port is not reset properly
  // 2. Also does not seems to detect disconnection
  hcd_reg->PORTSC1 |= PORTSC1_FORCE_FULL_SPEED;

  return ehci_init(rhport, (uint32_t) &hcd_reg->CAPLENGTH, (uint32_t) &hcd_reg->USBCMD);
}

void hcd_int_enable(uint8_t rhport) {
  CI_HCD_INT_ENABLE(rhport);
}

void hcd_int_disable(uint8_t rhport) {
  CI_HCD_INT_DISABLE(rhport);
}

#endif
