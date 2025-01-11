/*
* The MIT License (MIT)
 *
 * Copyright (c) 2024 Ha Thach (tinyusb.org)
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

#ifndef TUSB_DWC2_COMMON_H
#define TUSB_DWC2_COMMON_H

#include "common/tusb_common.h"
#include "dwc2_type.h"

// Following symbols must be defined by port header
// - _dwc2_controller[]: array of controllers
// - DWC2_EP_MAX: largest EP counts of all controllers
// - dwc2_phy_init/dwc2_phy_update: phy init called before and after core reset
// - dwc2_dcd_int_enable/dwc2_dcd_int_disable
// - dwc2_remote_wakeup_delay

#if defined(TUP_USBIP_DWC2_STM32)
  #include "dwc2_stm32.h"
#elif defined(TUP_USBIP_DWC2_ESP32)
  #include "dwc2_esp32.h"
#elif TU_CHECK_MCU(OPT_MCU_GD32VF103)
  #include "dwc2_gd32.h"
#elif TU_CHECK_MCU(OPT_MCU_BCM2711, OPT_MCU_BCM2835, OPT_MCU_BCM2837)
  #include "dwc2_bcm.h"
#elif TU_CHECK_MCU(OPT_MCU_EFM32GG)
  #include "dwc2_efm32.h"
#elif TU_CHECK_MCU(OPT_MCU_XMC4000)
  #include "dwc2_xmc.h"
#else
  #error "Unsupported MCUs"
#endif

enum {
  DWC2_CONTROLLER_COUNT = TU_ARRAY_SIZE(_dwc2_controller)
};

enum {
  OTG_INT_COMMON = 0 // GINTSTS_DISCINT | GINTSTS_CONIDSTSCHNG
};

//--------------------------------------------------------------------+
// Core/Controller
//--------------------------------------------------------------------+
TU_ATTR_ALWAYS_INLINE static inline dwc2_regs_t* DWC2_REG(uint8_t rhport) {
  if (rhport >= DWC2_CONTROLLER_COUNT) {
    // user mis-configured, ignore and use first controller
    rhport = 0;
  }
  return (dwc2_regs_t*)_dwc2_controller[rhport].reg_base;
}

bool dwc2_core_is_highspeed(dwc2_regs_t* dwc2, tusb_role_t role);
bool dwc2_core_init(uint8_t rhport, bool is_highspeed, bool is_dma);
void dwc2_core_handle_common_irq(uint8_t rhport, bool in_isr);

//--------------------------------------------------------------------+
// DFIFO
//--------------------------------------------------------------------+
TU_ATTR_ALWAYS_INLINE static inline void dfifo_flush_tx(dwc2_regs_t* dwc2, uint8_t fnum) {
  // flush TX fifo and wait for it cleared
  dwc2->grstctl = GRSTCTL_TXFFLSH | (fnum << GRSTCTL_TXFNUM_Pos);
  while (dwc2->grstctl & GRSTCTL_TXFFLSH_Msk) {}
}

TU_ATTR_ALWAYS_INLINE static inline void dfifo_flush_rx(dwc2_regs_t* dwc2) {
  // flush RX fifo and wait for it cleared
  dwc2->grstctl = GRSTCTL_RXFFLSH;
  while (dwc2->grstctl & GRSTCTL_RXFFLSH_Msk) {}
}

void dfifo_read_packet(dwc2_regs_t* dwc2, uint8_t* dst, uint16_t len);
void dfifo_write_packet(dwc2_regs_t* dwc2, uint8_t fifo_num, uint8_t const* src, uint16_t len);

//--------------------------------------------------------------------+
// DMA
//--------------------------------------------------------------------+

#endif
