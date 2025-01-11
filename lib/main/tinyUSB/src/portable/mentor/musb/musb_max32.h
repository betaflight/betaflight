/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024, Brent Kowal (Analog Devices, Inc)
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

#ifndef TUSB_MUSB_MAX32_H_
#define TUSB_MUSB_MAX32_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mxc_device.h"
#include "usbhs_regs.h"

#define MUSB_CFG_SHARED_FIFO   1 // shared FIFO for TX and RX endpoints
#define MUSB_CFG_DYNAMIC_FIFO  0 // dynamic EP FIFO sizing

const uintptr_t MUSB_BASES[] = { MXC_BASE_USBHS };

#if CFG_TUD_ENABLED
#define USBHS_M31_CLOCK_RECOVERY

// Mapping of IRQ numbers to port. Currently just 1.
static const IRQn_Type musb_irqs[] = {
    USB_IRQn
};

TU_ATTR_ALWAYS_INLINE static inline void musb_dcd_int_enable(uint8_t rhport) {
  NVIC_EnableIRQ(musb_irqs[rhport]);
}

TU_ATTR_ALWAYS_INLINE static inline void musb_dcd_int_disable(uint8_t rhport) {
  NVIC_DisableIRQ(musb_irqs[rhport]);
}

TU_ATTR_ALWAYS_INLINE static inline unsigned musb_dcd_get_int_enable(uint8_t rhport) {
  #ifdef NVIC_GetEnableIRQ // only defined in CMSIS 5
  return NVIC_GetEnableIRQ(musb_irqs[rhport]);
  #else
  uint32_t IRQn = (uint32_t) musb_irqs[rhport];
  return ((NVIC->ISER[IRQn >> 5UL] & (1UL << (IRQn & 0x1FUL))) != 0UL) ? 1UL : 0UL;
  #endif
}

TU_ATTR_ALWAYS_INLINE static inline void musb_dcd_int_clear(uint8_t rhport) {
  NVIC_ClearPendingIRQ(musb_irqs[rhport]);
}

static inline void musb_dcd_int_handler_enter(uint8_t rhport) {
  mxc_usbhs_regs_t* hs_phy = MXC_USBHS;
  uint32_t mxm_int, mxm_int_en, mxm_is;

  //Handle PHY specific events
  mxm_int = hs_phy->mxm_int;
  mxm_int_en = hs_phy->mxm_int_en;
  mxm_is = mxm_int & mxm_int_en;
  hs_phy->mxm_int = mxm_is;

  if (mxm_is & MXC_F_USBHS_MXM_INT_NOVBUS) {
    dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
  }
}

static inline void musb_dcd_phy_init(uint8_t rhport) {
  (void) rhport;
  mxc_usbhs_regs_t* hs_phy = MXC_USBHS;

  // Interrupt for VBUS disconnect
  hs_phy->mxm_int_en |= MXC_F_USBHS_MXM_INT_EN_NOVBUS;

  musb_dcd_int_clear(rhport);

  // Unsuspend the MAC
  hs_phy->mxm_suspend = 0;

  // Configure PHY
  hs_phy->m31_phy_xcfgi_31_0 = (0x1 << 3) | (0x1 << 11);
  hs_phy->m31_phy_xcfgi_63_32 = 0;
  hs_phy->m31_phy_xcfgi_95_64 = 0x1 << (72 - 64);
  hs_phy->m31_phy_xcfgi_127_96 = 0;

  #ifdef USBHS_M31_CLOCK_RECOVERY
  hs_phy->m31_phy_noncry_rstb = 1;
  hs_phy->m31_phy_noncry_en = 1;
  hs_phy->m31_phy_outclksel = 0;
  hs_phy->m31_phy_coreclkin = 0;
  hs_phy->m31_phy_xtlsel = 2; /* Select 25 MHz clock */
  #else
  hs_phy->m31_phy_noncry_rstb = 0;
  hs_phy->m31_phy_noncry_en = 0;
  hs_phy->m31_phy_outclksel = 1;
  hs_phy->m31_phy_coreclkin = 1;
  hs_phy->m31_phy_xtlsel = 3; /* Select 30 MHz clock */
  #endif
  hs_phy->m31_phy_pll_en = 1;
  hs_phy->m31_phy_oscouten = 1;

  /* Reset PHY */
  hs_phy->m31_phy_ponrst = 0;
  hs_phy->m31_phy_ponrst = 1;
}

// static inline void musb_dcd_setup_fifo(uint8_t rhport, unsigned epnum, unsigned dir_in, unsigned mps) {
//   (void) mps;
//
//   //Most likely the caller has already grabbed the right register block. But
//   //as a precaution save and restore the register bank anyways
//   unsigned saved_index = musb_periph_inst[rhport]->index;
//
//   musb_periph_inst[rhport]->index = epnum;
//
//   //Disable double buffering
//   if (dir_in) {
//     musb_periph_inst[rhport]->incsru |= (MXC_F_USBHS_INCSRU_DPKTBUFDIS | MXC_F_USBHS_INCSRU_MODE);
//   } else {
//     musb_periph_inst[rhport]->outcsru |= (MXC_F_USBHS_OUTCSRU_DPKTBUFDIS);
//   }
//
//   musb_periph_inst[rhport]->index = saved_index;
// }

#endif // CFG_TUD_ENABLED

#ifdef __cplusplus
}
#endif

#endif // TUSB_MUSB_MAX32_H_
