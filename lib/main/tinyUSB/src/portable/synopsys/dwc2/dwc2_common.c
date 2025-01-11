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

#include "tusb_option.h"

#define DWC2_COMMON_DEBUG   2

#if defined(TUP_USBIP_DWC2) && (CFG_TUH_ENABLED || CFG_TUD_ENABLED)

#if CFG_TUD_ENABLED
#include "device/dcd.h"
#endif

#if CFG_TUH_ENABLED
#include "host/hcd.h"
#endif

#include "dwc2_common.h"

//--------------------------------------------------------------------
//
//--------------------------------------------------------------------
static void reset_core(dwc2_regs_t* dwc2) {
  // reset core
  dwc2->grstctl |= GRSTCTL_CSRST;

  if ((dwc2->gsnpsid & DWC2_CORE_REV_MASK) < (DWC2_CORE_REV_4_20a & DWC2_CORE_REV_MASK)) {
    // prior v42.0 CSRST is self-clearing
    while (dwc2->grstctl & GRSTCTL_CSRST) {}
  } else {
    // From v4.20a CSRST bit is write only, CSRT_DONE (w1c) is introduced for checking.
    // CSRST must also be explicitly cleared
    while (!(dwc2->grstctl & GRSTCTL_CSRST_DONE)) {}
    dwc2->grstctl =  (dwc2->grstctl & ~GRSTCTL_CSRST) | GRSTCTL_CSRST_DONE;
  }

  while (!(dwc2->grstctl & GRSTCTL_AHBIDL)) {} // wait for AHB master IDLE
}

static void phy_fs_init(dwc2_regs_t* dwc2) {
  TU_LOG(DWC2_COMMON_DEBUG, "Fullspeed PHY init\r\n");

  uint32_t gusbcfg = dwc2->gusbcfg;

  // Select FS PHY
  gusbcfg |= GUSBCFG_PHYSEL;
  dwc2->gusbcfg = gusbcfg;

  // MCU specific PHY init before reset
  dwc2_phy_init(dwc2, GHWCFG2_HSPHY_NOT_SUPPORTED);

  // Reset core after selecting PHY
  reset_core(dwc2);

  // USB turnaround time is critical for certification where long cables and 5-Hubs are used.
  // So if you need the AHB to run at less than 30 MHz, and if USB turnaround time is not critical,
  // these bits can be programmed to a larger value. Default is 5
  gusbcfg &= ~GUSBCFG_TRDT_Msk;
  gusbcfg |= 5u << GUSBCFG_TRDT_Pos;
  dwc2->gusbcfg = gusbcfg;

  // MCU specific PHY update post reset
  dwc2_phy_update(dwc2, GHWCFG2_HSPHY_NOT_SUPPORTED);
}

static void phy_hs_init(dwc2_regs_t* dwc2) {
  uint32_t gusbcfg = dwc2->gusbcfg;

  // De-select FS PHY
  gusbcfg &= ~GUSBCFG_PHYSEL;

  if (dwc2->ghwcfg2_bm.hs_phy_type == GHWCFG2_HSPHY_ULPI) {
    TU_LOG(DWC2_COMMON_DEBUG, "Highspeed ULPI PHY init\r\n");

    // Select ULPI PHY (external)
    gusbcfg |= GUSBCFG_ULPI_UTMI_SEL;

    // ULPI is always 8-bit interface
    gusbcfg &= ~GUSBCFG_PHYIF16;

    // ULPI select single data rate
    gusbcfg &= ~GUSBCFG_DDRSEL;

    // default internal VBUS Indicator and Drive
    gusbcfg &= ~(GUSBCFG_ULPIEVBUSD | GUSBCFG_ULPIEVBUSI);

    // Disable FS/LS ULPI
    gusbcfg &= ~(GUSBCFG_ULPIFSLS | GUSBCFG_ULPICSM);
  } else {
    TU_LOG(DWC2_COMMON_DEBUG, "Highspeed UTMI+ PHY init\r\n");

    // Select UTMI+ PHY (internal)
    gusbcfg &= ~GUSBCFG_ULPI_UTMI_SEL;

    // Set 16-bit interface if supported
    if (dwc2->ghwcfg4_bm.phy_data_width) {
      gusbcfg |= GUSBCFG_PHYIF16; // 16 bit
    } else {
      gusbcfg &= ~GUSBCFG_PHYIF16; // 8 bit
    }
  }

  // Apply config
  dwc2->gusbcfg = gusbcfg;

  // mcu specific phy init
  dwc2_phy_init(dwc2, dwc2->ghwcfg2_bm.hs_phy_type);

  // Reset core after selecting PHY
  reset_core(dwc2);

  // Set turn-around, must after core reset otherwise it will be clear
  // - 9 if using 8-bit PHY interface
  // - 5 if using 16-bit PHY interface
  gusbcfg &= ~GUSBCFG_TRDT_Msk;
  gusbcfg |= (dwc2->ghwcfg4_bm.phy_data_width ? 5u : 9u) << GUSBCFG_TRDT_Pos;
  dwc2->gusbcfg = gusbcfg;

  // MCU specific PHY update post reset
  dwc2_phy_update(dwc2, dwc2->ghwcfg2_bm.hs_phy_type);
}

static bool check_dwc2(dwc2_regs_t* dwc2) {
#if CFG_TUSB_DEBUG >= DWC2_COMMON_DEBUG
  // print guid, gsnpsid, ghwcfg1, ghwcfg2, ghwcfg3, ghwcfg4
  // Run 'python dwc2_info.py' and check dwc2_info.md for bit-field value and comparison with other ports
  volatile uint32_t const* p = (volatile uint32_t const*) &dwc2->guid;
  TU_LOG1("guid, gsnpsid, ghwcfg1, ghwcfg2, ghwcfg3, ghwcfg4\r\n");
  for (size_t i = 0; i < 5; i++) {
    TU_LOG1("0x%08" PRIX32 ", ", p[i]);
  }
  TU_LOG1("0x%08" PRIX32 "\r\n", p[5]);
#endif

  // For some reason: GD32VF103 gsnpsid and all hwcfg register are always zero (skip it)
  (void)dwc2;
#if !TU_CHECK_MCU(OPT_MCU_GD32VF103)
  enum { GSNPSID_ID_MASK = TU_GENMASK(31, 16) };
  const uint32_t gsnpsid = dwc2->gsnpsid & GSNPSID_ID_MASK;
  TU_ASSERT(gsnpsid == DWC2_OTG_ID || gsnpsid == DWC2_FS_IOT_ID || gsnpsid == DWC2_HS_IOT_ID);
#endif

  return true;
}

//--------------------------------------------------------------------
//
//--------------------------------------------------------------------
bool dwc2_core_is_highspeed(dwc2_regs_t* dwc2, tusb_role_t role) {
  (void)dwc2;

#if CFG_TUD_ENABLED
  if (role == TUSB_ROLE_DEVICE && !TUD_OPT_HIGH_SPEED) {
    return false;
  }
#endif
#if CFG_TUH_ENABLED
  if (role == TUSB_ROLE_HOST && !TUH_OPT_HIGH_SPEED) {
    return false;
  }
#endif

  return dwc2->ghwcfg2_bm.hs_phy_type != GHWCFG2_HSPHY_NOT_SUPPORTED;
}

/* dwc2 has several PHYs option
 * - UTMI+ is internal highspeed PHY, clock can be 30 Mhz (8-bit) or 60 Mhz (16-bit)
 * - ULPI is external highspeed PHY, clock is 60Mhz with only 8-bit interface
 * - Dedicated FS PHY is internal with clock 48Mhz.
 *
 * In addition, UTMI+/ULPI can be shared to run at fullspeed mode with 48Mhz
 *
*/
bool dwc2_core_init(uint8_t rhport, bool is_highspeed, bool is_dma) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  // Check Synopsys ID register, failed if controller clock/power is not enabled
  TU_ASSERT(check_dwc2(dwc2));

  // disable global interrupt
  dwc2->gahbcfg &= ~GAHBCFG_GINT;

  if (is_highspeed) {
    phy_hs_init(dwc2);
  } else {
    phy_fs_init(dwc2);
  }

  /* Set HS/FS Timeout Calibration to 7 (max available value).
   * The number of PHY clocks that the application programs in
   * this field is added to the high/full speed interpacket timeout
   * duration in the core to account for any additional delays
   * introduced by the PHY. This can be required, because the delay
   * introduced by the PHY in generating the linestate condition
   * can vary from one PHY to another. */
  dwc2->gusbcfg |= (7ul << GUSBCFG_TOCAL_Pos);

  // Enable PHY clock TODO stop/gate clock when suspended mode
  dwc2->pcgcctl &= ~(PCGCCTL_STOPPCLK | PCGCCTL_GATEHCLK | PCGCCTL_PWRCLMP | PCGCCTL_RSTPDWNMODULE);

  dfifo_flush_tx(dwc2, 0x10); // all tx fifo
  dfifo_flush_rx(dwc2);

  // Clear pending and disable all interrupts
  dwc2->gintsts = 0xFFFFFFFFU;
  dwc2->gotgint = 0xFFFFFFFFU;
  dwc2->gintmsk = 0;

  TU_LOG(DWC2_COMMON_DEBUG, "DMA = %u\r\n", is_dma);

  if (is_dma) {
    // DMA seems to be only settable after a core reset, and not possible to switch on-the-fly
    dwc2->gahbcfg |= GAHBCFG_DMAEN | GAHBCFG_HBSTLEN_2;
  } else {
    dwc2->gintmsk |= GINTSTS_RXFLVL;
  }

  return true;
}

// void dwc2_core_handle_common_irq(uint8_t rhport, bool in_isr) {
//   (void) in_isr;
//   dwc2_regs_t * const dwc2 = DWC2_REG(rhport);
//   const uint32_t int_mask = dwc2->gintmsk;
//   const uint32_t int_status = dwc2->gintsts & int_mask;
//
//   // Device disconnect
//   if (int_status & GINTSTS_DISCINT) {
//     dwc2->gintsts = GINTSTS_DISCINT;
//   }
//
// }

//--------------------------------------------------------------------
// DFIFO
//--------------------------------------------------------------------
// Read a single data packet from receive DFIFO
void dfifo_read_packet(dwc2_regs_t* dwc2, uint8_t* dst, uint16_t len) {
  const volatile uint32_t* rx_fifo = dwc2->fifo[0];

  // Reading full available 32 bit words from fifo
  uint16_t word_count = len >> 2;
  while (word_count--) {
    tu_unaligned_write32(dst, *rx_fifo);
    dst += 4;
  }

  // Read the remaining 1-3 bytes from fifo
  const uint8_t bytes_rem = len & 0x03;
  if (bytes_rem != 0) {
    const uint32_t tmp = *rx_fifo;
    dst[0] = tu_u32_byte0(tmp);
    if (bytes_rem > 1) {
      dst[1] = tu_u32_byte1(tmp);
    }
    if (bytes_rem > 2) {
      dst[2] = tu_u32_byte2(tmp);
    }
  }
}

// Write a single data packet to DFIFO
void dfifo_write_packet(dwc2_regs_t* dwc2, uint8_t fifo_num, const uint8_t* src, uint16_t len) {
  volatile uint32_t* tx_fifo = dwc2->fifo[fifo_num];

  // Pushing full available 32 bit words to fifo
  uint16_t word_count = len >> 2;
  while (word_count--) {
    *tx_fifo = tu_unaligned_read32(src);
    src += 4;
  }

  // Write the remaining 1-3 bytes into fifo
  const uint8_t bytes_rem = len & 0x03;
  if (bytes_rem) {
    uint32_t tmp_word = src[0];
    if (bytes_rem > 1) {
      tmp_word |= (src[1] << 8);
    }
    if (bytes_rem > 2) {
      tmp_word |= (src[2] << 16);
    }

    *tx_fifo = tmp_word;
  }
}

#endif
