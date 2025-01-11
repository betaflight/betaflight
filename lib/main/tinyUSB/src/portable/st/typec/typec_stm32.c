/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Ha Thach (tinyusb.org)
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
 */

#include "tusb_option.h"
#include "typec/tcd.h"

#if CFG_TUC_ENABLED && defined(TUP_USBIP_TYPEC_STM32)

#include "common/tusb_common.h"

#if CFG_TUSB_MCU == OPT_MCU_STM32G4
  #include "stm32g4xx.h"
  #include "stm32g4xx_ll_dma.h" // for UCPD REQID
#else
  #error "Unsupported STM32 family"
#endif

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

enum {
  IMR_ATTACHED = UCPD_IMR_TXMSGDISCIE | UCPD_IMR_TXMSGSENTIE | UCPD_IMR_TXMSGABTIE | UCPD_IMR_TXUNDIE |
                 UCPD_IMR_RXHRSTDETIE | UCPD_IMR_RXOVRIE | UCPD_IMR_RXMSGENDIE | UCPD_IMR_RXORDDETIE |
                 UCPD_IMR_HRSTDISCIE | UCPD_IMR_HRSTSENTIE | UCPD_IMR_FRSEVTIE
};

#define PHY_SYNC1 0x18u
#define PHY_SYNC2 0x11u
#define PHY_SYNC3 0x06u
#define PHY_RST1  0x07u
#define PHY_RST2  0x19u
#define PHY_EOP   0x0Du

#define PHY_ORDERED_SET_SOP          (PHY_SYNC1 | (PHY_SYNC1<<5u) | (PHY_SYNC1<<10u) | (PHY_SYNC2<<15u)) // SOP Ordered set coding
#define PHY_ORDERED_SET_SOP_P        (PHY_SYNC1 | (PHY_SYNC1<<5u) | (PHY_SYNC3<<10u) | (PHY_SYNC3<<15u)) // SOP' Ordered set coding
#define PHY_ORDERED_SET_SOP_PP       (PHY_SYNC1 | (PHY_SYNC3<<5u) | (PHY_SYNC1<<10u) | (PHY_SYNC3<<15u)) // SOP'' Ordered set coding
#define PHY_ORDERED_SET_HARD_RESET   (PHY_RST1  | (PHY_RST1<<5u)  | (PHY_RST1<<10u)  | (PHY_RST2<<15u )) // Hard Reset Ordered set coding
#define PHY_ORDERED_SET_CABLE_RESET  (PHY_RST1  | (PHY_SYNC1<<5u) | (PHY_RST1<<10u)  | (PHY_SYNC3<<15u)) // Cable Reset Ordered set coding
#define PHY_ORDERED_SET_SOP_P_DEBUG  (PHY_SYNC1 | (PHY_RST2<<5u)  | (PHY_RST2<<10u)  | (PHY_SYNC3<<15u)) // SOP' Debug Ordered set coding
#define PHY_ORDERED_SET_SOP_PP_DEBUG (PHY_SYNC1 | (PHY_RST2<<5u)  | (PHY_SYNC3<<10u) | (PHY_SYNC2<<15u)) // SOP'' Debug Ordered set coding


static uint8_t const* _rx_buf;
static uint8_t const* _tx_pending_buf;
static uint16_t _tx_pending_bytes;
static uint16_t _tx_xferring_bytes;

static pd_header_t _good_crc = {
    .msg_type   = PD_CTRL_GOOD_CRC,
    .data_role  = 0, // UFP
    .specs_rev  = PD_REV_20,
    .power_role = 0, // Sink
    .msg_id     = 0,
    .n_data_obj = 0,
    .extended   = 0
};

// address of DMA channel rx, tx for each port
#define CFG_TUC_STM32_DMA  { { DMA1_Channel1_BASE, DMA1_Channel2_BASE } }

//--------------------------------------------------------------------+
// DMA
//--------------------------------------------------------------------+

static const uint32_t _dma_addr_arr[TUP_TYPEC_RHPORTS_NUM][2] = CFG_TUC_STM32_DMA;

TU_ATTR_ALWAYS_INLINE static inline uint32_t dma_get_addr(uint8_t rhport, bool is_rx) {
  return _dma_addr_arr[rhport][is_rx ? 0 : 1];
}

static void dma_init(uint8_t rhport, bool is_rx) {
  uint32_t dma_addr = dma_get_addr(rhport, is_rx);
  DMA_Channel_TypeDef* dma_ch = (DMA_Channel_TypeDef*) dma_addr;
  uint32_t req_id;

  if (is_rx) {
    // Peripheral -> Memory, Memory inc, 8-bit, High priority
    dma_ch->CCR = DMA_CCR_MINC | DMA_CCR_PL_1;
    dma_ch->CPAR = (uint32_t) &UCPD1->RXDR;

    req_id = LL_DMAMUX_REQ_UCPD1_RX;
  } else {
    // Memory -> Peripheral, Memory inc, 8-bit, High priority
    dma_ch->CCR = DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_DIR;
    dma_ch->CPAR = (uint32_t) &UCPD1->TXDR;

    req_id = LL_DMAMUX_REQ_UCPD1_TX;
  }

  // find and set up mux channel TODO support mcu with multiple DMAMUXs
  enum {
    CH_DIFF = DMA1_Channel2_BASE - DMA1_Channel1_BASE
  };
  uint32_t mux_ch_num;

  #ifdef DMA2_BASE
  if (dma_addr > DMA2_BASE) {
    mux_ch_num = 8 * ((dma_addr - DMA2_Channel1_BASE) / CH_DIFF);
  } else
  #endif
  {
    mux_ch_num = (dma_addr - DMA1_Channel1_BASE) / CH_DIFF;
  }

  DMAMUX_Channel_TypeDef* mux_ch = DMAMUX1_Channel0 + mux_ch_num;

  uint32_t mux_ccr = mux_ch->CCR & ~(DMAMUX_CxCR_DMAREQ_ID);
  mux_ccr |= req_id;
  mux_ch->CCR = mux_ccr;
}

TU_ATTR_ALWAYS_INLINE static inline void dma_start(uint8_t rhport, bool is_rx, void const* buf, uint16_t len) {
  DMA_Channel_TypeDef* dma_ch = (DMA_Channel_TypeDef*) dma_get_addr(rhport, is_rx);

  dma_ch->CMAR = (uint32_t) buf;
  dma_ch->CNDTR = len;
  dma_ch->CCR |= DMA_CCR_EN;
}

TU_ATTR_ALWAYS_INLINE static inline void dma_stop(uint8_t rhport, bool is_rx) {
  DMA_Channel_TypeDef* dma_ch = (DMA_Channel_TypeDef*) dma_get_addr(rhport, is_rx);
  dma_ch->CCR &= ~DMA_CCR_EN;
}

TU_ATTR_ALWAYS_INLINE static inline bool dma_enabled(uint8_t rhport, bool is_rx) {
  DMA_Channel_TypeDef* dma_ch = (DMA_Channel_TypeDef*) dma_get_addr(rhport, is_rx);
  return dma_ch->CCR & DMA_CCR_EN;
}

TU_ATTR_ALWAYS_INLINE static inline void dma_tx_start(uint8_t rhport, void const* buf, uint16_t len) {
  UCPD1->TX_ORDSET = PHY_ORDERED_SET_SOP;
  UCPD1->TX_PAYSZ = len;
  dma_start(rhport, false, buf, len);
  UCPD1->CR |= UCPD_CR_TXSEND;
}

TU_ATTR_ALWAYS_INLINE static inline void dma_tx_stop(uint8_t rhport) {
  dma_stop(rhport, false);
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

bool tcd_init(uint8_t rhport, uint32_t port_type) {
  (void) rhport;

  // Init DMA for RX, TX
  dma_init(rhport, true);
  dma_init(rhport, false);

  // Initialization phase: CFG1, detect all SOPs
  UCPD1->CFG1 = (0x0d << UCPD_CFG1_HBITCLKDIV_Pos) | (0x10 << UCPD_CFG1_IFRGAP_Pos) | (0x07 << UCPD_CFG1_TRANSWIN_Pos) |
                (0x01 << UCPD_CFG1_PSC_UCPDCLK_Pos) | (0x1f << UCPD_CFG1_RXORDSETEN_Pos);
  UCPD1->CFG1 |= UCPD_CFG1_UCPDEN;

  // General programming sequence (with UCPD configured then enabled)
  if (port_type == TUSB_TYPEC_PORT_SNK) {
    // Set analog mode enable both CC Phy
    UCPD1->CR = (0x01 << UCPD_CR_ANAMODE_Pos) | (UCPD_CR_CCENABLE_0 | UCPD_CR_CCENABLE_1);

    // Read Voltage State on CC1 & CC2 fore initial state
    uint32_t v_cc[2];
    (void) v_cc;
    v_cc[0] = (UCPD1->SR >> UCPD_SR_TYPEC_VSTATE_CC1_Pos) & 0x03;
    v_cc[1] = (UCPD1->SR >> UCPD_SR_TYPEC_VSTATE_CC2_Pos) & 0x03;
    TU_LOG1("Initial VState CC1 = %lu, CC2 = %lu\r\n", v_cc[0], v_cc[1]);

    // Enable CC1 & CC2 Interrupt
    UCPD1->IMR = UCPD_IMR_TYPECEVT1IE | UCPD_IMR_TYPECEVT2IE;
  }

  // Disable dead battery in PWR's CR3
  PWR->CR3 |= PWR_CR3_UCPD_DBDIS;

  return true;
}

// Enable interrupt
void tcd_int_enable (uint8_t rhport) {
  (void) rhport;
  NVIC_EnableIRQ(UCPD1_IRQn);
}

// Disable interrupt
void tcd_int_disable(uint8_t rhport) {
  (void) rhport;
  NVIC_DisableIRQ(UCPD1_IRQn);
}

bool tcd_msg_receive(uint8_t rhport, uint8_t* buffer, uint16_t total_bytes) {
  _rx_buf = buffer;
  dma_start(rhport, true, buffer, total_bytes);
  return true;
}

bool tcd_msg_send(uint8_t rhport, uint8_t const* buffer, uint16_t total_bytes) {
  (void) rhport;

  if (dma_enabled(rhport, false)) {
    // DMA is busy, probably sending GoodCRC, save as pending TX
    _tx_pending_buf = buffer;
    _tx_pending_bytes = total_bytes;
  }else {
    // DMA is free, start sending
    _tx_pending_buf = NULL;
    _tx_pending_bytes = 0;

    _tx_xferring_bytes = total_bytes;
    dma_tx_start(rhport, buffer, total_bytes);
  }

  return true;
}

void tcd_int_handler(uint8_t rhport) {
  (void) rhport;

  uint32_t sr = UCPD1->SR;
  sr &= UCPD1->IMR;

  if (sr & (UCPD_SR_TYPECEVT1 | UCPD_SR_TYPECEVT2)) {
    uint32_t v_cc[2];
    v_cc[0] = (UCPD1->SR >> UCPD_SR_TYPEC_VSTATE_CC1_Pos) & 0x03;
    v_cc[1] = (UCPD1->SR >> UCPD_SR_TYPEC_VSTATE_CC2_Pos) & 0x03;

    TU_LOG3("VState CC1 = %lu, CC2 = %lu\r\n", v_cc[0], v_cc[1]);

    uint32_t cr = UCPD1->CR;

    // TODO only support SNK for now, required highest voltage for now
    // Enable PHY on active CC and disable Rd on other CC
    // FIXME somehow CC2 is vstate is not correct, always 1 even not attached.
    // on DPOW1 board, it is connected to PA10 (USBPD_DBCC2), we probably miss something.
    if ((sr & UCPD_SR_TYPECEVT1) && (v_cc[0] == 3)) {
      TU_LOG3("Attach CC1\r\n");
      cr &= ~(UCPD_CR_PHYCCSEL | UCPD_CR_CCENABLE);
      cr |= UCPD_CR_PHYRXEN | UCPD_CR_CCENABLE_0;
    } else if ((sr & UCPD_SR_TYPECEVT2) && (v_cc[1] == 3)) {
      TU_LOG3("Attach CC2\r\n");
      cr &= ~UCPD_CR_CCENABLE;
      cr |= (UCPD_CR_PHYCCSEL | UCPD_CR_PHYRXEN | UCPD_CR_CCENABLE_1);
    } else {
      TU_LOG3("Detach\r\n");
      cr &= ~UCPD_CR_PHYRXEN;
      cr |= UCPD_CR_CCENABLE_0 | UCPD_CR_CCENABLE_1;
    }

    if (cr & UCPD_CR_PHYRXEN) {
      // Attached
      UCPD1->IMR |= IMR_ATTACHED;
      UCPD1->CFG1 |= UCPD_CFG1_RXDMAEN | UCPD_CFG1_TXDMAEN;
    }else {
      // Detached
      UCPD1->CFG1 &= ~(UCPD_CFG1_RXDMAEN | UCPD_CFG1_TXDMAEN);
      UCPD1->IMR &= ~IMR_ATTACHED;
    }

    // notify stack
    tcd_event_cc_changed(rhport, v_cc[0], v_cc[1], true);

    UCPD1->CR = cr;

    // ack
    UCPD1->ICR = UCPD_ICR_TYPECEVT1CF | UCPD_ICR_TYPECEVT2CF;
  }

  //------------- RX -------------//
  if (sr & UCPD_SR_RXORDDET) {
    // SOP: Start of Packet.
    TU_LOG3("SOP\r\n");
    // UCPD1->RX_ORDSET & UCPD_RX_ORDSET_RXORDSET_Msk;

    // ack
    UCPD1->ICR = UCPD_ICR_RXORDDETCF;
  }

  // Received full message
  if (sr & UCPD_SR_RXMSGEND) {
    TU_LOG3("RX MSG END\r\n");

    // stop TX
    dma_stop(rhport, true);

    uint8_t result;

    if (!(sr & UCPD_SR_RXERR)) {
      // response with good crc
      // TODO move this to usbc stack
      if (_rx_buf) {
        _good_crc.msg_id = ((pd_header_t const *) _rx_buf)->msg_id;
        dma_tx_start(rhport, &_good_crc, 2);
      }

      result = XFER_RESULT_SUCCESS;
    }else {
      // CRC failed
      result = XFER_RESULT_FAILED;
    }

    // notify stack
    tcd_event_rx_complete(rhport, UCPD1->RX_PAYSZ, result, true);

    // ack
    UCPD1->ICR = UCPD_ICR_RXMSGENDCF;
  }

  if (sr & UCPD_SR_RXOVR) {
    TU_LOG3("RXOVR\r\n");
    // ack
    UCPD1->ICR = UCPD_ICR_RXOVRCF;
  }

  //------------- TX -------------//
  // All tx events: complete and error
  if (sr & (UCPD_SR_TXMSGSENT | (UCPD_SR_TXMSGDISC | UCPD_SR_TXMSGABT | UCPD_SR_TXUND))) {
    // force TX stop
    dma_tx_stop(rhport);

    uint16_t const xferred_bytes = _tx_xferring_bytes - UCPD1->TX_PAYSZ;
    uint8_t result;

    if ( sr & UCPD_SR_TXMSGSENT ) {
      TU_LOG3("TX MSG SENT\r\n");
      result = XFER_RESULT_SUCCESS;
      // ack
      UCPD1->ICR = UCPD_ICR_TXMSGSENTCF;
    }else {
      TU_LOG3("TX Error\r\n");
      result = XFER_RESULT_FAILED;
      // ack
      UCPD1->ICR = UCPD_SR_TXMSGDISC | UCPD_SR_TXMSGABT | UCPD_SR_TXUND;
    }

    // start pending TX if any
    if (_tx_pending_buf && _tx_pending_bytes ) {
      // Start the pending TX
      dma_tx_start(rhport, _tx_pending_buf, _tx_pending_bytes);

      // clear pending
      _tx_pending_buf = NULL;
      _tx_pending_bytes = 0;
    }

    // notify stack
    tcd_event_tx_complete(rhport, xferred_bytes, result, true);
  }
}

#endif
