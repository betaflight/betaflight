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

/******************************************************************************
*
* Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*
*  Redistributions in binary form must reproduce the above copyright
*  notice, this list of conditions and the following disclaimer in the
*  documentation and/or other materials provided with the
*  distribution.
*
*  Neither the name of Texas Instruments Incorporated nor the names of
*  its contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/

#ifndef TUSB_MUSB_TYPE_H_
#define TUSB_MUSB_TYPE_H_

#include "stdint.h"

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef __IO
  #define __IO volatile
#endif

#ifndef __I
  #define __I  volatile const
#endif

#ifndef __O
  #define __O  volatile
#endif

#ifndef __R
  #define __R  volatile const
#endif

typedef struct TU_ATTR_PACKED {
  __IO uint16_t maxp;          // 0x00, 0x04: MAXP
  __IO uint8_t  csrl;          // 0x02, 0x06: CSRL
  __IO uint8_t  csrh;          // 0x03, 0x07: CSRH
}musb_ep_maxp_csr_t;

// 0: TX (device IN, host OUT)
// 1: RX (device OUT, host IN)
typedef struct TU_ATTR_PACKED {
  union {
    struct {
      __IO uint16_t tx_maxp;       // 0x00: TXMAXP
      union {
        __IO uint8_t csr0l;        // 0x02: CSR0
        __IO uint8_t tx_csrl;      // 0x02: TX CSRL
      };
      union {
        __IO uint8_t csr0h;        // 0x03: CSR0H
        __IO uint8_t tx_csrh;      // 0x03: TX CSRH
      };

      __IO uint16_t rx_maxp;       // 0x04: RX MAXP
      __IO uint8_t  rx_csrl;       // 0x06: RX CSRL
      __IO uint8_t  rx_csrh;       // 0x07: RX CSRH
    };

    musb_ep_maxp_csr_t maxp_csr[2];
  };

  union {
    __IO uint16_t count0;      // 0x08: COUNT0
    __IO uint16_t rx_count;    // 0x08: RX COUNT
  };
  union {
    __IO uint8_t type0;        // 0x0A: TYPE0 (host only)
    __IO uint8_t tx_type;      // 0x0A: TX TYPE
  };
  __IO uint8_t tx_interval;    // 0x0B: TX INTERVAL
  __IO uint8_t rx_type;        // 0x0C: RX TYPE
  __IO uint8_t rx_interval;    // 0x0D: RX INTERVAL
  __IO uint8_t reserved_0x0e;  // 0x0E: Reserved
  union {
    __IO uint8_t config_data0; // 0x0F: CONFIG DATA
    struct {
      __IO uint8_t utmi_data_width : 1; // [0] UTMI Data Width
      __IO uint8_t softconn_en     : 1; // [1] Soft Connect Enable
      __IO uint8_t dynamic_fifo    : 1; // [2] Dynamic FIFO Sizing
      __IO uint8_t hb_tx_en        : 1; // [3] High Bandwidth TX ISO Enable
      __IO uint8_t hb_rx_en        : 1; // [4] High Bandwidth RX ISO Enable
      __IO uint8_t big_endian      : 1; // [5] Big Endian
      __IO uint8_t mp_tx_en        : 1; // [6] Auto splitting BULK TX Enable
      __IO uint8_t mp_rx_en        : 1; // [7] Auto amalgamation BULK RX Enable
    } config_data0_bit;

    __IO uint8_t fifo_size;    // 0x0F: FIFO_SIZE
    struct {
      __IO uint8_t tx : 4;  // [3:0] TX FIFO Size
      __IO uint8_t rx : 4;  // [7:4] RX FIFO Size
    }fifo_size_bit;
  };
} musb_ep_csr_t;

TU_VERIFY_STATIC(sizeof(musb_ep_csr_t) == 16, "size is not correct");

typedef struct TU_ATTR_PACKED {
  //------------- Common -------------//
  __IO uint8_t  faddr;             // 0x00: FADDR
  union {
    __IO uint8_t  power;             // 0x01: POWER
    struct {
      __IO uint8_t suspend_mode_en : 1; // [0] SUSPEND Mode Enable
      __IO uint8_t suspend_mode    : 1; // [1] SUSPEND Mode
      __IO uint8_t resume_mode     : 1; // [2] RESUME
      __IO uint8_t reset           : 1; // [3] RESET
      __IO uint8_t highspeed_mode  : 1; // [4] High Speed Mode
      __IO uint8_t highspeed_en    : 1; // [5] High Speed Enable
      __IO uint8_t soft_conn       : 1; // [6] Soft Connect/Disconnect
      __IO uint8_t iso_update      : 1; // [7] Isochronous Update
    } power_bit;
  };

  union {
    struct {
      __IO uint16_t intr_tx;           // 0x02: INTR_TX
      __IO uint16_t intr_rx;           // 0x04: INTR_RX
    };

    __IO uint16_t intr_ep[2];         // 0x02-0x05: INTR_EP0-1
  };

  union {
    struct {
      __IO uint16_t intr_txen;         // 0x06: INTR_TXEN
      __IO uint16_t intr_rxen;         // 0x08: INTR_RXEN
    };

    __IO uint16_t intren_ep[2];       // 0x06-0x09: INTREN_EP0-1
  };

  __IO uint8_t  intr_usb;          // 0x0A: INTRUSB
  __IO uint8_t  intr_usben;        // 0x0B: INTRUSBEN

  __IO uint16_t frame;             // 0x0C: FRAME
  __IO uint8_t  index;             // 0x0E: INDEX
  __IO uint8_t  testmode;          // 0x0F: TESTMODE

  //------------- Endpoint CSR (indexed) -------------//
  musb_ep_csr_t indexed_csr;       // 0x10-0x1F: Indexed CSR 0-15

  //------------- FIFOs -------------//
  __IO uint32_t fifo[16];          // 0x20-0x5C: FIFO 0-15

  // Common (2)
  __IO uint8_t  devctl;            // 0x60: DEVCTL
  __IO uint8_t  misc;              // 0x61: MISC

  //------------- Dynammic FIFO (indexed) -------------//
  union {
    struct {
      __IO uint8_t  txfifo_sz;         // 0x62: TXFIFO_SZ
      __IO uint8_t  rxfifo_sz;         // 0x63: RXFIFO_SZ
    };
    __IO uint8_t fifo_size[2];
  };

  union {
    struct {
      __IO uint16_t txfifo_addr;       // 0x64: TXFIFO_ADDR
      __IO uint16_t rxfifo_addr;       // 0x66: RXFIFO_ADDR
    };
    __IO uint16_t fifo_addr[2];
  };

  //------------- Additional Control and Configuration -------------//
  union {
    __O  uint32_t vcontrol;        // 0x68: PHY VCONTROL
    __IO uint32_t vstatus;         // 0x68: PHY VSTATUS
  };
  union {
    __IO uint16_t hwvers;            // 0x6C: HWVERS
    struct {
      __IO uint16_t minor : 10;     // [9:0] Minor
      __IO uint16_t major : 5;      // [14:10] Major
      __IO uint16_t rc    : 1;      // [15] Release Candidate
    } hwvers_bit;
  };
  __R  uint16_t rsv_0x6e_0x77[5];  // 0x6E-0x77: Reserved

   //------------- Additional Configuration -------------//
  union {
    __IO uint8_t  epinfo;            // 0x78: EPINFO
    struct {
      __IO uint8_t tx_ep_num : 4;    // [3:0] TX Endpoints
      __IO uint8_t rx_ep_num : 4;    // [7:4] RX Endpoints
    } epinfo_bit;
  };
  union {
    __IO uint8_t  raminfo;           // 0x79: RAMINFO
    struct {
      __IO uint8_t ram_bits    : 4;  // [3:0] RAM Address Bus Width
      __IO uint8_t dma_channel : 4;  // [7:4] DMA Channels
    }raminfo_bit;
  };
  union {
    __IO uint8_t  link_info;       // 0x7A: LINK_INFO
    __IO uint8_t  adi_softreset;   // 0x7A: AnalogDevice SOFTRESET
  };
  __IO uint8_t  vplen;             // 0x7B: VPLEN
  __IO uint8_t  hs_eof1;           // 0x7C: HS_EOF1
  __IO uint8_t  fs_eof1;           // 0x7D: FS_EOF1
  __IO uint8_t  ls_eof1;           // 0x7E: LS_EOF1
  __IO uint8_t  soft_rst;          // 0x7F: SOFT_RST

  //------------- Target Endpoints (multipoint option) -------------//
  __IO uint16_t ctuch;             // 0x80: CTUCH
  __IO uint16_t cthsrtn;           // 0x82: CTHSRTN
  __R  uint32_t rsv_0x84_0xff[31]; // 0x84-0xFF: Reserved

  //------------- Non-Indexed Endpoint CSRs -------------//
  // TI tm4c can access this directly, but should use indexed_csr for portability
  musb_ep_csr_t abs_csr[16];        // 0x100-0x1FF: EP0-15 CSR

  //------------- DMA -------------//
  __IO uint8_t dma_intr;             // 0x200: DMA_INTR
  __R  uint8_t rsv_0x201_0x203[3];   // 0x201-0x203: Reserved
  struct {
    __IO uint16_t cntl;             // 0x204: DMA_CNTL
    __IO uint16_t rsv_0x206;        // 0x206: Reserved
    __IO uint32_t addr;             // 0x208: DMA_ADDR
    __IO uint32_t count;            // 0x20C: DMA_COUNT
    __IO uint32_t rsv_0x210;        // 0x210: Reserved
  }dma[8];
  __R  uint32_t rsv_0x284_0x2FF[31]; // 0x284-0x2FF: Reserved

  //------------- Extended -------------//
  __R uint32_t rsv_0x300;           // 0x300: Reserved
  struct {
    __IO uint16_t count;            // 0x304: REQ_PACKET_COUNT
    __R  uint16_t rsv_0x306;        // 0x306: Reserved
  }req_packet[15];

  __IO uint16_t rx_doulbe_packet_disable; // 0x340: RX_DOUBLE_PACKET_DISABLE
  __IO uint16_t tx_double_packet_disable; // 0x342: TX_DOUBLE_PACKET_DISABLE

  __IO uint16_t chirp_timeout;            // 0x344: CHIRP_TIMEOUT
  __IO uint16_t hs_to_utm;                // 0x346: HS_TO_UTM delay
  __IO uint16_t hs_timeout_adder;         // 0x348: HS_TIMEOUT_ADDER

  __R uint8_t rsv_34A_34f[6];             // 0x34A-0x34F: Reserved
} musb_regs_t;

TU_VERIFY_STATIC(sizeof(musb_regs_t) == 0x350, "size is not correct");

//--------------------------------------------------------------------+
// Helper
//--------------------------------------------------------------------+
TU_ATTR_ALWAYS_INLINE static inline musb_ep_csr_t* get_ep_csr(musb_regs_t* musb_regs, unsigned epnum) {
  musb_regs->index = epnum;
  return &musb_regs->indexed_csr;
}

//--------------------------------------------------------------------+
// Register Bit Field
//--------------------------------------------------------------------+

// 0x01: Power
#define MUSB_POWER_ISOUP         0x0080  // Isochronous Update
#define MUSB_POWER_SOFTCONN      0x0040  // Soft Connect/Disconnect
#define MUSB_POWER_HSENAB        0x0020  // High Speed Enable
#define MUSB_POWER_HSMODE        0x0010  // High Speed Enable
#define MUSB_POWER_RESET         0x0008  // RESET Signaling
#define MUSB_POWER_RESUME        0x0004  // RESUME Signaling
#define MUSB_POWER_SUSPEND       0x0002  // SUSPEND Mode
#define MUSB_POWER_PWRDNPHY      0x0001  // Power Down PHY

// Interrupt TX/RX Status and Enable: each bit is for an endpoint

// 0x6c: HWVERS
#define MUSB_HWVERS_RC_SHIFT    15
#define MUSB_HWVERS_RC_MASK     0x8000
#define MUSB_HWVERS_MAJOR_SHIFT 10
#define MUSB_HWVERS_MAJOR_MASK  0x7C00
#define MUSB_HWVERS_MINOR_SHIFT 0
#define MUSB_HWVERS_MINOR_MASK  0x03FF

// 0x12, 0x16: TX/RX CSRL
#define MUSB_CSRL_PACKET_READY(_rx)      (1u << 0)
#define MUSB_CSRL_FLUSH_FIFO(_rx)        (1u << ((_rx) ? 4 : 3))
#define MUSB_CSRL_SEND_STALL(_rx)        (1u << ((_rx) ? 5 : 4))
#define MUSB_CSRL_STALLED(_rx)           (1u << ((_rx) ? 6 : 5))
#define MUSB_CSRL_CLEAR_DATA_TOGGLE(_rx) (1u << ((_rx) ? 7 : 6))

// 0x13, 0x17: TX/RX CSRH
#define MUSB_CSRH_DISABLE_DOUBLE_PACKET(_rx) (1u << 1)
#define MUSB_CSRH_TX_MODE                    (1u << 5) // 1 = TX, 0 = RX. only relevant for SHARED FIFO
#define MUSB_CSRH_ISO                        (1u << 6)

// 0x62, 0x63: TXFIFO_SZ, RXFIFO_SZ
#define MUSB_FIFOSZ_DOUBLE_PACKET            (1u << 4)


//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_IS register.
//
//*****************************************************************************
#define MUSB_IS_VBUSERR          0x0080  // VBUS Error (OTG only)
#define MUSB_IS_SESREQ           0x0040  // SESSION REQUEST (OTG only)
#define MUSB_IS_DISCON           0x0020  // Session Disconnect (OTG only)
#define MUSB_IS_CONN             0x0010  // Session Connect
#define MUSB_IS_SOF              0x0008  // Start of Frame
#define MUSB_IS_BABBLE           0x0004  // Babble Detected
#define MUSB_IS_RESET            0x0004  // RESET Signaling Detected
#define MUSB_IS_RESUME           0x0002  // RESUME Signaling Detected
#define MUSB_IS_SUSPEND          0x0001  // SUSPEND Signaling Detected

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_IE register.
//
//*****************************************************************************
#define MUSB_IE_VBUSERR          0x0080  // Enable VBUS Error Interrupt (OTG only)
#define MUSB_IE_SESREQ           0x0040  // Enable Session Request (OTG only)
#define MUSB_IE_DISCON           0x0020  // Enable Disconnect Interrupt
#define MUSB_IE_CONN             0x0010  // Enable Connect Interrupt
#define MUSB_IE_SOF              0x0008  // Enable Start-of-Frame Interrupt
#define MUSB_IE_BABBLE           0x0004  // Enable Babble Interrupt
#define MUSB_IE_RESET            0x0004  // Enable RESET Interrupt
#define MUSB_IE_RESUME           0x0002  // Enable RESUME Interrupt
#define MUSB_IE_SUSPND           0x0001  // Enable SUSPEND Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_FRAME register.
//
//*****************************************************************************
#define MUSB_FRAME_M             0x07FF  // Frame Number
#define MUSB_FRAME_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_TEST register.
//
//*****************************************************************************
#define MUSB_TEST_FORCEH         0x0080  // Force Host Mode
#define MUSB_TEST_FIFOACC        0x0040  // FIFO Access
#define MUSB_TEST_FORCEFS        0x0020  // Force Full-Speed Mode
#define MUSB_TEST_FORCEHS        0x0010  // Force High-Speed Mode
#define MUSB_TEST_TESTPKT        0x0008  // Test Packet Mode Enable
#define MUSB_TEST_TESTK          0x0004  // Test_K Mode Enable
#define MUSB_TEST_TESTJ          0x0002  // Test_J Mode Enable
#define MUSB_TEST_TESTSE0NAK     0x0001  // Test_SE0_NAK Test Mode Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_DEVCTL register.
//
//*****************************************************************************
#define MUSB_DEVCTL_DEV          0x0080  // Device Mode (OTG only)
#define MUSB_DEVCTL_FSDEV        0x0040  // Full-Speed Device Detected
#define MUSB_DEVCTL_LSDEV        0x0020  // Low-Speed Device Detected
#define MUSB_DEVCTL_VBUS_M       0x0018  // VBUS Level (OTG only)
#define MUSB_DEVCTL_VBUS_NONE    0x0000  // Below SessionEnd
#define MUSB_DEVCTL_VBUS_SEND    0x0008  // Above SessionEnd, below AValid
#define MUSB_DEVCTL_VBUS_AVALID  0x0010  // Above AValid, below VBUSValid
#define MUSB_DEVCTL_VBUS_VALID   0x0018  // Above VBUSValid
#define MUSB_DEVCTL_HOST         0x0004  // Host Mode
#define MUSB_DEVCTL_HOSTREQ      0x0002  // Host Request (OTG only)
#define MUSB_DEVCTL_SESSION      0x0001  // Session Start/End (OTG only)

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_CCONF register.
//
//*****************************************************************************
#define MUSB_CCONF_TXEDMA        0x0002  // TX Early DMA Enable
#define MUSB_CCONF_RXEDMA        0x0001  // TX Early DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_ULPIVBUSCTL
// register.
//
//*****************************************************************************
#define MUSB_ULPIVBUSCTL_USEEXTVBUSIND  0x0002  // Use External VBUS Indicator
#define MUSB_ULPIVBUSCTL_USEEXTVBUS     0x0001  // Use External VBUS

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_ULPIREGDATA
// register.
//
//*****************************************************************************
#define MUSB_ULPIREGDATA_REGDATA_M      0x00FF  // Register Data
#define MUSB_ULPIREGDATA_REGDATA_S      0
//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_ULPIREGADDR
// register.
//
//*****************************************************************************
#define MUSB_ULPIREGADDR_ADDR_M  0x00FF  // Register Address
#define MUSB_ULPIREGADDR_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_ULPIREGCTL
// register.
//
//*****************************************************************************
#define MUSB_ULPIREGCTL_RDWR     0x0004  // Read/Write Control
#define MUSB_ULPIREGCTL_REGCMPLT 0x0002  // Register Access Complete
#define MUSB_ULPIREGCTL_REGACC   0x0001  // Initiate Register Access

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_EPINFO register.
//
//*****************************************************************************
#define MUSB_EPINFO_RXEP_M       0x00F0  // RX Endpoints
#define MUSB_EPINFO_TXEP_M       0x000F  // TX Endpoints
#define MUSB_EPINFO_RXEP_S       4
#define MUSB_EPINFO_TXEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_RAMINFO register.
//
//*****************************************************************************
#define MUSB_RAMINFO_DMACHAN_M   0x00F0  // DMA Channels
#define MUSB_RAMINFO_RAMBITS_M   0x000F  // RAM Address Bus Width
#define MUSB_RAMINFO_DMACHAN_S   4
#define MUSB_RAMINFO_RAMBITS_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_CONTIM register.
//
//*****************************************************************************
#define MUSB_CONTIM_WTCON_M      0x00F0  // Connect Wait
#define MUSB_CONTIM_WTID_M       0x000F  // Wait ID
#define MUSB_CONTIM_WTCON_S      4
#define MUSB_CONTIM_WTID_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_VPLEN register.
//
//*****************************************************************************
#define MUSB_VPLEN_VPLEN_M       0x00FF  // VBUS Pulse Length
#define MUSB_VPLEN_VPLEN_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_HSEOF register.
//
//*****************************************************************************
#define MUSB_HSEOF_HSEOFG_M      0x00FF  // HIgh-Speed End-of-Frame Gap
#define MUSB_HSEOF_HSEOFG_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_FSEOF register.
//
//*****************************************************************************
#define MUSB_FSEOF_FSEOFG_M      0x00FF  // Full-Speed End-of-Frame Gap
#define MUSB_FSEOF_FSEOFG_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_LSEOF register.
//
//*****************************************************************************
#define MUSB_LSEOF_LSEOFG_M      0x00FF  // Low-Speed End-of-Frame Gap
#define MUSB_LSEOF_LSEOFG_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_CSRL0 register.
//
//*****************************************************************************
#define MUSB_CSRL0_NAKTO         0x0080  // NAK Timeout
#define MUSB_CSRL0_SETENDC       0x0080  // Setup End Clear
#define MUSB_CSRL0_STATUS        0x0040  // STATUS Packet
#define MUSB_CSRL0_RXRDYC        0x0040  // RXRDY Clear
#define MUSB_CSRL0_REQPKT        0x0020  // Request Packet
#define MUSB_CSRL0_STALL         0x0020  // Send Stall
#define MUSB_CSRL0_SETEND        0x0010  // Setup End
#define MUSB_CSRL0_ERROR         0x0010  // Error
#define MUSB_CSRL0_DATAEND       0x0008  // Data End
#define MUSB_CSRL0_SETUP         0x0008  // Setup Packet
#define MUSB_CSRL0_STALLED       0x0004  // Endpoint Stalled
#define MUSB_CSRL0_TXRDY         0x0002  // Transmit Packet Ready
#define MUSB_CSRL0_RXRDY         0x0001  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_CSRH0 register.
//
//*****************************************************************************
#define MUSB_CSRH0_DISPING       0x0008  // PING Disable
#define MUSB_CSRH0_DTWE          0x0004  // Data Toggle Write Enable
#define MUSB_CSRH0_DT            0x0002  // Data Toggle
#define MUSB_CSRH0_FLUSH         0x0001  // Flush FIFO

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_TYPE0 register.
//
//*****************************************************************************
#define MUSB_TYPE0_SPEED_M       0x00C0  // Operating Speed
#define MUSB_TYPE0_SPEED_HIGH    0x0040  // High
#define MUSB_TYPE0_SPEED_FULL    0x0080  // Full
#define MUSB_TYPE0_SPEED_LOW     0x00C0  // Low

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_NAKLMT register.
//
//*****************************************************************************
#define MUSB_NAKLMT_NAKLMT_M     0x001F  // EP0 NAK Limit
#define MUSB_NAKLMT_NAKLMT_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_TXCSRL1 register.
//
//*****************************************************************************
#define MUSB_TXCSRL1_NAKTO       0x0080  // NAK Timeout
#define MUSB_TXCSRL1_CLRDT       0x0040  // Clear Data Toggle
#define MUSB_TXCSRL1_STALLED     0x0020  // Endpoint Stalled
#define MUSB_TXCSRL1_STALL       0x0010  // Send STALL
#define MUSB_TXCSRL1_SETUP       0x0010  // Setup Packet
#define MUSB_TXCSRL1_FLUSH       0x0008  // Flush FIFO
#define MUSB_TXCSRL1_ERROR       0x0004  // Error
#define MUSB_TXCSRL1_UNDRN       0x0004  // Underrun
#define MUSB_TXCSRL1_FIFONE      0x0002  // FIFO Not Empty
#define MUSB_TXCSRL1_TXRDY       0x0001  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_TXCSRH1 register.
//
//*****************************************************************************
#define MUSB_TXCSRH1_AUTOSET     0x0080  // Auto Set
#define MUSB_TXCSRH1_ISO         0x0040  // Isochronous Transfers
#define MUSB_TXCSRH1_MODE        0x0020  // Mode
#define MUSB_TXCSRH1_DMAEN       0x0010  // DMA Request Enable
#define MUSB_TXCSRH1_FDT         0x0008  // Force Data Toggle
#define MUSB_TXCSRH1_DMAMOD      0x0004  // DMA Request Mode
#define MUSB_TXCSRH1_DTWE        0x0002  // Data Toggle Write Enable
#define MUSB_TXCSRH1_DT          0x0001  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_RXCSRL1 register.
//
//*****************************************************************************
#define MUSB_RXCSRL1_CLRDT       0x0080  // Clear Data Toggle
#define MUSB_RXCSRL1_STALLED     0x0040  // Endpoint Stalled
#define MUSB_RXCSRL1_STALL       0x0020  // Send STALL
#define MUSB_RXCSRL1_REQPKT      0x0020  // Request Packet
#define MUSB_RXCSRL1_FLUSH       0x0010  // Flush FIFO
#define MUSB_RXCSRL1_DATAERR     0x0008  // Data Error
#define MUSB_RXCSRL1_NAKTO       0x0008  // NAK Timeout
#define MUSB_RXCSRL1_OVER        0x0004  // Overrun
#define MUSB_RXCSRL1_ERROR       0x0004  // Error
#define MUSB_RXCSRL1_FULL        0x0002  // FIFO Full
#define MUSB_RXCSRL1_RXRDY       0x0001  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_RXCSRH1 register.
//
//*****************************************************************************
#define MUSB_RXCSRH1_AUTOCL      0x0080  // Auto Clear
#define MUSB_RXCSRH1_AUTORQ      0x0040  // Auto Request
#define MUSB_RXCSRH1_ISO         0x0040  // Isochronous Transfers
#define MUSB_RXCSRH1_DMAEN       0x0020  // DMA Request Enable
#define MUSB_RXCSRH1_DISNYET     0x0010  // Disable NYET
#define MUSB_RXCSRH1_PIDERR      0x0010  // PID Error
#define MUSB_RXCSRH1_DMAMOD      0x0008  // DMA Request Mode
#define MUSB_RXCSRH1_DTWE        0x0004  // Data Toggle Write Enable
#define MUSB_RXCSRH1_DT          0x0002  // Data Toggle
#define MUSB_RXCSRH1_INCOMPRX    0x0001  // Incomplete RX Transmission Status

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_TXTYPE1 register.
//
//*****************************************************************************
#define MUSB_TXTYPE1_SPEED_M     0x00C0  // Operating Speed
#define MUSB_TXTYPE1_SPEED_DFLT  0x0000  // Default
#define MUSB_TXTYPE1_SPEED_HIGH  0x0040  // High
#define MUSB_TXTYPE1_SPEED_FULL  0x0080  // Full
#define MUSB_TXTYPE1_SPEED_LOW   0x00C0  // Low
#define MUSB_TXTYPE1_PROTO_M     0x0030  // Protocol
#define MUSB_TXTYPE1_PROTO_CTRL  0x0000  // Control
#define MUSB_TXTYPE1_PROTO_ISOC  0x0010  // Isochronous
#define MUSB_TXTYPE1_PROTO_BULK  0x0020  // Bulk
#define MUSB_TXTYPE1_PROTO_INT   0x0030  // Interrupt
#define MUSB_TXTYPE1_TEP_M       0x000F  // Target Endpoint Number
#define MUSB_TXTYPE1_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_TXINTERVAL1
// register.
//
//*****************************************************************************
#define MUSB_TXINTERVAL1_NAKLMT_M 0x00FF  // NAK Limit
#define MUSB_TXINTERVAL1_TXPOLL_M 0x00FF  // TX Polling
#define MUSB_TXINTERVAL1_TXPOLL_S 0
#define MUSB_TXINTERVAL1_NAKLMT_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_RXTYPE1 register.
//
//*****************************************************************************
#define MUSB_RXTYPE1_SPEED_M     0x00C0  // Operating Speed
#define MUSB_RXTYPE1_SPEED_DFLT  0x0000  // Default
#define MUSB_RXTYPE1_SPEED_HIGH  0x0040  // High
#define MUSB_RXTYPE1_SPEED_FULL  0x0080  // Full
#define MUSB_RXTYPE1_SPEED_LOW   0x00C0  // Low
#define MUSB_RXTYPE1_PROTO_M     0x0030  // Protocol
#define MUSB_RXTYPE1_PROTO_CTRL  0x0000  // Control
#define MUSB_RXTYPE1_PROTO_ISOC  0x0010  // Isochronous
#define MUSB_RXTYPE1_PROTO_BULK  0x0020  // Bulk
#define MUSB_RXTYPE1_PROTO_INT   0x0030  // Interrupt
#define MUSB_RXTYPE1_TEP_M       0x000F  // Target Endpoint Number
#define MUSB_RXTYPE1_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_RXINTERVAL1
// register.
//
//*****************************************************************************
#define MUSB_RXINTERVAL1_TXPOLL_M 0x00FF  // RX Polling
#define MUSB_RXINTERVAL1_NAKLMT_M 0x00FF  // NAK Limit
#define MUSB_RXINTERVAL1_TXPOLL_S 0
#define MUSB_RXINTERVAL1_NAKLMT_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_DMACTL0 register.
//
//*****************************************************************************
#define MUSB_DMACTL0_BRSTM_M     0x0600  // Burst Mode
#define MUSB_DMACTL0_BRSTM_ANY   0x0000  // Bursts of unspecified length
#define MUSB_DMACTL0_BRSTM_INC4  0x0200  // INCR4 or unspecified length
#define MUSB_DMACTL0_BRSTM_INC8  0x0400  // INCR8, INCR4 or unspecified
                                            // length
#define MUSB_DMACTL0_BRSTM_INC16 0x0600  // INCR16, INCR8, INCR4 or
                                            // unspecified length
#define MUSB_DMACTL0_ERR         0x0100  // Bus Error Bit
#define MUSB_DMACTL0_EP_M        0x00F0  // Endpoint number
#define MUSB_DMACTL0_IE          0x0008  // DMA Interrupt Enable
#define MUSB_DMACTL0_MODE        0x0004  // DMA Transfer Mode
#define MUSB_DMACTL0_DIR         0x0002  // DMA Direction
#define MUSB_DMACTL0_ENABLE      0x0001  // DMA Transfer Enable
#define MUSB_DMACTL0_EP_S        4

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_DMAADDR0 register.
//
//*****************************************************************************
#define MUSB_DMAADDR0_ADDR_M     0xFFFFFFFC  // DMA Address
#define MUSB_DMAADDR0_ADDR_S     2

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_DMACOUNT0
// register.
//
//*****************************************************************************
#define MUSB_DMACOUNT0_COUNT_M   0xFFFFFFFC  // DMA Count
#define MUSB_DMACOUNT0_COUNT_S   2

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_CTO register.
//
//*****************************************************************************
#define MUSB_CTO_CCTV_M          0xFFFF  // Configurable Chirp Timeout Value
#define MUSB_CTO_CCTV_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_HHSRTN register.
//
//*****************************************************************************
#define MUSB_HHSRTN_HHSRTN_M     0xFFFF  // HIgh Speed to UTM Operating
                                            // Delay
#define MUSB_HHSRTN_HHSRTN_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the MUSB_O_HSBT register.
//
//*****************************************************************************
#define MUSB_HSBT_HSBT_M         0x000F  // High Speed Timeout Adder
#define MUSB_HSBT_HSBT_S         0

#ifdef __cplusplus
 }
#endif

#endif
