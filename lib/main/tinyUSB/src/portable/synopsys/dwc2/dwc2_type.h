/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024, hathach (tinyusb.org)
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
 */
/** <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  */

#ifndef TUSB_DWC2_TYPES_H_
#define TUSB_DWC2_TYPES_H_

#include "stdint.h"

#ifdef __cplusplus
 extern "C" {
#endif

// Controller
typedef struct
{
  uintptr_t reg_base;
  uint32_t  irqnum;
  uint8_t   ep_count;
  uint8_t   ep_in_count;
  uint32_t  ep_fifo_size;
}dwc2_controller_t;

// DWC OTG HW Release versions
#define DWC2_CORE_REV_2_71a   0x4f54271a
#define DWC2_CORE_REV_2_72a   0x4f54272a
#define DWC2_CORE_REV_2_80a   0x4f54280a
#define DWC2_CORE_REV_2_90a   0x4f54290a
#define DWC2_CORE_REV_2_91a   0x4f54291a
#define DWC2_CORE_REV_2_92a   0x4f54292a
#define DWC2_CORE_REV_2_94a   0x4f54294a
#define DWC2_CORE_REV_3_00a   0x4f54300a
#define DWC2_CORE_REV_3_10a   0x4f54310a
#define DWC2_CORE_REV_4_00a   0x4f54400a
#define DWC2_CORE_REV_4_11a   0x4f54411a
#define DWC2_CORE_REV_4_20a   0x4f54420a
#define DWC2_FS_IOT_REV_1_00a 0x5531100a
#define DWC2_HS_IOT_REV_1_00a 0x5532100a
#define DWC2_CORE_REV_MASK    0x0000ffff

// DWC OTG HW Core ID
#define DWC2_OTG_ID           0x4f540000
#define DWC2_FS_IOT_ID        0x55310000
#define DWC2_HS_IOT_ID        0x55320000

#if 0
// HS PHY
typedef struct
{
  volatile uint32_t HS_PHYC_PLL;         // 000h This register is used to control the PLL of the HS PHY.
  volatile uint32_t Reserved04;          // 004h Reserved
  volatile uint32_t Reserved08;          // 008h Reserved
  volatile uint32_t HS_PHYC_TUNE;        // 00Ch This register is used to control the tuning interface of the High Speed PHY.
  volatile uint32_t Reserved10;          // 010h Reserved
  volatile uint32_t Reserved14;          // 014h Reserved
  volatile uint32_t HS_PHYC_LDO;         // 018h This register is used to control the regulator (LDO).
} HS_PHYC_GlobalTypeDef;
#endif

enum {
  GOTGCTL_OTG_VERSION_1_3 = 0,
  GOTGCTL_OTG_VERSION_2_0 = 1,
};

enum {
  GHWCFG2_OPMODE_HNP_SRP         = 0,
  GHWCFG2_OPMODE_SRP             = 1,
  GHWCFG2_OPMODE_NON_HNP_NON_SRP = 2,
  GHWCFG2_OPMODE_SRP_DEVICE      = 3,
  GHWCFFG2_OPMODE_NON_OTG_DEVICE = 4,
  GHWCFG2_OPMODE_SRP_HOST        = 5,
  GHWCFG2_OPMODE_NON_OTG_HOST    = 6,
};
enum {
  GHWCFG2_ARCH_SLAVE_ONLY   = 0,
  GHWCFG2_ARCH_EXTERNAL_DMA = 1,
  GHWCFG2_ARCH_INTERNAL_DMA = 2,
};

enum {
  GHWCFG2_HSPHY_NOT_SUPPORTED = 0,
  GHWCFG2_HSPHY_UTMI          = 1, // internal PHY (mostly)
  GHWCFG2_HSPHY_ULPI          = 2, // external PHY (mostly)
  GHWCFG2_HSPHY_UTMI_ULPI     = 3, // both

};

enum {
  GHWCFG2_FSPHY_NOT_SUPPORTED = 0,
  GHWCFG2_FSPHY_DEDICATED     = 1, // have dedicated FS PHY
  GHWCFG2_FSPHY_UTMI          = 2, // shared with UTMI+
  GHWCFG2_FSPHY_ULPI          = 3, // shared with ULPI
};

enum {
  GHWCFFG4_PHY_DATA_WIDTH_8    = 0,
  GHWCFFG4_PHY_DATA_WIDTH_16   = 1,
  GHWCFFG4_PHY_DATA_WIDTH_8_16 = 2, // software selectable
};

enum {
  HPRT_SPEED_HIGH = 0,
  HPRT_SPEED_FULL = 1,
  HPRT_SPEED_LOW  = 2
};

enum {
  GINTSTS_CMODE_DEVICE = 0,
  GINTSTS_CMODE_HOST   = 1,
};

enum {
  HCTSIZ_PID_DATA0 = 0, // 00b
  HCTSIZ_PID_DATA2 = 1, // 01b
  HCTSIZ_PID_DATA1 = 2, // 10b
  HCTSIZ_PID_SETUP = 3, // 11b
};
enum {
  HCTSIZ_PID_MDATA = 3,
};

enum {
  GRXSTS_PKTSTS_GLOBAL_OUT_NAK      = 1,
  GRXSTS_PKTSTS_RX_DATA             = 2,
  GRXSTS_PKTSTS_RX_COMPLETE         = 3,
  GRXSTS_PKTSTS_SETUP_DONE          = 4,
  GRXSTS_PKTSTS_HOST_DATATOGGLE_ERR = 5,
  GRXSTS_PKTSTS_SETUP_RX            = 6,
  GRXSTS_PKTSTS_HOST_CHANNEL_HALTED = 7
};

// Same as TUSB_XFER_*
enum {
  HCCHAR_EPTYPE_CONTROL     = 0,
  HCCHAR_EPTYPE_ISOCHRONOUS = 1,
  HCCHAR_EPTYPE_BULK        = 2,
  HCCHAR_EPTYPE_INTERRUPT   = 3
};

enum {
  DCFG_SPEED_HIGH          = 0, // Highspeed with 30/60 Mhz
  DCFG_SPEED_FULL_30_60MHZ = 1, // Fullspeed with UTMI+/ULPI 30/60 Mhz
  DCFG_SPEED_LOW           = 2, // Lowspeed with FS PHY at 6 Mhz
  DCFG_SPEED_FULL_48MHZ    = 3, // Fullspeed with dedicated FS PHY at 48 Mhz
};

// Same as TUSB_XFER_*
enum {
  DEPCTL_EPTYPE_CONTROL     = 0,
  DEPCTL_EPTYPE_ISOCHRONOUS = 1,
  DEPCTL_EPTYPE_BULK        = 2,
  DEPCTL_EPTYPE_INTERRUPT   = 3
};

//--------------------------------------------------------------------
// Common Register Bitfield
//--------------------------------------------------------------------
typedef struct TU_ATTR_PACKED {
  uint32_t ses_req_scs           : 1; //  0 Session request success
  uint32_t ses_req               : 1; //  1 Session request
  uint32_t vbval_ov_en           : 1; //  2 VBUS valid override enable
  uint32_t vbval_ov_val          : 1; //  3 VBUS valid override value
  uint32_t aval_ov_en            : 1; //  4 A-peripheral session valid override enable
  uint32_t aval_ov_al            : 1; //  5 A-peripheral session valid override value
  uint32_t bval_ov_en            : 1; //  6 B-peripheral session valid override enable
  uint32_t bval_ov_val           : 1; //  7 B-peripheral session valid override value
  uint32_t hng_scs               : 1; //  8 Host negotiation success
  uint32_t hnp_rq                : 1; //  9 HNP (host negotiation protocol) request
  uint32_t host_set_hnp_en       : 1; // 10 Host set HNP enable
  uint32_t dev_hnp_en            : 1; // 11 Device HNP enabled
  uint32_t embedded_host_en      : 1; // 12 Embedded host enable
  uint32_t rsv13_14              : 2; // 13.14 Reserved
  uint32_t dbnc_filter_bypass    : 1; // 15 Debounce filter bypass
  uint32_t cid_status            : 1; // 16 Connector ID status
  uint32_t dbnc_done             : 1; // 17 Debounce done
  uint32_t ases_valid            : 1; // 18 A-session valid
  uint32_t bses_valid            : 1; // 19 B-session valid
  uint32_t otg_ver               : 1; // 20 OTG version 0: v1.3, 1: v2.0
  uint32_t current_mode          : 1; // 21 Current mode of operation. Only from v3.00a
  uint32_t mult_val_id_bc        : 5; // 22..26 Multi-valued input pin ID battery charger
  uint32_t chirp_en              : 1; // 27 Chirp detection enable
  uint32_t rsv28_30              : 3; // 28.30: Reserved
  uint32_t test_mode_corr_eusb2  : 1; // 31 Test mode control for eUSB2 PHY
} dwc2_gotgctl_t;
TU_VERIFY_STATIC(sizeof(dwc2_gotgctl_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t rsv0_1                : 2; //  0..1  Reserved
  uint32_t ses_end_det           : 1; //  2     Session end detected
  uint32_t rsv3_7                : 5; //  3..7  Reserved
  uint32_t srs_status_change     : 1; //  8     Session request success status change
  uint32_t hns_status_change     : 1; //  9     Host negotiation success status change
  uint32_t rsv10_16              : 7; // 10..16 Reserved
  uint32_t hng_det               : 1; // 17     Host negotiation detected
  uint32_t adev_timeout_change   : 1; // 18     A-device timeout change
  uint32_t dbnc_done             : 1; // 19     Debounce done
  uint32_t mult_val_lp_change    : 1; // 20     Multi-valued input pin change
  uint32_t rsv21_31              :11; // 21..31 Reserved
} dwc2_gotgint_t;
TU_VERIFY_STATIC(sizeof(dwc2_gotgint_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t gintmask              :  1; //  0 Global interrupt mask
  uint32_t hbst_len              :  4; //  1..4 Burst length/type
  uint32_t dma_en                :  1; //  5 DMA enable
  uint32_t rsv6                  :  1; //  6 Reserved
  uint32_t nptxf_empty_lvl       :  1; //  7 Non-periodic Tx FIFO empty level
  uint32_t ptxf_empty_lvl        :  1; //  8 Periodic Tx FIFO empty level
  uint32_t rsv9_20               : 12; //  9.20: Reserved
  uint32_t remote_mem_support    :  1; // 21 Remote memory support
  uint32_t notify_all_dma_write  :  1; // 22 Notify all DMA writes
  uint32_t ahb_single            :  1; // 23 AHB single
  uint32_t inv_desc_endian       :  1; // 24 Inverse descriptor endian
  uint32_t rsv25_31              :  7; // 25..31 Reserved
} dwc2_gahbcfg_t;
TU_VERIFY_STATIC(sizeof(dwc2_gahbcfg_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t timeout_cal             :  3; /* 0..2 Timeout calibration.
    The USB standard timeout value for high-speed operation is 736 to 816 (inclusive) bit times. The USB standard
    timeout value for full- speed operation is 16 to 18 (inclusive) bit times. The application must program this field
    based on the speed of enumeration. The number of bit times added per PHY clock are as follows:
    - High-speed: PHY clock One 30-MHz = 16 bit times, One 60-MHz = 8 bit times
    - Full-speed: PHY clock One 30-MHz = 0.4 bit times, One 60-MHz = 0.2 bit times, One 48-MHz = 0.25 bit times */
  uint32_t phy_if16                : 1; // 3 PHY interface. 0: 8 bits, 1: 16 bits
  uint32_t ulpi_utmi_sel           : 1; // 4 ULPI/UTMI select. 0: UTMI+, 1: ULPI
  uint32_t fs_intf_sel             : 1; // 5 Fullspeed serial interface select. 0: 6-pin, 1: 3-pin
  uint32_t phy_sel                 : 1; // 6 HS/FS PHY selection. 0: HS UTMI+ or ULPI, 1: FS serial transceiver
  uint32_t ddr_sel                 : 1; // 7 ULPI DDR select. 0: Single data rate 8-bit, 1: Double data rate 4-bit
  uint32_t srp_capable             : 1; // 8 SRP-capable
  uint32_t hnp_capable             : 1; // 9 HNP-capable
  uint32_t turnaround_time         : 4; // 10..13 Turnaround time. 9: 8-bit UTMI+, 5: 16-bit UTMI+
  uint32_t rsv14                   : 1; // 14 Reserved
  uint32_t phy_low_power_clk_sel   : 1; /* 15 PHY low-power clock select either 480-MHz or 48-MHz (low-power) PHY mode.
    In FS/LS modes, the PHY can usually operate on a 48-MHz clock to save power. This bit is valid only for UTMI+ PHYs.
    - 0: 480 Mhz internal PLL: the UTMI interface operates at either 60 MHz (8 bit) or 30 MHz (16-bit)
    - 1 48 Mhz external clock: the UTMI interface operates at 48 MHz in FS mode and at either 48 or 6 MHz in LS mode */
  uint32_t otg_i2c_sel             : 1; // 16 OTG I2C interface select. 0: UTMI-FS, 1: I2C for OTG signals
  uint32_t ulpi_fsls               : 1; /* 17 ULPI FS/LS select. 0: ULPI, 1: ULPI FS/LS.
                                             valid only when the FS serial transceiver is selected on the ULPI PHY. */
  uint32_t ulpi_auto_resume        : 1; // 18 ULPI Auto-resume
  uint32_t ulpi_clk_sus_m          : 1; // 19 ULPI Clock SuspendM
  uint32_t ulpi_ext_vbus_drv       : 1; // 20 ULPI External VBUS Drive
  uint32_t ulpi_int_vbus_indicator : 1; // 21 ULPI Internal VBUS Indicator
  uint32_t term_sel_dl_pulse       : 1; // 22 TermSel DLine pulsing
  uint32_t indicator_complement    : 1; // 23 Indicator complement
  uint32_t indicator_pass_through  : 1; // 24 Indicator pass through
  uint32_t ulpi_if_protect_disable : 1; // 25 ULPI interface protect disable
  uint32_t ic_usb_capable          : 1; // 26 IC_USB Capable
  uint32_t ic_usb_traf_ctl         : 1; // 27 IC_USB Traffic Control
  uint32_t tx_end_delay            : 1; // 28 TX end delay
  uint32_t force_host_mode         : 1; // 29 Force host mode
  uint32_t force_dev_mode          : 1; // 30 Force device mode
  uint32_t corrupt_tx_pkt          : 1; // 31 Corrupt Tx packet. 0: normal, 1: debug
} dwc2_gusbcfg_t;
TU_VERIFY_STATIC(sizeof(dwc2_gusbcfg_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t core_soft_rst          : 1; // 0 Core Soft Reset
  uint32_t piufs_soft_rst         : 1; // 1 PIU FS Dedicated Controller Soft Reset
  uint32_t frame_counter_rst      : 1; // 2 Frame Counter Reset (host)
  uint32_t intoken_q_flush        : 1; // 3 IN Token Queue Flush
  uint32_t rx_fifo_flush          : 1; // 4 RX FIFO Flush
  uint32_t tx_fifo_flush          : 1; // 5 TX FIFO Flush
  uint32_t tx_fifo_num            : 5; // 6..10 TX FIFO Number
  uint32_t rsv11_28               :18; // 11..28 Reserved
  uint32_t core_soft_rst_done     : 1; // 29 Core Soft Reset Done, from v4.20a
  uint32_t dma_req                : 1; // 30 DMA Request
  uint32_t ahb_idle               : 1; // 31 AHB Idle
} dwc2_grstctl_t;
TU_VERIFY_STATIC(sizeof(dwc2_grstctl_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t ep_ch_num             : 4; // 0..3 Endpoint/Channel Number
  uint32_t byte_count            :11; // 4..14 Byte Count
  uint32_t dpid                  : 2; // 15..16 Data PID
  uint32_t packet_status         : 4; // 17..20 Packet Status
  uint32_t frame_number          : 4; // 21..24 Frame Number
  uint32_t rsv25_31              : 7; // 25..31 Reserved
} dwc2_grxstsp_t;
TU_VERIFY_STATIC(sizeof(dwc2_grxstsp_t) == 4, "incorrect size");

// Hardware Configuration
typedef struct TU_ATTR_PACKED {
  uint32_t op_mode                : 3; // 0..2 HNP/SRP Host/Device/OTG mode
  uint32_t arch                   : 2; // 3..4 Slave/External/Internal DMA
  uint32_t single_point           : 1; // 5 0: support hub and split | 1: no hub, no split
  uint32_t hs_phy_type            : 2; // 6..7 0: not supported | 1: UTMI+ | 2: ULPI | 3: UTMI+ and ULPI
  uint32_t fs_phy_type            : 2; // 8..9 0: not supported | 1: dedicated | 2: UTMI+ | 3: ULPI
  uint32_t num_dev_ep             : 4; // 10..13 Number of device endpoints (excluding EP0)
  uint32_t num_host_ch            : 4; // 14..17 Number of host channel (excluding control)
  uint32_t period_channel_support : 1; // 18 Support Periodic OUT Host Channel
  uint32_t enable_dynamic_fifo    : 1; // 19 Dynamic FIFO Sizing Enabled
  uint32_t mul_proc_intrpt        : 1; // 20 Multi-Processor Interrupt enabled (OTG_MULTI_PROC_INTRPT)
  uint32_t reserved21             : 1; // 21 reserved
  uint32_t nptx_q_depth           : 2; // 22..23 Non-periodic request queue depth: 0 = 2.  1 = 4, 2 = 8
  uint32_t ptx_q_depth            : 2; // 24..25 Host periodic request queue depth: 0 = 2.  1 = 4, 2 = 8
  uint32_t token_q_depth          : 5; // 26..30 Device IN token sequence learning queue depth: 0-30
  uint32_t otg_enable_ic_usb      : 1; // 31 IC_USB mode specified for mode of operation
} dwc2_ghwcfg2_t;
TU_VERIFY_STATIC(sizeof(dwc2_ghwcfg2_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t xfer_size_width          : 4;  // 0..3 Transfer size counter in bits = 11 + n (max 19 bits)
  uint32_t packet_size_width        : 3;  // 4..6 Packet size counter in bits = 4 + n (max 10 bits)
  uint32_t otg_enable               : 1;  // 7 OTG capable
  uint32_t i2c_enable               : 1;  // 8 I2C interface is available
  uint32_t vendor_ctrl_itf          : 1;  // 9 Vendor control interface is available
  uint32_t optional_feature_removed : 1;  // 10 remove User ID, GPIO, SOF toggle & counter to save gate count
  uint32_t synch_reset              : 1;  // 11 0: async reset | 1: synch reset
  uint32_t otg_adp_support          : 1;  // 12 ADP logic is present along with HSOTG controller
  uint32_t otg_enable_hsic          : 1;  // 13 1: HSIC-capable with shared UTMI PHY interface | 0: non-HSIC
  uint32_t battery_charger_support  : 1;  // s14 upport battery charger
  uint32_t lpm_mode                 : 1;  // 15 LPM mode
  uint32_t dfifo_depth              : 16; // DFIFO depth - EP_LOC_CNT in terms of 32-bit words
}dwc2_ghwcfg3_t;
TU_VERIFY_STATIC(sizeof(dwc2_ghwcfg3_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t num_dev_period_in_ep     : 4; // 0..3 Number of Device Periodic IN Endpoints
  uint32_t partial_powerdown        : 1; // 4 Partial Power Down Enabled
  uint32_t ahb_freq_min             : 1; // 5 1: minimum of AHB frequency is less than 60 MHz
  uint32_t hibernation              : 1; // 6 Hibernation feature is enabled
  uint32_t extended_hibernation     : 1; // 7 Extended Hibernation feature is enabled
  uint32_t reserved8                : 1; // 8 Reserved
  uint32_t enhanced_lpm_support1    : 1; // 9 Enhanced LPM Support1
  uint32_t service_interval_flow    : 1; // 10 Service Interval flow is supported
  uint32_t ipg_isoc_support         : 1; // 11 Interpacket GAP ISO OUT worst-case is supported
  uint32_t acg_support              : 1; // 12 Active clock gating is supported
  uint32_t enhanced_lpm_support     : 1; // 13 Enhanced LPM Support
  uint32_t phy_data_width           : 2; // 14..15 0: 8 bits | 1: 16 bits | 2: 8/16 software selectable
  uint32_t ctrl_ep_num              : 4; // 16..19 Number of Device control endpoints in addition to EP0
  uint32_t iddg_filter              : 1; // 20 IDDG Filter Enabled
  uint32_t vbus_valid_filter        : 1; // 21 VBUS Valid Filter Enabled
  uint32_t a_valid_filter           : 1; // 22 A Valid Filter Enabled
  uint32_t b_valid_filter           : 1; // 23 B Valid Filter Enabled
  uint32_t session_end_filter       : 1; // 24 Session End Filter Enabled
  uint32_t dedicated_fifos          : 1; // 25 Dedicated tx fifo for device IN Endpoint
  uint32_t num_dev_in_eps           : 4; // 26..29 Number of Device IN Endpoints including EP0
  uint32_t dma_desc_enabled         : 1; // scatter/gather DMA configuration enabled
  uint32_t dma_desc_dynamic         : 1; // Dynamic scatter/gather DMA
}dwc2_ghwcfg4_t;
TU_VERIFY_STATIC(sizeof(dwc2_ghwcfg4_t) == 4, "incorrect size");

//--------------------------------------------------------------------
// Host Register Bitfield
//--------------------------------------------------------------------

typedef struct TU_ATTR_PACKED {
  uint32_t fifo_available      : 16; // 0..15 Number of words available in the Tx FIFO
  uint32_t req_queue_available :  8; // 16..23 Number of spaces available in the NPT transmit request queue for both IN and OU
  // 24..31 is top entry in the request queue that is currently being processed by the MAC
  uint32_t qtop_terminate      :  1; // 24 Last entry for selected channel
  uint32_t qtop_type           :  2; // 25..26 Token (0) In/Out (1) ZLP, (2) Ping/cspit, (3) Channel halt command
  uint32_t qtop_ch_num         :  4; // 27..30 Channel number
} dwc2_hnptxsts_t;
TU_VERIFY_STATIC(sizeof(dwc2_hnptxsts_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t fifo_available      : 16; // 0..15 Number of words available in the Tx FIFO
  uint32_t req_queue_available : 7; // 16..22 Number of spaces available in the PTX transmit request queue
  uint32_t qtop_terminate      : 1; // 23 Last entry for selected channel
  uint32_t qtop_last_period    : 1; // 24 Last entry for selected channel is a periodic entry
  uint32_t qtop_type           : 2; // 25..26 Token (0) In/Out (1) ZLP, (2) Ping/cspit, (3) Channel halt command
  uint32_t qtop_ch_num         : 4; // 27..30 Channel number
  uint32_t qtop_odd_frame      : 1; // 31 Send in odd frame
} dwc2_hptxsts_t;
TU_VERIFY_STATIC(sizeof(dwc2_hptxsts_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t conn_status         : 1; // 0 Port connect status
  uint32_t conn_detected       : 1; // 1 Port connect detected
  uint32_t enable              : 1; // 2 Port enable status
  uint32_t enable_change       : 1; // 3 Port enable change
  uint32_t over_current_active : 1; // 4 Port Over-current active
  uint32_t over_current_change : 1; // 5 Port Over-current change
  uint32_t resume              : 1; // 6 Port resume
  uint32_t suspend             : 1; // 7 Port suspend
  uint32_t reset               : 1; // 8 Port reset
  uint32_t rsv9                : 1; // 9 Reserved
  uint32_t line_status         : 2; // 10..11 Line status
  uint32_t power               : 1; // 12 Port power
  uint32_t test_control        : 4; // 13..16 Port Test control
  uint32_t speed               : 2; // 17..18 Port speed
  uint32_t rsv19_31            :13; // 19..31 Reserved
}dwc2_hprt_t;
TU_VERIFY_STATIC(sizeof(dwc2_hprt_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t ep_size         : 11; // 0..10 Maximum packet size
  uint32_t ep_num          :  4; // 11..14 Endpoint number
  uint32_t ep_dir          :  1; // 15 Endpoint direction
  uint32_t rsv16           :  1; // 16 Reserved
  uint32_t low_speed_dev   :  1; // 17 Low-speed device
  uint32_t ep_type         :  2; // 18..19 Endpoint type
  uint32_t err_multi_count :  2; // 20..21 Error (splitEn = 1) / Multi (SplitEn = 0)  count
  uint32_t dev_addr        :  7; // 22..28 Device address
  uint32_t odd_frame       :  1; // 29 Odd frame
  uint32_t disable         :  1; // 30 Channel disable
  uint32_t enable          :  1; // 31 Channel enable
} dwc2_channel_char_t;
TU_VERIFY_STATIC(sizeof(dwc2_channel_char_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t hub_port        :  7; // 0..6 Hub port number
  uint32_t hub_addr        :  7; // 7..13 Hub address
  uint32_t xact_pos        :  2; // 14..15 Transaction position
  uint32_t split_compl     :  1; // 16 Split completion
  uint32_t rsv17_30        : 14; // 17..30 Reserved
  uint32_t split_en        :  1; // 31 Split enable
} dwc2_channel_split_t;
TU_VERIFY_STATIC(sizeof(dwc2_channel_split_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t xfer_size      : 19; // 0..18 Transfer size in bytes
  uint32_t packet_count   : 10; // 19..28 Number of packets
  uint32_t pid            :  2; // 29..30 Packet ID
  uint32_t do_ping        :  1; // 31 Do PING
} dwc2_channel_tsize_t;
TU_VERIFY_STATIC(sizeof(dwc2_channel_tsize_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t num        : 16; // 0..15 Frame number
  uint32_t remainning : 16; // 16..31 Frame remaining
} dwc2_hfnum_t;
TU_VERIFY_STATIC(sizeof(dwc2_hfnum_t) == 4, "incorrect size");

// Host Channel
typedef struct {
  union {
    volatile uint32_t hcchar;           // 500 + 20*ch Host Channel Characteristics
    volatile dwc2_channel_char_t hcchar_bm;
  };
  union {
    volatile uint32_t hcsplt;           // 504 + 20*ch Host Channel Split Control
    volatile dwc2_channel_split_t hcsplt_bm;
  };
    volatile uint32_t hcint;            // 508 + 20*ch Host Channel Interrupt
    volatile uint32_t hcintmsk;         // 50C + 20*ch Host Channel Interrupt Mask
  union {
    volatile uint32_t hctsiz;           // 510 + 20*ch Host Channel Transfer Size
    volatile dwc2_channel_tsize_t hctsiz_bm;
  };
    volatile uint32_t hcdma;            // 514 + 20*ch Host Channel DMA Address
             uint32_t reserved518;      // 518 + 20*ch
    volatile uint32_t hcdmab;           // 51C + 20*ch Host Channel DMA Address
} dwc2_channel_t;

//--------------------------------------------------------------------
// Device Register Bitfield
//--------------------------------------------------------------------
typedef struct TU_ATTR_PACKED {
  uint32_t speed                    : 2; // 0..1 Speed
  uint32_t nzsts_out_handshake      : 1; // 2 Non-zero-length status OUT handshake
  uint32_t en_32khz_suspsend        : 1; // 3 Enable 32-kHz SUSPEND mode
  uint32_t address                  : 7; // 4..10 Device address
  uint32_t period_frame_interval    : 2; // 11..12 Periodic frame interval
  uint32_t en_out_nak               : 1; // 13 Enable Device OUT NAK
  uint32_t xcvr_delay               : 1; // 14 Transceiver delay
  uint32_t erratic_int_mask         : 1; // 15 Erratic interrupt mask
  uint32_t rsv16                    : 1; // 16 Reserved
  uint32_t ipg_iso_support          : 1; // 17 Interpacket gap ISO support
  uint32_t epin_mismatch_count      : 5; // 18..22 EP IN mismatch count
  uint32_t dma_desc                 : 1; // 23 Enable scatter/gatter DMA descriptor
  uint32_t period_schedule_interval : 2; // 24..25 Periodic schedule interval for scatter/gatter DMA
  uint32_t resume_valid             : 6; // 26..31 Resume valid period
} dwc2_dcfg_t;
TU_VERIFY_STATIC(sizeof(dwc2_dcfg_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t remote_wakeup_signal   : 1; // 0 Remote wakeup signal
  uint32_t soft_disconnet         : 1; // 1 Soft disconnect
  uint32_t gnp_in_nak_status      : 1; // 2 Global non-periodic NAK IN status
  uint32_t gout_nak_status        : 1; // 3 Global OUT NAK status
  uint32_t test_control           : 3; // 4..6 Test control
  uint32_t set_gnp_in_nak         : 1; // 7 Set global non-periodic IN NAK
  uint32_t clear_gnp_in_nak       : 1; // 8 Clear global non-periodic IN NAK
  uint32_t set_gout_nak           : 1; // 9 Set global OUT NAK
  uint32_t clear_gout_nak         : 1; // 10 Clear global OUT NAK
  uint32_t poweron_prog_done      : 1; // 11 Power-on programming done
  uint32_t rsv12                  : 1; // 12 Reserved
  uint32_t global_multi_count     : 2; // 13..14 Global multi-count
  uint32_t ignore_frame_number    : 1; // 15 Ignore frame number
  uint32_t nak_on_babble          : 1; // 16 NAK on babble
  uint32_t en_cont_on_bna         : 1; // 17 Enable continue on BNA
  uint32_t deep_sleep_besl_reject : 1; // 18 Deep sleep BESL reject
  uint32_t service_interval       : 1; // 19 Service interval for ISO IN endpoint
  uint32_t rsv20_31               :12; // 20..31 Reserved
} dwc2_dctl_t;
TU_VERIFY_STATIC(sizeof(dwc2_dctl_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t suspend_status : 1; // 0 Suspend status
  uint32_t enum_speed     : 2; // 1..2 Enumerated speed
  uint32_t erratic_err    : 1; // 3 Erratic error
  uint32_t rsv4_7         : 4; // 4..7 Reserved
  uint32_t frame_number   : 14; // 8..21  Frame/MicroFrame number
  uint32_t line_status    : 2; // 22..23 Line status
  uint32_t rsv24_31       : 8; // 24..31 Reserved
} dwc2_dsts_t;
TU_VERIFY_STATIC(sizeof(dwc2_dsts_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t xfer_complete       : 1; // 0 Transfer complete
  uint32_t disabled            : 1; // 1 Endpoint disabled
  uint32_t ahb_err             : 1; // 2 AHB error
  uint32_t timeout             : 1; // 3 Timeout
  uint32_t in_rx_txfe          : 1; // 4 IN token received when TxFIFO is empty
  uint32_t in_rx_ep_mismatch   : 1; // 5 IN token received with EP mismatch
  uint32_t in_ep_nak_effective : 1; // 6 IN endpoint NAK effective
  uint32_t txfifo_empty        : 1; // 7 TX FIFO empty
  uint32_t txfifo_underrun     : 1; // 8 Tx FIFO under run
  uint32_t bna                 : 1; // 9 Buffer not available
  uint32_t rsv10               : 1; // 10 Reserved
  uint32_t iso_packet_drop     : 1; // 11 Isochronous OUT packet drop status
  uint32_t babble_err          : 1; // 12 Babble error
  uint32_t nak                 : 1; // 13 NAK
  uint32_t nyet                : 1; // 14 NYET
  uint32_t rsv14_31            :17; // 15..31 Reserved
} dwc2_diepint_t;
TU_VERIFY_STATIC(sizeof(dwc2_diepint_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
        uint32_t mps                : 11; // 0..10 Maximum packet size, EP0 only use 2 bit
        uint32_t next_ep            : 4; // 11..14 Next endpoint number
        uint32_t active             : 1; // 15 Active
  const uint32_t dpid_iso_odd       : 1; // 16 DATA0/DATA1 for bulk/interrupt, odd frame for isochronous
  const uint32_t nak_status         : 1; // 17 NAK status
        uint32_t type               : 2; // 18..19 Endpoint type
        uint32_t rsv20              : 1; // 20 Reserved
        uint32_t stall              : 1; // 21 Stall
        uint32_t tx_fifo_num        : 4; // 22..25 Tx FIFO number (IN)
        uint32_t clear_nak          : 1; // 26 Clear NAK
        uint32_t set_nak            : 1; // 27 Set NAK
        uint32_t set_data0_iso_even : 1; // 28 Set DATA0 if bulk/interrupt, even frame for isochronous
        uint32_t set_data1_iso_odd  : 1; // 29 Set DATA1 if bulk/interrupt, odd frame for isochronous
        uint32_t disable            : 1; // 30 Disable
        uint32_t enable             : 1; // 31 Enable
} dwc2_depctl_t;
TU_VERIFY_STATIC(sizeof(dwc2_depctl_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t xfer_complete      : 1; // 0 Transfer complete
  uint32_t disabled           : 1; // 1 Endpoint disabled
  uint32_t ahb_err            : 1; // 2 AHB error
  uint32_t setup_phase_done   : 1; // 3 Setup phase done
  uint32_t out_rx_ep_disabled : 1; // 4 OUT token received when endpoint disabled
  uint32_t status_phase_rx    : 1; // 5 Status phase received
  uint32_t setup_b2b          : 1; // 6 Setup packet back-to-back
  uint32_t rsv7               : 1; // 7 Reserved
  uint32_t out_packet_err     : 1; // 8 OUT packet error
  uint32_t bna                : 1; // 9 Buffer not available
  uint32_t rsv10              : 1; // 10 Reserved
  uint32_t iso_packet_drop    : 1; // 11 Isochronous OUT packet drop status
  uint32_t babble_err         : 1; // 12 Babble error
  uint32_t nak                : 1; // 13 NAK
  uint32_t nyet               : 1; // 14 NYET
  uint32_t setup_packet_rx    : 1; // 15 Setup packet received (Buffer DMA Mode only)
  uint32_t rsv16_31           :16; // 16..31 Reserved
} dwc2_doepint_t;
TU_VERIFY_STATIC(sizeof(dwc2_doepint_t) == 4, "incorrect size");

typedef struct TU_ATTR_PACKED {
  uint32_t xfer_size    : 19; // 0..18 Transfer size in bytes
  uint32_t packet_count : 10; // 19..28 Number of packets
  uint32_t mc_pid       :  2; // 29..30 IN: Multi Count, OUT: PID
} dwc2_ep_tsize_t;
TU_VERIFY_STATIC(sizeof(dwc2_ep_tsize_t) == 4, "incorrect size");

// Device IN/OUT Endpoint
typedef struct {
  union {
    volatile uint32_t diepctl;
    volatile uint32_t doepctl;

    volatile uint32_t ctl;
    volatile dwc2_depctl_t ctl_bm;
  };
  uint32_t rsv04;
  union {
    volatile uint32_t intr;

    volatile uint32_t diepint;
    volatile dwc2_diepint_t diepint_bm;

    volatile uint32_t doepint;
    volatile dwc2_doepint_t doepint_bm;
  };
  uint32_t rsv0c;
  union {
    volatile uint32_t dieptsiz;
    volatile uint32_t doeptsiz;
    volatile uint32_t tsiz;
    volatile dwc2_ep_tsize_t tsiz_bm;
  };
  union {
    volatile uint32_t diepdma;
    volatile uint32_t doepdma;
  };
  volatile uint32_t dtxfsts;
  uint32_t rsv1c;
}dwc2_dep_t;

TU_VERIFY_STATIC(sizeof(dwc2_dep_t) == 0x20, "incorrect size");

//--------------------------------------------------------------------
// CSR Register Map
//--------------------------------------------------------------------
typedef struct {
    //------------- Core Global -------------//
  union {
    volatile uint32_t gotgctl;          // 000 OTG Control and Status
    volatile dwc2_gotgctl_t gotgctl_bm;
  };
  union {
    volatile uint32_t gotgint;          // 004 OTG Interrupt
    volatile dwc2_gotgint_t gotgint_bm;
  };
  union {
    volatile uint32_t gahbcfg;          // 008 AHB Configuration
    volatile dwc2_gahbcfg_t gahbcfg_bm;
  };
  union {
    volatile uint32_t gusbcfg;          // 00c USB Configuration
    volatile dwc2_gusbcfg_t gusbcfg_bm;
  };
  union {
    volatile uint32_t grstctl;          // 010 Reset
    volatile dwc2_grstctl_t grstctl_bm;
  };
    volatile uint32_t gintsts;          // 014 Interrupt
    volatile uint32_t gintmsk;          // 018 Interrupt Mask
    volatile uint32_t grxstsr;          // 01c Receive Status Debug Read
  union {
    volatile uint32_t grxstsp;          // 020 Receive Status Read/Pop
    volatile dwc2_grxstsp_t grxstsp_bm;
  };
    volatile uint32_t grxfsiz;          // 024 Receive FIFO Size
  union {
    volatile uint32_t dieptxf0;         // 028 EP0 Tx FIFO Size
    volatile uint32_t gnptxfsiz;        // 028 Non-periodic Transmit FIFO Size
  };
  union {
    volatile uint32_t hnptxsts;         // 02c Non-periodic Transmit FIFO/Queue Status
    volatile dwc2_hnptxsts_t hnptxsts_bm;
    volatile uint32_t gnptxsts;
  };
    volatile uint32_t gi2cctl;          // 030 I2C Address
    volatile uint32_t gpvndctl;         // 034 PHY Vendor Control
  union {
    volatile uint32_t ggpio;            // 038 General Purpose IO
    volatile uint32_t stm32_gccfg;      // 038 STM32 General Core Configuration
  };
    volatile uint32_t guid;             // 03C User (Application programmable) ID
    volatile uint32_t gsnpsid;          // 040 Synopsys ID + Release version
    volatile uint32_t ghwcfg1;          // 044 User Hardware Configuration1: endpoint dir (2 bit per ep)
  union {
    volatile uint32_t ghwcfg2;          // 048 User Hardware Configuration2
    volatile dwc2_ghwcfg2_t ghwcfg2_bm;
  };
  union {
    volatile uint32_t ghwcfg3;          // 04C User Hardware Configuration3
    volatile dwc2_ghwcfg3_t ghwcfg3_bm;
  };
  union {
    volatile uint32_t ghwcfg4;          // 050 User Hardware Configuration4
    volatile dwc2_ghwcfg4_t ghwcfg4_bm;
  };
    volatile uint32_t glpmcfg;          // 054 Core LPM Configuration
    volatile uint32_t gpwrdn;           // 058 Power Down
    volatile uint32_t gdfifocfg;        // 05C DFIFO Software Configuration
    volatile uint32_t gadpctl;          // 060 ADP Timer, Control and Status
             uint32_t reserved64[39];   // 064..0FF
    volatile uint32_t hptxfsiz;         // 100 Host Periodic Tx FIFO Size
    volatile uint32_t dieptxf[15];      // 104..13C Device Periodic Transmit FIFO Size
             uint32_t reserved140[176]; // 140..3FF

    //------------ Host -------------//
    volatile uint32_t hcfg;             // 400 Host Configuration
    volatile uint32_t hfir;             // 404 Host Frame Interval
  union {
    volatile uint32_t hfnum;            // 408 Host Frame Number / Frame Remaining
    volatile dwc2_hfnum_t hfnum_bm;
  };
             uint32_t reserved40c;      // 40C
  union {
    volatile uint32_t hptxsts;          // 410 Host Periodic TX FIFO / Queue Status
    volatile dwc2_hptxsts_t hptxsts_bm;
  };
    volatile uint32_t haint;            // 414 Host All Channels Interrupt
    volatile uint32_t haintmsk;         // 418 Host All Channels Interrupt Mask
    volatile uint32_t hflbaddr;         // 41C Host Frame List Base Address
             uint32_t reserved420[8];   // 420..43F
  union {
    volatile uint32_t hprt;             // 440 Host Port Control and Status
    volatile dwc2_hprt_t hprt_bm;
  };
             uint32_t reserved444[47];  // 444..4FF

    //------------- Host Channel -------------//
    dwc2_channel_t    channel[16];      // 500..6FF Host Channels 0-15
             uint32_t reserved700[64];  // 700..7FF

    //------------- Device -----------//
  union {
    volatile uint32_t dcfg;             // 800 Device Configuration
    volatile dwc2_dcfg_t dcfg_bm;
  };
  union {
    volatile uint32_t dctl;             // 804 Device Control
    volatile dwc2_dctl_t dctl_bm;
  };
  union {
    volatile uint32_t dsts;             // 808 Device Status (RO)
    volatile dwc2_dsts_t dsts_bm;
  };
             uint32_t reserved80c;      // 80C
  union {
    volatile uint32_t diepmsk;          // 810 Device IN Endpoint Interrupt Mask
    volatile dwc2_diepint_t diepmsk_bm;
  };
  union {
    volatile uint32_t doepmsk;          // 814 Device OUT Endpoint Interrupt Mask
    volatile dwc2_doepint_t doepmsk_bm;
  };
    volatile uint32_t daint;            // 818 Device All Endpoints Interrupt
    volatile uint32_t daintmsk;         // 81C Device All Endpoints Interrupt Mask
    volatile uint32_t dtknqr1;          // 820 Device IN token sequence learning queue read1
    volatile uint32_t dtknqr2;          // 824 Device IN token sequence learning queue read2
    volatile uint32_t dvbusdis;         // 828 Device VBUS Discharge Time
    volatile uint32_t dvbuspulse;       // 82C Device VBUS Pulsing Time
    volatile uint32_t dthrctl;          // 830 Device threshold Control
    volatile uint32_t diepempmsk;       // 834 Device IN Endpoint FIFO Empty Interrupt Mask

    // Device Each Endpoint (IN/OUT) Interrupt/Mask for generating dedicated EP interrupt line
    // require OTG_MULTI_PROC_INTRPT=1
    volatile uint32_t deachint;         // 838 Device Each Endpoint Interrupt
    volatile uint32_t deachmsk;         // 83C Device Each Endpoint Interrupt mask
    volatile uint32_t diepeachmsk[16];  // 840..87C Device Each IN Endpoint mask
    volatile uint32_t doepeachmsk[16];  // 880..8BF Device Each OUT Endpoint mask
             uint32_t reserved8c0[16];  // 8C0..8FF

    //------------- Device Endpoint -------------//
    union {
      dwc2_dep_t ep[2][16];            // 0: IN, 1 OUT
      struct {
        dwc2_dep_t  epin[16];         // 900..AFF  IN Endpoints
        dwc2_dep_t epout[16];        // B00..CFF  OUT Endpoints
      };
    };
    uint32_t reservedd00[64];  // D00..DFF

    //------------- Power Clock -------------//
    volatile uint32_t pcgcctl;          // E00 Power and Clock Gating Characteristic Control
    volatile uint32_t pcgcctl1;         // E04 Power and Clock Gating Characteristic Control 1
             uint32_t reservede08[126]; // E08..FFF

    //------------- FIFOs -------------//
    // Word-accessed only using first pointer since it auto shift
    volatile uint32_t fifo[16][0x400];  // 1000..FFFF Endpoint FIFO
} dwc2_regs_t;

TU_VERIFY_STATIC(offsetof(dwc2_regs_t, hcfg   ) == 0x0400, "incorrect size");
TU_VERIFY_STATIC(offsetof(dwc2_regs_t, channel) == 0x0500, "incorrect size");
TU_VERIFY_STATIC(offsetof(dwc2_regs_t, dcfg   ) == 0x0800, "incorrect size");
TU_VERIFY_STATIC(offsetof(dwc2_regs_t, epin   ) == 0x0900, "incorrect size");
TU_VERIFY_STATIC(offsetof(dwc2_regs_t, epout  ) == 0x0B00, "incorrect size");
TU_VERIFY_STATIC(offsetof(dwc2_regs_t, pcgcctl) == 0x0E00, "incorrect size");
TU_VERIFY_STATIC(offsetof(dwc2_regs_t, fifo   ) == 0x1000, "incorrect size");

//--------------------------------------------------------------------+
// Register Bit Definitions
//--------------------------------------------------------------------+

/********************  Bit definition for GOTGCTL register  ********************/
#define GOTGCTL_SRQSCS_Pos               (0U)
#define GOTGCTL_SRQSCS_Msk               (0x1UL << GOTGCTL_SRQSCS_Pos)            // 0x00000001
#define GOTGCTL_SRQSCS                   GOTGCTL_SRQSCS_Msk                       // Session request success
#define GOTGCTL_SRQ_Pos                  (1U)
#define GOTGCTL_SRQ_Msk                  (0x1UL << GOTGCTL_SRQ_Pos)               // 0x00000002
#define GOTGCTL_SRQ                      GOTGCTL_SRQ_Msk                          // Session request
#define GOTGCTL_VBVALOEN_Pos             (2U)
#define GOTGCTL_VBVALOEN_Msk             (0x1UL << GOTGCTL_VBVALOEN_Pos)          // 0x00000004
#define GOTGCTL_VBVALOEN                 GOTGCTL_VBVALOEN_Msk                     // VBUS valid override enable
#define GOTGCTL_VBVALOVAL_Pos            (3U)
#define GOTGCTL_VBVALOVAL_Msk            (0x1UL << GOTGCTL_VBVALOVAL_Pos)         // 0x00000008
#define GOTGCTL_VBVALOVAL                GOTGCTL_VBVALOVAL_Msk                    // VBUS valid override value
#define GOTGCTL_AVALOEN_Pos              (4U)
#define GOTGCTL_AVALOEN_Msk              (0x1UL << GOTGCTL_AVALOEN_Pos)           // 0x00000010
#define GOTGCTL_AVALOEN                  GOTGCTL_AVALOEN_Msk                      // A-peripheral session valid override enable
#define GOTGCTL_AVALOVAL_Pos             (5U)
#define GOTGCTL_AVALOVAL_Msk             (0x1UL << GOTGCTL_AVALOVAL_Pos)          // 0x00000020
#define GOTGCTL_AVALOVAL                 GOTGCTL_AVALOVAL_Msk                     // A-peripheral session valid override value
#define GOTGCTL_BVALOEN_Pos              (6U)
#define GOTGCTL_BVALOEN_Msk              (0x1UL << GOTGCTL_BVALOEN_Pos)           // 0x00000040
#define GOTGCTL_BVALOEN                  GOTGCTL_BVALOEN_Msk                      // B-peripheral session valid override enable
#define GOTGCTL_BVALOVAL_Pos             (7U)
#define GOTGCTL_BVALOVAL_Msk             (0x1UL << GOTGCTL_BVALOVAL_Pos)          // 0x00000080
#define GOTGCTL_BVALOVAL                 GOTGCTL_BVALOVAL_Msk                     // B-peripheral session valid override value
#define GOTGCTL_HNGSCS_Pos               (8U)
#define GOTGCTL_HNGSCS_Msk               (0x1UL << GOTGCTL_HNGSCS_Pos)            // 0x00000100
#define GOTGCTL_HNGSCS                   GOTGCTL_HNGSCS_Msk                       // Host set HNP enable
#define GOTGCTL_HNPRQ_Pos                (9U)
#define GOTGCTL_HNPRQ_Msk                (0x1UL << GOTGCTL_HNPRQ_Pos)             // 0x00000200
#define GOTGCTL_HNPRQ                    GOTGCTL_HNPRQ_Msk                        // HNP request
#define GOTGCTL_HSHNPEN_Pos              (10U)
#define GOTGCTL_HSHNPEN_Msk              (0x1UL << GOTGCTL_HSHNPEN_Pos)           // 0x00000400
#define GOTGCTL_HSHNPEN                  GOTGCTL_HSHNPEN_Msk                      // Host set HNP enable
#define GOTGCTL_DHNPEN_Pos               (11U)
#define GOTGCTL_DHNPEN_Msk               (0x1UL << GOTGCTL_DHNPEN_Pos)            // 0x00000800
#define GOTGCTL_DHNPEN                   GOTGCTL_DHNPEN_Msk                       // Device HNP enabled
#define GOTGCTL_EHEN_Pos                 (12U)
#define GOTGCTL_EHEN_Msk                 (0x1UL << GOTGCTL_EHEN_Pos)              // 0x00001000
#define GOTGCTL_EHEN                     GOTGCTL_EHEN_Msk                         // Embedded host enable
#define GOTGCTL_CIDSTS_Pos               (16U)
#define GOTGCTL_CIDSTS_Msk               (0x1UL << GOTGCTL_CIDSTS_Pos)            // 0x00010000
#define GOTGCTL_CIDSTS                   GOTGCTL_CIDSTS_Msk                       // Connector ID status
#define GOTGCTL_DBCT_Pos                 (17U)
#define GOTGCTL_DBCT_Msk                 (0x1UL << GOTGCTL_DBCT_Pos)              // 0x00020000
#define GOTGCTL_DBCT                     GOTGCTL_DBCT_Msk                         // Long/short debounce time
#define GOTGCTL_ASVLD_Pos                (18U)
#define GOTGCTL_ASVLD_Msk                (0x1UL << GOTGCTL_ASVLD_Pos)             // 0x00040000
#define GOTGCTL_ASVLD                    GOTGCTL_ASVLD_Msk                        // A-session valid
#define GOTGCTL_BSESVLD_Pos              (19U)
#define GOTGCTL_BSESVLD_Msk              (0x1UL << GOTGCTL_BSESVLD_Pos)           // 0x00080000
#define GOTGCTL_BSESVLD                  GOTGCTL_BSESVLD_Msk                      // B-session valid
#define GOTGCTL_OTGVER_Pos               (20U)
#define GOTGCTL_OTGVER_Msk               (0x1UL << GOTGCTL_OTGVER_Pos)            // 0x00100000
#define GOTGCTL_OTGVER                   GOTGCTL_OTGVER_Msk                       // OTG version

/********************  Bit definition for HCFG register  ********************/
#define HCFG_FSLS_PHYCLK_SEL_Pos         (0U)
#define HCFG_FSLS_PHYCLK_SEL_Msk         (0x3UL << HCFG_FSLS_PHYCLK_SEL_Pos)      // 0x00000003
#define HCFG_FSLS_PHYCLK_SEL             HCFG_FSLS_PHYCLK_SEL_Msk                 // FS/LS PHY clock select
#define HCFG_FSLS_PHYCLK_SEL_30_60MHZ    (0x0UL << HCFG_FSLS_PHYCLK_SEL_Pos)      // 0x00000000
#define HCFG_FSLS_PHYCLK_SEL_48MHZ       (0x1UL << HCFG_FSLS_PHYCLK_SEL_Pos)      // 0x00000001
#define HCFG_FSLS_PHYCLK_SEL_6MHZ        (0x2UL << HCFG_FSLS_PHYCLK_SEL_Pos)      // 0x00000002

#define HCFG_FSLS_ONLY_Pos               (2U)
#define HCFG_FSLS_ONLY_Msk               (0x1UL << HCFG_FSLS_ONLY_Pos)            // 0x00000004
#define HCFG_FSLS_ONLY                   HCFG_FSLS_ONLY_Msk                       // FS- and LS-only support

/********************  Bit definition for PCGCR register  ********************/
#define PCGCR_STPPCLK_Pos                (0U)
#define PCGCR_STPPCLK_Msk                (0x1UL << PCGCR_STPPCLK_Pos)             // 0x00000001
#define PCGCR_STPPCLK                    PCGCR_STPPCLK_Msk                        // Stop PHY clock
#define PCGCR_GATEHCLK_Pos               (1U)
#define PCGCR_GATEHCLK_Msk               (0x1UL << PCGCR_GATEHCLK_Pos)            // 0x00000002
#define PCGCR_GATEHCLK                   PCGCR_GATEHCLK_Msk                       // Gate HCLK
#define PCGCR_PHYSUSP_Pos                (4U)
#define PCGCR_PHYSUSP_Msk                (0x1UL << PCGCR_PHYSUSP_Pos)             // 0x00000010
#define PCGCR_PHYSUSP                    PCGCR_PHYSUSP_Msk                        // PHY suspended

/********************  Bit definition for GOTGINT register  ********************/
#define GOTGINT_SEDET_Pos                (2U)
#define GOTGINT_SEDET_Msk                (0x1UL << GOTGINT_SEDET_Pos)             // 0x00000004
#define GOTGINT_SEDET                    GOTGINT_SEDET_Msk                        // Session end detected
#define GOTGINT_SRSSCHG_Pos              (8U)
#define GOTGINT_SRSSCHG_Msk              (0x1UL << GOTGINT_SRSSCHG_Pos)           // 0x00000100
#define GOTGINT_SRSSCHG                  GOTGINT_SRSSCHG_Msk                      // Session request success status change
#define GOTGINT_HNSSCHG_Pos              (9U)
#define GOTGINT_HNSSCHG_Msk              (0x1UL << GOTGINT_HNSSCHG_Pos)           // 0x00000200
#define GOTGINT_HNSSCHG                  GOTGINT_HNSSCHG_Msk                      // Host negotiation success status change
#define GOTGINT_HNGDET_Pos               (17U)
#define GOTGINT_HNGDET_Msk               (0x1UL << GOTGINT_HNGDET_Pos)            // 0x00020000
#define GOTGINT_HNGDET                   GOTGINT_HNGDET_Msk                       // Host negotiation detected
#define GOTGINT_ADTOCHG_Pos              (18U)
#define GOTGINT_ADTOCHG_Msk              (0x1UL << GOTGINT_ADTOCHG_Pos)           // 0x00040000
#define GOTGINT_ADTOCHG                  GOTGINT_ADTOCHG_Msk                      // A-device timeout change
#define GOTGINT_DBCDNE_Pos               (19U)
#define GOTGINT_DBCDNE_Msk               (0x1UL << GOTGINT_DBCDNE_Pos)            // 0x00080000
#define GOTGINT_DBCDNE                   GOTGINT_DBCDNE_Msk                       // Debounce done
#define GOTGINT_IDCHNG_Pos               (20U)
#define GOTGINT_IDCHNG_Msk               (0x1UL << GOTGINT_IDCHNG_Pos)            // 0x00100000
#define GOTGINT_IDCHNG                   GOTGINT_IDCHNG_Msk                       // Change in ID pin input value

/********************  Bit definition for DCFG register  ********************/
#define DCFG_DSPD_Pos                    (0U)
#define DCFG_DSPD_Msk                    (0x3UL << DCFG_DSPD_Pos)                 // 0x00000003
#define DCFG_DSPD_HS                     0    // Highspeed
#define DCFG_DSPD_FS_HSPHY               1    // Fullspeed on HS PHY
#define DCFG_DSPD_LS                     2    // Lowspeed
#define DCFG_DSPD_FS                     3    // Fullspeed on FS PHY

#define DCFG_NZLSOHSK_Pos                (2U)
#define DCFG_NZLSOHSK_Msk                (0x1UL << DCFG_NZLSOHSK_Pos)             // 0x00000004
#define DCFG_NZLSOHSK                    DCFG_NZLSOHSK_Msk                        // Nonzero-length status OUT handshake

#define DCFG_DAD_Pos                     (4U)
#define DCFG_DAD_Msk                     (0x7FUL << DCFG_DAD_Pos)                 // 0x000007F0
#define DCFG_DAD                         DCFG_DAD_Msk                             // Device address
#define DCFG_DAD_0                       (0x01UL << DCFG_DAD_Pos)                 // 0x00000010
#define DCFG_DAD_1                       (0x02UL << DCFG_DAD_Pos)                 // 0x00000020
#define DCFG_DAD_2                       (0x04UL << DCFG_DAD_Pos)                 // 0x00000040
#define DCFG_DAD_3                       (0x08UL << DCFG_DAD_Pos)                 // 0x00000080
#define DCFG_DAD_4                       (0x10UL << DCFG_DAD_Pos)                 // 0x00000100
#define DCFG_DAD_5                       (0x20UL << DCFG_DAD_Pos)                 // 0x00000200
#define DCFG_DAD_6                       (0x40UL << DCFG_DAD_Pos)                 // 0x00000400

#define DCFG_PFIVL_Pos                   (11U)
#define DCFG_PFIVL_Msk                   (0x3UL << DCFG_PFIVL_Pos)                // 0x00001800
#define DCFG_PFIVL                       DCFG_PFIVL_Msk                           // Periodic (micro)frame interval
#define DCFG_PFIVL_0                     (0x1UL << DCFG_PFIVL_Pos)                // 0x00000800
#define DCFG_PFIVL_1                     (0x2UL << DCFG_PFIVL_Pos)                // 0x00001000

#define DCFG_XCVRDLY_Pos                 (14U)
#define DCFG_XCVRDLY_Msk                 (0x1UL << DCFG_XCVRDLY_Pos)             // 0x00004000
#define DCFG_XCVRDLY                     DCFG_XCVRDLY_Msk                        // Enables delay between xcvr_sel and txvalid during device chirp

#define DCFG_PERSCHIVL_Pos               (24U)
#define DCFG_PERSCHIVL_Msk               (0x3UL << DCFG_PERSCHIVL_Pos)            // 0x03000000
#define DCFG_PERSCHIVL                   DCFG_PERSCHIVL_Msk                       // Periodic scheduling interval
#define DCFG_PERSCHIVL_0                 (0x1UL << DCFG_PERSCHIVL_Pos)            // 0x01000000
#define DCFG_PERSCHIVL_1                 (0x2UL << DCFG_PERSCHIVL_Pos)            // 0x02000000

/********************  Bit definition for DCTL register  ********************/
#define DCTL_RWUSIG_Pos                  (0U)
#define DCTL_RWUSIG_Msk                  (0x1UL << DCTL_RWUSIG_Pos)               // 0x00000001
#define DCTL_RWUSIG                      DCTL_RWUSIG_Msk                          // Remote wakeup signaling
#define DCTL_SDIS_Pos                    (1U)
#define DCTL_SDIS_Msk                    (0x1UL << DCTL_SDIS_Pos)                 // 0x00000002
#define DCTL_SDIS                        DCTL_SDIS_Msk                            // Soft disconnect
#define DCTL_GINSTS_Pos                  (2U)
#define DCTL_GINSTS_Msk                  (0x1UL << DCTL_GINSTS_Pos)               // 0x00000004
#define DCTL_GINSTS                      DCTL_GINSTS_Msk                          // Global IN NAK status
#define DCTL_GONSTS_Pos                  (3U)
#define DCTL_GONSTS_Msk                  (0x1UL << DCTL_GONSTS_Pos)               // 0x00000008
#define DCTL_GONSTS                      DCTL_GONSTS_Msk                          // Global OUT NAK status

#define DCTL_TCTL_Pos                    (4U)
#define DCTL_TCTL_Msk                    (0x7UL << DCTL_TCTL_Pos)                 // 0x00000070
#define DCTL_TCTL                        DCTL_TCTL_Msk                            // Test control
#define DCTL_TCTL_0                      (0x1UL << DCTL_TCTL_Pos)                 // 0x00000010
#define DCTL_TCTL_1                      (0x2UL << DCTL_TCTL_Pos)                 // 0x00000020
#define DCTL_TCTL_2                      (0x4UL << DCTL_TCTL_Pos)                 // 0x00000040
#define DCTL_SGINAK_Pos                  (7U)
#define DCTL_SGINAK_Msk                  (0x1UL << DCTL_SGINAK_Pos)               // 0x00000080
#define DCTL_SGINAK                      DCTL_SGINAK_Msk                          // Set global IN NAK
#define DCTL_CGINAK_Pos                  (8U)
#define DCTL_CGINAK_Msk                  (0x1UL << DCTL_CGINAK_Pos)               // 0x00000100
#define DCTL_CGINAK                      DCTL_CGINAK_Msk                          // Clear global IN NAK
#define DCTL_SGONAK_Pos                  (9U)
#define DCTL_SGONAK_Msk                  (0x1UL << DCTL_SGONAK_Pos)               // 0x00000200
#define DCTL_SGONAK                      DCTL_SGONAK_Msk                          // Set global OUT NAK
#define DCTL_CGONAK_Pos                  (10U)
#define DCTL_CGONAK_Msk                  (0x1UL << DCTL_CGONAK_Pos)               // 0x00000400
#define DCTL_CGONAK                      DCTL_CGONAK_Msk                          // Clear global OUT NAK
#define DCTL_POPRGDNE_Pos                (11U)
#define DCTL_POPRGDNE_Msk                (0x1UL << DCTL_POPRGDNE_Pos)             // 0x00000800
#define DCTL_POPRGDNE                    DCTL_POPRGDNE_Msk                        // Power-on programming done

/********************  Bit definition for HFIR register  ********************/
#define HFIR_FRIVL_Pos                   (0U)
#define HFIR_FRIVL_Msk                   (0xFFFFUL << HFIR_FRIVL_Pos)             // 0x0000FFFF
#define HFIR_FRIVL                       HFIR_FRIVL_Msk                           // Frame interval
#define HFIR_RELOAD_CTRL_Pos             (16U)                                    // available since v2.92a
#define HFIR_RELOAD_CTRL_Msk             (0x1UL << HFIR_RELOAD_CTRL_Pos)
#define HFIR_RELOAD_CTRL                  HFIR_RELOAD_CTRL_Msk

/********************  Bit definition for HFNUM register  ********************/
#define HFNUM_FRNUM_Pos                  (0U)
#define HFNUM_FRNUM_Msk                  (0xFFFFUL << HFNUM_FRNUM_Pos)            // 0x0000FFFF
#define HFNUM_FRNUM                      HFNUM_FRNUM_Msk                          // Frame number
#define HFNUM_FTREM_Pos                  (16U)
#define HFNUM_FTREM_Msk                  (0xFFFFUL << HFNUM_FTREM_Pos)            // 0xFFFF0000
#define HFNUM_FTREM                      HFNUM_FTREM_Msk                          // Frame time remaining

/********************  Bit definition for DSTS register  ********************/
#define DSTS_SUSPSTS_Pos                 (0U)
#define DSTS_SUSPSTS_Msk                 (0x1UL << DSTS_SUSPSTS_Pos)              // 0x00000001
#define DSTS_SUSPSTS                     DSTS_SUSPSTS_Msk                         // Suspend status
#define DSTS_ENUMSPD_Pos                 (1U)
#define DSTS_ENUMSPD_Msk                 (0x3UL << DSTS_ENUMSPD_Pos)              // 0x00000006
#define DSTS_ENUMSPD                     DSTS_ENUMSPD_Msk                         // Enumerated speed
#define DSTS_ENUMSPD_HS                  0    // Highspeed
#define DSTS_ENUMSPD_FS_HSPHY            1    // Fullspeed on HS PHY
#define DSTS_ENUMSPD_LS                  2    // Lowspeed
#define DSTS_ENUMSPD_FS                  3    // Fullspeed on FS PHY


#define DSTS_EERR_Pos                    (3U)
#define DSTS_EERR_Msk                    (0x1UL << DSTS_EERR_Pos)                 // 0x00000008
#define DSTS_EERR                        DSTS_EERR_Msk                            // Erratic error
#define DSTS_FNSOF_Pos                   (8U)
#define DSTS_FNSOF_Msk                   (0x3FFFUL << DSTS_FNSOF_Pos)             // 0x003FFF00
#define DSTS_FNSOF                       DSTS_FNSOF_Msk                           // Frame number of the received SOF

/********************  Bit definition for GAHBCFG register  ********************/
#define GAHBCFG_GINT_Pos                 (0U)
#define GAHBCFG_GINT_Msk                 (0x1UL << GAHBCFG_GINT_Pos)              // 0x00000001
#define GAHBCFG_GINT                     GAHBCFG_GINT_Msk                         // Global interrupt mask
#define GAHBCFG_HBSTLEN_Pos              (1U)
#define GAHBCFG_HBSTLEN_Msk              (0xFUL << GAHBCFG_HBSTLEN_Pos)           // 0x0000001E
#define GAHBCFG_HBSTLEN                  GAHBCFG_HBSTLEN_Msk                      // Burst length/type
#define GAHBCFG_HBSTLEN_0                (0x0UL << GAHBCFG_HBSTLEN_Pos)           // Single
#define GAHBCFG_HBSTLEN_1                (0x1UL << GAHBCFG_HBSTLEN_Pos)           // INCR
#define GAHBCFG_HBSTLEN_2                (0x3UL << GAHBCFG_HBSTLEN_Pos)           // INCR4
#define GAHBCFG_HBSTLEN_3                (0x5UL << GAHBCFG_HBSTLEN_Pos)           // INCR8
#define GAHBCFG_HBSTLEN_4                (0x7UL << GAHBCFG_HBSTLEN_Pos)           // INCR16
#define GAHBCFG_DMAEN_Pos                (5U)
#define GAHBCFG_DMAEN_Msk                (0x1UL << GAHBCFG_DMAEN_Pos)             // 0x00000020
#define GAHBCFG_DMAEN                    GAHBCFG_DMAEN_Msk                        // DMA enable
#define GAHBCFG_TX_FIFO_EPMTY_LVL_Pos    (7U)
#define GAHBCFG_TX_FIFO_EPMTY_LVL_Msk    (0x1UL << GAHBCFG_TX_FIFO_EPMTY_LVL_Pos) // 0x00000080
#define GAHBCFG_TX_FIFO_EPMTY_LVL        GAHBCFG_TX_FIFO_EPMTY_LVL_Msk            // TxFIFO empty level
#define GAHBCFG_PTX_FIFO_EPMTY_LVL_Pos   (8U)
#define GAHBCFG_PTX_FIFO_EPMTY_LVL_Msk   (0x1UL << GAHBCFG_PTX_FIFO_EPMTY_LVL_Pos) // 0x00000100
#define GAHBCFG_PTX_FIFO_EPMTY_LVL       GAHBCFG_PTX_FIFO_EPMTY_LVL_Msk            // Periodic TxFIFO empty level

/********************  Bit definition for GUSBCFG register  ********************/
#define GUSBCFG_TOCAL_Pos                (0U)
#define GUSBCFG_TOCAL_Msk                (0x7UL << GUSBCFG_TOCAL_Pos)             // 0x00000007
#define GUSBCFG_TOCAL                    GUSBCFG_TOCAL_Msk                        // HS/FS timeout calibration
#define GUSBCFG_PHYIF16_Pos              (3U)
#define GUSBCFG_PHYIF16_Msk              (0x1UL << GUSBCFG_PHYIF16_Pos)           // 0x00000008
#define GUSBCFG_PHYIF16                  GUSBCFG_PHYIF16_Msk                      // PHY Interface (PHYIf)
#define GUSBCFG_ULPI_UTMI_SEL_Pos        (4U)
#define GUSBCFG_ULPI_UTMI_SEL_Msk        (0x1UL << GUSBCFG_ULPI_UTMI_SEL_Pos)     // 0x00000010
#define GUSBCFG_ULPI_UTMI_SEL            GUSBCFG_ULPI_UTMI_SEL_Msk                // ULPI or UTMI+ Select (ULPI_UTMI_Sel)
#define GUSBCFG_PHYSEL_Pos               (6U)
#define GUSBCFG_PHYSEL_Msk               (0x1UL << GUSBCFG_PHYSEL_Pos)            // 0x00000040
#define GUSBCFG_PHYSEL                   GUSBCFG_PHYSEL_Msk                       // USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select
#define GUSBCFG_DDRSEL                   TU_BIT(7)                                // Single Data Rate (SDR) or Double Data Rate (DDR) or ULPI interface.
#define GUSBCFG_SRPCAP_Pos               (8U)
#define GUSBCFG_SRPCAP_Msk               (0x1UL << GUSBCFG_SRPCAP_Pos)            // 0x00000100
#define GUSBCFG_SRPCAP                   GUSBCFG_SRPCAP_Msk                       // SRP-capable
#define GUSBCFG_HNPCAP_Pos               (9U)
#define GUSBCFG_HNPCAP_Msk               (0x1UL << GUSBCFG_HNPCAP_Pos)            // 0x00000200
#define GUSBCFG_HNPCAP                   GUSBCFG_HNPCAP_Msk                       // HNP-capable
#define GUSBCFG_TRDT_Pos                 (10U)
#define GUSBCFG_TRDT_Msk                 (0xFUL << GUSBCFG_TRDT_Pos)              // 0x00003C00
#define GUSBCFG_TRDT                     GUSBCFG_TRDT_Msk                         // USB turnaround time
#define GUSBCFG_PHYLPCS_Pos              (15U)
#define GUSBCFG_PHYLPCS_Msk              (0x1UL << GUSBCFG_PHYLPCS_Pos)           // 0x00008000
#define GUSBCFG_PHYLPCS                  GUSBCFG_PHYLPCS_Msk                      // PHY Low-power clock select
#define GUSBCFG_ULPIFSLS_Pos             (17U)
#define GUSBCFG_ULPIFSLS_Msk             (0x1UL << GUSBCFG_ULPIFSLS_Pos)          // 0x00020000
#define GUSBCFG_ULPIFSLS                 GUSBCFG_ULPIFSLS_Msk                     // ULPI FS/LS select
#define GUSBCFG_ULPIAR_Pos               (18U)
#define GUSBCFG_ULPIAR_Msk               (0x1UL << GUSBCFG_ULPIAR_Pos)            // 0x00040000
#define GUSBCFG_ULPIAR                   GUSBCFG_ULPIAR_Msk                       // ULPI Auto-resume
#define GUSBCFG_ULPICSM_Pos              (19U)
#define GUSBCFG_ULPICSM_Msk              (0x1UL << GUSBCFG_ULPICSM_Pos)           // 0x00080000
#define GUSBCFG_ULPICSM                  GUSBCFG_ULPICSM_Msk                      // ULPI Clock SuspendM
#define GUSBCFG_ULPIEVBUSD_Pos           (20U)
#define GUSBCFG_ULPIEVBUSD_Msk           (0x1UL << GUSBCFG_ULPIEVBUSD_Pos)        // 0x00100000
#define GUSBCFG_ULPIEVBUSD               GUSBCFG_ULPIEVBUSD_Msk                   // ULPI External VBUS Drive
#define GUSBCFG_ULPIEVBUSI_Pos           (21U)
#define GUSBCFG_ULPIEVBUSI_Msk           (0x1UL << GUSBCFG_ULPIEVBUSI_Pos)        // 0x00200000
#define GUSBCFG_ULPIEVBUSI               GUSBCFG_ULPIEVBUSI_Msk                   // ULPI external VBUS indicator
#define GUSBCFG_TSDPS_Pos                (22U)
#define GUSBCFG_TSDPS_Msk                (0x1UL << GUSBCFG_TSDPS_Pos)             // 0x00400000
#define GUSBCFG_TSDPS                    GUSBCFG_TSDPS_Msk                        // TermSel DLine pulsing selection
#define GUSBCFG_PCCI_Pos                 (23U)
#define GUSBCFG_PCCI_Msk                 (0x1UL << GUSBCFG_PCCI_Pos)              // 0x00800000
#define GUSBCFG_PCCI                     GUSBCFG_PCCI_Msk                         // Indicator complement
#define GUSBCFG_PTCI_Pos                 (24U)
#define GUSBCFG_PTCI_Msk                 (0x1UL << GUSBCFG_PTCI_Pos)              // 0x01000000
#define GUSBCFG_PTCI                     GUSBCFG_PTCI_Msk                         // Indicator pass through
#define GUSBCFG_ULPIIPD_Pos              (25U)
#define GUSBCFG_ULPIIPD_Msk              (0x1UL << GUSBCFG_ULPIIPD_Pos)           // 0x02000000
#define GUSBCFG_ULPIIPD                  GUSBCFG_ULPIIPD_Msk                      // ULPI interface protect disable
#define GUSBCFG_FHMOD_Pos                (29U)
#define GUSBCFG_FHMOD_Msk                (0x1UL << GUSBCFG_FHMOD_Pos)             // 0x20000000
#define GUSBCFG_FHMOD                    GUSBCFG_FHMOD_Msk                        // Forced host mode
#define GUSBCFG_FDMOD_Pos                (30U)
#define GUSBCFG_FDMOD_Msk                (0x1UL << GUSBCFG_FDMOD_Pos)             // 0x40000000
#define GUSBCFG_FDMOD                    GUSBCFG_FDMOD_Msk                        // Forced peripheral mode
#define GUSBCFG_CTXPKT_Pos               (31U)
#define GUSBCFG_CTXPKT_Msk               (0x1UL << GUSBCFG_CTXPKT_Pos)            // 0x80000000
#define GUSBCFG_CTXPKT                   GUSBCFG_CTXPKT_Msk                       // Corrupt Tx packet

/********************  Bit definition for GRSTCTL register  ********************/
#define GRSTCTL_CSRST_Pos                (0U)
#define GRSTCTL_CSRST_Msk                (0x1UL << GRSTCTL_CSRST_Pos)             // 0x00000001
#define GRSTCTL_CSRST                    GRSTCTL_CSRST_Msk                        // Core soft reset
#define GRSTCTL_HSRST_Pos                (1U)
#define GRSTCTL_HSRST_Msk                (0x1UL << GRSTCTL_HSRST_Pos)             // 0x00000002
#define GRSTCTL_HSRST                    GRSTCTL_HSRST_Msk                        // HCLK soft reset
#define GRSTCTL_FCRST_Pos                (2U)
#define GRSTCTL_FCRST_Msk                (0x1UL << GRSTCTL_FCRST_Pos)             // 0x00000004
#define GRSTCTL_FCRST                    GRSTCTL_FCRST_Msk                        // Host frame counter reset
#define GRSTCTL_RXFFLSH_Pos              (4U)
#define GRSTCTL_RXFFLSH_Msk              (0x1UL << GRSTCTL_RXFFLSH_Pos)           // 0x00000010
#define GRSTCTL_RXFFLSH                  GRSTCTL_RXFFLSH_Msk                      // RxFIFO flush
#define GRSTCTL_TXFFLSH_Pos              (5U)
#define GRSTCTL_TXFFLSH_Msk              (0x1UL << GRSTCTL_TXFFLSH_Pos)           // 0x00000020
#define GRSTCTL_TXFFLSH                  GRSTCTL_TXFFLSH_Msk                      // TxFIFO flush
#define GRSTCTL_TXFNUM_Pos               (6U)
#define GRSTCTL_TXFNUM_Msk               (0x1FUL << GRSTCTL_TXFNUM_Pos)           // 0x000007C0
#define GRSTCTL_TXFNUM                   GRSTCTL_TXFNUM_Msk                       // TxFIFO number
#define GRSTCTL_TXFNUM_0                 (0x01UL << GRSTCTL_TXFNUM_Pos)           // 0x00000040
#define GRSTCTL_TXFNUM_1                 (0x02UL << GRSTCTL_TXFNUM_Pos)           // 0x00000080
#define GRSTCTL_TXFNUM_2                 (0x04UL << GRSTCTL_TXFNUM_Pos)           // 0x00000100
#define GRSTCTL_TXFNUM_3                 (0x08UL << GRSTCTL_TXFNUM_Pos)           // 0x00000200
#define GRSTCTL_TXFNUM_4                 (0x10UL << GRSTCTL_TXFNUM_Pos)           // 0x00000400
#define GRSTCTL_CSRST_DONE_Pos           (29)
#define GRSTCTL_CSRST_DONE               (1u << GRSTCTL_CSRST_DONE_Pos)         // Reset Done, only available from v4.20a
#define GRSTCTL_DMAREQ_Pos               (30U)
#define GRSTCTL_DMAREQ_Msk               (0x1UL << GRSTCTL_DMAREQ_Pos)            // 0x40000000
#define GRSTCTL_DMAREQ                   GRSTCTL_DMAREQ_Msk                       // DMA request signal
#define GRSTCTL_AHBIDL_Pos               (31U)
#define GRSTCTL_AHBIDL_Msk               (0x1UL << GRSTCTL_AHBIDL_Pos)            // 0x80000000
#define GRSTCTL_AHBIDL                   GRSTCTL_AHBIDL_Msk                       // AHB master idle

/********************  Bit definition for DIEPMSK register  ********************/
#define DIEPMSK_XFRCM_Pos                (0U)
#define DIEPMSK_XFRCM_Msk                (0x1UL << DIEPMSK_XFRCM_Pos)             // 0x00000001
#define DIEPMSK_XFRCM                    DIEPMSK_XFRCM_Msk                        // Transfer completed interrupt mask
#define DIEPMSK_EPDM_Pos                 (1U)
#define DIEPMSK_EPDM_Msk                 (0x1UL << DIEPMSK_EPDM_Pos)              // 0x00000002
#define DIEPMSK_EPDM                     DIEPMSK_EPDM_Msk                         // Endpoint disabled interrupt mask
#define DIEPMSK_TOM_Pos                  (3U)
#define DIEPMSK_TOM_Msk                  (0x1UL << DIEPMSK_TOM_Pos)               // 0x00000008
#define DIEPMSK_TOM                      DIEPMSK_TOM_Msk                          // Timeout condition mask (nonisochronous endpoints)
#define DIEPMSK_ITTXFEMSK_Pos            (4U)
#define DIEPMSK_ITTXFEMSK_Msk            (0x1UL << DIEPMSK_ITTXFEMSK_Pos)         // 0x00000010
#define DIEPMSK_ITTXFEMSK                DIEPMSK_ITTXFEMSK_Msk                    // IN token received when TxFIFO empty mask
#define DIEPMSK_INEPNMM_Pos              (5U)
#define DIEPMSK_INEPNMM_Msk              (0x1UL << DIEPMSK_INEPNMM_Pos)           // 0x00000020
#define DIEPMSK_INEPNMM                  DIEPMSK_INEPNMM_Msk                      // IN token received with EP mismatch mask
#define DIEPMSK_INEPNEM_Pos              (6U)
#define DIEPMSK_INEPNEM_Msk              (0x1UL << DIEPMSK_INEPNEM_Pos)           // 0x00000040
#define DIEPMSK_INEPNEM                  DIEPMSK_INEPNEM_Msk                      // IN endpoint NAK effective mask
#define DIEPMSK_TXFURM_Pos               (8U)
#define DIEPMSK_TXFURM_Msk               (0x1UL << DIEPMSK_TXFURM_Pos)            // 0x00000100
#define DIEPMSK_TXFURM                   DIEPMSK_TXFURM_Msk                       // FIFO underrun mask
#define DIEPMSK_BIM_Pos                  (9U)
#define DIEPMSK_BIM_Msk                  (0x1UL << DIEPMSK_BIM_Pos)               // 0x00000200
#define DIEPMSK_BIM                      DIEPMSK_BIM_Msk                          // BNA interrupt mask

/********************  Bit definition for HPTXSTS register  ********************/
#define HPTXSTS_PTXFSAVL_Pos             (0U)
#define HPTXSTS_PTXFSAVL_Msk             (0xFFFFUL << HPTXSTS_PTXFSAVL_Pos)       // 0x0000FFFF
#define HPTXSTS_PTXFSAVL                 HPTXSTS_PTXFSAVL_Msk                     // Periodic transmit data FIFO space available
#define HPTXSTS_PTXQSAV_Pos              (16U)
#define HPTXSTS_PTXQSAV_Msk              (0xFFUL << HPTXSTS_PTXQSAV_Pos)          // 0x00FF0000
#define HPTXSTS_PTXQSAV                  HPTXSTS_PTXQSAV_Msk                      // Periodic transmit request queue space available
#define HPTXSTS_PTXQSAV_0                (0x01UL << HPTXSTS_PTXQSAV_Pos)          // 0x00010000
#define HPTXSTS_PTXQSAV_1                (0x02UL << HPTXSTS_PTXQSAV_Pos)          // 0x00020000
#define HPTXSTS_PTXQSAV_2                (0x04UL << HPTXSTS_PTXQSAV_Pos)          // 0x00040000
#define HPTXSTS_PTXQSAV_3                (0x08UL << HPTXSTS_PTXQSAV_Pos)          // 0x00080000
#define HPTXSTS_PTXQSAV_4                (0x10UL << HPTXSTS_PTXQSAV_Pos)          // 0x00100000
#define HPTXSTS_PTXQSAV_5                (0x20UL << HPTXSTS_PTXQSAV_Pos)          // 0x00200000
#define HPTXSTS_PTXQSAV_6                (0x40UL << HPTXSTS_PTXQSAV_Pos)          // 0x00400000
#define HPTXSTS_PTXQSAV_7                (0x80UL << HPTXSTS_PTXQSAV_Pos)          // 0x00800000

#define HPTXSTS_PTXQTOP_Pos              (24U)
#define HPTXSTS_PTXQTOP_Msk              (0xFFUL << HPTXSTS_PTXQTOP_Pos)          // 0xFF000000
#define HPTXSTS_PTXQTOP                  HPTXSTS_PTXQTOP_Msk                      // Top of the periodic transmit request queue
#define HPTXSTS_PTXQTOP_0                (0x01UL << HPTXSTS_PTXQTOP_Pos)          // 0x01000000
#define HPTXSTS_PTXQTOP_1                (0x02UL << HPTXSTS_PTXQTOP_Pos)          // 0x02000000
#define HPTXSTS_PTXQTOP_2                (0x04UL << HPTXSTS_PTXQTOP_Pos)          // 0x04000000
#define HPTXSTS_PTXQTOP_3                (0x08UL << HPTXSTS_PTXQTOP_Pos)          // 0x08000000
#define HPTXSTS_PTXQTOP_4                (0x10UL << HPTXSTS_PTXQTOP_Pos)          // 0x10000000
#define HPTXSTS_PTXQTOP_5                (0x20UL << HPTXSTS_PTXQTOP_Pos)          // 0x20000000
#define HPTXSTS_PTXQTOP_6                (0x40UL << HPTXSTS_PTXQTOP_Pos)          // 0x40000000
#define HPTXSTS_PTXQTOP_7                (0x80UL << HPTXSTS_PTXQTOP_Pos)          // 0x80000000

/********************  Bit definition for HAINT register  ********************/
#define HAINT_HAINT_Pos                  (0U)
#define HAINT_HAINT_Msk                  (0xFFFFUL << HAINT_HAINT_Pos)            // 0x0000FFFF
#define HAINT_HAINT                      HAINT_HAINT_Msk                          // Channel interrupts

/********************  Bit definition for DOEPMSK register  ********************/
#define DOEPMSK_XFRCM_Pos                (0U)
#define DOEPMSK_XFRCM_Msk                (0x1UL << DOEPMSK_XFRCM_Pos)             // 0x00000001
#define DOEPMSK_XFRCM                    DOEPMSK_XFRCM_Msk                        // Transfer completed interrupt mask
#define DOEPMSK_EPDM_Pos                 (1U)
#define DOEPMSK_EPDM_Msk                 (0x1UL << DOEPMSK_EPDM_Pos)              // 0x00000002
#define DOEPMSK_EPDM                     DOEPMSK_EPDM_Msk                         // Endpoint disabled interrupt mask
#define DOEPMSK_AHBERRM_Pos              (2U)
#define DOEPMSK_AHBERRM_Msk              (0x1UL << DOEPMSK_AHBERRM_Pos)           // 0x00000004
#define DOEPMSK_AHBERRM                  DOEPMSK_AHBERRM_Msk                      // OUT transaction AHB Error interrupt mask
#define DOEPMSK_STUPM_Pos                (3U)
#define DOEPMSK_STUPM_Msk                (0x1UL << DOEPMSK_STUPM_Pos)             // 0x00000008
#define DOEPMSK_STUPM                    DOEPMSK_STUPM_Msk                        // SETUP phase done mask
#define DOEPMSK_OTEPDM_Pos               (4U)
#define DOEPMSK_OTEPDM_Msk               (0x1UL << DOEPMSK_OTEPDM_Pos)            // 0x00000010
#define DOEPMSK_OTEPDM                   DOEPMSK_OTEPDM_Msk                       // OUT token received when endpoint disabled mask
#define DOEPMSK_OTEPSPRM_Pos             (5U)
#define DOEPMSK_OTEPSPRM_Msk             (0x1UL << DOEPMSK_OTEPSPRM_Pos)          // 0x00000020
#define DOEPMSK_OTEPSPRM                 DOEPMSK_OTEPSPRM_Msk                     // Status Phase Received mask
#define DOEPMSK_B2BSTUP_Pos              (6U)
#define DOEPMSK_B2BSTUP_Msk              (0x1UL << DOEPMSK_B2BSTUP_Pos)           // 0x00000040
#define DOEPMSK_B2BSTUP                  DOEPMSK_B2BSTUP_Msk                      // Back-to-back SETUP packets received mask
#define DOEPMSK_OPEM_Pos                 (8U)
#define DOEPMSK_OPEM_Msk                 (0x1UL << DOEPMSK_OPEM_Pos)              // 0x00000100
#define DOEPMSK_OPEM                     DOEPMSK_OPEM_Msk                         // OUT packet error mask
#define DOEPMSK_BOIM_Pos                 (9U)
#define DOEPMSK_BOIM_Msk                 (0x1UL << DOEPMSK_BOIM_Pos)              // 0x00000200
#define DOEPMSK_BOIM                     DOEPMSK_BOIM_Msk                         // BNA interrupt mask
#define DOEPMSK_BERRM_Pos                (12U)
#define DOEPMSK_BERRM_Msk                (0x1UL << DOEPMSK_BERRM_Pos)             // 0x00001000
#define DOEPMSK_BERRM                    DOEPMSK_BERRM_Msk                        // Babble error interrupt mask
#define DOEPMSK_NAKM_Pos                 (13U)
#define DOEPMSK_NAKM_Msk                 (0x1UL << DOEPMSK_NAKM_Pos)              // 0x00002000
#define DOEPMSK_NAKM                     DOEPMSK_NAKM_Msk                         // OUT Packet NAK interrupt mask
#define DOEPMSK_NYETM_Pos                (14U)
#define DOEPMSK_NYETM_Msk                (0x1UL << DOEPMSK_NYETM_Pos)             // 0x00004000
#define DOEPMSK_NYETM                    DOEPMSK_NYETM_Msk                        // NYET interrupt mask

/********************  Bit definition for GINTSTS register  ********************/
#define GINTSTS_CMOD_Pos                 (0U)
#define GINTSTS_CMOD_Msk                 (0x1UL << GINTSTS_CMOD_Pos)              // 0x00000001
#define GINTSTS_CMOD                     GINTSTS_CMOD_Msk                         // Current mode of operation
#define GINTSTS_MMIS_Pos                 (1U)
#define GINTSTS_MMIS_Msk                 (0x1UL << GINTSTS_MMIS_Pos)              // 0x00000002
#define GINTSTS_MMIS                     GINTSTS_MMIS_Msk                         // Mode mismatch interrupt
#define GINTSTS_OTGINT_Pos               (2U)
#define GINTSTS_OTGINT_Msk               (0x1UL << GINTSTS_OTGINT_Pos)            // 0x00000004
#define GINTSTS_OTGINT                   GINTSTS_OTGINT_Msk                       // OTG interrupt
#define GINTSTS_SOF_Pos                  (3U)
#define GINTSTS_SOF_Msk                  (0x1UL << GINTSTS_SOF_Pos)               // 0x00000008
#define GINTSTS_SOF                      GINTSTS_SOF_Msk                          // Start of frame
#define GINTSTS_RXFLVL_Pos               (4U)
#define GINTSTS_RXFLVL_Msk               (0x1UL << GINTSTS_RXFLVL_Pos)            // 0x00000010
#define GINTSTS_RXFLVL                   GINTSTS_RXFLVL_Msk                       // RxFIFO nonempty
#define GINTSTS_NPTX_FIFO_EMPTY_Pos      (5U)
#define GINTSTS_NPTX_FIFO_EMPTY_Msk      (0x1UL << GINTSTS_NPTX_FIFO_EMPTY_Pos)   // 0x00000020
#define GINTSTS_NPTX_FIFO_EMPTY          GINTSTS_NPTX_FIFO_EMPTY_Msk              // Nonperiodic TxFIFO empty
#define GINTSTS_GINAKEFF_Pos             (6U)
#define GINTSTS_GINAKEFF_Msk             (0x1UL << GINTSTS_GINAKEFF_Pos)          // 0x00000040
#define GINTSTS_GINAKEFF                 GINTSTS_GINAKEFF_Msk                     // Global IN nonperiodic NAK effective
#define GINTSTS_BOUTNAKEFF_Pos           (7U)
#define GINTSTS_BOUTNAKEFF_Msk           (0x1UL << GINTSTS_BOUTNAKEFF_Pos)        // 0x00000080
#define GINTSTS_BOUTNAKEFF               GINTSTS_BOUTNAKEFF_Msk                   // Global OUT NAK effective
#define GINTSTS_ESUSP_Pos                (10U)
#define GINTSTS_ESUSP_Msk                (0x1UL << GINTSTS_ESUSP_Pos)             // 0x00000400
#define GINTSTS_ESUSP                    GINTSTS_ESUSP_Msk                        // Early suspend
#define GINTSTS_USBSUSP_Pos              (11U)
#define GINTSTS_USBSUSP_Msk              (0x1UL << GINTSTS_USBSUSP_Pos)           // 0x00000800
#define GINTSTS_USBSUSP                  GINTSTS_USBSUSP_Msk                      // USB suspend
#define GINTSTS_USBRST_Pos               (12U)
#define GINTSTS_USBRST_Msk               (0x1UL << GINTSTS_USBRST_Pos)            // 0x00001000
#define GINTSTS_USBRST                   GINTSTS_USBRST_Msk                       // USB reset
#define GINTSTS_ENUMDNE_Pos              (13U)
#define GINTSTS_ENUMDNE_Msk              (0x1UL << GINTSTS_ENUMDNE_Pos)           // 0x00002000
#define GINTSTS_ENUMDNE                  GINTSTS_ENUMDNE_Msk                      // Enumeration done
#define GINTSTS_ISOODRP_Pos              (14U)
#define GINTSTS_ISOODRP_Msk              (0x1UL << GINTSTS_ISOODRP_Pos)           // 0x00004000
#define GINTSTS_ISOODRP                  GINTSTS_ISOODRP_Msk                      // Isochronous OUT packet dropped interrupt
#define GINTSTS_EOPF_Pos                 (15U)
#define GINTSTS_EOPF_Msk                 (0x1UL << GINTSTS_EOPF_Pos)              // 0x00008000
#define GINTSTS_EOPF                     GINTSTS_EOPF_Msk                         // End of periodic frame interrupt
#define GINTSTS_IEPINT_Pos               (18U)
#define GINTSTS_IEPINT_Msk               (0x1UL << GINTSTS_IEPINT_Pos)            // 0x00040000
#define GINTSTS_IEPINT                   GINTSTS_IEPINT_Msk                       // IN endpoint interrupt
#define GINTSTS_OEPINT_Pos               (19U)
#define GINTSTS_OEPINT_Msk               (0x1UL << GINTSTS_OEPINT_Pos)            // 0x00080000
#define GINTSTS_OEPINT                   GINTSTS_OEPINT_Msk                       // OUT endpoint interrupt
#define GINTSTS_IISOIXFR_Pos             (20U)
#define GINTSTS_IISOIXFR_Msk             (0x1UL << GINTSTS_IISOIXFR_Pos)          // 0x00100000
#define GINTSTS_IISOIXFR                 GINTSTS_IISOIXFR_Msk                     // Incomplete isochronous IN transfer
#define GINTSTS_PXFR_INCOMPISOOUT_Pos    (21U)
#define GINTSTS_PXFR_INCOMPISOOUT_Msk    (0x1UL << GINTSTS_PXFR_INCOMPISOOUT_Pos) // 0x00200000
#define GINTSTS_PXFR_INCOMPISOOUT        GINTSTS_PXFR_INCOMPISOOUT_Msk            // Incomplete periodic transfer
#define GINTSTS_DATAFSUSP_Pos            (22U)
#define GINTSTS_DATAFSUSP_Msk            (0x1UL << GINTSTS_DATAFSUSP_Pos)         // 0x00400000
#define GINTSTS_DATAFSUSP                GINTSTS_DATAFSUSP_Msk                    // Data fetch suspended
#define GINTSTS_RSTDET_Pos               (23U)
#define GINTSTS_RSTDET_Msk               (0x1UL << GINTSTS_RSTDET_Pos)            // 0x00800000
#define GINTSTS_RSTDET                   GINTSTS_RSTDET_Msk                       // Reset detected interrupt
#define GINTSTS_HPRTINT_Pos              (24U)
#define GINTSTS_HPRTINT_Msk              (0x1UL << GINTSTS_HPRTINT_Pos)           // 0x01000000
#define GINTSTS_HPRTINT                  GINTSTS_HPRTINT_Msk                      // Host port interrupt
#define GINTSTS_HCINT_Pos                (25U)
#define GINTSTS_HCINT_Msk                (0x1UL << GINTSTS_HCINT_Pos)             // 0x02000000
#define GINTSTS_HCINT                    GINTSTS_HCINT_Msk                        // Host channels interrupt
#define GINTSTS_PTX_FIFO_EMPTY_Pos       (26U)
#define GINTSTS_PTX_FIFO_EMPTY_Msk       (0x1UL << GINTSTS_PTX_FIFO_EMPTY_Pos)    // 0x04000000
#define GINTSTS_PTX_FIFO_EMPTY           GINTSTS_PTX_FIFO_EMPTY_Msk               // Periodic TxFIFO empty
#define GINTSTS_LPMINT_Pos               (27U)
#define GINTSTS_LPMINT_Msk               (0x1UL << GINTSTS_LPMINT_Pos)            // 0x08000000
#define GINTSTS_LPMINT                   GINTSTS_LPMINT_Msk                       // LPM interrupt
#define GINTSTS_CONIDSTSCHNG_Pos         (28U)
#define GINTSTS_CONIDSTSCHNG_Msk         (0x1UL << GINTSTS_CONIDSTSCHNG_Pos)      // 0x10000000
#define GINTSTS_CONIDSTSCHNG             GINTSTS_CONIDSTSCHNG_Msk                 // Connector ID status change
#define GINTSTS_DISCINT_Pos              (29U)
#define GINTSTS_DISCINT_Msk              (0x1UL << GINTSTS_DISCINT_Pos)           // 0x20000000
#define GINTSTS_DISCINT                  GINTSTS_DISCINT_Msk                      // Disconnect detected interrupt
#define GINTSTS_SRQINT_Pos               (30U)
#define GINTSTS_SRQINT_Msk               (0x1UL << GINTSTS_SRQINT_Pos)            // 0x40000000
#define GINTSTS_SRQINT                   GINTSTS_SRQINT_Msk                       // Session request/new session detected interrupt
#define GINTSTS_WKUINT_Pos               (31U)
#define GINTSTS_WKUINT_Msk               (0x1UL << GINTSTS_WKUINT_Pos)            // 0x80000000
#define GINTSTS_WKUINT                   GINTSTS_WKUINT_Msk                       // Resume/remote wakeup detected interrupt

/********************  Bit definition for GINTMSK register  ********************/
#define GINTMSK_MMISM_Pos                (1U)
#define GINTMSK_MMISM_Msk                (0x1UL << GINTMSK_MMISM_Pos)             // 0x00000002
#define GINTMSK_MMISM                    GINTMSK_MMISM_Msk                        // Mode mismatch interrupt mask
#define GINTMSK_OTGINT_Pos               (2U)
#define GINTMSK_OTGINT_Msk               (0x1UL << GINTMSK_OTGINT_Pos)            // 0x00000004
#define GINTMSK_OTGINT                   GINTMSK_OTGINT_Msk                       // OTG interrupt mask
#define GINTMSK_SOFM_Pos                 (3U)
#define GINTMSK_SOFM_Msk                 (0x1UL << GINTMSK_SOFM_Pos)              // 0x00000008
#define GINTMSK_SOFM                     GINTMSK_SOFM_Msk                         // Start of frame mask
#define GINTMSK_RXFLVLM_Pos              (4U)
#define GINTMSK_RXFLVLM_Msk              (0x1UL << GINTMSK_RXFLVLM_Pos)           // 0x00000010
#define GINTMSK_RXFLVLM                  GINTMSK_RXFLVLM_Msk                      // Receive FIFO nonempty mask
#define GINTMSK_NPTXFEM_Pos              (5U)
#define GINTMSK_NPTXFEM_Msk              (0x1UL << GINTMSK_NPTXFEM_Pos)           // 0x00000020
#define GINTMSK_NPTXFEM                  GINTMSK_NPTXFEM_Msk                      // Nonperiodic TxFIFO empty mask
#define GINTMSK_GINAKEFFM_Pos            (6U)
#define GINTMSK_GINAKEFFM_Msk            (0x1UL << GINTMSK_GINAKEFFM_Pos)         // 0x00000040
#define GINTMSK_GINAKEFFM                GINTMSK_GINAKEFFM_Msk                    // Global nonperiodic IN NAK effective mask
#define GINTMSK_GONAKEFFM_Pos            (7U)
#define GINTMSK_GONAKEFFM_Msk            (0x1UL << GINTMSK_GONAKEFFM_Pos)         // 0x00000080
#define GINTMSK_GONAKEFFM                GINTMSK_GONAKEFFM_Msk                    // Global OUT NAK effective mask
#define GINTMSK_ESUSPM_Pos               (10U)
#define GINTMSK_ESUSPM_Msk               (0x1UL << GINTMSK_ESUSPM_Pos)            // 0x00000400
#define GINTMSK_ESUSPM                   GINTMSK_ESUSPM_Msk                       // Early suspend mask
#define GINTMSK_USBSUSPM_Pos             (11U)
#define GINTMSK_USBSUSPM_Msk             (0x1UL << GINTMSK_USBSUSPM_Pos)          // 0x00000800
#define GINTMSK_USBSUSPM                 GINTMSK_USBSUSPM_Msk                     // USB suspend mask
#define GINTMSK_USBRST_Pos               (12U)
#define GINTMSK_USBRST_Msk               (0x1UL << GINTMSK_USBRST_Pos)            // 0x00001000
#define GINTMSK_USBRST                   GINTMSK_USBRST_Msk                       // USB reset mask
#define GINTMSK_ENUMDNEM_Pos             (13U)
#define GINTMSK_ENUMDNEM_Msk             (0x1UL << GINTMSK_ENUMDNEM_Pos)          // 0x00002000
#define GINTMSK_ENUMDNEM                 GINTMSK_ENUMDNEM_Msk                     // Enumeration done mask
#define GINTMSK_ISOODRPM_Pos             (14U)
#define GINTMSK_ISOODRPM_Msk             (0x1UL << GINTMSK_ISOODRPM_Pos)          // 0x00004000
#define GINTMSK_ISOODRPM                 GINTMSK_ISOODRPM_Msk                     // Isochronous OUT packet dropped interrupt mask
#define GINTMSK_EOPFM_Pos                (15U)
#define GINTMSK_EOPFM_Msk                (0x1UL << GINTMSK_EOPFM_Pos)             // 0x00008000
#define GINTMSK_EOPFM                    GINTMSK_EOPFM_Msk                        // End of periodic frame interrupt mask
#define GINTMSK_EPMISM_Pos               (17U)
#define GINTMSK_EPMISM_Msk               (0x1UL << GINTMSK_EPMISM_Pos)            // 0x00020000
#define GINTMSK_EPMISM                   GINTMSK_EPMISM_Msk                       // Endpoint mismatch interrupt mask
#define GINTMSK_IEPINT_Pos               (18U)
#define GINTMSK_IEPINT_Msk               (0x1UL << GINTMSK_IEPINT_Pos)            // 0x00040000
#define GINTMSK_IEPINT                   GINTMSK_IEPINT_Msk                       // IN endpoints interrupt mask
#define GINTMSK_OEPINT_Pos               (19U)
#define GINTMSK_OEPINT_Msk               (0x1UL << GINTMSK_OEPINT_Pos)            // 0x00080000
#define GINTMSK_OEPINT                   GINTMSK_OEPINT_Msk                       // OUT endpoints interrupt mask
#define GINTMSK_IISOIXFRM_Pos            (20U)
#define GINTMSK_IISOIXFRM_Msk            (0x1UL << GINTMSK_IISOIXFRM_Pos)         // 0x00100000
#define GINTMSK_IISOIXFRM                GINTMSK_IISOIXFRM_Msk                    // Incomplete isochronous IN transfer mask
#define GINTMSK_PXFRM_IISOOXFRM_Pos      (21U)
#define GINTMSK_PXFRM_IISOOXFRM_Msk      (0x1UL << GINTMSK_PXFRM_IISOOXFRM_Pos)   // 0x00200000
#define GINTMSK_PXFRM_IISOOXFRM          GINTMSK_PXFRM_IISOOXFRM_Msk              // Incomplete periodic transfer mask
#define GINTMSK_FSUSPM_Pos               (22U)
#define GINTMSK_FSUSPM_Msk               (0x1UL << GINTMSK_FSUSPM_Pos)            // 0x00400000
#define GINTMSK_FSUSPM                   GINTMSK_FSUSPM_Msk                       // Data fetch suspended mask
#define GINTMSK_RSTDEM_Pos               (23U)
#define GINTMSK_RSTDEM_Msk               (0x1UL << GINTMSK_RSTDEM_Pos)            // 0x00800000
#define GINTMSK_RSTDEM                   GINTMSK_RSTDEM_Msk                       // Reset detected interrupt mask
#define GINTMSK_PRTIM_Pos                (24U)
#define GINTMSK_PRTIM_Msk                (0x1UL << GINTMSK_PRTIM_Pos)             // 0x01000000
#define GINTMSK_PRTIM                    GINTMSK_PRTIM_Msk                        // Host port interrupt mask
#define GINTMSK_HCIM_Pos                 (25U)
#define GINTMSK_HCIM_Msk                 (0x1UL << GINTMSK_HCIM_Pos)              // 0x02000000
#define GINTMSK_HCIM                     GINTMSK_HCIM_Msk                         // Host channels interrupt mask
#define GINTMSK_PTXFEM_Pos               (26U)
#define GINTMSK_PTXFEM_Msk               (0x1UL << GINTMSK_PTXFEM_Pos)            // 0x04000000
#define GINTMSK_PTXFEM                   GINTMSK_PTXFEM_Msk                       // Periodic TxFIFO empty mask
#define GINTMSK_LPMINTM_Pos              (27U)
#define GINTMSK_LPMINTM_Msk              (0x1UL << GINTMSK_LPMINTM_Pos)           // 0x08000000
#define GINTMSK_LPMINTM                  GINTMSK_LPMINTM_Msk                      // LPM interrupt Mask
#define GINTMSK_CONIDSTSCHNGM_Pos        (28U)
#define GINTMSK_CONIDSTSCHNGM_Msk        (0x1UL << GINTMSK_CONIDSTSCHNGM_Pos)     // 0x10000000
#define GINTMSK_CONIDSTSCHNGM            GINTMSK_CONIDSTSCHNGM_Msk                // Connector ID status change mask
#define GINTMSK_DISCINT_Pos              (29U)
#define GINTMSK_DISCINT_Msk              (0x1UL << GINTMSK_DISCINT_Pos)           // 0x20000000
#define GINTMSK_DISCINT                  GINTMSK_DISCINT_Msk                      // Disconnect detected interrupt mask
#define GINTMSK_SRQIM_Pos                (30U)
#define GINTMSK_SRQIM_Msk                (0x1UL << GINTMSK_SRQIM_Pos)             // 0x40000000
#define GINTMSK_SRQIM                    GINTMSK_SRQIM_Msk                        // Session request/new session detected interrupt mask
#define GINTMSK_WUIM_Pos                 (31U)
#define GINTMSK_WUIM_Msk                 (0x1UL << GINTMSK_WUIM_Pos)              // 0x80000000
#define GINTMSK_WUIM                     GINTMSK_WUIM_Msk                         // Resume/remote wakeup detected interrupt mask

/********************  Bit definition for DAINT register  ********************/
#define DAINT_IEPINT_Pos                 (0U)
#define DAINT_IEPINT_Msk                 (0xFFFFUL << DAINT_IEPINT_Pos)           // 0x0000FFFF
#define DAINT_IEPINT                     DAINT_IEPINT_Msk                         // IN endpoint interrupt bits
#define DAINT_OEPINT_Pos                 (16U)
#define DAINT_OEPINT_Msk                 (0xFFFFUL << DAINT_OEPINT_Pos)           // 0xFFFF0000
#define DAINT_OEPINT                     DAINT_OEPINT_Msk                         // OUT endpoint interrupt bits

/********************  Bit definition for HAINTMSK register  ********************/
#define HAINTMSK_HAINTM_Pos              (0U)
#define HAINTMSK_HAINTM_Msk              (0xFFFFUL << HAINTMSK_HAINTM_Pos)        // 0x0000FFFF
#define HAINTMSK_HAINTM                  HAINTMSK_HAINTM_Msk                      // Channel interrupt mask

/********************  Bit definition for GRXSTSP register  ********************/
#define GRXSTSP_EPNUM_Pos                (0U)
#define GRXSTSP_EPNUM_Msk                (0xFUL << GRXSTSP_EPNUM_Pos)             // 0x0000000F
#define GRXSTSP_EPNUM                    GRXSTSP_EPNUM_Msk                        // IN EP interrupt mask bits
#define GRXSTSP_BCNT_Pos                 (4U)
#define GRXSTSP_BCNT_Msk                 (0x7FFUL << GRXSTSP_BCNT_Pos)            // 0x00007FF0
#define GRXSTSP_BCNT                     GRXSTSP_BCNT_Msk                         // OUT EP interrupt mask bits
#define GRXSTSP_DPID_Pos                 (15U)
#define GRXSTSP_DPID_Msk                 (0x3UL << GRXSTSP_DPID_Pos)              // 0x00018000
#define GRXSTSP_DPID                     GRXSTSP_DPID_Msk                         // OUT EP interrupt mask bits
#define GRXSTSP_PKTSTS_Pos               (17U)
#define GRXSTSP_PKTSTS_Msk               (0xFUL << GRXSTSP_PKTSTS_Pos)            // 0x001E0000
#define GRXSTSP_PKTSTS                   GRXSTSP_PKTSTS_Msk                       // OUT EP interrupt mask bits

/********************  Bit definition for DAINTMSK register  ********************/
#define DAINTMSK_IEPM_Pos                (0U)
#define DAINTMSK_IEPM_Msk                (0xFFFFUL << DAINTMSK_IEPM_Pos)          // 0x0000FFFF
#define DAINTMSK_IEPM                    DAINTMSK_IEPM_Msk                        // IN EP interrupt mask bits
#define DAINTMSK_OEPM_Pos                (16U)
#define DAINTMSK_OEPM_Msk                (0xFFFFUL << DAINTMSK_OEPM_Pos)          // 0xFFFF0000
#define DAINTMSK_OEPM                    DAINTMSK_OEPM_Msk                        // OUT EP interrupt mask bits

#define DAINT_SHIFT(_dir)            ((_dir == TUSB_DIR_IN) ? 0 : 16)

#if 0
/********************  Bit definition for OTG register  ********************/
#define CHNUM_Pos                        (0U)
#define CHNUM_Msk                        (0xFUL << CHNUM_Pos)                     // 0x0000000F
#define CHNUM                            CHNUM_Msk                                // Channel number
#define CHNUM_0                          (0x1UL << CHNUM_Pos)                     // 0x00000001
#define CHNUM_1                          (0x2UL << CHNUM_Pos)                     // 0x00000002
#define CHNUM_2                          (0x4UL << CHNUM_Pos)                     // 0x00000004
#define CHNUM_3                          (0x8UL << CHNUM_Pos)                     // 0x00000008
#define BCNT_Pos                         (4U)
#define BCNT_Msk                         (0x7FFUL << BCNT_Pos)                    // 0x00007FF0
#define BCNT                             BCNT_Msk                                 // Byte count

#define DPID_Pos                         (15U)
#define DPID_Msk                         (0x3UL << DPID_Pos)                      // 0x00018000
#define DPID                             DPID_Msk                                 // Data PID
#define DPID_0                           (0x1UL << DPID_Pos)                      // 0x00008000
#define DPID_1                           (0x2UL << DPID_Pos)                      // 0x00010000

#define PKTSTS_Pos                       (17U)
#define PKTSTS_Msk                       (0xFUL << PKTSTS_Pos)                    // 0x001E0000
#define PKTSTS                           PKTSTS_Msk                               // Packet status
#define PKTSTS_0                         (0x1UL << PKTSTS_Pos)                    // 0x00020000
#define PKTSTS_1                         (0x2UL << PKTSTS_Pos)                    // 0x00040000
#define PKTSTS_2                         (0x4UL << PKTSTS_Pos)                    // 0x00080000
#define PKTSTS_3                         (0x8UL << PKTSTS_Pos)                    // 0x00100000

#define EPNUM_Pos                        (0U)
#define EPNUM_Msk                        (0xFUL << EPNUM_Pos)                     // 0x0000000F
#define EPNUM                            EPNUM_Msk                                // Endpoint number
#define EPNUM_0                          (0x1UL << EPNUM_Pos)                     // 0x00000001
#define EPNUM_1                          (0x2UL << EPNUM_Pos)                     // 0x00000002
#define EPNUM_2                          (0x4UL << EPNUM_Pos)                     // 0x00000004
#define EPNUM_3                          (0x8UL << EPNUM_Pos)                     // 0x00000008

#define FRMNUM_Pos                       (21U)
#define FRMNUM_Msk                       (0xFUL << FRMNUM_Pos)                    // 0x01E00000
#define FRMNUM                           FRMNUM_Msk                               // Frame number
#define FRMNUM_0                         (0x1UL << FRMNUM_Pos)                    // 0x00200000
#define FRMNUM_1                         (0x2UL << FRMNUM_Pos)                    // 0x00400000
#define FRMNUM_2                         (0x4UL << FRMNUM_Pos)                    // 0x00800000
#define FRMNUM_3                         (0x8UL << FRMNUM_Pos)                    // 0x01000000
#endif

/********************  Bit definition for GRXFSIZ register  ********************/
#define GRXFSIZ_RXFD_Pos                 (0U)
#define GRXFSIZ_RXFD_Msk                 (0xFFFFUL << GRXFSIZ_RXFD_Pos)           // 0x0000FFFF
#define GRXFSIZ_RXFD                     GRXFSIZ_RXFD_Msk                         // RxFIFO depth

/********************  Bit definition for DVBUSDIS register  ********************/
#define DVBUSDIS_VBUSDT_Pos              (0U)
#define DVBUSDIS_VBUSDT_Msk              (0xFFFFUL << DVBUSDIS_VBUSDT_Pos)        // 0x0000FFFF
#define DVBUSDIS_VBUSDT                  DVBUSDIS_VBUSDT_Msk                      // Device VBUS discharge time

/********************  Bit definition for OTG register  ********************/
#define GNPTXFSIZ_NPTXFSA_Pos            (0U)
#define GNPTXFSIZ_NPTXFSA_Msk            (0xFFFFUL << GNPTXFSIZ_NPTXFSA_Pos)                // 0x0000FFFF
#define GNPTXFSIZ_NPTXFSA                GNPTXFSIZ_NPTXFSA_Msk                    // Nonperiodic transmit RAM start address
#define GNPTXFSIZ_NPTXFD_Pos             (16U)
#define GNPTXFSIZ_NPTXFD_Msk             (0xFFFFUL << GNPTXFSIZ_NPTXFD_Pos)                 // 0xFFFF0000
#define GNPTXFSIZ_NPTXFD                 GNPTXFSIZ_NPTXFD_Msk                     // Nonperiodic TxFIFO depth
#define DIEPTXF0_TX0FSA_Pos              (0U)
#define DIEPTXF0_TX0FSA_Msk              (0xFFFFUL << DIEPTXF0_TX0FSA_Pos)                 // 0x0000FFFF
#define DIEPTXF0_TX0FSA                  DIEPTXF0_TX0FSA_Msk                      // Endpoint 0 transmit RAM start address
#define DIEPTXF0_TX0FD_Pos               (16U)
#define DIEPTXF0_TX0FD_Msk               (0xFFFFUL << DIEPTXF0_TX0FD_Pos)                  // 0xFFFF0000
#define DIEPTXF0_TX0FD                   DIEPTXF0_TX0FD_Msk                       // Endpoint 0 TxFIFO depth

/********************  Bit definition for DVBUSPULSE register  ********************/
#define DVBUSPULSE_DVBUSP_Pos            (0U)
#define DVBUSPULSE_DVBUSP_Msk            (0xFFFUL << DVBUSPULSE_DVBUSP_Pos)       // 0x00000FFF
#define DVBUSPULSE_DVBUSP                DVBUSPULSE_DVBUSP_Msk                    // Device VBUS pulsing time

/********************  Bit definition for GNPTXSTS register  ********************/
#define GNPTXSTS_NPTXFSAV_Pos            (0U)
#define GNPTXSTS_NPTXFSAV_Msk            (0xFFFFUL << GNPTXSTS_NPTXFSAV_Pos)      // 0x0000FFFF
#define GNPTXSTS_NPTXFSAV                GNPTXSTS_NPTXFSAV_Msk                    // Nonperiodic TxFIFO space available

#define GNPTXSTS_NPTQXSAV_Pos            (16U)
#define GNPTXSTS_NPTQXSAV_Msk            (0xFFUL << GNPTXSTS_NPTQXSAV_Pos)        // 0x00FF0000
#define GNPTXSTS_NPTQXSAV                GNPTXSTS_NPTQXSAV_Msk                    // Nonperiodic transmit request queue space available
#define GNPTXSTS_NPTQXSAV_0              (0x01UL << GNPTXSTS_NPTQXSAV_Pos)        // 0x00010000
#define GNPTXSTS_NPTQXSAV_1              (0x02UL << GNPTXSTS_NPTQXSAV_Pos)        // 0x00020000
#define GNPTXSTS_NPTQXSAV_2              (0x04UL << GNPTXSTS_NPTQXSAV_Pos)        // 0x00040000
#define GNPTXSTS_NPTQXSAV_3              (0x08UL << GNPTXSTS_NPTQXSAV_Pos)        // 0x00080000
#define GNPTXSTS_NPTQXSAV_4              (0x10UL << GNPTXSTS_NPTQXSAV_Pos)        // 0x00100000
#define GNPTXSTS_NPTQXSAV_5              (0x20UL << GNPTXSTS_NPTQXSAV_Pos)        // 0x00200000
#define GNPTXSTS_NPTQXSAV_6              (0x40UL << GNPTXSTS_NPTQXSAV_Pos)        // 0x00400000
#define GNPTXSTS_NPTQXSAV_7              (0x80UL << GNPTXSTS_NPTQXSAV_Pos)        // 0x00800000

#define GNPTXSTS_NPTXQTOP_Pos            (24U)
#define GNPTXSTS_NPTXQTOP_Msk            (0x7FUL << GNPTXSTS_NPTXQTOP_Pos)        // 0x7F000000
#define GNPTXSTS_NPTXQTOP                GNPTXSTS_NPTXQTOP_Msk                    // Top of the nonperiodic transmit request queue
#define GNPTXSTS_NPTXQTOP_0              (0x01UL << GNPTXSTS_NPTXQTOP_Pos)        // 0x01000000
#define GNPTXSTS_NPTXQTOP_1              (0x02UL << GNPTXSTS_NPTXQTOP_Pos)        // 0x02000000
#define GNPTXSTS_NPTXQTOP_2              (0x04UL << GNPTXSTS_NPTXQTOP_Pos)        // 0x04000000
#define GNPTXSTS_NPTXQTOP_3              (0x08UL << GNPTXSTS_NPTXQTOP_Pos)        // 0x08000000
#define GNPTXSTS_NPTXQTOP_4              (0x10UL << GNPTXSTS_NPTXQTOP_Pos)        // 0x10000000
#define GNPTXSTS_NPTXQTOP_5              (0x20UL << GNPTXSTS_NPTXQTOP_Pos)        // 0x20000000
#define GNPTXSTS_NPTXQTOP_6              (0x40UL << GNPTXSTS_NPTXQTOP_Pos)        // 0x40000000

/********************  Bit definition for DTHRCTL register  ********************/
#define DTHRCTL_NONISOTHREN_Pos          (0U)
#define DTHRCTL_NONISOTHREN_Msk          (0x1UL << DTHRCTL_NONISOTHREN_Pos)       // 0x00000001
#define DTHRCTL_NONISOTHREN              DTHRCTL_NONISOTHREN_Msk                  // Nonisochronous IN endpoints threshold enable
#define DTHRCTL_ISOTHREN_Pos             (1U)
#define DTHRCTL_ISOTHREN_Msk             (0x1UL << DTHRCTL_ISOTHREN_Pos)          // 0x00000002
#define DTHRCTL_ISOTHREN                 DTHRCTL_ISOTHREN_Msk                     // ISO IN endpoint threshold enable

#define DTHRCTL_TXTHRLEN_Pos             (2U)
#define DTHRCTL_TXTHRLEN_Msk             (0x1FFUL << DTHRCTL_TXTHRLEN_Pos)        // 0x000007FC
#define DTHRCTL_TXTHRLEN                 DTHRCTL_TXTHRLEN_Msk                     // Transmit threshold length
#define DTHRCTL_TXTHRLEN_0               (0x001UL << DTHRCTL_TXTHRLEN_Pos)        // 0x00000004
#define DTHRCTL_TXTHRLEN_1               (0x002UL << DTHRCTL_TXTHRLEN_Pos)        // 0x00000008
#define DTHRCTL_TXTHRLEN_2               (0x004UL << DTHRCTL_TXTHRLEN_Pos)        // 0x00000010
#define DTHRCTL_TXTHRLEN_3               (0x008UL << DTHRCTL_TXTHRLEN_Pos)        // 0x00000020
#define DTHRCTL_TXTHRLEN_4               (0x010UL << DTHRCTL_TXTHRLEN_Pos)        // 0x00000040
#define DTHRCTL_TXTHRLEN_5               (0x020UL << DTHRCTL_TXTHRLEN_Pos)        // 0x00000080
#define DTHRCTL_TXTHRLEN_6               (0x040UL << DTHRCTL_TXTHRLEN_Pos)        // 0x00000100
#define DTHRCTL_TXTHRLEN_7               (0x080UL << DTHRCTL_TXTHRLEN_Pos)        // 0x00000200
#define DTHRCTL_TXTHRLEN_8               (0x100UL << DTHRCTL_TXTHRLEN_Pos)        // 0x00000400
#define DTHRCTL_RXTHREN_Pos              (16U)
#define DTHRCTL_RXTHREN_Msk              (0x1UL << DTHRCTL_RXTHREN_Pos)           // 0x00010000
#define DTHRCTL_RXTHREN                  DTHRCTL_RXTHREN_Msk                      // Receive threshold enable

#define DTHRCTL_RXTHRLEN_Pos             (17U)
#define DTHRCTL_RXTHRLEN_Msk             (0x1FFUL << DTHRCTL_RXTHRLEN_Pos)        // 0x03FE0000
#define DTHRCTL_RXTHRLEN                 DTHRCTL_RXTHRLEN_Msk                     // Receive threshold length
#define DTHRCTL_RXTHRLEN_0               (0x001UL << DTHRCTL_RXTHRLEN_Pos)        // 0x00020000
#define DTHRCTL_RXTHRLEN_1               (0x002UL << DTHRCTL_RXTHRLEN_Pos)        // 0x00040000
#define DTHRCTL_RXTHRLEN_2               (0x004UL << DTHRCTL_RXTHRLEN_Pos)        // 0x00080000
#define DTHRCTL_RXTHRLEN_3               (0x008UL << DTHRCTL_RXTHRLEN_Pos)        // 0x00100000
#define DTHRCTL_RXTHRLEN_4               (0x010UL << DTHRCTL_RXTHRLEN_Pos)        // 0x00200000
#define DTHRCTL_RXTHRLEN_5               (0x020UL << DTHRCTL_RXTHRLEN_Pos)        // 0x00400000
#define DTHRCTL_RXTHRLEN_6               (0x040UL << DTHRCTL_RXTHRLEN_Pos)        // 0x00800000
#define DTHRCTL_RXTHRLEN_7               (0x080UL << DTHRCTL_RXTHRLEN_Pos)        // 0x01000000
#define DTHRCTL_RXTHRLEN_8               (0x100UL << DTHRCTL_RXTHRLEN_Pos)        // 0x02000000
#define DTHRCTL_ARPEN_Pos                (27U)
#define DTHRCTL_ARPEN_Msk                (0x1UL << DTHRCTL_ARPEN_Pos)             // 0x08000000
#define DTHRCTL_ARPEN                    DTHRCTL_ARPEN_Msk                        // Arbiter parking enable

/********************  Bit definition for DIEPEMPMSK register  ********************/
#define DIEPEMPMSK_INEPTXFEM_Pos         (0U)
#define DIEPEMPMSK_INEPTXFEM_Msk         (0xFFFFUL << DIEPEMPMSK_INEPTXFEM_Pos)   // 0x0000FFFF
#define DIEPEMPMSK_INEPTXFEM             DIEPEMPMSK_INEPTXFEM_Msk                 // IN EP Tx FIFO empty interrupt mask bits

/********************  Bit definition for DEACHINT register  ********************/
#define DEACHINT_IEP1INT_Pos             (1U)
#define DEACHINT_IEP1INT_Msk             (0x1UL << DEACHINT_IEP1INT_Pos)          // 0x00000002
#define DEACHINT_IEP1INT                 DEACHINT_IEP1INT_Msk                     // IN endpoint 1interrupt bit
#define DEACHINT_OEP1INT_Pos             (17U)
#define DEACHINT_OEP1INT_Msk             (0x1UL << DEACHINT_OEP1INT_Pos)          // 0x00020000
#define DEACHINT_OEP1INT                 DEACHINT_OEP1INT_Msk                     // OUT endpoint 1 interrupt bit

/********************  Bit definition for GCCFG register  ********************/
#define STM32_GCCFG_DCDET_Pos            (0U)
#define STM32_GCCFG_DCDET_Msk            (0x1UL << STM32_GCCFG_DCDET_Pos)         // 0x00000001
#define STM32_GCCFG_DCDET                STM32_GCCFG_DCDET_Msk                    // Data contact detection (DCD) status

#define STM32_GCCFG_PDET_Pos             (1U)
#define STM32_GCCFG_PDET_Msk             (0x1UL << STM32_GCCFG_PDET_Pos)          // 0x00000002
#define STM32_GCCFG_PDET                 STM32_GCCFG_PDET_Msk                     // Primary detection (PD) status

#define STM32_GCCFG_SDET_Pos             (2U)
#define STM32_GCCFG_SDET_Msk             (0x1UL << STM32_GCCFG_SDET_Pos)          // 0x00000004
#define STM32_GCCFG_SDET                 STM32_GCCFG_SDET_Msk                     // Secondary detection (SD) status

#define STM32_GCCFG_PS2DET_Pos           (3U)
#define STM32_GCCFG_PS2DET_Msk           (0x1UL << STM32_GCCFG_PS2DET_Pos)        // 0x00000008
#define STM32_GCCFG_PS2DET               STM32_GCCFG_PS2DET_Msk                   // DM pull-up detection status

#define STM32_GCCFG_PWRDWN_Pos           (16U)
#define STM32_GCCFG_PWRDWN_Msk           (0x1UL << STM32_GCCFG_PWRDWN_Pos)        // 0x00010000
#define STM32_GCCFG_PWRDWN               STM32_GCCFG_PWRDWN_Msk                   // Power down

#define STM32_GCCFG_BCDEN_Pos            (17U)
#define STM32_GCCFG_BCDEN_Msk            (0x1UL << STM32_GCCFG_BCDEN_Pos)         // 0x00020000
#define STM32_GCCFG_BCDEN                STM32_GCCFG_BCDEN_Msk                    // Battery charging detector (BCD) enable

#define STM32_GCCFG_DCDEN_Pos            (18U)
#define STM32_GCCFG_DCDEN_Msk            (0x1UL << STM32_GCCFG_DCDEN_Pos)         // 0x00040000
#define STM32_GCCFG_DCDEN                STM32_GCCFG_DCDEN_Msk                    // Data contact detection (DCD) mode enable*/

#define STM32_GCCFG_PDEN_Pos             (19U)
#define STM32_GCCFG_PDEN_Msk             (0x1UL << STM32_GCCFG_PDEN_Pos)          // 0x00080000
#define STM32_GCCFG_PDEN                 STM32_GCCFG_PDEN_Msk                     // Primary detection (PD) mode enable*/

#define STM32_GCCFG_SDEN_Pos             (20U)
#define STM32_GCCFG_SDEN_Msk             (0x1UL << STM32_GCCFG_SDEN_Pos)          // 0x00100000
#define STM32_GCCFG_SDEN                 STM32_GCCFG_SDEN_Msk                     // Secondary detection (SD) mode enable

#define STM32_GCCFG_VBDEN_Pos            (21U)
#define STM32_GCCFG_VBDEN_Msk            (0x1UL << STM32_GCCFG_VBDEN_Pos)         // 0x00200000
#define STM32_GCCFG_VBDEN                STM32_GCCFG_VBDEN_Msk                    // VBUS mode enable

#define STM32_GCCFG_OTGIDEN_Pos          (22U)
#define STM32_GCCFG_OTGIDEN_Msk          (0x1UL << STM32_GCCFG_OTGIDEN_Pos)       // 0x00400000
#define STM32_GCCFG_OTGIDEN              STM32_GCCFG_OTGIDEN_Msk                  // OTG Id enable

#define STM32_GCCFG_PHYHSEN_Pos          (23U)
#define STM32_GCCFG_PHYHSEN_Msk          (0x1UL << STM32_GCCFG_PHYHSEN_Pos)       // 0x00800000
#define STM32_GCCFG_PHYHSEN              STM32_GCCFG_PHYHSEN_Msk                  // HS PHY enable

// TODO stm32u5a5 SDEN is 22nd bit, conflict with 20th bit above
//#define STM32_GCCFG_SDEN_Pos                   (22U)
//#define STM32_GCCFG_SDEN_Msk                   (0x1U << STM32_GCCFG_SDEN_Pos)             // 0x00400000
//#define STM32_GCCFG_SDEN                       STM32_GCCFG_SDEN_Msk                       // Secondary detection (PD) mode enable

// TODO stm32u5a5 VBVALOVA is 23rd bit, conflict with PHYHSEN bit above
#define STM32_GCCFG_VBVALOVAL_Pos              (23U)
#define STM32_GCCFG_VBVALOVAL_Msk              (0x1U << STM32_GCCFG_VBVALOVAL_Pos)        // 0x00800000
#define STM32_GCCFG_VBVALOVAL                  STM32_GCCFG_VBVALOVAL_Msk                  // Value of VBUSVLDEXT0 femtoPHY input

#define STM32_GCCFG_VBVALEXTOEN_Pos            (24U)
#define STM32_GCCFG_VBVALEXTOEN_Msk            (0x1U << STM32_GCCFG_VBVALEXTOEN_Pos)      // 0x01000000
#define STM32_GCCFG_VBVALEXTOEN                STM32_GCCFG_VBVALEXTOEN_Msk                // Enables of VBUSVLDEXT0 femtoPHY input override

#define STM32_GCCFG_PULLDOWNEN_Pos             (25U)
#define STM32_GCCFG_PULLDOWNEN_Msk             (0x1U << STM32_GCCFG_PULLDOWNEN_Pos)       // 0x02000000
#define STM32_GCCFG_PULLDOWNEN                 STM32_GCCFG_PULLDOWNEN_Msk                 // Enables of femtoPHY pulldown resistors, used when ID PAD is disabled


/********************  Bit definition for DEACHINTMSK register  ********************/
#define DEACHINTMSK_IEP1INTM_Pos         (1U)
#define DEACHINTMSK_IEP1INTM_Msk         (0x1UL << DEACHINTMSK_IEP1INTM_Pos)      // 0x00000002
#define DEACHINTMSK_IEP1INTM             DEACHINTMSK_IEP1INTM_Msk                 // IN Endpoint 1 interrupt mask bit
#define DEACHINTMSK_OEP1INTM_Pos         (17U)
#define DEACHINTMSK_OEP1INTM_Msk         (0x1UL << DEACHINTMSK_OEP1INTM_Pos)      // 0x00020000
#define DEACHINTMSK_OEP1INTM             DEACHINTMSK_OEP1INTM_Msk                 // OUT Endpoint 1 interrupt mask bit

/********************  Bit definition for CID register  ********************/
#define CID_PRODUCT_ID_Pos               (0U)
#define CID_PRODUCT_ID_Msk               (0xFFFFFFFFUL << CID_PRODUCT_ID_Pos)     // 0xFFFFFFFF
#define CID_PRODUCT_ID                   CID_PRODUCT_ID_Msk                       // Product ID field

/********************  Bit definition for GLPMCFG register  ********************/
#define GLPMCFG_LPMEN_Pos                (0U)
#define GLPMCFG_LPMEN_Msk                (0x1UL << GLPMCFG_LPMEN_Pos)             // 0x00000001
#define GLPMCFG_LPMEN                    GLPMCFG_LPMEN_Msk                        // LPM support enable
#define GLPMCFG_LPMACK_Pos               (1U)
#define GLPMCFG_LPMACK_Msk               (0x1UL << GLPMCFG_LPMACK_Pos)            // 0x00000002
#define GLPMCFG_LPMACK                   GLPMCFG_LPMACK_Msk                       // LPM Token acknowledge enable
#define GLPMCFG_BESL_Pos                 (2U)
#define GLPMCFG_BESL_Msk                 (0xFUL << GLPMCFG_BESL_Pos)              // 0x0000003C
#define GLPMCFG_BESL                     GLPMCFG_BESL_Msk                         // BESL value received with last ACKed LPM Token
#define GLPMCFG_REMWAKE_Pos              (6U)
#define GLPMCFG_REMWAKE_Msk              (0x1UL << GLPMCFG_REMWAKE_Pos)           // 0x00000040
#define GLPMCFG_REMWAKE                  GLPMCFG_REMWAKE_Msk                      // bRemoteWake value received with last ACKed LPM Token
#define GLPMCFG_L1SSEN_Pos               (7U)
#define GLPMCFG_L1SSEN_Msk               (0x1UL << GLPMCFG_L1SSEN_Pos)            // 0x00000080
#define GLPMCFG_L1SSEN                   GLPMCFG_L1SSEN_Msk                       // L1 shallow sleep enable
#define GLPMCFG_BESLTHRS_Pos             (8U)
#define GLPMCFG_BESLTHRS_Msk             (0xFUL << GLPMCFG_BESLTHRS_Pos)          // 0x00000F00
#define GLPMCFG_BESLTHRS                 GLPMCFG_BESLTHRS_Msk                     // BESL threshold
#define GLPMCFG_L1DSEN_Pos               (12U)
#define GLPMCFG_L1DSEN_Msk               (0x1UL << GLPMCFG_L1DSEN_Pos)            // 0x00001000
#define GLPMCFG_L1DSEN                   GLPMCFG_L1DSEN_Msk                       // L1 deep sleep enable
#define GLPMCFG_LPMRSP_Pos               (13U)
#define GLPMCFG_LPMRSP_Msk               (0x3UL << GLPMCFG_LPMRSP_Pos)            // 0x00006000
#define GLPMCFG_LPMRSP                   GLPMCFG_LPMRSP_Msk                       // LPM response
#define GLPMCFG_SLPSTS_Pos               (15U)
#define GLPMCFG_SLPSTS_Msk               (0x1UL << GLPMCFG_SLPSTS_Pos)            // 0x00008000
#define GLPMCFG_SLPSTS                   GLPMCFG_SLPSTS_Msk                       // Port sleep status
#define GLPMCFG_L1RSMOK_Pos              (16U)
#define GLPMCFG_L1RSMOK_Msk              (0x1UL << GLPMCFG_L1RSMOK_Pos)           // 0x00010000
#define GLPMCFG_L1RSMOK                  GLPMCFG_L1RSMOK_Msk                      // Sleep State Resume OK
#define GLPMCFG_LPMCHIDX_Pos             (17U)
#define GLPMCFG_LPMCHIDX_Msk             (0xFUL << GLPMCFG_LPMCHIDX_Pos)          // 0x001E0000
#define GLPMCFG_LPMCHIDX                 GLPMCFG_LPMCHIDX_Msk                     // LPM Channel Index
#define GLPMCFG_LPMRCNT_Pos              (21U)
#define GLPMCFG_LPMRCNT_Msk              (0x7UL << GLPMCFG_LPMRCNT_Pos)           // 0x00E00000
#define GLPMCFG_LPMRCNT                  GLPMCFG_LPMRCNT_Msk                      // LPM retry count
#define GLPMCFG_SNDLPM_Pos               (24U)
#define GLPMCFG_SNDLPM_Msk               (0x1UL << GLPMCFG_SNDLPM_Pos)            // 0x01000000
#define GLPMCFG_SNDLPM                   GLPMCFG_SNDLPM_Msk                       // Send LPM transaction
#define GLPMCFG_LPMRCNTSTS_Pos           (25U)
#define GLPMCFG_LPMRCNTSTS_Msk           (0x7UL << GLPMCFG_LPMRCNTSTS_Pos)        // 0x0E000000
#define GLPMCFG_LPMRCNTSTS               GLPMCFG_LPMRCNTSTS_Msk                   // LPM retry count status
#define GLPMCFG_ENBESL_Pos               (28U)
#define GLPMCFG_ENBESL_Msk               (0x1UL << GLPMCFG_ENBESL_Pos)            // 0x10000000
#define GLPMCFG_ENBESL                   GLPMCFG_ENBESL_Msk                       // Enable best effort service latency

// GDFIFOCFG
#define GDFIFOCFG_EPINFOBASE_MASK   (0xffff << 16)
#define GDFIFOCFG_EPINFOBASE_SHIFT  16
#define GDFIFOCFG_GDFIFOCFG_MASK    (0xffff << 0)
#define GDFIFOCFG_GDFIFOCFG_SHIFT   0

/********************  Bit definition for DIEPEACHMSK1 register  ********************/
#define DIEPEACHMSK1_XFRCM_Pos           (0U)
#define DIEPEACHMSK1_XFRCM_Msk           (0x1UL << DIEPEACHMSK1_XFRCM_Pos)        // 0x00000001
#define DIEPEACHMSK1_XFRCM               DIEPEACHMSK1_XFRCM_Msk                   // Transfer completed interrupt mask
#define DIEPEACHMSK1_EPDM_Pos            (1U)
#define DIEPEACHMSK1_EPDM_Msk            (0x1UL << DIEPEACHMSK1_EPDM_Pos)         // 0x00000002
#define DIEPEACHMSK1_EPDM                DIEPEACHMSK1_EPDM_Msk                    // Endpoint disabled interrupt mask
#define DIEPEACHMSK1_TOM_Pos             (3U)
#define DIEPEACHMSK1_TOM_Msk             (0x1UL << DIEPEACHMSK1_TOM_Pos)          // 0x00000008
#define DIEPEACHMSK1_TOM                 DIEPEACHMSK1_TOM_Msk                     // Timeout condition mask (nonisochronous endpoints)
#define DIEPEACHMSK1_ITTXFEMSK_Pos       (4U)
#define DIEPEACHMSK1_ITTXFEMSK_Msk       (0x1UL << DIEPEACHMSK1_ITTXFEMSK_Pos)    // 0x00000010
#define DIEPEACHMSK1_ITTXFEMSK           DIEPEACHMSK1_ITTXFEMSK_Msk               // IN token received when TxFIFO empty mask
#define DIEPEACHMSK1_INEPNMM_Pos         (5U)
#define DIEPEACHMSK1_INEPNMM_Msk         (0x1UL << DIEPEACHMSK1_INEPNMM_Pos)      // 0x00000020
#define DIEPEACHMSK1_INEPNMM             DIEPEACHMSK1_INEPNMM_Msk                 // IN token received with EP mismatch mask
#define DIEPEACHMSK1_INEPNEM_Pos         (6U)
#define DIEPEACHMSK1_INEPNEM_Msk         (0x1UL << DIEPEACHMSK1_INEPNEM_Pos)      // 0x00000040
#define DIEPEACHMSK1_INEPNEM             DIEPEACHMSK1_INEPNEM_Msk                 // IN endpoint NAK effective mask
#define DIEPEACHMSK1_TXFURM_Pos          (8U)
#define DIEPEACHMSK1_TXFURM_Msk          (0x1UL << DIEPEACHMSK1_TXFURM_Pos)       // 0x00000100
#define DIEPEACHMSK1_TXFURM              DIEPEACHMSK1_TXFURM_Msk                  // FIFO underrun mask
#define DIEPEACHMSK1_BIM_Pos             (9U)
#define DIEPEACHMSK1_BIM_Msk             (0x1UL << DIEPEACHMSK1_BIM_Pos)          // 0x00000200
#define DIEPEACHMSK1_BIM                 DIEPEACHMSK1_BIM_Msk                     // BNA interrupt mask
#define DIEPEACHMSK1_NAKM_Pos            (13U)
#define DIEPEACHMSK1_NAKM_Msk            (0x1UL << DIEPEACHMSK1_NAKM_Pos)         // 0x00002000
#define DIEPEACHMSK1_NAKM                DIEPEACHMSK1_NAKM_Msk                    // NAK interrupt mask

/********************  Bit definition for HPRT register  ********************/
#define HPRT_CONN_STATUS_Pos           (0U)
#define HPRT_CONN_STATUS_Msk           (0x1UL << HPRT_CONN_STATUS_Pos)         // 0x00000001
#define HPRT_CONN_STATUS               HPRT_CONN_STATUS_Msk                    // Port connect status
#define HPRT_CONN_DETECT_Pos           (1U)
#define HPRT_CONN_DETECT_Msk           (0x1UL << HPRT_CONN_DETECT_Pos)         // 0x00000002
#define HPRT_CONN_DETECT               HPRT_CONN_DETECT_Msk                    // Port connect detected
#define HPRT_ENABLE_Pos                (2U)
#define HPRT_ENABLE_Msk                (0x1UL << HPRT_ENABLE_Pos)              // 0x00000004
#define HPRT_ENABLE                    HPRT_ENABLE_Msk                         // Port enable
#define HPRT_ENABLE_CHANGE_Pos         (3U)
#define HPRT_ENABLE_CHANGE_Msk         (0x1UL << HPRT_ENABLE_CHANGE_Pos)       // 0x00000008
#define HPRT_ENABLE_CHANGE             HPRT_ENABLE_CHANGE_Msk                  // Port enable/disable change
#define HPRT_OVER_CURRENT_ACTIVE_Pos   (4U)
#define HPRT_OVER_CURRENT_ACTIVE_Msk   (0x1UL << HPRT_OVER_CURRENT_ACTIVE_Pos) // 0x00000010
#define HPRT_OVER_CURRENT_ACTIVE       HPRT_OVER_CURRENT_ACTIVE_Msk            // Port overcurrent active
#define HPRT_OVER_CURRENT_CHANGE_Pos   (5U)
#define HPRT_OVER_CURRENT_CHANGE_Msk   (0x1UL << HPRT_OVER_CURRENT_CHANGE_Pos) // 0x00000020
#define HPRT_OVER_CURRENT_CHANGE       HPRT_OVER_CURRENT_CHANGE_Msk            // Port overcurrent change
#define HPRT_RESUME_Pos                (6U)
#define HPRT_RESUME_Msk                (0x1UL << HPRT_RESUME_Pos)              // 0x00000040
#define HPRT_RESUME                    HPRT_RESUME_Msk                         // Port resume
#define HPRT_SUSPEND_Pos               (7U)
#define HPRT_SUSPEND_Msk               (0x1UL << HPRT_SUSPEND_Pos)             // 0x00000080
#define HPRT_SUSPEND                   HPRT_SUSPEND_Msk                        // Port suspend
#define HPRT_RESET_Pos                 (8U)
#define HPRT_RESET_Msk                 (0x1UL << HPRT_RESET_Pos)               // 0x00000100
#define HPRT_RESET                     HPRT_RESET_Msk                          // Port reset
#define HPRT_LINE_STATUS_Pos           (10U)
#define HPRT_LINE_STATUS_Msk           (0x3UL << HPRT_LINE_STATUS_Pos)         // 0x00000C00
#define HPRT_LINE_STATUS               HPRT_LINE_STATUS_Msk                    // Port line status
#define HPRT_LINE_STATUS_0             (0x1UL << HPRT_LINE_STATUS_Pos)         // 0x00000400
#define HPRT_LINE_STATUS_1             (0x2UL << HPRT_LINE_STATUS_Pos)         // 0x00000800
#define HPRT_POWER_Pos                 (12U)
#define HPRT_POWER_Msk                 (0x1UL << HPRT_POWER_Pos)               // 0x00001000
#define HPRT_POWER                     HPRT_POWER_Msk                          // Port power
#define HPRT_TEST_CONTROL_Pos          (13U)
#define HPRT_TEST_CONTROL_Msk          (0xFUL << HPRT_TEST_CONTROL_Pos)        // 0x0001E000
#define HPRT_TEST_CONTROL              HPRT_TEST_CONTROL_Msk                   // Port test control
#define HPRT_TEST_CONTROL_0            (0x1UL << HPRT_TEST_CONTROL_Pos)        // 0x00002000
#define HPRT_TEST_CONTROL_1            (0x2UL << HPRT_TEST_CONTROL_Pos)        // 0x00004000
#define HPRT_TEST_CONTROL_2            (0x4UL << HPRT_TEST_CONTROL_Pos)        // 0x00008000
#define HPRT_TEST_CONTROL_3            (0x8UL << HPRT_TEST_CONTROL_Pos)        // 0x00010000
#define HPRT_SPEED_Pos                 (17U)
#define HPRT_SPEED_Msk                 (0x3UL << HPRT_SPEED_Pos)               // 0x00060000
#define HPRT_SPEED                     HPRT_SPEED_Msk                          // Port speed
#define HPRT_SPEED_0                   (0x1UL << HPRT_SPEED_Pos)               // 0x00020000
#define HPRT_SPEED_1                   (0x2UL << HPRT_SPEED_Pos)               // 0x00040000

/********************  Bit definition for DOEPEACHMSK1 register  ********************/
#define DOEPEACHMSK1_XFRCM_Pos           (0U)
#define DOEPEACHMSK1_XFRCM_Msk           (0x1UL << DOEPEACHMSK1_XFRCM_Pos)        // 0x00000001
#define DOEPEACHMSK1_XFRCM               DOEPEACHMSK1_XFRCM_Msk                   // Transfer completed interrupt mask
#define DOEPEACHMSK1_EPDM_Pos            (1U)
#define DOEPEACHMSK1_EPDM_Msk            (0x1UL << DOEPEACHMSK1_EPDM_Pos)         // 0x00000002
#define DOEPEACHMSK1_EPDM                DOEPEACHMSK1_EPDM_Msk                    // Endpoint disabled interrupt mask
#define DOEPEACHMSK1_TOM_Pos             (3U)
#define DOEPEACHMSK1_TOM_Msk             (0x1UL << DOEPEACHMSK1_TOM_Pos)          // 0x00000008
#define DOEPEACHMSK1_TOM                 DOEPEACHMSK1_TOM_Msk                     // Timeout condition mask
#define DOEPEACHMSK1_ITTXFEMSK_Pos       (4U)
#define DOEPEACHMSK1_ITTXFEMSK_Msk       (0x1UL << DOEPEACHMSK1_ITTXFEMSK_Pos)    // 0x00000010
#define DOEPEACHMSK1_ITTXFEMSK           DOEPEACHMSK1_ITTXFEMSK_Msk               // IN token received when TxFIFO empty mask
#define DOEPEACHMSK1_INEPNMM_Pos         (5U)
#define DOEPEACHMSK1_INEPNMM_Msk         (0x1UL << DOEPEACHMSK1_INEPNMM_Pos)      // 0x00000020
#define DOEPEACHMSK1_INEPNMM             DOEPEACHMSK1_INEPNMM_Msk                 // IN token received with EP mismatch mask
#define DOEPEACHMSK1_INEPNEM_Pos         (6U)
#define DOEPEACHMSK1_INEPNEM_Msk         (0x1UL << DOEPEACHMSK1_INEPNEM_Pos)      // 0x00000040
#define DOEPEACHMSK1_INEPNEM             DOEPEACHMSK1_INEPNEM_Msk                 // IN endpoint NAK effective mask
#define DOEPEACHMSK1_TXFURM_Pos          (8U)
#define DOEPEACHMSK1_TXFURM_Msk          (0x1UL << DOEPEACHMSK1_TXFURM_Pos)       // 0x00000100
#define DOEPEACHMSK1_TXFURM              DOEPEACHMSK1_TXFURM_Msk                  // OUT packet error mask
#define DOEPEACHMSK1_BIM_Pos             (9U)
#define DOEPEACHMSK1_BIM_Msk             (0x1UL << DOEPEACHMSK1_BIM_Pos)          // 0x00000200
#define DOEPEACHMSK1_BIM                 DOEPEACHMSK1_BIM_Msk                     // BNA interrupt mask
#define DOEPEACHMSK1_BERRM_Pos           (12U)
#define DOEPEACHMSK1_BERRM_Msk           (0x1UL << DOEPEACHMSK1_BERRM_Pos)        // 0x00001000
#define DOEPEACHMSK1_BERRM               DOEPEACHMSK1_BERRM_Msk                   // Bubble error interrupt mask
#define DOEPEACHMSK1_NAKM_Pos            (13U)
#define DOEPEACHMSK1_NAKM_Msk            (0x1UL << DOEPEACHMSK1_NAKM_Pos)         // 0x00002000
#define DOEPEACHMSK1_NAKM                DOEPEACHMSK1_NAKM_Msk                    // NAK interrupt mask
#define DOEPEACHMSK1_NYETM_Pos           (14U)
#define DOEPEACHMSK1_NYETM_Msk           (0x1UL << DOEPEACHMSK1_NYETM_Pos)        // 0x00004000
#define DOEPEACHMSK1_NYETM               DOEPEACHMSK1_NYETM_Msk                   // NYET interrupt mask

/********************  Bit definition for HPTXFSIZ register  ********************/
#define HPTXFSIZ_PTXSA_Pos               (0U)
#define HPTXFSIZ_PTXSA_Msk               (0xFFFFUL << HPTXFSIZ_PTXSA_Pos)         // 0x0000FFFF
#define HPTXFSIZ_PTXSA                   HPTXFSIZ_PTXSA_Msk                       // Host periodic TxFIFO start address
#define HPTXFSIZ_PTXFD_Pos               (16U)
#define HPTXFSIZ_PTXFD_Msk               (0xFFFFUL << HPTXFSIZ_PTXFD_Pos)         // 0xFFFF0000
#define HPTXFSIZ_PTXFD                   HPTXFSIZ_PTXFD_Msk                       // Host periodic TxFIFO depth

/********************  Bit definition for DIEPCTL register  ********************/
#define DIEPCTL_MPSIZ_Pos                (0U)
#define DIEPCTL_MPSIZ_Msk                (0x7FFUL << DIEPCTL_MPSIZ_Pos)           // 0x000007FF
#define DIEPCTL_MPSIZ                    DIEPCTL_MPSIZ_Msk                        // Maximum packet size
#define DIEPCTL_USBAEP_Pos               (15U)
#define DIEPCTL_USBAEP_Msk               (0x1UL << DIEPCTL_USBAEP_Pos)            // 0x00008000
#define DIEPCTL_USBAEP                   DIEPCTL_USBAEP_Msk                       // USB active endpoint
#define DIEPCTL_EONUM_DPID_Pos           (16U)
#define DIEPCTL_EONUM_DPID_Msk           (0x1UL << DIEPCTL_EONUM_DPID_Pos)        // 0x00010000
#define DIEPCTL_EONUM_DPID               DIEPCTL_EONUM_DPID_Msk                   // Even/odd frame
#define DIEPCTL_NAKSTS_Pos               (17U)
#define DIEPCTL_NAKSTS_Msk               (0x1UL << DIEPCTL_NAKSTS_Pos)            // 0x00020000
#define DIEPCTL_NAKSTS                   DIEPCTL_NAKSTS_Msk                       // NAK status

#define DIEPCTL_EPTYP_Pos                (18U)
#define DIEPCTL_EPTYP_Msk                (0x3UL << DIEPCTL_EPTYP_Pos)             // 0x000C0000
#define DIEPCTL_EPTYP                    DIEPCTL_EPTYP_Msk                        // Endpoint type
#define DIEPCTL_EPTYP_0                  (0x1UL << DIEPCTL_EPTYP_Pos)             // 0x00040000
#define DIEPCTL_EPTYP_1                  (0x2UL << DIEPCTL_EPTYP_Pos)             // 0x00080000
#define DIEPCTL_STALL_Pos                (21U)
#define DIEPCTL_STALL_Msk                (0x1UL << DIEPCTL_STALL_Pos)             // 0x00200000
#define DIEPCTL_STALL                    DIEPCTL_STALL_Msk                        // STALL handshake

#define DIEPCTL_TXFNUM_Pos               (22U)
#define DIEPCTL_TXFNUM_Msk               (0xFUL << DIEPCTL_TXFNUM_Pos)            // 0x03C00000
#define DIEPCTL_TXFNUM                   DIEPCTL_TXFNUM_Msk                       // TxFIFO number
#define DIEPCTL_TXFNUM_0                 (0x1UL << DIEPCTL_TXFNUM_Pos)            // 0x00400000
#define DIEPCTL_TXFNUM_1                 (0x2UL << DIEPCTL_TXFNUM_Pos)            // 0x00800000
#define DIEPCTL_TXFNUM_2                 (0x4UL << DIEPCTL_TXFNUM_Pos)            // 0x01000000
#define DIEPCTL_TXFNUM_3                 (0x8UL << DIEPCTL_TXFNUM_Pos)            // 0x02000000
#define DIEPCTL_CNAK_Pos                 (26U)
#define DIEPCTL_CNAK_Msk                 (0x1UL << DIEPCTL_CNAK_Pos)              // 0x04000000
#define DIEPCTL_CNAK                     DIEPCTL_CNAK_Msk                         // Clear NAK
#define DIEPCTL_SNAK_Pos                 (27U)
#define DIEPCTL_SNAK_Msk                 (0x1UL << DIEPCTL_SNAK_Pos)              // 0x08000000
#define DIEPCTL_SNAK                     DIEPCTL_SNAK_Msk                         // Set NAK
#define DIEPCTL_SD0PID_SEVNFRM_Pos       (28U)
#define DIEPCTL_SD0PID_SEVNFRM_Msk       (0x1UL << DIEPCTL_SD0PID_SEVNFRM_Pos)    // 0x10000000
#define DIEPCTL_SD0PID_SEVNFRM           DIEPCTL_SD0PID_SEVNFRM_Msk               // Set DATA0 PID
#define DIEPCTL_SODDFRM_Pos              (29U)
#define DIEPCTL_SODDFRM_Msk              (0x1UL << DIEPCTL_SODDFRM_Pos)           // 0x20000000
#define DIEPCTL_SODDFRM                  DIEPCTL_SODDFRM_Msk                      // Set odd frame
#define DIEPCTL_EPDIS_Pos                (30U)
#define DIEPCTL_EPDIS_Msk                (0x1UL << DIEPCTL_EPDIS_Pos)             // 0x40000000
#define DIEPCTL_EPDIS                    DIEPCTL_EPDIS_Msk                        // Endpoint disable
#define DIEPCTL_EPENA_Pos                (31U)
#define DIEPCTL_EPENA_Msk                (0x1UL << DIEPCTL_EPENA_Pos)             // 0x80000000
#define DIEPCTL_EPENA                    DIEPCTL_EPENA_Msk                        // Endpoint enable

/********************  Bit definition for HCCHAR register  ********************/
#define HCCHAR_MPSIZ_Pos                 (0U)
#define HCCHAR_MPSIZ_Msk                 (0x7FFUL << HCCHAR_MPSIZ_Pos)            // 0x000007FF
#define HCCHAR_MPSIZ                     HCCHAR_MPSIZ_Msk                         // Maximum packet size

#define HCCHAR_EPNUM_Pos                 (11U)
#define HCCHAR_EPNUM_Msk                 (0xFUL << HCCHAR_EPNUM_Pos)              // 0x00007800
#define HCCHAR_EPNUM                     HCCHAR_EPNUM_Msk                         // Endpoint number
#define HCCHAR_EPNUM_0                   (0x1UL << HCCHAR_EPNUM_Pos)              // 0x00000800
#define HCCHAR_EPNUM_1                   (0x2UL << HCCHAR_EPNUM_Pos)              // 0x00001000
#define HCCHAR_EPNUM_2                   (0x4UL << HCCHAR_EPNUM_Pos)              // 0x00002000
#define HCCHAR_EPNUM_3                   (0x8UL << HCCHAR_EPNUM_Pos)              // 0x00004000
#define HCCHAR_EPDIR_Pos                 (15U)
#define HCCHAR_EPDIR_Msk                 (0x1UL << HCCHAR_EPDIR_Pos)              // 0x00008000
#define HCCHAR_EPDIR                     HCCHAR_EPDIR_Msk                         // Endpoint direction
#define HCCHAR_LSDEV_Pos                 (17U)
#define HCCHAR_LSDEV_Msk                 (0x1UL << HCCHAR_LSDEV_Pos)              // 0x00020000
#define HCCHAR_LSDEV                     HCCHAR_LSDEV_Msk                         // Low-speed device

#define HCCHAR_EPTYP_Pos                 (18U)
#define HCCHAR_EPTYP_Msk                 (0x3UL << HCCHAR_EPTYP_Pos)              // 0x000C0000
#define HCCHAR_EPTYP                     HCCHAR_EPTYP_Msk                         // Endpoint type
#define HCCHAR_EPTYP_0                   (0x1UL << HCCHAR_EPTYP_Pos)              // 0x00040000
#define HCCHAR_EPTYP_1                   (0x2UL << HCCHAR_EPTYP_Pos)              // 0x00080000

#define HCCHAR_MC_Pos                    (20U)
#define HCCHAR_MC_Msk                    (0x3UL << HCCHAR_MC_Pos)                 // 0x00300000
#define HCCHAR_MC                        HCCHAR_MC_Msk                            // Multi Count (MC) / Error Count (EC)
#define HCCHAR_MC_0                      (0x1UL << HCCHAR_MC_Pos)                 // 0x00100000
#define HCCHAR_MC_1                      (0x2UL << HCCHAR_MC_Pos)                 // 0x00200000

#define HCCHAR_DAD_Pos                   (22U)
#define HCCHAR_DAD_Msk                   (0x7FUL << HCCHAR_DAD_Pos)               // 0x1FC00000
#define HCCHAR_DAD                       HCCHAR_DAD_Msk                           // Device address
#define HCCHAR_DAD_0                     (0x01UL << HCCHAR_DAD_Pos)               // 0x00400000
#define HCCHAR_DAD_1                     (0x02UL << HCCHAR_DAD_Pos)               // 0x00800000
#define HCCHAR_DAD_2                     (0x04UL << HCCHAR_DAD_Pos)               // 0x01000000
#define HCCHAR_DAD_3                     (0x08UL << HCCHAR_DAD_Pos)               // 0x02000000
#define HCCHAR_DAD_4                     (0x10UL << HCCHAR_DAD_Pos)               // 0x04000000
#define HCCHAR_DAD_5                     (0x20UL << HCCHAR_DAD_Pos)               // 0x08000000
#define HCCHAR_DAD_6                     (0x40UL << HCCHAR_DAD_Pos)               // 0x10000000
#define HCCHAR_ODDFRM_Pos                (29U)
#define HCCHAR_ODDFRM_Msk                (0x1UL << HCCHAR_ODDFRM_Pos)             // 0x20000000
#define HCCHAR_ODDFRM                    HCCHAR_ODDFRM_Msk                        // Odd frame
#define HCCHAR_CHDIS_Pos                 (30U)
#define HCCHAR_CHDIS_Msk                 (0x1UL << HCCHAR_CHDIS_Pos)              // 0x40000000
#define HCCHAR_CHDIS                     HCCHAR_CHDIS_Msk                         // Channel disable
#define HCCHAR_CHENA_Pos                 (31U)
#define HCCHAR_CHENA_Msk                 (0x1UL << HCCHAR_CHENA_Pos)              // 0x80000000
#define HCCHAR_CHENA                     HCCHAR_CHENA_Msk                         // Channel enable

/********************  Bit definition for HCSPLT register  ********************/

#define HCSPLT_PRTADDR_Pos               (0U)
#define HCSPLT_PRTADDR_Msk               (0x7FUL << HCSPLT_PRTADDR_Pos)           // 0x0000007F
#define HCSPLT_PRTADDR                   HCSPLT_PRTADDR_Msk                       // Port address
#define HCSPLT_PRTADDR_0                 (0x01UL << HCSPLT_PRTADDR_Pos)           // 0x00000001
#define HCSPLT_PRTADDR_1                 (0x02UL << HCSPLT_PRTADDR_Pos)           // 0x00000002
#define HCSPLT_PRTADDR_2                 (0x04UL << HCSPLT_PRTADDR_Pos)           // 0x00000004
#define HCSPLT_PRTADDR_3                 (0x08UL << HCSPLT_PRTADDR_Pos)           // 0x00000008
#define HCSPLT_PRTADDR_4                 (0x10UL << HCSPLT_PRTADDR_Pos)           // 0x00000010
#define HCSPLT_PRTADDR_5                 (0x20UL << HCSPLT_PRTADDR_Pos)           // 0x00000020
#define HCSPLT_PRTADDR_6                 (0x40UL << HCSPLT_PRTADDR_Pos)           // 0x00000040

#define HCSPLT_HUBADDR_Pos               (7U)
#define HCSPLT_HUBADDR_Msk               (0x7FUL << HCSPLT_HUBADDR_Pos)           // 0x00003F80
#define HCSPLT_HUBADDR                   HCSPLT_HUBADDR_Msk                       // Hub address
#define HCSPLT_HUBADDR_0                 (0x01UL << HCSPLT_HUBADDR_Pos)           // 0x00000080
#define HCSPLT_HUBADDR_1                 (0x02UL << HCSPLT_HUBADDR_Pos)           // 0x00000100
#define HCSPLT_HUBADDR_2                 (0x04UL << HCSPLT_HUBADDR_Pos)           // 0x00000200
#define HCSPLT_HUBADDR_3                 (0x08UL << HCSPLT_HUBADDR_Pos)           // 0x00000400
#define HCSPLT_HUBADDR_4                 (0x10UL << HCSPLT_HUBADDR_Pos)           // 0x00000800
#define HCSPLT_HUBADDR_5                 (0x20UL << HCSPLT_HUBADDR_Pos)           // 0x00001000
#define HCSPLT_HUBADDR_6                 (0x40UL << HCSPLT_HUBADDR_Pos)           // 0x00002000

#define HCSPLT_XACTPOS_Pos               (14U)
#define HCSPLT_XACTPOS_Msk               (0x3UL << HCSPLT_XACTPOS_Pos)            // 0x0000C000
#define HCSPLT_XACTPOS                   HCSPLT_XACTPOS_Msk                       // XACTPOS
#define HCSPLT_XACTPOS_0                 (0x1UL << HCSPLT_XACTPOS_Pos)            // 0x00004000
#define HCSPLT_XACTPOS_1                 (0x2UL << HCSPLT_XACTPOS_Pos)            // 0x00008000
#define HCSPLT_COMPLSPLT_Pos             (16U)
#define HCSPLT_COMPLSPLT_Msk             (0x1UL << HCSPLT_COMPLSPLT_Pos)          // 0x00010000
#define HCSPLT_COMPLSPLT                 HCSPLT_COMPLSPLT_Msk                     // Do complete split
#define HCSPLT_SPLITEN_Pos               (31U)
#define HCSPLT_SPLITEN_Msk               (0x1UL << HCSPLT_SPLITEN_Pos)            // 0x80000000
#define HCSPLT_SPLITEN                   HCSPLT_SPLITEN_Msk                       // Split enable

/********************  Bit definition for HCINT register  ********************/
#define HCINT_XFER_COMPLETE_Pos          (0U)
#define HCINT_XFER_COMPLETE_Msk          (0x1UL << HCINT_XFER_COMPLETE_Pos)       // 0x00000001
#define HCINT_XFER_COMPLETE              HCINT_XFER_COMPLETE_Msk                  // Transfer completed
#define HCINT_HALTED_Pos                 (1U)
#define HCINT_HALTED_Msk                 (0x1UL << HCINT_HALTED_Pos)              // 0x00000002
#define HCINT_HALTED                     HCINT_HALTED_Msk                         // Channel halted
#define HCINT_AHB_ERR_Pos                (2U)
#define HCINT_AHB_ERR_Msk                (0x1UL << HCINT_AHB_ERR_Pos)              // 0x00000004
#define HCINT_AHB_ERR                     HCINT_AHB_ERR_Msk                         // AHB error
#define HCINT_STALL_Pos                  (3U)
#define HCINT_STALL_Msk                  (0x1UL << HCINT_STALL_Pos)               // 0x00000008
#define HCINT_STALL                      HCINT_STALL_Msk                          // STALL response received interrupt
#define HCINT_NAK_Pos                    (4U)
#define HCINT_NAK_Msk                    (0x1UL << HCINT_NAK_Pos)                 // 0x00000010
#define HCINT_NAK                        HCINT_NAK_Msk                            // NAK response received interrupt
#define HCINT_ACK_Pos                    (5U)
#define HCINT_ACK_Msk                    (0x1UL << HCINT_ACK_Pos)                 // 0x00000020
#define HCINT_ACK                        HCINT_ACK_Msk                            // ACK response received/transmitted interrupt
#define HCINT_NYET_Pos                   (6U)
#define HCINT_NYET_Msk                   (0x1UL << HCINT_NYET_Pos)                // 0x00000040
#define HCINT_NYET                       HCINT_NYET_Msk                           // Response received interrupt
#define HCINT_XACT_ERR_Pos               (7U)
#define HCINT_XACT_ERR_Msk               (0x1UL << HCINT_XACT_ERR_Pos)            // 0x00000080
#define HCINT_XACT_ERR                   HCINT_XACT_ERR_Msk                       // Transaction error
#define HCINT_BABBLE_ERR_Pos             (8U)
#define HCINT_BABBLE_ERR_Msk             (0x1UL << HCINT_BABBLE_ERR_Pos)          // 0x00000100
#define HCINT_BABBLE_ERR                 HCINT_BABBLE_ERR_Msk                     // Babble error
#define HCINT_FARME_OVERRUN_Pos          (9U)
#define HCINT_FARME_OVERRUN_Msk          (0x1UL << HCINT_FARME_OVERRUN_Pos)       // 0x00000200
#define HCINT_FARME_OVERRUN              HCINT_FARME_OVERRUN_Msk                  // Frame overrun
#define HCINT_DATATOGGLE_ERR_Pos         (10U)
#define HCINT_DATATOGGLE_ERR_Msk         (0x1UL << HCINT_DATATOGGLE_ERR_Pos)      // 0x00000400
#define HCINT_DATATOGGLE_ERR             HCINT_DATATOGGLE_ERR_Msk                 // Data toggle error
#define HCINT_BUFFER_NA_Pos             (11U)
#define HCINT_BUFFER_NA_Msk             (0x1UL << HCINT_BUFFER_NA_Pos)          // 0x00000800
#define HCINT_BUFFER_NA                 HCINT_BUFFER_NA_Msk                     // Buffer not available interrupt
#define HCINT_XCS_XACT_ERR_Pos           (12U)
#define HCINT_XCS_XACT_ERR_Msk           (0x1UL << HCINT_XCS_XACT_ERR_Pos)        // 0x00001000
#define HCINT_XCS_XACT_ERR               HCINT_XCS_XACT_ERR_Msk                   // Excessive transaction error
#define HCINT_DESC_ROLLOVER_Pos          (13U)
#define HCINT_DESC_ROLLOVER_Msk          (0x1UL << HCINT_DESC_ROLLOVER_Pos)       // 0x00002000
#define HCINT_DESC_ROLLOVER              HCINT_DESC_ROLLOVER_Msk                  // Descriptor rollover

/********************  Bit definition for DIEPINT register  ********************/
#define DIEPINT_XFRC_Pos                 (0U)
#define DIEPINT_XFRC_Msk                 (0x1UL << DIEPINT_XFRC_Pos)              // 0x00000001
#define DIEPINT_XFRC                     DIEPINT_XFRC_Msk                         // Transfer completed interrupt
#define DIEPINT_EPDISD_Pos               (1U)
#define DIEPINT_EPDISD_Msk               (0x1UL << DIEPINT_EPDISD_Pos)            // 0x00000002
#define DIEPINT_EPDISD                   DIEPINT_EPDISD_Msk                       // Endpoint disabled interrupt
#define DIEPINT_AHBERR_Pos               (2U)
#define DIEPINT_AHBERR_Msk               (0x1UL << DIEPINT_AHBERR_Pos)            // 0x00000004
#define DIEPINT_AHBERR                   DIEPINT_AHBERR_Msk                       // AHB Error (AHBErr) during an IN transaction
#define DIEPINT_TOC_Pos                  (3U)
#define DIEPINT_TOC_Msk                  (0x1UL << DIEPINT_TOC_Pos)               // 0x00000008
#define DIEPINT_TOC                      DIEPINT_TOC_Msk                          // Timeout condition
#define DIEPINT_ITTXFE_Pos               (4U)
#define DIEPINT_ITTXFE_Msk               (0x1UL << DIEPINT_ITTXFE_Pos)            // 0x00000010
#define DIEPINT_ITTXFE                   DIEPINT_ITTXFE_Msk                       // IN token received when TxFIFO is empty
#define DIEPINT_INEPNM_Pos               (5U)
#define DIEPINT_INEPNM_Msk               (0x1UL << DIEPINT_INEPNM_Pos)            // 0x00000020
#define DIEPINT_INEPNM                   DIEPINT_INEPNM_Msk                       // IN token received with EP mismatch
#define DIEPINT_INEPNE_Pos               (6U)
#define DIEPINT_INEPNE_Msk               (0x1UL << DIEPINT_INEPNE_Pos)            // 0x00000040
#define DIEPINT_INEPNE                   DIEPINT_INEPNE_Msk                       // IN endpoint NAK effective
#define DIEPINT_TXFE_Pos                 (7U)
#define DIEPINT_TXFE_Msk                 (0x1UL << DIEPINT_TXFE_Pos)              // 0x00000080
#define DIEPINT_TXFE                     DIEPINT_TXFE_Msk                         // Transmit FIFO empty
#define DIEPINT_TXFIFOUDRN_Pos           (8U)
#define DIEPINT_TXFIFOUDRN_Msk           (0x1UL << DIEPINT_TXFIFOUDRN_Pos)        // 0x00000100
#define DIEPINT_TXFIFOUDRN               DIEPINT_TXFIFOUDRN_Msk                   // Transmit Fifo Underrun
#define DIEPINT_BNA_Pos                  (9U)
#define DIEPINT_BNA_Msk                  (0x1UL << DIEPINT_BNA_Pos)               // 0x00000200
#define DIEPINT_BNA                      DIEPINT_BNA_Msk                          // Buffer not available interrupt
#define DIEPINT_PKTDRPSTS_Pos            (11U)
#define DIEPINT_PKTDRPSTS_Msk            (0x1UL << DIEPINT_PKTDRPSTS_Pos)         // 0x00000800
#define DIEPINT_PKTDRPSTS                DIEPINT_PKTDRPSTS_Msk                    // Packet dropped status
#define DIEPINT_BERR_Pos                 (12U)
#define DIEPINT_BERR_Msk                 (0x1UL << DIEPINT_BERR_Pos)              // 0x00001000
#define DIEPINT_BERR                     DIEPINT_BERR_Msk                         // Babble error interrupt
#define DIEPINT_NAK_Pos                  (13U)
#define DIEPINT_NAK_Msk                  (0x1UL << DIEPINT_NAK_Pos)               // 0x00002000
#define DIEPINT_NAK                      DIEPINT_NAK_Msk                          // NAK interrupt

/********************  Bit definition for DIEPTSIZ register  ********************/

#define DIEPTSIZ_XFRSIZ_Pos              (0U)
#define DIEPTSIZ_XFRSIZ_Msk              (0x7FFFFUL << DIEPTSIZ_XFRSIZ_Pos)       // 0x0007FFFF
#define DIEPTSIZ_XFRSIZ                  DIEPTSIZ_XFRSIZ_Msk                      // Transfer size
#define DIEPTSIZ_PKTCNT_Pos              (19U)
#define DIEPTSIZ_PKTCNT_Msk              (0x3FFUL << DIEPTSIZ_PKTCNT_Pos)         // 0x1FF80000
#define DIEPTSIZ_PKTCNT                  DIEPTSIZ_PKTCNT_Msk                      // Packet count
#define DIEPTSIZ_MULCNT_Pos              (29U)
#define DIEPTSIZ_MULCNT_Msk              (0x3UL << DIEPTSIZ_MULCNT_Pos)           // 0x60000000
#define DIEPTSIZ_MULCNT                  DIEPTSIZ_MULCNT_Msk                      // Packet count
                                                                                  /********************  Bit definition for HCTSIZ register  ********************/
#define HCTSIZ_XFRSIZ_Pos                (0U)
#define HCTSIZ_XFRSIZ_Msk                (0x7FFFFUL << HCTSIZ_XFRSIZ_Pos)         // 0x0007FFFF
#define HCTSIZ_XFRSIZ                    HCTSIZ_XFRSIZ_Msk                        // Transfer size
#define HCTSIZ_PKTCNT_Pos                (19U)
#define HCTSIZ_PKTCNT_Msk                (0x3FFUL << HCTSIZ_PKTCNT_Pos)           // 0x1FF80000
#define HCTSIZ_PKTCNT                    HCTSIZ_PKTCNT_Msk                        // Packet count
#define HCTSIZ_DOPING_Pos                (31U)
#define HCTSIZ_DOPING_Msk                (0x1UL << HCTSIZ_DOPING_Pos)             // 0x80000000
#define HCTSIZ_DOPING                    HCTSIZ_DOPING_Msk                        // Do PING
#define HCTSIZ_PID_Pos                  (29U)
#define HCTSIZ_PID_Msk                  (0x3UL << HCTSIZ_PID_Pos)               // 0x60000000
#define HCTSIZ_PID                      HCTSIZ_PID_Msk                          // Data PID

/********************  Bit definition for DIEPDMA register  ********************/
#define DIEPDMA_DMAADDR_Pos              (0U)
#define DIEPDMA_DMAADDR_Msk              (0xFFFFFFFFUL << DIEPDMA_DMAADDR_Pos)    // 0xFFFFFFFF
#define DIEPDMA_DMAADDR                  DIEPDMA_DMAADDR_Msk                      // DMA address

/********************  Bit definition for HCDMA register  ********************/
#define HCDMA_DMAADDR_Pos                (0U)
#define HCDMA_DMAADDR_Msk                (0xFFFFFFFFUL << HCDMA_DMAADDR_Pos)      // 0xFFFFFFFF
#define HCDMA_DMAADDR                    HCDMA_DMAADDR_Msk                        // DMA address

                                                                                  /********************  Bit definition for DTXFSTS register  ********************/
#define DTXFSTS_INEPTFSAV_Pos            (0U)
#define DTXFSTS_INEPTFSAV_Msk            (0xFFFFUL << DTXFSTS_INEPTFSAV_Pos)      // 0x0000FFFF
#define DTXFSTS_INEPTFSAV                DTXFSTS_INEPTFSAV_Msk                    // IN endpoint TxFIFO space available

                                                                                  /********************  Bit definition for DIEPTXF register  ********************/
#define DIEPTXF_INEPTXSA_Pos             (0U)
#define DIEPTXF_INEPTXSA_Msk             (0xFFFFUL << DIEPTXF_INEPTXSA_Pos)       // 0x0000FFFF
#define DIEPTXF_INEPTXSA                 DIEPTXF_INEPTXSA_Msk                     // IN endpoint FIFOx transmit RAM start address
#define DIEPTXF_INEPTXFD_Pos             (16U)
#define DIEPTXF_INEPTXFD_Msk             (0xFFFFUL << DIEPTXF_INEPTXFD_Pos)       // 0xFFFF0000
#define DIEPTXF_INEPTXFD                 DIEPTXF_INEPTXFD_Msk                     // IN endpoint TxFIFO depth


/********************  Bit definition for Common EPCTL register  ********************/
#define EPCTL_MPSIZ_Pos                (0U)
#define EPCTL_MPSIZ_Msk                (0x7FFUL << EPCTL_MPSIZ_Pos)           // 0x000007FF
#define EPCTL_MPSIZ                    EPCTL_MPSIZ_Msk                        // Maximum packet size          //Bit 1
#define EPCTL_USBAEP_Pos               (15U)
#define EPCTL_USBAEP_Msk               (0x1UL << EPCTL_USBAEP_Pos)            // 0x00008000
#define EPCTL_USBAEP                   EPCTL_USBAEP_Msk                       // USB active endpoint
#define EPCTL_NAKSTS_Pos               (17U)
#define EPCTL_NAKSTS_Msk               (0x1UL << EPCTL_NAKSTS_Pos)            // 0x00020000
#define EPCTL_NAKSTS                   EPCTL_NAKSTS_Msk                       // NAK status
#define EPCTL_EPTYP_Pos                (18U)
#define EPCTL_EPTYP_Msk                (0x3UL << EPCTL_EPTYP_Pos)             // 0x000C0000
#define EPCTL_EPTYP                    EPCTL_EPTYP_Msk                        // Endpoint type
#define EPCTL_EPTYP_0                  (0x1UL << EPCTL_EPTYP_Pos)             // 0x00040000
#define EPCTL_EPTYP_1                  (0x2UL << EPCTL_EPTYP_Pos)             // 0x00080000
#define EPCTL_SNPM                     EPCTL_SNPM_Msk                         // Snoop mode
#define EPCTL_STALL_Pos                (21U)
#define EPCTL_STALL_Msk                (0x1UL << EPCTL_STALL_Pos)             // 0x00200000
#define EPCTL_STALL                    EPCTL_STALL_Msk                        // STALL handshake
#define EPCTL_CNAK_Pos                 (26U)
#define EPCTL_CNAK_Msk                 (0x1UL << EPCTL_CNAK_Pos)              // 0x04000000
#define EPCTL_CNAK                     EPCTL_CNAK_Msk                         // Clear NAK
#define EPCTL_SNAK_Pos                 (27U)
#define EPCTL_SNAK_Msk                 (0x1UL << EPCTL_SNAK_Pos)              // 0x08000000
#define EPCTL_SNAK                     EPCTL_SNAK_Msk                         // Set NAK
#define EPCTL_SD0PID_SEVNFRM_Pos       (28U)
#define EPCTL_SD0PID_SEVNFRM_Msk       (0x1UL << EPCTL_SD0PID_SEVNFRM_Pos)    // 0x10000000
#define EPCTL_SD0PID_SEVNFRM           EPCTL_SD0PID_SEVNFRM_Msk               // Set DATA0 PID
#define EPCTL_SODDFRM_Pos              (29U)
#define EPCTL_SODDFRM_Msk              (0x1UL << EPCTL_SODDFRM_Pos)           // 0x20000000
#define EPCTL_SODDFRM                  EPCTL_SODDFRM_Msk                      // Set odd frame
#define EPCTL_EPDIS_Pos                (30U)
#define EPCTL_EPDIS_Msk                (0x1UL << EPCTL_EPDIS_Pos)             // 0x40000000
#define EPCTL_EPDIS                    EPCTL_EPDIS_Msk                        // Endpoint disable
#define EPCTL_EPENA_Pos                (31U)
#define EPCTL_EPENA_Msk                (0x1UL << EPCTL_EPENA_Pos)             // 0x80000000
#define EPCTL_EPENA                    EPCTL_EPENA_Msk                        // Endpoint enable

/********************  Bit definition for DOEPCTL register  ********************/
#define DOEPCTL_MPSIZ_Pos                (0U)
#define DOEPCTL_MPSIZ_Msk                (0x7FFUL << DOEPCTL_MPSIZ_Pos)           // 0x000007FF
#define DOEPCTL_MPSIZ                    DOEPCTL_MPSIZ_Msk                        // Maximum packet size          //Bit 1
#define DOEPCTL_USBAEP_Pos               (15U)
#define DOEPCTL_USBAEP_Msk               (0x1UL << DOEPCTL_USBAEP_Pos)            // 0x00008000
#define DOEPCTL_USBAEP                   DOEPCTL_USBAEP_Msk                       // USB active endpoint
#define DOEPCTL_NAKSTS_Pos               (17U)
#define DOEPCTL_NAKSTS_Msk               (0x1UL << DOEPCTL_NAKSTS_Pos)            // 0x00020000
#define DOEPCTL_NAKSTS                   DOEPCTL_NAKSTS_Msk                       // NAK status
#define DOEPCTL_SD0PID_SEVNFRM_Pos       (28U)
#define DOEPCTL_SD0PID_SEVNFRM_Msk       (0x1UL << DOEPCTL_SD0PID_SEVNFRM_Pos)    // 0x10000000
#define DOEPCTL_SD0PID_SEVNFRM           DOEPCTL_SD0PID_SEVNFRM_Msk               // Set DATA0 PID
#define DOEPCTL_SODDFRM_Pos              (29U)
#define DOEPCTL_SODDFRM_Msk              (0x1UL << DOEPCTL_SODDFRM_Pos)           // 0x20000000
#define DOEPCTL_SODDFRM                  DOEPCTL_SODDFRM_Msk                      // Set odd frame
#define DOEPCTL_EPTYP_Pos                (18U)
#define DOEPCTL_EPTYP_Msk                (0x3UL << DOEPCTL_EPTYP_Pos)             // 0x000C0000
#define DOEPCTL_EPTYP                    DOEPCTL_EPTYP_Msk                        // Endpoint type
#define DOEPCTL_EPTYP_0                  (0x1UL << DOEPCTL_EPTYP_Pos)             // 0x00040000
#define DOEPCTL_EPTYP_1                  (0x2UL << DOEPCTL_EPTYP_Pos)             // 0x00080000
#define DOEPCTL_SNPM_Pos                 (20U)
#define DOEPCTL_SNPM_Msk                 (0x1UL << DOEPCTL_SNPM_Pos)              // 0x00100000
#define DOEPCTL_SNPM                     DOEPCTL_SNPM_Msk                         // Snoop mode
#define DOEPCTL_STALL_Pos                (21U)
#define DOEPCTL_STALL_Msk                (0x1UL << DOEPCTL_STALL_Pos)             // 0x00200000
#define DOEPCTL_STALL                    DOEPCTL_STALL_Msk                        // STALL handshake
#define DOEPCTL_CNAK_Pos                 (26U)
#define DOEPCTL_CNAK_Msk                 (0x1UL << DOEPCTL_CNAK_Pos)              // 0x04000000
#define DOEPCTL_CNAK                     DOEPCTL_CNAK_Msk                         // Clear NAK
#define DOEPCTL_SNAK_Pos                 (27U)
#define DOEPCTL_SNAK_Msk                 (0x1UL << DOEPCTL_SNAK_Pos)              // 0x08000000
#define DOEPCTL_SNAK                     DOEPCTL_SNAK_Msk                         // Set NAK
#define DOEPCTL_EPDIS_Pos                (30U)
#define DOEPCTL_EPDIS_Msk                (0x1UL << DOEPCTL_EPDIS_Pos)             // 0x40000000
#define DOEPCTL_EPDIS                    DOEPCTL_EPDIS_Msk                        // Endpoint disable
#define DOEPCTL_EPENA_Pos                (31U)
#define DOEPCTL_EPENA_Msk                (0x1UL << DOEPCTL_EPENA_Pos)             // 0x80000000
#define DOEPCTL_EPENA                    DOEPCTL_EPENA_Msk                        // Endpoint enable

/********************  Bit definition for DOEPINT register  ********************/
#define DOEPINT_XFRC_Pos                 (0U)
#define DOEPINT_XFRC_Msk                 (0x1UL << DOEPINT_XFRC_Pos)              // 0x00000001
#define DOEPINT_XFRC                     DOEPINT_XFRC_Msk                         // Transfer completed interrupt
#define DOEPINT_EPDISD_Pos               (1U)
#define DOEPINT_EPDISD_Msk               (0x1UL << DOEPINT_EPDISD_Pos)            // 0x00000002
#define DOEPINT_EPDISD                   DOEPINT_EPDISD_Msk                       // Endpoint disabled interrupt
#define DOEPINT_AHBERR_Pos               (2U)
#define DOEPINT_AHBERR_Msk               (0x1UL << DOEPINT_AHBERR_Pos)            // 0x00000004
#define DOEPINT_AHBERR                   DOEPINT_AHBERR_Msk                       // AHB Error (AHBErr) during an OUT transaction

#define DOEPINT_SETUP_Pos                (3U)
#define DOEPINT_SETUP_Msk                (0x1UL << DOEPINT_SETUP_Pos)             // 0x00000008
#define DOEPINT_SETUP                    DOEPINT_SETUP_Msk                        // SETUP phase done

#define DOEPINT_OTEPDIS_Pos              (4U)
#define DOEPINT_OTEPDIS_Msk              (0x1UL << DOEPINT_OTEPDIS_Pos)           // 0x00000010
#define DOEPINT_OTEPDIS                  DOEPINT_OTEPDIS_Msk                      // OUT token received when endpoint disabled

#define DOEPINT_STSPHSRX_Pos             (5U)
#define DOEPINT_STSPHSRX_Msk             (0x1UL << DOEPINT_STSPHSRX_Pos)          // 0x00000020
#define DOEPINT_STSPHSRX                  DOEPINT_STSPHSRX_Msk                    // Status Phase Received For Control Write

#define DOEPINT_B2BSTUP_Pos              (6U)
#define DOEPINT_B2BSTUP_Msk              (0x1UL << DOEPINT_B2BSTUP_Pos)           // 0x00000040
#define DOEPINT_B2BSTUP                  DOEPINT_B2BSTUP_Msk                      // Back-to-back SETUP packets received
#define DOEPINT_OUTPKTERR_Pos            (8U)
#define DOEPINT_OUTPKTERR_Msk            (0x1UL << DOEPINT_OUTPKTERR_Pos)         // 0x00000100
#define DOEPINT_OUTPKTERR                DOEPINT_OUTPKTERR_Msk                    // OUT packet error
#define DOEPINT_NAK_Pos                  (13U)
#define DOEPINT_NAK_Msk                  (0x1UL << DOEPINT_NAK_Pos)               // 0x00002000
#define DOEPINT_NAK                      DOEPINT_NAK_Msk                          // NAK Packet is transmitted by the device
#define DOEPINT_NYET_Pos                 (14U)
#define DOEPINT_NYET_Msk                 (0x1UL << DOEPINT_NYET_Pos)              // 0x00004000
#define DOEPINT_NYET                     DOEPINT_NYET_Msk                         // NYET interrupt

#define DOEPINT_STPKTRX_Pos              (15U)
#define DOEPINT_STPKTRX_Msk              (0x1UL << DOEPINT_STPKTRX_Pos)           // 0x00008000
#define DOEPINT_STPKTRX                  DOEPINT_STPKTRX_Msk                      // Setup Packet Received

/********************  Bit definition for DOEPTSIZ register  ********************/
#define DOEPTSIZ_XFRSIZ_Pos              (0U)
#define DOEPTSIZ_XFRSIZ_Msk              (0x7FFFFUL << DOEPTSIZ_XFRSIZ_Pos)       // 0x0007FFFF
#define DOEPTSIZ_XFRSIZ                  DOEPTSIZ_XFRSIZ_Msk                      // Transfer size
#define DOEPTSIZ_PKTCNT_Pos              (19U)
#define DOEPTSIZ_PKTCNT_Msk              (0x3FFUL << DOEPTSIZ_PKTCNT_Pos)         // 0x1FF80000
#define DOEPTSIZ_PKTCNT                  DOEPTSIZ_PKTCNT_Msk                      // Packet count

#define DOEPTSIZ_STUPCNT_Pos             (29U)
#define DOEPTSIZ_STUPCNT_Msk             (0x3UL << DOEPTSIZ_STUPCNT_Pos)          // 0x60000000
#define DOEPTSIZ_STUPCNT                 DOEPTSIZ_STUPCNT_Msk                     // SETUP packet count
#define DOEPTSIZ_STUPCNT_0               (0x1UL << DOEPTSIZ_STUPCNT_Pos)          // 0x20000000
#define DOEPTSIZ_STUPCNT_1               (0x2UL << DOEPTSIZ_STUPCNT_Pos)          // 0x40000000

/********************  Bit definition for PCGCTL register  ********************/
#define PCGCCTL_IF_DEV_MODE              TU_BIT(31)
#define PCGCCTL_P2HD_PRT_SPD_MASK        (0x3ul << 29)
#define PCGCCTL_P2HD_PRT_SPD_SHIFT       29
#define PCGCCTL_P2HD_DEV_ENUM_SPD_MASK   (0x3ul << 27)
#define PCGCCTL_P2HD_DEV_ENUM_SPD_SHIFT  27
#define PCGCCTL_MAC_DEV_ADDR_MASK        (0x7ful << 20)
#define PCGCCTL_MAC_DEV_ADDR_SHIFT       20
#define PCGCCTL_MAX_TERMSEL              TU_BIT(19)
#define PCGCCTL_MAX_XCVRSELECT_MASK      (0x3ul << 17)
#define PCGCCTL_MAX_XCVRSELECT_SHIFT     17
#define PCGCCTL_PORT_POWER               TU_BIT(16)
#define PCGCCTL_PRT_CLK_SEL_MASK         (0x3ul << 14)
#define PCGCCTL_PRT_CLK_SEL_SHIFT        14
#define PCGCCTL_ESS_REG_RESTORED         TU_BIT(13)
#define PCGCCTL_EXTND_HIBER_SWITCH       TU_BIT(12)
#define PCGCCTL_EXTND_HIBER_PWRCLMP      TU_BIT(11)
#define PCGCCTL_ENBL_EXTND_HIBER         TU_BIT(10)
#define PCGCCTL_RESTOREMODE              TU_BIT(9)
#define PCGCCTL_RESETAFTSUSP             TU_BIT(8)
#define PCGCCTL_DEEP_SLEEP               TU_BIT(7)
#define PCGCCTL_PHY_IN_SLEEP             TU_BIT(6)
#define PCGCCTL_ENBL_SLEEP_GATING        TU_BIT(5)
#define PCGCCTL_RSTPDWNMODULE            TU_BIT(3)
#define PCGCCTL_PWRCLMP                  TU_BIT(2)
#define PCGCCTL_GATEHCLK                 TU_BIT(1)
#define PCGCCTL_STOPPCLK                 TU_BIT(0)

#define PCGCTL1_TIMER                   (0x3ul << 1)
#define PCGCTL1_GATEEN                  TU_BIT(0)

#ifdef __cplusplus
 }
#endif

#endif
