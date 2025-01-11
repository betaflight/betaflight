/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Reinhard Panhuber, Jerzy Kasenberg
 * Copyright (c) 2023 HiFiPhile
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

/*
 * This driver supports at most one out EP, one in EP, one control EP, and one feedback EP and one alternative interface other than zero. Hence, only one input terminal and one output terminal are support, if you need more adjust the driver!
 * It supports multiple TX and RX channels.
 *
 * In case you need more alternate interfaces, you need to define additional defines for this specific alternate interface. Just define them and set them in the set_interface function.
 *
 * There are three data flow structures currently implemented, where at least one SW-FIFO is used to decouple the asynchronous processes MCU vs. host
 *
 * 1. Input data -> SW-FIFO -> MCU USB
 *
 * The most easiest version, available in case the target MCU can handle the software FIFO (SW-FIFO) and if it is implemented in the device driver (if yes then dcd_edpt_xfer_fifo() is available)
 *
 * 2. Input data -> SW-FIFO -> Linear buffer -> MCU USB
 *
 * In case the target MCU can not handle a SW-FIFO, a linear buffer is used. This uses the default function dcd_edpt_xfer(). In this case more memory is required.
 *
 * 3. (Input data 1 | Input data 2 | ... | Input data N) ->  (SW-FIFO 1 | SW-FIFO 2 | ... | SW-FIFO N) -> Linear buffer -> MCU USB
 *
 * This case is used if you have more channels which need to be combined into one stream. Every channel has its own SW-FIFO. All data is encoded into an Linear buffer.
 *
 * The same holds in the RX case.
 *
 * */

#include "tusb_option.h"

#if (CFG_TUD_ENABLED && CFG_TUD_AUDIO)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "audio_device.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

// Use ring buffer if it's available, some MCUs need extra RAM requirements
// For DWC2 enable ring buffer will disable DMA (if available)
#ifndef TUD_AUDIO_PREFER_RING_BUFFER
  #if CFG_TUSB_MCU == OPT_MCU_LPC43XX || CFG_TUSB_MCU == OPT_MCU_LPC18XX || CFG_TUSB_MCU == OPT_MCU_MIMXRT1XXX || \
      defined(TUP_USBIP_DWC2)
    #define TUD_AUDIO_PREFER_RING_BUFFER 0
  #else
    #define TUD_AUDIO_PREFER_RING_BUFFER 1
  #endif
#endif

// Linear buffer in case target MCU is not capable of handling a ring buffer FIFO e.g. no hardware buffer
// is available or driver is would need to be changed dramatically

// Only STM32 and dcd_transdimension use non-linear buffer for now
// dwc2 except esp32sx (since it may use dcd_esp32sx)
// Ring buffer is incompatible with dcache, since neither address nor size is aligned to cache line
#if (defined(TUP_USBIP_DWC2) && !TU_CHECK_MCU(OPT_MCU_ESP32S2, OPT_MCU_ESP32S3)) || \
    defined(TUP_USBIP_FSDEV) ||                                                     \
    CFG_TUSB_MCU == OPT_MCU_RX63X ||                                                \
    CFG_TUSB_MCU == OPT_MCU_RX65X ||                                                \
    CFG_TUSB_MCU == OPT_MCU_RX72N ||                                                \
    CFG_TUSB_MCU == OPT_MCU_LPC18XX ||                                              \
    CFG_TUSB_MCU == OPT_MCU_LPC43XX ||                                              \
    CFG_TUSB_MCU == OPT_MCU_MIMXRT1XXX ||                                           \
    CFG_TUSB_MCU == OPT_MCU_MSP432E4
  #if TUD_AUDIO_PREFER_RING_BUFFER && !CFG_TUD_MEM_DCACHE_ENABLE
    #define USE_LINEAR_BUFFER 0
  #else
    #define USE_LINEAR_BUFFER 1
  #endif
#else
  #define USE_LINEAR_BUFFER 1
#endif

// Declaration of buffers

// Check for maximum supported numbers
#if CFG_TUD_AUDIO > 3
  #error Maximum number of audio functions restricted to three!
#endif

// Put swap buffer in USB section only if necessary
#if USE_LINEAR_BUFFER || CFG_TUD_AUDIO_ENABLE_ENCODING
  #define IN_SW_BUF_MEM_ATTR TU_ATTR_ALIGNED(4)
#else
  #define IN_SW_BUF_MEM_ATTR CFG_TUD_MEM_SECTION CFG_TUD_MEM_ALIGN
#endif
#if USE_LINEAR_BUFFER || CFG_TUD_AUDIO_ENABLE_DECODING
  #define OUT_SW_BUF_MEM_ATTR TU_ATTR_ALIGNED(4)
#else
  #define OUT_SW_BUF_MEM_ATTR CFG_TUD_MEM_SECTION CFG_TUD_MEM_ALIGN
#endif

// EP IN software buffers and mutexes
#if CFG_TUD_AUDIO_ENABLE_EP_IN && !CFG_TUD_AUDIO_ENABLE_ENCODING
tu_static IN_SW_BUF_MEM_ATTR struct {
  #if CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ > 0
  TUD_EPBUF_DEF(buf_1, CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ);
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_IN_SW_BUF_SZ > 0
  TUD_EPBUF_DEF(buf_2, CFG_TUD_AUDIO_FUNC_2_EP_IN_SW_BUF_SZ);
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_IN_SW_BUF_SZ > 0
  TUD_EPBUF_DEF(buf_3, CFG_TUD_AUDIO_FUNC_3_EP_IN_SW_BUF_SZ);
  #endif
} ep_in_sw_buf;

  #if CFG_FIFO_MUTEX
    #if CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ > 0
    tu_static osal_mutex_def_t ep_in_ff_mutex_wr_1;
    #endif
    #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_IN_SW_BUF_SZ > 0
    tu_static osal_mutex_def_t ep_in_ff_mutex_wr_2;
    #endif
    #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_IN_SW_BUF_SZ > 0
    tu_static osal_mutex_def_t ep_in_ff_mutex_wr_3;
    #endif
  #endif
#endif// CFG_TUD_AUDIO_ENABLE_EP_IN && !CFG_TUD_AUDIO_ENABLE_ENCODING

// Linear buffer TX in case:
// - target MCU is not capable of handling a ring buffer FIFO e.g. no hardware buffer is available or driver is would need to be changed dramatically OR
// - the software encoding is used - in this case the linear buffers serve as a target memory where logical channels are encoded into
#if CFG_TUD_AUDIO_ENABLE_EP_IN && (USE_LINEAR_BUFFER || CFG_TUD_AUDIO_ENABLE_ENCODING)
tu_static CFG_TUD_MEM_SECTION struct {
  #if CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX > 0
  TUD_EPBUF_DEF(buf_1, CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX);
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_IN_SZ_MAX > 0
  TUD_EPBUF_DEF(buf_2, CFG_TUD_AUDIO_FUNC_2_EP_IN_SZ_MAX);
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_IN_SZ_MAX > 0
  TUD_EPBUF_DEF(buf_3, CFG_TUD_AUDIO_FUNC_3_EP_IN_SZ_MAX);
  #endif
} lin_buf_in;
#endif// CFG_TUD_AUDIO_ENABLE_EP_IN && (USE_LINEAR_BUFFER || CFG_TUD_AUDIO_ENABLE_DECODING)

// EP OUT software buffers and mutexes
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && !CFG_TUD_AUDIO_ENABLE_DECODING
tu_static OUT_SW_BUF_MEM_ATTR struct {
  #if CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ > 0
  TUD_EPBUF_DEF(buf_1, CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ);
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_OUT_SW_BUF_SZ > 0
  TUD_EPBUF_DEF(buf_2, CFG_TUD_AUDIO_FUNC_2_EP_OUT_SW_BUF_SZ);
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_OUT_SW_BUF_SZ > 0
  TUD_EPBUF_DEF(buf_3, CFG_TUD_AUDIO_FUNC_3_EP_OUT_SW_BUF_SZ);
  #endif
} ep_out_sw_buf;

  #if CFG_FIFO_MUTEX
    #if CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ > 0
    tu_static osal_mutex_def_t ep_out_ff_mutex_rd_1;
    #endif
    #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_OUT_SW_BUF_SZ > 0
    tu_static osal_mutex_def_t ep_out_ff_mutex_rd_2;
    #endif
    #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_OUT_SW_BUF_SZ > 0
    tu_static osal_mutex_def_t ep_out_ff_mutex_rd_3;
    #endif
  #endif
#endif// CFG_TUD_AUDIO_ENABLE_EP_OUT && !CFG_TUD_AUDIO_ENABLE_DECODING

// Linear buffer RX in case:
// - target MCU is not capable of handling a ring buffer FIFO e.g. no hardware buffer is available or driver is would need to be changed dramatically OR
// - the software encoding is used - in this case the linear buffers serve as a target memory where logical channels are encoded into
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && (USE_LINEAR_BUFFER || CFG_TUD_AUDIO_ENABLE_DECODING)
tu_static CFG_TUD_MEM_SECTION struct {
  #if CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX > 0
  TUD_EPBUF_DEF(buf_1, CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX);
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_OUT_SZ_MAX > 0
  TUD_EPBUF_DEF(buf_2, CFG_TUD_AUDIO_FUNC_2_EP_OUT_SZ_MAX);
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_OUT_SZ_MAX > 0
  TUD_EPBUF_DEF(buf_3, CFG_TUD_AUDIO_FUNC_3_EP_OUT_SZ_MAX);
  #endif
} lin_buf_out;
#endif// CFG_TUD_AUDIO_ENABLE_EP_OUT && (USE_LINEAR_BUFFER || CFG_TUD_AUDIO_ENABLE_DECODING)

// Control buffers
tu_static uint8_t ctrl_buf_1[CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ];

#if CFG_TUD_AUDIO > 1
tu_static uint8_t ctrl_buf_2[CFG_TUD_AUDIO_FUNC_2_CTRL_BUF_SZ];
#endif

#if CFG_TUD_AUDIO > 2
tu_static uint8_t ctrl_buf_3[CFG_TUD_AUDIO_FUNC_3_CTRL_BUF_SZ];
#endif

// Active alternate setting of interfaces
tu_static uint8_t alt_setting_1[CFG_TUD_AUDIO_FUNC_1_N_AS_INT];

#if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_N_AS_INT > 0
tu_static uint8_t alt_setting_2[CFG_TUD_AUDIO_FUNC_2_N_AS_INT];
#endif

#if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_N_AS_INT > 0
tu_static uint8_t alt_setting_3[CFG_TUD_AUDIO_FUNC_3_N_AS_INT];
#endif

// Software encoding/decoding support FIFOs
#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_ENCODING
  #if CFG_TUD_AUDIO_FUNC_1_TX_SUPP_SW_FIFO_SZ > 0
    tu_static TU_ATTR_ALIGNED(4) uint8_t tx_supp_ff_buf_1[CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO][CFG_TUD_AUDIO_FUNC_1_TX_SUPP_SW_FIFO_SZ];
    tu_static tu_fifo_t tx_supp_ff_1[CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO];
    #if CFG_FIFO_MUTEX
    tu_static osal_mutex_def_t tx_supp_ff_mutex_wr_1[CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO];// No need for read mutex as only USB driver reads from FIFO
    #endif
  #endif

  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_TX_SUPP_SW_FIFO_SZ > 0
    tu_static TU_ATTR_ALIGNED(4) uint8_t tx_supp_ff_buf_2[CFG_TUD_AUDIO_FUNC_2_N_TX_SUPP_SW_FIFO][CFG_TUD_AUDIO_FUNC_2_TX_SUPP_SW_FIFO_SZ];
    tu_static tu_fifo_t tx_supp_ff_2[CFG_TUD_AUDIO_FUNC_2_N_TX_SUPP_SW_FIFO];
    #if CFG_FIFO_MUTEX
    tu_static osal_mutex_def_t tx_supp_ff_mutex_wr_2[CFG_TUD_AUDIO_FUNC_2_N_TX_SUPP_SW_FIFO];// No need for read mutex as only USB driver reads from FIFO
    #endif
  #endif

  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_TX_SUPP_SW_FIFO_SZ > 0
    tu_static TU_ATTR_ALIGNED(4) uint8_t tx_supp_ff_buf_3[CFG_TUD_AUDIO_FUNC_3_N_TX_SUPP_SW_FIFO][CFG_TUD_AUDIO_FUNC_3_TX_SUPP_SW_FIFO_SZ];
    tu_static tu_fifo_t tx_supp_ff_3[CFG_TUD_AUDIO_FUNC_3_N_TX_SUPP_SW_FIFO];
    #if CFG_FIFO_MUTEX
    tu_static osal_mutex_def_t tx_supp_ff_mutex_wr_3[CFG_TUD_AUDIO_FUNC_3_N_TX_SUPP_SW_FIFO];// No need for read mutex as only USB driver reads from FIFO
    #endif
  #endif
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_DECODING
  #if CFG_TUD_AUDIO_FUNC_1_RX_SUPP_SW_FIFO_SZ > 0
    tu_static TU_ATTR_ALIGNED(4) uint8_t rx_supp_ff_buf_1[CFG_TUD_AUDIO_FUNC_1_N_RX_SUPP_SW_FIFO][CFG_TUD_AUDIO_FUNC_1_RX_SUPP_SW_FIFO_SZ];
    tu_static tu_fifo_t rx_supp_ff_1[CFG_TUD_AUDIO_FUNC_1_N_RX_SUPP_SW_FIFO];
    #if CFG_FIFO_MUTEX
    tu_static osal_mutex_def_t rx_supp_ff_mutex_rd_1[CFG_TUD_AUDIO_FUNC_1_N_RX_SUPP_SW_FIFO];// No need for write mutex as only USB driver writes into FIFO
    #endif
  #endif

  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_RX_SUPP_SW_FIFO_SZ > 0
    tu_static TU_ATTR_ALIGNED(4) uint8_t rx_supp_ff_buf_2[CFG_TUD_AUDIO_FUNC_2_N_RX_SUPP_SW_FIFO][CFG_TUD_AUDIO_FUNC_2_RX_SUPP_SW_FIFO_SZ];
    tu_static tu_fifo_t rx_supp_ff_2[CFG_TUD_AUDIO_FUNC_2_N_RX_SUPP_SW_FIFO];
    #if CFG_FIFO_MUTEX
    tu_static osal_mutex_def_t rx_supp_ff_mutex_rd_2[CFG_TUD_AUDIO_FUNC_2_N_RX_SUPP_SW_FIFO];// No need for write mutex as only USB driver writes into FIFO
    #endif
  #endif

  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_RX_SUPP_SW_FIFO_SZ > 0
    tu_static TU_ATTR_ALIGNED(4) uint8_t rx_supp_ff_buf_3[CFG_TUD_AUDIO_FUNC_3_N_RX_SUPP_SW_FIFO][CFG_TUD_AUDIO_FUNC_3_RX_SUPP_SW_FIFO_SZ];
    tu_static tu_fifo_t rx_supp_ff_3[CFG_TUD_AUDIO_FUNC_3_N_RX_SUPP_SW_FIFO];
    #if CFG_FIFO_MUTEX
    tu_static osal_mutex_def_t rx_supp_ff_mutex_rd_3[CFG_TUD_AUDIO_FUNC_3_N_RX_SUPP_SW_FIFO];// No need for write mutex as only USB driver writes into FIFO
    #endif
  #endif
#endif

// Aligned buffer for feedback EP
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
tu_static CFG_TUD_MEM_SECTION struct {
  #if CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX > 0
  TUD_EPBUF_TYPE_DEF(uint32_t, buf_1);
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_OUT_SZ_MAX > 0
  TUD_EPBUF_TYPE_DEF(uint32_t, buf_2);
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_OUT_SZ_MAX > 0
  TUD_EPBUF_TYPE_DEF(uint32_t, buf_3);
  #endif
} fb_ep_buf;
#endif

// Aligned buffer for interrupt EP
#if CFG_TUD_AUDIO_ENABLE_INTERRUPT_EP
tu_static CFG_TUD_MEM_SECTION struct {
  TUD_EPBUF_DEF(buf, CFG_TUD_AUDIO_INTERRUPT_EP_SZ);
} int_ep_buf[CFG_TUD_AUDIO];
#endif

typedef struct
{
  uint8_t rhport;
  uint8_t const *p_desc;// Pointer pointing to Standard AC Interface Descriptor(4.7.1) - Audio Control descriptor defining audio function

#if CFG_TUD_AUDIO_ENABLE_EP_IN
  uint8_t ep_in;            // TX audio data EP.
  uint16_t ep_in_sz;        // Current size of TX EP
  uint8_t ep_in_as_intf_num;// Corresponding Standard AS Interface Descriptor (4.9.1) belonging to output terminal to which this EP belongs - 0 is invalid (this fits to UAC2 specification since AS interfaces can not have interface number equal to zero)
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_OUT
  uint8_t ep_out;            // Incoming (into uC) audio data EP.
  uint16_t ep_out_sz;        // Current size of RX EP
  uint8_t ep_out_as_intf_num;// Corresponding Standard AS Interface Descriptor (4.9.1) belonging to input terminal to which this EP belongs - 0 is invalid (this fits to UAC2 specification since AS interfaces can not have interface number equal to zero)

  #if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
  uint8_t ep_fb;// Feedback EP.
  #endif

#endif

#if CFG_TUD_AUDIO_ENABLE_INTERRUPT_EP
  uint8_t ep_int;// Audio control interrupt EP.
#endif

  bool mounted;// Device opened

  uint16_t desc_length;// Length of audio function descriptor

#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
  struct {
    uint32_t value;    // Feedback value for asynchronous mode (in 16.16 format).
    uint32_t min_value;// min value according to UAC2 FMT-2.0 section 2.3.1.1.
    uint32_t max_value;// max value according to UAC2 FMT-2.0 section 2.3.1.1.

    uint8_t frame_shift;// bInterval-1 in unit of frame (FS), micro-frame (HS)
    uint8_t compute_method;
    bool format_correction;
    union {
      uint8_t power_of_2;// pre-computed power of 2 shift
      float float_const; // pre-computed float constant

      struct {
        uint32_t sample_freq;
        uint32_t mclk_freq;
      } fixed;

      struct {
        uint32_t nom_value;    // In 16.16 format
        uint32_t fifo_lvl_avg; // In 16.16 format
        uint16_t fifo_lvl_thr; // fifo level threshold
        uint16_t rate_const[2];// pre-computed feedback/fifo_depth rate
      } fifo_count;
    } compute;

  } feedback;
#endif// CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP

// Decoding parameters - parameters are set when alternate AS interface is set by host
// Coding is currently only supported for EP. Software coding corresponding to AS interfaces without EPs are not supported currently.
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_DECODING
  audio_format_type_t format_type_rx;
  uint8_t n_channels_rx;

  #if CFG_TUD_AUDIO_ENABLE_TYPE_I_DECODING
  audio_data_format_type_I_t format_type_I_rx;
  uint8_t n_bytes_per_sample_rx;
  uint8_t n_ff_used_rx;
  #endif
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL
  uint32_t sample_rate_tx;
  uint16_t packet_sz_tx[3];
  uint8_t bclock_id_tx;
  uint8_t interval_tx;
#endif

// Encoding parameters - parameters are set when alternate AS interface is set by host
#if CFG_TUD_AUDIO_ENABLE_EP_IN && (CFG_TUD_AUDIO_ENABLE_ENCODING || CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL)
  audio_format_type_t format_type_tx;
  uint8_t n_channels_tx;
  uint8_t n_bytes_per_sample_tx;

  #if CFG_TUD_AUDIO_ENABLE_TYPE_I_ENCODING
  audio_data_format_type_I_t format_type_I_tx;
  uint8_t n_ff_used_tx;
  #endif
#endif

  /*------------- From this point, data is not cleared by bus reset -------------*/

  // Buffer for control requests
  uint8_t *ctrl_buf;
  uint8_t ctrl_buf_sz;

  // Current active alternate settings
  uint8_t *alt_setting;// We need to save the current alternate setting this way, because it is possible that there are AS interfaces which do not have an EP!

// EP Transfer buffers and FIFOs
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && !CFG_TUD_AUDIO_ENABLE_DECODING
  tu_fifo_t ep_out_ff;
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN && !CFG_TUD_AUDIO_ENABLE_ENCODING
  tu_fifo_t ep_in_ff;
#endif

// Support FIFOs for software encoding and decoding
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_DECODING
  tu_fifo_t *rx_supp_ff;
  uint8_t n_rx_supp_ff;
  uint16_t rx_supp_ff_sz_max;
  #if CFG_TUD_AUDIO_ENABLE_TYPE_I_DECODING
  uint8_t n_channels_per_ff_rx;
  #endif
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_ENCODING
  tu_fifo_t *tx_supp_ff;
  uint8_t n_tx_supp_ff;
  uint16_t tx_supp_ff_sz_max;
  #if CFG_TUD_AUDIO_ENABLE_TYPE_I_ENCODING
  uint8_t n_channels_per_ff_tx;
  #endif
#endif

// Linear buffer in case target MCU is not capable of handling a ring buffer FIFO e.g. no hardware buffer is available or driver is would need to be changed dramatically OR the support FIFOs are used
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && (USE_LINEAR_BUFFER || CFG_TUD_AUDIO_ENABLE_DECODING)
  uint8_t *lin_buf_out;
  #define USE_LINEAR_BUFFER_RX 1
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN && (USE_LINEAR_BUFFER || CFG_TUD_AUDIO_ENABLE_ENCODING)
  uint8_t *lin_buf_in;
  #define USE_LINEAR_BUFFER_TX 1
#endif

#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
  uint32_t *fb_buf;
#endif
} audiod_function_t;

#ifndef USE_LINEAR_BUFFER_TX
  #define USE_LINEAR_BUFFER_TX 0
#endif

#ifndef USE_LINEAR_BUFFER_RX
  #define USE_LINEAR_BUFFER_RX 0
#endif

#define ITF_MEM_RESET_SIZE offsetof(audiod_function_t, ctrl_buf)

//--------------------------------------------------------------------+
// WEAK FUNCTION STUBS
//--------------------------------------------------------------------+

#if CFG_TUD_AUDIO_ENABLE_EP_IN
TU_ATTR_WEAK bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  (void) rhport;
  (void) func_id;
  (void) ep_in;
  (void) cur_alt_setting;
  return true;
}

TU_ATTR_WEAK bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  (void) rhport;
  (void) n_bytes_copied;
  (void) func_id;
  (void) ep_in;
  (void) cur_alt_setting;
  return true;
}
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_OUT
TU_ATTR_WEAK bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  (void) rhport;
  (void) n_bytes_received;
  (void) func_id;
  (void) ep_out;
  (void) cur_alt_setting;
  return true;
}

TU_ATTR_WEAK bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  (void) rhport;
  (void) n_bytes_received;
  (void) func_id;
  (void) ep_out;
  (void) cur_alt_setting;
  return true;
}
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
TU_ATTR_WEAK void tud_audio_fb_done_cb(uint8_t func_id) {
  (void) func_id;
}

TU_ATTR_WEAK void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t *feedback_param) {
  (void) func_id;
  (void) alt_itf;
  feedback_param->method = AUDIO_FEEDBACK_METHOD_DISABLED;
}

TU_ATTR_WEAK bool tud_audio_feedback_format_correction_cb(uint8_t func_id) {
  (void) func_id;
  return CFG_TUD_AUDIO_ENABLE_FEEDBACK_FORMAT_CORRECTION;
}

TU_ATTR_WEAK TU_ATTR_FAST_FUNC void tud_audio_feedback_interval_isr(uint8_t func_id, uint32_t frame_number, uint8_t interval_shift) {
  (void) func_id;
  (void) frame_number;
  (void) interval_shift;
}
#endif

#if CFG_TUD_AUDIO_ENABLE_INTERRUPT_EP
TU_ATTR_WEAK void tud_audio_int_done_cb(uint8_t rhport) {
  (void) rhport;
}
#endif

// Invoked when audio set interface request received
TU_ATTR_WEAK bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  (void) p_request;
  return true;
}

// Invoked when audio set interface request received which closes an EP
TU_ATTR_WEAK bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  (void) p_request;
  return true;
}

// Invoked when audio class specific set request received for an EP
TU_ATTR_WEAK bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;
  (void) p_request;
  (void) pBuff;
  TU_LOG2("  No EP set request callback available!\r\n");
  return false;// In case no callback function is present or request can not be conducted we stall it
}

// Invoked when audio class specific set request received for an interface
TU_ATTR_WEAK bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;
  (void) p_request;
  (void) pBuff;
  TU_LOG2("  No interface set request callback available!\r\n");
  return false;// In case no callback function is present or request can not be conducted we stall it
}

// Invoked when audio class specific set request received for an entity
TU_ATTR_WEAK bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;
  (void) p_request;
  (void) pBuff;
  TU_LOG2("  No entity set request callback available!\r\n");
  return false;// In case no callback function is present or request can not be conducted we stall it
}

// Invoked when audio class specific get request received for an EP
TU_ATTR_WEAK bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  (void) p_request;
  TU_LOG2("  No EP get request callback available!\r\n");
  return false;// Stall
}

// Invoked when audio class specific get request received for an interface
TU_ATTR_WEAK bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  (void) p_request;
  TU_LOG2("  No interface get request callback available!\r\n");
  return false;// Stall
}

// Invoked when audio class specific get request received for an entity
TU_ATTR_WEAK bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  (void) p_request;
  TU_LOG2("  No entity get request callback available!\r\n");
  return false;// Stall
}

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
tu_static CFG_TUD_MEM_SECTION audiod_function_t _audiod_fct[CFG_TUD_AUDIO];

#if CFG_TUD_AUDIO_ENABLE_EP_OUT
static bool audiod_rx_done_cb(uint8_t rhport, audiod_function_t *audio, uint16_t n_bytes_received);
#endif

#if CFG_TUD_AUDIO_ENABLE_DECODING && CFG_TUD_AUDIO_ENABLE_EP_OUT
static bool audiod_decode_type_I_pcm(uint8_t rhport, audiod_function_t *audio, uint16_t n_bytes_received);
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN
static bool audiod_tx_done_cb(uint8_t rhport, audiod_function_t *audio);
#endif

#if CFG_TUD_AUDIO_ENABLE_ENCODING && CFG_TUD_AUDIO_ENABLE_EP_IN
static uint16_t audiod_encode_type_I_pcm(uint8_t rhport, audiod_function_t *audio);
#endif

static bool audiod_get_interface(uint8_t rhport, tusb_control_request_t const *p_request);
static bool audiod_set_interface(uint8_t rhport, tusb_control_request_t const *p_request);

static bool audiod_get_AS_interface_index_global(uint8_t itf, uint8_t *func_id, uint8_t *idxItf, uint8_t const **pp_desc_int);
static bool audiod_get_AS_interface_index(uint8_t itf, audiod_function_t *audio, uint8_t *idxItf, uint8_t const **pp_desc_int);
static bool audiod_verify_entity_exists(uint8_t itf, uint8_t entityID, uint8_t *func_id);
static bool audiod_verify_itf_exists(uint8_t itf, uint8_t *func_id);
static bool audiod_verify_ep_exists(uint8_t ep, uint8_t *func_id);
static uint8_t audiod_get_audio_fct_idx(audiod_function_t *audio);

#if (CFG_TUD_AUDIO_ENABLE_EP_IN && (CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL || CFG_TUD_AUDIO_ENABLE_ENCODING)) || (CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_DECODING)
static void audiod_parse_for_AS_params(audiod_function_t *audio, uint8_t const *p_desc, uint8_t const *p_desc_end, uint8_t const as_itf);

static inline uint8_t tu_desc_subtype(void const *desc) {
  return ((uint8_t const *) desc)[2];
}
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL
static bool audiod_calc_tx_packet_sz(audiod_function_t *audio);
static uint16_t audiod_tx_packet_size(const uint16_t *norminal_size, uint16_t data_count, uint16_t fifo_depth, uint16_t max_size);
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
static bool audiod_set_fb_params_freq(audiod_function_t *audio, uint32_t sample_freq, uint32_t mclk_freq);
static void audiod_fb_fifo_count_update(audiod_function_t *audio, uint16_t lvl_new);
#endif

bool tud_audio_n_mounted(uint8_t func_id) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO);
  audiod_function_t *audio = &_audiod_fct[func_id];

  return audio->mounted;
}

//--------------------------------------------------------------------+
// READ API
//--------------------------------------------------------------------+

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && !CFG_TUD_AUDIO_ENABLE_DECODING

uint16_t tud_audio_n_available(uint8_t func_id) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL);
  return tu_fifo_count(&_audiod_fct[func_id].ep_out_ff);
}

uint16_t tud_audio_n_read(uint8_t func_id, void *buffer, uint16_t bufsize) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL);
  return tu_fifo_read_n(&_audiod_fct[func_id].ep_out_ff, buffer, bufsize);
}

bool tud_audio_n_clear_ep_out_ff(uint8_t func_id) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL);
  return tu_fifo_clear(&_audiod_fct[func_id].ep_out_ff);
}

tu_fifo_t *tud_audio_n_get_ep_out_ff(uint8_t func_id) {
  if (func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL) return &_audiod_fct[func_id].ep_out_ff;
  return NULL;
}

#endif

#if CFG_TUD_AUDIO_ENABLE_DECODING && CFG_TUD_AUDIO_ENABLE_EP_OUT
// Delete all content in the support RX FIFOs
bool tud_audio_n_clear_rx_support_ff(uint8_t func_id, uint8_t ff_idx) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL && ff_idx < _audiod_fct[func_id].n_rx_supp_ff);
  return tu_fifo_clear(&_audiod_fct[func_id].rx_supp_ff[ff_idx]);
}

uint16_t tud_audio_n_available_support_ff(uint8_t func_id, uint8_t ff_idx) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL && ff_idx < _audiod_fct[func_id].n_rx_supp_ff);
  return tu_fifo_count(&_audiod_fct[func_id].rx_supp_ff[ff_idx]);
}

uint16_t tud_audio_n_read_support_ff(uint8_t func_id, uint8_t ff_idx, void *buffer, uint16_t bufsize) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL && ff_idx < _audiod_fct[func_id].n_rx_supp_ff);
  return tu_fifo_read_n(&_audiod_fct[func_id].rx_supp_ff[ff_idx], buffer, bufsize);
}

tu_fifo_t *tud_audio_n_get_rx_support_ff(uint8_t func_id, uint8_t ff_idx) {
  if (func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL && ff_idx < _audiod_fct[func_id].n_rx_supp_ff) return &_audiod_fct[func_id].rx_supp_ff[ff_idx];
  return NULL;
}
#endif

// This function is called once an audio packet is received by the USB and is responsible for putting data from USB memory into EP_OUT_FIFO (or support FIFOs + decoding of received stream into audio channels).
// If you prefer your own (more efficient) implementation suiting your purpose set CFG_TUD_AUDIO_ENABLE_DECODING = 0.

#if CFG_TUD_AUDIO_ENABLE_EP_OUT

static bool audiod_rx_done_cb(uint8_t rhport, audiod_function_t *audio, uint16_t n_bytes_received) {
  uint8_t idxItf = 0;
  uint8_t const *dummy2;
  uint8_t idx_audio_fct = 0;

  idx_audio_fct = audiod_get_audio_fct_idx(audio);
  TU_VERIFY(audiod_get_AS_interface_index(audio->ep_out_as_intf_num, audio, &idxItf, &dummy2));

  // Call a weak callback here - a possibility for user to get informed an audio packet was received and data gets now loaded into EP FIFO (or decoded into support RX software FIFO)
  TU_VERIFY(tud_audio_rx_done_pre_read_cb(rhport, n_bytes_received, idx_audio_fct, audio->ep_out, audio->alt_setting[idxItf]));

  #if CFG_TUD_AUDIO_ENABLE_DECODING

  switch (audio->format_type_rx) {
    case AUDIO_FORMAT_TYPE_UNDEFINED:
      // INDIVIDUAL DECODING PROCEDURE REQUIRED HERE!
      TU_LOG2("  Desired CFG_TUD_AUDIO_FORMAT encoding not implemented!\r\n");
      TU_BREAKPOINT();
      break;

    case AUDIO_FORMAT_TYPE_I:

      switch (audio->format_type_I_rx) {
        case AUDIO_DATA_FORMAT_TYPE_I_PCM:
          TU_VERIFY(audiod_decode_type_I_pcm(rhport, audio, n_bytes_received));
          break;

        default:
          // DESIRED CFG_TUD_AUDIO_FORMAT_TYPE_I_RX NOT IMPLEMENTED!
          TU_LOG2("  Desired CFG_TUD_AUDIO_FORMAT_TYPE_I_RX encoding not implemented!\r\n");
          TU_BREAKPOINT();
          break;
      }
      break;

    default:
      // Desired CFG_TUD_AUDIO_FORMAT_TYPE_RX not implemented!
      TU_LOG2("  Desired CFG_TUD_AUDIO_FORMAT_TYPE_RX not implemented!\r\n");
      TU_BREAKPOINT();
      break;
  }

  // Prepare for next transmission
  TU_VERIFY(usbd_edpt_xfer(rhport, audio->ep_out, audio->lin_buf_out, audio->ep_out_sz), false);

  #else

    #if USE_LINEAR_BUFFER_RX
  // Data currently is in linear buffer, copy into EP OUT FIFO
  TU_VERIFY(tu_fifo_write_n(&audio->ep_out_ff, audio->lin_buf_out, n_bytes_received));

  // Schedule for next receive
  TU_VERIFY(usbd_edpt_xfer(rhport, audio->ep_out, audio->lin_buf_out, audio->ep_out_sz), false);
    #else
  // Data is already placed in EP FIFO, schedule for next receive
  TU_VERIFY(usbd_edpt_xfer_fifo(rhport, audio->ep_out, &audio->ep_out_ff, audio->ep_out_sz), false);
    #endif

    #if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
  if (audio->feedback.compute_method == AUDIO_FEEDBACK_METHOD_FIFO_COUNT) {
    audiod_fb_fifo_count_update(audio, tu_fifo_count(&audio->ep_out_ff));
  }
    #endif

  #endif

  // Call a weak callback here - a possibility for user to get informed decoding was completed
  TU_VERIFY(tud_audio_rx_done_post_read_cb(rhport, n_bytes_received, idx_audio_fct, audio->ep_out, audio->alt_setting[idxItf]));

  return true;
}

#endif//CFG_TUD_AUDIO_ENABLE_EP_OUT

// The following functions are used in case CFG_TUD_AUDIO_ENABLE_DECODING != 0
#if CFG_TUD_AUDIO_ENABLE_DECODING && CFG_TUD_AUDIO_ENABLE_EP_OUT

// Decoding according to 2.3.1.5 Audio Streams

// Helper function
static inline void *audiod_interleaved_copy_bytes_fast_decode(uint16_t const nBytesPerSample, void *dst, const void *dst_end, void *src, uint8_t const n_ff_used) {
  // Due to one FIFO contains 2 channels, data always aligned to (nBytesPerSample * 2)
  uint16_t *dst16 = dst;
  uint16_t *src16 = src;
  const uint16_t *dst_end16 = dst_end;
  uint32_t *dst32 = dst;
  uint32_t *src32 = src;
  const uint32_t *dst_end32 = dst_end;

  if (nBytesPerSample == 1) {
    while (dst16 < dst_end16) {
      *dst16++ = *src16++;
      src16 += n_ff_used - 1;
    }
    return src16;
  } else if (nBytesPerSample == 2) {
    while (dst32 < dst_end32) {
      *dst32++ = *src32++;
      src32 += n_ff_used - 1;
    }
    return src32;
  } else if (nBytesPerSample == 3) {
    while (dst16 < dst_end16) {
      *dst16++ = *src16++;
      *dst16++ = *src16++;
      *dst16++ = *src16++;
      src16 += 3 * (n_ff_used - 1);
    }
    return src16;
  } else// nBytesPerSample == 4
  {
    while (dst32 < dst_end32) {
      *dst32++ = *src32++;
      *dst32++ = *src32++;
      src32 += 2 * (n_ff_used - 1);
    }
    return src32;
  }
}

static bool audiod_decode_type_I_pcm(uint8_t rhport, audiod_function_t *audio, uint16_t n_bytes_received) {
  (void) rhport;

  // Determine amount of samples
  uint8_t const n_ff_used = audio->n_ff_used_rx;
  uint16_t const nBytesPerFFToRead = n_bytes_received / n_ff_used;
  uint8_t cnt_ff;

  // Decode
  uint8_t *src;
  uint8_t *dst_end;

  tu_fifo_buffer_info_t info;

  for (cnt_ff = 0; cnt_ff < n_ff_used; cnt_ff++) {
    tu_fifo_get_write_info(&audio->rx_supp_ff[cnt_ff], &info);

    if (info.len_lin != 0) {
      info.len_lin = tu_min16(nBytesPerFFToRead, info.len_lin);
      src = &audio->lin_buf_out[cnt_ff * audio->n_channels_per_ff_rx * audio->n_bytes_per_sample_rx];
      dst_end = info.ptr_lin + info.len_lin;
      src = audiod_interleaved_copy_bytes_fast_decode(audio->n_bytes_per_sample_rx, info.ptr_lin, dst_end, src, n_ff_used);

      // Handle wrapped part of FIFO
      info.len_wrap = tu_min16(nBytesPerFFToRead - info.len_lin, info.len_wrap);
      if (info.len_wrap != 0) {
        dst_end = info.ptr_wrap + info.len_wrap;
        audiod_interleaved_copy_bytes_fast_decode(audio->n_bytes_per_sample_rx, info.ptr_wrap, dst_end, src, n_ff_used);
      }
      tu_fifo_advance_write_pointer(&audio->rx_supp_ff[cnt_ff], info.len_lin + info.len_wrap);
    }
  }

  // Number of bytes should be a multiple of CFG_TUD_AUDIO_N_BYTES_PER_SAMPLE_RX * CFG_TUD_AUDIO_N_CHANNELS_RX but checking makes no sense - no way to correct it
  // TU_VERIFY(cnt != n_bytes);

  #if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
  if (audio->feedback.compute_method == AUDIO_FEEDBACK_METHOD_FIFO_COUNT) {
    audiod_fb_fifo_count_update(audio, tu_fifo_count(&audio->rx_supp_ff[0]));
  }
  #endif

  return true;
}
#endif//CFG_TUD_AUDIO_ENABLE_DECODING

//--------------------------------------------------------------------+
// WRITE API
//--------------------------------------------------------------------+

#if CFG_TUD_AUDIO_ENABLE_EP_IN && !CFG_TUD_AUDIO_ENABLE_ENCODING

/**
 * \brief           Write data to EP in buffer
 *
 *  Write data to buffer. If it is full, new data can be inserted once a transmit was scheduled. See audiod_tx_done_cb().
 *  If TX FIFOs are used, this function is not available in order to not let the user mess up the encoding process.
 *
 * \param[in]       func_id: Index of audio function interface
 * \param[in]       data: Pointer to data array to be copied from
 * \param[in]       len: # of array elements to copy
 * \return          Number of bytes actually written
 */
uint16_t tud_audio_n_write(uint8_t func_id, const void *data, uint16_t len) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL);
  return tu_fifo_write_n(&_audiod_fct[func_id].ep_in_ff, data, len);
}

bool tud_audio_n_clear_ep_in_ff(uint8_t func_id)// Delete all content in the EP IN FIFO
{
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL);
  return tu_fifo_clear(&_audiod_fct[func_id].ep_in_ff);
}

tu_fifo_t *tud_audio_n_get_ep_in_ff(uint8_t func_id) {
  if (func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL) return &_audiod_fct[func_id].ep_in_ff;
  return NULL;
}

#endif

#if CFG_TUD_AUDIO_ENABLE_ENCODING && CFG_TUD_AUDIO_ENABLE_EP_IN

uint16_t tud_audio_n_flush_tx_support_ff(uint8_t func_id)// Force all content in the support TX FIFOs to be written into linear buffer and schedule a transmit
{
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL);
  audiod_function_t *audio = &_audiod_fct[func_id];

  uint16_t n_bytes_copied = tu_fifo_count(&audio->tx_supp_ff[0]);

  TU_VERIFY(audiod_tx_done_cb(audio->rhport, audio));

  n_bytes_copied -= tu_fifo_count(&audio->tx_supp_ff[0]);
  n_bytes_copied = n_bytes_copied * audio->tx_supp_ff[0].item_size;

  return n_bytes_copied;
}

bool tud_audio_n_clear_tx_support_ff(uint8_t func_id, uint8_t ff_idx) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL && ff_idx < _audiod_fct[func_id].n_tx_supp_ff);
  return tu_fifo_clear(&_audiod_fct[func_id].tx_supp_ff[ff_idx]);
}

uint16_t tud_audio_n_write_support_ff(uint8_t func_id, uint8_t ff_idx, const void *data, uint16_t len) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL && ff_idx < _audiod_fct[func_id].n_tx_supp_ff);
  return tu_fifo_write_n(&_audiod_fct[func_id].tx_supp_ff[ff_idx], data, len);
}

tu_fifo_t *tud_audio_n_get_tx_support_ff(uint8_t func_id, uint8_t ff_idx) {
  if (func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL && ff_idx < _audiod_fct[func_id].n_tx_supp_ff) return &_audiod_fct[func_id].tx_supp_ff[ff_idx];
  return NULL;
}

#endif


#if CFG_TUD_AUDIO_ENABLE_INTERRUPT_EP
// If no interrupt transmit is pending bytes get written into buffer and a transmit is scheduled - once transmit completed tud_audio_int_done_cb() is called in inform user
bool tud_audio_int_n_write(uint8_t func_id, const audio_interrupt_data_t *data) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL);

  TU_VERIFY(_audiod_fct[func_id].ep_int != 0);

  // We write directly into the EP's buffer - abort if previous transfer not complete
  TU_VERIFY(usbd_edpt_claim(_audiod_fct[func_id].rhport, _audiod_fct[func_id].ep_int));

  // Check length
  if (tu_memcpy_s(int_ep_buf[func_id].buf, sizeof(int_ep_buf[func_id].buf), data, sizeof(audio_interrupt_data_t)) == 0) {
    // Schedule transmit
    TU_ASSERT(usbd_edpt_xfer(_audiod_fct[func_id].rhport, _audiod_fct[func_id].ep_int, int_ep_buf[func_id].buf, sizeof(int_ep_buf[func_id].buf)), 0);
  } else {
    // Release endpoint since we don't make any transfer
    usbd_edpt_release(_audiod_fct[func_id].rhport, _audiod_fct[func_id].ep_int);
  }

  return true;
}
#endif

// This function is called once a transmit of an audio packet was successfully completed. Here, we encode samples and place it in IN EP's buffer for next transmission.
// If you prefer your own (more efficient) implementation suiting your purpose set CFG_TUD_AUDIO_ENABLE_ENCODING = 0 and use tud_audio_n_write.

// n_bytes_copied - Informs caller how many bytes were loaded. In case n_bytes_copied = 0, a ZLP is scheduled to inform host no data is available for current frame.
#if CFG_TUD_AUDIO_ENABLE_EP_IN
static bool audiod_tx_done_cb(uint8_t rhport, audiod_function_t *audio) {
  uint8_t idxItf;
  uint8_t const *dummy2;

  uint8_t idx_audio_fct = audiod_get_audio_fct_idx(audio);
  TU_VERIFY(audiod_get_AS_interface_index(audio->ep_in_as_intf_num, audio, &idxItf, &dummy2));

  // Only send something if current alternate interface is not 0 as in this case nothing is to be sent due to UAC2 specifications
  if (audio->alt_setting[idxItf] == 0) return false;

  // Call a weak callback here - a possibility for user to get informed former TX was completed and data gets now loaded into EP in buffer (in case FIFOs are used) or
  // if no FIFOs are used the user may use this call back to load its data into the EP IN buffer by use of tud_audio_n_write_ep_in_buffer().
  TU_VERIFY(tud_audio_tx_done_pre_load_cb(rhport, idx_audio_fct, audio->ep_in, audio->alt_setting[idxItf]));

  // Send everything in ISO EP FIFO
  uint16_t n_bytes_tx;

  // If support FIFOs are used, encode and schedule transmit
  #if CFG_TUD_AUDIO_ENABLE_ENCODING && CFG_TUD_AUDIO_ENABLE_EP_IN
  switch (audio->format_type_tx) {
    case AUDIO_FORMAT_TYPE_UNDEFINED:
      // INDIVIDUAL ENCODING PROCEDURE REQUIRED HERE!
      TU_LOG2("  Desired CFG_TUD_AUDIO_FORMAT encoding not implemented!\r\n");
      TU_BREAKPOINT();
      n_bytes_tx = 0;
      break;

    case AUDIO_FORMAT_TYPE_I:

      switch (audio->format_type_I_tx) {
        case AUDIO_DATA_FORMAT_TYPE_I_PCM:

          n_bytes_tx = audiod_encode_type_I_pcm(rhport, audio);
          break;

        default:
          // YOUR ENCODING IS REQUIRED HERE!
          TU_LOG2("  Desired CFG_TUD_AUDIO_FORMAT_TYPE_I_TX encoding not implemented!\r\n");
          TU_BREAKPOINT();
          n_bytes_tx = 0;
          break;
      }
      break;

    default:
      // Desired CFG_TUD_AUDIO_FORMAT_TYPE_TX not implemented!
      TU_LOG2("  Desired CFG_TUD_AUDIO_FORMAT_TYPE_TX not implemented!\r\n");
      TU_BREAKPOINT();
      n_bytes_tx = 0;
      break;
  }

  TU_VERIFY(usbd_edpt_xfer(rhport, audio->ep_in, audio->lin_buf_in, n_bytes_tx));

  #else
    // No support FIFOs, if no linear buffer required schedule transmit, else put data into linear buffer and schedule
    #if CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL
  // packet_sz_tx is based on total packet size, here we want size for each support buffer.
  n_bytes_tx = audiod_tx_packet_size(audio->packet_sz_tx, tu_fifo_count(&audio->ep_in_ff), audio->ep_in_ff.depth, audio->ep_in_sz);
    #else
  n_bytes_tx = tu_min16(tu_fifo_count(&audio->ep_in_ff), audio->ep_in_sz);// Limit up to max packet size, more can not be done for ISO
    #endif
    #if USE_LINEAR_BUFFER_TX
  tu_fifo_read_n(&audio->ep_in_ff, audio->lin_buf_in, n_bytes_tx);
  TU_VERIFY(usbd_edpt_xfer(rhport, audio->ep_in, audio->lin_buf_in, n_bytes_tx));
    #else
  // Send everything in ISO EP FIFO
  TU_VERIFY(usbd_edpt_xfer_fifo(rhport, audio->ep_in, &audio->ep_in_ff, n_bytes_tx));
    #endif

  #endif

  // Call a weak callback here - a possibility for user to get informed former TX was completed and how many bytes were loaded for the next frame
  TU_VERIFY(tud_audio_tx_done_post_load_cb(rhport, n_bytes_tx, idx_audio_fct, audio->ep_in, audio->alt_setting[idxItf]));

  return true;
}

#endif//CFG_TUD_AUDIO_ENABLE_EP_IN

#if CFG_TUD_AUDIO_ENABLE_ENCODING && CFG_TUD_AUDIO_ENABLE_EP_IN
// Take samples from the support buffer and encode them into the IN EP software FIFO
// Returns number of bytes written into linear buffer

/* 2.3.1.7.1 PCM Format
The PCM (Pulse Coded Modulation) format is the most commonly used audio format to represent audio
data streams. The audio data is not compressed and uses a signed two’s-complement fixed point format. It
is left-justified (the sign bit is the Msb) and data is padded with trailing zeros to fill the remaining unused
bits of the subslot. The binary point is located to the right of the sign bit so that all values lie within the
range [-1, +1)
 */

/*
 * This function encodes channels saved within the support FIFOs into one stream by interleaving the PCM samples
 * in the support FIFOs according to 2.3.1.5 Audio Streams. It does not control justification (left or right) and
 * does not change the number of bytes per sample.
 * */

// Helper function
static inline void *audiod_interleaved_copy_bytes_fast_encode(uint16_t const nBytesPerSample, void *src, const void *src_end, void *dst, uint8_t const n_ff_used) {
  // Due to one FIFO contains 2 channels, data always aligned to (nBytesPerSample * 2)
  uint16_t *dst16 = dst;
  uint16_t *src16 = src;
  const uint16_t *src_end16 = src_end;
  uint32_t *dst32 = dst;
  uint32_t *src32 = src;
  const uint32_t *src_end32 = src_end;

  if (nBytesPerSample == 1) {
    while (src16 < src_end16) {
      *dst16++ = *src16++;
      dst16 += n_ff_used - 1;
    }
    return dst16;
  } else if (nBytesPerSample == 2) {
    while (src32 < src_end32) {
      *dst32++ = *src32++;
      dst32 += n_ff_used - 1;
    }
    return dst32;
  } else if (nBytesPerSample == 3) {
    while (src16 < src_end16) {
      *dst16++ = *src16++;
      *dst16++ = *src16++;
      *dst16++ = *src16++;
      dst16 += 3 * (n_ff_used - 1);
    }
    return dst16;
  } else// nBytesPerSample == 4
  {
    while (src32 < src_end32) {
      *dst32++ = *src32++;
      *dst32++ = *src32++;
      dst32 += 2 * (n_ff_used - 1);
    }
    return dst32;
  }
}

static uint16_t audiod_encode_type_I_pcm(uint8_t rhport, audiod_function_t *audio) {
  // This function relies on the fact that the length of the support FIFOs was configured to be a multiple of the active sample size in bytes s.t. no sample is split within a wrap
  // This is ensured within set_interface, where the FIFOs are reconfigured according to this size

  // We encode directly into IN EP's linear buffer - abort if previous transfer not complete
  TU_VERIFY(!usbd_edpt_busy(rhport, audio->ep_in));

  // Determine amount of samples
  uint8_t const n_ff_used = audio->n_ff_used_tx;
  uint16_t nBytesPerFFToSend = tu_fifo_count(&audio->tx_supp_ff[0]);
  uint8_t cnt_ff;

  for (cnt_ff = 1; cnt_ff < n_ff_used; cnt_ff++) {
    uint16_t const count = tu_fifo_count(&audio->tx_supp_ff[cnt_ff]);
    if (count < nBytesPerFFToSend) {
      nBytesPerFFToSend = count;
    }
  }

  #if CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL
  const uint16_t norm_packet_sz_tx[3] = {audio->packet_sz_tx[0] / n_ff_used,
                                         audio->packet_sz_tx[1] / n_ff_used,
                                         audio->packet_sz_tx[2] / n_ff_used};
  // packet_sz_tx is based on total packet size, here we want size for each support buffer.
  nBytesPerFFToSend = audiod_tx_packet_size(norm_packet_sz_tx, nBytesPerFFToSend, audio->tx_supp_ff[0].depth, audio->ep_in_sz / n_ff_used);
  // Check if there is enough data
  if (nBytesPerFFToSend == 0) return 0;
  #else
  // Check if there is enough data
  if (nBytesPerFFToSend == 0) return 0;
  // Limit to maximum sample number - THIS IS A POSSIBLE ERROR SOURCE IF TOO MANY SAMPLE WOULD NEED TO BE SENT BUT CAN NOT!
  nBytesPerFFToSend = tu_min16(nBytesPerFFToSend, audio->ep_in_sz / n_ff_used);
  // Round to full number of samples (flooring)
  uint16_t const nSlotSize = audio->n_channels_per_ff_tx * audio->n_bytes_per_sample_tx;
  nBytesPerFFToSend = (nBytesPerFFToSend / nSlotSize) * nSlotSize;
  #endif

  // Encode
  uint8_t *dst;
  uint8_t *src_end;

  tu_fifo_buffer_info_t info;

  for (cnt_ff = 0; cnt_ff < n_ff_used; cnt_ff++) {
    dst = &audio->lin_buf_in[cnt_ff * audio->n_channels_per_ff_tx * audio->n_bytes_per_sample_tx];

    tu_fifo_get_read_info(&audio->tx_supp_ff[cnt_ff], &info);

    if (info.len_lin != 0) {
      info.len_lin = tu_min16(nBytesPerFFToSend, info.len_lin);// Limit up to desired length
      src_end = (uint8_t *) info.ptr_lin + info.len_lin;
      dst = audiod_interleaved_copy_bytes_fast_encode(audio->n_bytes_per_sample_tx, info.ptr_lin, src_end, dst, n_ff_used);

      // Limit up to desired length
      info.len_wrap = tu_min16(nBytesPerFFToSend - info.len_lin, info.len_wrap);

      // Handle wrapped part of FIFO
      if (info.len_wrap != 0) {
        src_end = (uint8_t *) info.ptr_wrap + info.len_wrap;
        audiod_interleaved_copy_bytes_fast_encode(audio->n_bytes_per_sample_tx, info.ptr_wrap, src_end, dst, n_ff_used);
      }

      tu_fifo_advance_read_pointer(&audio->tx_supp_ff[cnt_ff], info.len_lin + info.len_wrap);
    }
  }

  return nBytesPerFFToSend * n_ff_used;
}
#endif//CFG_TUD_AUDIO_ENABLE_ENCODING

// This function is called once a transmit of a feedback packet was successfully completed. Here, we get the next feedback value to be sent

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
static inline bool audiod_fb_send(audiod_function_t *audio) {
  bool apply_correction = (TUSB_SPEED_FULL == tud_speed_get()) && audio->feedback.format_correction;
  // Format the feedback value
  if (apply_correction) {
    uint8_t *fb = (uint8_t *) audio->fb_buf;

    // For FS format is 10.14
    *(fb++) = (audio->feedback.value >> 2) & 0xFF;
    *(fb++) = (audio->feedback.value >> 10) & 0xFF;
    *(fb++) = (audio->feedback.value >> 18) & 0xFF;
    *fb = 0;
  } else {
    *audio->fb_buf = audio->feedback.value;
  }

  // About feedback format on FS
  //
  // 3 variables: Format | packetSize | sendSize | Working OS:
  //              16.16    4            4          Linux, Windows
  //              16.16    4            3          Linux
  //              16.16    3            4          Linux
  //              16.16    3            3          Linux
  //              10.14    4            4          Linux
  //              10.14    4            3          Linux
  //              10.14    3            4          Linux, OSX
  //              10.14    3            3          Linux, OSX
  //
  // We send 3 bytes since sending packet larger than wMaxPacketSize is pretty ugly
  return usbd_edpt_xfer(audio->rhport, audio->ep_fb, (uint8_t *) audio->fb_buf, apply_correction ? 3 : 4);
}
#endif

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void audiod_init(void) {
  tu_memclr(_audiod_fct, sizeof(_audiod_fct));

  for (uint8_t i = 0; i < CFG_TUD_AUDIO; i++) {
    audiod_function_t *audio = &_audiod_fct[i];

    // Initialize control buffers
    switch (i) {
      case 0:
        audio->ctrl_buf = ctrl_buf_1;
        audio->ctrl_buf_sz = CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ;
        break;
#if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_CTRL_BUF_SZ > 0
      case 1:
        audio->ctrl_buf = ctrl_buf_2;
        audio->ctrl_buf_sz = CFG_TUD_AUDIO_FUNC_2_CTRL_BUF_SZ;
        break;
#endif
#if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_CTRL_BUF_SZ > 0
      case 2:
        audio->ctrl_buf = ctrl_buf_3;
        audio->ctrl_buf_sz = CFG_TUD_AUDIO_FUNC_3_CTRL_BUF_SZ;
        break;
#endif
    }

    // Initialize active alternate interface buffers
    switch (i) {
#if CFG_TUD_AUDIO_FUNC_1_N_AS_INT > 0
      case 0:
        audio->alt_setting = alt_setting_1;
        break;
#endif
#if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_N_AS_INT > 0
      case 1:
        audio->alt_setting = alt_setting_2;
        break;
#endif
#if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_N_AS_INT > 0
      case 2:
        audio->alt_setting = alt_setting_3;
        break;
#endif
    }

      // Initialize IN EP FIFO if required
#if CFG_TUD_AUDIO_ENABLE_EP_IN && !CFG_TUD_AUDIO_ENABLE_ENCODING

    switch (i) {
  #if CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ > 0
      case 0:
        tu_fifo_config(&audio->ep_in_ff, ep_in_sw_buf.buf_1, CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ, 1, true);
    #if CFG_FIFO_MUTEX
        tu_fifo_config_mutex(&audio->ep_in_ff, osal_mutex_create(&ep_in_ff_mutex_wr_1), NULL);
    #endif
        break;
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_IN_SW_BUF_SZ > 0
      case 1:
        tu_fifo_config(&audio->ep_in_ff, ep_in_sw_buf.buf_2, CFG_TUD_AUDIO_FUNC_2_EP_IN_SW_BUF_SZ, 1, true);
    #if CFG_FIFO_MUTEX
        tu_fifo_config_mutex(&audio->ep_in_ff, osal_mutex_create(&ep_in_ff_mutex_wr_2), NULL);
    #endif
        break;
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_IN_SW_BUF_SZ > 0
      case 2:
        tu_fifo_config(&audio->ep_in_ff, ep_in_sw_buf.buf_3, CFG_TUD_AUDIO_FUNC_3_EP_IN_SW_BUF_SZ, 1, true);
    #if CFG_FIFO_MUTEX
        tu_fifo_config_mutex(&audio->ep_in_ff, osal_mutex_create(&ep_in_ff_mutex_wr_3), NULL);
    #endif
        break;
  #endif
    }
#endif// CFG_TUD_AUDIO_ENABLE_EP_IN && !CFG_TUD_AUDIO_ENABLE_ENCODING

      // Initialize linear buffers
#if USE_LINEAR_BUFFER_TX
    switch (i) {
  #if CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX > 0
      case 0:
        audio->lin_buf_in = lin_buf_in.buf_1;
        break;
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_IN_SZ_MAX > 0
      case 1:
        audio->lin_buf_in = lin_buf_in.buf_2;
        break;
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_IN_SZ_MAX > 0
      case 2:
        audio->lin_buf_in = lin_buf_in.buf_3;
        break;
  #endif
    }
#endif// USE_LINEAR_BUFFER_TX

      // Initialize OUT EP FIFO if required
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && !CFG_TUD_AUDIO_ENABLE_DECODING

    switch (i) {
  #if CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ > 0
      case 0:
        tu_fifo_config(&audio->ep_out_ff, ep_out_sw_buf.buf_1, CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ, 1, true);
    #if CFG_FIFO_MUTEX
        tu_fifo_config_mutex(&audio->ep_out_ff, NULL, osal_mutex_create(&ep_out_ff_mutex_rd_1));
    #endif
        break;
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_OUT_SW_BUF_SZ > 0
      case 1:
        tu_fifo_config(&audio->ep_out_ff, ep_out_sw_buf.buf_2, CFG_TUD_AUDIO_FUNC_2_EP_OUT_SW_BUF_SZ, 1, true);
    #if CFG_FIFO_MUTEX
        tu_fifo_config_mutex(&audio->ep_out_ff, NULL, osal_mutex_create(&ep_out_ff_mutex_rd_2));
    #endif
        break;
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_OUT_SW_BUF_SZ > 0
      case 2:
        tu_fifo_config(&audio->ep_out_ff, ep_out_sw_buf.buf_3, CFG_TUD_AUDIO_FUNC_3_EP_OUT_SW_BUF_SZ, 1, true);
    #if CFG_FIFO_MUTEX
        tu_fifo_config_mutex(&audio->ep_out_ff, NULL, osal_mutex_create(&ep_out_ff_mutex_rd_3));
    #endif
        break;
  #endif
    }
#endif// CFG_TUD_AUDIO_ENABLE_EP_OUT && !CFG_TUD_AUDIO_ENABLE_DECODING

      // Initialize linear buffers
#if USE_LINEAR_BUFFER_RX
    switch (i) {
  #if CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX > 0
      case 0:
        audio->lin_buf_out = lin_buf_out.buf_1;
        break;
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_OUT_SZ_MAX > 0
      case 1:
        audio->lin_buf_out = lin_buf_out.buf_2;
        break;
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_OUT_SZ_MAX > 0
      case 2:
        audio->lin_buf_out = lin_buf_out.buf_3;
        break;
  #endif
    }
#endif// USE_LINEAR_BUFFER_RX

#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
    switch (i) {
  #if CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX > 0
      case 0:
        audio->fb_buf = &fb_ep_buf.buf_1;
        break;
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_EP_OUT_SZ_MAX > 0
      case 1:
        audio->fb_buf = &fb_ep_buf.buf_2;
        break;
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_EP_OUT_SZ_MAX > 0
      case 2:
        audio->fb_buf = &fb_ep_buf.buf_3;
        break;
  #endif
    }
#endif// CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP

      // Initialize TX support FIFOs if required
#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_ENCODING

    switch (i) {
  #if CFG_TUD_AUDIO_FUNC_1_TX_SUPP_SW_FIFO_SZ > 0
      case 0:
        audio->tx_supp_ff = tx_supp_ff_1;
        audio->n_tx_supp_ff = CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO;
        audio->tx_supp_ff_sz_max = CFG_TUD_AUDIO_FUNC_1_TX_SUPP_SW_FIFO_SZ;
        for (uint8_t cnt = 0; cnt < CFG_TUD_AUDIO_FUNC_1_N_TX_SUPP_SW_FIFO; cnt++) {
          tu_fifo_config(&tx_supp_ff_1[cnt], tx_supp_ff_buf_1[cnt], CFG_TUD_AUDIO_FUNC_1_TX_SUPP_SW_FIFO_SZ, 1, true);
    #if CFG_FIFO_MUTEX
          tu_fifo_config_mutex(&tx_supp_ff_1[cnt], osal_mutex_create(&tx_supp_ff_mutex_wr_1[cnt]), NULL);
    #endif
        }

        break;
  #endif// CFG_TUD_AUDIO_FUNC_1_TX_SUPP_SW_FIFO_SZ > 0

  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_TX_SUPP_SW_FIFO_SZ > 0
      case 1:
        audio->tx_supp_ff = tx_supp_ff_2;
        audio->n_tx_supp_ff = CFG_TUD_AUDIO_FUNC_2_N_TX_SUPP_SW_FIFO;
        audio->tx_supp_ff_sz_max = CFG_TUD_AUDIO_FUNC_2_TX_SUPP_SW_FIFO_SZ;
        for (uint8_t cnt = 0; cnt < CFG_TUD_AUDIO_FUNC_2_N_TX_SUPP_SW_FIFO; cnt++) {
          tu_fifo_config(&tx_supp_ff_2[cnt], tx_supp_ff_buf_2[cnt], CFG_TUD_AUDIO_FUNC_2_TX_SUPP_SW_FIFO_SZ, 1, true);
    #if CFG_FIFO_MUTEX
          tu_fifo_config_mutex(&tx_supp_ff_2[cnt], osal_mutex_create(&tx_supp_ff_mutex_wr_2[cnt]), NULL);
    #endif
        }

        break;
  #endif// CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_TX_SUPP_SW_FIFO_SZ > 0

  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_TX_SUPP_SW_FIFO_SZ > 0
      case 2:
        audio->tx_supp_ff = tx_supp_ff_3;
        audio->n_tx_supp_ff = CFG_TUD_AUDIO_FUNC_3_N_TX_SUPP_SW_FIFO;
        audio->tx_supp_ff_sz_max = CFG_TUD_AUDIO_FUNC_3_TX_SUPP_SW_FIFO_SZ;
        for (uint8_t cnt = 0; cnt < CFG_TUD_AUDIO_FUNC_3_N_TX_SUPP_SW_FIFO; cnt++) {
          tu_fifo_config(&tx_supp_ff_3[cnt], tx_supp_ff_buf_3[cnt], CFG_TUD_AUDIO_FUNC_3_TX_SUPP_SW_FIFO_SZ, 1, true);
    #if CFG_FIFO_MUTEX
          tu_fifo_config_mutex(&tx_supp_ff_3[cnt], osal_mutex_create(&tx_supp_ff_mutex_wr_3[cnt]), NULL);
    #endif
        }

        break;
  #endif// CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_TX_SUPP_SW_FIFO_SZ > 0
    }
#endif// CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_ENCODING

      // Set encoding parameters for Type_I formats
#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_TYPE_I_ENCODING
    switch (i) {
  #if CFG_TUD_AUDIO_FUNC_1_TX_SUPP_SW_FIFO_SZ > 0
      case 0:
        audio->n_channels_per_ff_tx = CFG_TUD_AUDIO_FUNC_1_CHANNEL_PER_FIFO_TX;
        break;
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_TX_SUPP_SW_FIFO_SZ > 0
      case 1:
        audio->n_channels_per_ff_tx = CFG_TUD_AUDIO_FUNC_2_CHANNEL_PER_FIFO_TX;
        break;
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_TX_SUPP_SW_FIFO_SZ > 0
      case 2:
        audio->n_channels_per_ff_tx = CFG_TUD_AUDIO_FUNC_3_CHANNEL_PER_FIFO_TX;
        break;
  #endif
    }
#endif// CFG_TUD_AUDIO_ENABLE_TYPE_I_ENCODING

      // Initialize RX support FIFOs if required
#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_DECODING

    switch (i) {
  #if CFG_TUD_AUDIO_FUNC_1_RX_SUPP_SW_FIFO_SZ > 0
      case 0:
        audio->rx_supp_ff = rx_supp_ff_1;
        audio->n_rx_supp_ff = CFG_TUD_AUDIO_FUNC_1_N_RX_SUPP_SW_FIFO;
        audio->rx_supp_ff_sz_max = CFG_TUD_AUDIO_FUNC_1_RX_SUPP_SW_FIFO_SZ;
        for (uint8_t cnt = 0; cnt < CFG_TUD_AUDIO_FUNC_1_N_RX_SUPP_SW_FIFO; cnt++) {
          tu_fifo_config(&rx_supp_ff_1[cnt], rx_supp_ff_buf_1[cnt], CFG_TUD_AUDIO_FUNC_1_RX_SUPP_SW_FIFO_SZ, 1, true);
    #if CFG_FIFO_MUTEX
          tu_fifo_config_mutex(&rx_supp_ff_1[cnt], osal_mutex_create(&rx_supp_ff_mutex_rd_1[cnt]), NULL);
    #endif
        }

        break;
  #endif// CFG_TUD_AUDIO_FUNC_1_RX_SUPP_SW_FIFO_SZ > 0

  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_RX_SUPP_SW_FIFO_SZ > 0
      case 1:
        audio->rx_supp_ff = rx_supp_ff_2;
        audio->n_rx_supp_ff = CFG_TUD_AUDIO_FUNC_2_N_RX_SUPP_SW_FIFO;
        audio->rx_supp_ff_sz_max = CFG_TUD_AUDIO_FUNC_2_RX_SUPP_SW_FIFO_SZ;
        for (uint8_t cnt = 0; cnt < CFG_TUD_AUDIO_FUNC_2_N_RX_SUPP_SW_FIFO; cnt++) {
          tu_fifo_config(&rx_supp_ff_2[cnt], rx_supp_ff_buf_2[cnt], CFG_TUD_AUDIO_FUNC_2_RX_SUPP_SW_FIFO_SZ, 1, true);
    #if CFG_FIFO_MUTEX
          tu_fifo_config_mutex(&rx_supp_ff_2[cnt], osal_mutex_create(&rx_supp_ff_mutex_rd_2[cnt]), NULL);
    #endif
        }

        break;
  #endif// CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_RX_SUPP_SW_FIFO_SZ > 0

  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_RX_SUPP_SW_FIFO_SZ > 0
      case 2:
        audio->rx_supp_ff = rx_supp_ff_3;
        audio->n_rx_supp_ff = CFG_TUD_AUDIO_FUNC_3_N_RX_SUPP_SW_FIFO;
        audio->rx_supp_ff_sz_max = CFG_TUD_AUDIO_FUNC_3_RX_SUPP_SW_FIFO_SZ;
        for (uint8_t cnt = 0; cnt < CFG_TUD_AUDIO_FUNC_3_N_RX_SUPP_SW_FIFO; cnt++) {
          tu_fifo_config(&rx_supp_ff_3[cnt], rx_supp_ff_buf_3[cnt], CFG_TUD_AUDIO_FUNC_3_RX_SUPP_SW_FIFO_SZ, 1, true);
    #if CFG_FIFO_MUTEX
          tu_fifo_config_mutex(&rx_supp_ff_3[cnt], osal_mutex_create(&rx_supp_ff_mutex_rd_3[cnt]), NULL);
    #endif
        }

        break;
  #endif// CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_RX_SUPP_SW_FIFO_SZ > 0
    }
#endif// CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_ENCODING

      // Set encoding parameters for Type_I formats
#if CFG_TUD_AUDIO_ENABLE_TYPE_I_DECODING
    switch (i) {
  #if CFG_TUD_AUDIO_FUNC_1_RX_SUPP_SW_FIFO_SZ > 0
      case 0:
        audio->n_channels_per_ff_rx = CFG_TUD_AUDIO_FUNC_1_CHANNEL_PER_FIFO_RX;
        break;
  #endif
  #if CFG_TUD_AUDIO > 1 && CFG_TUD_AUDIO_FUNC_2_RX_SUPP_SW_FIFO_SZ > 0
      case 1:
        audio->n_channels_per_ff_rx = CFG_TUD_AUDIO_FUNC_2_CHANNEL_PER_FIFO_RX;
        break;
  #endif
  #if CFG_TUD_AUDIO > 2 && CFG_TUD_AUDIO_FUNC_3_RX_SUPP_SW_FIFO_SZ > 0
      case 2:
        audio->n_channels_per_ff_rx = CFG_TUD_AUDIO_FUNC_3_CHANNEL_PER_FIFO_RX;
        break;
  #endif
    }
#endif// CFG_TUD_AUDIO_ENABLE_TYPE_I_DECODING
  }
}

bool audiod_deinit(void) {
  return false;// TODO not implemented yet
}

void audiod_reset(uint8_t rhport) {
  (void) rhport;

  for (uint8_t i = 0; i < CFG_TUD_AUDIO; i++) {
    audiod_function_t *audio = &_audiod_fct[i];
    tu_memclr(audio, ITF_MEM_RESET_SIZE);

#if CFG_TUD_AUDIO_ENABLE_EP_IN && !CFG_TUD_AUDIO_ENABLE_ENCODING
    tu_fifo_clear(&audio->ep_in_ff);
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && !CFG_TUD_AUDIO_ENABLE_DECODING
    tu_fifo_clear(&audio->ep_out_ff);
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_ENCODING
    for (uint8_t cnt = 0; cnt < audio->n_tx_supp_ff; cnt++) {
      tu_fifo_clear(&audio->tx_supp_ff[cnt]);
    }
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_DECODING
    for (uint8_t cnt = 0; cnt < audio->n_rx_supp_ff; cnt++) {
      tu_fifo_clear(&audio->rx_supp_ff[cnt]);
    }
#endif
  }
}

uint16_t audiod_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
  (void) max_len;

  TU_VERIFY(TUSB_CLASS_AUDIO == itf_desc->bInterfaceClass &&
            AUDIO_SUBCLASS_CONTROL == itf_desc->bInterfaceSubClass);

  // Verify version is correct - this check can be omitted
  TU_VERIFY(itf_desc->bInterfaceProtocol == AUDIO_INT_PROTOCOL_CODE_V2);

  // Verify interrupt control EP is enabled if demanded by descriptor
  TU_ASSERT(itf_desc->bNumEndpoints <= 1);// 0 or 1 EPs are allowed
  if (itf_desc->bNumEndpoints == 1) {
    TU_ASSERT(CFG_TUD_AUDIO_ENABLE_INTERRUPT_EP);
  }

  // Alternate setting MUST be zero - this check can be omitted
  TU_VERIFY(itf_desc->bAlternateSetting == 0);

  // Find available audio driver interface
  uint8_t i;
  for (i = 0; i < CFG_TUD_AUDIO; i++) {
    if (!_audiod_fct[i].p_desc) {
      _audiod_fct[i].p_desc = (uint8_t const *) itf_desc;// Save pointer to AC descriptor which is by specification always the first one
      _audiod_fct[i].rhport = rhport;

      // Setup descriptor lengths
      switch (i) {
        case 0:
          _audiod_fct[i].desc_length = CFG_TUD_AUDIO_FUNC_1_DESC_LEN;
          break;
#if CFG_TUD_AUDIO > 1
        case 1:
          _audiod_fct[i].desc_length = CFG_TUD_AUDIO_FUNC_2_DESC_LEN;
          break;
#endif
#if CFG_TUD_AUDIO > 2
        case 2:
          _audiod_fct[i].desc_length = CFG_TUD_AUDIO_FUNC_3_DESC_LEN;
          break;
#endif
      }

#ifdef TUP_DCD_EDPT_ISO_ALLOC
      {
  #if CFG_TUD_AUDIO_ENABLE_EP_IN
        uint8_t ep_in = 0;
        uint16_t ep_in_size = 0;
  #endif

  #if CFG_TUD_AUDIO_ENABLE_EP_OUT
        uint8_t ep_out = 0;
        uint16_t ep_out_size = 0;
  #endif

  #if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
        uint8_t ep_fb = 0;
  #endif
        uint8_t const *p_desc = _audiod_fct[i].p_desc;
        uint8_t const *p_desc_end = p_desc + _audiod_fct[i].desc_length - TUD_AUDIO_DESC_IAD_LEN;
        // Condition modified from p_desc < p_desc_end to prevent gcc>=12 strict-overflow warning
        while (p_desc_end - p_desc > 0) {
          if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
            tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *) p_desc;
            if (desc_ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
  #if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
              // Explicit feedback EP
              if (desc_ep->bmAttributes.usage == 1) {
                ep_fb = desc_ep->bEndpointAddress;
              }
  #endif
              // Data EP
              if (desc_ep->bmAttributes.usage == 0) {
                if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN) {
  #if CFG_TUD_AUDIO_ENABLE_EP_IN
                  ep_in = desc_ep->bEndpointAddress;
                  ep_in_size = TU_MAX(tu_edpt_packet_size(desc_ep), ep_in_size);
  #endif
                } else {
  #if CFG_TUD_AUDIO_ENABLE_EP_OUT
                  ep_out = desc_ep->bEndpointAddress;
                  ep_out_size = TU_MAX(tu_edpt_packet_size(desc_ep), ep_out_size);
  #endif
                }
              }
            }
          }

          p_desc = tu_desc_next(p_desc);
        }

  #if CFG_TUD_AUDIO_ENABLE_EP_IN
        if (ep_in) {
          usbd_edpt_iso_alloc(rhport, ep_in, ep_in_size);
        }
  #endif

  #if CFG_TUD_AUDIO_ENABLE_EP_OUT
        if (ep_out) {
          usbd_edpt_iso_alloc(rhport, ep_out, ep_out_size);
        }
  #endif

  #if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
        if (ep_fb) {
          usbd_edpt_iso_alloc(rhport, ep_fb, 4);
        }
  #endif
      }
#endif// TUP_DCD_EDPT_ISO_ALLOC

#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL
      {
        uint8_t const *p_desc = _audiod_fct[i].p_desc;
        uint8_t const *p_desc_end = p_desc + _audiod_fct[i].desc_length - TUD_AUDIO_DESC_IAD_LEN;
        // Condition modified from p_desc < p_desc_end to prevent gcc>=12 strict-overflow warning
        while (p_desc_end - p_desc > 0) {
          if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
            tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *) p_desc;
            if (desc_ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
              if (desc_ep->bmAttributes.usage == 0) {
                if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN) {
                  _audiod_fct[i].interval_tx = desc_ep->bInterval;
                }
              }
            }
          } else if (tu_desc_type(p_desc) == TUSB_DESC_CS_INTERFACE && tu_desc_subtype(p_desc) == AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL) {
            if (tu_unaligned_read16(p_desc + 4) == AUDIO_TERM_TYPE_USB_STREAMING) {
              _audiod_fct[i].bclock_id_tx = p_desc[8];
            }
          }
          p_desc = tu_desc_next(p_desc);
        }
      }
#endif// CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL

#if CFG_TUD_AUDIO_ENABLE_INTERRUPT_EP
      {
        uint8_t const *p_desc = _audiod_fct[i].p_desc;
        uint8_t const *p_desc_end = p_desc + _audiod_fct[i].desc_length - TUD_AUDIO_DESC_IAD_LEN;
        // Condition modified from p_desc < p_desc_end to prevent gcc>=12 strict-overflow warning
        while (p_desc_end - p_desc > 0) {
          // For each endpoint
          if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
            tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *) p_desc;
            uint8_t const ep_addr = desc_ep->bEndpointAddress;
            // If endpoint is input-direction and interrupt-type
            if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN && desc_ep->bmAttributes.xfer == TUSB_XFER_INTERRUPT) {
              // Store endpoint number and open endpoint
              _audiod_fct[i].ep_int = ep_addr;
              TU_ASSERT(usbd_edpt_open(_audiod_fct[i].rhport, desc_ep));
            }
          }
          p_desc = tu_desc_next(p_desc);
        }
      }
#endif

      _audiod_fct[i].mounted = true;
      break;
    }
  }

  // Verify we found a free one
  TU_ASSERT(i < CFG_TUD_AUDIO);

  // This is all we need so far - the EPs are setup by a later set_interface request (as per UAC2 specification)
  uint16_t drv_len = _audiod_fct[i].desc_length - TUD_AUDIO_DESC_IAD_LEN;// - TUD_AUDIO_DESC_IAD_LEN since tinyUSB already handles the IAD descriptor

  return drv_len;
}

static bool audiod_get_interface(uint8_t rhport, tusb_control_request_t const *p_request) {
  uint8_t const itf = tu_u16_low(p_request->wIndex);

  // Find index of audio streaming interface
  uint8_t func_id, idxItf;
  uint8_t const *dummy;

  TU_VERIFY(audiod_get_AS_interface_index_global(itf, &func_id, &idxItf, &dummy));
  TU_VERIFY(tud_control_xfer(rhport, p_request, &_audiod_fct[func_id].alt_setting[idxItf], 1));

  TU_LOG2("  Get itf: %u - current alt: %u\r\n", itf, _audiod_fct[func_id].alt_setting[idxItf]);

  return true;
}

static bool audiod_set_interface(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;

  // Here we need to do the following:

  // 1. Find the audio driver assigned to the given interface to be set
  // Since one audio driver interface has to be able to cover an unknown number of interfaces (AC, AS + its alternate settings), the best memory efficient way to solve this is to always search through the descriptors.
  // The audio driver is mapped to an audio function by a reference pointer to the corresponding AC interface of this audio function which serves as a starting point for searching

  // 2. Close EPs which are currently open
  // To do so it is not necessary to know the current active alternate interface since we already save the current EP addresses - we simply close them

  // 3. Open new EP

  uint8_t const itf = tu_u16_low(p_request->wIndex);
  uint8_t const alt = tu_u16_low(p_request->wValue);

  TU_LOG2("  Set itf: %u - alt: %u\r\n", itf, alt);

  // Find index of audio streaming interface and index of interface
  uint8_t func_id, idxItf;
  uint8_t const *p_desc;
  TU_VERIFY(audiod_get_AS_interface_index_global(itf, &func_id, &idxItf, &p_desc));

  audiod_function_t *audio = &_audiod_fct[func_id];

// Look if there is an EP to be closed - for this driver, there are only 3 possible EPs which may be closed (only AS related EPs can be closed, AC EP (if present) is always open)
#if CFG_TUD_AUDIO_ENABLE_EP_IN
  if (audio->ep_in_as_intf_num == itf) {
    audio->ep_in_as_intf_num = 0;
  #ifndef TUP_DCD_EDPT_ISO_ALLOC
    usbd_edpt_close(rhport, audio->ep_in);
  #endif

    // Clear FIFOs, since data is no longer valid
  #if !CFG_TUD_AUDIO_ENABLE_ENCODING
    tu_fifo_clear(&audio->ep_in_ff);
  #else
    for (uint8_t cnt = 0; cnt < audio->n_tx_supp_ff; cnt++) {
      tu_fifo_clear(&audio->tx_supp_ff[cnt]);
    }
  #endif

    // Invoke callback - can be used to stop data sampling
    TU_VERIFY(tud_audio_set_itf_close_EP_cb(rhport, p_request));

    audio->ep_in = 0;// Necessary?

  #if CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL
    audio->packet_sz_tx[0] = 0;
    audio->packet_sz_tx[1] = 0;
    audio->packet_sz_tx[2] = 0;
  #endif
  }
#endif// CFG_TUD_AUDIO_ENABLE_EP_IN

#if CFG_TUD_AUDIO_ENABLE_EP_OUT
  if (audio->ep_out_as_intf_num == itf) {
    audio->ep_out_as_intf_num = 0;
  #ifndef TUP_DCD_EDPT_ISO_ALLOC
    usbd_edpt_close(rhport, audio->ep_out);
  #endif

    // Clear FIFOs, since data is no longer valid
  #if !CFG_TUD_AUDIO_ENABLE_DECODING
    tu_fifo_clear(&audio->ep_out_ff);
  #else
    for (uint8_t cnt = 0; cnt < audio->n_rx_supp_ff; cnt++) {
      tu_fifo_clear(&audio->rx_supp_ff[cnt]);
    }
  #endif

    // Invoke callback - can be used to stop data sampling
    TU_VERIFY(tud_audio_set_itf_close_EP_cb(rhport, p_request));

    audio->ep_out = 0;// Necessary?

    // Close corresponding feedback EP
  #if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
    #ifndef TUP_DCD_EDPT_ISO_ALLOC
    usbd_edpt_close(rhport, audio->ep_fb);
    #endif
    audio->ep_fb = 0;
    tu_memclr(&audio->feedback, sizeof(audio->feedback));
  #endif
  }
#endif// CFG_TUD_AUDIO_ENABLE_EP_OUT

  // Save current alternative interface setting
  audio->alt_setting[idxItf] = alt;

  // Open new EP if necessary - EPs are only to be closed or opened for AS interfaces - Look for AS interface with correct alternate interface
  // Get pointer at end
  uint8_t const *p_desc_end = audio->p_desc + audio->desc_length - TUD_AUDIO_DESC_IAD_LEN;

  // p_desc starts at required interface with alternate setting zero
  // Condition modified from p_desc < p_desc_end to prevent gcc>=12 strict-overflow warning
  while (p_desc_end - p_desc > 0) {
    // Find correct interface
    if (tu_desc_type(p_desc) == TUSB_DESC_INTERFACE && ((tusb_desc_interface_t const *) p_desc)->bInterfaceNumber == itf && ((tusb_desc_interface_t const *) p_desc)->bAlternateSetting == alt) {
#if (CFG_TUD_AUDIO_ENABLE_EP_IN && (CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL || CFG_TUD_AUDIO_ENABLE_ENCODING)) || (CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_DECODING)
      uint8_t const *p_desc_parse_for_params = p_desc;
#endif
      // From this point forward follow the EP descriptors associated to the current alternate setting interface - Open EPs if necessary
      uint8_t foundEPs = 0, nEps = ((tusb_desc_interface_t const *) p_desc)->bNumEndpoints;
      // Condition modified from p_desc < p_desc_end to prevent gcc>=12 strict-overflow warning
      while (foundEPs < nEps && (p_desc_end - p_desc > 0)) {
        if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
          tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *) p_desc;
#ifdef TUP_DCD_EDPT_ISO_ALLOC
          TU_ASSERT(usbd_edpt_iso_activate(rhport, desc_ep));
#else
          TU_ASSERT(usbd_edpt_open(rhport, desc_ep));
#endif
          uint8_t const ep_addr = desc_ep->bEndpointAddress;

          //TODO: We need to set EP non busy since this is not taken care of right now in ep_close() - THIS IS A WORKAROUND!
          usbd_edpt_clear_stall(rhport, ep_addr);

#if CFG_TUD_AUDIO_ENABLE_EP_IN
          if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN && desc_ep->bmAttributes.usage == 0x00)// Check if usage is data EP
          {
            // Save address
            audio->ep_in = ep_addr;
            audio->ep_in_as_intf_num = itf;
            audio->ep_in_sz = tu_edpt_packet_size(desc_ep);

            // If software encoding is enabled, parse for the corresponding parameters - doing this here means only AS interfaces with EPs get scanned for parameters
  #if CFG_TUD_AUDIO_ENABLE_ENCODING || CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL
            audiod_parse_for_AS_params(audio, p_desc_parse_for_params, p_desc_end, itf);

              // Reconfigure size of support FIFOs - this is necessary to avoid samples to get split in case of a wrap
    #if CFG_TUD_AUDIO_ENABLE_ENCODING && CFG_TUD_AUDIO_ENABLE_TYPE_I_ENCODING
            const uint16_t active_fifo_depth = (uint16_t) ((audio->tx_supp_ff_sz_max / (audio->n_channels_per_ff_tx * audio->n_bytes_per_sample_tx)) * (audio->n_channels_per_ff_tx * audio->n_bytes_per_sample_tx));
            for (uint8_t cnt = 0; cnt < audio->n_tx_supp_ff; cnt++) {
              tu_fifo_config(&audio->tx_supp_ff[cnt], audio->tx_supp_ff[cnt].buffer, active_fifo_depth, 1, true);
            }
            audio->n_ff_used_tx = audio->n_channels_tx / audio->n_channels_per_ff_tx;
            TU_ASSERT(audio->n_ff_used_tx <= audio->n_tx_supp_ff);
    #endif
  #endif

            // Schedule first transmit if alternate interface is not zero i.e. streaming is disabled - in case no sample data is available a ZLP is loaded
            // It is necessary to trigger this here since the refill is done with an RX FIFO empty interrupt which can only trigger if something was in there
            TU_VERIFY(audiod_tx_done_cb(rhport, &_audiod_fct[func_id]));
          }
#endif// CFG_TUD_AUDIO_ENABLE_EP_IN

#if CFG_TUD_AUDIO_ENABLE_EP_OUT

          if (tu_edpt_dir(ep_addr) == TUSB_DIR_OUT)// Checking usage not necessary
          {
            // Save address
            audio->ep_out = ep_addr;
            audio->ep_out_as_intf_num = itf;
            audio->ep_out_sz = tu_edpt_packet_size(desc_ep);

  #if CFG_TUD_AUDIO_ENABLE_DECODING
            audiod_parse_for_AS_params(audio, p_desc_parse_for_params, p_desc_end, itf);

              // Reconfigure size of support FIFOs - this is necessary to avoid samples to get split in case of a wrap
    #if CFG_TUD_AUDIO_ENABLE_TYPE_I_DECODING
            const uint16_t active_fifo_depth = (audio->rx_supp_ff_sz_max / audio->n_bytes_per_sample_rx) * audio->n_bytes_per_sample_rx;
            for (uint8_t cnt = 0; cnt < audio->n_rx_supp_ff; cnt++) {
              tu_fifo_config(&audio->rx_supp_ff[cnt], audio->rx_supp_ff[cnt].buffer, active_fifo_depth, 1, true);
            }
            audio->n_ff_used_rx = audio->n_channels_rx / audio->n_channels_per_ff_rx;
            TU_ASSERT(audio->n_ff_used_rx <= audio->n_rx_supp_ff);
    #endif
  #endif

            // Prepare for incoming data
  #if USE_LINEAR_BUFFER_RX
            TU_VERIFY(usbd_edpt_xfer(rhport, audio->ep_out, audio->lin_buf_out, audio->ep_out_sz), false);
  #else
            TU_VERIFY(usbd_edpt_xfer_fifo(rhport, audio->ep_out, &audio->ep_out_ff, audio->ep_out_sz), false);
  #endif
          }

  #if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
          if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN && desc_ep->bmAttributes.usage == 1)// Check if usage is explicit data feedback
          {
            audio->ep_fb = ep_addr;
            audio->feedback.frame_shift = desc_ep->bInterval - 1;
          }
  #endif
#endif// CFG_TUD_AUDIO_ENABLE_EP_OUT

          foundEPs += 1;
        }
        p_desc = tu_desc_next(p_desc);
      }

      TU_VERIFY(foundEPs == nEps);

      // Invoke one callback for a final set interface
      TU_VERIFY(tud_audio_set_itf_cb(rhport, p_request));

#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
      // Prepare feedback computation if endpoint is available
      if (audio->ep_fb != 0) {
        audio_feedback_params_t fb_param;

        tud_audio_feedback_params_cb(func_id, alt, &fb_param);
        audio->feedback.compute_method = fb_param.method;

        if (TUSB_SPEED_FULL == tud_speed_get())
          audio->feedback.format_correction = tud_audio_feedback_format_correction_cb(func_id);

        // Minimal/Maximum value in 16.16 format for full speed (1ms per frame) or high speed (125 us per frame)
        uint32_t const frame_div = (TUSB_SPEED_FULL == tud_speed_get()) ? 1000 : 8000;
        audio->feedback.min_value = ((fb_param.sample_freq - 1) / frame_div) << 16;
        audio->feedback.max_value = (fb_param.sample_freq / frame_div + 1) << 16;

        switch (fb_param.method) {
          case AUDIO_FEEDBACK_METHOD_FREQUENCY_FIXED:
          case AUDIO_FEEDBACK_METHOD_FREQUENCY_FLOAT:
          case AUDIO_FEEDBACK_METHOD_FREQUENCY_POWER_OF_2:
            audiod_set_fb_params_freq(audio, fb_param.sample_freq, fb_param.frequency.mclk_freq);
            break;

          case AUDIO_FEEDBACK_METHOD_FIFO_COUNT: {
            // Initialize the threshold level to half filled
            uint16_t fifo_lvl_thr;
  #if CFG_TUD_AUDIO_ENABLE_DECODING
            fifo_lvl_thr = tu_fifo_depth(&audio->rx_supp_ff[0]) / 2;
  #else
            fifo_lvl_thr = tu_fifo_depth(&audio->ep_out_ff) / 2;
  #endif
            audio->feedback.compute.fifo_count.fifo_lvl_thr = fifo_lvl_thr;
            audio->feedback.compute.fifo_count.fifo_lvl_avg = ((uint32_t) fifo_lvl_thr) << 16;
            // Avoid 64bit division
            uint32_t nominal = ((fb_param.sample_freq / 100) << 16) / (frame_div / 100);
            audio->feedback.compute.fifo_count.nom_value = nominal;
            audio->feedback.compute.fifo_count.rate_const[0] = (uint16_t) ((audio->feedback.max_value - nominal) / fifo_lvl_thr);
            audio->feedback.compute.fifo_count.rate_const[1] = (uint16_t) ((nominal - audio->feedback.min_value) / fifo_lvl_thr);
            // On HS feedback is more sensitive since packet size can vary every MSOF, could cause instability
            if (tud_speed_get() == TUSB_SPEED_HIGH) {
              audio->feedback.compute.fifo_count.rate_const[0] /= 8;
              audio->feedback.compute.fifo_count.rate_const[1] /= 8;
            }
          } break;

          // nothing to do
          default:
            break;
        }
      }
#endif// CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP

      // We are done - abort loop
      break;
    }

    // Moving forward
    p_desc = tu_desc_next(p_desc);
  }

#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
  // Disable SOF interrupt if no driver has any enabled feedback EP
  bool enable_sof = false;
  for (uint8_t i = 0; i < CFG_TUD_AUDIO; i++) {
    if (_audiod_fct[i].ep_fb != 0 &&
        (_audiod_fct[i].feedback.compute_method == AUDIO_FEEDBACK_METHOD_FREQUENCY_FIXED ||
         _audiod_fct[i].feedback.compute_method == AUDIO_FEEDBACK_METHOD_FREQUENCY_FLOAT ||
         _audiod_fct[i].feedback.compute_method == AUDIO_FEEDBACK_METHOD_FREQUENCY_POWER_OF_2)) {
      enable_sof = true;
      break;
    }
  }
  usbd_sof_enable(rhport, SOF_CONSUMER_AUDIO, enable_sof);
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL
  audiod_calc_tx_packet_sz(audio);
#endif

  tud_control_status(rhport, p_request);

  return true;
}

// Invoked when class request DATA stage is finished.
// return false to stall control EP (e.g Host send non-sense DATA)
static bool audiod_control_complete(uint8_t rhport, tusb_control_request_t const *p_request) {
  // Handle audio class specific set requests
  if (p_request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS && p_request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
    uint8_t func_id;

    switch (p_request->bmRequestType_bit.recipient) {
      case TUSB_REQ_RCPT_INTERFACE: {
        uint8_t itf = TU_U16_LOW(p_request->wIndex);
        uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

        if (entityID != 0) {
          // Check if entity is present and get corresponding driver index
          TU_VERIFY(audiod_verify_entity_exists(itf, entityID, &func_id));

#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL
          uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
          if (_audiod_fct[func_id].bclock_id_tx == entityID && ctrlSel == AUDIO_CS_CTRL_SAM_FREQ && p_request->bRequest == AUDIO_CS_REQ_CUR) {
            _audiod_fct[func_id].sample_rate_tx = tu_unaligned_read32(_audiod_fct[func_id].ctrl_buf);
          }
#endif

          // Invoke callback
          return tud_audio_set_req_entity_cb(rhport, p_request, _audiod_fct[func_id].ctrl_buf);
        } else {
          // Find index of audio driver structure and verify interface really exists
          TU_VERIFY(audiod_verify_itf_exists(itf, &func_id));

          // Invoke callback
          return tud_audio_set_req_itf_cb(rhport, p_request, _audiod_fct[func_id].ctrl_buf);
        }
      } break;

      case TUSB_REQ_RCPT_ENDPOINT: {
        uint8_t ep = TU_U16_LOW(p_request->wIndex);

        // Check if entity is present and get corresponding driver index
        TU_VERIFY(audiod_verify_ep_exists(ep, &func_id));

        // Invoke callback
        return tud_audio_set_req_ep_cb(rhport, p_request, _audiod_fct[func_id].ctrl_buf);
      } break;
      // Unknown/Unsupported recipient
      default:
        TU_BREAKPOINT();
        return false;
    }
  }
  return true;
}

// Handle class control request
// return false to stall control endpoint (e.g unsupported request)
static bool audiod_control_request(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;

  // Handle standard requests - standard set requests usually have no data stage so we also handle set requests here
  if (p_request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
    switch (p_request->bRequest) {
      case TUSB_REQ_GET_INTERFACE:
        return audiod_get_interface(rhport, p_request);

      case TUSB_REQ_SET_INTERFACE:
        return audiod_set_interface(rhport, p_request);

      case TUSB_REQ_CLEAR_FEATURE:
        return true;

      // Unknown/Unsupported request
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  // Handle class requests
  if (p_request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    uint8_t func_id;

    // Conduct checks which depend on the recipient
    switch (p_request->bmRequestType_bit.recipient) {
      case TUSB_REQ_RCPT_INTERFACE: {
        uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

        // Verify if entity is present
        if (entityID != 0) {
          // Find index of audio driver structure and verify entity really exists
          TU_VERIFY(audiod_verify_entity_exists(itf, entityID, &func_id));

          // In case we got a get request invoke callback - callback needs to answer as defined in UAC2 specification page 89 - 5. Requests
          if (p_request->bmRequestType_bit.direction == TUSB_DIR_IN) {
            return tud_audio_get_req_entity_cb(rhport, p_request);
          }
        } else {
          // Find index of audio driver structure and verify interface really exists
          TU_VERIFY(audiod_verify_itf_exists(itf, &func_id));

          // In case we got a get request invoke callback - callback needs to answer as defined in UAC2 specification page 89 - 5. Requests
          if (p_request->bmRequestType_bit.direction == TUSB_DIR_IN) {
            return tud_audio_get_req_itf_cb(rhport, p_request);
          }
        }
      } break;

      case TUSB_REQ_RCPT_ENDPOINT: {
        uint8_t ep = TU_U16_LOW(p_request->wIndex);

        // Find index of audio driver structure and verify EP really exists
        TU_VERIFY(audiod_verify_ep_exists(ep, &func_id));

        // In case we got a get request invoke callback - callback needs to answer as defined in UAC2 specification page 89 - 5. Requests
        if (p_request->bmRequestType_bit.direction == TUSB_DIR_IN) {
          return tud_audio_get_req_ep_cb(rhport, p_request);
        }
      } break;

      // Unknown/Unsupported recipient
      default:
        TU_LOG2("  Unsupported recipient: %d\r\n", p_request->bmRequestType_bit.recipient);
        TU_BREAKPOINT();
        return false;
    }

    // If we end here, the received request is a set request - we schedule a receive for the data stage and return true here. We handle the rest later in audiod_control_complete() once the data stage was finished
    TU_VERIFY(tud_control_xfer(rhport, p_request, _audiod_fct[func_id].ctrl_buf, _audiod_fct[func_id].ctrl_buf_sz));
    return true;
  }

  // There went something wrong - unsupported control request type
  TU_BREAKPOINT();
  return false;
}

bool audiod_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  if (stage == CONTROL_STAGE_SETUP) {
    return audiod_control_request(rhport, request);
  } else if (stage == CONTROL_STAGE_DATA) {
    return audiod_control_complete(rhport, request);
  }

  return true;
}

bool audiod_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  (void) result;
  (void) xferred_bytes;

  // Search for interface belonging to given end point address and proceed as required
  for (uint8_t func_id = 0; func_id < CFG_TUD_AUDIO; func_id++) {
    audiod_function_t *audio = &_audiod_fct[func_id];

#if CFG_TUD_AUDIO_ENABLE_INTERRUPT_EP

    // Data transmission of control interrupt finished
    if (audio->ep_int == ep_addr) {
      // According to USB2 specification, maximum payload of interrupt EP is 8 bytes on low speed, 64 bytes on full speed, and 1024 bytes on high speed (but only if an alternate interface other than 0 is used - see specification p. 49)
      // In case there is nothing to send we have to return a NAK - this is taken care of by PHY ???
      // In case of an erroneous transmission a retransmission is conducted - this is taken care of by PHY ???

      // I assume here, that things above are handled by PHY
      // All transmission is done - what remains to do is to inform job was completed

      tud_audio_int_done_cb(rhport);
      return true;
    }

#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN

    // Data transmission of audio packet finished
    if (audio->ep_in == ep_addr && audio->alt_setting != 0) {
      // USB 2.0, section 5.6.4, third paragraph, states "An isochronous endpoint must specify its required bus access period. However, an isochronous endpoint must be prepared to handle poll rates faster than the one specified."
      // That paragraph goes on to say "An isochronous IN endpoint must return a zero-length packet whenever data is requested at a faster interval than the specified interval and data is not available."
      // This can only be solved reliably if we load a ZLP after every IN transmission since we can not say if the host requests samples earlier than we declared! Once all samples are collected we overwrite the loaded ZLP.

      // Check if there is data to load into EPs buffer - if not load it with ZLP
      // Be aware - we as a device are not able to know if the host polls for data with a faster rate as we stated this in the descriptors. Therefore we always have to put something into the EPs buffer. However, once we did that, there is no way of aborting this or replacing what we put into the buffer before!
      // This is the only place where we can fill something into the EPs buffer!

      // Load new data
      TU_VERIFY(audiod_tx_done_cb(rhport, audio));

      // Transmission of ZLP is done by audiod_tx_done_cb()
      return true;
    }
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_OUT

    // New audio packet received
    if (audio->ep_out == ep_addr) {
      TU_VERIFY(audiod_rx_done_cb(rhport, audio, (uint16_t) xferred_bytes));
      return true;
    }


  #if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
    // Transmission of feedback EP finished
    if (audio->ep_fb == ep_addr) {
      tud_audio_fb_done_cb(func_id);

      // Schedule a transmit with the new value if EP is not busy
      if (usbd_edpt_claim(rhport, audio->ep_fb)) {
        // Schedule next transmission - value is changed bytud_audio_n_fb_set() in the meantime or the old value gets sent
        return audiod_fb_send(audio);
      }
    }
  #endif
#endif
  }

  return false;
}

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP

static bool audiod_set_fb_params_freq(audiod_function_t *audio, uint32_t sample_freq, uint32_t mclk_freq) {
  // Check if frame interval is within sane limits
  // The interval value n_frames was taken from the descriptors within audiod_set_interface()

  // n_frames_min is ceil(2^10 * f_s / f_m) for full speed and ceil(2^13 * f_s / f_m) for high speed
  // this lower limit ensures the measures feedback value has sufficient precision
  uint32_t const k = (TUSB_SPEED_FULL == tud_speed_get()) ? 10 : 13;
  uint32_t const n_frame = (1UL << audio->feedback.frame_shift);

  if ((((1UL << k) * sample_freq / mclk_freq) + 1) > n_frame) {
    TU_LOG1("  UAC2 feedback interval too small\r\n");
    TU_BREAKPOINT();
    return false;
  }

  // Check if parameters really allow for a power of two division
  if ((mclk_freq % sample_freq) == 0 && tu_is_power_of_two(mclk_freq / sample_freq)) {
    audio->feedback.compute_method = AUDIO_FEEDBACK_METHOD_FREQUENCY_POWER_OF_2;
    audio->feedback.compute.power_of_2 = (uint8_t) (16 - (audio->feedback.frame_shift - 1) - tu_log2(mclk_freq / sample_freq));
  } else if (audio->feedback.compute_method == AUDIO_FEEDBACK_METHOD_FREQUENCY_FLOAT) {
    audio->feedback.compute.float_const = (float) sample_freq / (float) mclk_freq * (1UL << (16 - (audio->feedback.frame_shift - 1)));
  } else {
    audio->feedback.compute.fixed.sample_freq = sample_freq;
    audio->feedback.compute.fixed.mclk_freq = mclk_freq;
  }

  return true;
}

static void audiod_fb_fifo_count_update(audiod_function_t *audio, uint16_t lvl_new) {
  /* Low-pass (averaging) filter */
  uint32_t lvl = audio->feedback.compute.fifo_count.fifo_lvl_avg;
  lvl = (uint32_t) (((uint64_t) lvl * 63 + ((uint32_t) lvl_new << 16)) >> 6);
  audio->feedback.compute.fifo_count.fifo_lvl_avg = lvl;

  uint32_t const ff_lvl = lvl >> 16;
  uint16_t const ff_thr = audio->feedback.compute.fifo_count.fifo_lvl_thr;
  uint16_t const *rate = audio->feedback.compute.fifo_count.rate_const;

  uint32_t feedback;

  if (ff_lvl < ff_thr) {
    feedback = audio->feedback.compute.fifo_count.nom_value + (ff_thr - ff_lvl) * rate[0];
  } else {
    feedback = audio->feedback.compute.fifo_count.nom_value - (ff_lvl - ff_thr) * rate[1];
  }

  if (feedback > audio->feedback.max_value) feedback = audio->feedback.max_value;
  if (feedback < audio->feedback.min_value) feedback = audio->feedback.min_value;
  audio->feedback.value = feedback;

  // Schedule a transmit with the new value if EP is not busy - this triggers repetitive scheduling of the feedback value
  if (usbd_edpt_claim(audio->rhport, audio->ep_fb)) {
    audiod_fb_send(audio);
  }
}

uint32_t tud_audio_feedback_update(uint8_t func_id, uint32_t cycles) {
  audiod_function_t *audio = &_audiod_fct[func_id];
  uint32_t feedback;

  switch (audio->feedback.compute_method) {
    case AUDIO_FEEDBACK_METHOD_FREQUENCY_POWER_OF_2:
      feedback = (cycles << audio->feedback.compute.power_of_2);
      break;

    case AUDIO_FEEDBACK_METHOD_FREQUENCY_FLOAT:
      feedback = (uint32_t) ((float) cycles * audio->feedback.compute.float_const);
      break;

    case AUDIO_FEEDBACK_METHOD_FREQUENCY_FIXED: {
      uint64_t fb64 = (((uint64_t) cycles) * audio->feedback.compute.fixed.sample_freq) << (16 - (audio->feedback.frame_shift - 1));
      feedback = (uint32_t) (fb64 / audio->feedback.compute.fixed.mclk_freq);
    } break;

    default:
      return 0;
  }

  // For Windows: https://docs.microsoft.com/en-us/windows-hardware/drivers/audio/usb-2-0-audio-drivers
  // The size of isochronous packets created by the device must be within the limits specified in FMT-2.0 section 2.3.1.1.
  // This means that the deviation of actual packet size from nominal size must not exceed +/- one audio slot
  // (audio slot = channel count samples).
  if (feedback > audio->feedback.max_value) feedback = audio->feedback.max_value;
  if (feedback < audio->feedback.min_value) feedback = audio->feedback.min_value;

  tud_audio_n_fb_set(func_id, feedback);

  return feedback;
}

bool tud_audio_n_fb_set(uint8_t func_id, uint32_t feedback) {
  TU_VERIFY(func_id < CFG_TUD_AUDIO && _audiod_fct[func_id].p_desc != NULL);

  _audiod_fct[func_id].feedback.value = feedback;

  // Schedule a transmit with the new value if EP is not busy - this triggers repetitive scheduling of the feedback value
  if (usbd_edpt_claim(_audiod_fct[func_id].rhport, _audiod_fct[func_id].ep_fb)) {
    return audiod_fb_send(&_audiod_fct[func_id]);
  }

  return true;
}
#endif

TU_ATTR_FAST_FUNC void audiod_sof_isr(uint8_t rhport, uint32_t frame_count) {
  (void) rhport;
  (void) frame_count;

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
  // Determine feedback value - The feedback method is described in 5.12.4.2 of the USB 2.0 spec
  // Boiled down, the feedback value Ff = n_samples / (micro)frame.
  // Since an accuracy of less than 1 Sample / second is desired, at least n_frames = ceil(2^K * f_s / f_m) frames need to be measured, where K = 10 for full speed and K = 13 for high speed, f_s is the sampling frequency e.g. 48 kHz and f_m is the cpu clock frequency e.g. 100 MHz (or any other master clock whose clock count is available and locked to f_s)
  // The update interval in the (4.10.2.1) Feedback Endpoint Descriptor must be less or equal to 2^(K - P), where P = min( ceil(log2(f_m / f_s)), K)
  // feedback = n_cycles / n_frames * f_s / f_m in 16.16 format, where n_cycles are the number of main clock cycles within fb_n_frames

  // Iterate over audio functions and set feedback value
  for (uint8_t i = 0; i < CFG_TUD_AUDIO; i++) {
    audiod_function_t *audio = &_audiod_fct[i];

    if (audio->ep_fb != 0) {
      // HS shift need to be adjusted since SOF event is generated for frame only
      uint8_t const hs_adjust = (TUSB_SPEED_HIGH == tud_speed_get()) ? 3 : 0;
      uint32_t const interval = 1UL << (audio->feedback.frame_shift - hs_adjust);
      if (0 == (frame_count & (interval - 1))) {
        tud_audio_feedback_interval_isr(i, frame_count, audio->feedback.frame_shift);
      }
    }
  }
#endif// CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
}

bool tud_audio_buffer_and_schedule_control_xfer(uint8_t rhport, tusb_control_request_t const *p_request, void *data, uint16_t len) {
  // Handles only sending of data not receiving
  if (p_request->bmRequestType_bit.direction == TUSB_DIR_OUT) return false;

  // Get corresponding driver index
  uint8_t func_id;
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  // Conduct checks which depend on the recipient
  switch (p_request->bmRequestType_bit.recipient) {
    case TUSB_REQ_RCPT_INTERFACE: {
      uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

      // Verify if entity is present
      if (entityID != 0) {
        // Find index of audio driver structure and verify entity really exists
        TU_VERIFY(audiod_verify_entity_exists(itf, entityID, &func_id));
      } else {
        // Find index of audio driver structure and verify interface really exists
        TU_VERIFY(audiod_verify_itf_exists(itf, &func_id));
      }
    } break;

    case TUSB_REQ_RCPT_ENDPOINT: {
      uint8_t ep = TU_U16_LOW(p_request->wIndex);

      // Find index of audio driver structure and verify EP really exists
      TU_VERIFY(audiod_verify_ep_exists(ep, &func_id));
    } break;

    // Unknown/Unsupported recipient
    default:
      TU_LOG2("  Unsupported recipient: %d\r\n", p_request->bmRequestType_bit.recipient);
      TU_BREAKPOINT();
      return false;
  }

  // Crop length
  if (len > _audiod_fct[func_id].ctrl_buf_sz) len = _audiod_fct[func_id].ctrl_buf_sz;

  // Copy into buffer
  TU_VERIFY(0 == tu_memcpy_s(_audiod_fct[func_id].ctrl_buf, _audiod_fct[func_id].ctrl_buf_sz, data, (size_t) len));

#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL
  // Find data for sampling_frequency_control
  if (p_request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS && p_request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE) {
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    if (_audiod_fct[func_id].bclock_id_tx == entityID && ctrlSel == AUDIO_CS_CTRL_SAM_FREQ && p_request->bRequest == AUDIO_CS_REQ_CUR) {
      _audiod_fct[func_id].sample_rate_tx = tu_unaligned_read32(_audiod_fct[func_id].ctrl_buf);
    }
  }
#endif

  // Schedule transmit
  return tud_control_xfer(rhport, p_request, (void *) _audiod_fct[func_id].ctrl_buf, len);
}

// This helper function finds for a given audio function and AS interface number the index of the attached driver structure, the index of the interface in the audio function
// (e.g. the std. AS interface with interface number 15 is the first AS interface for the given audio function and thus gets index zero), and
// finally a pointer to the std. AS interface, where the pointer always points to the first alternate setting i.e. alternate interface zero.
static bool audiod_get_AS_interface_index(uint8_t itf, audiod_function_t *audio, uint8_t *idxItf, uint8_t const **pp_desc_int) {
  if (audio->p_desc) {
    // Get pointer at end
    uint8_t const *p_desc_end = audio->p_desc + audio->desc_length - TUD_AUDIO_DESC_IAD_LEN;

    // Advance past AC descriptors
    uint8_t const *p_desc = tu_desc_next(audio->p_desc);
    p_desc += ((audio_desc_cs_ac_interface_t const *) p_desc)->wTotalLength;

    uint8_t tmp = 0;
    // Condition modified from p_desc < p_desc_end to prevent gcc>=12 strict-overflow warning
    while (p_desc_end - p_desc > 0) {
      // We assume the number of alternate settings is increasing thus we return the index of alternate setting zero!
      if (tu_desc_type(p_desc) == TUSB_DESC_INTERFACE && ((tusb_desc_interface_t const *) p_desc)->bAlternateSetting == 0) {
        if (((tusb_desc_interface_t const *) p_desc)->bInterfaceNumber == itf) {
          *idxItf = tmp;
          *pp_desc_int = p_desc;
          return true;
        }
        // Increase index, bytes read, and pointer
        tmp++;
      }
      p_desc = tu_desc_next(p_desc);
    }
  }
  return false;
}

// This helper function finds for a given AS interface number the index of the attached driver structure, the index of the interface in the audio function
// (e.g. the std. AS interface with interface number 15 is the first AS interface for the given audio function and thus gets index zero), and
// finally a pointer to the std. AS interface, where the pointer always points to the first alternate setting i.e. alternate interface zero.
static bool audiod_get_AS_interface_index_global(uint8_t itf, uint8_t *func_id, uint8_t *idxItf, uint8_t const **pp_desc_int) {
  // Loop over audio driver interfaces
  uint8_t i;
  for (i = 0; i < CFG_TUD_AUDIO; i++) {
    if (audiod_get_AS_interface_index(itf, &_audiod_fct[i], idxItf, pp_desc_int)) {
      *func_id = i;
      return true;
    }
  }

  return false;
}

// Verify an entity with the given ID exists and returns also the corresponding driver index
static bool audiod_verify_entity_exists(uint8_t itf, uint8_t entityID, uint8_t *func_id) {
  uint8_t i;
  for (i = 0; i < CFG_TUD_AUDIO; i++) {
    // Look for the correct driver by checking if the unique standard AC interface number fits
    if (_audiod_fct[i].p_desc && ((tusb_desc_interface_t const *) _audiod_fct[i].p_desc)->bInterfaceNumber == itf) {
      // Get pointers after class specific AC descriptors and end of AC descriptors - entities are defined in between
      uint8_t const *p_desc = tu_desc_next(_audiod_fct[i].p_desc);// Points to CS AC descriptor
      uint8_t const *p_desc_end = ((audio_desc_cs_ac_interface_t const *) p_desc)->wTotalLength + p_desc;
      p_desc = tu_desc_next(p_desc);// Get past CS AC descriptor

      // Condition modified from p_desc < p_desc_end to prevent gcc>=12 strict-overflow warning
      while (p_desc_end - p_desc > 0) {
        if (p_desc[3] == entityID)// Entity IDs are always at offset 3
        {
          *func_id = i;
          return true;
        }
        p_desc = tu_desc_next(p_desc);
      }
    }
  }
  return false;
}

static bool audiod_verify_itf_exists(uint8_t itf, uint8_t *func_id) {
  uint8_t i;
  for (i = 0; i < CFG_TUD_AUDIO; i++) {
    if (_audiod_fct[i].p_desc) {
      // Get pointer at beginning and end
      uint8_t const *p_desc = _audiod_fct[i].p_desc;
      uint8_t const *p_desc_end = _audiod_fct[i].p_desc + _audiod_fct[i].desc_length - TUD_AUDIO_DESC_IAD_LEN;
      // Condition modified from p_desc < p_desc_end to prevent gcc>=12 strict-overflow warning
      while (p_desc_end - p_desc > 0) {
        if (tu_desc_type(p_desc) == TUSB_DESC_INTERFACE && ((tusb_desc_interface_t const *) _audiod_fct[i].p_desc)->bInterfaceNumber == itf) {
          *func_id = i;
          return true;
        }
        p_desc = tu_desc_next(p_desc);
      }
    }
  }
  return false;
}

static bool audiod_verify_ep_exists(uint8_t ep, uint8_t *func_id) {
  uint8_t i;
  for (i = 0; i < CFG_TUD_AUDIO; i++) {
    if (_audiod_fct[i].p_desc) {
      // Get pointer at end
      uint8_t const *p_desc_end = _audiod_fct[i].p_desc + _audiod_fct[i].desc_length;

      // Advance past AC descriptors - EP we look for are streaming EPs
      uint8_t const *p_desc = tu_desc_next(_audiod_fct[i].p_desc);
      p_desc += ((audio_desc_cs_ac_interface_t const *) p_desc)->wTotalLength;

      // Condition modified from p_desc < p_desc_end to prevent gcc>=12 strict-overflow warning
      while (p_desc_end - p_desc > 0) {
        if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT && ((tusb_desc_endpoint_t const *) p_desc)->bEndpointAddress == ep) {
          *func_id = i;
          return true;
        }
        p_desc = tu_desc_next(p_desc);
      }
    }
  }
  return false;
}

#if (CFG_TUD_AUDIO_ENABLE_EP_IN && (CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL || CFG_TUD_AUDIO_ENABLE_ENCODING)) || (CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_DECODING)
// p_desc points to the AS interface of alternate setting zero
// itf is the interface number of the corresponding interface - we check if the interface belongs to EP in or EP out to see if it is a TX or RX parameter
// Currently, only AS interfaces with an EP (in or out) are supposed to be parsed for!
static void audiod_parse_for_AS_params(audiod_function_t *audio, uint8_t const *p_desc, uint8_t const *p_desc_end, uint8_t const as_itf) {
  #if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_EP_OUT
  if (as_itf != audio->ep_in_as_intf_num && as_itf != audio->ep_out_as_intf_num) return;// Abort, this interface has no EP, this driver does not support this currently
  #endif
  #if CFG_TUD_AUDIO_ENABLE_EP_IN && !CFG_TUD_AUDIO_ENABLE_EP_OUT
  if (as_itf != audio->ep_in_as_intf_num) return;
  #endif
  #if !CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_EP_OUT
  if (as_itf != audio->ep_out_as_intf_num) return;
  #endif

  p_desc = tu_desc_next(p_desc);// Exclude standard AS interface descriptor of current alternate interface descriptor
  // Condition modified from p_desc < p_desc_end to prevent gcc>=12 strict-overflow warning
  while (p_desc_end - p_desc > 0) {
    // Abort if follow up descriptor is a new standard interface descriptor - indicates the last AS descriptor was already finished
    if (tu_desc_type(p_desc) == TUSB_DESC_INTERFACE) break;

    // Look for a Class-Specific AS Interface Descriptor(4.9.2) to verify format type and format and also to get number of physical channels
    if (tu_desc_type(p_desc) == TUSB_DESC_CS_INTERFACE && tu_desc_subtype(p_desc) == AUDIO_CS_AS_INTERFACE_AS_GENERAL) {
  #if CFG_TUD_AUDIO_ENABLE_EP_IN
      if (as_itf == audio->ep_in_as_intf_num) {
        audio->n_channels_tx = ((audio_desc_cs_as_interface_t const *) p_desc)->bNrChannels;
        audio->format_type_tx = (audio_format_type_t) (((audio_desc_cs_as_interface_t const *) p_desc)->bFormatType);

    #if CFG_TUD_AUDIO_ENABLE_TYPE_I_ENCODING
        audio->format_type_I_tx = (audio_data_format_type_I_t) (((audio_desc_cs_as_interface_t const *) p_desc)->bmFormats);
    #endif
      }
  #endif

  #if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_DECODING
      if (as_itf == audio->ep_out_as_intf_num) {
        audio->n_channels_rx = ((audio_desc_cs_as_interface_t const *) p_desc)->bNrChannels;
        audio->format_type_rx = ((audio_desc_cs_as_interface_t const *) p_desc)->bFormatType;
    #if CFG_TUD_AUDIO_ENABLE_TYPE_I_DECODING
        audio->format_type_I_rx = ((audio_desc_cs_as_interface_t const *) p_desc)->bmFormats;
    #endif
      }
  #endif
    }

    // Look for a Type I Format Type Descriptor(2.3.1.6 - Audio Formats)
  #if CFG_TUD_AUDIO_ENABLE_TYPE_I_ENCODING || CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL || CFG_TUD_AUDIO_ENABLE_TYPE_I_DECODING
    if (tu_desc_type(p_desc) == TUSB_DESC_CS_INTERFACE && tu_desc_subtype(p_desc) == AUDIO_CS_AS_INTERFACE_FORMAT_TYPE && ((audio_desc_type_I_format_t const *) p_desc)->bFormatType == AUDIO_FORMAT_TYPE_I) {
    #if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_EP_OUT
      if (as_itf != audio->ep_in_as_intf_num && as_itf != audio->ep_out_as_intf_num) break;// Abort loop, this interface has no EP, this driver does not support this currently
    #endif
    #if CFG_TUD_AUDIO_ENABLE_EP_IN && !CFG_TUD_AUDIO_ENABLE_EP_OUT
      if (as_itf != audio->ep_in_as_intf_num) break;
    #endif
    #if !CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_ENABLE_EP_OUT
      if (as_itf != audio->ep_out_as_intf_num) break;
    #endif

    #if CFG_TUD_AUDIO_ENABLE_EP_IN
      if (as_itf == audio->ep_in_as_intf_num) {
        audio->n_bytes_per_sample_tx = ((audio_desc_type_I_format_t const *) p_desc)->bSubslotSize;
      }
    #endif

    #if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_DECODING
      if (as_itf == audio->ep_out_as_intf_num) {
        audio->n_bytes_per_sample_rx = ((audio_desc_type_I_format_t const *) p_desc)->bSubslotSize;
      }
    #endif
    }
  #endif

    // Other format types are not supported yet

    p_desc = tu_desc_next(p_desc);
  }
}
#endif

#if CFG_TUD_AUDIO_ENABLE_EP_IN && CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL

static bool audiod_calc_tx_packet_sz(audiod_function_t *audio) {
  TU_VERIFY(audio->format_type_tx == AUDIO_FORMAT_TYPE_I);
  TU_VERIFY(audio->n_channels_tx);
  TU_VERIFY(audio->n_bytes_per_sample_tx);
  TU_VERIFY(audio->interval_tx);
  TU_VERIFY(audio->sample_rate_tx);

  const uint8_t interval = (tud_speed_get() == TUSB_SPEED_FULL) ? audio->interval_tx : 1 << (audio->interval_tx - 1);

  const uint16_t sample_normimal = (uint16_t) (audio->sample_rate_tx * interval / ((tud_speed_get() == TUSB_SPEED_FULL) ? 1000 : 8000));
  const uint16_t sample_reminder = (uint16_t) (audio->sample_rate_tx * interval % ((tud_speed_get() == TUSB_SPEED_FULL) ? 1000 : 8000));

  const uint16_t packet_sz_tx_min = (uint16_t) ((sample_normimal - 1) * audio->n_channels_tx * audio->n_bytes_per_sample_tx);
  const uint16_t packet_sz_tx_norm = (uint16_t) (sample_normimal * audio->n_channels_tx * audio->n_bytes_per_sample_tx);
  const uint16_t packet_sz_tx_max = (uint16_t) ((sample_normimal + 1) * audio->n_channels_tx * audio->n_bytes_per_sample_tx);

  // Endpoint size must larger than packet size
  TU_ASSERT(packet_sz_tx_max <= audio->ep_in_sz);

  // Frmt20.pdf 2.3.1.1 USB Packets
  if (sample_reminder) {
    // All virtual frame packets must either contain INT(nav) audio slots (small VFP) or INT(nav)+1 (large VFP) audio slots
    audio->packet_sz_tx[0] = packet_sz_tx_norm;
    audio->packet_sz_tx[1] = packet_sz_tx_norm;
    audio->packet_sz_tx[2] = packet_sz_tx_max;
  } else {
    // In the case where nav = INT(nav), ni may vary between INT(nav)-1 (small VFP), INT(nav)
    // (medium VFP) and INT(nav)+1 (large VFP).
    audio->packet_sz_tx[0] = packet_sz_tx_min;
    audio->packet_sz_tx[1] = packet_sz_tx_norm;
    audio->packet_sz_tx[2] = packet_sz_tx_max;
  }

  return true;
}

static uint16_t audiod_tx_packet_size(const uint16_t *norminal_size, uint16_t data_count, uint16_t fifo_depth, uint16_t max_depth) {
  // Flow control need a FIFO size of at least 4*Navg
  if (norminal_size[1] && norminal_size[1] <= fifo_depth * 4) {
    // Use blackout to prioritize normal size packet
    static int ctrl_blackout = 0;
    uint16_t packet_size;
    uint16_t slot_size = norminal_size[2] - norminal_size[1];
    if (data_count < norminal_size[0]) {
      // If you get here frequently, then your I2S clock deviation is too big !
      packet_size = 0;
    } else if (data_count < fifo_depth / 2 - slot_size && !ctrl_blackout) {
      packet_size = norminal_size[0];
      ctrl_blackout = 10;
    } else if (data_count > fifo_depth / 2 + slot_size && !ctrl_blackout) {
      packet_size = norminal_size[2];
      if (norminal_size[0] == norminal_size[1]) {
        // nav > INT(nav), eg. 44.1k, 88.2k
        ctrl_blackout = 0;
      } else {
        // nav = INT(nav), eg. 48k, 96k
        ctrl_blackout = 10;
      }
    } else {
      packet_size = norminal_size[1];
      if (ctrl_blackout) {
        ctrl_blackout--;
      }
    }
    // Normally this cap is not necessary
    return tu_min16(packet_size, max_depth);
  } else {
    return tu_min16(data_count, max_depth);
  }
}

#endif

// No security checks here - internal function only which should always succeed
static uint8_t audiod_get_audio_fct_idx(audiod_function_t *audio) {
  for (uint8_t cnt = 0; cnt < CFG_TUD_AUDIO; cnt++) {
    if (&_audiod_fct[cnt] == audio) return cnt;
  }
  return 0;
}

#endif // (CFG_TUD_ENABLED && CFG_TUD_AUDIO)
