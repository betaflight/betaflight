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

#if CFG_TUH_ENABLED && CFG_TUH_MSC

#include "host/usbh.h"
#include "host/usbh_pvt.h"

#include "msc_host.h"

// Level where CFG_TUSB_DEBUG must be at least for this driver is logged
#ifndef CFG_TUH_MSC_LOG_LEVEL
  #define CFG_TUH_MSC_LOG_LEVEL   CFG_TUH_LOG_LEVEL
#endif

#define TU_LOG_DRV(...)   TU_LOG(CFG_TUH_MSC_LOG_LEVEL, __VA_ARGS__)

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
enum {
  MSC_STAGE_IDLE = 0,
  MSC_STAGE_CMD,
  MSC_STAGE_DATA,
  MSC_STAGE_STATUS,
};

typedef struct {
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t ep_out;
  uint8_t max_lun;

  volatile bool configured; // Receive SET_CONFIGURE
  volatile bool mounted;    // Enumeration is complete

  // SCSI command data
  uint8_t stage;
  void* buffer;
  tuh_msc_complete_cb_t complete_cb;
  uintptr_t complete_arg;

  struct {
    uint32_t block_size;
    uint32_t block_count;
  } capacity[CFG_TUH_MSC_MAXLUN];
} msch_interface_t;

typedef struct {
  TUH_EPBUF_TYPE_DEF(msc_cbw_t, cbw);
  TUH_EPBUF_TYPE_DEF(msc_csw_t, csw);
} msch_epbuf_t;

static msch_interface_t _msch_itf[CFG_TUH_DEVICE_MAX];
CFG_TUH_MEM_SECTION static msch_epbuf_t _msch_epbuf[CFG_TUH_DEVICE_MAX];

TU_ATTR_ALWAYS_INLINE static inline msch_interface_t* get_itf(uint8_t daddr) {
  return &_msch_itf[daddr - 1];
}

TU_ATTR_ALWAYS_INLINE static inline msch_epbuf_t* get_epbuf(uint8_t daddr) {
  return &_msch_epbuf[daddr - 1];
}

//--------------------------------------------------------------------+
// PUBLIC API
//--------------------------------------------------------------------+
uint8_t tuh_msc_get_maxlun(uint8_t dev_addr) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->max_lun;
}

uint32_t tuh_msc_get_block_count(uint8_t dev_addr, uint8_t lun) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->capacity[lun].block_count;
}

uint32_t tuh_msc_get_block_size(uint8_t dev_addr, uint8_t lun) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->capacity[lun].block_size;
}

bool tuh_msc_mounted(uint8_t dev_addr) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->mounted;
}

bool tuh_msc_ready(uint8_t dev_addr) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->mounted && !usbh_edpt_busy(dev_addr, p_msc->ep_in) && !usbh_edpt_busy(dev_addr, p_msc->ep_out);
}

//--------------------------------------------------------------------+
// PUBLIC API: SCSI COMMAND
//--------------------------------------------------------------------+
static inline void cbw_init(msc_cbw_t* cbw, uint8_t lun) {
  tu_memclr(cbw, sizeof(msc_cbw_t));
  cbw->signature = MSC_CBW_SIGNATURE;
  cbw->tag       = 0x54555342; // TUSB
  cbw->lun       = lun;
}

bool tuh_msc_scsi_command(uint8_t daddr, msc_cbw_t const* cbw, void* data,
                          tuh_msc_complete_cb_t complete_cb, uintptr_t arg) {
  msch_interface_t* p_msc = get_itf(daddr);
  TU_VERIFY(p_msc->configured);

  // claim endpoint
  TU_VERIFY(usbh_edpt_claim(daddr, p_msc->ep_out));
  msch_epbuf_t* epbuf = get_epbuf(daddr);

  epbuf->cbw = *cbw;
  p_msc->buffer = data;
  p_msc->complete_cb = complete_cb;
  p_msc->complete_arg = arg;
  p_msc->stage = MSC_STAGE_CMD;

  if (!usbh_edpt_xfer(daddr, p_msc->ep_out, (uint8_t*) &epbuf->cbw, sizeof(msc_cbw_t))) {
    usbh_edpt_release(daddr, p_msc->ep_out);
    return false;
  }

  return true;
}

bool tuh_msc_read_capacity(uint8_t dev_addr, uint8_t lun, scsi_read_capacity10_resp_t* response,
                           tuh_msc_complete_cb_t complete_cb, uintptr_t arg) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->configured);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = sizeof(scsi_read_capacity10_resp_t);
  cbw.dir        = TUSB_DIR_IN_MASK;
  cbw.cmd_len    = sizeof(scsi_read_capacity10_t);
  cbw.command[0] = SCSI_CMD_READ_CAPACITY_10;

  return tuh_msc_scsi_command(dev_addr, &cbw, response, complete_cb, arg);
}

bool tuh_msc_inquiry(uint8_t dev_addr, uint8_t lun, scsi_inquiry_resp_t* response,
                     tuh_msc_complete_cb_t complete_cb, uintptr_t arg) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = sizeof(scsi_inquiry_resp_t);
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_inquiry_t);

  scsi_inquiry_t const cmd_inquiry = {
      .cmd_code     = SCSI_CMD_INQUIRY,
      .alloc_length = sizeof(scsi_inquiry_resp_t)
  };
  memcpy(cbw.command, &cmd_inquiry, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, response, complete_cb, arg);
}

bool tuh_msc_test_unit_ready(uint8_t dev_addr, uint8_t lun, tuh_msc_complete_cb_t complete_cb, uintptr_t arg) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->configured);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 0;
  cbw.dir        = TUSB_DIR_OUT;
  cbw.cmd_len    = sizeof(scsi_test_unit_ready_t);
  cbw.command[0] = SCSI_CMD_TEST_UNIT_READY;
  cbw.command[1] = lun; // according to wiki TODO need verification

  return tuh_msc_scsi_command(dev_addr, &cbw, NULL, complete_cb, arg);
}

bool tuh_msc_request_sense(uint8_t dev_addr, uint8_t lun, void* response,
                           tuh_msc_complete_cb_t complete_cb, uintptr_t arg) {
  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 18; // TODO sense response
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_request_sense_t);

  scsi_request_sense_t const cmd_request_sense = {
      .cmd_code     = SCSI_CMD_REQUEST_SENSE,
      .alloc_length = 18
  };
  memcpy(cbw.command, &cmd_request_sense, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, response, complete_cb, arg);
}

bool tuh_msc_read10(uint8_t dev_addr, uint8_t lun, void* buffer, uint32_t lba, uint16_t block_count,
                    tuh_msc_complete_cb_t complete_cb, uintptr_t arg) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = block_count * p_msc->capacity[lun].block_size;
  cbw.dir = TUSB_DIR_IN_MASK;
  cbw.cmd_len = sizeof(scsi_read10_t);

  scsi_read10_t const cmd_read10 = {
      .cmd_code    = SCSI_CMD_READ_10,
      .lba         = tu_htonl(lba),
      .block_count = tu_htons(block_count)
  };
  memcpy(cbw.command, &cmd_read10, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb, arg);
}

bool tuh_msc_write10(uint8_t dev_addr, uint8_t lun, void const* buffer, uint32_t lba, uint16_t block_count,
                     tuh_msc_complete_cb_t complete_cb, uintptr_t arg) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = block_count * p_msc->capacity[lun].block_size;
  cbw.dir         = TUSB_DIR_OUT;
  cbw.cmd_len     = sizeof(scsi_write10_t);

  scsi_write10_t const cmd_write10 = {
      .cmd_code    = SCSI_CMD_WRITE_10,
      .lba         = tu_htonl(lba),
      .block_count = tu_htons(block_count)
  };
  memcpy(cbw.command, &cmd_write10, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, (void*) (uintptr_t) buffer, complete_cb, arg);
}

#if 0
// MSC interface Reset (not used now)
bool tuh_msc_reset(uint8_t dev_addr) {
  tusb_control_request_t const new_request = {
    .bmRequestType_bit = {
      .recipient = TUSB_REQ_RCPT_INTERFACE,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = MSC_REQ_RESET,
    .wValue   = 0,
    .wIndex   = p_msc->itf_num,
    .wLength  = 0
  };
  TU_ASSERT( usbh_control_xfer( dev_addr, &new_request, NULL ) );
}
#endif

//--------------------------------------------------------------------+
// CLASS-USBH API
//--------------------------------------------------------------------+
bool msch_init(void) {
  TU_LOG_DRV("sizeof(msch_interface_t) = %u\r\n", sizeof(msch_interface_t));
  TU_LOG_DRV("sizeof(msch_epbuf_t) = %u\r\n", sizeof(msch_epbuf_t));
  tu_memclr(_msch_itf, sizeof(_msch_itf));
  return true;
}

bool msch_deinit(void) {
  return true;
}

void msch_close(uint8_t dev_addr) {
  TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX,);
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->configured,);

  TU_LOG_DRV("  MSCh close addr = %d\r\n", dev_addr);

  // invoke Application Callback
  if (p_msc->mounted) {
    if (tuh_msc_umount_cb) {
      tuh_msc_umount_cb(dev_addr);
    }
  }

  tu_memclr(p_msc, sizeof(msch_interface_t));
}

bool msch_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  msch_epbuf_t* epbuf = get_epbuf(dev_addr);
  msc_cbw_t const * cbw = &epbuf->cbw;
  msc_csw_t       * csw = &epbuf->csw;

  switch (p_msc->stage) {
    case MSC_STAGE_CMD:
      // Must be Command Block
      TU_ASSERT(ep_addr == p_msc->ep_out && event == XFER_RESULT_SUCCESS && xferred_bytes == sizeof(msc_cbw_t));
      if (cbw->total_bytes && p_msc->buffer) {
        // Data stage if any
        p_msc->stage = MSC_STAGE_DATA;
        uint8_t const ep_data = (cbw->dir & TUSB_DIR_IN_MASK) ? p_msc->ep_in : p_msc->ep_out;
        TU_ASSERT(usbh_edpt_xfer(dev_addr, ep_data, p_msc->buffer, (uint16_t) cbw->total_bytes));
        break;
      }

      TU_ATTR_FALLTHROUGH; // fallthrough to status stage

    case MSC_STAGE_DATA:
      // Status stage
      p_msc->stage = MSC_STAGE_STATUS;
      TU_ASSERT(usbh_edpt_xfer(dev_addr, p_msc->ep_in, (uint8_t*) csw, (uint16_t) sizeof(msc_csw_t)));
      break;

    case MSC_STAGE_STATUS:
      // SCSI op is complete
      p_msc->stage = MSC_STAGE_IDLE;

      if (p_msc->complete_cb) {
        tuh_msc_complete_data_t const cb_data = {
            .cbw = cbw,
            .csw = csw,
            .scsi_data = p_msc->buffer,
            .user_arg = p_msc->complete_arg
        };
        p_msc->complete_cb(dev_addr, &cb_data);
      }
      break;

      // unknown state
    default:
      break;
  }

  return true;
}

//--------------------------------------------------------------------+
// MSC Enumeration
//--------------------------------------------------------------------+
static void config_get_maxlun_complete(tuh_xfer_t* xfer);
static bool config_test_unit_ready_complete(uint8_t dev_addr, tuh_msc_complete_data_t const* cb_data);
static bool config_request_sense_complete(uint8_t dev_addr, tuh_msc_complete_data_t const* cb_data);
static bool config_read_capacity_complete(uint8_t dev_addr, tuh_msc_complete_data_t const* cb_data);

bool msch_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const* desc_itf, uint16_t max_len) {
  (void) rhport;
  TU_VERIFY (MSC_SUBCLASS_SCSI == desc_itf->bInterfaceSubClass &&
             MSC_PROTOCOL_BOT == desc_itf->bInterfaceProtocol);

  // msc driver length is fixed
  uint16_t const drv_len = (uint16_t) (sizeof(tusb_desc_interface_t) +
                                       desc_itf->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
  TU_ASSERT(drv_len <= max_len);

  msch_interface_t* p_msc = get_itf(dev_addr);
  tusb_desc_endpoint_t const* ep_desc = (tusb_desc_endpoint_t const*) tu_desc_next(desc_itf);

  for (uint32_t i = 0; i < 2; i++) {
    TU_ASSERT(TUSB_DESC_ENDPOINT == ep_desc->bDescriptorType && TUSB_XFER_BULK == ep_desc->bmAttributes.xfer);
    TU_ASSERT(tuh_edpt_open(dev_addr, ep_desc));

    if (TUSB_DIR_IN == tu_edpt_dir(ep_desc->bEndpointAddress)) {
      p_msc->ep_in = ep_desc->bEndpointAddress;
    } else {
      p_msc->ep_out = ep_desc->bEndpointAddress;
    }

    ep_desc = (tusb_desc_endpoint_t const*) tu_desc_next(ep_desc);
  }

  p_msc->itf_num = desc_itf->bInterfaceNumber;

  return true;
}

bool msch_set_config(uint8_t daddr, uint8_t itf_num) {
  msch_interface_t* p_msc = get_itf(daddr);
  TU_ASSERT(p_msc->itf_num == itf_num);
  p_msc->configured = true;

  //------------- Get Max Lun -------------//
  TU_LOG_DRV("MSC Get Max Lun\r\n");
  tusb_control_request_t const request = {
      .bmRequestType_bit = {
          .recipient = TUSB_REQ_RCPT_INTERFACE,
          .type      = TUSB_REQ_TYPE_CLASS,
          .direction = TUSB_DIR_IN
      },
      .bRequest = MSC_REQ_GET_MAX_LUN,
      .wValue   = 0,
      .wIndex   = itf_num,
      .wLength  = 1
  };

  uint8_t* enum_buf = usbh_get_enum_buf();
  tuh_xfer_t xfer = {
      .daddr       = daddr,
      .ep_addr     = 0,
      .setup       = &request,
      .buffer      = enum_buf,
      .complete_cb = config_get_maxlun_complete,
      .user_data    = 0
  };
  TU_ASSERT(tuh_control_xfer(&xfer));

  return true;
}

static void config_get_maxlun_complete(tuh_xfer_t* xfer) {
  uint8_t const daddr = xfer->daddr;
  msch_interface_t* p_msc = get_itf(daddr);

  // MAXLUN's response is minus 1 by specs, STALL means 1
  if (XFER_RESULT_SUCCESS == xfer->result) {
    uint8_t* enum_buf = usbh_get_enum_buf();
    p_msc->max_lun = enum_buf[0] + 1;
  } else {
    p_msc->max_lun = 1;
  }

  TU_LOG_DRV("  Max LUN = %u\r\n", p_msc->max_lun);

  // TODO multiple LUN support
  TU_LOG_DRV("SCSI Test Unit Ready\r\n");
  uint8_t const lun = 0;
  tuh_msc_test_unit_ready(daddr, lun, config_test_unit_ready_complete, 0);
}

static bool config_test_unit_ready_complete(uint8_t dev_addr, tuh_msc_complete_data_t const* cb_data) {
  msc_cbw_t const* cbw = cb_data->cbw;
  msc_csw_t const* csw = cb_data->csw;
  uint8_t* enum_buf = usbh_get_enum_buf();

  if (csw->status == 0) {
    // Unit is ready, read its capacity
    TU_LOG_DRV("SCSI Read Capacity\r\n");
    tuh_msc_read_capacity(dev_addr, cbw->lun, (scsi_read_capacity10_resp_t*) (uintptr_t) enum_buf,
                          config_read_capacity_complete, 0);
  } else {
    // Note: During enumeration, some device fails Test Unit Ready and require a few retries
    // with Request Sense to start working !!
    // TODO limit number of retries
    TU_LOG_DRV("SCSI Request Sense\r\n");
    TU_ASSERT(tuh_msc_request_sense(dev_addr, cbw->lun, enum_buf, config_request_sense_complete, 0));
  }

  return true;
}

static bool config_request_sense_complete(uint8_t dev_addr, tuh_msc_complete_data_t const* cb_data) {
  msc_cbw_t const* cbw = cb_data->cbw;
  msc_csw_t const* csw = cb_data->csw;

  TU_ASSERT(csw->status == 0);
  TU_ASSERT(tuh_msc_test_unit_ready(dev_addr, cbw->lun, config_test_unit_ready_complete, 0));
  return true;
}

static bool config_read_capacity_complete(uint8_t dev_addr, tuh_msc_complete_data_t const* cb_data) {
  msc_cbw_t const* cbw = cb_data->cbw;
  msc_csw_t const* csw = cb_data->csw;
  TU_ASSERT(csw->status == 0);
  msch_interface_t* p_msc = get_itf(dev_addr);
  uint8_t* enum_buf = usbh_get_enum_buf();

  // Capacity response field: Block size and Last LBA are both Big-Endian
  scsi_read_capacity10_resp_t* resp = (scsi_read_capacity10_resp_t*) (uintptr_t) enum_buf;
  p_msc->capacity[cbw->lun].block_count = tu_ntohl(resp->last_lba) + 1;
  p_msc->capacity[cbw->lun].block_size  = tu_ntohl(resp->block_size);

  // Mark enumeration is complete
  p_msc->mounted = true;
  if (tuh_msc_mount_cb) {
    tuh_msc_mount_cb(dev_addr);
  }

  // notify usbh that driver enumeration is complete
  usbh_driver_set_config_complete(dev_addr, p_msc->itf_num);

  return true;
}

#endif
