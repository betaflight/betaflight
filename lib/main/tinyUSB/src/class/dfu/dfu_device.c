/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 XMOS LIMITED
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

#if (CFG_TUD_ENABLED && CFG_TUD_DFU)

#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "dfu_device.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

// Level where CFG_TUSB_DEBUG must be at least for this driver is logged
#ifndef CFG_TUD_DFU_LOG_LEVEL
  #define CFG_TUD_DFU_LOG_LEVEL   CFG_TUD_LOG_LEVEL
#endif

#define TU_LOG_DRV(...)   TU_LOG(CFG_TUD_DFU_LOG_LEVEL, __VA_ARGS__)

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
typedef struct {
  uint8_t attrs;
  uint8_t alt;

  dfu_state_t state;
  dfu_status_t status;

  bool flashing_in_progress;
  uint16_t block;
  uint16_t length;
} dfu_state_ctx_t;

// Only a single dfu state is allowed
static dfu_state_ctx_t _dfu_ctx;

CFG_TUD_MEM_SECTION static struct {
  TUD_EPBUF_DEF(transfer_buf, CFG_TUD_DFU_XFER_BUFSIZE);
} _dfu_epbuf;

static void reset_state(void) {
  _dfu_ctx.state = DFU_IDLE;
  _dfu_ctx.status = DFU_STATUS_OK;
  _dfu_ctx.flashing_in_progress = false;
}

static bool reply_getstatus(uint8_t rhport, const tusb_control_request_t* request, dfu_state_t state, dfu_status_t status, uint32_t timeout);
static bool process_download_get_status(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request);
static bool process_manifest_get_status(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request);

//--------------------------------------------------------------------+
// Debug
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= 2

tu_static tu_lookup_entry_t const _dfu_request_lookup[] = {
  { .key = DFU_REQUEST_DETACH   , .data = "DETACH"    },
  { .key = DFU_REQUEST_DNLOAD   , .data = "DNLOAD"    },
  { .key = DFU_REQUEST_UPLOAD   , .data = "UPLOAD"    },
  { .key = DFU_REQUEST_GETSTATUS, .data = "GETSTATUS" },
  { .key = DFU_REQUEST_CLRSTATUS, .data = "CLRSTATUS" },
  { .key = DFU_REQUEST_GETSTATE , .data = "GETSTATE"  },
  { .key = DFU_REQUEST_ABORT    , .data = "ABORT"     },
};

tu_static tu_lookup_table_t const _dfu_request_table = {
  .count = TU_ARRAY_SIZE(_dfu_request_lookup),
  .items = _dfu_request_lookup
};

tu_static tu_lookup_entry_t const _dfu_state_lookup[] = {
  { .key = APP_IDLE               , .data = "APP_IDLE"            },
  { .key = APP_DETACH             , .data = "APP_DETACH"          },
  { .key = DFU_IDLE               , .data = "IDLE"                },
  { .key = DFU_DNLOAD_SYNC        , .data = "DNLOAD_SYNC"         },
  { .key = DFU_DNBUSY             , .data = "DNBUSY"              },
  { .key = DFU_DNLOAD_IDLE        , .data = "DNLOAD_IDLE"         },
  { .key = DFU_MANIFEST_SYNC      , .data = "MANIFEST_SYNC"       },
  { .key = DFU_MANIFEST           , .data = "MANIFEST"            },
  { .key = DFU_MANIFEST_WAIT_RESET, .data = "MANIFEST_WAIT_RESET" },
  { .key = DFU_UPLOAD_IDLE        , .data = "UPLOAD_IDLE"         },
  { .key = DFU_ERROR              , .data = "ERROR"               },
};

tu_static tu_lookup_table_t const _dfu_state_table = {
  .count = TU_ARRAY_SIZE(_dfu_state_lookup),
  .items = _dfu_state_lookup
};

tu_static tu_lookup_entry_t const _dfu_status_lookup[] = {
  { .key = DFU_STATUS_OK               , .data = "OK"              },
  { .key = DFU_STATUS_ERR_TARGET       , .data = "errTARGET"       },
  { .key = DFU_STATUS_ERR_FILE         , .data = "errFILE"         },
  { .key = DFU_STATUS_ERR_WRITE        , .data = "errWRITE"        },
  { .key = DFU_STATUS_ERR_ERASE        , .data = "errERASE"        },
  { .key = DFU_STATUS_ERR_CHECK_ERASED , .data = "errCHECK_ERASED" },
  { .key = DFU_STATUS_ERR_PROG         , .data = "errPROG"         },
  { .key = DFU_STATUS_ERR_VERIFY       , .data = "errVERIFY"       },
  { .key = DFU_STATUS_ERR_ADDRESS      , .data = "errADDRESS"      },
  { .key = DFU_STATUS_ERR_NOTDONE      , .data = "errNOTDONE"      },
  { .key = DFU_STATUS_ERR_FIRMWARE     , .data = "errFIRMWARE"     },
  { .key = DFU_STATUS_ERR_VENDOR       , .data = "errVENDOR"       },
  { .key = DFU_STATUS_ERR_USBR         , .data = "errUSBR"         },
  { .key = DFU_STATUS_ERR_POR          , .data = "errPOR"          },
  { .key = DFU_STATUS_ERR_UNKNOWN      , .data = "errUNKNOWN"      },
  { .key = DFU_STATUS_ERR_STALLEDPKT   , .data = "errSTALLEDPKT"   },
};

tu_static tu_lookup_table_t const _dfu_status_table = {
  .count = TU_ARRAY_SIZE(_dfu_status_lookup),
  .items = _dfu_status_lookup
};

#endif

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void dfu_moded_reset(uint8_t rhport) {
  (void) rhport;
  _dfu_ctx.attrs = 0;
  _dfu_ctx.alt = 0;
  reset_state();
}

void dfu_moded_init(void) {
  dfu_moded_reset(0);
}

bool dfu_moded_deinit(void) {
  return true;
}

uint16_t dfu_moded_open(uint8_t rhport, const tusb_desc_interface_t* itf_desc, uint16_t max_len) {
  (void) rhport;

  //------------- Interface (with Alt) descriptor -------------//
  const uint8_t itf_num = itf_desc->bInterfaceNumber;
  uint8_t alt_count = 0;

  uint16_t drv_len = 0;
  TU_VERIFY(itf_desc->bInterfaceSubClass == TUD_DFU_APP_SUBCLASS && itf_desc->bInterfaceProtocol == DFU_PROTOCOL_DFU, 0);

  while(itf_desc->bInterfaceSubClass == TUD_DFU_APP_SUBCLASS && itf_desc->bInterfaceProtocol == DFU_PROTOCOL_DFU) {
    TU_ASSERT(max_len > drv_len, 0);

    // Alternate must have the same interface number
    TU_ASSERT(itf_desc->bInterfaceNumber == itf_num, 0);

    // Alt should increase by one every time
    TU_ASSERT(itf_desc->bAlternateSetting == alt_count, 0);
    alt_count++;

    drv_len += tu_desc_len(itf_desc);
    itf_desc = (const tusb_desc_interface_t*) tu_desc_next(itf_desc);
  }

  //------------- DFU Functional descriptor -------------//
  const tusb_desc_dfu_functional_t*func_desc = (const tusb_desc_dfu_functional_t*) itf_desc;
  TU_ASSERT(tu_desc_type(func_desc) == TUSB_DESC_FUNCTIONAL, 0);
  drv_len += sizeof(tusb_desc_dfu_functional_t);

  _dfu_ctx.attrs = func_desc->bAttributes;

  // CFG_TUD_DFU_XFER_BUFSIZE has to be set to the buffer size used in TUD_DFU_DESCRIPTOR
  const uint16_t transfer_size = tu_le16toh( tu_unaligned_read16((const uint8_t*) func_desc + offsetof(tusb_desc_dfu_functional_t, wTransferSize)) );
  TU_ASSERT(transfer_size <= CFG_TUD_DFU_XFER_BUFSIZE, drv_len);

  return drv_len;
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool dfu_moded_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
  TU_VERIFY(request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE);
  TU_LOG_DRV("  DFU State  : %s, Status: %s\r\n", tu_lookup_find(&_dfu_state_table, _dfu_ctx.state), tu_lookup_find(&_dfu_status_table, _dfu_ctx.status));

  if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
    // Standard request include GET/SET_INTERFACE
    switch (request->bRequest) {
      case TUSB_REQ_SET_INTERFACE:
        if (stage == CONTROL_STAGE_SETUP) {
          // Switch Alt interface and reset state machine
          _dfu_ctx.alt = (uint8_t)request->wValue;
          reset_state();
          return tud_control_status(rhport, request);
        }
        break;

      case TUSB_REQ_GET_INTERFACE:
        if (stage == CONTROL_STAGE_SETUP) {
          return tud_control_xfer(rhport, request, &_dfu_ctx.alt, 1);
        }
        break;

      // unsupported request
      default: return false;
    }
  } else if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
    TU_LOG_DRV("  DFU Request: %s\r\n", tu_lookup_find(&_dfu_request_table, request->bRequest));

    // Class request
    switch (request->bRequest) {
      case DFU_REQUEST_DETACH:
        if (stage == CONTROL_STAGE_SETUP) {
          tud_control_status(rhport, request);
        } else if (stage == CONTROL_STAGE_ACK) {
          if (tud_dfu_detach_cb) {
            tud_dfu_detach_cb();
          }
        }
        break;

      case DFU_REQUEST_CLRSTATUS:
        if (stage == CONTROL_STAGE_SETUP) {
          reset_state();
          tud_control_status(rhport, request);
        }
        break;

      case DFU_REQUEST_GETSTATE:
        if (stage == CONTROL_STAGE_SETUP) {
          tud_control_xfer(rhport, request, &_dfu_ctx.state, 1);
        }
        break;

      case DFU_REQUEST_ABORT:
        if (stage == CONTROL_STAGE_SETUP) {
          reset_state();
          tud_control_status(rhport, request);
        } else if (stage == CONTROL_STAGE_ACK) {
          if (tud_dfu_abort_cb) {
            tud_dfu_abort_cb(_dfu_ctx.alt);
          }
        }
        break;

      case DFU_REQUEST_UPLOAD:
        if (stage == CONTROL_STAGE_SETUP) {
          TU_VERIFY(_dfu_ctx.attrs & DFU_ATTR_CAN_UPLOAD);
          TU_VERIFY(tud_dfu_upload_cb);
          TU_VERIFY(request->wLength <= CFG_TUD_DFU_XFER_BUFSIZE);

          const uint16_t xfer_len = tud_dfu_upload_cb(_dfu_ctx.alt, request->wValue, _dfu_epbuf.transfer_buf,
                                                      request->wLength);

          return tud_control_xfer(rhport, request, _dfu_epbuf.transfer_buf, xfer_len);
        }
        break;

      case DFU_REQUEST_DNLOAD:
        if (stage == CONTROL_STAGE_SETUP) {
          TU_VERIFY(_dfu_ctx.attrs & DFU_ATTR_CAN_DOWNLOAD);
          TU_VERIFY(_dfu_ctx.state == DFU_IDLE || _dfu_ctx.state == DFU_DNLOAD_IDLE);
          TU_VERIFY(request->wLength <= CFG_TUD_DFU_XFER_BUFSIZE);

          // set to true for both download and manifest
          _dfu_ctx.flashing_in_progress = true;

          // save block and length for flashing
          _dfu_ctx.block = request->wValue;
          _dfu_ctx.length = request->wLength;

          if (request->wLength) {
            // Download with payload -> transition to DOWNLOAD SYNC
            _dfu_ctx.state = DFU_DNLOAD_SYNC;
            return tud_control_xfer(rhport, request, _dfu_epbuf.transfer_buf, request->wLength);
          } else {
            // Download is complete -> transition to MANIFEST SYNC
            _dfu_ctx.state = DFU_MANIFEST_SYNC;
            return tud_control_status(rhport, request);
          }
        }
        break;

      case DFU_REQUEST_GETSTATUS:
        switch (_dfu_ctx.state) {
          case DFU_DNLOAD_SYNC:
            return process_download_get_status(rhport, stage, request);
            break;

          case DFU_MANIFEST_SYNC:
            return process_manifest_get_status(rhport, stage, request);
            break;

          default:
            if (stage == CONTROL_STAGE_SETUP) {
              return reply_getstatus(rhport, request, _dfu_ctx.state, _dfu_ctx.status, 0);
            }
            break;
        }
        break;

      default: return false; // stall unsupported request
    }
  } else {
    return false; // unsupported request
  }

  return true;
}

void tud_dfu_finish_flashing(uint8_t status) {
  _dfu_ctx.flashing_in_progress = false;

  if (status == DFU_STATUS_OK) {
    if (_dfu_ctx.state == DFU_DNBUSY) {
      _dfu_ctx.state = DFU_DNLOAD_SYNC;
    } else if (_dfu_ctx.state == DFU_MANIFEST) {
      _dfu_ctx.state = (_dfu_ctx.attrs & DFU_ATTR_MANIFESTATION_TOLERANT)
                         ? DFU_MANIFEST_SYNC
                         : DFU_MANIFEST_WAIT_RESET;
    }
  } else {
    // failed while flashing, move to dfuError
    _dfu_ctx.state = DFU_ERROR;
    _dfu_ctx.status = (dfu_status_t)status;
  }
}

static bool process_download_get_status(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
  if (stage == CONTROL_STAGE_SETUP) {
    // only transition to next state on CONTROL_STAGE_ACK
    dfu_state_t next_state;
    uint32_t timeout;

    if (_dfu_ctx.flashing_in_progress) {
      next_state = DFU_DNBUSY;
      timeout = tud_dfu_get_timeout_cb(_dfu_ctx.alt, (uint8_t)next_state);
    } else {
      next_state = DFU_DNLOAD_IDLE;
      timeout = 0;
    }

    return reply_getstatus(rhport, request, next_state, _dfu_ctx.status, timeout);
  } else if (stage == CONTROL_STAGE_ACK) {
    if (_dfu_ctx.flashing_in_progress) {
      _dfu_ctx.state = DFU_DNBUSY;
      tud_dfu_download_cb(_dfu_ctx.alt, _dfu_ctx.block, _dfu_epbuf.transfer_buf, _dfu_ctx.length);
    } else {
      _dfu_ctx.state = DFU_DNLOAD_IDLE;
    }
  }

  return true;
}

static bool process_manifest_get_status(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
  if (stage == CONTROL_STAGE_SETUP) {
    // only transition to next state on CONTROL_STAGE_ACK
    dfu_state_t next_state;
    uint32_t timeout;

    if (_dfu_ctx.flashing_in_progress) {
      next_state = DFU_MANIFEST;
      timeout = tud_dfu_get_timeout_cb(_dfu_ctx.alt, next_state);
    } else {
      next_state = DFU_IDLE;
      timeout = 0;
    }

    return reply_getstatus(rhport, request, next_state, _dfu_ctx.status, timeout);
  } else if (stage == CONTROL_STAGE_ACK) {
    if (_dfu_ctx.flashing_in_progress) {
      _dfu_ctx.state = DFU_MANIFEST;
      tud_dfu_manifest_cb(_dfu_ctx.alt);
    } else {
      _dfu_ctx.state = DFU_IDLE;
    }
  }

  return true;
}

static bool reply_getstatus(uint8_t rhport, const tusb_control_request_t* request, dfu_state_t state,
                            dfu_status_t status, uint32_t timeout) {
  dfu_status_response_t resp;
  resp.bStatus = (uint8_t)status;
  resp.bwPollTimeout[0] = TU_U32_BYTE0(timeout);
  resp.bwPollTimeout[1] = TU_U32_BYTE1(timeout);
  resp.bwPollTimeout[2] = TU_U32_BYTE2(timeout);
  resp.bState = (uint8_t)state;
  resp.iString = 0;

  return tud_control_xfer(rhport, request, &resp, sizeof(dfu_status_response_t));
}

#endif
