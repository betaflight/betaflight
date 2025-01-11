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

#if CFG_TUD_ENABLED

#include "device/dcd.h"
#include "tusb.h"
#include "common/tusb_private.h"

#include "device/usbd.h"
#include "device/usbd_pvt.h"

//--------------------------------------------------------------------+
// USBD Configuration
//--------------------------------------------------------------------+
#ifndef CFG_TUD_TASK_QUEUE_SZ
  #define CFG_TUD_TASK_QUEUE_SZ   16
#endif

//--------------------------------------------------------------------+
// Weak stubs: invoked if no strong implementation is available
//--------------------------------------------------------------------+
TU_ATTR_WEAK void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr) {
  (void) rhport; (void) eventid; (void) in_isr;
}

TU_ATTR_WEAK void tud_sof_cb(uint32_t frame_count) {
  (void) frame_count;
}

TU_ATTR_WEAK uint8_t const* tud_descriptor_bos_cb(void) {
  return NULL;
}

TU_ATTR_WEAK uint8_t const* tud_descriptor_device_qualifier_cb(void) {
  return NULL;
}

TU_ATTR_WEAK uint8_t const* tud_descriptor_other_speed_configuration_cb(uint8_t index) {
  (void) index;
  return NULL;
}

TU_ATTR_WEAK void tud_mount_cb(void) {
}

TU_ATTR_WEAK void tud_umount_cb(void) {
}

TU_ATTR_WEAK void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
}

TU_ATTR_WEAK void tud_resume_cb(void) {
}

TU_ATTR_WEAK bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
  (void) rhport; (void) stage; (void) request;
  return false;
}

TU_ATTR_WEAK bool dcd_deinit(uint8_t rhport) {
  (void) rhport;
  return false;
}

TU_ATTR_WEAK void dcd_connect(uint8_t rhport) {
  (void) rhport;
}

TU_ATTR_WEAK void dcd_disconnect(uint8_t rhport) {
  (void) rhport;
}

TU_ATTR_WEAK bool dcd_dcache_clean(const void* addr, uint32_t data_size) {
  (void) addr; (void) data_size;
  return true;
}

TU_ATTR_WEAK bool dcd_dcache_invalidate(const void* addr, uint32_t data_size) {
  (void) addr; (void) data_size;
  return true;
}

TU_ATTR_WEAK bool dcd_dcache_clean_invalidate(const void* addr, uint32_t data_size) {
  (void) addr; (void) data_size;
  return true;
}

//--------------------------------------------------------------------+
// Device Data
//--------------------------------------------------------------------+

// Invalid driver ID in itf2drv[] ep2drv[][] mapping
enum { DRVID_INVALID = 0xFFu };

typedef struct {
  struct TU_ATTR_PACKED {
    volatile uint8_t connected    : 1;
    volatile uint8_t addressed    : 1;
    volatile uint8_t suspended    : 1;

    uint8_t remote_wakeup_en      : 1; // enable/disable by host
    uint8_t remote_wakeup_support : 1; // configuration descriptor's attribute
    uint8_t self_powered          : 1; // configuration descriptor's attribute
  };
  volatile uint8_t cfg_num; // current active configuration (0x00 is not configured)
  uint8_t speed;
  volatile uint8_t sof_consumer;

  uint8_t itf2drv[CFG_TUD_INTERFACE_MAX];   // map interface number to driver (0xff is invalid)
  uint8_t ep2drv[CFG_TUD_ENDPPOINT_MAX][2]; // map endpoint to driver ( 0xff is invalid ), can use only 4-bit each

  tu_edpt_state_t ep_status[CFG_TUD_ENDPPOINT_MAX][2];

}usbd_device_t;

tu_static usbd_device_t _usbd_dev;
static volatile uint8_t _usbd_queued_setup;

//--------------------------------------------------------------------+
// Class Driver
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
  #define DRIVER_NAME(_name)  _name
#else
  #define DRIVER_NAME(_name)  NULL
#endif

// Built-in class drivers
tu_static usbd_class_driver_t const _usbd_driver[] = {
    #if CFG_TUD_CDC
    {
        .name             = DRIVER_NAME("CDC"),
        .init             = cdcd_init,
        .deinit           = cdcd_deinit,
        .reset            = cdcd_reset,
        .open             = cdcd_open,
        .control_xfer_cb  = cdcd_control_xfer_cb,
        .xfer_cb          = cdcd_xfer_cb,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_MSC
    {
        .name             = DRIVER_NAME("MSC"),
        .init             = mscd_init,
        .deinit           = NULL,
        .reset            = mscd_reset,
        .open             = mscd_open,
        .control_xfer_cb  = mscd_control_xfer_cb,
        .xfer_cb          = mscd_xfer_cb,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_HID
    {
        .name             = DRIVER_NAME("HID"),
        .init             = hidd_init,
        .deinit           = hidd_deinit,
        .reset            = hidd_reset,
        .open             = hidd_open,
        .control_xfer_cb  = hidd_control_xfer_cb,
        .xfer_cb          = hidd_xfer_cb,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_AUDIO
    {
        .name             = DRIVER_NAME("AUDIO"),
        .init             = audiod_init,
        .deinit           = audiod_deinit,
        .reset            = audiod_reset,
        .open             = audiod_open,
        .control_xfer_cb  = audiod_control_xfer_cb,
        .xfer_cb          = audiod_xfer_cb,
        .sof              = audiod_sof_isr
    },
    #endif

    #if CFG_TUD_VIDEO
    {
        .name             = DRIVER_NAME("VIDEO"),
        .init             = videod_init,
        .deinit           = videod_deinit,
        .reset            = videod_reset,
        .open             = videod_open,
        .control_xfer_cb  = videod_control_xfer_cb,
        .xfer_cb          = videod_xfer_cb,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_MIDI
    {
        .name             = DRIVER_NAME("MIDI"),
        .init             = midid_init,
        .deinit           = midid_deinit,
        .open             = midid_open,
        .reset            = midid_reset,
        .control_xfer_cb  = midid_control_xfer_cb,
        .xfer_cb          = midid_xfer_cb,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_VENDOR
    {
        .name             = DRIVER_NAME("VENDOR"),
        .init             = vendord_init,
        .deinit           = vendord_deinit,
        .reset            = vendord_reset,
        .open             = vendord_open,
        .control_xfer_cb  = tud_vendor_control_xfer_cb,
        .xfer_cb          = vendord_xfer_cb,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_USBTMC
    {
        .name             = DRIVER_NAME("TMC"),
        .init             = usbtmcd_init_cb,
        .deinit           = usbtmcd_deinit,
        .reset            = usbtmcd_reset_cb,
        .open             = usbtmcd_open_cb,
        .control_xfer_cb  = usbtmcd_control_xfer_cb,
        .xfer_cb          = usbtmcd_xfer_cb,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_DFU_RUNTIME
    {
        .name             = DRIVER_NAME("DFU-RUNTIME"),
        .init             = dfu_rtd_init,
        .deinit           = dfu_rtd_deinit,
        .reset            = dfu_rtd_reset,
        .open             = dfu_rtd_open,
        .control_xfer_cb  = dfu_rtd_control_xfer_cb,
        .xfer_cb          = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_DFU
    {
        .name             = DRIVER_NAME("DFU"),
        .init             = dfu_moded_init,
        .deinit           = dfu_moded_deinit,
        .reset            = dfu_moded_reset,
        .open             = dfu_moded_open,
        .control_xfer_cb  = dfu_moded_control_xfer_cb,
        .xfer_cb          = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_ECM_RNDIS || CFG_TUD_NCM
    {
        .name             = DRIVER_NAME("NET"),
        .init             = netd_init,
        .deinit           = netd_deinit,
        .reset            = netd_reset,
        .open             = netd_open,
        .control_xfer_cb  = netd_control_xfer_cb,
        .xfer_cb          = netd_xfer_cb,
        .sof                  = NULL,
    },
    #endif

    #if CFG_TUD_BTH
    {
        .name             = DRIVER_NAME("BTH"),
        .init             = btd_init,
        .deinit           = btd_deinit,
        .reset            = btd_reset,
        .open             = btd_open,
        .control_xfer_cb  = btd_control_xfer_cb,
        .xfer_cb          = btd_xfer_cb,
        .sof              = NULL
    },
    #endif
};

enum { BUILTIN_DRIVER_COUNT = TU_ARRAY_SIZE(_usbd_driver) };

// Additional class drivers implemented by application
tu_static usbd_class_driver_t const * _app_driver = NULL;
tu_static uint8_t _app_driver_count = 0;

#define TOTAL_DRIVER_COUNT    (_app_driver_count + BUILTIN_DRIVER_COUNT)

// virtually joins built-in and application drivers together.
// Application is positioned first to allow overwriting built-in ones.
TU_ATTR_ALWAYS_INLINE static inline usbd_class_driver_t const * get_driver(uint8_t drvid) {
  usbd_class_driver_t const * driver = NULL;
  if ( drvid < _app_driver_count ) {
    // Application drivers
    driver = &_app_driver[drvid];
  } else if ( drvid < TOTAL_DRIVER_COUNT && BUILTIN_DRIVER_COUNT > 0 ){
    driver = &_usbd_driver[drvid - _app_driver_count];
  }
  return driver;
}


//--------------------------------------------------------------------+
// DCD Event
//--------------------------------------------------------------------+

enum { RHPORT_INVALID = 0xFFu };
tu_static uint8_t _usbd_rhport = RHPORT_INVALID;

// Event queue
// usbd_int_set() is used as mutex in OS NONE config
OSAL_QUEUE_DEF(usbd_int_set, _usbd_qdef, CFG_TUD_TASK_QUEUE_SZ, dcd_event_t);
tu_static osal_queue_t _usbd_q;

// Mutex for claiming endpoint
#if OSAL_MUTEX_REQUIRED
  tu_static osal_mutex_def_t _ubsd_mutexdef;
  tu_static osal_mutex_t _usbd_mutex;
#else
  #define _usbd_mutex   NULL
#endif

TU_ATTR_ALWAYS_INLINE static inline bool queue_event(dcd_event_t const * event, bool in_isr) {
  TU_ASSERT(osal_queue_send(_usbd_q, event, in_isr));
  tud_event_hook_cb(event->rhport, event->event_id, in_isr);
  return true;
}

//--------------------------------------------------------------------+
// Prototypes
//--------------------------------------------------------------------+
static bool process_control_request(uint8_t rhport, tusb_control_request_t const * p_request);
static bool process_set_config(uint8_t rhport, uint8_t cfg_num);
static bool process_get_descriptor(uint8_t rhport, tusb_control_request_t const * p_request);

#if CFG_TUD_TEST_MODE
static bool process_test_mode_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
  TU_VERIFY(CONTROL_STAGE_ACK == stage);
  uint8_t const selector = tu_u16_high(request->wIndex);
  TU_LOG_USBD("    Enter Test Mode (test selector index: %d)\r\n", selector);
  dcd_enter_test_mode(rhport, (tusb_feature_test_mode_t) selector);
  return true;
}
#endif

// from usbd_control.c
void usbd_control_reset(void);
void usbd_control_set_request(tusb_control_request_t const *request);
void usbd_control_set_complete_callback( usbd_control_xfer_cb_t fp );
bool usbd_control_xfer_cb (uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);


//--------------------------------------------------------------------+
// Debug
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
tu_static char const* const _usbd_event_str[DCD_EVENT_COUNT] = {
    "Invalid",
    "Bus Reset",
    "Unplugged",
    "SOF",
    "Suspend",
    "Resume",
    "Setup Received",
    "Xfer Complete",
    "Func Call"
};

// for usbd_control to print the name of control complete driver
void usbd_driver_print_control_complete_name(usbd_control_xfer_cb_t callback) {
  for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++) {
    usbd_class_driver_t const* driver = get_driver(i);
    if (driver && driver->control_xfer_cb == callback) {
      TU_LOG_USBD("%s control complete\r\n", driver->name);
      return;
    }
  }
}

#endif

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+
tusb_speed_t tud_speed_get(void) {
  return (tusb_speed_t) _usbd_dev.speed;
}

bool tud_connected(void) {
  return _usbd_dev.connected;
}

bool tud_mounted(void) {
  return _usbd_dev.cfg_num ? true : false;
}

bool tud_suspended(void) {
  return _usbd_dev.suspended;
}

bool tud_remote_wakeup(void) {
  // only wake up host if this feature is supported and enabled and we are suspended
  TU_VERIFY (_usbd_dev.suspended && _usbd_dev.remote_wakeup_support && _usbd_dev.remote_wakeup_en);
  dcd_remote_wakeup(_usbd_rhport);
  return true;
}

bool tud_disconnect(void) {
  dcd_disconnect(_usbd_rhport);
  return true;
}

bool tud_connect(void) {
  dcd_connect(_usbd_rhport);
  return true;
}

void tud_sof_cb_enable(bool en) {
  usbd_sof_enable(_usbd_rhport, SOF_CONSUMER_USER, en);
}

//--------------------------------------------------------------------+
// USBD Task
//--------------------------------------------------------------------+
bool tud_inited(void) {
  return _usbd_rhport != RHPORT_INVALID;
}

bool tud_rhport_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  if (tud_inited()) {
    return true; // skip if already initialized
  }
  TU_ASSERT(rh_init);

  TU_LOG_USBD("USBD init on controller %u, speed = %s\r\n", rhport,
    rh_init->speed == TUSB_SPEED_HIGH ? "High" : "Full");
  TU_LOG_INT(CFG_TUD_LOG_LEVEL, sizeof(usbd_device_t));
  TU_LOG_INT(CFG_TUD_LOG_LEVEL, sizeof(dcd_event_t));
  TU_LOG_INT(CFG_TUD_LOG_LEVEL, sizeof(tu_fifo_t));
  TU_LOG_INT(CFG_TUD_LOG_LEVEL, sizeof(tu_edpt_stream_t));

  tu_varclr(&_usbd_dev);
  _usbd_queued_setup = 0;

#if OSAL_MUTEX_REQUIRED
  // Init device mutex
  _usbd_mutex = osal_mutex_create(&_ubsd_mutexdef);
  TU_ASSERT(_usbd_mutex);
#endif

  // Init device queue & task
  _usbd_q = osal_queue_create(&_usbd_qdef);
  TU_ASSERT(_usbd_q);

  // Get application driver if available
  if (usbd_app_driver_get_cb) {
    _app_driver = usbd_app_driver_get_cb(&_app_driver_count);
  }

  // Init class drivers
  for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++) {
    usbd_class_driver_t const* driver = get_driver(i);
    TU_ASSERT(driver && driver->init);
    TU_LOG_USBD("%s init\r\n", driver->name);
    driver->init();
  }

  _usbd_rhport = rhport;

  // Init device controller driver
  TU_ASSERT(dcd_init(rhport, rh_init));
  dcd_int_enable(rhport);

  return true;
}

bool tud_deinit(uint8_t rhport) {
  if (!tud_inited()) {
    return true; // skip if not initialized
  }

  TU_LOG_USBD("USBD deinit on controller %u\r\n", rhport);

  // Deinit device controller driver
  dcd_int_disable(rhport);
  dcd_disconnect(rhport);
  dcd_deinit(rhport);

  // Deinit class drivers
  for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++) {
    usbd_class_driver_t const* driver = get_driver(i);
    if(driver && driver->deinit) {
      TU_LOG_USBD("%s deinit\r\n", driver->name);
      driver->deinit();
    }
  }

  // Deinit device queue & task
  osal_queue_delete(_usbd_q);
  _usbd_q = NULL;

#if OSAL_MUTEX_REQUIRED
  // TODO make sure there is no task waiting on this mutex
  osal_mutex_delete(_usbd_mutex);
  _usbd_mutex = NULL;
#endif

  _usbd_rhport = RHPORT_INVALID;

  return true;
}

static void configuration_reset(uint8_t rhport) {
  for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++) {
    usbd_class_driver_t const* driver = get_driver(i);
    TU_ASSERT(driver,);
    driver->reset(rhport);
  }

  tu_varclr(&_usbd_dev);
  memset(_usbd_dev.itf2drv, DRVID_INVALID, sizeof(_usbd_dev.itf2drv)); // invalid mapping
  memset(_usbd_dev.ep2drv, DRVID_INVALID, sizeof(_usbd_dev.ep2drv)); // invalid mapping
}

static void usbd_reset(uint8_t rhport) {
  configuration_reset(rhport);
  usbd_control_reset();
}

bool tud_task_event_ready(void) {
  // Skip if stack is not initialized
  if (!tud_inited()) return false;
  return !osal_queue_empty(_usbd_q);
}

/* USB Device Driver task
 * This top level thread manages all device controller event and delegates events to class-specific drivers.
 * This should be called periodically within the mainloop or rtos thread.
 *
    int main(void) {
      application_init();
      tusb_init(0, TUSB_ROLE_DEVICE);

      while(1) { // the mainloop
        application_code();
        tud_task(); // tinyusb device task
      }
    }
 */
void tud_task_ext(uint32_t timeout_ms, bool in_isr) {
  (void) in_isr; // not implemented yet

  // Skip if stack is not initialized
  if (!tud_inited()) return;

  // Loop until there is no more events in the queue
  while (1) {
    dcd_event_t event;
    if (!osal_queue_receive(_usbd_q, &event, timeout_ms)) return;

#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
    if (event.event_id == DCD_EVENT_SETUP_RECEIVED) TU_LOG_USBD("\r\n"); // extra line for setup
    TU_LOG_USBD("USBD %s ", event.event_id < DCD_EVENT_COUNT ? _usbd_event_str[event.event_id] : "CORRUPTED");
#endif

    switch (event.event_id) {
      case DCD_EVENT_BUS_RESET:
        TU_LOG_USBD(": %s Speed\r\n", tu_str_speed[event.bus_reset.speed]);
        usbd_reset(event.rhport);
        _usbd_dev.speed = event.bus_reset.speed;
        break;

      case DCD_EVENT_UNPLUGGED:
        TU_LOG_USBD("\r\n");
        usbd_reset(event.rhport);
        tud_umount_cb();
        break;

      case DCD_EVENT_SETUP_RECEIVED:
        TU_ASSERT(_usbd_queued_setup > 0,);
        _usbd_queued_setup--;
        TU_LOG_BUF(CFG_TUD_LOG_LEVEL, &event.setup_received, 8);
        if (_usbd_queued_setup) {
          TU_LOG_USBD("  Skipped since there is other SETUP in queue\r\n");
          break;
        }

        // Mark as connected after receiving 1st setup packet.
        // But it is easier to set it every time instead of wasting time to check then set
        _usbd_dev.connected = 1;

        // mark both in & out control as free
        _usbd_dev.ep_status[0][TUSB_DIR_OUT].busy = 0;
        _usbd_dev.ep_status[0][TUSB_DIR_OUT].claimed = 0;
        _usbd_dev.ep_status[0][TUSB_DIR_IN].busy = 0;
        _usbd_dev.ep_status[0][TUSB_DIR_IN].claimed = 0;

        // Process control request
        if (!process_control_request(event.rhport, &event.setup_received)) {
          TU_LOG_USBD("  Stall EP0\r\n");
          // Failed -> stall both control endpoint IN and OUT
          dcd_edpt_stall(event.rhport, 0);
          dcd_edpt_stall(event.rhport, 0 | TUSB_DIR_IN_MASK);
        }
        break;

      case DCD_EVENT_XFER_COMPLETE: {
        // Invoke the class callback associated with the endpoint address
        uint8_t const ep_addr = event.xfer_complete.ep_addr;
        uint8_t const epnum = tu_edpt_number(ep_addr);
        uint8_t const ep_dir = tu_edpt_dir(ep_addr);

        TU_LOG_USBD("on EP %02X with %u bytes\r\n", ep_addr, (unsigned int) event.xfer_complete.len);

        _usbd_dev.ep_status[epnum][ep_dir].busy = 0;
        _usbd_dev.ep_status[epnum][ep_dir].claimed = 0;

        if (0 == epnum) {
          usbd_control_xfer_cb(event.rhport, ep_addr, (xfer_result_t) event.xfer_complete.result,
                               event.xfer_complete.len);
        } else {
          usbd_class_driver_t const* driver = get_driver(_usbd_dev.ep2drv[epnum][ep_dir]);
          TU_ASSERT(driver,);

          TU_LOG_USBD("  %s xfer callback\r\n", driver->name);
          driver->xfer_cb(event.rhport, ep_addr, (xfer_result_t) event.xfer_complete.result, event.xfer_complete.len);
        }
        break;
      }

      case DCD_EVENT_SUSPEND:
        // NOTE: When plugging/unplugging device, the D+/D- state are unstable and
        // can accidentally meet the SUSPEND condition ( Bus Idle for 3ms ), which result in a series of event
        // e.g suspend -> resume -> unplug/plug. Skip suspend/resume if not connected
        if (_usbd_dev.connected) {
          TU_LOG_USBD(": Remote Wakeup = %u\r\n", _usbd_dev.remote_wakeup_en);
          tud_suspend_cb(_usbd_dev.remote_wakeup_en);
        } else {
          TU_LOG_USBD(" Skipped\r\n");
        }
        break;

      case DCD_EVENT_RESUME:
        if (_usbd_dev.connected) {
          TU_LOG_USBD("\r\n");
          tud_resume_cb();
        } else {
          TU_LOG_USBD(" Skipped\r\n");
        }
        break;

      case USBD_EVENT_FUNC_CALL:
        TU_LOG_USBD("\r\n");
        if (event.func_call.func) event.func_call.func(event.func_call.param);
        break;

      case DCD_EVENT_SOF:
        if (tu_bit_test(_usbd_dev.sof_consumer, SOF_CONSUMER_USER)) {
          TU_LOG_USBD("\r\n");
          tud_sof_cb(event.sof.frame_count);
        }
      break;

      default:
        TU_BREAKPOINT();
        break;
    }

#if CFG_TUSB_OS != OPT_OS_NONE && CFG_TUSB_OS != OPT_OS_PICO
    // return if there is no more events, for application to run other background
    if (osal_queue_empty(_usbd_q)) return;
#endif
  }
}

//--------------------------------------------------------------------+
// Control Request Parser & Handling
//--------------------------------------------------------------------+

// Helper to invoke class driver control request handler
static bool invoke_class_control(uint8_t rhport, usbd_class_driver_t const * driver, tusb_control_request_t const * request) {
  usbd_control_set_complete_callback(driver->control_xfer_cb);
  TU_LOG_USBD("  %s control request\r\n", driver->name);
  return driver->control_xfer_cb(rhport, CONTROL_STAGE_SETUP, request);
}

// This handles the actual request and its response.
// Returns false if unable to complete the request, causing caller to stall control endpoints.
static bool process_control_request(uint8_t rhport, tusb_control_request_t const * p_request) {
  usbd_control_set_complete_callback(NULL);
  TU_ASSERT(p_request->bmRequestType_bit.type < TUSB_REQ_TYPE_INVALID);

  // Vendor request
  if ( p_request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR ) {
    usbd_control_set_complete_callback(tud_vendor_control_xfer_cb);
    return tud_vendor_control_xfer_cb(rhport, CONTROL_STAGE_SETUP, p_request);
  }

#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
  if (TUSB_REQ_TYPE_STANDARD == p_request->bmRequestType_bit.type && p_request->bRequest <= TUSB_REQ_SYNCH_FRAME) {
    TU_LOG_USBD("  %s", tu_str_std_request[p_request->bRequest]);
    if (TUSB_REQ_GET_DESCRIPTOR != p_request->bRequest) TU_LOG_USBD("\r\n");
  }
#endif

  switch ( p_request->bmRequestType_bit.recipient ) {
    //------------- Device Requests e.g in enumeration -------------//
    case TUSB_REQ_RCPT_DEVICE:
      if ( TUSB_REQ_TYPE_CLASS == p_request->bmRequestType_bit.type ) {
        uint8_t const itf = tu_u16_low(p_request->wIndex);
        TU_VERIFY(itf < TU_ARRAY_SIZE(_usbd_dev.itf2drv));

        usbd_class_driver_t const * driver = get_driver(_usbd_dev.itf2drv[itf]);
        TU_VERIFY(driver);

        // forward to class driver: "non-STD request to Interface"
        return invoke_class_control(rhport, driver, p_request);
      }

      if ( TUSB_REQ_TYPE_STANDARD != p_request->bmRequestType_bit.type ) {
        // Non-standard request is not supported
        TU_BREAKPOINT();
        return false;
      }

      switch ( p_request->bRequest ) {
        case TUSB_REQ_SET_ADDRESS:
          // Depending on mcu, status phase could be sent either before or after changing device address,
          // or even require stack to not response with status at all
          // Therefore DCD must take full responsibility to response and include zlp status packet if needed.
          usbd_control_set_request(p_request); // set request since DCD has no access to tud_control_status() API
          dcd_set_address(rhport, (uint8_t) p_request->wValue);
          // skip tud_control_status()
          _usbd_dev.addressed = 1;
        break;

        case TUSB_REQ_GET_CONFIGURATION: {
          uint8_t cfg_num = _usbd_dev.cfg_num;
          tud_control_xfer(rhport, p_request, &cfg_num, 1);
        }
        break;

        case TUSB_REQ_SET_CONFIGURATION: {
          uint8_t const cfg_num = (uint8_t) p_request->wValue;

          // Only process if new configure is different
          if (_usbd_dev.cfg_num != cfg_num) {
            if ( _usbd_dev.cfg_num ) {
              // already configured: need to clear all endpoints and driver first
              TU_LOG_USBD("  Clear current Configuration (%u) before switching\r\n", _usbd_dev.cfg_num);

              // disable SOF
              dcd_sof_enable(rhport, false);

              // close all non-control endpoints, cancel all pending transfers if any
              dcd_edpt_close_all(rhport);

              // close all drivers and current configured state except bus speed
              uint8_t const speed = _usbd_dev.speed;
              configuration_reset(rhport);

              _usbd_dev.speed = speed; // restore speed
            }

            _usbd_dev.cfg_num = cfg_num;

            // Handle the new configuration and execute the corresponding callback
            if ( cfg_num ) {
              // switch to new configuration if not zero
              if (!process_set_config(rhport, cfg_num)) {
                TU_MESS_FAILED();
                TU_BREAKPOINT();
                _usbd_dev.cfg_num = 0;
                return false;
              }
              tud_mount_cb();
            } else {
              tud_umount_cb();
            }
          }

          tud_control_status(rhport, p_request);
        }
        break;

        case TUSB_REQ_GET_DESCRIPTOR:
          TU_VERIFY( process_get_descriptor(rhport, p_request) );
        break;

        case TUSB_REQ_SET_FEATURE:
          switch(p_request->wValue) {
            case TUSB_REQ_FEATURE_REMOTE_WAKEUP:
              TU_LOG_USBD("    Enable Remote Wakeup\r\n");
              // Host may enable remote wake up before suspending especially HID device
              _usbd_dev.remote_wakeup_en = true;
              tud_control_status(rhport, p_request);
            break;

            #if CFG_TUD_TEST_MODE
            case TUSB_REQ_FEATURE_TEST_MODE: {
              // Only handle the test mode if supported and valid
              TU_VERIFY(0 == tu_u16_low(p_request->wIndex));

              uint8_t const selector = tu_u16_high(p_request->wIndex);
              TU_VERIFY(TUSB_FEATURE_TEST_J <= selector && selector <= TUSB_FEATURE_TEST_FORCE_ENABLE);

              usbd_control_set_complete_callback(process_test_mode_cb);
              tud_control_status(rhport, p_request);
              break;
            }
            #endif /* CFG_TUD_TEST_MODE */

            // Stall unsupported feature selector
            default: return false;
          }
        break;

        case TUSB_REQ_CLEAR_FEATURE:
          // Only support remote wakeup for device feature
          TU_VERIFY(TUSB_REQ_FEATURE_REMOTE_WAKEUP == p_request->wValue);

          TU_LOG_USBD("    Disable Remote Wakeup\r\n");

          // Host may disable remote wake up after resuming
          _usbd_dev.remote_wakeup_en = false;
          tud_control_status(rhport, p_request);
        break;

        case TUSB_REQ_GET_STATUS: {
          // Device status bit mask
          // - Bit 0: Self Powered
          // - Bit 1: Remote Wakeup enabled
          uint16_t status = (uint16_t) ((_usbd_dev.self_powered ? 1u : 0u) | (_usbd_dev.remote_wakeup_en ? 2u : 0u));
          tud_control_xfer(rhport, p_request, &status, 2);
          break;
        }

        // Unknown/Unsupported request
        default: TU_BREAKPOINT(); return false;
      }
    break;

    //------------- Class/Interface Specific Request -------------//
    case TUSB_REQ_RCPT_INTERFACE: {
      uint8_t const itf = tu_u16_low(p_request->wIndex);
      TU_VERIFY(itf < TU_ARRAY_SIZE(_usbd_dev.itf2drv));

      usbd_class_driver_t const * driver = get_driver(_usbd_dev.itf2drv[itf]);
      TU_VERIFY(driver);

      // all requests to Interface (STD or Class) is forwarded to class driver.
      // notable requests are: GET HID REPORT DESCRIPTOR, SET_INTERFACE, GET_INTERFACE
      if ( !invoke_class_control(rhport, driver, p_request) ) {
        // For GET_INTERFACE and SET_INTERFACE, it is mandatory to respond even if the class
        // driver doesn't use alternate settings or implement this
        TU_VERIFY(TUSB_REQ_TYPE_STANDARD == p_request->bmRequestType_bit.type);

        switch(p_request->bRequest) {
          case TUSB_REQ_GET_INTERFACE:
          case TUSB_REQ_SET_INTERFACE:
            // Clear complete callback if driver set since it can also stall the request.
            usbd_control_set_complete_callback(NULL);

            if (TUSB_REQ_GET_INTERFACE == p_request->bRequest) {
              uint8_t alternate = 0;
              tud_control_xfer(rhport, p_request, &alternate, 1);
            }else {
              tud_control_status(rhport, p_request);
            }
          break;

          default: return false;
        }
      }
      break;
    }

    //------------- Endpoint Request -------------//
    case TUSB_REQ_RCPT_ENDPOINT: {
      uint8_t const ep_addr = tu_u16_low(p_request->wIndex);
      uint8_t const ep_num  = tu_edpt_number(ep_addr);
      uint8_t const ep_dir  = tu_edpt_dir(ep_addr);

      TU_ASSERT(ep_num < TU_ARRAY_SIZE(_usbd_dev.ep2drv) );
      usbd_class_driver_t const * driver = get_driver(_usbd_dev.ep2drv[ep_num][ep_dir]);

      if ( TUSB_REQ_TYPE_STANDARD != p_request->bmRequestType_bit.type ) {
        // Forward class request to its driver
        TU_VERIFY(driver);
        return invoke_class_control(rhport, driver, p_request);
      } else {
        // Handle STD request to endpoint
        switch ( p_request->bRequest ) {
          case TUSB_REQ_GET_STATUS: {
            uint16_t status = usbd_edpt_stalled(rhport, ep_addr) ? 0x0001 : 0x0000;
            tud_control_xfer(rhport, p_request, &status, 2);
          }
          break;

          case TUSB_REQ_CLEAR_FEATURE:
          case TUSB_REQ_SET_FEATURE: {
            if ( TUSB_REQ_FEATURE_EDPT_HALT == p_request->wValue ) {
              if ( TUSB_REQ_CLEAR_FEATURE ==  p_request->bRequest ) {
                usbd_edpt_clear_stall(rhport, ep_addr);
              }else {
                usbd_edpt_stall(rhport, ep_addr);
              }
            }

            if (driver) {
              // Some classes such as USBTMC needs to clear/re-init its buffer when receiving CLEAR_FEATURE request
              // We will also forward std request targeted endpoint to class drivers as well

              // STD request must always be ACKed regardless of driver returned value
              // Also clear complete callback if driver set since it can also stall the request.
              (void) invoke_class_control(rhport, driver, p_request);
              usbd_control_set_complete_callback(NULL);

              // skip ZLP status if driver already did that
              if ( !_usbd_dev.ep_status[0][TUSB_DIR_IN].busy ) tud_control_status(rhport, p_request);
            }
          }
          break;

          // Unknown/Unsupported request
          default:
            TU_BREAKPOINT();
            return false;
        }
      }
    }
    break;

    // Unknown recipient
    default:
      TU_BREAKPOINT();
      return false;
  }

  return true;
}

// Process Set Configure Request
// This function parse configuration descriptor & open drivers accordingly
static bool process_set_config(uint8_t rhport, uint8_t cfg_num)
{
  // index is cfg_num-1
  tusb_desc_configuration_t const * desc_cfg = (tusb_desc_configuration_t const *) tud_descriptor_configuration_cb(cfg_num-1);
  TU_ASSERT(desc_cfg != NULL && desc_cfg->bDescriptorType == TUSB_DESC_CONFIGURATION);

  // Parse configuration descriptor
  _usbd_dev.remote_wakeup_support = (desc_cfg->bmAttributes & TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP) ? 1u : 0u;
  _usbd_dev.self_powered          = (desc_cfg->bmAttributes & TUSB_DESC_CONFIG_ATT_SELF_POWERED ) ? 1u : 0u;

  // Parse interface descriptor
  uint8_t const * p_desc   = ((uint8_t const*) desc_cfg) + sizeof(tusb_desc_configuration_t);
  uint8_t const * desc_end = ((uint8_t const*) desc_cfg) + tu_le16toh(desc_cfg->wTotalLength);

  while( p_desc < desc_end )
  {
    uint8_t assoc_itf_count = 1;

    // Class will always starts with Interface Association (if any) and then Interface descriptor
    if ( TUSB_DESC_INTERFACE_ASSOCIATION == tu_desc_type(p_desc) )
    {
      tusb_desc_interface_assoc_t const * desc_iad = (tusb_desc_interface_assoc_t const *) p_desc;
      assoc_itf_count = desc_iad->bInterfaceCount;

      p_desc = tu_desc_next(p_desc); // next to Interface

      // IAD's first interface number and class should match with opened interface
      //TU_ASSERT(desc_iad->bFirstInterface == desc_itf->bInterfaceNumber &&
      //          desc_iad->bFunctionClass  == desc_itf->bInterfaceClass);
    }

    TU_ASSERT( TUSB_DESC_INTERFACE == tu_desc_type(p_desc) );
    tusb_desc_interface_t const * desc_itf = (tusb_desc_interface_t const*) p_desc;

    // Find driver for this interface
    uint16_t const remaining_len = (uint16_t) (desc_end-p_desc);
    uint8_t drv_id;
    for (drv_id = 0; drv_id < TOTAL_DRIVER_COUNT; drv_id++)
    {
      usbd_class_driver_t const *driver = get_driver(drv_id);
      TU_ASSERT(driver);
      uint16_t const drv_len = driver->open(rhport, desc_itf, remaining_len);

      if ( (sizeof(tusb_desc_interface_t) <= drv_len)  && (drv_len <= remaining_len) )
      {
        // Open successfully
        TU_LOG_USBD("  %s opened\r\n", driver->name);

        // Some drivers use 2 or more interfaces but may not have IAD e.g MIDI (always) or
        // BTH (even CDC) with class in device descriptor (single interface)
        if ( assoc_itf_count == 1)
        {
          #if CFG_TUD_CDC
          if ( driver->open == cdcd_open ) assoc_itf_count = 2;
          #endif

          #if CFG_TUD_MIDI
          if ( driver->open == midid_open ) assoc_itf_count = 2;
          #endif

          #if CFG_TUD_BTH && CFG_TUD_BTH_ISO_ALT_COUNT
          if ( driver->open == btd_open ) assoc_itf_count = 2;
          #endif
        }

        // bind (associated) interfaces to found driver
        for(uint8_t i=0; i<assoc_itf_count; i++)
        {
          uint8_t const itf_num = desc_itf->bInterfaceNumber+i;

          // Interface number must not be used already
          TU_ASSERT(DRVID_INVALID == _usbd_dev.itf2drv[itf_num]);
          _usbd_dev.itf2drv[itf_num] = drv_id;
        }

        // bind all endpoints to found driver
        tu_edpt_bind_driver(_usbd_dev.ep2drv, desc_itf, drv_len, drv_id);

        // next Interface
        p_desc += drv_len;

        break; // exit driver find loop
      }
    }

    // Failed if there is no supported drivers
    TU_ASSERT(drv_id < TOTAL_DRIVER_COUNT);
  }

  return true;
}

// return descriptor's buffer and update desc_len
static bool process_get_descriptor(uint8_t rhport, tusb_control_request_t const * p_request)
{
  tusb_desc_type_t const desc_type = (tusb_desc_type_t) tu_u16_high(p_request->wValue);
  uint8_t const desc_index = tu_u16_low( p_request->wValue );

  switch(desc_type)
  {
    case TUSB_DESC_DEVICE: {
      TU_LOG_USBD(" Device\r\n");

      void* desc_device = (void*) (uintptr_t) tud_descriptor_device_cb();
      TU_ASSERT(desc_device);

      // Only response with exactly 1 Packet if: not addressed and host requested more data than device descriptor has.
      // This only happens with the very first get device descriptor and EP0 size = 8 or 16.
      if ((CFG_TUD_ENDPOINT0_SIZE < sizeof(tusb_desc_device_t)) && !_usbd_dev.addressed &&
          ((tusb_control_request_t const*) p_request)->wLength > sizeof(tusb_desc_device_t)) {
        // Hack here: we modify the request length to prevent usbd_control response with zlp
        // since we are responding with 1 packet & less data than wLength.
        tusb_control_request_t mod_request = *p_request;
        mod_request.wLength = CFG_TUD_ENDPOINT0_SIZE;

        return tud_control_xfer(rhport, &mod_request, desc_device, CFG_TUD_ENDPOINT0_SIZE);
      }else {
        return tud_control_xfer(rhport, p_request, desc_device, sizeof(tusb_desc_device_t));
      }
    }
    // break; // unreachable

    case TUSB_DESC_BOS: {
      TU_LOG_USBD(" BOS\r\n");

      // requested by host if USB > 2.0 ( i.e 2.1 or 3.x )
      uintptr_t desc_bos = (uintptr_t) tud_descriptor_bos_cb();
      TU_VERIFY(desc_bos);

      // Use offsetof to avoid pointer to the odd/misaligned address
      uint16_t const total_len = tu_le16toh( tu_unaligned_read16((const void*) (desc_bos + offsetof(tusb_desc_bos_t, wTotalLength))) );

      return tud_control_xfer(rhport, p_request, (void*) desc_bos, total_len);
    }
    // break; // unreachable

    case TUSB_DESC_CONFIGURATION:
    case TUSB_DESC_OTHER_SPEED_CONFIG: {
      uintptr_t desc_config;

      if ( desc_type == TUSB_DESC_CONFIGURATION ) {
        TU_LOG_USBD(" Configuration[%u]\r\n", desc_index);
        desc_config = (uintptr_t) tud_descriptor_configuration_cb(desc_index);
        TU_ASSERT(desc_config);
      }else {
        // Host only request this after getting Device Qualifier descriptor
        TU_LOG_USBD(" Other Speed Configuration\r\n");
        desc_config = (uintptr_t) tud_descriptor_other_speed_configuration_cb(desc_index);
        TU_VERIFY(desc_config);
      }

      // Use offsetof to avoid pointer to the odd/misaligned address
      uint16_t const total_len = tu_le16toh( tu_unaligned_read16((const void*) (desc_config + offsetof(tusb_desc_configuration_t, wTotalLength))) );

      return tud_control_xfer(rhport, p_request, (void*) desc_config, total_len);
    }
    // break; // unreachable

    case TUSB_DESC_STRING:
    {
      TU_LOG_USBD(" String[%u]\r\n", desc_index);

      // String Descriptor always uses the desc set from user
      uint8_t const* desc_str = (uint8_t const*) tud_descriptor_string_cb(desc_index, tu_le16toh(p_request->wIndex));
      TU_VERIFY(desc_str);

      // first byte of descriptor is its size
      return tud_control_xfer(rhport, p_request, (void*) (uintptr_t) desc_str, tu_desc_len(desc_str));
    }
    // break; // unreachable

    case TUSB_DESC_DEVICE_QUALIFIER: {
      TU_LOG_USBD(" Device Qualifier\r\n");
      uint8_t const* desc_qualifier = tud_descriptor_device_qualifier_cb();
      TU_VERIFY(desc_qualifier);
      return tud_control_xfer(rhport, p_request, (void*) (uintptr_t) desc_qualifier, tu_desc_len(desc_qualifier));
    }
    // break; // unreachable

    default: return false;
  }
}

//--------------------------------------------------------------------+
// DCD Event Handler
//--------------------------------------------------------------------+
TU_ATTR_FAST_FUNC void dcd_event_handler(dcd_event_t const* event, bool in_isr) {
  bool send = false;
  switch (event->event_id) {
    case DCD_EVENT_UNPLUGGED:
      _usbd_dev.connected = 0;
      _usbd_dev.addressed = 0;
      _usbd_dev.cfg_num = 0;
      _usbd_dev.suspended = 0;
      send = true;
      break;

    case DCD_EVENT_SUSPEND:
      // NOTE: When plugging/unplugging device, the D+/D- state are unstable and
      // can accidentally meet the SUSPEND condition ( Bus Idle for 3ms ).
      // In addition, some MCUs such as SAMD or boards that haven no VBUS detection cannot distinguish
      // suspended vs disconnected. We will skip handling SUSPEND/RESUME event if not currently connected
      if (_usbd_dev.connected) {
        _usbd_dev.suspended = 1;
        send = true;
      }
      break;

    case DCD_EVENT_RESUME:
      // skip event if not connected (especially required for SAMD)
      if (_usbd_dev.connected) {
        _usbd_dev.suspended = 0;
        send = true;
      }
      break;

    case DCD_EVENT_SOF:
      // SOF driver handler in ISR context
      for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++) {
        usbd_class_driver_t const* driver = get_driver(i);
        if (driver && driver->sof) {
          driver->sof(event->rhport, event->sof.frame_count);
        }
      }

      // Some MCUs after running dcd_remote_wakeup() does not have way to detect the end of remote wakeup
      // which last 1-15 ms. DCD can use SOF as a clear indicator that bus is back to operational
      if (_usbd_dev.suspended) {
        _usbd_dev.suspended = 0;

        dcd_event_t const event_resume = {.rhport = event->rhport, .event_id = DCD_EVENT_RESUME};
        queue_event(&event_resume, in_isr);
      }

      if (tu_bit_test(_usbd_dev.sof_consumer, SOF_CONSUMER_USER)) {
        dcd_event_t const event_sof = {.rhport = event->rhport, .event_id = DCD_EVENT_SOF, .sof.frame_count = event->sof.frame_count};
        queue_event(&event_sof, in_isr);
      }
      break;

    case DCD_EVENT_SETUP_RECEIVED:
      _usbd_queued_setup++;
      send = true;
      break;

    default:
      send = true;
      break;
  }

  if (send) {
    queue_event(event, in_isr);
  }
}

//--------------------------------------------------------------------+
// USBD API For Class Driver
//--------------------------------------------------------------------+

void usbd_int_set(bool enabled)
{
  if (enabled)
  {
    dcd_int_enable(_usbd_rhport);
  }else
  {
    dcd_int_disable(_usbd_rhport);
  }
}

// Parse consecutive endpoint descriptors (IN & OUT)
bool usbd_open_edpt_pair(uint8_t rhport, uint8_t const* p_desc, uint8_t ep_count, uint8_t xfer_type, uint8_t* ep_out, uint8_t* ep_in)
{
  for(int i=0; i<ep_count; i++)
  {
    tusb_desc_endpoint_t const * desc_ep = (tusb_desc_endpoint_t const *) p_desc;

    TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType && xfer_type == desc_ep->bmAttributes.xfer);
    TU_ASSERT(usbd_edpt_open(rhport, desc_ep));

    if ( tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN )
    {
      (*ep_in) = desc_ep->bEndpointAddress;
    }else
    {
      (*ep_out) = desc_ep->bEndpointAddress;
    }

    p_desc = tu_desc_next(p_desc);
  }

  return true;
}

// Helper to defer an isr function
void usbd_defer_func(osal_task_func_t func, void* param, bool in_isr) {
  dcd_event_t event = {
      .rhport   = 0,
      .event_id = USBD_EVENT_FUNC_CALL,
  };
  event.func_call.func  = func;
  event.func_call.param = param;

  queue_event(&event, in_isr);
}

//--------------------------------------------------------------------+
// USBD Endpoint API
//--------------------------------------------------------------------+

bool usbd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const* desc_ep) {
  rhport = _usbd_rhport;

  TU_ASSERT(tu_edpt_number(desc_ep->bEndpointAddress) < CFG_TUD_ENDPPOINT_MAX);
  TU_ASSERT(tu_edpt_validate(desc_ep, (tusb_speed_t) _usbd_dev.speed));

  return dcd_edpt_open(rhport, desc_ep);
}

bool usbd_edpt_claim(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  // TODO add this check later, also make sure we don't starve an out endpoint while suspending
  // TU_VERIFY(tud_ready());

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);
  tu_edpt_state_t* ep_state = &_usbd_dev.ep_status[epnum][dir];

  return tu_edpt_claim(ep_state, _usbd_mutex);
}

bool usbd_edpt_release(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);
  tu_edpt_state_t* ep_state = &_usbd_dev.ep_status[epnum][dir];

  return tu_edpt_release(ep_state, _usbd_mutex);
}

bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
  rhport = _usbd_rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  // TODO skip ready() check for now since enumeration also use this API
  // TU_VERIFY(tud_ready());

  TU_LOG_USBD("  Queue EP %02X with %u bytes ...\r\n", ep_addr, total_bytes);
#if CFG_TUD_LOG_LEVEL >= 3
  if(dir == TUSB_DIR_IN) {
    TU_LOG_MEM(CFG_TUD_LOG_LEVEL, buffer, total_bytes, 2);
  }
#endif

  // Attempt to transfer on a busy endpoint, sound like an race condition !
  TU_ASSERT(_usbd_dev.ep_status[epnum][dir].busy == 0);

  // Set busy first since the actual transfer can be complete before dcd_edpt_xfer()
  // could return and USBD task can preempt and clear the busy
  _usbd_dev.ep_status[epnum][dir].busy = 1;

  if (dcd_edpt_xfer(rhport, ep_addr, buffer, total_bytes)) {
    return true;
  } else {
    // DCD error, mark endpoint as ready to allow next transfer
    _usbd_dev.ep_status[epnum][dir].busy = 0;
    _usbd_dev.ep_status[epnum][dir].claimed = 0;
    TU_LOG_USBD("FAILED\r\n");
    TU_BREAKPOINT();
    return false;
  }
}

// The number of bytes has to be given explicitly to allow more flexible control of how many
// bytes should be written and second to keep the return value free to give back a boolean
// success message. If total_bytes is too big, the FIFO will copy only what is available
// into the USB buffer!
bool usbd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t* ff, uint16_t total_bytes) {
  rhport = _usbd_rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  TU_LOG_USBD("  Queue ISO EP %02X with %u bytes ... ", ep_addr, total_bytes);

  // Attempt to transfer on a busy endpoint, sound like an race condition !
  TU_ASSERT(_usbd_dev.ep_status[epnum][dir].busy == 0);

  // Set busy first since the actual transfer can be complete before dcd_edpt_xfer() could return
  // and usbd task can preempt and clear the busy
  _usbd_dev.ep_status[epnum][dir].busy = 1;

  if (dcd_edpt_xfer_fifo(rhport, ep_addr, ff, total_bytes)) {
    TU_LOG_USBD("OK\r\n");
    return true;
  } else {
    // DCD error, mark endpoint as ready to allow next transfer
    _usbd_dev.ep_status[epnum][dir].busy = 0;
    _usbd_dev.ep_status[epnum][dir].claimed = 0;
    TU_LOG_USBD("failed\r\n");
    TU_BREAKPOINT();
    return false;
  }
}

bool usbd_edpt_busy(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  return _usbd_dev.ep_status[epnum][dir].busy;
}

void usbd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  rhport = _usbd_rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  // only stalled if currently cleared
  TU_LOG_USBD("    Stall EP %02X\r\n", ep_addr);
  dcd_edpt_stall(rhport, ep_addr);
  _usbd_dev.ep_status[epnum][dir].stalled = 1;
  _usbd_dev.ep_status[epnum][dir].busy = 1;
}

void usbd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
  rhport = _usbd_rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  // only clear if currently stalled
  TU_LOG_USBD("    Clear Stall EP %02X\r\n", ep_addr);
  dcd_edpt_clear_stall(rhport, ep_addr);
  _usbd_dev.ep_status[epnum][dir].stalled = 0;
  _usbd_dev.ep_status[epnum][dir].busy = 0;
}

bool usbd_edpt_stalled(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  return _usbd_dev.ep_status[epnum][dir].stalled;
}

/**
 * usbd_edpt_close will disable an endpoint.
 * In progress transfers on this EP may be delivered after this call.
 */
void usbd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
  (void) rhport; (void) ep_addr;
  // ISO alloc/activate Should be used instead
#else
  rhport = _usbd_rhport;

  TU_LOG_USBD("  CLOSING Endpoint: 0x%02X\r\n", ep_addr);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  dcd_edpt_close(rhport, ep_addr);
  _usbd_dev.ep_status[epnum][dir].stalled = 0;
  _usbd_dev.ep_status[epnum][dir].busy = 0;
  _usbd_dev.ep_status[epnum][dir].claimed = 0;
#endif

  return;
}

void usbd_sof_enable(uint8_t rhport, sof_consumer_t consumer, bool en) {
  rhport = _usbd_rhport;

  uint8_t consumer_old = _usbd_dev.sof_consumer;
  // Keep track how many class instances need the SOF interrupt
  if (en) {
    _usbd_dev.sof_consumer |= (uint8_t)(1 << consumer);
  } else {
    _usbd_dev.sof_consumer &= (uint8_t)(~(1 << consumer));
  }

  // Test logically unequal
  if(!_usbd_dev.sof_consumer != !consumer_old) {
    dcd_sof_enable(rhport, _usbd_dev.sof_consumer);
  }
}

bool usbd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
  rhport = _usbd_rhport;

  TU_ASSERT(tu_edpt_number(ep_addr) < CFG_TUD_ENDPPOINT_MAX);
  return dcd_edpt_iso_alloc(rhport, ep_addr, largest_packet_size);
#else
  (void) rhport; (void) ep_addr; (void) largest_packet_size;
  return false;
#endif
}

bool usbd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const* desc_ep) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
  rhport = _usbd_rhport;

  uint8_t const epnum = tu_edpt_number(desc_ep->bEndpointAddress);
  uint8_t const dir = tu_edpt_dir(desc_ep->bEndpointAddress);

  TU_ASSERT(epnum < CFG_TUD_ENDPPOINT_MAX);
  TU_ASSERT(tu_edpt_validate(desc_ep, (tusb_speed_t) _usbd_dev.speed));

  _usbd_dev.ep_status[epnum][dir].stalled = 0;
  _usbd_dev.ep_status[epnum][dir].busy = 0;
  _usbd_dev.ep_status[epnum][dir].claimed = 0;
  return dcd_edpt_iso_activate(rhport, desc_ep);
#else
  (void) rhport; (void) desc_ep;
  return false;
#endif
}

#endif
