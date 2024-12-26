

#pragma once

#include "pico/stdio_usb.h"

#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE)
#define CFG_TUD_CDC             (1)
#define CFG_TUD_CDC_RX_BUFSIZE  (256)
#define CFG_TUD_CDC_TX_BUFSIZE  (256)
