#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

#if defined(USE_VCP) && defined(USE_USB_CDC_DEBUG)

#include "usb_cdc_debug.h"

#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#include "vcp_hal/usbd_cdc_interface.h"
#elif defined(STM32F4)
#include "usbd_cdc_vcp.h"
#endif
#include "drivers/usb_io.h"

#define USB_CDC_DEBUG_BUFFER_SIZE 256

static char usbCdcDebugBuffer[USB_CDC_DEBUG_BUFFER_SIZE];

int usbCdcPrintf(const char* format, ...)
{
    if (usbIsConnected() && usbIsConfigured()) {
        va_list args;
        va_start(args, format);
        int len = vsnprintf(usbCdcDebugBuffer, USB_CDC_DEBUG_BUFFER_SIZE, format, args);
        va_end(args);

        if (len > 0 && len < USB_CDC_DEBUG_BUFFER_SIZE) {
            CDC_Send_DATA((uint8_t*)usbCdcDebugBuffer, len);
            return len;
        }
    }
    return -1;
}

#endif
