#pragma once

#include <stdarg.h>

#if defined(USE_VCP) && defined(USE_USB_CDC_DEBUG)

// Printf-style debug output over USB CDC (bypasses MSP)
// Returns number of characters transmitted, or negative on error
int usbCdcPrintf(const char* format, ...);

#else

// No-op when debug disabled or no VCP hardware
#define usbCdcPrintf(...) 0

#endif