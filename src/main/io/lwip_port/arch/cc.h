/*
 * lwip compiler/architecture port for betaflight (gcc, arm cortex-m7, little-endian, no_sys).
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>

#define LWIP_NO_UNISTD_H        1
#define LWIP_TIMEVAL_PRIVATE    0

/* lwip diagnostics/asserts become no-ops, no console in phone-config mode */
#define LWIP_PLATFORM_DIAG(x)
#define LWIP_PLATFORM_ASSERT(x) do { } while (0)
