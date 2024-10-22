/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_STDIO_RTT_H
#define _PICO_STDIO_RTT_H

#include "pico/stdio.h"

/** \brief Support for stdin/stdout using SEGGER RTT
 *  \defgroup pico_stdio_rtt pico_stdio_rtt
 *  \ingroup pico_stdio
 *
 *  Linking this library or calling `pico_enable_stdio_rtt(TARGET)` in the CMake (which
 *  achieves the same thing) will add RTT to the drivers used for standard output
 */

// PICO_CONFIG: PICO_STDIO_RTT_DEFAULT_CRLF, Default state of CR/LF translation for rtt output, type=bool, default=PICO_STDIO_DEFAULT_CRLF, group=pico_stdio_rtt
#ifndef PICO_STDIO_RTT_DEFAULT_CRLF
#define PICO_STDIO_RTT_DEFAULT_CRLF PICO_STDIO_DEFAULT_CRLF
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern stdio_driver_t stdio_rtt;

/*! \brief Explicitly initialize stdin/stdout over RTT and add it to the current set of stdin/stdout drivers
 *  \ingroup pico_stdio_rtt
 *
 * \note this method is automatically called by \ref stdio_init_all() if `pico_stdio_rtt` is included in the build
 */
void stdio_rtt_init(void);

/*! \brief Explicitly deinitialize stdin/stdout over RTT and remove it from the current set of stdin/stdout drivers
 *  \ingroup pico_stdio_rtt
 *
 * \note this method is automatically called by \ref stdio_deinit_all() if `pico_stdio_rtt` is included in the build
 */
void stdio_rtt_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
