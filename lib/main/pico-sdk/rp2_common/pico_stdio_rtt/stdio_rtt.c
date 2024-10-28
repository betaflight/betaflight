/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/binary_info.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_rtt.h"
#include "SEGGER_RTT.h"

#if PICO_NO_BI_STDIO_RTT
#define stdio_bi_decl_if_func_used(x)
#else
#define stdio_bi_decl_if_func_used bi_decl_if_func_used
#endif

void stdio_rtt_init(void) {
    SEGGER_RTT_Init();
    stdio_set_driver_enabled(&stdio_rtt, true);
    stdio_bi_decl_if_func_used(bi_program_feature("RTT stdin / stdout"));
}

void stdio_rtt_deinit(void) {
    stdio_set_driver_enabled(&stdio_rtt, false);
}

static void stdio_rtt_out_chars(const char *buf, int length) {
    SEGGER_RTT_Write(0, buf, length);
}

static int stdio_rtt_in_chars(char *buf, int length) {
    return SEGGER_RTT_Read(0, buf, length);
}

stdio_driver_t stdio_rtt = {
    .out_chars = stdio_rtt_out_chars,
    .in_chars = stdio_rtt_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_RTT_DEFAULT_CRLF
#endif
};
