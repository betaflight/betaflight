#pragma once

extern void picotrace_prefix(void);

#define tprintf(fmt,...) do {                        \
        picotrace_prefix();                          \
        stdio_printf(fmt, ## __VA_ARGS__);           \
        stdio_printf("\n");                          \
    } while (0)

