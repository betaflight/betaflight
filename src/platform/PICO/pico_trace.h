#pragma once

extern void picotrace_prefix(void);

#if 0

// with depth prefix
#define tprintf(fmt,...) do {                        \
        picotrace_prefix();                          \
        stdio_printf(fmt, ## __VA_ARGS__);           \
        stdio_printf("\n");                          \
    } while (0)

#else

#define tprintf(fmt,...) do {                        \
        stdio_printf(fmt, ## __VA_ARGS__);           \
        stdio_printf("\n");                          \
    } while (0)

#endif
