#include "pico/stdio.h"
#include "pico/platform/compiler.h"

#define tprintf(fmt,...) do {stdio_printf(fmt, ## __VA_ARGS__); stdio_printf("\n"); } while (0)

