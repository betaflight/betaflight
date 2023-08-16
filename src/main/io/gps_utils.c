#include <stdarg.h>

#include "common/printf.h"
#include "io/serial.h"

struct nmea_putp {
    serialPort_t* port;
    uint8_t csum;
};

static void nmea_putcf_csum(void* putp, char c)
{
    struct nmea_putp* p = putp;
    serialWrite(p->port, c);
    p->csum ^= c;
}

static void nmea_putcf(void* putp, char c)
{
    struct nmea_putp* p = putp;
    serialWrite(p->port, c);
}

static void nmeaPrintfva_raw(struct nmea_putp* putp, const char *format, va_list va)
{
    tfp_format(putp, nmea_putcf, format, va);
}

static void nmeaPrintf_raw(struct nmea_putp* putp, const char *format, ...)
{
    va_list va;
    va_start(va, format);
    nmeaPrintfva_raw(putp, format, va);
    va_end(va);
}

static void nmeaPrintfva(serialPort_t *port, const char *format, va_list va)
{
    struct nmea_putp putp = {
        .port = port,
        .csum = '$'             // don't checkfum '$' in header
    };
    tfp_format(&putp, nmea_putcf_csum, format, va);
    nmeaPrintf_raw(&putp, "*%02X\r\n", putp.csum);
}

// printf format (+ arguments), append "*<csum>\r\n"
// nmeaPrintf(port, "$PUBX,41,1,0003,0001,%d,0", 115200);
void nmeaPrintf(serialPort_t *port, const char *format, ...)
{
    va_list va;
    va_start(va, format);
    nmeaPrintfva(port, format, va);
    va_end(va);
}

