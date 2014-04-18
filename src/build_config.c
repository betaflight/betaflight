#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdarg.h"

#include "platform.h"

#include "build_config.h"
#include "common/printf.h"

#include "drivers/gpio_common.h"
#include "drivers/timer_common.h"
#include "drivers/pwm_common.h"
#include "flight_mixer.h"
#include "sensors_common.h"
#include "battery.h"
#include "boardalignment.h"
#include "config.h"

#include "drivers/serial_common.h"
#include "runtime_config.h"


#if MAX_MOTORS != MAX_SUPPORTED_MOTORS
#error Motor configuration mismatch
#endif

#if MAX_SERVOS != MAX_SUPPORTED_SERVOS
#error Servo configuration mismatch
#endif


#ifdef REQUIRE_CC_ARM_PRINTF_SUPPORT

// gcc/GNU version
static void _putc(void *p, char c)
{
    serialWrite(core.mainport, c);
}

void initPrintfSupport(void)
{
    init_printf(NULL, _putc);
}

#else

// keil/armcc version
int fputc(int c, FILE *f)
{
    // let DMA catch up a bit when using set or dump, we're too fast.
    while (!isSerialTransmitBufferEmpty(core.mainport));
    serialWrite(core.mainport, c);
    return c;
}

void initPrintfSupport(void)
{
    // nothing to do
}
#endif
