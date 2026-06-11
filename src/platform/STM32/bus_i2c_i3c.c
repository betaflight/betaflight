/*
 * STM32C5 I3C1 controller used as legacy-I2C master, exported through BF's
 * generic bus_i2c.h interface so the baro / mag / OSD I2C drivers can drive
 * a plain I2C slave hanging off the I3C1 pins (PC10 = SCL, PC11 = SDA, AF4).
 *
 * The C5 has no I2C3 peripheral on PC10/PC11 -- those pads are I3C1-only
 * (datasheet AF table). Boards that need I2C on these pads run the I3C1
 * controller in MTYPE_LEGACY_I2C mode with the arbitration header (7'h7E)
 * suppressed. Result is bit-for-bit identical to a real I2C master from
 * the slave's perspective.
 *
 * Activated by USE_I3C_AS_I2C in the board config. Mutually exclusive with
 * the LL hardware I2C driver (bus_i2c_ll*) because both would otherwise
 * define i2cInit / i2cRead / etc.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && defined(USE_I3C_AS_I2C)

#include "stm32c5xx.h"
#include "stm32c5xx_ll_bus.h"
#include "stm32c5xx_ll_gpio.h"
#include "stm32c5xx_ll_i3c.h"
#include "stm32c5xx_ll_rcc.h"

#include "build/build_config.h"
#include "drivers/bus_i2c.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#if !defined(I3C_AS_I2C_SCL_PIN) || !defined(I3C_AS_I2C_SDA_PIN)
#error "USE_I3C_AS_I2C requires I3C_AS_I2C_SCL_PIN and I3C_AS_I2C_SDA_PIN in the board config"
#endif

#define I3C_AS_I2C_GUARD_LOOPS 100000U

static volatile uint16_t i2cErrorCount = 0;
static bool i2cInitialised = false;

// Debug instrumentation for bring-up. Last failure point + last SER/EVR
// snapshot. Each error path sets `i2cI3cLastFail` to a unique step code so
// SWD inspection tells which i2cRead phase failed (TXFNF / FC / RXFNE).
volatile uint16_t i2cI3cLastFail __attribute__((used));
volatile uint32_t i2cI3cLastEvr  __attribute__((used));
volatile uint32_t i2cI3cLastSer  __attribute__((used));
volatile uint32_t i2cI3cReadCalls __attribute__((used));
volatile uint32_t i2cI3cInitCalls __attribute__((used));
volatile uint32_t i2cI3cWriteCalls __attribute__((used));

static void i3cWaitOrError(uint32_t *guard)
{
    if (*guard == 0) {
        return;
    }
    --(*guard);
}

static void i3cClearAllEventFlags(void)
{
    I3C1->CEVR = I3C1->EVR;
}

void i2cInit(i2cDevice_e device)
{
    UNUSED(device);
    i2cI3cInitCalls++;

    if (i2cInitialised) {
        return;
    }

    // RCC: clock the peripheral. Kernel clock stays on PCLK1 (reset default;
    // HSIK has caused silent wedges on other C5 peripherals so we keep PCLK1).
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I3C1);
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I3C1);
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I3C1);

    // SCL/SDA pins come from I3C_AS_I2C_SCL_PIN / I3C_AS_I2C_SDA_PIN. AF4 is
    // I3C1 on the STM32C5 AF table. Open-drain, HIGH slew rate; external
    // pull-ups required (BF policy for I2C buses). Claim ownership via
    // IOInit so unusedPinsInit doesn't revert the pads.
    const IO_t scl = IOGetByTag(IO_TAG(I3C_AS_I2C_SCL_PIN));
    const IO_t sda = IOGetByTag(IO_TAG(I3C_AS_I2C_SDA_PIN));
    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(0));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(0));

    const ioConfig_t af_od_high = IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL);
    IOConfigGPIOAF(scl, af_od_high, LL_GPIO_AF_4);
    IOConfigGPIOAF(sda, af_od_high, LL_GPIO_AF_4);

    // Controller mode, arbitration header suppressed (we're talking to plain
    // I2C slaves that don't speak I3C's broadcast 7'h7E).
    LL_I3C_SetMode(I3C1, LL_I3C_MODE_CONTROLLER);
    LL_I3C_DisableArbitrationHeader(I3C1);

    // TIMINGR0: [31:24]=SCLH_I2C, [23:16]=SCLL_OD, [15:8]=SCLH_I3C, [7:0]=SCLL_PP.
    // Leaving SCLH_I2C at 0 (which was the first-cut bug) means SCL never
    // rises during legacy-I2C messages -- slaves see no clock pulses and
    // NACK everything. 0xF0/0xF0 at 144 MHz PCLK1 = ~300 kHz I2C with
    // comfortable rise/fall margin; well below the chip's max (3.4 MHz for
    // SPA06). SCLL_PP / SCLH_I3C must be > 0 even though we don't use the
    // I3C push-pull phase.
    LL_I3C_ConfigClockWaveForm(I3C1, 0xF0F00505U);

    // Bus characteristic (TIMINGR1): SDA hold + free / available timing.
    // Copied from the CubeMX-generated reference init for I3C1 on this
    // family; the chip ACKs across the full byte sweep so this is in spec.
    LL_I3C_SetBusCharacteristic(I3C1, 0x1D008EU);

    // RX threshold must be 1_8 (byte granularity). The 1_2 setting is one WORD
    // in a 2-word FIFO, so RXFNEF only asserts once a full 4-byte word is
    // present -- a trailing partial word (1-3 bytes) of a read never raises the
    // flag and those bytes can't be drained. Byte granularity makes every
    // received byte readable.
    LL_I3C_ConfigCtrlFifo(I3C1,
                          LL_I3C_RXFIFO_THRESHOLD_1_8,
                          LL_I3C_TXFIFO_THRESHOLD_1_2,
                          LL_I3C_CTRL_FIFO_CONTROL_ONLY);

    LL_I3C_Enable(I3C1);

    i2cInitialised = true;
}

void i2cPinConfigure(const struct i2cConfig_s *i2cConfig)
{
    UNUSED(i2cConfig);
    // Pin assignment is fixed at compile time (I3C_AS_I2C_SCL_PIN /
    // I3C_AS_I2C_SDA_PIN) -- there are no alternate pad options for I3C1 on
    // C5.
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    UNUSED(device);
    if (error) {
        *error = false;
    }
    return false;       // synchronous driver; never busy on return
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

// Issue a legacy-I2C message via the I3C controller. Direction = READ or
// WRITE. end_mode = STOP or RESTART. Returns true on completion (FC flag),
// false on any error (ERR flag) -- caller bumps i2cErrorCount.
static bool i3cI2cMessage(uint8_t addr, uint32_t direction, uint32_t end_mode, uint32_t count)
{
    LL_I3C_ControllerHandleMessage(I3C1,
                                   addr,
                                   count,
                                   direction,
                                   LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C,
                                   end_mode);
    return true;
}

bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    UNUSED(device);
    i2cI3cWriteCalls++;
    if (!i2cInitialised) {
        return false;
    }

    // Single I2C frame: reg byte + data bytes, then STOP.
    i3cClearAllEventFlags();
    i3cI2cMessage(addr, LL_I3C_DIRECTION_WRITE, LL_I3C_GENERATE_STOP, (uint32_t)len + 1);

    uint32_t guard = I3C_AS_I2C_GUARD_LOOPS;
    while (!LL_I3C_IsActiveFlag_TXFNF(I3C1) && !LL_I3C_IsActiveFlag_ERR(I3C1) && guard) { i3cWaitOrError(&guard); }
    if (!guard || LL_I3C_IsActiveFlag_ERR(I3C1)) { i2cErrorCount++; i3cClearAllEventFlags(); return false; }
    LL_I3C_TransmitData8(I3C1, reg);

    for (uint8_t i = 0; i < len; i++) {
        guard = I3C_AS_I2C_GUARD_LOOPS;
        while (!LL_I3C_IsActiveFlag_TXFNF(I3C1) && !LL_I3C_IsActiveFlag_ERR(I3C1) && guard) { i3cWaitOrError(&guard); }
        if (!guard || LL_I3C_IsActiveFlag_ERR(I3C1)) { i2cErrorCount++; i3cClearAllEventFlags(); return false; }
        LL_I3C_TransmitData8(I3C1, data[i]);
    }

    guard = I3C_AS_I2C_GUARD_LOOPS;
    while (!LL_I3C_IsActiveFlag_FC(I3C1) && !LL_I3C_IsActiveFlag_ERR(I3C1) && guard) { i3cWaitOrError(&guard); }
    const bool ok = guard && !LL_I3C_IsActiveFlag_ERR(I3C1);
    if (!ok) {
        i2cErrorCount++;
    }
    i3cClearAllEventFlags();
    return ok;
}

bool i2cWrite(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t data)
{
    return i2cWriteBuffer(device, addr, reg, 1, &data);
}

// Mirror the bring-up probe's transaction pattern exactly. The probe code
// reads SPA06 CHIP_ID cleanly with this sequence; only difference vs the
// earlier driver version was that the driver called i3cClearAllEventFlags()
// BEFORE writing each CR. ANACK / COVR landed on the address byte despite
// CR / TIMINGR being identical to the probe's. Dropping the pre-clear
// matches the probe and is fine on this peripheral because the flags get
// cleared at the end of each transaction anyway.
bool i2cReadBuffer(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    UNUSED(device);
    i2cI3cReadCalls++;
    if (!i2cInitialised || len == 0) {
        i2cI3cLastFail = 0xAA00;
        return false;
    }

    // Phase A: write the register pointer, end with RESTART so the bus is
    // held for the read that follows.
    LL_I3C_ControllerHandleMessage(I3C1, addr, 1,
                                   LL_I3C_DIRECTION_WRITE,
                                   LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C,
                                   LL_I3C_GENERATE_RESTART);
    uint32_t guard = I3C_AS_I2C_GUARD_LOOPS;
    while (!LL_I3C_IsActiveFlag_TXFNF(I3C1) && !LL_I3C_IsActiveFlag_ERR(I3C1) && guard) { i3cWaitOrError(&guard); }
    if (!guard || LL_I3C_IsActiveFlag_ERR(I3C1)) {
        i2cI3cLastEvr = I3C1->EVR; i2cI3cLastSer = I3C1->SER;
        i2cI3cLastFail = 0xA1E0 | (guard ? 0 : 1);
        i2cErrorCount++; i3cClearAllEventFlags(); return false;
    }
    LL_I3C_TransmitData8(I3C1, reg);

    // FC marks the closing STOP; this RESTART message issues no STOP (FC fires
    // after the Phase B read), and FC is unreliable in legacy-I2C controller
    // mode regardless -- so a guard timeout here is the expected case. Only a
    // hard ERR is a real failure.
    guard = I3C_AS_I2C_GUARD_LOOPS;
    while (!LL_I3C_IsActiveFlag_FC(I3C1) && !LL_I3C_IsActiveFlag_ERR(I3C1) && guard) { i3cWaitOrError(&guard); }
    if (LL_I3C_IsActiveFlag_ERR(I3C1)) {
        i2cI3cLastEvr = I3C1->EVR; i2cI3cLastSer = I3C1->SER;
        i2cI3cLastFail = 0xA2E0;
        i2cErrorCount++; i3cClearAllEventFlags(); return false;
    }
    i3cClearAllEventFlags();

    // Phase B: read `len` bytes, STOP at end.
    LL_I3C_ControllerHandleMessage(I3C1, addr, len,
                                   LL_I3C_DIRECTION_READ,
                                   LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C,
                                   LL_I3C_GENERATE_STOP);

    for (uint8_t i = 0; i < len; i++) {
        guard = I3C_AS_I2C_GUARD_LOOPS;
        while (!LL_I3C_IsActiveFlag_RXFNE(I3C1) && !LL_I3C_IsActiveFlag_ERR(I3C1) && guard) { i3cWaitOrError(&guard); }
        if (!guard || LL_I3C_IsActiveFlag_ERR(I3C1)) {
            i2cI3cLastEvr = I3C1->EVR; i2cI3cLastSer = I3C1->SER;
            i2cI3cLastFail = 0xA3E0 | i;
            i2cErrorCount++; i3cClearAllEventFlags(); return false;
        }
        buf[i] = LL_I3C_ReceiveData8(I3C1);
    }

    // All payload bytes are already latched above. The trailing FC/STOP is bus
    // housekeeping that does not reliably assert in legacy-I2C mode, so a
    // timeout is benign here -- only a hard ERR is a real failure.
    guard = I3C_AS_I2C_GUARD_LOOPS;
    while (!LL_I3C_IsActiveFlag_FC(I3C1) && !LL_I3C_IsActiveFlag_ERR(I3C1) && guard) { i3cWaitOrError(&guard); }
    if (LL_I3C_IsActiveFlag_ERR(I3C1)) {
        i2cI3cLastEvr = I3C1->EVR; i2cI3cLastSer = I3C1->SER;
        i2cI3cLastFail = 0xA4E0;
        i2cErrorCount++; i3cClearAllEventFlags(); return false;
    }
    i2cI3cLastFail = 0x0000;
    i3cClearAllEventFlags();
    return true;
}

bool i2cRead(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return i2cReadBuffer(device, addr, reg, len, buf);
}

#endif // USE_I2C && USE_I3C_AS_I2C
