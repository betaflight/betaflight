/*
 * SPA06-003 baro probe over STM32C5 I3C1 in legacy-I2C mode.
 *
 * First-cut, blocking-only probe. Brings up I3C1 on PC10/PC11 (AF4) as a
 * legacy-I2C controller, runs a 1-byte register read at I2C address 0x77,
 * and stashes the result in two SWD-readable globals so the bring-up loop
 * can see the chip's CHIP_ID (reg 0x0D, expected 0x11 for SPA06/SPL07-003).
 *
 * Wire-up:
 *   - PC10  ->  I3C1_SCL  (AF4)
 *   - PC11  ->  I3C1_SDA  (AF4)
 *   - external pull-ups to VDDIO (board)
 *
 * Built only when ENABLE_BARO_SPA06_PROBE is set on the platform; gate from
 * the board config.h.
 */

#include "platform.h"

#if defined(STM32C5) && ENABLE_BARO_SPA06_PROBE

#include "stm32c5xx.h"
#include "stm32c5xx_ll_bus.h"
#include "stm32c5xx_ll_gpio.h"
#include "stm32c5xx_ll_i3c.h"
#include "stm32c5xx_ll_rcc.h"

#include "drivers/io.h"
#include "drivers/time.h"

#define SPA06_I2C_ADDR        0x77   // SDO tied high; 0x76 if SDO low
#define SPA06_REG_CHIP_ID     0x0D
#define SPA06_CHIP_ID_VAL     0x11   // also matches SPL07_003 in BF DPS310 driver

volatile uint32_t spa06ProbeStatus __attribute__((used));
volatile uint8_t  spa06ProbeChipId __attribute__((used));
volatile uint32_t spa06ProbeEvr    __attribute__((used));
volatile uint32_t spa06ProbeSer    __attribute__((used));
volatile uint32_t spa06ProbeGpioModer __attribute__((used));
volatile uint32_t spa06ProbeGpioAfrh  __attribute__((used));
volatile uint32_t spa06ProbeI3cCfgr   __attribute__((used));
// Bitmap of 7-bit I2C addresses that ACKed an address-only ping. 128 bits.
volatile uint32_t spa06ProbeAckMap[4] __attribute__((used));
// GPIO continuity test results. byte layout per slot:
//   bit 0 = SCL state, bit 1 = SDA state
// One slot per test step (see comments in spa06ProbeGpioCheck).
volatile uint32_t spa06ProbeGpio __attribute__((used));
// Bit-bang I2C address-scan map. Same layout as spa06ProbeAckMap. Independent
// of the I3C peripheral so we can tell the bus side apart from the peripheral
// side: if bit-bang ACKs an address but I3C doesn't, the I3C peripheral
// config is wrong; if neither ACKs anything, the chip itself isn't there or
// not powered.
volatile uint32_t spa06ProbeBbAckMap[4] __attribute__((used));

static void spa06ProbeI3cInit(void)
{
    // 1. Clock the I3C1 peripheral on APB1; PCLK1 is the kernel clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I3C1);
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I3C1);
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I3C1);

    // 2. GPIOC clock and pin setup (PC10=SCL, PC11=SDA, AF4)
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    // Claim the pins via BF's IO ownership so later cleanup (unusedPinsInit)
    // doesn't revert them to input-pull-up.
    IOInit(IOGetByTag(DEFIO_TAG_E(PC10)), OWNER_I2C_SCL, 1);
    IOInit(IOGetByTag(DEFIO_TAG_E(PC11)), OWNER_I2C_SDA, 1);

    LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_10 | LL_GPIO_PIN_11, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_10, LL_GPIO_AF_4);
    LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_11, LL_GPIO_AF_4);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);

    // 3. Controller mode + I2C-mode-friendly timing
    //    Hand-picked timing from ST example for I3C1 in mixed-bus at PCLK1=144 MHz,
    //    targeting ~400 kHz on the open-drain (I2C) phase. Values pulled from the
    //    CubeMX-generated I3C1 init helper for the C562 platform.
    LL_I3C_SetMode(I3C1, LL_I3C_MODE_CONTROLLER);
    LL_I3C_DisableArbitrationHeader(I3C1);          // suppress 7'h7E ARB header
    // Very slow I2C bus (~50 kHz OD on PCLK1=144 MHz). Wider SCL low/high
    // duty so marginal pull-ups can pull SDA high cleanly between bytes.
    // TIMINGR0: SCL OD high = 0xFF clocks, SCL OD low = 0xFF clocks, no I3C
    // push-pull use. Empirical bring-up value -- can tighten once a chip ACKs.
    LL_I3C_ConfigClockWaveForm(I3C1, 0x00FFFFFFU);
    LL_I3C_SetBusCharacteristic(I3C1, 0x1D00FFU);
    LL_I3C_ConfigCtrlFifo(I3C1,
                          LL_I3C_RXFIFO_THRESHOLD_1_2,
                          LL_I3C_TXFIFO_THRESHOLD_1_2,
                          LL_I3C_CTRL_FIFO_CONTROL_ONLY);
    LL_I3C_Enable(I3C1);

    // Snapshot GPIO + I3C state right after init so the post-boot SWD read
    // can confirm AF mux survived (otherwise BF's pin cleanup pass clobbered
    // it and the I3C peripheral never actually drove the pads).
    spa06ProbeGpioModer = GPIOC->MODER;
    spa06ProbeGpioAfrh  = GPIOC->AFR[1];
    spa06ProbeI3cCfgr   = I3C1->CFGR;
}

static bool spa06ProbeWriteReg(uint8_t addr, uint8_t reg, uint8_t value, bool stop)
{
    // Write phase: 2 bytes ([reg, value]) with STOP. For a "register pointer set"
    // (no value, restart), pass value=0 and stop=false; chip ignores the second
    // byte if the FIFO already starts a restart -- but typical SPA06 read
    // protocol uses just one byte (reg address), then restart + read.
    (void)value;
    uint32_t bytes = stop ? 2 : 1;
    LL_I3C_ControllerHandleMessage(I3C1,
                                   addr,
                                   bytes,
                                   LL_I3C_DIRECTION_WRITE,
                                   LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C,
                                   stop ? LL_I3C_GENERATE_STOP : LL_I3C_GENERATE_RESTART);
    // Push register byte
    while (!LL_I3C_IsActiveFlag_TXFNF(I3C1));
    LL_I3C_TransmitData8(I3C1, reg);
    if (stop) {
        while (!LL_I3C_IsActiveFlag_TXFNF(I3C1));
        LL_I3C_TransmitData8(I3C1, value);
    }
    // Wait for frame complete or error
    uint32_t guard = 100000;
    while (!LL_I3C_IsActiveFlag_FC(I3C1) && !LL_I3C_IsActiveFlag_ERR(I3C1) && guard--);
    spa06ProbeEvr = I3C1->EVR;
    spa06ProbeSer = I3C1->SER;
    if (LL_I3C_IsActiveFlag_ERR(I3C1)) {
        I3C1->CEVR = I3C1->EVR;  // clear all flags
        return false;
    }
    I3C1->CEVR = I3C1->EVR;
    return true;
}

static bool spa06ProbeReadByte(uint8_t addr, uint8_t reg, uint8_t *out)
{
    // I2C-style "read register": write reg pointer (no STOP, restart), then read
    // 1 byte (STOP).
    if (!spa06ProbeWriteReg(addr, reg, 0, false)) {
        return false;
    }
    LL_I3C_ControllerHandleMessage(I3C1,
                                   addr,
                                   1,
                                   LL_I3C_DIRECTION_READ,
                                   LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C,
                                   LL_I3C_GENERATE_STOP);
    uint32_t guard = 100000;
    while (!LL_I3C_IsActiveFlag_RXFNE(I3C1) && !LL_I3C_IsActiveFlag_ERR(I3C1) && guard--);
    if (LL_I3C_IsActiveFlag_ERR(I3C1)) {
        spa06ProbeEvr = I3C1->EVR;
        spa06ProbeSer = I3C1->SER;
        I3C1->CEVR = I3C1->EVR;
        return false;
    }
    *out = LL_I3C_ReceiveData8(I3C1);
    // Wait for STOP / frame complete
    guard = 100000;
    while (!LL_I3C_IsActiveFlag_FC(I3C1) && guard--);
    spa06ProbeEvr = I3C1->EVR;
    spa06ProbeSer = I3C1->SER;
    I3C1->CEVR = I3C1->EVR;
    return true;
}

// Address-ping: send a 0-byte write (address only, STOP). ACK → device exists.
static bool spa06ProbePing(uint8_t addr)
{
    LL_I3C_ControllerHandleMessage(I3C1,
                                   addr,
                                   0,
                                   LL_I3C_DIRECTION_WRITE,
                                   LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C,
                                   LL_I3C_GENERATE_STOP);
    uint32_t guard = 100000;
    while (!LL_I3C_IsActiveFlag_FC(I3C1) && !LL_I3C_IsActiveFlag_ERR(I3C1) && guard--);
    const bool ack = !LL_I3C_IsActiveFlag_ERR(I3C1) || (I3C1->SER & I3C_SER_ANACK) == 0;
    I3C1->CEVR = I3C1->EVR;
    return ack;
}

// Wire-side continuity check before bringing I3C up. Configure PC10/PC11 as
// plain GPIO (input pull-up first, then floating-input, then output) and look
// at what the pads read. The 32-bit result encodes four steps:
//   bits  3:0   step 0 - input pull-up   : bit0 = SCL idle, bit1 = SDA idle
//   bits  7:4   step 1 - input floating  : bit0 = SCL, bit1 = SDA
//   bits 11:8   step 2 - SCL drive low   : SCL should be 0, SDA still 1
//   bits 15:12  step 3 - SDA drive low   : SCL high (released), SDA should be 0
// External I2C pull-ups will hold the lines HIGH in the floating step; if
// they read LOW with no pull-up, the bus has no pull-up resistors and the
// chip can never assert ACK (ACK is the slave pulling SDA low against the
// bus pull-up). If a pin can't be pulled low by the MCU at all, it's not
// physically routed to PC10/PC11 the way we think it is.
static void spa06ProbeGpioCheck(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    uint32_t result = 0;

    // Step 0: input + pull-up. Both lines should read 1.
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }
    if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_10)) result |= (1U << 0);
    if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_11)) result |= (1U << 1);

    // Step 1: floating input. If the bus has an external pull-up the line
    // stays HIGH; with no external pull-up the line floats and likely reads
    // 0 (unless there's residual charge).
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);
    for (volatile int i = 0; i < 10000; i++) { __NOP(); }
    if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_10)) result |= (1U << 4);
    if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_11)) result |= (1U << 5);

    // Step 2: drive SCL (PC10) LOW as open-drain output, SDA stays floating.
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_10);  // drive low
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }
    if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_10)) result |= (1U << 8);
    if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_11)) result |= (1U << 9);
    // Release SCL (back to input pull-up so it can float high)
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

    // Step 3: drive SDA (PC11) LOW open-drain.
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_11);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }
    if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_10)) result |= (1U << 12);
    if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_11)) result |= (1U << 13);

    // Release both pins back to inputs floating so the I3C init can take
    // them over cleanly.
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_NO);

    spa06ProbeGpio = result;
}

// ---- GPIO bit-bang I2C, independent of the I3C peripheral ----------------
//
// PC10 = SCL, PC11 = SDA. Both open-drain with external pull-ups (verified
// by spa06ProbeGpioCheck). To "drive low" the pin is set as MODE_OUTPUT with
// OTYPE=OPENDRAIN and ODR=0; to "release" (let the bus pull high) the pin is
// set back to MODE_INPUT (floating, external pull-ups take over).

#define BB_SCL_PIN  LL_GPIO_PIN_10
#define BB_SDA_PIN  LL_GPIO_PIN_11

static inline void bbDelay(void)
{
    // ~5 us at 144 MHz CPU -- yields ~100 kHz I2C, plenty of margin.
    for (volatile int i = 0; i < 720; i++) { __NOP(); }
}

static inline void bbSclLow(void)
{
    LL_GPIO_ResetOutputPin(GPIOC, BB_SCL_PIN);
    LL_GPIO_SetPinMode(GPIOC, BB_SCL_PIN, LL_GPIO_MODE_OUTPUT);
}
static inline void bbSclRelease(void)
{
    LL_GPIO_SetPinMode(GPIOC, BB_SCL_PIN, LL_GPIO_MODE_INPUT);
}
static inline void bbSdaLow(void)
{
    LL_GPIO_ResetOutputPin(GPIOC, BB_SDA_PIN);
    LL_GPIO_SetPinMode(GPIOC, BB_SDA_PIN, LL_GPIO_MODE_OUTPUT);
}
static inline void bbSdaRelease(void)
{
    LL_GPIO_SetPinMode(GPIOC, BB_SDA_PIN, LL_GPIO_MODE_INPUT);
}
static inline int bbSdaRead(void)
{
    return LL_GPIO_IsInputPinSet(GPIOC, BB_SDA_PIN);
}

static void bbI2cInit(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    // Open-drain output type, no internal pull (use external pull-ups).
    LL_GPIO_SetPinOutputType(GPIOC, BB_SCL_PIN | BB_SDA_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOC, BB_SCL_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GPIOC, BB_SDA_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinSpeed(GPIOC, BB_SCL_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOC, BB_SDA_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    // Idle: both lines released (pulled high externally).
    bbSclRelease();
    bbSdaRelease();
    bbDelay();
}

static void bbStart(void)
{
    bbSdaRelease(); bbSclRelease(); bbDelay();
    bbSdaLow();     bbDelay();
    bbSclLow();     bbDelay();
}

static void bbStop(void)
{
    bbSdaLow();     bbDelay();
    bbSclRelease(); bbDelay();
    bbSdaRelease(); bbDelay();
}

// Shift out one byte MSB first; sample the ACK on the 9th SCL pulse. Returns
// 1 if ACK (slave pulled SDA low), 0 if NACK.
static int bbWriteByte(uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        if (b & (1 << i)) {
            bbSdaRelease();
        } else {
            bbSdaLow();
        }
        bbDelay();
        bbSclRelease(); bbDelay();
        bbSclLow();     bbDelay();
    }
    // ACK slot: release SDA so the slave can drive it.
    bbSdaRelease(); bbDelay();
    bbSclRelease(); bbDelay();
    const int ack = (bbSdaRead() == 0) ? 1 : 0;
    bbSclLow();     bbDelay();
    return ack;
}

// Address-only ping: START + write address byte + STOP. Returns ACK status.
static int bbPing(uint8_t addr7)
{
    bbStart();
    const int ack = bbWriteByte((uint8_t)(addr7 << 1));     // R/W=0 (write)
    bbStop();
    return ack;
}

// Read one byte, send NACK (no more reads coming) + STOP.
static uint8_t bbReadByteNack(void)
{
    uint8_t v = 0;
    bbSdaRelease();
    for (int i = 7; i >= 0; i--) {
        bbDelay();
        bbSclRelease(); bbDelay();
        if (bbSdaRead()) v |= (1 << i);
        bbSclLow();
    }
    // NACK: keep SDA released (high) during the 9th clock.
    bbSdaRelease(); bbDelay();
    bbSclRelease(); bbDelay();
    bbSclLow();     bbDelay();
    return v;
}

volatile uint8_t spa06ProbeBbReadStep __attribute__((used));

// I2C read register: S addr W reg Sr addr R data NACK P
static int bbReadReg(uint8_t addr7, uint8_t reg, uint8_t *out)
{
    spa06ProbeBbReadStep = 0x10;
    bbStart();
    spa06ProbeBbReadStep = 0x11;
    if (!bbWriteByte((uint8_t)(addr7 << 1))) { spa06ProbeBbReadStep = 0xE1; bbStop(); return 0; }
    spa06ProbeBbReadStep = 0x12;
    if (!bbWriteByte(reg))                    { spa06ProbeBbReadStep = 0xE2; bbStop(); return 0; }
    spa06ProbeBbReadStep = 0x13;
    // Repeated start
    bbSdaRelease(); bbDelay();
    bbSclRelease(); bbDelay();
    bbSdaLow();     bbDelay();
    bbSclLow();     bbDelay();
    spa06ProbeBbReadStep = 0x14;
    if (!bbWriteByte((uint8_t)((addr7 << 1) | 1))) { spa06ProbeBbReadStep = 0xE3; bbStop(); return 0; }
    spa06ProbeBbReadStep = 0x15;
    *out = bbReadByteNack();
    spa06ProbeBbReadStep = 0x16;
    bbStop();
    spa06ProbeBbReadStep = 0x17;
    return 1;
}

volatile uint8_t spa06ProbeBbChipId __attribute__((used));
volatile uint8_t spa06ProbeBbProdRev __attribute__((used));

static void spa06ProbeBbScan(void)
{
    bbI2cInit();
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (bbPing(a)) {
            spa06ProbeBbAckMap[a / 32] |= (1U << (a % 32));
        }
    }
    // If 0x76 ACKed, read CHIP_ID (reg 0x0D). SPA06-003 = 0x11.
    if (spa06ProbeBbAckMap[3] & (1U << 22)) {
        uint8_t v = 0;
        if (bbReadReg(0x76, 0x0D, &v)) {
            spa06ProbeBbProdRev = v;
            spa06ProbeBbChipId  = v & 0x0F;
        }
    } else if (spa06ProbeBbAckMap[3] & (1U << 23)) {
        uint8_t v = 0;
        if (bbReadReg(0x77, 0x0D, &v)) {
            spa06ProbeBbProdRev = v;
            spa06ProbeBbChipId  = v & 0x0F;
        }
    }
}

void spa06ProbeRun(void)
{
    spa06ProbeStatus = 0xAA000000U;
    spa06ProbeGpioCheck();
    spa06ProbeBbScan();
    spa06ProbeI3cInit();
    delay(2);

    // Address-only scan across 7-bit I2C address range. Sets bit in ackMap[]
    // for any address that ACKs.
    spa06ProbeStatus = 0xBB000000U;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (spa06ProbePing(a)) {
            spa06ProbeAckMap[a / 32] |= (1U << (a % 32));
        }
    }

    spa06ProbeStatus = 0xCC000000U;
    uint8_t chipId = 0;
    bool ok76 = false, ok77 = false;
    if (spa06ProbeReadByte(SPA06_I2C_ADDR, SPA06_REG_CHIP_ID, &chipId)) {
        ok77 = true;
        spa06ProbeChipId = chipId;
        spa06ProbeStatus = 0xCC770000U | (uint32_t)chipId;
    }
    if (!ok77) {
        chipId = 0;
        if (spa06ProbeReadByte(0x76, SPA06_REG_CHIP_ID, &chipId)) {
            ok76 = true;
            spa06ProbeChipId = chipId;
            spa06ProbeStatus = 0xCC760000U | (uint32_t)chipId;
        }
    }
    if (!ok76 && !ok77) {
        spa06ProbeStatus |= 0x000000EEU;
    }
}

#endif
