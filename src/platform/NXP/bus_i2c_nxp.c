/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * LPI2C master driver for the NXP parts. Each MTDR word is a 3 bit command and a
 * data byte, so a whole register read is queued up front. The interrupt path feeds
 * the FIFO; the DMA path uses a scatter gather chain.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_utils.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "platform/nxp_clock.h"

#include "pg/bus_i2c.h"

#include "iomux_nxp.h"
#include "iomux_signal_def.h"

// Below the gyro EXTI group, so the gyro never waits behind an I2C word.
#define I2C_IMXRT_PRIORITY NVIC_BUILD_PRIORITY(2, 0)

#if defined(USE_DMA)
#include "drivers/dma.h"
#include "platform/dma.h"
#define NXP_I2C_DMA
#endif

// Pin arrays are left empty; pads are resolved against the generated nxpLpi2cPins table.
const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *)LPI2C1,
        .ev_irq = LPI2C1_IRQn,
        .er_irq = LPI2C1_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *)LPI2C2,
        .ev_irq = LPI2C2_IRQn,
        .er_irq = LPI2C2_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = (i2cResource_t *)LPI2C3,
        .ev_irq = LPI2C3_IRQn,
        .er_irq = LPI2C3_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = (i2cResource_t *)LPI2C4,
        .ev_irq = LPI2C4_IRQn,
        .er_irq = LPI2C4_IRQn,
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

// MTDR[CMD] values
#define LPI2C_CMD_TXD       0u  // send the data byte
#define LPI2C_CMD_RXD       1u  // receive (data byte + 1) bytes
#define LPI2C_CMD_STOP      2u
#define LPI2C_CMD_START     4u  // (repeated) start, then send the address byte

// Start, register index, repeated start, receive.
#define I2C_CMD_MAX         4

// Any of these ends the transfer.
#define LPI2C_MSR_ERROR_FLAGS   (LPI2C_MSR_NDF_MASK | LPI2C_MSR_ALF_MASK \
                                 | LPI2C_MSR_FEF_MASK | LPI2C_MSR_PLTF_MASK)

// Cleared before a transfer starts: these flags are sticky.
#define LPI2C_MSR_CLEAR_FLAGS   (LPI2C_MSR_ERROR_FLAGS | LPI2C_MSR_SDF_MASK \
                                 | LPI2C_MSR_EPF_MASK)

typedef struct i2cTransfer_s {
    volatile bool busy;
    volatile bool error;
    bool reading;
    bool usingDma;
    uint8_t *buf;
    uint8_t len;
    uint8_t cmdCount;       // command words this transfer needs
    uint8_t cmdIndex;       // command words already in the FIFO
    uint8_t txIndex;        // data bytes already pushed into the FIFO
    uint8_t rxIndex;        // data bytes already taken out of the FIFO
    bool stopQueued;
    volatile bool recover;  // a pin low timeout or repeated lost arbitration, so the bus needs recovery
    uint8_t alfCount;       // transfers lost to arbitration since the last good one
    timeUs_t startUs;
    timeUs_t recoverUs;     // when the bus was last recovered
    bool held;              // the bus read busy after its last recovery
    bool masterBusy;        // a start found the master busy and none has found it idle since
    timeUs_t masterBusyUs;  // when a start first found the master busy
} i2cTransfer_t;

static i2cTransfer_t i2cTransfers[I2CDEV_COUNT];

// Master transmit FIFO depth, from PARAM[MTXFIFO] (log2).
static uint8_t i2cTxFifoDepth[I2CDEV_COUNT];

// The engine reads the command words itself, so they go in DTCM.
static DMA_DATA_ZERO_INIT uint16_t i2cCmdWords[I2CDEV_COUNT][I2C_CMD_MAX];

static volatile uint16_t i2cErrorCount = 0;

// A line held low makes every transfer lose arbitration, so this many in a row recover the bus.
#define I2C_ALF_RECOVER_COUNT       3
#define I2C_RECOVER_INTERVAL_US     100000
// BUSIDLE is about two SCL periods, 20 us at 100 kHz.
#define I2C_BUS_IDLE_WAIT_US        25

// Read over SWD only: the status at the last failed transfer end, and the bus recoveries.
static volatile uint32_t i2cLastErrorStatus[I2CDEV_COUNT];
static volatile uint16_t i2cTimeoutCount = 0;

// The clock enum lists the LPI2C instances in order, one entry each.
STATIC_ASSERT(NXP_CLK_FLEXIO1 - NXP_CLK_LPI2C1 >= I2CDEV_COUNT,
              clock_enum_needs_one_entry_per_lpi2c_instance);

static uint32_t i2cClockHz(i2cDevice_e device)
{
    return nxpClockHz((nxpClock_e)(NXP_CLK_LPI2C1 + (device - I2CDEV_1)));
}

/* SCL period = CLKLO + 1 + CLKHI + 1 + floor((2 + FILTSCL) / 2^PRESCALE) prescaled cycles.
 * Fast mode needs 1.3 us low and 0.6 us high, so CLKLO = 2 * CLKHI, SETHOLD = CLKHI and
 * DATAVD = CLKHI / 2. The fields are 6 bits; the fastest rate not above the request wins.
 */
#define LPI2C_CLKHI_MAX     31
#define LPI2C_PRESCALE_MAX  LPI2C_MCFGR1_PRESCALE_MASK

// Fast mode ignores spikes up to 50 ns; filter cycles are the clock rate divided by this.
#define LPI2C_GLITCH_HZ     20000000u
#define LPI2C_FILT_MAX      15

typedef struct i2cTiming_s {
    uint8_t prescale;
    uint8_t clkHi;
    uint8_t clkLo;
    uint8_t setHold;
    uint8_t dataVd;
    uint8_t filt;
    uint16_t busIdle;
    uint16_t pinLow;
} i2cTiming_t;

static void i2cComputeTiming(uint32_t clockHz, uint32_t baudHz, i2cTiming_t *timing)
{
    const uint32_t filt = constrain((clockHz + LPI2C_GLITCH_HZ - 1) / LPI2C_GLITCH_HZ, 1, LPI2C_FILT_MAX);

    // The slowest setting the fields can hold is also the starting guess.
    uint32_t bestPrescale = LPI2C_PRESCALE_MAX;
    uint32_t bestClkHi = LPI2C_CLKHI_MAX;
    uint32_t bestRate = 0;

    for (uint32_t prescale = 0; prescale <= LPI2C_PRESCALE_MAX; prescale++) {
        const uint32_t prescaled = clockHz >> prescale;
        const uint32_t filterCycles = (2 + filt) >> prescale;

        for (uint32_t clkHi = 1; clkHi <= LPI2C_CLKHI_MAX; clkHi++) {
            // CLKLO is 2 * CLKHI, so CLKLO + 1 + CLKHI + 1 is 3 * CLKHI + 2.
            const uint32_t cycles = 3 * clkHi + 2 + filterCycles;
            const uint32_t rate = prescaled / cycles;

            if (rate <= baudHz && rate > bestRate) {
                bestRate = rate;
                bestPrescale = prescale;
                bestClkHi = clkHi;
            }
        }
    }

    timing->prescale = bestPrescale;
    timing->clkHi = bestClkHi;
    timing->clkLo = 2 * bestClkHi;
    timing->setHold = bestClkHi;
    timing->dataVd = bestClkHi > 1 ? bestClkHi / 2 : 1;
    timing->filt = filt;

    // BUSIDLE must be at least CLKLO + SETHOLD + 2; twice that leaves room.
    const uint32_t busIdle = 2 * (timing->clkLo + timing->setHold + 2);
    timing->busIdle = busIdle > LPI2C_MCFGR2_BUSIDLE_MASK ? LPI2C_MCFGR2_BUSIDLE_MASK : busIdle;

    // PINLOW counts 256 prescaled cycles, half the software timeout so it fires first.
    const uint32_t prescaled = clockHz >> timing->prescale;
    const uint32_t pinLow = (uint32_t)(((uint64_t)prescaled * (I2C_TIMEOUT_US / 2)) / (1000000ull * 256ull));
    timing->pinLow = constrain(pinLow, 1, 0xfff);
}

// A register index of 0xff means the slave has none.
static uint8_t i2cBuildCommands(uint16_t *cmd, uint8_t addr, uint8_t reg, bool reading, uint8_t len)
{
    uint8_t n = 0;
    const bool sendReg = reg != 0xff;

    if (sendReg || !reading) {
        // 7 bit address in bits 7:1, direction in bit 0.
        cmd[n++] = LPI2C_MTDR_CMD(LPI2C_CMD_START) | ((uint16_t)addr << 1);
        if (sendReg) {
            cmd[n++] = LPI2C_MTDR_CMD(LPI2C_CMD_TXD) | reg;
        }
    }

    if (reading) {
        cmd[n++] = LPI2C_MTDR_CMD(LPI2C_CMD_START) | ((uint16_t)addr << 1) | 1u;
        // The byte count is held as count minus one.
        cmd[n++] = LPI2C_MTDR_CMD(LPI2C_CMD_RXD) | (uint16_t)(len - 1);
    }

    return n;
}

// i2cDevice_t keeps a claimed pin as an IO_t; the chip table is keyed by tag.
static ioTag_t i2cPinTag(IO_t io)
{
    return io ? DEFIO_TAG_MAKE(IO_GPIOPortIdx(io), IO_GPIOPinIdx(io)) : IO_TAG_NONE;
}

// NULL when the chip cannot route the signal to the pin.
static const nxpPinMux_t *i2cPadMux(i2cDevice_e device, bool sda, ioTag_t pin)
{
    return NXP_PIN_MUX(nxpLpi2cPins, (uint8_t)(device - I2CDEV_1 + 1),
                         sda ? NXP_ROLE_LPI2C_SDA : NXP_ROLE_LPI2C_SCL, pin);
}

// True once every word of the transfer is in the FIFO.
static bool i2cAllQueued(const i2cTransfer_t *t)
{
    return t->stopQueued;
}

// Command words, then write data, then the stop. False once all are queued.
static bool i2cPushCommand(i2cDevice_e device)
{
    LPI2C_Type *lpi2c = I2C_INST(i2cDevice[device].reg);
    i2cTransfer_t *t = &i2cTransfers[device];

    if (t->cmdIndex < t->cmdCount) {
        lpi2c->MTDR = i2cCmdWords[device][t->cmdIndex++];
        return true;
    }

    if (!t->reading && t->txIndex < t->len) {
        lpi2c->MTDR = LPI2C_MTDR_CMD(LPI2C_CMD_TXD) | t->buf[t->txIndex++];
        return true;
    }

    if (!t->stopQueued) {
        lpi2c->MTDR = LPI2C_MTDR_CMD(LPI2C_CMD_STOP);
        t->stopQueued = true;
        return true;
    }

    return false;
}

// Feed the transmit FIFO while it has room; MFSR[TXCOUNT] says how full it is.
static FAST_IRQ_HANDLER void i2cPushCommands(i2cDevice_e device)
{
    const LPI2C_Type *lpi2c = I2C_INST(i2cDevice[device].reg);

    while (((lpi2c->MFSR & LPI2C_MFSR_TXCOUNT_MASK) >> LPI2C_MFSR_TXCOUNT_SHIFT) < i2cTxFifoDepth[device]) {
        if (!i2cPushCommand(device)) {
            return;
        }
    }
}

// MRDR[RXEMPTY] marks an empty read, so a byte landing during the loop is not missed.
static FAST_IRQ_HANDLER void i2cDrainRx(i2cDevice_e device)
{
    LPI2C_Type *lpi2c = I2C_INST(i2cDevice[device].reg);
    i2cTransfer_t *t = &i2cTransfers[device];

    while (t->rxIndex < t->len) {
        const uint32_t mrdr = lpi2c->MRDR;
        if (mrdr & LPI2C_MRDR_RXEMPTY_MASK) {
            return;
        }
        t->buf[t->rxIndex++] = mrdr & LPI2C_MRDR_DATA_MASK;
    }
}

#ifdef NXP_I2C_DMA

// Shorter transfers use the interrupt path. Above 255 disables DMA, as the length is one byte.
#ifndef I2C_DMA_MIN_LEN
#define I2C_DMA_MIN_LEN     16
#endif

// Command words, the transmit to receive request switch, and the data.
#define I2C_TCD_MAX         3

// The engine reads these, so they go in DTCM.
static DMA_TCD_LIST(i2cTcds, I2CDEV_COUNT * I2C_TCD_MAX);

// One DMA request per instance, so a read writes this into MDER to switch to receive.
static DMA_DATA const uint8_t i2cMderReceive = LPI2C_MDER_RDDE_MASK;

// DMAMUX request sources, one per instance.
static const uint32_t i2cDmaMuxIds[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    [I2CDEV_1] = kDmaRequestMuxLPI2C1,
#endif
#ifdef USE_I2C_DEVICE_2
    [I2CDEV_2] = kDmaRequestMuxLPI2C2,
#endif
#ifdef USE_I2C_DEVICE_3
    [I2CDEV_3] = kDmaRequestMuxLPI2C3,
#endif
#ifdef USE_I2C_DEVICE_4
    [I2CDEV_4] = kDmaRequestMuxLPI2C4,
#endif
};

static dmaIdentifier_e i2cDmaChannel[I2CDEV_COUNT];

static FAST_IRQ_HANDLER void i2cTransferEnd(i2cDevice_e device, bool failed);

// The chain ends before the bus does; the stop detect interrupt ends the transfer.
static FAST_IRQ_HANDLER void i2cDmaCallback(dmaChannelDescriptor_t *descriptor)
{
    const i2cDevice_e device = (i2cDevice_e)descriptor->userParam;

    if (device >= I2CDEV_COUNT) {
        return;
    }

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF) && i2cTransfers[device].busy) {
        i2cTransferEnd(device, true);
        return;
    }

    dmaDisableRequest(DMA_CHANNEL_TO_IDENTIFIER(descriptor->channel));
}

static void i2cDmaInit(i2cDevice_e device)
{
    if (i2cDmaChannel[device] != DMA_NONE) {
        return;
    }

    const dmaIdentifier_e identifier = dmaGetFreeIdentifierForRequest(i2cDmaMuxIds[device]);

    if (identifier == DMA_NONE || !dmaAllocate(identifier, OWNER_I2C_SDA, RESOURCE_INDEX(device))) {
        return;
    }

    dmaEnable(identifier);
    dmaMuxEnable(identifier, i2cDmaMuxIds[device]);
    dmaSetHandler(identifier, i2cDmaCallback, I2C_IMXRT_PRIORITY, device);

    i2cDmaChannel[device] = identifier;
}

// False falls back to the interrupt path.
static bool i2cDmaBuildChain(i2cDevice_e device)
{
    LPI2C_Type *lpi2c = I2C_INST(i2cDevice[device].reg);
    const i2cTransfer_t *t = &i2cTransfers[device];
    dmaTcd_t *list = &i2cTcds[device * I2C_TCD_MAX];
    unsigned count = 0;

    // One word per request. TXWATER 1 keeps a word in the FIFO between links, so AUTOSTOP does not fire early.
    const dmaTransfer_t cmdTransfer = {
        .src = i2cCmdWords[device],
        .dst = (void *)&lpi2c->MTDR,
        .srcOffset = sizeof(uint16_t),
        .dstOffset = 0,
        .srcSize = DMA_XFER_SIZE_2B,
        .dstSize = DMA_XFER_SIZE_2B,
        .minorBytes = sizeof(uint16_t),
        .majorCount = t->cmdCount,
    };

    if (!dmaBuildTcd(&list[count++], &cmdTransfer)) {
        return false;
    }

    if (t->reading) {
        const dmaTransfer_t mderTransfer = {
            .src = &i2cMderReceive,
            .dst = (void *)&lpi2c->MDER,
            .srcOffset = 0,
            .dstOffset = 0,
            .srcSize = DMA_XFER_SIZE_1B,
            .dstSize = DMA_XFER_SIZE_1B,
            .minorBytes = 1,
            .majorCount = 1,
        };

        if (!dmaBuildTcd(&list[count++], &mderTransfer)) {
            return false;
        }
    }

    // A byte write to MTDR leaves CMD zero, which sends the byte.
    dmaTransfer_t dataTransfer = {
        .srcSize = DMA_XFER_SIZE_1B,
        .dstSize = DMA_XFER_SIZE_1B,
        .minorBytes = 1,
        .majorCount = t->len,
    };

    if (t->reading) {
        dataTransfer.src = (const void *)&lpi2c->MRDR;
        dataTransfer.dst = t->buf;
        dataTransfer.srcOffset = 0;
        dataTransfer.dstOffset = 1;
    } else {
        dataTransfer.src = t->buf;
        dataTransfer.dst = (void *)&lpi2c->MTDR;
        dataTransfer.srcOffset = 1;
        dataTransfer.dstOffset = 0;
    }

    if (!dmaBuildTcd(&list[count++], &dataTransfer)) {
        return false;
    }

    return dmaSetTransferChain(i2cDmaChannel[device], list, count);
}

#endif // NXP_I2C_DMA

static FAST_IRQ_HANDLER void i2cTransferEnd(i2cDevice_e device, bool failed)
{
    LPI2C_Type *lpi2c = I2C_INST(i2cDevice[device].reg);
    i2cTransfer_t *t = &i2cTransfers[device];
    const uint32_t status = lpi2c->MSR;

    lpi2c->MIER = 0;
    // Marked ended before anything else, so an interrupt already pending sees it.
    t->error = failed;
    t->busy = false;

#ifdef NXP_I2C_DMA
    if (t->usingDma) {
        dmaDisableRequest(i2cDmaChannel[device]);
        lpi2c->MDER = 0;
    }
#endif

    if (failed) {
        // Drop the rest of the command list before the stop.
        lpi2c->MCR |= LPI2C_MCR_RTF_MASK | LPI2C_MCR_RRF_MASK;
        // A stop word left with an idle master stays in the FIFO, so only a busy master gets one.
        const bool needStop = (lpi2c->MSR & (LPI2C_MSR_MBF_MASK | LPI2C_MSR_SDF_MASK)) == LPI2C_MSR_MBF_MASK;
        lpi2c->MSR = LPI2C_MSR_ERROR_FLAGS;
        i2cLastErrorStatus[device] = status;
        if (needStop) {
            lpi2c->MTDR = LPI2C_MTDR_CMD(LPI2C_CMD_STOP);
        }
        if ((status & LPI2C_MSR_PLTF_MASK)
            || ((status & LPI2C_MSR_ALF_MASK) && ++t->alfCount >= I2C_ALF_RECOVER_COUNT)) {
            t->recover = true;
        }
        i2cErrorCount++;
    } else {
        t->alfCount = 0;
    }

    t->stopQueued = true;
}

static FAST_IRQ_HANDLER void i2cIrqHandler(i2cDevice_e device)
{
    LPI2C_Type *lpi2c = I2C_INST(i2cDevice[device].reg);
    i2cTransfer_t *t = &i2cTransfers[device];

    // A late interrupt for a transfer the timeout already ended.
    if (!t->busy) {
        lpi2c->MIER = 0;
        return;
    }

    const uint32_t status = lpi2c->MSR;

    if (status & LPI2C_MSR_ERROR_FLAGS) {
        // No acknowledge is the normal answer from an absent device.
        i2cTransferEnd(device, true);
        return;
    }

    // In the DMA path the engine owns both data registers.
    if (t->reading && !t->usingDma) {
        i2cDrainRx(device);
    }

    if ((status & LPI2C_MSR_TDF_MASK) && !t->usingDma) {
        i2cPushCommands(device);
        if (i2cAllQueued(t)) {
            // The stop flag ends the transfer.
            lpi2c->MIER &= ~LPI2C_MIER_TDIE_MASK;
        }
    }

    if (status & LPI2C_MSR_SDF_MASK) {
        lpi2c->MSR = LPI2C_MSR_SDF_MASK;
        // The last bytes can land between the drain above and here.
        if (t->reading && !t->usingDma) {
            i2cDrainRx(device);
        }
        // Only the interrupt path keeps a count to detect a short read.
        const bool shortRead = t->reading && !t->usingDma && t->rxIndex < t->len;
        i2cTransferEnd(device, shortRead);
    }
}

void i2cPinConfigure(const i2cConfig_t *i2cConfig)
{
    for (int index = 0; index < I2CDEV_COUNT; index++) {
        const i2cHardware_t *hardware = &i2cHardware[index];

        if (!hardware->reg) {
            continue;
        }

        const i2cDevice_e device = hardware->device;
        i2cDevice_t *pDev = &i2cDevice[device];

        memset(pDev, 0, sizeof(*pDev));

        // A pad the chip cannot route leaves the bus unclaimed.
        if (i2cPadMux(device, false, i2cConfig[device].ioTagScl)) {
            pDev->scl = IOGetByTag(i2cConfig[device].ioTagScl);
        }
        if (i2cPadMux(device, true, i2cConfig[device].ioTagSda)) {
            pDev->sda = IOGetByTag(i2cConfig[device].ioTagSda);
        }

        if (pDev->scl && pDev->sda) {
            pDev->hardware = hardware;
            pDev->reg = hardware->reg;
            pDev->pullUp = i2cConfig[device].pullUp;
            pDev->clockSpeed = i2cConfig[device].clockSpeed;
        }
    }
}

static void i2cSetup(i2cDevice_e device)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];
    const i2cHardware_t *hardware = pDev->hardware;

    if (!hardware || !pDev->scl || !pDev->sda) {
        return;
    }

    LPI2C_Type *lpi2c = I2C_INST(pDev->reg);
    i2cTransfer_t *t = &i2cTransfers[device];

    t->busy = false;
    t->error = false;
    t->usingDma = false;
    t->stopQueued = true;
    t->recover = false;
    t->alfCount = 0;

    // Resolved before claiming anything, so a failed bus leaves nothing behind.
    const nxpPinMux_t *sclMux = i2cPadMux(device, false, i2cPinTag(pDev->scl));
    const nxpPinMux_t *sdaMux = i2cPadMux(device, true, i2cPinTag(pDev->sda));

    if (!sclMux || !sdaMux) {
        return;
    }

    if (!nxpClockGateEnable((nxpClock_e)(NXP_CLK_LPI2C1 + (device - I2CDEV_1)))) {
        return;
    }

    // PARAM reports the FIFO depth as a power of two and survives the reset.
    i2cTxFifoDepth[device] =
        1u << ((lpi2c->PARAM & LPI2C_PARAM_MTXFIFO_MASK) >> LPI2C_PARAM_MTXFIFO_SHIFT);

    // Release the pads before they are driven by hand; MCR[RST] clears the FIFOs too.
    lpi2c->MIER = 0;
    lpi2c->MCR = LPI2C_MCR_RST_MASK;
    lpi2c->MCR = 0;

    IOInit(pDev->scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(pDev->sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // A slave cut off mid byte by a supply dip holds SDA low; clock the bus by hand and send a stop.
    i2cUnstick(pDev->scl, pDev->sda);

    // Open drain with SION, so the master reads the line for arbitration and clock stretching.
    const ioConfig_t padCfg = IO_CONFIG(IOCFG_MODE_AF, IOCFG_DRIVE_MEDIUM,
                                        pDev->pullUp ? IOCFG_PULL_UP : IOCFG_PULL_NONE, 1, 1);

    nxpIomuxConfigPin(sclMux, padCfg);
    nxpIomuxConfigPin(sdaMux, padCfg);

    i2cTiming_t timing;
    const uint32_t baudHz = (pDev->clockSpeed ? pDev->clockSpeed : 400u) * 1000u;
    i2cComputeTiming(i2cClockHz(device), baudHz, &timing);

    // Host request off: it can otherwise gate the master on an input pin.
    lpi2c->MCFGR0 = 0;
    // No AUTOSTOP and IGNACK clear; PINCFG zero is two pin open drain.
    lpi2c->MCFGR1 = LPI2C_MCFGR1_PRESCALE(timing.prescale);
    lpi2c->MCFGR2 = LPI2C_MCFGR2_FILTSCL(timing.filt)
                  | LPI2C_MCFGR2_FILTSDA(timing.filt)
                  | LPI2C_MCFGR2_BUSIDLE(timing.busIdle);
    lpi2c->MCFGR3 = LPI2C_MCFGR3_PINLOW(timing.pinLow);
    lpi2c->MCCR0 = LPI2C_MCCR0_CLKLO(timing.clkLo)
                 | LPI2C_MCCR0_CLKHI(timing.clkHi)
                 | LPI2C_MCCR0_SETHOLD(timing.setHold)
                 | LPI2C_MCCR0_DATAVD(timing.dataVd);
    // Request while one word is still queued, and on any received data.
    lpi2c->MFCR = LPI2C_MFCR_TXWATER(1) | LPI2C_MFCR_RXWATER(0);
    lpi2c->MDER = 0;
    lpi2c->MSR = LPI2C_MSR_CLEAR_FLAGS;

    NVIC_SetPriority((IRQn_Type)hardware->ev_irq, NVIC_PRIORITY_LEVEL(I2C_IMXRT_PRIORITY));
    NVIC_EnableIRQ((IRQn_Type)hardware->ev_irq);

    lpi2c->MCR = LPI2C_MCR_MEN_MASK;

#ifdef NXP_I2C_DMA
    i2cDmaInit(device);
#endif
}

void i2cInit(i2cDevice_e device)
{
    i2cSetup(device);
}

// Runs in thread context only: it waits on the bus and sets the master up again.
static void i2cRecover(i2cDevice_e device)
{
    if (i2cTransfers[device].busy) {
        i2cTransferEnd(device, true);
    }
    i2cTimeoutCount++;
    i2cTransfers[device].masterBusy = false;
    i2cTransfers[device].recoverUs = microsISR();
    i2cSetup(device);
    // The master reads the bus busy until both lines have been high for one bus idle time.
    LPI2C_Type *lpi2c = I2C_INST(i2cDevice[device].reg);
    const timeUs_t initUs = microsISR();
    while ((lpi2c->MSR & LPI2C_MSR_BBF_MASK) && cmpTimeUs(microsISR(), initUs) < I2C_BUS_IDLE_WAIT_US) {
    }
    const bool held = (lpi2c->MSR & LPI2C_MSR_BBF_MASK) != 0;
    // A bus still held after the bus clear is tried again only once a second.
    i2cTransfers[device].held = held;
    if (held) {
        i2cTransfers[device].recoverUs += 1000000 - I2C_RECOVER_INTERVAL_US;
    }
    i2cTransfers[device].error = true;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

// A stop still in the FIFO would raise MSR[SDF] in the next transfer, so a start needs an idle master.
// Busy for I2C_TIMEOUT_US recovers the bus; true once idle, or recovered with the bus let go.
static bool i2cMasterIdle(i2cDevice_e device, bool wait)
{
    const LPI2C_Type *lpi2c = I2C_INST(i2cDevice[device].reg);
    i2cTransfer_t *t = &i2cTransfers[device];

    // MSR[BBF] with MBF clear is the bus held by something other than this master.
    while ((lpi2c->MSR & (LPI2C_MSR_MBF_MASK | LPI2C_MSR_BBF_MASK)) || (lpi2c->MFSR & LPI2C_MFSR_TXCOUNT_MASK)) {
        const timeUs_t now = microsISR();
        if (!t->masterBusy) {
            t->masterBusy = true;
            t->masterBusyUs = now;
        } else if (cmpTimeUs(now, t->masterBusyUs) >= (timeDelta_t)I2C_TIMEOUT_US) {
            i2cRecover(device);
            return !t->held;
        }
        if (!wait) {
            return false;
        }
    }
    t->masterBusy = false;
    return true;
}

// Only a blocking caller waits for a busy master.
static bool i2cStart(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf, bool reading, bool wait)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    i2cDevice_t *pDev = &i2cDevice[device];

    if (!pDev->reg) {
        return false;
    }

    // The receive command carries the count less one.
    if (reading && len == 0) {
        return false;
    }

    if (i2cBusy(device, NULL)) {
        return false;
    }

    LPI2C_Type *lpi2c = I2C_INST(pDev->reg);
    i2cTransfer_t *t = &i2cTransfers[device];

    // A held bus is left alone until its back-off ends, so it costs no idle wait per attempt.
    if (t->held) {
        if (cmpTimeUs(microsISR(), t->recoverUs) < 0) {
            return false;
        }
        t->held = false;
    }

    if (!i2cMasterIdle(device, wait)) {
        return false;
    }

    t->buf = buf;
    t->len = len;
    t->cmdCount = i2cBuildCommands(i2cCmdWords[device], addr, reg, reading, len);
    t->cmdIndex = 0;
    t->txIndex = 0;
    t->rxIndex = 0;
    t->reading = reading;
    t->stopQueued = false;
    t->usingDma = false;
    t->error = false;
    t->busy = true;
    t->startUs = microsISR();

    // Drop anything left in the FIFOs and the sticky flags of an earlier failure.
    lpi2c->MCR |= LPI2C_MCR_RTF_MASK | LPI2C_MCR_RRF_MASK;
    lpi2c->MSR = LPI2C_MSR_CLEAR_FLAGS;

#ifdef NXP_I2C_DMA
    if (len >= I2C_DMA_MIN_LEN && i2cDmaChannel[device] != DMA_NONE && i2cDmaBuildChain(device)) {
        t->usingDma = true;
        // The chain carries no stop word, so AUTOSTOP ends the frame. IGNACK stays off to keep MSR[NDF].
        lpi2c->MCFGR1 |= LPI2C_MCFGR1_AUTOSTOP_MASK;
        lpi2c->MIER = LPI2C_MIER_SDIE_MASK | LPI2C_MIER_NDIE_MASK | LPI2C_MIER_ALIE_MASK
                    | LPI2C_MIER_FEIE_MASK | LPI2C_MIER_PLTIE_MASK;
        dmaEnableRequest(i2cDmaChannel[device]);
        // Start on the transmit request; a read swaps to receive.
        lpi2c->MDER = LPI2C_MDER_TDDE_MASK;
        return true;
    }

    lpi2c->MCFGR1 &= ~(LPI2C_MCFGR1_AUTOSTOP_MASK | LPI2C_MCFGR1_IGNACK_MASK);
    lpi2c->MDER = 0;
#endif

    i2cPushCommands(device);

    uint32_t mier = LPI2C_MIER_SDIE_MASK | LPI2C_MIER_NDIE_MASK | LPI2C_MIER_ALIE_MASK
                  | LPI2C_MIER_FEIE_MASK | LPI2C_MIER_PLTIE_MASK;
    if (reading) {
        mier |= LPI2C_MIER_RDIE_MASK;
    }
    if (!i2cAllQueued(t)) {
        mier |= LPI2C_MIER_TDIE_MASK;
    }
    lpi2c->MIER = mier;

    return true;
}

FAST_CODE bool i2cBusy(i2cDevice_e device, bool *error)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        if (error) {
            *error = true;
        }
        return false;
    }

    i2cTransfer_t *t = &i2cTransfers[device];

    const timeUs_t now = microsISR();

    if ((t->recover && !t->busy && cmpTimeUs(now, t->recoverUs) >= I2C_RECOVER_INTERVAL_US)
        || (t->busy && cmpTimeUs(now, t->startUs) >= (timeDelta_t)I2C_TIMEOUT_US)) {
        i2cRecover(device);
    }

    // Busy first: every end writes error before it clears busy.
    const bool busy = t->busy;

    if (error) {
        *error = t->error;
    }

    return busy;
}

// i2cBusy() gives up after I2C_TIMEOUT_US.
static bool i2cWait(i2cDevice_e device)
{
    bool error = false;

    while (i2cBusy(device, &error)) {
    }

    return !error;
}

bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    return i2cStart(device, addr, reg, len, data, false, false);
}

bool i2cWrite(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t data)
{
    // One byte per bus, since a timed out transfer may still read it while another bus writes.
    static uint8_t byte[I2CDEV_COUNT];

    if (device == I2CINVALID || device >= I2CDEV_COUNT || i2cBusy(device, NULL)) {
        return false;
    }

    byte[device] = data;

    return i2cStart(device, addr, reg, 1, &byte[device], false, true) && i2cWait(device);
}

bool i2cReadBuffer(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return i2cStart(device, addr, reg, len, buf, true, false);
}

bool i2cRead(i2cDevice_e device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return i2cStart(device, addr, reg, len, buf, true, true) && i2cWait(device);
}

// One vector per instance for transmit, receive and errors.
#ifdef USE_I2C_DEVICE_1
FAST_IRQ_HANDLER void LPI2C1_IRQHandler(void)
{
    i2cIrqHandler(I2CDEV_1);
}
#endif

#ifdef USE_I2C_DEVICE_2
FAST_IRQ_HANDLER void LPI2C2_IRQHandler(void)
{
    i2cIrqHandler(I2CDEV_2);
}
#endif

#ifdef USE_I2C_DEVICE_3
FAST_IRQ_HANDLER void LPI2C3_IRQHandler(void)
{
    i2cIrqHandler(I2CDEV_3);
}
#endif

#ifdef USE_I2C_DEVICE_4
FAST_IRQ_HANDLER void LPI2C4_IRQHandler(void)
{
    i2cIrqHandler(I2CDEV_4);
}
#endif

#endif // defined(USE_I2C) && !defined(USE_SOFT_I2C)
