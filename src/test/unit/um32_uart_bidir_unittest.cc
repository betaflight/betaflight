/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <cstring>

extern "C" {
#include "um32_uart_bidir/platform.h"
#include "build/debug.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/serial.h"
#include "drivers/serial_impl.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "platform/serial_uart_bidir.h"
void LPTIMER1_IRQHandler(void);

mockLptim_t mockLptim;
uartDevice_t uartDevice[UARTDEV_COUNT];
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;
}

#include "gtest/gtest.h"

namespace {
uartHardware_t hardware[UARTDEV_COUNT];
UART_TypeDef registers[UARTDEV_COUNT];
ioConfig_t pinConfig[64];
ioRec_t pinRecords[64];
uint32_t now;
uint32_t clockHz;

class Um32UartBidirTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(uartDevice, 0, sizeof(uartDevice));
        memset(hardware, 0, sizeof(hardware));
        memset(registers, 0, sizeof(registers));
        memset(pinConfig, 0, sizeof(pinConfig));
        memset(pinRecords, 0, sizeof(pinRecords));
        memset(debug, 0, sizeof(debug));
        mockLptim = {};
        now = 0;
        clockHz = 288000000;
        debugMode = DEBUG_UART_BIDIR;
        for (unsigned i = 0; i < UARTDEV_COUNT; i++) {
            const serialPortIdentifier_e id = i == UARTDEV_1 ? SERIAL_PORT_UART1
                : (i == UARTDEV_2 ? SERIAL_PORT_UART2 : SERIAL_PORT_UART8);
            const unsigned tx = i == UARTDEV_1 ? 6 : (i == UARTDEV_2 ? 16 : 26);
            hardware[i].af = 7;
            hardware[i].rxPins[0].pin = tx + 1;
            uartDevice[i].hardware = &hardware[i];
            uartDevice[i].tx.pin = tx;
            uartDevice[i].rx.pin = i == UARTDEV_1 ? 10 : tx + 1;
            uartDevice[i].port.port.identifier = id;
            uartDevice[i].port.port.baudRate = 4800;
            uartDevice[i].port.USARTx = reinterpret_cast<usartResource_t *>(&registers[i]);
            uartBidirConfigure(&uartDevice[i].port); // Clear any preceding test's binding/fault.
        }
    }

    uartPort_t *open(uartDeviceIdx_e index = UARTDEV_1, portOptions_e options = SERIAL_BIDIR, uint32_t baud = 4800)
    {
        uartPort_t *s = &uartDevice[index].port;
        s->port.options = options;
        s->port.baudRate = baud;
        uartBidirConfigure(s);
        return s;
    }

    void expectReceive(unsigned tx, ioConfig_t pull)
    {
        EXPECT_EQ(pull, pinConfig[tx]);
        EXPECT_EQ(pull | IOCFG_AF_PP, pinConfig[tx + 1]);
    }
};

TEST_F(Um32UartBidirTest, WaitsForShiftRegisterEmptyAndReleasesWithoutAPrefix)
{
    auto *s = open();
    expectReceive(6, IOCFG_IPU);
    EXPECT_EQ(IOCFG_IN_FLOATING, pinConfig[10]);
    ASSERT_TRUE(uartBidirStart(s));
    EXPECT_EQ(IOCFG_AF_PP_UP, pinConfig[6]);
    EXPECT_EQ(IOCFG_IN_FLOATING, pinConfig[7]);
    EXPECT_TRUE(uartBidirIsTransmitting(s));
    uartBidirDrain(s);
    EXPECT_NE(0U, mockLptim.IE);
    now = 100;
    LPTIMER1_IRQHandler(); // FIFO empty alone does not mean TEMT.
    EXPECT_TRUE(uartBidirIsTransmitting(s));
    registers[UARTDEV_1].LSR = UART_EX_LSR_TEMT;
    LPTIMER1_IRQHandler();
    expectReceive(6, IOCFG_IPU);
    EXPECT_FALSE(uartBidirIsTransmitting(s));
    EXPECT_EQ(0U, mockLptim.IE);
}

TEST_F(Um32UartBidirTest, QueuedDataAndNewTransmissionCancelPendingRelease)
{
    auto *s = open();
    uartBidirStart(s);
    uartBidirDrain(s);
    s->port.txBufferHead = 1;
    registers[UARTDEV_1].LSR = UART_EX_LSR_TEMT;
    LPTIMER1_IRQHandler();
    EXPECT_TRUE(uartBidirIsTransmitting(s));
    s->port.txBufferTail = 1;
    uartBidirDrain(s);
    EXPECT_FALSE(uartBidirIsTransmitting(s));
    registers[UARTDEV_1].LSR = 0;
    uartBidirStart(s);
    uartBidirDrain(s);
    uartBidirStart(s);
    registers[UARTDEV_1].LSR = UART_EX_LSR_TEMT;
    LPTIMER1_IRQHandler();
    EXPECT_TRUE(uartBidirIsTransmitting(s));
    uartBidirDrain(s);
    expectReceive(6, IOCFG_IPU);
}

TEST_F(Um32UartBidirTest, StuckTemtReleasesBusAndReportsFaultUntilReconfigured)
{
    auto *s = open();
    const int previousCount = debug[1];
    const int expectedCount = previousCount < INT16_MAX ? previousCount + 1 : INT16_MAX;
    uartBidirStart(s);
    uartBidirDrain(s);
    now = 10000;
    LPTIMER1_IRQHandler();
    expectReceive(6, IOCFG_IPU);
    EXPECT_FALSE(uartBidirStart(s));
    EXPECT_EQ(1, debug[0]);
    EXPECT_EQ(expectedCount, debug[1]);
    EXPECT_EQ(1, debug[2]);
    uartBidirConfigure(s);
    EXPECT_EQ(expectedCount, debug[1]); // Diagnostic survives reconfiguration.
    EXPECT_EQ(0, debug[2]);
    EXPECT_TRUE(uartBidirStart(s));
    registers[UARTDEV_1].LSR = 0;
    uartBidirDrain(s);
    now += 10000;
    LPTIMER1_IRQHandler();
    EXPECT_EQ(1, debug[2]);
    s->port.options = SERIAL_UNIDIR;
    uartBidirConfigure(s);
    EXPECT_EQ(0, debug[2]); // Disabling BIDIR also clears the displayed latch.
}

TEST_F(Um32UartBidirTest, TxDmaFallsBackToInterruptsOnlyForAValidDualPadPort)
{
    auto *s = &uartDevice[UARTDEV_1].port;
    auto *dma = reinterpret_cast<dmaResource_t *>(1);
    s->txDMAResource = dma;
    s->rxDMAResource = dma;
    open();
    EXPECT_EQ(nullptr, s->txDMAResource);
    EXPECT_EQ(dma, s->rxDMAResource);
    EXPECT_TRUE(uartBidirStart(s));
    EXPECT_EQ(IOCFG_AF_PP_UP, pinConfig[6]);
    s->txDMAResource = dma;
    open(UARTDEV_1, SERIAL_UNIDIR);
    EXPECT_EQ(dma, s->txDMAResource); // Ordinary UART DMA is untouched.
    hardware[UARTDEV_1].rxPins[0].pin = 3;
    open();
    EXPECT_EQ(dma, s->txDMAResource); // Invalid RX mapping never owns this port.
    EXPECT_FALSE(uartBidirIsTransmitting(s));
}

TEST_F(Um32UartBidirTest, OnlyAcceptsUsableHalfDuplexMappings)
{
    auto &dev = uartDevice[UARTDEV_1];
    EXPECT_TRUE(uartBidirCanOpen(&dev, 4800));
    EXPECT_FALSE(uartBidirCanOpen(&dev, 0));

    dev.tx.pin = 7;
    EXPECT_FALSE(uartBidirCanOpen(&dev, 4800)); // Same pad for TX and RX.
    dev.tx.pin = 0;
    EXPECT_FALSE(uartBidirCanOpen(&dev, 4800));
    dev.tx.pin = 6;

    hardware[UARTDEV_1].rxPins[0].pin = 10;
    EXPECT_FALSE(uartBidirCanOpen(&dev, 4800)); // Alternate pad absent from UART table.
    hardware[UARTDEV_1].rxPins[0].pin = 7;

    dev.port.port.identifier = SERIAL_PORT_UART3;
    EXPECT_FALSE(uartBidirCanOpen(&dev, 4800)); // No alternate pad defined for UART3.
}

TEST_F(Um32UartBidirTest, RejectsAnAlternateRxPadOwnedByAnotherResource)
{
    const auto rx = IOGetByTag(7);
    const resourceOwner_t conflicts[] = {{OWNER_I2C_SDA, 1}, {OWNER_SERIAL_RX, 2}, {OWNER_PREINIT, 0}};
    for (const auto &conflict : conflicts) {
        IOInit(rx, conflict.owner, conflict.index);
        IOConfigGPIOAF(rx, IOCFG_AF_PP_PD, 4);

        EXPECT_FALSE(uartBidirCanOpen(&uartDevice[UARTDEV_1], 4800));
        open(); // Also check the reconfiguration path without the open guard.

        EXPECT_EQ(conflict.owner, IO_Rec(rx)->owner);
        EXPECT_EQ(conflict.index, IO_Rec(rx)->index);
        EXPECT_EQ(IOCFG_AF_PP_PD, pinConfig[7]);
    }
}

TEST_F(Um32UartBidirTest, ReconfigureOnlyTouchesRxPadsOwnedByThisUart)
{
    const auto rx = IOGetByTag(7);
    const auto normalRx = IOGetByTag(10);
    IOInit(normalRx, OWNER_I2C_SDA, 1);
    IOConfigGPIOAF(normalRx, IOCFG_AF_PP_PD, 4);

    auto *s = open();
    ASSERT_EQ(OWNER_SERIAL_RX, IO_Rec(rx)->owner);
    EXPECT_EQ(1, IO_Rec(rx)->index);
    EXPECT_EQ(IOCFG_AF_PP_PD, pinConfig[10]);
    ASSERT_TRUE(uartBidirCanOpen(&uartDevice[UARTDEV_1], 9600));

    s->port.baudRate = 9600;
    uartBidirConfigure(s);
    EXPECT_EQ(OWNER_SERIAL_RX, IO_Rec(rx)->owner);
    expectReceive(6, IOCFG_IPU);

    IOInit(rx, OWNER_I2C_SDA, 2);
    IOConfigGPIOAF(rx, IOCFG_AF_PP_PD, 4);
    uartBidirConfigure(s);
    EXPECT_EQ(OWNER_I2C_SDA, IO_Rec(rx)->owner);
    EXPECT_EQ(2, IO_Rec(rx)->index);
    EXPECT_EQ(IOCFG_AF_PP_PD, pinConfig[7]);
}

TEST_F(Um32UartBidirTest, SparseDeviceIndexesAndSamUsartUseTheCorrectStateAndRegisters)
{
    static_assert(UARTDEV_COUNT == 3, "Exercise compressed indexes with UART3..7 absent");
    USART_TypeDef sam = {};
    auto *s = &uartDevice[UARTDEV_8].port;
    s->USARTx = reinterpret_cast<usartResource_t *>(&sam);
    open(UARTDEV_8, SERIAL_BIDIR, 57600);
    uartBidirStart(s);
    uartBidirDrain(s);
    EXPECT_TRUE(uartBidirIsTransmitting(s));
    sam.CSR = USART_CSR_TXEMPTY;
    LPTIMER1_IRQHandler();
    expectReceive(26, IOCFG_IPU);
    EXPECT_FALSE(uartBidirIsTransmitting(s));
    uartPort_t invalid = {};
    invalid.port.identifier = SERIAL_PORT_NONE;
    uartBidirConfigure(&invalid);
    EXPECT_TRUE(uartBidirStart(&invalid));
    EXPECT_FALSE(uartBidirIsTransmitting(&invalid));
}

TEST_F(Um32UartBidirTest, ConcurrentBaudRatesDoNotPrematurelyTimeoutTheSlowPort)
{
    auto *slow = open();
    auto *fast = open(UARTDEV_2, SERIAL_BIDIR, 115200);
    uartBidirStart(slow);
    uartBidirDrain(slow);
    const uint32_t slowTarget = mockLptim.TARGET;
    uartBidirStart(fast);
    uartBidirDrain(fast);
    EXPECT_LT(mockLptim.TARGET, slowTarget);
    for (int i = 0; i < 20; i++) { now += 10; LPTIMER1_IRQHandler(); }
    EXPECT_TRUE(uartBidirIsTransmitting(slow));
    EXPECT_TRUE(uartBidirIsTransmitting(fast));
    registers[UARTDEV_2].LSR = UART_EX_LSR_TEMT;
    LPTIMER1_IRQHandler();
    EXPECT_FALSE(uartBidirIsTransmitting(fast));
    EXPECT_TRUE(uartBidirIsTransmitting(slow));
}

TEST_F(Um32UartBidirTest, DeadlineWraparoundUsesElapsedTime)
{
    auto *s = open();
    uartBidirStart(s);
    now = UINT32_MAX - 15;
    uartBidirDrain(s);
    now = 20;
    LPTIMER1_IRQHandler();
    EXPECT_TRUE(uartBidirIsTransmitting(s));
    now = 10000;
    LPTIMER1_IRQHandler();
    EXPECT_FALSE(uartBidirStart(s));
}

TEST_F(Um32UartBidirTest, ReportsLineErrorsOnce)
{
    auto *s = open();
    uartBidirStart(s);
    registers[UARTDEV_1].LSR = UART_EX_LSR_OE | UART_EX_LSR_FE;
    uartBidirDrain(s);
    EXPECT_EQ(UART_EX_LSR_OE | UART_EX_LSR_FE, uartBidirTakeErrors(s));
    EXPECT_EQ(0U, uartBidirTakeErrors(s));
}

TEST_F(Um32UartBidirTest, TimerRateTracksBothSupportedClocks)
{
    auto *s = open();
    for (const uint32_t hz : {288000000U, 336000000U}) {
        clockHz = hz;
        registers[UARTDEV_1].LSR = 0;
        uartBidirStart(s);
        uartBidirDrain(s);
        EXPECT_EQ(((hz / 128) * 100 + 999999) / 1000000, mockLptim.TARGET);
        registers[UARTDEV_1].LSR = UART_EX_LSR_TEMT;
        LPTIMER1_IRQHandler();
    }
}

TEST_F(Um32UartBidirTest, TimeoutCounterSaturatesInsteadOfWrappingNegative)
{
    auto *s = open();
    for (unsigned i = 0; i <= INT16_MAX; i++) {
        uartBidirConfigure(s);
        uartBidirStart(s);
        uartBidirDrain(s);
        now += 10000;
        LPTIMER1_IRQHandler();
    }
    EXPECT_EQ(INT16_MAX, debug[1]);
    EXPECT_EQ(1, debug[2]);
}

TEST_F(Um32UartBidirTest, BoardBiasPreservesSmartAudioTrampExplicitPullAndInversion)
{
    auto *s = open(UARTDEV_1, SERIAL_BIDIR, 9600); // Tramp: no option/frame changes.
    for (int i = 0; i < 3; i++) {
        registers[UARTDEV_1].LSR = 0;
        uartBidirStart(s);
        uartBidirDrain(s);
        registers[UARTDEV_1].LSR = UART_EX_LSR_TEMT;
        LPTIMER1_IRQHandler();
        expectReceive(6, IOCFG_IPU);
    }
    EXPECT_EQ(SERIAL_BIDIR, s->port.options);
    EXPECT_EQ(9600U, s->port.baudRate);
#ifdef USE_SMARTAUDIO_NOPULLDOWN
    const ioConfig_t saPull = IOCFG_IN_FLOATING;
#else
    const ioConfig_t saPull = IOCFG_IPD;
#endif
    open(UARTDEV_1, static_cast<portOptions_e>(SERIAL_BIDIR | SERIAL_PULL_SMARTAUDIO | SERIAL_STOPBITS_2));
#ifdef UART1_BIDIR_PULLUP
    expectReceive(6, IOCFG_IPU);
#else
    expectReceive(6, saPull);
#endif
    open(UARTDEV_2, static_cast<portOptions_e>(SERIAL_BIDIR | SERIAL_PULL_SMARTAUDIO));
    expectReceive(16, saPull); // No UART1 board bias on UART2.
    open(UARTDEV_1, static_cast<portOptions_e>(SERIAL_BIDIR | SERIAL_PULL_NONE));
    expectReceive(6, IOCFG_IN_FLOATING);
#ifdef UART1_BIDIR_PULLUP
    open(UARTDEV_1, static_cast<portOptions_e>(SERIAL_BIDIR | SERIAL_PULL_NONE | SERIAL_PULL_SMARTAUDIO));
    expectReceive(6, IOCFG_IN_FLOATING);
#endif
    open(UARTDEV_1, static_cast<portOptions_e>(SERIAL_BIDIR | SERIAL_INVERTED));
    expectReceive(6, IOCFG_IPD);
}
}

extern "C" {
uartDeviceIdx_e uartDeviceIdxFromIdentifier(serialPortIdentifier_e id)
{
    switch (id) {
    case SERIAL_PORT_UART1: return UARTDEV_1;
    case SERIAL_PORT_UART2: return UARTDEV_2;
    case SERIAL_PORT_UART8: return UARTDEV_8;
    default: return UARTDEV_INVALID;
    }
}
uartDevice_t *uartDeviceFromIdentifier(serialPortIdentifier_e id)
{
    const auto index = uartDeviceIdxFromIdentifier(id);
    return index == UARTDEV_INVALID ? nullptr : &uartDevice[index];
}
IO_t IOGetByTag(ioTag_t tag) { return reinterpret_cast<IO_t>(static_cast<uintptr_t>(tag)); }
void IOConfigGPIO(IO_t io, ioConfig_t config) { pinConfig[reinterpret_cast<uintptr_t>(io)] = config; }
void IOConfigGPIOAF(IO_t io, ioConfig_t config, uint8_t) { IOConfigGPIO(io, config); }
ioRec_t *IO_Rec(IO_t io) { return io ? &pinRecords[reinterpret_cast<uintptr_t>(io)] : nullptr; }
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index)
{
    if (auto *rec = IO_Rec(io)) {
        rec->owner = owner;
        rec->index = index;
    }
}
resourceOwner_e serialOwnerTxRx(serialPortIdentifier_e) { return OWNER_SERIAL_TX; }
int serialOwnerIndex(serialPortIdentifier_e id) { return id - SERIAL_PORT_UART1 + 1; }
uint32_t micros(void) { return now; }
uint32_t HAL_RCM_GetPCLK0Freq(void) { return clockHz; }
void HAL_NVIC_SetPriority(int, int, int) {}
void HAL_NVIC_EnableIRQ(int) {}
}
