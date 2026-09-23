/*
 * UART reconfiguration tests using the production IRQ-driven driver and a
 * register model. No UART/DMA hardware or wall-clock delays are required.
 */
#include <cstring>
#include "gtest/gtest.h"

extern "C" {
#include "platform.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"

struct USART_TypeDef_s { uint32_t CR1, CR3, ISR, RDR, TDR; };
#define USART_CR1_PEIE (1U << 0)
#define USART_CR1_TXEIE (1U << 1)
#define USART_CR1_TCIE (1U << 2)
#define USART_CR1_RXNEIE (1U << 3)
#define USART_CR1_IDLEIE (1U << 4)
#define USART_CR1_TE (1U << 5)
#define USART_CR1_RE (1U << 6)
#define USART_CR1_UE (1U << 7)
#define USART_CR1_M (1U << 8)
#define USART_CR1_PS (1U << 9)
#define USART_CR3_EIE 1U
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
#define SET_BIT(reg, bits) ((reg) |= (bits))
#define LL_USART_DATAWIDTH_9B 9
#define LL_USART_DATAWIDTH_8B 8
#define LL_USART_STOPBITS_2 2
#define LL_USART_STOPBITS_1 1
#define LL_USART_PARITY_EVEN 2
#define LL_USART_PARITY_NONE 0
#define LL_USART_HWCONTROL_NONE 0
#define LL_USART_OVERSAMPLING_16 16
#define LL_USART_DIRECTION_RX USART_CR1_RE
#define LL_USART_DIRECTION_TX USART_CR1_TE
#define LL_USART_RXPIN_LEVEL_INVERTED 1
#define LL_USART_TXPIN_LEVEL_INVERTED 1
#define SUCCESS 1
typedef int ErrorStatus;
struct LL_USART_InitTypeDef { uint32_t BaudRate, DataWidth, StopBits, Parity, HardwareFlowControl, OverSampling, TransferDirection; };

static bool irqEnabled, irqPending, initFails, writeDuringInit;
static uint32_t nowUs, resets, configuredBaud;
static bool resetWhileBusy, resetWithIrqEnabled;
static uartPort_t *observedPort;
static uint32_t NVIC_GetEnableIRQ(IRQn_Type) { return irqEnabled; }
static void NVIC_DisableIRQ(IRQn_Type) { irqEnabled = false; }
static void NVIC_EnableIRQ(IRQn_Type) { irqEnabled = true; }
static void NVIC_ClearPendingIRQ(IRQn_Type) { irqPending = false; }
static void __DSB() {}
static void __ISB() {}

#define FLAG_TC (1U << 0)
#define FLAG_TXE (1U << 1)
#define FLAG_RXNE (1U << 2)
#define FLAG_PE (1U << 3)
#define FLAG_FE (1U << 4)
#define FLAG_NE (1U << 5)
#define FLAG_ORE (1U << 6)
#define FLAG_IDLE (1U << 7)
#define MOCK_FLAG(name) \
    static bool LL_USART_IsActiveFlag_##name(USART_TypeDef *u) { return u->ISR & FLAG_##name; } \
    static void LL_USART_ClearFlag_##name(USART_TypeDef *u) { u->ISR &= ~FLAG_##name; }
MOCK_FLAG(TC)
MOCK_FLAG(PE)
MOCK_FLAG(FE)
MOCK_FLAG(NE)
MOCK_FLAG(ORE)
MOCK_FLAG(IDLE)
static bool LL_USART_IsActiveFlag_TXE(USART_TypeDef *u) { return u->ISR & FLAG_TXE; }
static bool LL_USART_IsActiveFlag_RXNE(USART_TypeDef *u) { return u->ISR & FLAG_RXNE; }
#define MOCK_IT(name) \
    static bool LL_USART_IsEnabledIT_##name(USART_TypeDef *u) { return u->CR1 & USART_CR1_##name##IE; }
MOCK_IT(TC)
MOCK_IT(TXE)
MOCK_IT(RXNE)
MOCK_IT(IDLE)
static void LL_USART_DisableIT_TXE(USART_TypeDef *u) { u->CR1 &= ~USART_CR1_TXEIE; }
static void LL_USART_RequestRxDataFlush(USART_TypeDef *u) { u->ISR &= ~FLAG_RXNE; u->RDR = 0; }
static void LL_USART_Disable(USART_TypeDef *u) { u->CR1 &= ~USART_CR1_UE; }
static void LL_USART_Enable(USART_TypeDef *u) { u->CR1 |= USART_CR1_UE; u->ISR |= FLAG_TC; }
static void LL_USART_DeInit(USART_TypeDef *u)
{
    resets++;
    resetWithIrqEnabled |= irqEnabled;
    resetWhileBusy |= observedPort->txHardwareBusy && !(u->ISR & FLAG_TC);
    *u = {};
}
static void LL_USART_StructInit(LL_USART_InitTypeDef *i) { *i = {}; }
static int LL_USART_Init(USART_TypeDef *u, LL_USART_InitTypeDef *i)
{
    if (writeDuringInit) {
        serialWrite(&observedPort->port, 0x55);
        EXPECT_FALSE(u->CR1 & USART_CR1_TXEIE);
    }
    if (initFails) return 0;
    configuredBaud = i->BaudRate;
    u->CR1 = i->TransferDirection;
    return SUCCESS;
}
static void LL_USART_SetRXPinLevel(USART_TypeDef *, int) {}
static void LL_USART_SetTXPinLevel(USART_TypeDef *, int) {}
static void LL_USART_DisableOverrunDetect(USART_TypeDef *) {}
static void LL_USART_ConfigAsyncMode(USART_TypeDef *) {}
static void LL_USART_EnableHalfDuplex(USART_TypeDef *) {}

timeUs_t micros(void) { return nowUs; }
void schedulerIgnoreTaskExecTime(void) {}
IO_t IOGetByTag(ioTag_t) { return nullptr; }
bool IORead(IO_t) { return true; }
void IOConfigGPIO(IO_t, ioConfig_t) {}
void IOConfigGPIOAF(IO_t, ioConfig_t, uint8_t) {}
serialType_e serialType(serialPortIdentifier_e) { return SERIALTYPE_UART; }
uartPort_t *serialUART(uartDevice_t *, uint32_t, portMode_e, portOptions_e) { return nullptr; }
void uartEnableTxInterrupt(uartPort_t *s)
{
    ((USART_TypeDef *)s->USARTx)->CR1 |= USART_CR1_TXEIE;
}

#include "../../platform/STM32/serial_uart_ll.c"
}

class UartReconfigureTest : public ::testing::Test {
protected:
    USART_TypeDef regs{};
    uartDevice_t device{};
    uartHardware_t hardware{};
    uint8_t tx[32]{}, rx[32]{};
    uartPort_t &s = device.port;
    void SetUp() override
    {
        irqEnabled = true; irqPending = false; initFails = writeDuringInit = false;
        resets = nowUs = configuredBaud = 0;
        resetWhileBusy = resetWithIrqEnabled = false;
        observedPort = &s;
        device.hardware = &hardware;
        device.tx.pin = device.rx.pin = 1;
        device.txPinState = TX_PIN_IGNORE;
        s.USARTx = (usartResource_t *)&regs;
        s.port.vTable = uartVTable;
        s.port.baudRate = 9600;
        s.port.mode = MODE_RXTX;
        s.port.txBuffer = tx; s.port.txBufferSize = sizeof(tx);
        s.port.rxBuffer = rx; s.port.rxBufferSize = sizeof(rx);
        regs.CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_TCIE | USART_CR1_TXEIE;
    }
    void SendHardwareByte()
    {
        serialWrite(&s.port, 0x41);
        regs.ISR = FLAG_TXE;
        uartIrqHandler(&s);
        ASSERT_TRUE(s.txHardwareBusy);
        ASSERT_TRUE(isSerialTransmitBufferEmpty(&s.port));
    }
};

TEST_F(UartReconfigureTest, EmptySoftwareBufferDoesNotMeanHardwareDrained)
{
    SendHardwareByte();
    EXPECT_FALSE(serialTrySetBaudRate(&s.port, 115200));
    EXPECT_EQ(0U, resets);
    EXPECT_EQ(9600U, s.port.baudRate);
    EXPECT_TRUE(regs.CR1 & USART_CR1_TE);
    EXPECT_FALSE(regs.CR1 & USART_CR1_TXEIE);
    EXPECT_TRUE(irqEnabled);
    EXPECT_TRUE(s.txInhibited);
}

TEST_F(UartReconfigureTest, QueuedWritesCannotRestartTxAndResumeAfterDrain)
{
    SendHardwareByte();
    ASSERT_FALSE(serialTrySetBaudRate(&s.port, 115200));
    serialWrite(&s.port, 0x42);
    const uint8_t bytes[] = {0x43, 0x44};
    serialWriteBuf(&s.port, bytes, sizeof(bytes));
    EXPECT_FALSE(regs.CR1 & USART_CR1_TXEIE);
    // Even a stale TXE enable cannot make the ISR feed the inhibited UART.
    regs.CR1 |= USART_CR1_TXEIE;
    regs.ISR = FLAG_TXE;
    uartIrqHandler(&s);
    EXPECT_EQ(0x41U, regs.TDR);
    regs.ISR = FLAG_TC | FLAG_RXNE | FLAG_ORE | FLAG_IDLE;
    irqPending = true;
    ASSERT_TRUE(serialTrySetBaudRate(&s.port, 115200));
    EXPECT_EQ(1U, resets);
    EXPECT_EQ(115200U, configuredBaud);
    EXPECT_FALSE(resetWhileBusy);
    EXPECT_FALSE(resetWithIrqEnabled);
    EXPECT_FALSE(irqPending);
    EXPECT_FALSE(s.txInhibited);
    EXPECT_EQ(0U, regs.ISR);
    for (const auto byte : {0x42U, 0x43U, 0x44U}) {
        regs.ISR = FLAG_TXE;
        uartIrqHandler(&s);
        EXPECT_EQ(byte, regs.TDR);
    }
    EXPECT_TRUE(isSerialTransmitBufferEmpty(&s.port));
}

TEST_F(UartReconfigureTest, CompletionAlreadyHandledByInterruptStillAllowsReset)
{
    SendHardwareByte();
    ASSERT_FALSE(serialTrySetBaudRate(&s.port, 115200));
    regs.ISR = FLAG_TC;
    uartIrqHandler(&s);
    EXPECT_FALSE(s.txHardwareBusy);
    EXPECT_FALSE(regs.ISR & FLAG_TC);
    EXPECT_TRUE(serialTrySetBaudRate(&s.port, 115200));
}

TEST_F(UartReconfigureTest, TimeoutDisablesPortWithoutResetOrBusyWait)
{
    SendHardwareByte();
    nowUs = UINT32_MAX - 1000;
    ASSERT_FALSE(serialTrySetBaudRate(&s.port, 115200));
    nowUs += 3500; // two 12-bit frames at 9600 baud plus margin; wraps micros
    EXPECT_FALSE(serialTrySetBaudRate(&s.port, 115200));
    EXPECT_TRUE(s.reconfigureFailed);
    EXPECT_TRUE(s.txInhibited);
    EXPECT_EQ(0U, regs.CR1 & (USART_CR1_UE | USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE));
    EXPECT_EQ(0U, resets);
    EXPECT_TRUE(irqEnabled);
    regs.ISR = FLAG_TC;
    EXPECT_FALSE(serialTrySetBaudRate(&s.port, 115200));
}

TEST_F(UartReconfigureTest, PreservesDisabledNvicAndHandlesInitializationFailure)
{
    irqEnabled = false;
    initFails = true;
    EXPECT_FALSE(serialTrySetBaudRate(&s.port, 115200));
    EXPECT_FALSE(irqEnabled);
    EXPECT_TRUE(s.reconfigureFailed);
    EXPECT_TRUE(s.txInhibited);
    EXPECT_FALSE(regs.CR1 & USART_CR1_UE);
}

TEST_F(UartReconfigureTest, UnusedTransmitterAndUnchangedBaudNeedNoDrain)
{
    EXPECT_TRUE(serialTrySetBaudRate(&s.port, 9600));
    EXPECT_EQ(0U, resets);
    EXPECT_TRUE(serialTrySetBaudRate(&s.port, 115200));
    EXPECT_EQ(1U, resets);
    EXPECT_TRUE(irqEnabled);
    EXPECT_FALSE(s.txInhibited);
}

TEST_F(UartReconfigureTest, WriterDuringResetQueuesUntilPeripheralReady)
{
    writeDuringInit = true;
    ASSERT_TRUE(serialTrySetBaudRate(&s.port, 115200));
    EXPECT_FALSE(resetWithIrqEnabled);
    EXPECT_FALSE(regs.ISR & FLAG_TC);
    regs.ISR = FLAG_TXE;
    uartIrqHandler(&s);
    EXPECT_EQ(0x55U, regs.TDR);
}

TEST_F(UartReconfigureTest, InitFailureRestoresNvicButKeepsPeripheralQuiescent)
{
    initFails = true;
    ASSERT_FALSE(serialTrySetBaudRate(&s.port, 115200));
    EXPECT_TRUE(irqEnabled);
    EXPECT_TRUE(s.txInhibited);
    EXPECT_EQ(0U, regs.CR1);
    EXPECT_EQ(0U, regs.CR3);
    serialWrite(&s.port, 0x55);
    EXPECT_EQ(0U, regs.CR1);
}

TEST_F(UartReconfigureTest, PortWithoutOptionalMethodKeepsLegacyBaudChange)
{
    serialPortVTable legacy = {};
    legacy.serialSetBaudRate = [](serialPort_t *p, uint32_t baud) { p->baudRate = baud; };
    s.port.vTable = &legacy;
    EXPECT_TRUE(serialTrySetBaudRate(&s.port, 38400));
    EXPECT_EQ(38400U, s.port.baudRate);
}
