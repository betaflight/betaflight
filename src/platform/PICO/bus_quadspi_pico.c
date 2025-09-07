/*
 * PICO RP2350 QUADSPI adapter implementing Betaflight quadSpi API via Pico SDK QMI controller.
 */

#include "platform.h"

#ifdef PICO
#ifdef USE_QUADSPI

#include <string.h>

#include "drivers/bus_quadspi.h"
#include "drivers/bus_quadspi_impl.h"
#include "pg/bus_quadspi.h"
#include "drivers/io.h"

#include "hardware/flash.h"
#include "pico/bootrom.h"
#include "hardware/regs/qmi.h"
#include "hardware/structs/qmi.h"
#include "hardware/xip_cache.h"
#include "drivers/time.h"

// Provide platform QUADSPI hardware table for RP2350 (single logical device)
const quadSpiHardware_t quadSpiHardware[QUADSPIDEV_COUNT] = {
    {
        .device = QUADSPIDEV_1,
        .reg = (QUADSPI_TypeDef *)0x1, // sentinel; common layer uses it only for identity
        .clkPins = { { 0 } },
        .bk1IO0Pins = { { 0 } },
        .bk1IO1Pins = { { 0 } },
        .bk1IO2Pins = { { 0 } },
        .bk1IO3Pins = { { 0 } },
        .bk1CSPins  = { { 0 } },
        .bk2IO0Pins = { { 0 } },
        .bk2IO1Pins = { { 0 } },
        .bk2IO2Pins = { { 0 } },
        .bk2IO3Pins = { { 0 } },
        .bk2CSPins  = { { 0 } },
    },
};

// Pin configure is a no-op on RP2350, QMI pins are fixed and controlled by ROM/QMI
void quadSpiPinConfigure(const quadSpiConfig_t *pConfig)
{
    UNUSED(pConfig);
    quadSpiDevice[QUADSPIDEV_1].dev = quadSpiHardware[0].reg;
}

static inline void pico_qspi_enter_cmd_mode(void)
{
    // Ensure XIP cache is clean before exiting XIP
    xip_cache_clean_all();
    rom_connect_internal_flash_fn connect_fn = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn exit_fn = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    connect_fn();
    exit_fn();
}

static inline void pico_qspi_exit_cmd_mode(void)
{
    flash_flush_cache();
}

// QMI direct helpers for CS1 transactions
static inline void qmi_direct_enable(void) { hw_set_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS); }
static inline void qmi_direct_disable(void) { hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS); }
static inline void qmi_cs1_assert(bool asserted) {
    if (asserted) hw_set_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_ASSERT_CS1N_BITS);
    else hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_ASSERT_CS1N_BITS);
}

static inline void qmi_timeout_cleanup(void)
{
    qmi_cs1_assert(false);
    qmi_direct_disable();
    pico_qspi_exit_cmd_mode();
}

// Default timeout for QSPI direct IO operations
#define QSPI_TIMEOUT_MS 10

// Number of dummy clock cycles per consumed byte in direct-mode loops.
// Justification: QSPI datasheets specify dummy cycles in bits. Our direct-mode
// helper consumes dummy time by shifting out whole bytes (8 bits at a time).
// We therefore convert from bits to bytes using 8 bits/byte. If a controller
// revision were to support sub-byte dummy handling, this could be revisited.
#define QSPI_DUMMY_BITS_PER_BYTE 8

static inline bool qmi_direct_io(const uint8_t *tx, uint8_t *rx, size_t count, uint32_t timeoutMs)
{
    size_t tx_remaining = count;
    size_t rx_remaining = count;
    // Prevent hangs: break out after timeout in ms
    const timeMs_t start_ms = millis();
    while (rx_remaining) {
        uint32_t flags = qmi_hw->direct_csr;
        bool can_put = !(flags & QMI_DIRECT_CSR_TXFULL_BITS);
        bool can_get = !(flags & QMI_DIRECT_CSR_RXEMPTY_BITS);

        if (can_put && tx_remaining) {
            qmi_hw->direct_tx = tx ? *tx++ : 0x00;
            --tx_remaining;
        }
        if (can_get && rx_remaining) {
            uint8_t b = (uint8_t)qmi_hw->direct_rx;
            if (rx) {
                *rx++ = b;
            }
            --rx_remaining;
        }
        if (cmpTimeMs(millis(), start_ms) > (int32_t)timeoutMs) {
            return false; // timeout; caller will deassert CS and disable direct mode
        }
    }
    return true;
}

bool quadSpiTransmit1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{
    UNUSED(instance);
    UNUSED(dummyCycles);
    uint8_t txbuf[1 + 0];
    txbuf[0] = instruction;

    pico_qspi_enter_cmd_mode();
    qmi_direct_enable();
    qmi_cs1_assert(true);
    // Send instruction
    if (!qmi_direct_io(txbuf, NULL, 1, QSPI_TIMEOUT_MS)) {
        qmi_timeout_cleanup();
        return false;
    }
    // Send payload (no readback)
    if (out && length > 0) {
        if (!qmi_direct_io(out, NULL, (size_t)length, QSPI_TIMEOUT_MS)) {
            qmi_timeout_cleanup();
            return false;
        }
    }
    qmi_cs1_assert(false);
    qmi_direct_disable();
    pico_qspi_exit_cmd_mode();
    return true;
}

bool quadSpiReceive1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    UNUSED(instance);
    uint8_t txbuf[1];
    txbuf[0] = instruction;

    pico_qspi_enter_cmd_mode();
    qmi_direct_enable();
    qmi_cs1_assert(true);
    // Send instruction
    if (!qmi_direct_io(txbuf, NULL, 1, QSPI_TIMEOUT_MS)) {
        qmi_timeout_cleanup();
        return false;
    }
    // Consume dummy cycles as bytes
    // Convert the number of dummy cycles into a number of bytes to transfer
    uint8_t dummyBytes = (dummyCycles + QSPI_DUMMY_BITS_PER_BYTE - 1) / QSPI_DUMMY_BITS_PER_BYTE;
    if (!qmi_direct_io(NULL, NULL, dummyBytes, QSPI_TIMEOUT_MS)) {
        qmi_timeout_cleanup();
        return false;
    }
    // Read response by clocking out zeros
    if (!qmi_direct_io(NULL, in, length, QSPI_TIMEOUT_MS)) {
        qmi_timeout_cleanup();
        return false;
    }

    qmi_cs1_assert(false);
    qmi_direct_disable();
    pico_qspi_exit_cmd_mode();
    return true;
}

bool quadSpiInstructionWithData1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{
    return quadSpiTransmit1LINE(instance, instruction, dummyCycles, out, length);
}

bool quadSpiInstructionWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles,
                                        uint32_t address, uint8_t addressSize)
{
    UNUSED(instance);
    // Build header [cmd][addr]
    uint8_t hdr[1 + 4];
    int idx = 0;
    hdr[idx++] = instruction;
    for (int i = (addressSize / 8) - 1; i >= 0; i--) {
        hdr[idx++] = (address >> (i * 8)) & 0xFF;
    }

    pico_qspi_enter_cmd_mode();
    qmi_direct_enable();
    qmi_cs1_assert(true);
    if (!qmi_direct_io(hdr, NULL, (size_t)idx, QSPI_TIMEOUT_MS)) {
        qmi_timeout_cleanup();
        return false;
    }
    // Consume dummy cycles (if any)
    uint8_t dummyBytes = (dummyCycles + QSPI_DUMMY_BITS_PER_BYTE - 1) / QSPI_DUMMY_BITS_PER_BYTE;
    if (dummyBytes) {
        if (!qmi_direct_io(NULL, NULL, dummyBytes, QSPI_TIMEOUT_MS)) {
            qmi_timeout_cleanup();
            return false;
        }
    }
    qmi_cs1_assert(false);
    qmi_direct_disable();
    pico_qspi_exit_cmd_mode();
    return true;
}

static int buildHeader(uint8_t *hdr, uint8_t instruction, uint32_t address, uint8_t addressSize)
{
    // Build tx: [cmd][addr bytes MSB..LSB][dummy]
    int idx = 0;
    hdr[idx++] = instruction;
    for (int i = (addressSize/8) - 1; i >= 0; i--) {
        hdr[idx++] = (address >> (i*8)) & 0xFF;
    }

    return idx;
}

bool quadSpiReceiveWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles,
                                    uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    UNUSED(instance);
    // Build header and perform in one CS window
    uint8_t hdr[1 + 4];

    int idx = buildHeader(hdr, instruction, address, addressSize);

    pico_qspi_enter_cmd_mode();
    qmi_direct_enable();
    qmi_cs1_assert(true);
    if (!qmi_direct_io(hdr, NULL, (size_t)idx, QSPI_TIMEOUT_MS)) {
        qmi_timeout_cleanup();
        return false;
    }
    // Dummy after header (if any)
    uint8_t dummyBytes = (dummyCycles + QSPI_DUMMY_BITS_PER_BYTE - 1) / QSPI_DUMMY_BITS_PER_BYTE;
    if (dummyBytes) {
        if (!qmi_direct_io(NULL, NULL, dummyBytes, QSPI_TIMEOUT_MS)) {
            qmi_timeout_cleanup();
            return false;
        }
    }
    // Read payload
    if (!qmi_direct_io(NULL, in, length, QSPI_TIMEOUT_MS)) {
        qmi_timeout_cleanup();
        return false;
    }
    qmi_cs1_assert(false);
    qmi_direct_disable();
    pico_qspi_exit_cmd_mode();
    return true;
}

bool quadSpiTransmitWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles,
                                     uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    UNUSED(instance);
    uint8_t hdr[1 + 4];

    int idx = buildHeader(hdr, instruction, address, addressSize);

    pico_qspi_enter_cmd_mode();
    qmi_direct_enable();
    qmi_cs1_assert(true);
    if (!qmi_direct_io(hdr, NULL, (size_t)idx, QSPI_TIMEOUT_MS)) {
        qmi_timeout_cleanup();
        return false;
    }
    // Dummy after header (if any)
    uint8_t dummyBytes = (dummyCycles + QSPI_DUMMY_BITS_PER_BYTE - 1) / QSPI_DUMMY_BITS_PER_BYTE;
    if (dummyBytes) {
        if (!qmi_direct_io(NULL, NULL, dummyBytes, QSPI_TIMEOUT_MS)) {
            qmi_timeout_cleanup();
            return false;
        }
    }
    if (out && length > 0) {
        if (!qmi_direct_io(out, NULL, (size_t)length, QSPI_TIMEOUT_MS)) {
            qmi_timeout_cleanup();
            return false;
        }
    }
    qmi_cs1_assert(false);
    qmi_direct_disable();
    pico_qspi_exit_cmd_mode();
    return true;
}

// 4LINE operations are not truly available via ROM command mode on QMI; emulate using 1LINE for now.
bool quadSpiReceive4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    uint8_t instr = (instruction == 0x6B) ? 0x0B : instruction; // fast quad->fast serial
    return quadSpiReceive1LINE(instance, instr, dummyCycles, in, length);
}

bool quadSpiReceiveWithAddress4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles,
                                     uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    uint8_t instr = (instruction == 0x6B) ? 0x0B : instruction; // map to serial fast read
    return quadSpiReceiveWithAddress1LINE(instance, instr, dummyCycles, address, addressSize, in, length);
}

bool quadSpiTransmitWithAddress4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles,
                                      uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    uint8_t instr = (instruction == 0x32) ? 0x02 : instruction; // quad page prog -> serial page prog
    return quadSpiTransmitWithAddress1LINE(instance, instr, dummyCycles, address, addressSize, out, length);
}

void quadSpiSetDivisor(QUADSPI_TypeDef *instance, uint16_t divisor)
{
    UNUSED(instance);
    UNUSED(divisor);
    // Pico ROM manages clocking for direct command mode; leave as-is.
}

bool quadSpiInit(quadSpiDevice_e device);
void quadSpiPreInit(void) {}

void quadSpiInitDevice(quadSpiDevice_e device)
{
    UNUSED(device);
    // Configure CS1 GPIO and size for external flash via FLASH_DEVINFO so ROM/QMI know about it.
#if PICO_RP2350
    // GPIO index for CS1 (bank0); default to 0 if not provided by target config
#ifdef PICO_QSPI_CS1_GPIO
    const uint cs1_gpio = PICO_QSPI_CS1_GPIO;
#else
    const uint cs1_gpio = 0u;
#endif
    // External flash size in bytes; default to 0 (none) if not provided
#ifdef PICO_QSPI_CS1_SIZE_BYTES
    const uint32_t cs1_size_bytes = PICO_QSPI_CS1_SIZE_BYTES;
#else
    const uint32_t cs1_size_bytes = 0u;
#endif

    if (cs1_gpio < NUM_BANK0_GPIOS) {
        flash_devinfo_set_cs_gpio(1, cs1_gpio);
    }
    flash_devinfo_set_cs_size(1, flash_devinfo_bytes_to_size(cs1_size_bytes));
    // Winbond W25Q series support 64k (D8h) erase; enable to speed up erases if present
    flash_devinfo_set_d8h_erase_supported(true);
#endif
}

#endif // USE_QUADSPI
#endif // PICO


