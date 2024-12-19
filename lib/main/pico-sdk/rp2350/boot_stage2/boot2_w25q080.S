// ----------------------------------------------------------------------------
// Second stage boot code
// Copyright (c) 2019-2022 Raspberry Pi (Trading) Ltd.
// SPDX-License-Identifier: BSD-3-Clause
//
// Device:      Winbond W25Q080
//              Also supports W25Q16JV (which has some different SR instructions)
//              Also supports AT25SF081
//              Also supports S25FL132K0
//
// Description: Configures W25Q080 to run in Quad I/O continuous read XIP mode
//
// Details:     * Check status register 2 to determine if QSPI mode is enabled,
//                and perform an SR2 programming cycle if necessary.
//              * Use SSI to perform a dummy 0xEB read command, with the mode
//                continuation bits set, so that the flash will not require
//                0xEB instruction prefix on subsequent reads.
//              * Configure SSI to write address, mode bits, but no instruction.
//                SSI + flash are now jointly in a state where continuous reads
//                can take place.
//              * Jump to exit pointer passed in via lr. Bootrom passes null,
//                in which case this code uses a default 256 byte flash offset
//
// Building:    * This code must be position-independent, and use stack only
//              * The code will be padded to a size of 256 bytes, including a
//                4-byte checksum. Therefore code size cannot exceed 252 bytes.
// ----------------------------------------------------------------------------

#include "pico/asm_helper.S"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/pads_qspi.h"
#include "hardware/regs/qmi.h"

// ----------------------------------------------------------------------------
// Config section
// ----------------------------------------------------------------------------
// It should be possible to support most flash devices by modifying this section

// The serial flash interface will run at clk_sys/PICO_FLASH_SPI_CLKDIV.
// This must be a positive integer.
// The bootrom is very conservative with SPI frequency, but here we should be
// as aggressive as possible.

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif
#if (PICO_FLASH_SPI_CLKDIV << QMI_M0_TIMING_CLKDIV_LSB) & ~QMI_M0_TIMING_CLKDIV_BITS
#error "CLKDIV greater than maximum"
#endif

// RX sampling delay is measured in units of one half clock cycle.

#ifndef PICO_FLASH_SPI_RXDELAY
#define PICO_FLASH_SPI_RXDELAY 2
#endif
#if (PICO_FLASH_SPI_RXDELAY << QMI_M0_TIMING_RXDELAY_LSB) & ~QMI_M0_TIMING_RXDELAY_BITS
#error "RX delay greater than maximum"
#endif

// Define interface width: single/dual/quad IO
#define FRAME_FORMAT SSI_CTRLR0_SPI_FRF_VALUE_QUAD

// For W25Q080 this is the "Read data fast quad IO" instruction:
#define CMD_READ 0xeb

// "Mode bits" are 8 special bits sent immediately after
// the address bits in a "Read Data Fast Quad I/O" command sequence. 
// On W25Q080, the four LSBs are don't care, and if MSBs == 0xa, the
// next read does not require the 0xeb instruction prefix.
#define MODE_CONTINUOUS_READ 0xa0

// How many clocks of Hi-Z following the mode bits. For W25Q080, 4 dummy cycles
// are required.
#define WAIT_CYCLES 4

// If defined, we will read status reg, compare to SREG_DATA, and overwrite
// with our value if the SR doesn't match.
// We do a two-byte write to SR1 (01h cmd) rather than a one-byte write to
// SR2 (31h cmd) as the latter command isn't supported by WX25Q080.
// This isn't great because it will remove block protections.
// A better solution is to use a volatile SR write if your device supports it.
#define PROGRAM_STATUS_REG

#define CMD_WRITE_ENABLE 0x06
#define CMD_READ_STATUS 0x05
#define CMD_READ_STATUS2 0x35
#define CMD_WRITE_STATUS 0x01
#define SREG_DATA 0x02  // Enable quad-SPI mode

// ----------------------------------------------------------------------------
// Register initialisation values -- same in Arm/RISC-V code.
// ----------------------------------------------------------------------------

// SCLK: 8mA drive, no slew limiting, input buffer disabled
#define INIT_PAD_SCLK  (\
    2 << PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_LSB | \
    PADS_QSPI_GPIO_QSPI_SCLK_SLEWFAST_BITS | \
0)

// Need to use direct serial mode to send SR commands. Choose a
// conservative direct-mode divisor (5 MHz at 150 MHz clk_sys)
// since the XIP-mode divisor may be unsafe without an RX delay.
#define INIT_DIRECT_CSR (\
    30 << QMI_DIRECT_CSR_CLKDIV_LSB | \
    QMI_DIRECT_CSR_EN_BITS | \
    QMI_DIRECT_CSR_AUTO_CS0N_BITS | \
0)

// Setup timing parameters: short sequential-access cooldown, configured
// CLKDIV and RXDELAY, and no constraints on CS max assertion, CS min
// deassertion, or page boundary burst breaks.
#define INIT_M0_TIMING (\
    1                      << QMI_M0_TIMING_COOLDOWN_LSB |\
    PICO_FLASH_SPI_RXDELAY << QMI_M0_TIMING_RXDELAY_LSB |\
    PICO_FLASH_SPI_CLKDIV  << QMI_M0_TIMING_CLKDIV_LSB |\
0)

// Set command constants (e.g. EBh + A0h)
#define INIT_M0_RCMD (\
    CMD_READ             << QMI_M0_RCMD_PREFIX_LSB |\
    MODE_CONTINUOUS_READ << QMI_M0_RCMD_SUFFIX_LSB |\
0)

// Set read format (serial command prefix, everything else quad)
// Initial dummy transfer has a serial EBh prefix, and has XIP mode bits
// set after the address. Subsequent transfers don't need the prefix, as long
// as the mode bits are set correctly.
#define INIT_M0_RFMT (\
    QMI_M0_RFMT_PREFIX_WIDTH_VALUE_S << QMI_M0_RFMT_PREFIX_WIDTH_LSB |\
    QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q   << QMI_M0_RFMT_ADDR_WIDTH_LSB |\
    QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB |\
    QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M0_RFMT_DUMMY_WIDTH_LSB |\
    QMI_M0_RFMT_DATA_WIDTH_VALUE_Q   << QMI_M0_RFMT_DATA_WIDTH_LSB |\
    QMI_M0_RFMT_PREFIX_LEN_VALUE_8   << QMI_M0_RFMT_PREFIX_LEN_LSB |\
    QMI_M0_RFMT_SUFFIX_LEN_VALUE_8   << QMI_M0_RFMT_SUFFIX_LEN_LSB |\
    WAIT_CYCLES                      << QMI_M0_RFMT_DUMMY_LEN_LSB |\
0)

// ----------------------------------------------------------------------------
// Start of 2nd Stage Boot Code
// ----------------------------------------------------------------------------

pico_default_asm_setup
.section .text

// ----------------------------------------------------------------------------
// RISC-V implementation

#ifdef __riscv

// On RP2350 boot stage2 is always called as a regular function, and should return normally
regular_func _stage2_boot
    mv t0, ra
_pad_config:
    li a3, PADS_QSPI_BASE
    li a0, INIT_PAD_SCLK
    sw a0, PADS_QSPI_GPIO_QSPI_SCLK_OFFSET(a3)
    // SDx: disable input Schmitt to reduce delay
    li a3, PADS_QSPI_BASE + REG_ALIAS_CLR_BITS
    li a0, PADS_QSPI_GPIO_QSPI_SD0_SCHMITT_BITS
    sw a0, PADS_QSPI_GPIO_QSPI_SD0_OFFSET(a3)
    sw a0, PADS_QSPI_GPIO_QSPI_SD1_OFFSET(a3)
    sw a0, PADS_QSPI_GPIO_QSPI_SD2_OFFSET(a3)
    sw a0, PADS_QSPI_GPIO_QSPI_SD3_OFFSET(a3)

    li a3, XIP_QMI_BASE

// On QSPI parts we usually need a 01h SR-write command to enable QSPI mode
// (i.e. turn WPn and HOLDn into IO2/IO3)
#ifdef PROGRAM_STATUS_REG
program_sregs:
    li a1, INIT_DIRECT_CSR
    sw a1, QMI_DIRECT_CSR_OFFSET(a3)
    // Wait for cooldown on last XIP transfer to expire, by polling BUSY
1:
    lw a1, QMI_DIRECT_CSR_OFFSET(a3)
    andi a1, a1, QMI_DIRECT_CSR_BUSY_BITS
    bnez a1, 1b

    // Check whether SR needs updating
    li a0, CMD_READ_STATUS2
    jal read_flash_sreg
    addi a0, a0, -SREG_DATA
    beqz a0, _skip_sreg_programming

    // Send write enable command, discard RX
    li a0, CMD_WRITE_ENABLE
    sw a0, QMI_DIRECT_TX_OFFSET(a3)
    jal wait_qmi_ready
    lw a0, QMI_DIRECT_RX_OFFSET(a3)

    // Send status write command followed by data bytes
    li a0, CMD_WRITE_STATUS
    sw a0, QMI_DIRECT_TX_OFFSET(a3)
    sw zero, QMI_DIRECT_TX_OFFSET(a3)
    li a0, SREG_DATA
    sw a0, QMI_DIRECT_TX_OFFSET(a3)
    jal wait_qmi_ready
    lw a0, QMI_DIRECT_RX_OFFSET(a3)
    lw a0, QMI_DIRECT_RX_OFFSET(a3)
    lw a0, QMI_DIRECT_RX_OFFSET(a3)

    // Poll status register for write completion
1:
    li a0, CMD_READ_STATUS
    jal read_flash_sreg
    andi a0, a0, 0x1
    bnez a0, 1b

_skip_sreg_programming:
    // Disable direct mode
    andi a1, a1, ~QMI_DIRECT_CSR_EN_BITS
    sw a1, QMI_DIRECT_CSR_OFFSET(a3)
#endif

_qmi_config:
    li a0, INIT_M0_TIMING
    sw a0, QMI_M0_TIMING_OFFSET(a3)
    li a0, INIT_M0_RCMD
    sw a0, QMI_M0_RCMD_OFFSET(a3)
    li a0, INIT_M0_RFMT
    sw a0, QMI_M0_RFMT_OFFSET(a3)

    // Dummy transfer
    li a1, XIP_NOCACHE_NOALLOC_BASE
    lw a1, (a1)

    // Set prefix length to 0, as flash no longer expects to see commands
    bclri a0, a0, QMI_M0_RFMT_PREFIX_LEN_LSB
    sw a0, QMI_M0_RFMT_OFFSET(a3)

// ----------------------------------------------------------------------------
// Arm implementation

#else

// On RP2350 boot stage2 is always called as a regular function, and should return normally
regular_func _stage2_boot
    push {lr}
_pad_config:
    ldr r3, =PADS_QSPI_BASE
    movs r0, INIT_PAD_SCLK
    str r0, [r3, #PADS_QSPI_GPIO_QSPI_SCLK_OFFSET]
    // SDx: disable input Schmitt to reduce delay
    adds r3, #REG_ALIAS_CLR_BITS
    movs r0, #PADS_QSPI_GPIO_QSPI_SD0_SCHMITT_BITS
    str r0, [r3, #PADS_QSPI_GPIO_QSPI_SD0_OFFSET]
    str r0, [r3, #PADS_QSPI_GPIO_QSPI_SD1_OFFSET]
    str r0, [r3, #PADS_QSPI_GPIO_QSPI_SD2_OFFSET]
    str r0, [r3, #PADS_QSPI_GPIO_QSPI_SD3_OFFSET]

    adds r3, (XIP_QMI_BASE - (PADS_QSPI_BASE + REG_ALIAS_CLR_BITS))

// On QSPI parts we usually need a 01h SR-write command to enable QSPI mode
// (i.e. turn WPn and HOLDn into IO2/IO3)
#ifdef PROGRAM_STATUS_REG
program_sregs:

    // Need to use direct serial mode to send SR commands. Choose a
    // conservative direct-mode divisor (5 MHz at 150 MHz clk_sys)
    // since the XIP-mode divisor may be unsafe without an RX delay.
    ldr r1, =INIT_DIRECT_CSR
    str r1, [r3, #QMI_DIRECT_CSR_OFFSET]
    // Need to poll for the cooldown on the last XIP transfer to expire
    // (via direct-mode BUSY flag) before it is safe to perform the first
    // direct-mode operation
1:
    ldr r0, [r3, #QMI_DIRECT_CSR_OFFSET]
    tst r0, #QMI_DIRECT_CSR_BUSY_BITS
    bne 1b

    // Check whether SR needs updating
    movs r0, #CMD_READ_STATUS2
    bl read_flash_sreg
    cmp r0, #SREG_DATA
    beq _skip_sreg_programming

    // Send write enable command, discard RX
    movs r0, #CMD_WRITE_ENABLE
    str r0, [r3, #QMI_DIRECT_TX_OFFSET]
    bl wait_qmi_ready
    ldr r0, [r3, #QMI_DIRECT_RX_OFFSET]

    // Send status write command followed by data bytes
    movs r0, #CMD_WRITE_STATUS
    str r0, [r3, #QMI_DIRECT_TX_OFFSET]
    movs r0, #0
    str r0, [r3, #QMI_DIRECT_TX_OFFSET]
    movs r0, #SREG_DATA
    str r0, [r3, #QMI_DIRECT_TX_OFFSET]
    bl wait_qmi_ready
    ldr r0, [r3, #QMI_DIRECT_RX_OFFSET]
    ldr r0, [r3, #QMI_DIRECT_RX_OFFSET]
    ldr r0, [r3, #QMI_DIRECT_RX_OFFSET]

    // Poll status register for write completion
1:
    movs r0, #CMD_READ_STATUS
    bl read_flash_sreg
    lsrs r0, #1
    bcs 1b

_skip_sreg_programming:
    // Disable direct mode
    bics r1, #QMI_DIRECT_CSR_EN_BITS
    str r1, [r3, #QMI_DIRECT_CSR_OFFSET]

#endif

_qmi_config:
    ldr r0, =INIT_M0_TIMING
    str r0, [r3, #QMI_M0_TIMING_OFFSET]
    ldr r0, =INIT_M0_RCMD
    str r0, [r3, #QMI_M0_RCMD_OFFSET]
    ldr r0, =INIT_M0_RFMT
    str r0, [r3, #QMI_M0_RFMT_OFFSET]

    // Dummy transfer
    mov r1, #XIP_NOCACHE_NOALLOC_BASE
    ldrb r1, [r1]

    // Set prefix length to 0, as flash no longer expects to see commands
    bic r0, #QMI_M0_RFMT_PREFIX_LEN_BITS
    str r0, [r3, #QMI_M0_RFMT_OFFSET]

#endif

// ----------------------------------------------------------------------------

// Pull in standard exit routine
#include "boot2_helpers/exit_from_boot2.S"

// Common functions
#include "boot2_helpers/wait_qmi_ready.S"
#ifdef PROGRAM_STATUS_REG
#include "boot2_helpers/read_flash_sreg.S"
#endif

#ifndef __riscv
.global literals
literals:
.ltorg
#endif
