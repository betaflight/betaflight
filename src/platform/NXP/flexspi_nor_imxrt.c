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
 * FlexSPI NOR erase and program through the boot ROM driver. The core cannot
 * fetch from the die while it erases or programs, so every function here runs
 * from ITCM with interrupts masked around each ROM call, and writes are limited
 * to the configuration region.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "flexspi_nor_imxrt.h"
#include "romapi_imxrt.h"
#include "xip/flexspi_nor_config.h"
#include "xip/flexspi_nor_profiles.h"

#ifndef IMXRT_FLEXSPI_NOR_INSTANCE
#error "Define IMXRT_FLEXSPI_NOR_INSTANCE in target.h: which FlexSPI controller carries the boot flash is board wiring"
#endif

// The 10xx ROM numbers FlexSPI controllers from zero, the 11xx ROM from one.
#if defined(IMXRT11XX)
#define IMXRT_FLEXSPI_NOR_ROM_INSTANCE      (IMXRT_FLEXSPI_NOR_INSTANCE + 1u)
#else
#define IMXRT_FLEXSPI_NOR_ROM_INSTANCE      (IMXRT_FLEXSPI_NOR_INSTANCE)
#endif

// An octal part boots in single line SPI and is switched to its fast mode with
// the second block its profile carries.
#ifdef FLEXSPI_NOR_HAS_FAST_CONFIG
#define FLEXSPI_NOR_RUNTIME_CONFIG      flexspiNorFastConfig
#else
#define FLEXSPI_NOR_RUNTIME_CONFIG      flexspiNorConfig
#endif

// Start of the memory mapped flash window; the ROM driver addresses flash from zero.
extern uint32_t __flash_start[];

// The ROM driver writes to its configuration block, so it must live in RAM.
static flexspiNorConfig_t romNorConfig;
static const flexspiNorDriverInterface_t *romNorDriver;

// The 10xx headers leave the first controller unnumbered, the 11xx headers call
// it FLEXSPI1.
#if IMXRT_FLEXSPI_NOR_INSTANCE == 0
#if defined(IMXRT11XX)
#define IMXRT_FLEXSPI_NOR_REGS  FLEXSPI1
#elif defined(IMXRT10XX)
#define IMXRT_FLEXSPI_NOR_REGS  FLEXSPI
#else
#error "Unsupported i.MXRT family: no register block name for FlexSPI instance 0"
#endif
#else
#define IMXRT_FLEXSPI_NOR_REGS  FLEXSPI2
#endif

static RAMFUNC uint32_t flexspiNorOffsetOf(uint32_t address)
{
    return address - (uint32_t)__flash_start;
}

// Bounds every wait on a controller status bit.
#define FLEXSPI_NOR_SPIN            200000u

#ifdef FLEXSPI_NOR_HAS_FAST_CONFIG
/*
 * DQS read strobe calibration: read a reference window back at every delay line
 * code and keep the middle of the widest passing run. The window is compiled
 * code past the vector table, since a mis-sampled read returns zeros.
 */
#define FLEXSPI_NOR_DLL_WORDS       128u        // 512 byte reference window
#define FLEXSPI_NOR_DLL_CHUNK       16u         // 64 bytes per command, the RX FIFO holds 128
#define FLEXSPI_NOR_DLL_SETTLE      4000u       // let the delay line settle after a lock, errata ERR011377
#define FLEXSPI_NOR_DLL_LOCK        (FLEXSPI_STS2_ASLVLOCK_MASK | FLEXSPI_STS2_AREFLOCK_MASK)
#define FLEXSPI_NOR_DLL_IDLE        (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)

// Flash address the reset handler copies ITCM from, from the linker script.
extern uint32_t __itcm_load[];

#define FLEXSPI_NOR_DLL_WINDOW      ((uint32_t)__itcm_load + 1024u)

static uint32_t dllReference[FLEXSPI_NOR_DLL_WORDS];

// Must be called while the flash still reads the way the boot ROM left it.
static RAMFUNC void flexspiNorCaptureDllReference(void)
{
    // Volatile keeps the loop from becoming a memcpy call in flash.
    const volatile uint32_t *window = (const volatile uint32_t *)FLEXSPI_NOR_DLL_WINDOW;
    for (unsigned i = 0; i < FLEXSPI_NOR_DLL_WORDS; i++) {
        dllReference[i] = window[i];
    }
}

// Program the port A delay line; DLLCR may only change while the controller is stopped.
static RAMFUNC bool flexspiNorDllWrite(FLEXSPI_Type *flexspi, uint32_t dllcr)
{
    for (uint32_t spin = FLEXSPI_NOR_SPIN;
         spin-- && (flexspi->STS0 & FLEXSPI_NOR_DLL_IDLE) != FLEXSPI_NOR_DLL_IDLE;) {
    }

    // The controller must not be stopped during a transfer.
    if ((flexspi->STS0 & FLEXSPI_NOR_DLL_IDLE) != FLEXSPI_NOR_DLL_IDLE) {
        return false;
    }

    flexspi->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
    __DSB();
    __ISB();
    flexspi->DLLCR[0] = dllcr;
    flexspi->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

    if (dllcr & FLEXSPI_DLLCR_DLLEN_MASK) {
        for (uint32_t spin = FLEXSPI_NOR_SPIN;
             spin-- && (flexspi->STS2 & FLEXSPI_NOR_DLL_LOCK) != FLEXSPI_NOR_DLL_LOCK;) {
        }

        if ((flexspi->STS2 & FLEXSPI_NOR_DLL_LOCK) != FLEXSPI_NOR_DLL_LOCK) {
            return false;
        }
    }

    for (volatile uint32_t settle = FLEXSPI_NOR_DLL_SETTLE; settle--;) {
    }

    return true;
}

// Read the window back with IP commands so no prefetch buffer or cache answers.
static RAMFUNC bool flexspiNorDllReadMatches(FLEXSPI_Type *flexspi, uint32_t offset)
{
    for (unsigned word = 0; word < FLEXSPI_NOR_DLL_WORDS; word += FLEXSPI_NOR_DLL_CHUNK) {
        const uint32_t bytes = FLEXSPI_NOR_DLL_CHUNK * 4u;

        flexspi->INTR = FLEXSPI_INTR_IPCMDDONE_MASK | FLEXSPI_INTR_IPCMDERR_MASK
                      | FLEXSPI_INTR_IPCMDGE_MASK | FLEXSPI_INTR_AHBCMDERR_MASK;
        flexspi->IPRXFCR |= FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
        flexspi->IPCR0 = offset + word * 4u;
        // Sequence 0 is the read sequence in every configuration block here.
        flexspi->IPCR1 = FLEXSPI_IPCR1_IDATSZ(bytes) | FLEXSPI_IPCR1_ISEQID(0)
                       | FLEXSPI_IPCR1_ISEQNUM(0);
        flexspi->IPCMD = FLEXSPI_IPCMD_TRG_MASK;

        uint32_t interrupts = 0;
        for (uint32_t spin = FLEXSPI_NOR_SPIN; spin--;) {
            interrupts = flexspi->INTR;
            if (interrupts & (FLEXSPI_INTR_IPCMDDONE_MASK | FLEXSPI_INTR_IPCMDERR_MASK
                              | FLEXSPI_INTR_IPCMDGE_MASK)) {
                break;
            }
        }

        if ((interrupts & FLEXSPI_INTR_IPCMDDONE_MASK) == 0
            || (interrupts & (FLEXSPI_INTR_IPCMDERR_MASK | FLEXSPI_INTR_IPCMDGE_MASK))) {
            return false;
        }

        // The fill level counts 64 bit words.
        for (uint32_t spin = FLEXSPI_NOR_SPIN;
             spin-- && ((flexspi->IPRXFSTS & FLEXSPI_IPRXFSTS_FILL_MASK)
                        >> FLEXSPI_IPRXFSTS_FILL_SHIFT) * 8u < bytes;) {
        }

        // A short FIFO still holds whatever the last command left there.
        if (((flexspi->IPRXFSTS & FLEXSPI_IPRXFSTS_FILL_MASK)
             >> FLEXSPI_IPRXFSTS_FILL_SHIFT) * 8u < bytes) {
            return false;
        }

        for (unsigned i = 0; i < FLEXSPI_NOR_DLL_CHUNK; i++) {
            if (flexspi->RFDR[i] != dllReference[word + i]) {
                return false;
            }
        }
    }

    return true;
}

// On anything unexpected the ROM's own delay is put back.
static RAMFUNC void flexspiNorCalibrateDllReads(FLEXSPI_Type *flexspi)
{
    // The delay line only applies to the DQS pad strobe.
    if (((flexspi->MCR0 & FLEXSPI_MCR0_RXCLKSRC_MASK) >> FLEXSPI_MCR0_RXCLKSRC_SHIFT)
        != FLASH_READ_SAMPLE_CLK_EXTERNALINPUT_FROM_DQSPAD) {
        return;
    }

    const uint32_t romDllcr = flexspi->DLLCR[0];
    const uint32_t offset = flexspiNorOffsetOf(FLEXSPI_NOR_DLL_WINDOW);
    uint16_t passMask = 0;

    // Code 0 is not a usable delay target.
    for (uint32_t code = 1; code < 16; code++) {
        if (flexspiNorDllWrite(flexspi, FLEXSPI_DLLCR_DLLEN_MASK | FLEXSPI_DLLCR_SLVDLYTARGET(code))
            && flexspiNorDllReadMatches(flexspi, offset)) {
            passMask |= (uint16_t)(1u << code);
        }
    }

    const int chosen = flexspiNorDllCenter(passMask);
    if (chosen < 0
        || !flexspiNorDllWrite(flexspi, FLEXSPI_DLLCR_DLLEN_MASK
                                        | FLEXSPI_DLLCR_SLVDLYTARGET((uint32_t)chosen))) {
        flexspiNorDllWrite(flexspi, romDllcr);
    }

    // The ROM driver starts its own commands with an empty receive FIFO.
    flexspi->IPRXFCR |= FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
}
#else
// A part left in the mode its boot block asked for keeps the ROM's read timing.
static RAMFUNC void flexspiNorCaptureDllReference(void) { }
static RAMFUNC void flexspiNorCalibrateDllReads(FLEXSPI_Type *flexspi) { (void)flexspi; }
#endif

// Drop the AHB read buffers so the next read goes to the die.
static RAMFUNC void flexspiNorClearAhbBuffers(FLEXSPI_Type *flexspi)
{
    flexspi->AHBCR |= FLEXSPI_AHBCR_CLRAHBRXBUF_MASK;
    for (uint32_t spin = FLEXSPI_NOR_SPIN;
         spin-- && (flexspi->AHBCR & FLEXSPI_AHBCR_CLRAHBRXBUF_MASK);) {
    }
}

// Give all AHB read buffer RAM to the last buffer, which serves every master,
// and enable prefetch.

static RAMFUNC void flexspiNorTuneAhbReads(FLEXSPI_Type *flexspi)
{
    flexspiNorClearAhbBuffers(flexspi);

    for (unsigned buffer = 0; buffer < FLEXSPI_AHBRXBUFCR0_COUNT - 1u; buffer++) {
        flexspi->AHBRXBUFCR0[buffer] = 0;
    }
    // BUFSZ counts 64 bit words.
    const uint32_t words = FSL_FEATURE_FLEXSPI_AHB_RX_BUFFER_SIZEn(IMXRT_FLEXSPI_NOR_INSTANCE) / 8u;
    flexspi->AHBRXBUFCR0[FLEXSPI_AHBRXBUFCR0_COUNT - 1u] =
        FLEXSPI_AHBRXBUFCR0_PREFETCHEN_MASK | FLEXSPI_AHBRXBUFCR0_BUFSZ(words);

    flexspi->AHBCR |= FLEXSPI_AHBCR_CACHABLEEN_MASK
                    | FLEXSPI_AHBCR_BUFFERABLEEN_MASK
                    | FLEXSPI_AHBCR_PREFETCHEN_MASK;
}

RAMFUNC bool flexspiNorInit(void)
{
    if (romNorDriver) {
        return true;
    }

    const flexspiNorDriverInterface_t *driver = romFlexspiNorDriver();
    if (!driver) {
        return false;
    }

    // Volatile keeps the copy from becoming a memcpy call in flash.
    const uint32_t *source = (const uint32_t *)&FLEXSPI_NOR_RUNTIME_CONFIG;
    volatile uint32_t *destination = (volatile uint32_t *)&romNorConfig;
    for (unsigned i = 0; i < sizeof(romNorConfig) / sizeof(uint32_t); i++) {
        destination[i] = source[i];
    }

    // The driver reconfigures the controller and part, so fence before returning.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Last moment the flash still reads the way the boot ROM left it.
    flexspiNorCaptureDllReference();

    const romStatus_t status = driver->init(IMXRT_FLEXSPI_NOR_ROM_INSTANCE, &romNorConfig);
    flexspiNorClearAhbBuffers(IMXRT_FLEXSPI_NOR_REGS);

    // Only after the driver has brought the controller up.
    if (status == ROM_STATUS_SUCCESS) {
        flexspiNorCalibrateDllReads(IMXRT_FLEXSPI_NOR_REGS);
        flexspiNorTuneAhbReads(IMXRT_FLEXSPI_NOR_REGS);
    }

    __DSB();
    __ISB();

    __set_PRIMASK(primask);

    if (status != ROM_STATUS_SUCCESS) {
        return false;
    }

    romNorDriver = driver;
    return true;
}

// Drop prefetched and cached reads of the range that just changed.
static RAMFUNC void flexspiNorDropStaleReads(uint32_t address, uint32_t length)
{
    flexspiNorClearAhbBuffers(IMXRT_FLEXSPI_NOR_REGS);
    SCB_InvalidateDCache_by_Addr((void *)address, (int32_t)length);
}

RAMFUNC bool flexspiNorEraseSector(uint32_t address)
{
    // Bounds of the reserved configuration region, from the linker script.
    if (!flexspiNorRangeAllowed((uint32_t)&__config_start, (uint32_t)&__config_end,
                                address, FLEXSPI_NOR_SECTOR_SIZE, FLEXSPI_NOR_SECTOR_SIZE)) {
        return false;
    }

    if (!romNorDriver) {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const romStatus_t status = romNorDriver->erase(IMXRT_FLEXSPI_NOR_ROM_INSTANCE, &romNorConfig,
                                                   flexspiNorOffsetOf(address), FLEXSPI_NOR_SECTOR_SIZE);
    flexspiNorDropStaleReads(address, FLEXSPI_NOR_SECTOR_SIZE);

    __set_PRIMASK(primask);

    return status == ROM_STATUS_SUCCESS;
}

RAMFUNC bool flexspiNorProgramPage(uint32_t address, const uint32_t *data)
{
    // Bounds of the reserved configuration region, from the linker script.
    if (!flexspiNorRangeAllowed((uint32_t)&__config_start, (uint32_t)&__config_end,
                                address, FLEXSPI_NOR_PAGE_SIZE, FLEXSPI_NOR_PAGE_SIZE)) {
        return false;
    }

    // The ROM driver reads the source a word at a time.
    if (!data || ((uintptr_t)data & 3u)) {
        return false;
    }

    if (!romNorDriver) {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const romStatus_t status = romNorDriver->program(IMXRT_FLEXSPI_NOR_ROM_INSTANCE, &romNorConfig,
                                                     flexspiNorOffsetOf(address), data);
    flexspiNorDropStaleReads(address, FLEXSPI_NOR_PAGE_SIZE);

    __set_PRIMASK(primask);

    return status == ROM_STATUS_SUCCESS;
}
