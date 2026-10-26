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
 * NavQ95 (i.MX 95 M7) FlexSPI NOR boot probe (NAVQ95_NOR_PROBE).
 *
 * Decides, on real hardware, whether the M7 can borrow the FlexSPI controller
 * that mcuboot already configured (read LUT in 8D-8D-8D) and write the NOR.
 * The earlier conclusion that "the M7 cannot write the NOR" was a false
 * negative (unconfigured LUT + run-from-flash hazard), the same class of
 * mistake as the TPM3 clock "DENIED" — this probe settles it with evidence.
 *
 * Two stages, both printed over the LPUART2 console:
 *   1) ALWAYS (read-only): dump the live FlexSPI controller state (MCR0, STS0,
 *      STS2, FLSHCR2[0], the read-LUT seq id + its first LUT word), then — if
 *      the flash writer is compiled in (NAVQ95_USE_FLEXSPI_NOR) — call
 *      flexspiNorInit() (installs the 4 write LUT slots, verifies the read LUT
 *      is valid octal DTR) and read the NOR status register (RDSR) over the IP
 *      engine. This proves M7<->NOR command comms with ZERO writes.
 *   2) ONLY under NAVQ95_NOR_WRITE_TEST (and NAVQ95_USE_FLEXSPI_NOR): a bounded
 *      destructive test — erase ONE scratch sector in the config partition,
 *      program a known pattern page, flush, read it back from the memory-mapped
 *      window, report PASS/FAIL. Touches only the config partition (the writer
 *      range-guards every op); the app slot is never at risk.
 *
 * Non-fatal: never hangs boot (every controller wait is bounded in the driver).
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef NAVQ95_NOR_PROBE

#include "platform/flexspi_regs.h"

extern void navq95_console_puts(const char *s);
extern void navq95_console_puthex(uint32_t v);
extern void navq95_console_drain(void);

#if defined(NAVQ95_USE_FLEXSPI_NOR)
#include "flexspi_nor_imxrt.h"
extern struct linker_symbol __config_start;
extern struct linker_symbol __config_end;
#endif

void navq95_nor_probe(void)
{
    navq95_console_puts("\n[NOR probe]\n");

    /* 1a. Read-only controller state (safe even with no writer compiled). */
    const uint32_t mcr0    = FLEXSPI->MCR0;
    const uint32_t sts0    = FLEXSPI->STS0;
    const uint32_t sts2    = FLEXSPI->STS2;
    const uint32_t flshcr2 = FLEXSPI->FLSHCR2[0];
    const uint32_t rdSeq   = (flshcr2 & FLEXSPI_FLSHCR2_ARDSEQID_MASK) >> FLEXSPI_FLSHCR2_ARDSEQID_SHIFT;
    const uint32_t rdFirst = FLEXSPI->LUT[rdSeq * 4u];

    navq95_console_puts("  MCR0=");
    navq95_console_puthex(mcr0);
    navq95_console_puts(" STS0=");
    navq95_console_puthex(sts0);
    navq95_console_puts(" STS2=");
    navq95_console_puthex(sts2);
    navq95_console_puts("\n  FLSHCR2[0]=");
    navq95_console_puthex(flshcr2);
    navq95_console_puts(" rdSeq=");
    navq95_console_puthex(rdSeq);
    navq95_console_puts(" rdLUT0=");
    navq95_console_puthex(rdFirst);
    navq95_console_puts("\n");
    navq95_console_drain();

#if !defined(NAVQ95_USE_FLEXSPI_NOR)
    navq95_console_puts("  writer not compiled (NAVQ95_USE_FLEXSPI_NOR off) -"
                        " read-only dump only.\n");
    navq95_console_puts("[/NOR probe]\n");
    navq95_console_drain();
#else
    /* 1b. Borrow the controller + install the write LUT slots (no NOR writes). */
    const bool initOk = flexspiNorInit();
    navq95_console_puts("  flexspiNorInit=");
    navq95_console_puts(initOk ? "OK" : "FAIL");
    navq95_console_puts(initOk ? " (read LUT is valid octal DTR; write slots installed)\n"
                               : " (read LUT not recognized -> not borrowable, no writes attempted)\n");
    navq95_console_drain();

    if (!initOk) {
        navq95_console_puts("[/NOR probe]\n");
        navq95_console_drain();
        return;
    }

    /* 1c. Read the NOR status register over the IP engine (proves comms). */
    uint32_t norStatus = 0;
    const bool rdsrOk = flexspiNorReadStatus(&norStatus);
    navq95_console_puts("  RDSR=");
    navq95_console_puts(rdsrOk ? "OK status=" : "FAIL status=");
    navq95_console_puthex(norStatus);
    navq95_console_puts("\n");
    navq95_console_drain();

#ifdef NAVQ95_NOR_WRITE_TEST
    /* 2. Bounded destructive test on ONE scratch sector of the config region. */
    const uint32_t sector = (uint32_t)(uintptr_t)&__config_start;  /* first config sector */
    navq95_console_puts("  WRITE TEST on sector=");
    navq95_console_puthex(sector);
    navq95_console_puts("\n");
    navq95_console_drain();

    bool eraseOk = flexspiNorEraseSector(sector);
    navq95_console_puts("  erase=");
    navq95_console_puts(eraseOk ? "OK" : "FAIL");

    /* Build a known 256-byte page pattern (word i = 0xA5A50000 | i). */
    static uint32_t page[FLEXSPI_NOR_PAGE_SIZE / 4];
    for (uint32_t i = 0; i < FLEXSPI_NOR_PAGE_SIZE / 4u; i++) {
        page[i] = 0xA5A50000u | i;
    }
    bool progOk = eraseOk && flexspiNorProgramPage(sector, page);
    navq95_console_puts(" program=");
    navq95_console_puts(progOk ? "OK" : "FAIL");

    bool flushOk = flexspiNorFlush();
    navq95_console_puts(" flush=");
    navq95_console_puts(flushOk ? "OK" : "FAIL");
    navq95_console_puts("\n");
    navq95_console_drain();

    /* Read back from the memory-mapped NOR window and verify. */
    if (progOk && flushOk) {
        const volatile uint32_t *nor = (const volatile uint32_t *)(uintptr_t)sector;
        uint32_t bad = 0;
        uint32_t firstBad = 0;
        for (uint32_t i = 0; i < FLEXSPI_NOR_PAGE_SIZE / 4u; i++) {
            if (nor[i] != page[i]) {
                if (!bad) {
                    firstBad = nor[i];
                }
                bad++;
            }
        }
        navq95_console_puts("  readback ");
        navq95_console_puts(bad ? "MISMATCH badWords=" : "PASS badWords=");
        navq95_console_puthex(bad);
        navq95_console_puts(" word0=");
        navq95_console_puthex(nor[0]);
        navq95_console_puts(" firstBad=");
        navq95_console_puthex(firstBad);
        navq95_console_puts("\n  VERDICT: ");
        navq95_console_puts(bad ? "NOR write did NOT persist correctly.\n"
                                : "NOR erase+program+read-back WORKS -> persistence is viable.\n");
    } else {
        navq95_console_puts("  VERDICT: erase/program/flush failed -> see codes above.\n");
    }
    navq95_console_drain();
#else
    navq95_console_puts("  (write test gated off: define NAVQ95_NOR_WRITE_TEST to erase/program)\n");
    navq95_console_drain();
#endif /* NAVQ95_NOR_WRITE_TEST */

    navq95_console_puts("[/NOR probe]\n");
    navq95_console_drain();
#endif /* NAVQ95_USE_FLEXSPI_NOR */
}

#endif /* NAVQ95_NOR_PROBE */
