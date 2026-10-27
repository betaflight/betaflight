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
 * bus_i2c_navq95.c — NavQ95 (NXP i.MX 95, Cortex-M7) LPI2C platform driver.
 *
 * PHASE 1 (bring-up): register-level, polled LPI2C master driver for two
 * instances, with Option-A Betaflight bus numbering:
 *   I2CDEV_1 -> LPI2C1 (AONMIX,    base 0x44340000, CCM clock root 6)
 *   I2CDEV_6 -> LPI2C6 (WAKEUPMIX, base 0x426C0000, CCM clock root 91)
 *
 * This supplies the platform half of Betaflight's generic I2C API declared in
 * drivers/bus_i2c.h (i2cInit / i2cRead[Buffer] / i2cWrite[Buffer] / i2cBusy /
 * i2cPinConfigure / ...). It mirrors the LPSPI1 polled driver in
 * platform_stubs.c: no DMA, no interrupts; the pads are muxed and the
 * functional clock ungated over SCMI by the System Manager (the M7 owns
 * neither IOMUXC nor the CCM) via the shared navq95_scmi_* helpers.
 *
 * The LPI2C register map / command encoding follows the NuttX imxrt_lpi2c.h
 * reference (same IP block as the i.MX 95) and the MCUXpresso fsl_lpi2c
 * master-transfer sequence, used here as a register/sequence reference only —
 * NO SDK sources are compiled in (platform.h is deliberately fsl_*-free).
 *
 * Clocking: the i.MX 95 LPI2C functional clock comes up on the 24 MHz OSC root
 * (same assumption as the LPSPI1 bring-up; matches PX4/NuttX NavQ95 board.h,
 * which sets LPI2C1/6_CLK = OSC_24M | CLOCK_DIV(1) = 24 MHz). At 100 kHz the
 * MCCR0 divisors are programmed PER BUS with PRESCALE = 2 (divide-by-4 ->
 * 6 MHz tick); see navq95_lpi2c_configure() and the NAVQ95_I2C{1,6}_MCCR0
 * macros. I2C1 uses a slightly faster nominal to compensate its weak-pull-up
 * SCL rise time so the measured rate is ~100 kHz.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

#include "pg/bus_i2c.h"

#include "navq95_scmi.h"

/* ── LPI2C register offsets (NuttX imxrt_lpi2c.h; i.MX 95 shares the IP) ──── */
#define LPI2C_VERID_OFFSET    0x00u   /* Version ID (reads only w/ func clock) */
#define LPI2C_PARAM_OFFSET    0x04u   /* Parameter (FIFO sizes)                */
#define LPI2C_MCR_OFFSET      0x10u   /* Master Control          */
#define LPI2C_MSR_OFFSET      0x14u   /* Master Status           */
#define LPI2C_MIER_OFFSET     0x18u   /* Master Interrupt Enable */
#define LPI2C_MCFGR1_OFFSET   0x24u   /* Master Config 1         */
#define LPI2C_MCFGR2_OFFSET   0x28u   /* Master Config 2         */
#define LPI2C_MCFGR3_OFFSET   0x2Cu   /* Master Config 3         */
#define LPI2C_MCCR0_OFFSET    0x48u   /* Master Clock Config 0   */
#define LPI2C_MFCR_OFFSET     0x58u   /* Master FIFO Control     */
#define LPI2C_MFSR_OFFSET     0x5Cu   /* Master FIFO Status      */
#define LPI2C_MTDR_OFFSET     0x60u   /* Master Transmit Data    */
#define LPI2C_MRDR_OFFSET     0x70u   /* Master Receive Data     */

/* MCR bits */
#define LPI2C_MCR_MEN         (1u << 0)   /* Master enable        */
#define LPI2C_MCR_RST         (1u << 1)   /* Software reset       */
#define LPI2C_MCR_RTF         (1u << 8)   /* Reset TX FIFO        */
#define LPI2C_MCR_RRF         (1u << 9)   /* Reset RX FIFO        */

/* MSR flags (write-1-to-clear for the latched ones) */
#define LPI2C_MSR_TDF         (1u << 0)   /* TX data (FIFO space) */
#define LPI2C_MSR_RDF         (1u << 1)   /* RX data available    */
#define LPI2C_MSR_EPF         (1u << 8)   /* End packet (STOP)    */
#define LPI2C_MSR_SDF         (1u << 9)   /* STOP detect          */
#define LPI2C_MSR_NDF         (1u << 10)  /* NACK detect          */
#define LPI2C_MSR_ALF         (1u << 11)  /* Arbitration lost     */
#define LPI2C_MSR_FEF         (1u << 12)  /* FIFO error           */
#define LPI2C_MSR_MBF         (1u << 24)  /* Master busy          */
#define LPI2C_MSR_BBF         (1u << 25)  /* Bus busy             */
#define LPI2C_MSR_ERROR_MASK  (LPI2C_MSR_NDF | LPI2C_MSR_ALF | LPI2C_MSR_FEF)
#define LPI2C_MSR_W1C         (LPI2C_MSR_EPF | LPI2C_MSR_SDF | LPI2C_MSR_NDF | \
                               LPI2C_MSR_ALF | LPI2C_MSR_FEF)

/* MCFGR1 */
#define LPI2C_MCFGR1_PRESCALE(n)  ((uint32_t)(n) & 0x7u)

/* MCCR0 clock-config fields */
#define LPI2C_MCCR0_CLKLO(n)    ((uint32_t)(n) & 0x3Fu)
#define LPI2C_MCCR0_CLKHI(n)    (((uint32_t)(n) & 0x3Fu) << 8)
#define LPI2C_MCCR0_SETHOLD(n)  (((uint32_t)(n) & 0x3Fu) << 16)
#define LPI2C_MCCR0_DATAVD(n)   (((uint32_t)(n) & 0x3Fu) << 24)

/* MTDR command field (bits 10:8) + data/len byte (bits 7:0) */
#define LPI2C_MTDR_CMD_TXD      (0u << 8)   /* transmit DATA byte               */
#define LPI2C_MTDR_CMD_RXD      (1u << 8)   /* receive (DATA+1) bytes           */
#define LPI2C_MTDR_CMD_STOP     (2u << 8)   /* generate STOP                    */
#define LPI2C_MTDR_CMD_START    (4u << 8)   /* START + transmit address (DATA)  */

/* MRDR */
#define LPI2C_MRDR_RXEMPTY      (1u << 14)

/* ── NavQ95 LPI2C clock roots (i.MX 95 CCM root index; SM id = root + 41) ─── */
#define NAVQ95_LPI2C1_CLK_ROOT  6u
#define NAVQ95_LPI2C6_CLK_ROOT  91u

/* LPI2C functional clock (OSC_24M root, as the LPSPI1 bring-up assumes). */
#define NAVQ95_LPI2C_SRC_HZ     24000000u

/* Transfer poll timeout (busy-wait iterations). Generous for a 100 kHz bus. */
#define NAVQ95_LPI2C_SPIN       2000000u

/* Short timeout for the scan probe + its STOP: a real address frame at 100 kHz
 * is ~90 us, so this is ample for an ACK/NACK, while keeping a full 0x00..0x7F
 * sweep quick even when the bus is dead/floating (128 probes must NOT each burn
 * the full NAVQ95_LPI2C_SPIN or the scan looks hung). */
#define NAVQ95_LPI2C_PROBE_SPIN 50000u

/* ── IOMUXC pad registers for the two buses (absolute addresses as given by
 * the board pin-mux table). IOMUXC_BASE = 0x443C0000. Tuple order from the
 * board header is <muxReg muxMode inputReg inputDaisy configReg>. ─────────── */

/* LPI2C6 — shared GPIO_IO02/03 pads, ALT 6, each needs a SELECT_INPUT daisy. */
#define NAVQ95_I2C6_SDA_MUX     0x443C0018u   /* GPIO_IO02 -> LPI2C6_SDA        */
#define NAVQ95_I2C6_SDA_DSY     0x443C051Cu   /* SDA SELECT_INPUT               */
#define NAVQ95_I2C6_SDA_CFG     0x443C021Cu
#define NAVQ95_I2C6_SCL_MUX     0x443C001Cu   /* GPIO_IO03 -> LPI2C6_SCL        */
#define NAVQ95_I2C6_SCL_DSY     0x443C0518u   /* SCL SELECT_INPUT               */
#define NAVQ95_I2C6_SCL_CFG     0x443C0220u
#define NAVQ95_I2C6_ALT         6u
#define NAVQ95_I2C6_DSY_VAL     0u            /* daisy selector = 0 (per table) */

/* LPI2C1 — dedicated I2C1_SCL/SDA pads, ALT 0, no daisy. */
#define NAVQ95_I2C1_SCL_MUX     0x443C01C0u   /* I2C1_SCL -> LPI2C1_SCL         */
#define NAVQ95_I2C1_SCL_CFG     0x443C03C4u
#define NAVQ95_I2C1_SDA_MUX     0x443C01C4u   /* I2C1_SDA -> LPI2C1_SDA         */
#define NAVQ95_I2C1_SDA_CFG     0x443C03C8u
#define NAVQ95_I2C1_ALT         0u

/*
 * Pad electrical CONFIG values (RAW SW_PAD_CTL values — see ENCODING below).
 * These match the PX4/NuttX MR-NavQ95B board pin-mux (board.h):
 *
 *   IOMUX_LPI2C_DEFAULT  = OD | FSEL_SFAST | DSE_X6           (LPI2C4/6/8)
 *   IOMUX_LPI2C1_DEFAULT = OD | PU_ON | FSEL_SFAST | DSE_X2   (LPI2C1 ONLY)
 *
 * IMPORTANT: LPI2C1 is special — it uses the pad's INTERNAL PULL-UP (PU_ON) and
 * a weaker drive (DSE_X2), whereas LPI2C6 (and the other I2C buses) use NO pull
 * and a strong drive (DSE_X6). The i2c1 net relies on that internal pull-up; if
 * we mux the i2c1 pad without PU, the line loses its pull-up and drops low once
 * i2cInit() runs (observed: high at reset, then low after boot). So the two
 * buses MUST use different pad values.
 *
 * MIMX9596 SW_PAD_CTL field layout (MIMX9596_cm7.h / NuttX imx9_iomuxc.h):
 *   DSE [6:1]  (X2 = 0x03<<1 = 0x06, X6 = 0x3f<<1 = 0x7E)
 *   FSEL1 [8:7] (slightly_fast = 2<<7 = 0x100)
 *   PU bit 9 (0x200), PD bit 10, OD bit 11 (0x800), HYS bit 12
 * input-enable is SION in the SW_MUX_CTL register (sion=1 arg to the pinmux
 * helpers), NOT a SW_PAD_CTL bit.
 *
 *   LPI2C6: raw = OD|FSEL_SFAST|DSE_X6      = 0x097E
 *   LPI2C1: raw = OD|PU|FSEL_SFAST|DSE_X2   = 0x0B06
 *
 * ENCODING: these are sent RAW (no >>1), matching Zephyr's SCMI pinctrl driver
 * (drivers/pinctrl/pinctrl_imx_scmi.c sends pin_ctrl_flags unshifted). The
 * earlier ">>1 pre-shift" assumption was wrong: sent unshifted, the old i2c1
 * value 0x0583 decodes to PD (pull-down) | no OD -> exactly the "line drops
 * low after init" symptom; the old i2c6 value 0x048F decoded to push-pull + PD,
 * which only worked because the external pull-ups overpower the weak PD.
 */
#define NAVQ95_I2C6_PADVAL      0x097Eu   /* OD | FSEL_SFAST | DSE_X6          */
#define NAVQ95_I2C1_PADVAL      0x0B06u   /* OD | PU_ON | FSEL_SFAST | DSE_X2  */

/*
 * ── Per-bus MCCR0 clock timing (PRESCALE = 2 -> 6 MHz tick) ───────────────
 *
 * Both buses share PRESCALE=2 (MCFGR1). The SCL rate model (fsl_lpi2c /
 * i.MX 95 RM) is:
 *   rate = (src / 2^PRESCALE) / (CLKLO + 1 + CLKHI + 1 + SCL_LATENCY)
 * with SCL_LATENCY = floor((2 + FILTSCL) / 2^PRESCALE) = 0 here.
 *
 * I2C6 (external pull-ups, DSE_X6): the driven low time and the pull-up rise
 * are both fast, so the measured SCL matches the nominal divisor. Target an
 * exact 100 kHz nominal: CLKLO=CLKHI=29 -> 6 MHz/(29+1+29+1)=100.0 kHz.
 * (Measured ~100 kHz on a logic analyzer — this bus is already correct.)
 *
 * I2C1 (dedicated pads, WEAK internal pull-up PU_ON + DSE_X2 like PX4's
 * IOMUX_LPI2C1_DEFAULT): the SCL *low* phase is driven precisely, but the
 * SCL *high* phase is extended by the slow RC rise of the weak internal
 * pull-up into the bus capacitance. On the logic analyzer this adds a roughly
 * FIXED ~1.0 us per SCL period, so the nominal-100 kHz divisor above measured
 * only 92.3 kHz (period 10.83 us instead of 10.0 us = +6 prescaled ticks).
 *
 * The rise overhead is pull-up/capacitance-limited and cannot be reduced by
 * drive strength (open-drain only drives LOW; the rise is done by the
 * pull-up). The clean hardware fix is an external pull-up on the I2C1 net;
 * absent that, we PRE-COMPENSATE the divisor so measured = 100 kHz:
 *   want measured period = 10.0 us = 60 ticks; fixed rise ~= 6 ticks
 *   => program 54 config ticks: CLKLO+1+CLKHI+1 = 54 => CLKLO=CLKHI=26
 *   nominal = 6 MHz/(26+1+26+1) = 111.1 kHz, measured ~= 100.0 kHz on I2C1.
 * SETHOLD/DATAVD scale with CLKHI (SETHOLD=CLKHI, DATAVD=CLKHI/2) as before.
 *
 * NOTE: this compensation assumes the ~1 us rise stays roughly constant. If
 * the I2C1 bus capacitance changes (more devices / longer traces) the measured
 * rate will drift; the robust fix remains an external pull-up on I2C1.
 */
/* I2C6: exact 100 kHz nominal. CLKLO=29 CLKHI=29 SETHOLD=29 DATAVD=14. */
#define NAVQ95_I2C6_MCCR0  (LPI2C_MCCR0_CLKLO(29u)   | LPI2C_MCCR0_CLKHI(29u) \
                          | LPI2C_MCCR0_SETHOLD(29u) | LPI2C_MCCR0_DATAVD(14u))
/* I2C1: 111 kHz nominal to compensate the ~1 us weak-pull-up rise so the
 * logic-analyzer-measured SCL lands at ~100 kHz. CLKLO=26 CLKHI=26. */
#define NAVQ95_I2C1_MCCR0  (LPI2C_MCCR0_CLKLO(26u)   | LPI2C_MCCR0_CLKHI(26u) \
                          | LPI2C_MCCR0_SETHOLD(26u) | LPI2C_MCCR0_DATAVD(13u))

/* ── Betaflight bus tables ──────────────────────────────────────────────── */

static volatile uint16_t i2cErrorCount = 0;

i2cDevice_t i2cDevice[I2CDEV_COUNT];

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    [I2CDEV_1] = {
        .device = I2CDEV_1,
        .reg    = (i2cResource_t *)I2C1,
    },
#endif
#ifdef USE_I2C_DEVICE_6
    [I2CDEV_6] = {
        .device = I2CDEV_6,
        .reg    = (i2cResource_t *)I2C6,
    },
#endif
};

/* ── Low-level register helpers ─────────────────────────────────────────── */

static inline volatile uint32_t *i2c_reg(uint32_t base, uint32_t off)
{
    return (volatile uint32_t *)(base + off);
}

/*
 * Mux the pads for one LPI2C instance over SCMI, then ungate its functional
 * clock. LPI2C6 uses shared GPIO pads that need the SELECT_INPUT daisy write;
 * LPI2C1 uses dedicated pads (no daisy). Returns the OR of the SCMI pinmux
 * statuses (0 = all success). The return is advisory only — the caller still
 * configures the controller regardless (mirrors the LPSPI1 bring-up, which
 * ignores the pinmux status).
 */
static int navq95_lpi2c_padmux(uint32_t base)
{
    int sclS, sdaS, rateS, clkS;

    if (base == (uint32_t)I2C6) {
        sclS = navq95_scmi_pinmux_daisy(NAVQ95_I2C6_SCL_MUX, NAVQ95_I2C6_ALT, 1u,
                                        NAVQ95_I2C6_SCL_DSY, NAVQ95_I2C6_DSY_VAL,
                                        NAVQ95_I2C6_PADVAL);
        sdaS = navq95_scmi_pinmux_daisy(NAVQ95_I2C6_SDA_MUX, NAVQ95_I2C6_ALT, 1u,
                                        NAVQ95_I2C6_SDA_DSY, NAVQ95_I2C6_DSY_VAL,
                                        NAVQ95_I2C6_PADVAL);
        /* Program the functional-clock rate THEN ungate it. */
        rateS = navq95_scmi_clock_rate_set(NAVQ95_LPI2C6_CLK_ROOT, NAVQ95_LPI2C_SRC_HZ);
        clkS  = navq95_scmi_clock_enable(NAVQ95_LPI2C6_CLK_ROOT);
    } else {
        /* LPI2C1 uses its own pad value WITH the internal pull-up (PU_ON) — the
         * i2c1 net needs it, else the line drops low after mux (see board.h). */
        sclS = navq95_scmi_pinmux(NAVQ95_I2C1_SCL_MUX, NAVQ95_I2C1_ALT, 1u,
                                  NAVQ95_I2C1_SCL_CFG, NAVQ95_I2C1_PADVAL);
        sdaS = navq95_scmi_pinmux(NAVQ95_I2C1_SDA_MUX, NAVQ95_I2C1_ALT, 1u,
                                  NAVQ95_I2C1_SDA_CFG, NAVQ95_I2C1_PADVAL);
        rateS = navq95_scmi_clock_rate_set(NAVQ95_LPI2C1_CLK_ROOT, NAVQ95_LPI2C_SRC_HZ);
        clkS  = navq95_scmi_clock_enable(NAVQ95_LPI2C1_CLK_ROOT);
    }

    return sclS | sdaS | rateS | clkS;
}

/*
 * Program the LPI2C master for ~100 kHz from the 24 MHz functional clock.
 *
 * PRESCALE = 2 (divide-by-4 -> 6 MHz tick) for both buses; the MCCR0 divisors
 * are selected PER BUS (see NAVQ95_I2C6_MCCR0 / NAVQ95_I2C1_MCCR0 above):
 *   I2C6 -> exact 100 kHz nominal (CLKLO=CLKHI=29); external pull-ups, so the
 *           measured rate matches the nominal.
 *   I2C1 -> 111 kHz nominal (CLKLO=CLKHI=26) to pre-compensate the weak
 *           internal-pull-up rise time so the *measured* SCL is ~100 kHz
 *           (nominal 100 kHz measured only 92.3 kHz on a logic analyzer).
 * The SCL rate model (fsl_lpi2c / i.MX 95 RM) is:
 *   rate = (src / 2^PRESCALE) / (CLKLO + 1 + CLKHI + 1 + SCL_LATENCY).
 *
 * MCFGR2 (glitch filters + bus-idle timeout) and MCFGR3 (pin-low timeout) are
 * cleared: a nonzero reset-default BUSIDLE/PINLOW timeout can stall the master
 * state machine so it never actually drives SCL (MBF stuck, no NDF/SDF).
 */
static void navq95_lpi2c_configure(uint32_t base)
{
    volatile uint32_t *mcr    = i2c_reg(base, LPI2C_MCR_OFFSET);
    volatile uint32_t *mcfgr1 = i2c_reg(base, LPI2C_MCFGR1_OFFSET);
    volatile uint32_t *mcfgr2 = i2c_reg(base, LPI2C_MCFGR2_OFFSET);
    volatile uint32_t *mcfgr3 = i2c_reg(base, LPI2C_MCFGR3_OFFSET);
    volatile uint32_t *mccr0  = i2c_reg(base, LPI2C_MCCR0_OFFSET);
    volatile uint32_t *mfcr   = i2c_reg(base, LPI2C_MFCR_OFFSET);

    /* Disable + software reset, then release. */
    *mcr = 0u;
    *mcr = LPI2C_MCR_RST | LPI2C_MCR_RTF | LPI2C_MCR_RRF;
    *mcr = 0u;

    /* PRESCALE = 2 (divide functional clock by 4 -> 6 MHz tick). */
    *mcfgr1 = LPI2C_MCFGR1_PRESCALE(2u);

    /* No glitch filters, no bus-idle timeout, no pin-low timeout. */
    *mcfgr2 = 0u;
    *mcfgr3 = 0u;

    /* Per-bus SCL timing: I2C1 is pre-compensated for its weak-pull-up rise. */
    *mccr0 = (base == (uint32_t)I2C1) ? NAVQ95_I2C1_MCCR0 : NAVQ95_I2C6_MCCR0;

    /* FIFO watermarks = 0 (we poll byte-by-byte). */
    *mfcr = 0u;

    /* Enable the master. */
    *mcr = LPI2C_MCR_MEN;
    (void)*mcr;
}

/* Clear all latched MSR error/status flags (write-1-to-clear). */
static inline void navq95_lpi2c_clear_status(uint32_t base)
{
    volatile uint32_t *msr = i2c_reg(base, LPI2C_MSR_OFFSET);
    *msr = LPI2C_MSR_W1C;
}


/*
 * Wait until the TX FIFO has space (TDF) or an error is latched. Returns false
 * on timeout or a latched bus error (NACK / arb-lost / FIFO error).
 */
static bool navq95_lpi2c_wait_txready(uint32_t base)
{
    volatile uint32_t *msr = i2c_reg(base, LPI2C_MSR_OFFSET);
    uint32_t spin = NAVQ95_LPI2C_SPIN;

    while (spin--) {
        uint32_t s = *msr;
        if (s & LPI2C_MSR_ERROR_MASK) {
            return false;
        }
        if (s & LPI2C_MSR_TDF) {
            return true;
        }
    }
    return false;
}

/* Push one MTDR command word, waiting for TX FIFO space first. */
static bool navq95_lpi2c_cmd(uint32_t base, uint32_t cmdWord)
{
    if (!navq95_lpi2c_wait_txready(base)) {
        return false;
    }
    *i2c_reg(base, LPI2C_MTDR_OFFSET) = cmdWord;
    return true;
}

/*
 * Issue a (repeated) START + address byte. rw = 0 write, 1 read. The 7-bit
 * address is shifted left and ORed with rw, written as the START command's
 * data byte. Clears stale status first.
 */
static bool navq95_lpi2c_start(uint32_t base, uint8_t addr7, uint8_t rw)
{
    navq95_lpi2c_clear_status(base);
    uint32_t data = (uint32_t)((addr7 << 1) | (rw & 1u)) & 0xFFu;
    return navq95_lpi2c_cmd(base, LPI2C_MTDR_CMD_START | data);
}

/*
 * Finish a transaction with a STOP and wait for completion. This must work both
 * after a clean transfer AND after a NACK/error: on a NACK the LPI2C latches NDF
 * and discards queued TX commands, so a STOP queued while the error is still
 * latched would be rejected (wait_txready() would see the error and bail). So:
 *   1) clear any latched error/status flags,
 *   2) queue the STOP (TX FIFO now accepts it),
 *   3) wait for STOP detect (SDF) or the master to go not-busy,
 *   4) reset the TX/RX FIFOs so the next transaction starts clean.
 * Returns true if a STOP completed (or the bus is already idle), false on
 * timeout. A NACK is reported by i2cWriteBuffer/i2cReadBuffer via the earlier
 * command failing, not here — this only guarantees the bus is released.
 */
static bool navq95_lpi2c_stop(uint32_t base)
{
    volatile uint32_t *msr = i2c_reg(base, LPI2C_MSR_OFFSET);
    volatile uint32_t *mcr = i2c_reg(base, LPI2C_MCR_OFFSET);

    /* Clear latched error/status so the TX FIFO will accept the STOP command. */
    navq95_lpi2c_clear_status(base);

    *i2c_reg(base, LPI2C_MTDR_OFFSET) = LPI2C_MTDR_CMD_STOP;

    /* Bounded wait for the STOP to complete. A STOP finishes in ~1 bus clock,
     * so a short budget is enough; on a wedged bus we fall through and recover
     * via the FIFO flush below (so the scan never hangs for long per probe). */
    uint32_t spin = NAVQ95_LPI2C_PROBE_SPIN;
    bool done = false;
    while (spin--) {
        uint32_t s = *msr;
        if (s & LPI2C_MSR_SDF) {           /* STOP detected */
            done = true;
            break;
        }
        if (!(s & LPI2C_MSR_MBF)) {        /* master no longer busy */
            done = true;
            break;
        }
    }

    /* Always clear status + flush FIFOs so the next transaction is clean, even
     * if the STOP timed out (prevents a wedged MBF carrying into the next one). */
    navq95_lpi2c_clear_status(base);
    *mcr = LPI2C_MCR_MEN | LPI2C_MCR_RTF | LPI2C_MCR_RRF;
    *mcr = LPI2C_MCR_MEN;

    return done;
}

/* ── Bank resolution ────────────────────────────────────────────────────── */

static uint32_t i2cBase(i2cDevice_e device)
{
    if ((unsigned)device >= I2CDEV_COUNT) {
        return 0;
    }
    return (uint32_t)i2cDevice[device].reg;
}

/* ── Betaflight generic I2C API (drivers/bus_i2c.h) ─────────────────────── */

void i2cPinConfigure(const i2cConfig_t *i2cConfig)
{
    /* Bind each configured hardware instance's handle into i2cDevice[] so
     * i2cInit()/i2cBase() can resolve the LPI2C base. Pin tags / clock speed
     * come from the PG config (target.h defaults); the actual pad mux is done
     * over SCMI in navq95_lpi2c_padmux() at init time, so we only record the
     * handle + options here (no STM32-style per-pin AF mux). */
    for (int index = 0; index < I2CDEV_COUNT; index++) {
        const i2cHardware_t *hw = &i2cHardware[index];
        if (!hw->reg) {
            continue;
        }
        i2cDevice_t *pDev = &i2cDevice[hw->device];
        memset(pDev, 0, sizeof(*pDev));
        pDev->hardware   = hw;
        pDev->reg        = hw->reg;
        pDev->pullUp     = i2cConfig[hw->device].pullUp;
        pDev->clockSpeed = i2cConfig[hw->device].clockSpeed;
    }
}

void i2cInit(i2cDevice_e device)
{
    if ((unsigned)device >= I2CDEV_COUNT) {
        return;
    }

    /* Bring up whichever bus we were handed. The padmux status is advisory;
     * always configure the controller (the LPSPI1 bring-up does the same). */
    uint32_t base = (uint32_t)i2cDevice[device].reg;
    if (base) {
        (void)navq95_lpi2c_padmux(base);
        navq95_lpi2c_configure(base);
    }

    /*
     * fc/init.c initPhase3() only calls i2cInit() for I2CDEV_0..I2CDEV_4, so
     * I2CDEV_6 (LPI2C6) would never be initialised on its own. Bring LPI2C6 up
     * here, piggy-backed on the I2CDEV_1 call, so both Phase-1 buses come up.
     * (Idempotent: configure() does a full reset each time.)
     */
#if defined(USE_I2C_DEVICE_1) && defined(USE_I2C_DEVICE_6)
    if (device == I2CDEV_1) {
        uint32_t base6 = (uint32_t)i2cDevice[I2CDEV_6].reg;
        if (base6) {
            (void)navq95_lpi2c_padmux(base6);
            navq95_lpi2c_configure(base6);
        }
    }
#endif
}

/*
 * Write `len_` bytes to register `reg_` of slave `addr_` on `device`:
 *   START+addr(W), [reg_], data[0..len_-1], STOP.
 * A NULL data pointer with len_ == 0 and reg_ == 0xFF issues an address-only
 * probe (START+addr(W), STOP). Returns true if the slave ACKed the whole
 * transfer.
 */
bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    uint32_t base = i2cBase(device);
    if (!base) {
        return false;
    }

    bool ok = navq95_lpi2c_start(base, addr_, 0u);

    if (ok && !(reg_ == 0xFFu && len_ == 0u && data == NULL)) {
        ok = navq95_lpi2c_cmd(base, LPI2C_MTDR_CMD_TXD | reg_);
        for (uint8_t i = 0; ok && i < len_; i++) {
            ok = navq95_lpi2c_cmd(base, LPI2C_MTDR_CMD_TXD | data[i]);
        }
    }

    /* Always issue STOP to release the bus, even on NACK. */
    bool stopped = navq95_lpi2c_stop(base);

    if (!ok || !stopped) {
        i2cErrorCount++;
        return false;
    }
    return true;
}

bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

/*
 * Read `len` bytes from register `reg` of slave `addr_` on `device`:
 *   START+addr(W), reg, REPEATED-START+addr(R), RXD(len), STOP.
 * Returns true if the slave ACKed and all bytes were received.
 */
bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint32_t base = i2cBase(device);
    if (!base || len == 0) {
        return false;
    }

    volatile uint32_t *msr = i2c_reg(base, LPI2C_MSR_OFFSET);
    volatile uint32_t *mrdr = i2c_reg(base, LPI2C_MRDR_OFFSET);

    bool ok = navq95_lpi2c_start(base, addr_, 0u);               /* write phase */
    if (ok) {
        ok = navq95_lpi2c_cmd(base, LPI2C_MTDR_CMD_TXD | reg);   /* register    */
    }
    if (ok) {
        ok = navq95_lpi2c_start(base, addr_, 1u);               /* repeated RD  */
    }
    if (ok) {
        /* Request len bytes (RXD command's byte field = count - 1). */
        ok = navq95_lpi2c_cmd(base, LPI2C_MTDR_CMD_RXD | (uint32_t)(len - 1));
    }

    for (uint8_t i = 0; ok && i < len; i++) {
        uint32_t spin = NAVQ95_LPI2C_SPIN;
        bool got = false;
        while (spin--) {
            uint32_t s = *msr;
            if (s & LPI2C_MSR_ERROR_MASK) {
                ok = false;
                break;
            }
            uint32_t rd = *mrdr;
            if (!(rd & LPI2C_MRDR_RXEMPTY)) {
                buf[i] = (uint8_t)(rd & 0xFFu);
                got = true;
                break;
            }
        }
        if (!got) {
            ok = false;
        }
    }

    bool stopped = navq95_lpi2c_stop(base);

    if (!ok || !stopped) {
        i2cErrorCount++;
        return false;
    }
    return true;
}

bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return i2cReadBuffer(device, addr_, reg, len, buf);
}

/*
 * Polled driver: a transfer never straddles i2cBusy() (each i2cRead/Write
 * completes synchronously), so we are never "busy" afterwards. Report the bus
 * busy flag only as a courtesy; error is cleared (callers that pass error get
 * the running error count via i2cGetErrorCounter()).
 */
bool i2cBusy(i2cDevice_e device, bool *error)
{
    uint32_t base = i2cBase(device);
    if (error) {
        *error = false;
    }
    if (!base) {
        return false;
    }
    return (*i2c_reg(base, LPI2C_MSR_OFFSET) & LPI2C_MSR_MBF) != 0u;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

/*
 * NOTE: i2cGetRegisteredDeviceCount() is NOT defined here. It is provided by
 * the common drivers/bus_i2c_busdev.c (which tracks the count as busDevices are
 * registered). Defining it in the platform file collides at link time
 * ("multiple definition of i2cGetRegisteredDeviceCount"), same as the PICO port
 * which only supplies i2cGetErrorCounter().
 */

#endif // defined(USE_I2C) && !defined(USE_SOFT_I2C)
