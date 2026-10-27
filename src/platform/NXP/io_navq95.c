/*
 * io_navq95.c — i.MX 95 (NavQ95, Cortex-M7) RGPIO IO primitives.
 *
 * Register-backed implementation of the Betaflight IO_* hardware primitives for
 * the i.MX 95 Rapid GPIO (RGPIO1..RGPIO5) blocks. The IO record keeping
 * (ioRecs[], IOInit/IOGetByTag/IOTraversePins, IO_GPIOPortIdx/PinIdx) lives in
 * platform_stubs.c / core io.c; this file only touches hardware.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"

/*
 * i.MX 95 RGPIO (Rapid GPIO) register access. Betaflight's IO_t is a pointer
 * into ioRecs[]; the 0-based gpioid (RGPIO1=0 .. RGPIO5=4) and pin are the
 * array index decomposed exactly as IO_GPIOPortIdx()/IO_GPIOPinIdx() do.
 *
 * RGPIO instance base addresses, from the i.MX 95 device header
 * (GPIO1_BASE..GPIO5_BASE). Index 0 is RGPIO1.
 */
static const uint32_t navq95_rgpioBase[5] = {
    0x47400000u,    /* RGPIO1 (GPIO1) */
    0x43810000u,    /* RGPIO2 (GPIO2) */
    0x43820000u,    /* RGPIO3 (GPIO3) */
    0x43840000u,    /* RGPIO4 (GPIO4) */
    0x43850000u,    /* RGPIO5 (GPIO5) */
};

/* RGPIO register offsets (i.MX 95 RGPIO block). */
#define RGPIO_PDOR_OFFSET  0x40u    /* Port Data Output                        */
#define RGPIO_PSOR_OFFSET  0x44u    /* Port Set Output      (write-1-to-set)   */
#define RGPIO_PCOR_OFFSET  0x48u    /* Port Clear Output    (write-1-to-clear) */
#define RGPIO_PTOR_OFFSET  0x4Cu    /* Port Toggle Output   (write-1-to-toggle)*/
#define RGPIO_PDIR_OFFSET  0x50u    /* Port Data Input                         */
#define RGPIO_PDDR_OFFSET  0x54u    /* Port Data Direction  (1 = output)       */

/* ── RGPIO per-pin access control (i.MX 95 attribute checker) ───────────────
 * The i.MX 95 RGPIO has a per-pin security/privilege filter on the data
 * registers. The RGPIO_Type layout (devices/MIMX9596/MIMX9596_cm7.h) places:
 *   PCNS (Pin Control Non-Secure)     at 0x10  — bit N: pin N is NON-secure
 *   PCNP (Pin Control Non-Privilege)  at 0x18  — bit N: pin N is NON-privileged
 * A write to PDOR/PSOR/PCOR/PTOR/PDDR is silently DROPPED (no bus fault, reads
 * return 0) when the accessing master's security/privilege level does not
 * MATCH the pin's attribute. The attribute is a match requirement, not a
 * permissive OR: a NON-secure pin (bit=1) rejects a SECURE master, and vice
 * versa.
 *
 * Our bare-metal Betaflight M7 runs SECURE/PRIVILEGED, same as NuttX's imx9
 * GPIO driver (arch/arm/src/imx9/imx9_gpio.c), which clears the PCNS bit for
 * every pin it configures. So the pin must be marked SECURE (PCNS bit = 0) and
 * PRIVILEGED (PCNP bit = 0); setting these non-secure locks the secure master
 * out (PDDR then reads back 0).
 */
#define RGPIO_PCNS_OFFSET  0x10u
#define RGPIO_PCNP_OFFSET  0x18u

/* ── IOMUXC (i.MX 95) ───────────────────────────────────────────────────────
 * A GPIO pad needs two IOMUXC writes:
 *   SW_MUX_CTL  — selects the pad's ALT function (GPIO alt for GPIO_IO* = ALT0)
 *   SW_PAD_CTL  — the electrical config (drive strength, pull, open-drain, ...)
 * The pad's ALT and the two register addresses are pad-specific and come from
 * the per-pad table below (values from devices/MIMX9596/drivers/fsl_iomuxc.h,
 * whose macros are { muxReg, muxMode, inputReg, inputDaisy, cfgReg }). The
 * SW_PAD_CTL *value* is no longer a constant: it is derived from the Betaflight
 * ioConfig_t by navq95_padCtlFromCfg() so every pin honours its requested
 * direction / pull / output type. */
/* SW_MUX_CTL fields (mask / shift / field-build macro — same style as the
 * MCUXpresso SDK devices/MIMX9596/drivers/fsl_iomuxc.h). */
#define IOMUXC_PAD_MUX_MODE_MASK  (0x7U)
#define IOMUXC_PAD_MUX_MODE_SHIFT (0U)
#define IOMUXC_PAD_MUX_MODE(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_MUX_MODE_SHIFT)) & IOMUXC_PAD_MUX_MODE_MASK)
#define IOMUXC_PAD_SION_MASK      (0x10U)
#define IOMUXC_PAD_SION_SHIFT     (4U)
#define IOMUXC_PAD_SION(x)        (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_SION_SHIFT)) & IOMUXC_PAD_SION_MASK)

/* SW_PAD_CTL fields (i.MX 95; from fsl_iomuxc.h IOMUXC_PAD_* masks):
 *   DSE   bits[6:1]  drive strength — 0 DISABLES the output driver
 *   FSEL1 bits[8:7]  slew / fast-select
 *   PU    bit 9      pull-up enable
 *   PD    bit 10     pull-down enable
 *   OD    bit 11     open-drain
 *   HYS   bit 12     input hysteresis (needed to read an input/open-drain pad)
 *   APC     bits[27:24] domain access (APC) field
 *   APC_LCK bits[31:28] APC lock bits
 * NOTE: the SDK header ships IOMUXC_PAD_HYS_SHIFT as 11 while IOMUXC_PAD_HYS_MASK
 * is 0x1000 (bit 12) — an inconsistency. The mask (bit 12) is authoritative, so
 * HYS_SHIFT is 12U here to keep IOMUXC_PAD_HYS(x) functional. */
#define IOMUXC_PAD_DSE_MASK    (0x7EU)
#define IOMUXC_PAD_DSE_SHIFT   (1U)
#define IOMUXC_PAD_DSE(x)      (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_DSE_SHIFT)) & IOMUXC_PAD_DSE_MASK)
#define IOMUXC_PAD_FSEL1_MASK  (0x180U)
#define IOMUXC_PAD_FSEL1_SHIFT (7U)
#define IOMUXC_PAD_FSEL1(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_FSEL1_SHIFT)) & IOMUXC_PAD_FSEL1_MASK)
#define IOMUXC_PAD_PU_MASK     (0x200U)
#define IOMUXC_PAD_PU_SHIFT    (9U)
#define IOMUXC_PAD_PU(x)       (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_PU_SHIFT)) & IOMUXC_PAD_PU_MASK)
#define IOMUXC_PAD_PD_MASK     (0x400U)
#define IOMUXC_PAD_PD_SHIFT    (10U)
#define IOMUXC_PAD_PD(x)       (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_PD_SHIFT)) & IOMUXC_PAD_PD_MASK)
#define IOMUXC_PAD_OD_MASK     (0x800U)
#define IOMUXC_PAD_OD_SHIFT    (11U)
#define IOMUXC_PAD_OD(x)       (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_OD_SHIFT)) & IOMUXC_PAD_OD_MASK)
#define IOMUXC_PAD_HYS_MASK    (0x1000U)
#define IOMUXC_PAD_HYS_SHIFT   (12U)
#define IOMUXC_PAD_HYS(x)      (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_HYS_SHIFT)) & IOMUXC_PAD_HYS_MASK)
#define IOMUXC_PAD_APC_MASK     (0xF000000U)
#define IOMUXC_PAD_APC_SHIFT    (24U)
#define IOMUXC_PAD_APC(x)       (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_APC_SHIFT)) & IOMUXC_PAD_APC_MASK)
#define IOMUXC_PAD_APC_LCK_MASK  (0xF0000000U)
#define IOMUXC_PAD_APC_LCK_SHIFT (28U)
#define IOMUXC_PAD_APC_LCK(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_PAD_APC_LCK_SHIFT)) & IOMUXC_PAD_APC_LCK_MASK)

/* Betaflight ioConfig_t packing (NAVQ95 platform.h):
 *   IO_CONFIG(mode, speed, pupd) = mode | (speed<<2) | (pupd<<5)
 * mode  bits[1:0]  GPIO_MODE_INPUT/OUTPUT/AF/ANALOG
 * speed bits[4:2]  GPIO_SPEED_LOW/HIGH  (mapped to a DSE level)
 * pupd  bits[6:5]  GPIO_NOPULL/PULLUP/PULLDOWN
 * Open-drain is not separately encoded by the IOCFG_* macros (IOCFG_OUT_OD ==
 * IOCFG_OUT_PP in this platform), so push-pull is assumed for outputs. */
#define NAVQ95_CFG_MODE(cfg)   ((cfg) & 0x03u)
#define NAVQ95_CFG_SPEED(cfg)  (((cfg) >> 2) & 0x07u)
#define NAVQ95_CFG_PUPD(cfg)   (((cfg) >> 5) & 0x03u)

/* Build a SW_PAD_CTL value from an ioConfig_t.
 *  - Outputs get a non-zero DSE (required: DSE 0 = driver off); HIGH speed uses
 *    a stronger drive than LOW.
 *  - Inputs get DSE 0 (no output drive) plus hysteresis so the level is
 *    readable via PDIR.
 *  - Pull-up/down from the pupd field. */
static uint32_t navq95_padCtlFromCfg(ioConfig_t cfg)
{
    uint32_t pad = 0u;

    if (NAVQ95_CFG_MODE(cfg) == GPIO_MODE_OUTPUT) {
        pad |= IOMUXC_PAD_DSE(NAVQ95_CFG_SPEED(cfg) == GPIO_SPEED_HIGH ? 0x3Fu : 0x0Fu);
    } else {
        pad |= IOMUXC_PAD_HYS(1u);  /* input: enable the input buffer / schmitt */
    }

    switch (NAVQ95_CFG_PUPD(cfg)) {
    case GPIO_PULLUP:   pad |= IOMUXC_PAD_PU(1u); break;
    case GPIO_PULLDOWN: pad |= IOMUXC_PAD_PD(1u); break;
    default:            break;      /* no pull */
    }

    return pad;
}

typedef struct {
    int      port;        /* 0-based RGPIO index (RGPIO2 = 1)                 */
    uint32_t pin;         /* bit number within the port                      */
    uint32_t muxReg;      /* IOMUXC_SW_MUX_CTL_PAD_* absolute address        */
    uint32_t muxMode;     /* alt value selecting the GPIO/RGPIO function     */
    uint32_t cfgReg;      /* IOMUXC_SW_PAD_CTL_PAD_* address (0 = skip)      */
} navq95_padMux_t;

/* Per-pad mux map: ALT + the SW_MUX_CTL / SW_PAD_CTL register addresses. The
 * SW_PAD_CTL value is computed from ioConfig_t at configure time, not stored.
 * Keyed by the (port, pin) that navq95_ioResolve() derives from the IO_t.
 * Extend one row per pad as they are brought up. */
static const navq95_padMux_t navq95_padMux[] = {
    /* GPIO_IO23 -> buzzer. Pad GPIO_IO23 = RGPIO2 bit 23 (GPIO2 = PB), reclaimed
     * as a plain GPIO (board netname TPM6_CH1). Uses the pad's GPIO ALT function
     * (ALT0).
     * From the i.MX 95 IOMUXC table
     * (IOMUXC_PAD_GPIO_IO23__GPIO2_IO_BIT23 = { muxReg, muxMode, inputReg,
     *  inputDaisy, cfgReg } = 0x443C006C, 0x0, 0x0, 0x0, 0x443C0270):
     *   SW_MUX_CTL_PAD_GPIO_IO23 = 0x443C006C, GPIO ALT muxMode = 0x0
     *   SW_PAD_CTL_PAD_GPIO_IO23 = 0x443C0270 */
    { 1, 23u, 0x443C006Cu, 0x0u, 0x443C0270u },
    /* add more pads here as they are brought up */
};

/* Program a pad's mux (ALT) and pad-ctl (derived from cfg) from the table.
 * Pads without a table entry are left as U-Boot/the System Manager configured
 * them (mux/pad untouched) — only the RGPIO direction below applies. */
static void navq95_ioMuxConfig(int port, uint32_t pin, ioConfig_t cfg)
{
    for (size_t i = 0; i < sizeof(navq95_padMux) / sizeof(navq95_padMux[0]); i++) {
        const navq95_padMux_t *p = &navq95_padMux[i];
        if (p->port == port && (int)p->pin == (int)pin) {
            *(volatile uint32_t *)p->muxReg = IOMUXC_PAD_MUX_MODE(p->muxMode);
            if (p->cfgReg) {
                *(volatile uint32_t *)p->cfgReg = navq95_padCtlFromCfg(cfg);
            }
            return;
        }
    }
}

/* Resolve an IO_t to its RGPIO base + bit mask. Returns false for a NULL IO_t
 * or an out-of-range port index. */
static bool navq95_ioResolve(IO_t io, volatile uint8_t **base, uint32_t *mask)
{
    int port = IO_GPIOPortIdx(io);
    int pin = IO_GPIOPinIdx(io);
    if (port < 0 || port >= (int)(sizeof(navq95_rgpioBase) / sizeof(navq95_rgpioBase[0]))) {
        return false;
    }
    *base = (volatile uint8_t *)navq95_rgpioBase[port];
    *mask = (1u << (uint32_t)pin);
    return true;
}

static inline volatile uint32_t *navq95_ioReg(volatile uint8_t *base, uint32_t offset)
{
    return (volatile uint32_t *)(base + offset);
}

bool IORead(IO_t io)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return false;
    }
    return (*navq95_ioReg(base, RGPIO_PDIR_OFFSET) & mask) != 0u;
}

void IOWrite(IO_t io, bool value)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return;
    }
    *navq95_ioReg(base, value ? RGPIO_PSOR_OFFSET : RGPIO_PCOR_OFFSET) = mask;
}

void IOHi(IO_t io)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return;
    }
    *navq95_ioReg(base, RGPIO_PSOR_OFFSET) = mask;
}

void IOLo(IO_t io)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return;
    }
    *navq95_ioReg(base, RGPIO_PCOR_OFFSET) = mask;
}

void IOToggle(IO_t io)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return;
    }
    *navq95_ioReg(base, RGPIO_PTOR_OFFSET) = mask;
}

/*
 * Configure a pin end-to-end from the Betaflight ioConfig_t:
 *   1. IOMUXC: route the pad to its GPIO ALT and program SW_PAD_CTL (drive
 *      strength / pull / input buffer) derived from cfg (per-pad entries only;
 *      pads with no table row keep the mux U-Boot/the SM left them in).
 *   2. RGPIO attribute: mark the pin SECURE + PRIVILEGED (PCNS/PCNP bit = 0) to
 *      match our secure M7, else the attribute checker drops data-reg writes.
 *   3. RGPIO direction: PDDR = output for GPIO_MODE_OUTPUT, input otherwise.
 */
void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    volatile uint8_t *base;
    uint32_t mask;
    if (!navq95_ioResolve(io, &base, &mask)) {
        return;
    }

    navq95_ioMuxConfig(IO_GPIOPortIdx(io), (uint32_t)IO_GPIOPinIdx(io), cfg);

    /* Match the pin's security/privilege attribute to the master that drives
     * it. The RGPIO attribute checker drops a data-register write when the
     * accessing master's security state does not MATCH the pin's PCNS/PCNP
     * attribute. Our Betaflight M7 runs secure/privileged (like NuttX's imx9
     * driver, which clears PCNS for the pin), so the pin must be marked SECURE
     * (PCNS bit = 0) and PRIVILEGED (PCNP bit = 0). */
    *navq95_ioReg(base, RGPIO_PCNS_OFFSET) &= ~mask;   /* pin = secure     */
    *navq95_ioReg(base, RGPIO_PCNP_OFFSET) &= ~mask;   /* pin = privileged */

    volatile uint32_t *pddr = navq95_ioReg(base, RGPIO_PDDR_OFFSET);
    if (NAVQ95_CFG_MODE(cfg) == GPIO_MODE_OUTPUT) {
        *pddr |= mask;
    } else {
        *pddr &= ~mask;
    }
}
#ifdef USE_TIMER_AF
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af) { (void)io; (void)cfg; (void)af; }
#endif

/* ── IO record keeping ──────────────────────────────────────────────────────
 * Like the STM32 port (io_stm32.c), the platform IO file owns the whole IO
 * layer, including the IO_t record lookup/init. ioRecs[] itself is defined by
 * the common io.c; here we only expose the platform-specific tag->record
 * mapping and the (empty) global init hook. */

void IOInitGlobal(void)
{
    /* ioRecs[] is a zero-initialised BSS array owned by common io.c; the
     * i.MX 95 RGPIO ports need no per-port clock/reset enable before use
     * (U-Boot / the System Manager leave them clocked), so there is nothing
     * platform-specific to do here. */
}

IO_t IOGetByTag(ioTag_t tag)
{
    extern ioRec_t ioRecs[];
    if (!tag) {
        return NULL;
    }
    return &ioRecs[DEFIO_TAG_GPIOID(tag) * DEFIO_PORT_PINS + DEFIO_TAG_PIN(tag)];
}
