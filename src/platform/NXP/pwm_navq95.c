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
 * NavQ95 (i.MX 95 M7) motor PWM output via TPM. Standard (analog) servo PWM;
 * DShot is a later upgrade. Per the board schematic the 4 motors are mixed
 * across two TPM instances (pad table below): M1/M2/M3 on TPM3, M4 on TPM4.
 * TPM3 (and TPM1) have no configurable root — they count the WAKEUPMIX bus
 * clock (not SCMI-enabled); TPM4 has its own root and is SCMI clock-enabled.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_PWM_OUTPUT) && defined(USE_MOTOR)

#include "common/maths.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/motor_types.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_impl.h"
#include "drivers/time.h"

#include "pg/motor.h"

#include "platform/tpm_regs.h"

/* Platform SCMI helpers (exported from platform_stubs.c). */
extern int  navq95_scmi_pinmux_ex(uint32_t muxReg, uint32_t muxMode, uint32_t sion,
                                  uint32_t padReg, uint32_t padVal);
extern int  navq95_scmi_clock_enable_ex(uint32_t clockRootId);
extern int  navq95_scmi_clock_rate_set_ex(uint32_t clockRootId, uint32_t rateHz);
extern void navq95_console_puts(const char *s);
extern void navq95_console_puthex(uint32_t v);
extern void navq95_console_drain(void);

/* TPM instances. TPM3 is bus-clocked (no settable root); TPM4 has CCM root 112
 * and is SCMI clock-enabled + RATE_SET to 1 MHz. The two run at different input
 * clocks, so MOD and us->ticks are derived per-timer. */
#define NAVQ95_TPM3                 ((TPM_Type *)0x424E0000u)
#define NAVQ95_TPM4                 ((TPM_Type *)0x424F0000u)
#define NAVQ95_TPM4_CLK_ROOT        112u
#define NAVQ95_TPM4_CLK_HZ          1000000u    /* TPM4 own root, set via RATE_SET */

/* TPM3 WAKEUPMIX bus clock ~135 MHz (confirmed on HW: MOD-for-480Hz gave 28 kHz
 * -> 28k*(MOD+1) ~= 135 MHz). The navq95TpmMeasureHz micros()-window reading is
 * off by ~58x here, so it is NOT used on the live path (selftest-only); hardcode
 * the real rate. MIN/MAX bound the selftest's validated fallback. */
#define NAVQ95_TPM3_BUS_HZ          135000000u
#define NAVQ95_TPM3_BUS_MIN         1500000u
#define NAVQ95_TPM3_BUS_MAX         3500000u

/* Output pad value (pre-shifted >>1): FSEL slow | DSE_X6 | PD_ON = raw 0x40C.
 * Pull-down keeps the ESC line low if the pad floats at boot; SION on. */
#define IOMUXC_BASE                 0x443C0000u
#define NAVQ95_TPM_PADVAL           (0x40Cu >> 1)
#define NAVQ95_TPM_SION             1u

typedef struct {
    TPM_Type *tpm;        /* which TPM instance drives this motor */
    uint32_t  muxReg;
    uint32_t  muxMode;    /* ALT value (per-pad) */
    uint32_t  padReg;
    uint8_t   channel;    /* channel index within that TPM (0..3) */
} navq95PwmPad_t;

/* Order = motor index 0..3 (board schematic). */
static const navq95PwmPad_t navq95PwmPads[MAX_SUPPORTED_MOTORS] = {
    { NAVQ95_TPM3, IOMUXC_BASE + 0x0020u, 1u, IOMUXC_BASE + 0x0224u, 0 }, /* M1 IO04 TPM3_CH0 */
    { NAVQ95_TPM3, IOMUXC_BASE + 0x0060u, 6u, IOMUXC_BASE + 0x0264u, 1 }, /* M2 IO20 TPM3_CH1 */
    { NAVQ95_TPM3, IOMUXC_BASE + 0x0070u, 4u, IOMUXC_BASE + 0x0274u, 3 }, /* M3 IO24 TPM3_CH3 */
    { NAVQ95_TPM4, IOMUXC_BASE + 0x0024u, 1u, IOMUXC_BASE + 0x0228u, 0 }, /* M4 IO05 TPM4_CH0 */
};

static bool navq95PwmClockEnabled;

/* Per-timer input clock + per-motor us->tick scale (filled in motorPwmDevInit).
 * TPM3 and TPM4 run at different clocks, so each motor scales to its own. */
static uint32_t navq95Tpm3ClkHz = NAVQ95_TPM3_BUS_HZ;
static float    navq95MotorTicksPerUs[MAX_SUPPORTED_MOTORS];

/* Measure a TPM input clock (free-run, count CNT over a 20 ms micros() window).
 * DIAGNOSTIC ONLY — unreliable in full init() (micros() timebase skew), so the
 * live path uses the hardcoded bus rate instead. */
#ifdef NAVQ95_PWM_SELFTEST
static uint32_t navq95TpmMeasureHz(TPM_Type *tpm)
{
    tpm->GLOBAL |= TPM_GLOBAL_RST;
    for (volatile int d = 0; d < 10000; d++) { }
    tpm->GLOBAL &= ~TPM_GLOBAL_RST;
    for (volatile int d = 0; d < 10000; d++) { }

    tpm->SC  = 0;
    tpm->CNT = 0;
    tpm->MOD = 0xFFFFu;
    tpm->SC  = TPM_SC_CMOD(1) | TPM_SC_PS(0);

    const uint32_t t0 = micros();
    while ((micros() - t0) < 20000u) { }
    const uint32_t ticks = tpm->CNT;

    tpm->SC = 0;
    return ticks * 50u;                            /* ticks / 0.02 s */
}

/* Measured TPM3 rate, clamped to a sane window with a hardcoded fallback. */
static uint32_t navq95Tpm3Hz(void)
{
    uint32_t hz = navq95TpmMeasureHz(NAVQ95_TPM3);
    if (hz < NAVQ95_TPM3_BUS_MIN || hz > NAVQ95_TPM3_BUS_MAX) {
        hz = NAVQ95_TPM3_BUS_HZ;
    }
    return hz;
}
#endif /* NAVQ95_PWM_SELFTEST */

/* Microseconds -> TPM compare ticks for a motor's (post-prescaler) timer clock. */
static inline uint16_t navq95PwmUsToTicks(uint8_t index, uint32_t us)
{
    return (uint16_t)((float)us * navq95MotorTicksPerUs[index] + 0.5f);
}

/* Init a TPM for edge-aligned high-true PWM at pwmHz. The counter is 16-bit, so
 * pick a prescaler (PS 0..7) making MOD = clkHz/2^PS/pwmHz fit in 16 bits.
 * Returns the EFFECTIVE (post-prescaler) clock so the caller scales us->ticks. */
static uint32_t navq95TpmInit(TPM_Type *tpm, uint32_t clkHz, uint32_t pwmHz)
{
    tpm->GLOBAL |= TPM_GLOBAL_RST;
    for (volatile int d = 0; d < 10000; d++) { }
    tpm->GLOBAL &= ~TPM_GLOBAL_RST;
    for (volatile int d = 0; d < 10000; d++) { }

    tpm->SC   = 0;
    tpm->CNT  = 0;
    tpm->POL  = 0;

    const uint32_t hz = pwmHz ? pwmHz : 1u;

    /* Smallest prescaler that makes MOD fit in 16 bits. */
    uint32_t ps = 0u;
    while (ps < 7u && (clkHz >> ps) / hz > 0xFFFFu) {
        ps++;
    }

    const uint32_t effHz = clkHz >> ps;
    uint32_t mod = effHz / hz;
    if (mod == 0u) {
        mod = 1u;
    }
    if (mod > 0x10000u) {
        mod = 0x10000u;
    }
    tpm->MOD = (uint16_t)(mod - 1u);

    tpm->SC = TPM_SC_CMOD(1) | TPM_SC_PS(ps);

    return effHz;
}

/* Configure one TPM channel for edge-aligned high-true PWM at an initial width. */
static void navq95TpmChannelInit(TPM_Type *tpm, uint8_t ch, uint16_t initialTicks)
{
    tpm->CONTROLS[ch].CnV  = initialTicks;
    tpm->CONTROLS[ch].CnSC = TPM_CnSC_PWM_HIGH;
}

/* Betaflight platform PWM hooks (drivers/pwm_output.h). channel->ccr points
 * straight at the TPM CnV register, so a write is a single store. */

void pwmOutputConfig(timerChannel_t *channel, const timerHardware_t *timerHardware,
                     uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
    (void)timerHardware; (void)hz; (void)period; (void)inversion;
    if (channel->ccr) {
        *channel->ccr = value;
    }
}

void pwmWriteChannel(timerChannel_t *channel, uint32_t value)
{
    if (channel->ccr) {
        *channel->ccr = value;
    }
}

/* Motor vTable. */

static float navq95MotorConvertToMotor(uint16_t external)
{
    return (float)external;               /* 1:1 (microseconds) */
}

static uint16_t navq95MotorConvertToExternal(float motor)
{
    return (uint16_t)(motor + 0.5f);
}

static bool navq95MotorEnable(void)
{
    return true;
}

static void navq95MotorDisable(void) { }

static bool navq95MotorIsEnabled(unsigned index)
{
    return index < pwmMotorCount && pwmMotors[index].enabled;
}

static void navq95MotorWriteInt(uint8_t index, uint16_t value)
{
    if (index < pwmMotorCount && pwmMotors[index].channel.ccr) {
        *pwmMotors[index].channel.ccr = navq95PwmUsToTicks(index, value);
    }
}

static void navq95MotorWrite(uint8_t index, float value)
{
    if (index < pwmMotorCount && pwmMotors[index].channel.ccr) {
        *pwmMotors[index].channel.ccr = navq95PwmUsToTicks(index, (uint32_t)(value + 0.5f));
    }
}

static void navq95MotorUpdateInit(void) { }
static void navq95MotorUpdateComplete(void) { }

static void navq95MotorShutdown(void)
{
    for (unsigned i = 0; i < pwmMotorCount; i++) {
        if (pwmMotors[i].channel.ccr) {
            *pwmMotors[i].channel.ccr = 0;
        }
    }
}

static bool navq95MotorIsIdle(unsigned index)
{
    return index < pwmMotorCount &&
           pwmMotors[index].channel.ccr && *pwmMotors[index].channel.ccr == 0;
}

static IO_t navq95MotorGetIO(unsigned index)
{
    return pwmGetMotorIO(index);
}

static void navq95MotorReleaseIO(unsigned index) { (void)index; }
static bool navq95MotorReinstateIO(unsigned index) { (void)index; return false; }
static void navq95MotorPostInit(void) { }
static void navq95MotorRequestTelemetry(unsigned index) { (void)index; }
static bool navq95MotorDecodeTelemetry(void) { return true; }
static bool navq95MotorTelemetryWait(void) { return false; }

static const motorVTable_t navq95MotorVTable = {
    .postInit               = navq95MotorPostInit,
    .convertExternalToMotor = navq95MotorConvertToMotor,
    .convertMotorToExternal = navq95MotorConvertToExternal,
    .enable                 = navq95MotorEnable,
    .disable                = navq95MotorDisable,
    .isMotorEnabled         = navq95MotorIsEnabled,
    .telemetryWait          = navq95MotorTelemetryWait,
    .decodeTelemetry        = navq95MotorDecodeTelemetry,
    .updateInit             = navq95MotorUpdateInit,
    .write                  = navq95MotorWrite,
    .writeInt               = navq95MotorWriteInt,
    .updateComplete         = navq95MotorUpdateComplete,
    .shutdown               = navq95MotorShutdown,
    .isMotorIdle            = navq95MotorIsIdle,
    .getMotorIO             = navq95MotorGetIO,
    .releaseMotorIO         = navq95MotorReleaseIO,
    .reinstateMotorIO       = navq95MotorReinstateIO,
    .requestTelemetry       = navq95MotorRequestTelemetry,
};

/* motorPwmDevInit — platform entry for standard PWM motors. */

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t idlePulse)
{
    if (!device) {
        return false;
    }

    const uint8_t count = device->count;
    const uint32_t pwmHz = motorConfig->motorPwmRate ? motorConfig->motorPwmRate : 480u;

    /* TPM4 has its own root -> SCMI clock-enable + RATE_SET to 1 MHz (a bare
     * enable leaves it at an arbitrary SM-default rate). TPM3 is bus-clocked ->
     * no enable; read VERID (nonzero) to confirm it responds before touching it
     * (crash-safe, no bus hang). */
    int clk4 = navq95_scmi_clock_enable_ex(NAVQ95_TPM4_CLK_ROOT);
    navq95_scmi_clock_rate_set_ex(NAVQ95_TPM4_CLK_ROOT, NAVQ95_TPM4_CLK_HZ);
    bool tpm3Ok = (NAVQ95_TPM3->VERID != 0u);
    navq95PwmClockEnabled = (clk4 == 0) && tpm3Ok;
    if (clk4 != 0 || !tpm3Ok) {
        device->vTable = NULL;
        return false;
    }

    /* TPM3 = hardcoded ~135 MHz bus clock; TPM4 = 1 MHz. navq95TpmInit picks a
     * prescaler per timer and returns the effective clock for us->ticks scaling. */
    navq95Tpm3ClkHz = NAVQ95_TPM3_BUS_HZ;

    const uint32_t tpm3Eff = navq95TpmInit(NAVQ95_TPM3, navq95Tpm3ClkHz, pwmHz);
    const uint32_t tpm4Eff = navq95TpmInit(NAVQ95_TPM4, NAVQ95_TPM4_CLK_HZ, pwmHz);

    const uint32_t idleUs = idlePulse ? idlePulse : 1000u;

    pwmMotorCount = 0;
    for (uint8_t i = 0; i < count && i < MAX_SUPPORTED_MOTORS; i++) {
        const navq95PwmPad_t *p = &navq95PwmPads[i];

        navq95_scmi_pinmux_ex(p->muxReg, p->muxMode, NAVQ95_TPM_SION,
                              p->padReg, NAVQ95_TPM_PADVAL);

        /* Per-motor us->tick scale from this motor's EFFECTIVE timer clock. */
        const uint32_t effHz = (p->tpm == NAVQ95_TPM4) ? tpm4Eff : tpm3Eff;
        navq95MotorTicksPerUs[i] = (float)effHz / 1000000.0f;

        navq95TpmChannelInit(p->tpm, p->channel, navq95PwmUsToTicks(i, idleUs));

        pwmMotors[i].channel.ccr = &p->tpm->CONTROLS[p->channel].CnV;
        pwmMotors[i].channel.tim = NULL;
        pwmMotors[i].pulseScale  = 1.0f;
        pwmMotors[i].pulseOffset = 0.0f;
        pwmMotors[i].forceOverflow = false;
        pwmMotors[i].enabled     = true;
        pwmMotors[i].io          = IO_NONE;
        pwmMotorCount++;
    }

    device->vTable = &navq95MotorVTable;
    device->initialized = true;
    device->enabled = false;
    device->motorEnableTimeMs = 0;

    return true;
}

/* Boot self-test (optional, guard NAVQ95_PWM_SELFTEST). Prints the TPM state
 * and drives a 1500 us pulse on all 4 channels so the pads can be scoped.
 * Bench only (props OFF) — it drives the ESC lines. */
#ifdef NAVQ95_PWM_SELFTEST
void navq95_pwm_selftest(void)
{
    navq95_console_puts("\n[PWM selftest]\n");

    /* TPM clock-grant scan: which TPM roots the SM lets the M7 enable
     * (0=granted, 0xfffffffd=-3 DENIED). TPM1/TPM3 are bus-clocked read-only
     * roots, so DENIED there is expected and fine. */
    {
        static const struct { const char *name; uint32_t base; uint32_t root; } tpms[] = {
            { "TPM1", 0x44310000u,  2u },
            { "TPM2", 0x44320000u, 19u },
            { "TPM3", 0x424E0000u, 78u },
            { "TPM4", 0x424F0000u, 112u },
            { "TPM5", 0x42500000u, 113u },
            { "TPM6", 0x42510000u, 114u },
        };
        navq95_console_puts("  TPM clock-grant scan (0=granted, 0xfffffffd=DENIED;"
                            " TPM3 denied is EXPECTED/OK — bus-clocked):\n");
        for (unsigned i = 0; i < sizeof(tpms) / sizeof(tpms[0]); i++) {
            int r = navq95_scmi_clock_enable_ex(tpms[i].root);
            navq95_console_puts("    ");
            navq95_console_puts(tpms[i].name);
            navq95_console_puts(" root=");
            navq95_console_puthex(tpms[i].root);
            navq95_console_puts(" clk=");
            navq95_console_puthex((uint32_t)r);
            navq95_console_puts("\n");
            navq95_console_drain();
        }
    }

    /* M1/M2/M3 on TPM3 (bus-clocked, read VERID to confirm it responds); M4 on
     * TPM4 (own root, SCMI clock-enable + RATE_SET to 1 MHz). */
    int clk4 = navq95_scmi_clock_enable_ex(NAVQ95_TPM4_CLK_ROOT);
    navq95_scmi_clock_rate_set_ex(NAVQ95_TPM4_CLK_ROOT, NAVQ95_TPM4_CLK_HZ);
    uint32_t tpm3Verid = NAVQ95_TPM3->VERID;
    navq95_console_puts("  TPM4 clk=");
    navq95_console_puthex((uint32_t)clk4);
    navq95_console_puts(" TPM3 VERID=");
    navq95_console_puthex(tpm3Verid);
    navq95_console_puts(" (nonzero=bus-clocked OK)\n");
    navq95_console_drain();
    if (clk4 != 0 || tpm3Verid == 0u) {
        navq95_console_puts("  TPM4 denied or TPM3 unresponsive -> skipping register"
                            " access (no bus hang).\n");
        navq95_console_puts("[/PWM selftest]\n");
        navq95_console_drain();
        return;
    }

    /* Print the (skewed) runtime measurement for reference, but DRIVE with the
     * hardcoded ~135 MHz bus rate so the selftest matches the live path. */
    navq95_console_puts("  TPM3 Hz measured=");
    navq95_console_puthex(navq95Tpm3Hz());
    navq95_console_puts(" used=");
    navq95_console_puthex(NAVQ95_TPM3_BUS_HZ);
    navq95_console_puts(" (measurement is ~58x low — not used)\n");
    navq95_console_drain();
    navq95Tpm3ClkHz = NAVQ95_TPM3_BUS_HZ;

    const uint32_t tpm3Eff = navq95TpmInit(NAVQ95_TPM3, navq95Tpm3ClkHz, 480u);
    const uint32_t tpm4Eff = navq95TpmInit(NAVQ95_TPM4, NAVQ95_TPM4_CLK_HZ, 480u);
    navq95_console_puts("  TPM3 MOD=");
    navq95_console_puthex(NAVQ95_TPM3->MOD);
    navq95_console_puts(" SC=");
    navq95_console_puthex(NAVQ95_TPM3->SC);
    navq95_console_puts("  TPM4 MOD=");
    navq95_console_puthex(NAVQ95_TPM4->MOD);
    navq95_console_puts(" SC=");
    navq95_console_puthex(NAVQ95_TPM4->SC);

    for (unsigned i = 0; i < 4; i++) {
        const navq95PwmPad_t *p = &navq95PwmPads[i];
        const uint32_t effHz = (p->tpm == NAVQ95_TPM4) ? tpm4Eff : tpm3Eff;
        navq95MotorTicksPerUs[i] = (float)effHz / 1000000.0f;
        const uint16_t mid = navq95PwmUsToTicks(i, 1500u);   /* 1.5 ms test pulse */
        int ps = navq95_scmi_pinmux_ex(p->muxReg, p->muxMode, NAVQ95_TPM_SION,
                                       p->padReg, NAVQ95_TPM_PADVAL);
        navq95TpmChannelInit(p->tpm, p->channel, mid);
        navq95_console_puts(" pin");
        navq95_console_puthex(i);
        navq95_console_puts("=");
        navq95_console_puthex((uint32_t)ps);
    }
    navq95_console_puts("\n  driving 1500us: M1/2/3 on IO04/IO20/IO24 (TPM3),"
                        " M4 on IO05 (TPM4) — scope the pads\n");
    navq95_console_puts("[/PWM selftest]\n");
    navq95_console_drain();
}
#endif /* NAVQ95_PWM_SELFTEST */

#endif /* USE_PWM_OUTPUT && USE_MOTOR */
