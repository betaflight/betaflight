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
 * NavQ95 (i.MX 95 M7) motor PWM output via TPM.
 *
 * i.MX 95 has no FlexPWM; the on-chip PWM-capable timer is the TPM. Standard
 * (analog) PWM only; DShot is a later upgrade. Per the board schematic the 4
 * motor outputs are mixed across two TPM instances:
 *   M1 GPIO_IO04 TPM3_CH0 ALT1 (MUX 0x0020, PAD 0x0224)
 *   M2 GPIO_IO20 TPM3_CH1 ALT6 (MUX 0x0060, PAD 0x0264)
 *   M3 GPIO_IO24 TPM3_CH3 ALT4 (MUX 0x0070, PAD 0x0274)
 *   M4 GPIO_IO05 TPM4_CH0 ALT1 (MUX 0x0024, PAD 0x0228)
 *
 * TPM3 (and TPM1) have no configurable root; they count the AONMIX/WAKEUPMIX
 * bus clock, so they are NOT SCMI clock-enabled (a CLOCK_CONFIG_SET returns
 * DENIED for them). TPM4 has its own root and IS clock-enabled over SCMI.
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
extern void navq95_console_puts(const char *s);
extern void navq95_console_puthex(uint32_t v);
extern void navq95_console_drain(void);

/* TPM instances. TPM3 base 0x424E0000 (bus-clocked, read-only root, not
 * SCMI-enabled). TPM4 base 0x424F0000, CCM root index 112 (SCMI clk id =
 * root + 41, added by navq95_scmi_clock_enable_ex). TPM input clock assumed
 * 1 MHz (PX4 uses OSC_24M/24); the self-test prints it so it can be checked
 * against HW. 1 MHz gives 1 us resolution for 1000-2000 us servo pulses. */
#define NAVQ95_TPM3                 ((TPM_Type *)0x424E0000u)
#define NAVQ95_TPM4                 ((TPM_Type *)0x424F0000u)
#define NAVQ95_TPM4_CLK_ROOT        112u
#define NAVQ95_TPM_CLK_HZ           1000000u

/* Output pad value (pre-shifted >>1, the SM re-applies <<1): FSEL slow |
 * DSE_X6 | PD_ON = raw 0x40C -> 0x206. Pull-down keeps the ESC line low if the
 * pad floats at boot. SION is set so the output state is readable. */
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

/* Convert microseconds to TPM compare ticks at NAVQ95_TPM_CLK_HZ. */
static inline uint16_t navq95PwmUsToTicks(uint32_t us)
{
    return (uint16_t)((uint64_t)us * NAVQ95_TPM_CLK_HZ / 1000000u);
}

/* Init a TPM for edge-aligned high-true PWM at pwmHz. */
static void navq95TpmInit(TPM_Type *tpm, uint32_t pwmHz)
{
    tpm->GLOBAL |= TPM_GLOBAL_RST;
    for (volatile int d = 0; d < 10000; d++) { }
    tpm->GLOBAL &= ~TPM_GLOBAL_RST;
    for (volatile int d = 0; d < 10000; d++) { }

    tpm->SC   = 0;
    tpm->CNT  = 0;
    tpm->POL  = 0;

    uint32_t mod = (NAVQ95_TPM_CLK_HZ / (pwmHz ? pwmHz : 1u));
    if (mod == 0u) {
        mod = 1u;
    }
    if (mod > 0x10000u) {
        mod = 0x10000u;
    }
    tpm->MOD = (uint16_t)(mod - 1u);

    tpm->SC = TPM_SC_CMOD(1) | TPM_SC_PS(0);
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
        *pwmMotors[index].channel.ccr = navq95PwmUsToTicks(value);
    }
}

static void navq95MotorWrite(uint8_t index, float value)
{
    if (index < pwmMotorCount && pwmMotors[index].channel.ccr) {
        *pwmMotors[index].channel.ccr = navq95PwmUsToTicks((uint32_t)(value + 0.5f));
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

    /* TPM4 (M4) has its own root and is granted to the M7 -> SCMI clock-enable.
     * TPM3 (M1/M2/M3) is bus-clocked (not settable) -> no clock-enable; verify
     * it responds by reading VERID (nonzero) so we never touch an unclocked
     * block (crash-safe, no bus hang). */
    int clk4 = navq95_scmi_clock_enable_ex(NAVQ95_TPM4_CLK_ROOT);
    bool tpm3Ok = (NAVQ95_TPM3->VERID != 0u);
    navq95PwmClockEnabled = (clk4 == 0) && tpm3Ok;
    if (clk4 != 0 || !tpm3Ok) {
        device->vTable = NULL;
        return false;
    }

    navq95TpmInit(NAVQ95_TPM3, pwmHz);
    navq95TpmInit(NAVQ95_TPM4, pwmHz);

    const uint16_t idleTicks = navq95PwmUsToTicks(idlePulse ? idlePulse : 1000u);

    pwmMotorCount = 0;
    for (uint8_t i = 0; i < count && i < MAX_SUPPORTED_MOTORS; i++) {
        const navq95PwmPad_t *p = &navq95PwmPads[i];

        navq95_scmi_pinmux_ex(p->muxReg, p->muxMode, NAVQ95_TPM_SION,
                              p->padReg, NAVQ95_TPM_PADVAL);

        navq95TpmChannelInit(p->tpm, p->channel, idleTicks);

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
     * TPM4 (own root, SCMI clock-enable). */
    int clk4 = navq95_scmi_clock_enable_ex(NAVQ95_TPM4_CLK_ROOT);
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

    navq95TpmInit(NAVQ95_TPM3, 480u);
    navq95TpmInit(NAVQ95_TPM4, 480u);
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
        const uint16_t mid = navq95PwmUsToTicks(1500u);   /* 1.5 ms test pulse */
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
