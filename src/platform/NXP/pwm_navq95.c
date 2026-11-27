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
 * Standard PWM motor output on the i.MX 95 TPM timers. The four motor outputs
 * are split across TPM3 and TPM4, which run from different clocks, so the
 * period and the microsecond-to-tick scale are set per timer.
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

#include "navq95_scmi.h"

#define NAVQ95_TPM3                 ((TPM_Type *)0x424E0000u)
#define NAVQ95_TPM4                 ((TPM_Type *)0x424F0000u)
#define NAVQ95_TPM4_CLK_ROOT        112u
#define NAVQ95_TPM4_CLK_HZ          1000000u

// TPM3 has no clock root of its own; it runs from the WAKEUPMIX bus clock, SYS_PLL1 / 30.
#define NAVQ95_TPM3_BUS_HZ          133333333u

// Slow slew, drive strength 6, pull-down so the ESC line stays low until driven.
#define IOMUXC_BASE                 0x443C0000u
#define NAVQ95_TPM_PADVAL           (0x40Cu >> 1)
#define NAVQ95_TPM_SION             1u

typedef struct {
    TPM_Type *tpm;
    uint32_t  muxReg;
    uint32_t  muxMode;
    uint32_t  padReg;
    uint8_t   channel;
} navq95PwmPad_t;

// Ordered by Betaflight motor index for a quad-X frame.
static const navq95PwmPad_t navq95PwmPads[MAX_SUPPORTED_MOTORS] = {
    { NAVQ95_TPM3, IOMUXC_BASE + 0x0070u, 4u, IOMUXC_BASE + 0x0274u, 3 }, /* M1 rear-right  IO24 TPM3_CH3 */
    { NAVQ95_TPM4, IOMUXC_BASE + 0x0024u, 1u, IOMUXC_BASE + 0x0228u, 0 }, /* M2 front-right IO05 TPM4_CH0 */
    { NAVQ95_TPM3, IOMUXC_BASE + 0x0020u, 1u, IOMUXC_BASE + 0x0224u, 0 }, /* M3 rear-left   IO04 TPM3_CH0 */
    { NAVQ95_TPM3, IOMUXC_BASE + 0x0060u, 6u, IOMUXC_BASE + 0x0264u, 1 }, /* M4 front-left  IO20 TPM3_CH1 */
};

static float navq95MotorTicksPerUs[MAX_SUPPORTED_MOTORS];

static inline uint16_t navq95PwmUsToTicks(uint8_t index, uint32_t us)
{
    return (uint16_t)((float)us * navq95MotorTicksPerUs[index] + 0.5f);
}

// The counter is 16 bit, so pick the smallest prescaler that fits the period. Returns the counter clock.
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

static void navq95TpmChannelInit(TPM_Type *tpm, uint8_t ch, uint16_t initialTicks)
{
    tpm->CONTROLS[ch].CnV  = initialTicks;
    tpm->CONTROLS[ch].CnSC = TPM_CnSC_PWM_HIGH;
}

// channel->ccr points at the TPM CnV register.

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

static float navq95MotorConvertToMotor(uint16_t external)
{
    return (float)external;
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

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t idlePulse)
{
    if (!device) {
        return false;
    }

    const uint8_t count = device->count;
    const uint32_t pwmHz = motorConfig->motorPwmRate ? motorConfig->motorPwmRate : 480u;

    // TPM4 has its own clock root: enable it and set its rate. TPM3 is already
    // clocked; a non-zero VERID confirms it responds before it is touched.
    int clk4 = navq95_scmi_clock_enable(NAVQ95_TPM4_CLK_ROOT);
    navq95_scmi_clock_rate_set(NAVQ95_TPM4_CLK_ROOT, NAVQ95_TPM4_CLK_HZ);
    bool tpm3Ok = (NAVQ95_TPM3->VERID != 0u);
    if (clk4 != 0 || !tpm3Ok) {
        device->vTable = NULL;
        return false;
    }

    const uint32_t tpm3Eff = navq95TpmInit(NAVQ95_TPM3, NAVQ95_TPM3_BUS_HZ, pwmHz);
    const uint32_t tpm4Eff = navq95TpmInit(NAVQ95_TPM4, NAVQ95_TPM4_CLK_HZ, pwmHz);

    const uint32_t idleUs = idlePulse ? idlePulse : 1000u;

    pwmMotorCount = 0;
    for (uint8_t i = 0; i < count && i < MAX_SUPPORTED_MOTORS; i++) {
        const navq95PwmPad_t *p = &navq95PwmPads[i];

        navq95_scmi_pinmux(p->muxReg, p->muxMode, NAVQ95_TPM_SION,
                           p->padReg, NAVQ95_TPM_PADVAL);

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

#endif /* USE_PWM_OUTPUT && USE_MOTOR */
