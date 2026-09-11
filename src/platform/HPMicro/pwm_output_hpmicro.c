/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Standard PWM motor output via HPM PWM timers and TRGM pin routing,
// implementing the Betaflight motor device interface.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pwm_output_impl.h"
#include "platform.h"
#include "dma.h"
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif
#include "timer_hpmicro.h"
#ifdef USE_PWM_OUTPUT

#include "drivers/io.h"
#include "io_hpmicro.h"
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "hpm_clock_drv.h"
#include "hpm_pwm_drv.h"
#include "pg/motor.h"
#include <stdio.h>
#include "timer_hw_ext.h"
#include "trgm_dshot_resource.h"
#include "motor_impl.h"

static bool useContinuousUpdate = true;

/* Per-motor TRGM resources kept at file scope so a later init attempt can
 * release them; a stack copy would drop the only handle and leak the
 * allocation in the static trgm_dshot_resource bitmaps.
 */
static trgmDshotResource_t trgmPwmRes[MAX_SUPPORTED_MOTORS];
static bool trgmPwmResAllocated[MAX_SUPPORTED_MOTORS];

#if defined(HPMSOC_HAS_HPMSDK_PWM)
/*
 * Release every TRGM resource taken by pwmOutputConfig().  Called at the start
 * of motorPwmDevInit() and on its partial-failure path so the static
 * allocation bitmaps return to the free pool instead of leaking until reboot.
 * Timer and IO pin ownership have no release API and stay owned.
 */
static void pwmOutputReleaseResources(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (trgmPwmResAllocated[i]) {
            trgmDshotResourceFree(&trgmPwmRes[i]);
            trgmPwmResAllocated[i] = false;
        }
    }
}

void pwmOutputConfig(timerChannel_t *channel, const timerHardware_t *timerHardware,
                     uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
    (void) value;
    trgm_output_t trgmIoConfig = { 0 };
    const hpmicroTimerHwExt_t *hwExt = hpmicroTimerHwExtByTimer(timerHardware);
    if (hwExt == NULL) {
        // No TRGM routing info for this timer; channel->ccr stays NULL and
        // pwmWriteStandard()/pwmShutdownPulsesForAllMotors() will skip it
        return;
    }

    /* Identify the motor owning this channel so the TRGM resource handle can
     * be stored per motor (the shared prototype in drivers/pwm_output.h
     * cannot carry the index).
     */
    int motorIndex = -1;
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (channel == &pwmMotors[i].channel) {
            motorIndex = i;
            break;
        }
    }
    if (motorIndex < 0) {
        return;
    }

    trgmDshotResource_t *res = &trgmPwmRes[motorIndex];
    if (!trgmDshotResourceAlloc(timerHardware->tag, res)) {
        return;
    }
    trgmPwmResAllocated[motorIndex] = true;
    if (!trgmDshotResourceAllocPwmRef(res, hwExt->pwm_ref_src)) {
        trgmDshotResourceFree(res);
        trgmPwmResAllocated[motorIndex] = false;
        return;
    }

    pwm_stop_counter((PWM_Type *) timerHardware->tim);
    uint32_t reload = 0;
    uint32_t freq;
    pwm_config_t pwmConfig = { 0 };
    pwm_cmp_config_t cmpConfig = { 0 };

    pwm_get_default_pwm_config((PWM_Type *) timerHardware->tim, &pwmConfig);

    pwmConfig.enable_output = true;
    pwmConfig.dead_zone_in_half_cycle = 0;
    pwmConfig.invert_output = (inversion) ? true : false;
    freq = clock_get_frequency(timerRCC((PWM_Type *) timerHardware->tim));
    reload = freq / hz * period - 1;
    /*
     * reload and start counter
     */
    pwm_set_reload((PWM_Type *) timerHardware->tim, 0, reload);
    pwm_set_start_count((PWM_Type *) timerHardware->tim, 0, 0);

    /*
     * config cmp = RELOAD + 1
     */
    cmpConfig.mode = pwm_cmp_mode_output_compare;
    cmpConfig.cmp = reload + 1;
    cmpConfig.update_trigger = pwm_shadow_register_update_on_modify;
    /*
     * config pwm as output driven by cmp
     */
    if (status_success !=
        pwm_setup_waveform((PWM_Type *) timerHardware->tim, timerHardware->channel,
                           &pwmConfig, hwExt->cmp_index, &cmpConfig, 1)) {
        printf("failed to setup waveform\n");
        failureMode(FAILURE_DEVELOPER);
    }
    pwm_start_counter((PWM_Type *) timerHardware->tim);
    pwm_issue_shadow_register_lock_event((PWM_Type *) timerHardware->tim);

    channel->ccr = timerChCCR(timerHardware);

    channel->tim = timerHardware->tim;

    // CMP beyond reload keeps the output low (0% high time) until the first
    // motor value is written
    *channel->ccr = PWM_CMP_CMP_SET(reload + 1);
    memset(&trgmIoConfig, 0, sizeof(trgmIoConfig));
    trgmIoConfig.invert = 0;
    trgmIoConfig.type = trgm_output_same_as_input;
    trgmIoConfig.input = res->pwm_ref_src;
    trgm_output_config(trgmDshotResourceTrgm(res), res->trgm_p_dst, &trgmIoConfig);
    trgm_enable_io_output(trgmDshotResourceTrgm(res), 1 << (res->port_index));
}

FAST_CODE static void pwmWriteStandard(uint8_t index, float value)
{
    /* TODO: move value to be a number between 0-1 (i.e. percent throttle from mixer) */
    if (pwmMotors[index].channel.ccr) {
        *pwmMotors[index].channel.ccr = lrintf(value * pwmMotors[index].pulseScale + pwmMotors[index].pulseOffset);
    }
}

FAST_CODE static void pwmShutdownPulsesForAllMotors(void)
{
    for (int index = 0; index < pwmMotorCount; index++) {
        // Set the compare value beyond reload, which stops the output pulsing
        if (pwmMotors[index].channel.ccr) {
            *pwmMotors[index].channel.ccr = PWM_CMP_CMP_SET(0xFFFFFFFF);
        }
    }
}

FAST_CODE void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors();
}

static void pwmCompleteMotorUpdate(void)
{
    if (useContinuousUpdate) {
        return;
    }

    for (int index = 0; index < pwmMotorCount; index++) {
        // Set the compare value beyond reload, which stops the output pulsing
        // until the next main loop writes a new pulse width
        if (pwmMotors[index].channel.ccr) {
            *pwmMotors[index].channel.ccr = PWM_CMP_CMP_SET(0xFFFFFFFF);
        }
    }
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float) externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t) motorValue;
}

static motorVTable_t motorPwmVTable = {
    .postInit = motorPostInitNull,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .convertExternalToMotor = pwmConvertFromExternal,
    .convertMotorToExternal = pwmConvertToExternal,
    .write = pwmWriteStandard,
    .updateComplete = pwmCompleteMotorUpdate,
    .getMotorIO = pwmGetMotorIO,
};

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t idlePulse)
{
    /* Roll back TRGM resources held by any earlier init attempt so the
     * allocations below start from a clean slate without requiring a reboot.
     */
    pwmOutputReleaseResources();

    memset(pwmMotors, 0, sizeof(pwmMotors));

    pwmMotorCount = device->count;
    device->vTable = &motorPwmVTable;
    useContinuousUpdate = motorConfig->useContinuousUpdate;

    float sMin = 0;
    float sLen = 0;

    switch (motorConfig->motorProtocol) {
    default:
    case MOTOR_PROTOCOL_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case MOTOR_PROTOCOL_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case MOTOR_PROTOCOL_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    case MOTOR_PROTOCOL_BRUSHED:
        sMin = 0;
        useContinuousUpdate = true;
        idlePulse = 0;
        break;
    case MOTOR_PROTOCOL_PWM:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useContinuousUpdate = true;
        idlePulse = 0;
        break;
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < pwmMotorCount; motorIndex++) {
        const unsigned reorderedMotorIndex = motorConfig->motorOutputReordering[motorIndex];
        const ioTag_t tag = motorConfig->ioTags[reorderedMotorIndex];
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));

        if (timerHardware == NULL) {
            /* not enough motors initialised for the mixer or a
             * break in the motors */
            device->vTable = NULL;
            pwmMotorCount = 0;
            pwmOutputReleaseResources();
            /* TODO: block arming and add reason system cannot arm */
            return false;
        }
        pwmMotors[motorIndex].io = IOGetByTag(tag);
        IOInit(pwmMotors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));

        IOConfigGPIOAF(pwmMotors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);
        /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
        const unsigned pwmRateHz = (useContinuousUpdate) ? motorConfig->motorPwmRate
                                                          : ceilf(1.0f / ((sMin + sLen) * 4));

        const uint32_t clock = getPWMFre((TIM_TypeDef *) timerHardware->tim);
        /* used to find the desired timer frequency for max resolution
         */
        const unsigned prescaler = (clock / pwmRateHz + 0xffff) / 0x10000;    /* rounding up */
        const uint32_t hz = clock / prescaler;
        const unsigned period = (useContinuousUpdate) ? hz / pwmRateHz : 0xffff;

        /*
           if brushed then it is the entire length of the period.
           TODO: this can be moved back to periodMin and periodLen
           once mixer outputs a 0..1 float value.
         */
        /*
         * HPM PWM output compare mode (non-inverted):
         *   HIGH time = reload - CMP
         * The PWM counter runs at the source clock frequency (clock), so all
         * CMP values must be in source-clock ticks, NOT prescaled hz ticks.
         * We want HIGH time = desired_pulse_seconds, so:
         *   CMP = reload - desired_pulse_ticks
         * desired_pulse_ticks = clock * (sMin + sLen * (value - 1000) / 1000)
         *   = clock*sMin + (clock*sLen/1000)*value - clock*sLen
         * CMP = reload + clock*(sLen - sMin) - (clock*sLen/1000)*value
         */
        const uint32_t reload = clock / hz * period - 1;

        if (motorConfig->motorProtocol == MOTOR_PROTOCOL_BRUSHED) {
            // Brushed: throttle 0..1000 maps to CMP = reload (0% high) .. 0 (~100% high)
            pwmMotors[motorIndex].pulseScale = -((float) reload / 1000.0f);
            pwmMotors[motorIndex].pulseOffset = (float) reload;
        } else {
            pwmMotors[motorIndex].pulseScale = -(sLen * clock) / 1000.0f;
            pwmMotors[motorIndex].pulseOffset = (float) reload + clock * (sLen - sMin);
        }

        pwmOutputConfig(&pwmMotors[motorIndex].channel, timerHardware, hz, period,
                        idlePulse, motorConfig->motorInversion);

        if (pwmMotors[motorIndex].channel.ccr == NULL) {
            device->vTable = NULL;
            pwmMotorCount = 0;
            pwmOutputReleaseResources();
            return false;
        }

        bool timerAlreadyUsed = false;

        for (int i = 0; i < motorIndex; i++) {
            if (pwmMotors[i].channel.tim == pwmMotors[motorIndex].channel.tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        pwmMotors[motorIndex].forceOverflow = !timerAlreadyUsed;
        pwmMotors[motorIndex].enabled = true;
    }
    return true;
}

FAST_CODE pwmOutputPort_t *pwmGetMotors(void)
{
    return pwmMotors;
}

#endif
#endif                          // USE_PWM_OUTPUT
