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
 * porting for ch32h41x by Temperslee
 */
#include "platform.h"

#ifdef USE_CAMERA_CONTROL

#include <math.h>

#include "build/atomic.h"

#include "drivers/camera_control_impl.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "platform/rcc.h"
#include "platform/timer.h"

#ifdef CURRENT_TARGET_CPU_VOLTAGE
#define CAMERA_CONTROL_VDD_VOLTAGE CURRENT_TARGET_CPU_VOLTAGE
#else
#define CAMERA_CONTROL_VDD_VOLTAGE 3.3f
#endif

#define CAMERA_CONTROL_DAC_MAX_CODE       4095U
#define CAMERA_CONTROL_SOFT_PWM_TIMER_MHZ 20U
#define CAMERA_CONTROL_IO_ANALOG_CFG      IO_CONFIG(DIR_IN, GPIO_MODE_IN_AN, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)

static cameraControlRuntime_t *cameraControlRuntime;
static cameraControlMode_e activeMode = CAMERA_CONTROL_MODES_COUNT;

#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE) && defined(USE_PWM_OUTPUT)
static timerChannel_t channel;
#endif

#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
static volatile uint16_t softPwmHiTime;
#endif

#ifdef CAMERA_CONTROL_DAC_AVAILABLE
static uint32_t activeDacChannel;
#endif

static float calculateKeyPressVoltage(const cameraControlKey_e key)
{
    const int buttonResistance = cameraControlConfig()->buttonResistanceValues[key] * 100;
    const int totalResistance = 100 * cameraControlConfig()->internalResistance + buttonResistance;
    if (totalResistance <= 0) {
        return 0.0f;
    }
    return 1.0e-2f * cameraControlConfig()->refVoltage * buttonResistance / totalResistance;
}

static float calculateKeyPressRatio(const cameraControlKey_e key)
{
    const float ratio = calculateKeyPressVoltage(key) / CAMERA_CONTROL_VDD_VOLTAGE;
    if (ratio <= 0.0f) {
        return 0.0f;
    }
    if (ratio >= 1.0f) {
        return 1.0f;
    }
    return ratio;
}

#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE) && defined(USE_PWM_OUTPUT)
static bool cameraControlHardwarePwmTryInit(void)
{
    const timerHardware_t *timerHardware = timerAllocate(cameraControlConfig()->ioTag, OWNER_CAMERA_CONTROL, 0);
    if (!timerHardware) {
        return false;
    }

    pwmOutputConfig(&channel, timerHardware, timerClock(timerHardware), CAMERA_CONTROL_PWM_RESOLUTION, CAMERA_CONTROL_PWM_RESOLUTION, cameraControlRuntime->inverted);

    cameraControlRuntime->period = CAMERA_CONTROL_PWM_RESOLUTION;
    *channel.ccr = cameraControlRuntime->period;
    IOConfigGPIOAF(cameraControlRuntime->io, IOCFG_AF_PP, timerHardware->alternateFunction);

    activeMode = CAMERA_CONTROL_MODE_HARDWARE_PWM;
    cameraControlRuntime->enabled = true;
    return true;
}
#endif

#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
static void cameraControlHi(void)
{
    if (!cameraControlRuntime) {
        return;
    }

    if (cameraControlRuntime->inverted) {
        IOLo(cameraControlRuntime->io);
    } else {
        IOHi(cameraControlRuntime->io);
    }
}

static void cameraControlLo(void)
{
    if (!cameraControlRuntime) {
        return;
    }

    if (cameraControlRuntime->inverted) {
        IOHi(cameraControlRuntime->io);
    } else {
        IOLo(cameraControlRuntime->io);
    }
}

__FAST_INTERRUPT
void TIM6_IRQHandler(void)
{
    TIM6->INTFR = 0;
    if (TIM6->CNT < softPwmHiTime) {
        cameraControlHi();
    }
}

__FAST_INTERRUPT
void TIM7_IRQHandler(void)
{
    TIM7->INTFR = 0;
    if (TIM6->CNT >= softPwmHiTime) {
        cameraControlLo();
    }
}

static void cameraControlSoftwarePwmDisable(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        softPwmHiTime = 0;
        TIM6->CTLR1 &= (uint16_t)~TIM_CEN;
        TIM7->CTLR1 &= (uint16_t)~TIM_CEN;
        TIM6->DMAINTENR = 0;
        TIM7->DMAINTENR = 0;
        TIM6->INTFR = 0;
        TIM7->INTFR = 0;
        ch32DmaBarrier();
        NVIC_ClearPendingIRQ(TIM6_IRQn);
        NVIC_ClearPendingIRQ(TIM7_IRQn);
    }
}

static void cameraControlSoftwarePwmEnable(uint32_t hiTime, uint32_t period)
{
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        TIM6->CTLR1 &= (uint16_t)~TIM_CEN;
        TIM7->CTLR1 &= (uint16_t)~TIM_CEN;
        TIM6->DMAINTENR = 0;
        TIM7->DMAINTENR = 0;

        softPwmHiTime = (uint16_t)hiTime;
        TIM6->CNT = (uint16_t)hiTime;
        TIM6->ATRLR = (uint16_t)period;

        TIM7->CNT = 0;
        TIM7->ATRLR = (uint16_t)period;

        TIM6->INTFR = 0;
        TIM7->INTFR = 0;
        ch32DmaBarrier();
        NVIC_ClearPendingIRQ(TIM6_IRQn);
        NVIC_ClearPendingIRQ(TIM7_IRQn);

        cameraControlLo();

        TIM6->DMAINTENR = TIM_IT_Update;
        TIM7->DMAINTENR = TIM_IT_Update;
        TIM6->CTLR1 = TIM_CEN;
        TIM7->CTLR1 = TIM_CEN;
    }
}

static bool cameraControlSoftwarePwmTryInit(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        RCC_ClockCmd(RCC_HB1(TIM6), ENABLE);
        RCC_ClockCmd(RCC_HB1(TIM7), ENABLE);
    }

    cameraControlSoftwarePwmDisable();

    const uint16_t prescaler = timerGetPrescalerByDesiredMhz((timerResource_t *)TIM6, CAMERA_CONTROL_SOFT_PWM_TIMER_MHZ);
    TIM6->PSC = prescaler;
    TIM7->PSC = prescaler;
    TIM6->SWEVGR = TIM_PSCReloadMode_Immediate;
    TIM7->SWEVGR = TIM_PSCReloadMode_Immediate;
    TIM6->INTFR = 0;
    TIM7->INTFR = 0;
    ch32DmaBarrier();
    NVIC_ClearPendingIRQ(TIM6_IRQn);
    NVIC_ClearPendingIRQ(TIM7_IRQn);

    NVIC_SetPriority(TIM6_IRQn, NVIC_PRIO_TIMER);
    NVIC_EnableIRQ(TIM6_IRQn);
    NVIC_SetPriority(TIM7_IRQn, NVIC_PRIO_TIMER);
    NVIC_EnableIRQ(TIM7_IRQn);

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        cameraControlHi();
        IOConfigGPIO(cameraControlRuntime->io, IOCFG_OUT_PP);
    }

    cameraControlRuntime->period = CAMERA_CONTROL_SOFT_PWM_RESOLUTION;
    activeMode = CAMERA_CONTROL_MODE_SOFTWARE_PWM;
    cameraControlRuntime->enabled = true;
    return true;
}
#endif

#ifdef CAMERA_CONTROL_DAC_AVAILABLE
static bool cameraControlResolveDacChannel(IO_t io, uint32_t *channelOut)
{
    if (!io || !channelOut) {
        return false;
    }

    const IO_t dac1Io = IOGetByTag(DEFIO_TAG_E(PA4));
    if (dac1Io != IO_NONE && io == dac1Io) {
        *channelOut = DAC_Channel_1;
        return true;
    }

    const IO_t dac2Io = IOGetByTag(DEFIO_TAG_E(PA5));
    if (dac2Io != IO_NONE && io == dac2Io) {
        *channelOut = DAC_Channel_2;
        return true;
    }

    return false;
}

static void cameraControlDacWriteCode(uint16_t rawCode)
{
    if (rawCode > CAMERA_CONTROL_DAC_MAX_CODE) {
        rawCode = CAMERA_CONTROL_DAC_MAX_CODE;
    }

    const uint16_t effectiveCode = cameraControlRuntime->inverted
        ? (uint16_t)(CAMERA_CONTROL_DAC_MAX_CODE - rawCode)
        : rawCode;

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        if (effectiveCode == 0) {
            DAC_Cmd(activeDacChannel, DISABLE);
            IOLo(cameraControlRuntime->io);
            IOConfigGPIO(cameraControlRuntime->io, IOCFG_OUT_PP);
        } else if (effectiveCode >= CAMERA_CONTROL_DAC_MAX_CODE) {
            DAC_Cmd(activeDacChannel, DISABLE);
            IOHi(cameraControlRuntime->io);
            IOConfigGPIO(cameraControlRuntime->io, IOCFG_OUT_PP);
        } else {
            IOConfigGPIO(cameraControlRuntime->io, CAMERA_CONTROL_IO_ANALOG_CFG);
            if (activeDacChannel == DAC_Channel_1) {
                DAC->R12BDHR1 = effectiveCode;
            } else {
                DAC->R12BDHR2 = effectiveCode;
            }
            DAC_Cmd(activeDacChannel, ENABLE);
        }
    }
}

static bool cameraControlDacTryInit(void)
{
    uint32_t dacChannel = 0;
    if (!cameraControlResolveDacChannel(cameraControlRuntime->io, &dacChannel)) {
        return false;
    }

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        RCC_ClockCmd(RCC_HB1(DAC), ENABLE);
    }

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        activeDacChannel = dacChannel;
        DAC_Cmd(activeDacChannel, DISABLE);

        DAC_InitTypeDef dacInit;
        DAC_StructInit(&dacInit);
        dacInit.DAC_Trigger = DAC_Trigger_None;
        dacInit.DAC_WaveGeneration = DAC_WaveGeneration_None;
        dacInit.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
        DAC_Init(activeDacChannel, &dacInit);
    }

    cameraControlRuntime->period = CAMERA_CONTROL_DAC_MAX_CODE;
    cameraControlDacWriteCode(CAMERA_CONTROL_DAC_MAX_CODE);
    activeMode = CAMERA_CONTROL_MODE_DAC;
    cameraControlRuntime->enabled = true;
    return true;
}
#endif

static void cameraControlStopActiveBackend(void)
{
    switch (activeMode) {
#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
    case CAMERA_CONTROL_MODE_SOFTWARE_PWM:
        cameraControlSoftwarePwmDisable();
        break;
#endif
#ifdef CAMERA_CONTROL_DAC_AVAILABLE
    case CAMERA_CONTROL_MODE_DAC:
        ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
            DAC_Cmd(activeDacChannel, DISABLE);
        }
        break;
#endif
    default:
        break;
    }
    activeMode = CAMERA_CONTROL_MODES_COUNT;
}

void cameraControlInitImpl(cameraControlRuntime_t *runtime)
{
    cameraControlStopActiveBackend();
    cameraControlRuntime = runtime;

    if (!runtime) {
        return;
    }

    runtime->enabled = false;
    runtime->endTimeMillis = 0;

    if (runtime->io == IO_NONE) {
        return;
    }

    switch (cameraControlConfig()->mode) {
    case CAMERA_CONTROL_MODE_DAC:
        // Prefer 12-bit hardware DAC when the pin supports it (PA4/PA5), then
        // fall back to timer-backed hardware PWM, and finally software PWM.
#ifdef CAMERA_CONTROL_DAC_AVAILABLE
        if (cameraControlDacTryInit()) {
            return;
        }
#endif
#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE) && defined(USE_PWM_OUTPUT)
        if (cameraControlHardwarePwmTryInit()) {
            return;
        }
#endif
#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
        cameraControlSoftwarePwmTryInit();
#endif
        break;

    case CAMERA_CONTROL_MODE_HARDWARE_PWM:
#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE) && defined(USE_PWM_OUTPUT)
        if (cameraControlHardwarePwmTryInit()) {
            return;
        }
#endif
        // Fall back automatically when the configured pin has no available
        // hardware timer channel (for example DAC-only pins).
#ifdef CAMERA_CONTROL_DAC_AVAILABLE
        if (cameraControlDacTryInit()) {
            return;
        }
#endif
#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
        cameraControlSoftwarePwmTryInit();
#endif
        break;

    case CAMERA_CONTROL_MODE_SOFTWARE_PWM:
#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
        cameraControlSoftwarePwmTryInit();
#endif
        break;

    default:
        break;
    }
}

void cameraControlProcessImpl(void)
{
    if (!cameraControlRuntime || !cameraControlRuntime->enabled) {
        return;
    }

    switch (activeMode) {
#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE) && defined(USE_PWM_OUTPUT)
    case CAMERA_CONTROL_MODE_HARDWARE_PWM:
        *channel.ccr = cameraControlRuntime->period;
        break;
#endif
#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
    case CAMERA_CONTROL_MODE_SOFTWARE_PWM:
        cameraControlSoftwarePwmDisable();
        cameraControlHi();
        break;
#endif
#ifdef CAMERA_CONTROL_DAC_AVAILABLE
    case CAMERA_CONTROL_MODE_DAC:
        cameraControlDacWriteCode(CAMERA_CONTROL_DAC_MAX_CODE);
        break;
#endif
    default:
        break;
    }
}

void cameraControlKeyPressImpl(cameraControlKey_e key, timeMs_t holdDurationMs)
{
    if (!cameraControlRuntime || !cameraControlRuntime->enabled) {
        return;
    }

    const float dutyCycle = calculateKeyPressRatio(key);
    timeMs_t holdEndMillis = millis() + cameraControlConfig()->keyDelayMs + holdDurationMs;
    if (holdEndMillis == 0) {
        holdEndMillis = 1;
    }

    switch (activeMode) {
#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE) && defined(USE_PWM_OUTPUT)
    case CAMERA_CONTROL_MODE_HARDWARE_PWM:
        *channel.ccr = lrintf(dutyCycle * cameraControlRuntime->period);
        cameraControlRuntime->endTimeMillis = holdEndMillis;
        break;
#endif
#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
    case CAMERA_CONTROL_MODE_SOFTWARE_PWM: {
        const uint32_t hiTime = (uint32_t)lrintf(dutyCycle * cameraControlRuntime->period);
        if (hiTime == 0) {
            cameraControlSoftwarePwmDisable();
            cameraControlLo();
        } else if (hiTime >= cameraControlRuntime->period) {
            cameraControlSoftwarePwmDisable();
            cameraControlHi();
        } else {
            cameraControlSoftwarePwmEnable(hiTime, cameraControlRuntime->period);
        }
        cameraControlRuntime->endTimeMillis = holdEndMillis;
        break;
    }
#endif
#ifdef CAMERA_CONTROL_DAC_AVAILABLE
    case CAMERA_CONTROL_MODE_DAC: {
        const uint16_t dacCode = (uint16_t)lrintf(dutyCycle * CAMERA_CONTROL_DAC_MAX_CODE);
        cameraControlDacWriteCode(dacCode);
        cameraControlRuntime->endTimeMillis = holdEndMillis;
        break;
    }
#endif
    default:
        break;
    }
}
#endif // USE_CAMERA_CONTROL
