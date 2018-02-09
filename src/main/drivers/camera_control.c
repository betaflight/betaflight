/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <platform.h>

#ifdef USE_CAMERA_CONTROL

#include "camera_control.h"
#include "io.h"
#include "math.h"
#include "nvic.h"
#include "pwm_output.h"
#include "time.h"
#include "pg/camera_control.h"

#if defined(STM32F40_41xxx)
#define CAMERA_CONTROL_TIMER_HZ   MHZ_TO_HZ(84)
#elif defined(STM32F7)
#define CAMERA_CONTROL_TIMER_HZ   MHZ_TO_HZ(216)
#else
#define CAMERA_CONTROL_TIMER_HZ   MHZ_TO_HZ(72)
#endif

#define CAMERA_CONTROL_PWM_RESOLUTION   128

#ifdef CURRENT_TARGET_CPU_VOLTAGE
#define ADC_VOLTAGE CURRENT_TARGET_CPU_VOLTAGE
#else
#define ADC_VOLTAGE 3.3f
#endif

#define CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
#include "timer.h"

#ifdef USE_OSD
#include "io/osd.h"
#endif

static struct {
    bool enabled;
    IO_t io;
    timerChannel_t channel;
    uint32_t period;
} cameraControlRuntime;

static uint32_t endTimeMillis;

void cameraControlInit(void)
{
    if (cameraControlConfig()->ioTag == IO_TAG_NONE)
        return;

    cameraControlRuntime.io = IOGetByTag(cameraControlConfig()->ioTag);
    IOInit(cameraControlRuntime.io, OWNER_CAMERA_CONTROL, 0);

    if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
#ifdef CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
        const timerHardware_t *timerHardware = timerGetByTag(cameraControlConfig()->ioTag, TIM_USE_ANY);

        if (!timerHardware) {
            return;
        }

        #ifdef STM32F1
            IOConfigGPIO(cameraControlRuntime.io, IOCFG_AF_PP);
        #else
            IOConfigGPIOAF(cameraControlRuntime.io, IOCFG_AF_PP, timerHardware->alternateFunction);
        #endif

        pwmOutConfig(&cameraControlRuntime.channel, timerHardware, CAMERA_CONTROL_TIMER_HZ, CAMERA_CONTROL_PWM_RESOLUTION, 0, 0);

        cameraControlRuntime.period = CAMERA_CONTROL_PWM_RESOLUTION;
        *cameraControlRuntime.channel.ccr = cameraControlRuntime.period;
        cameraControlRuntime.enabled = true;
#endif
    } else if (CAMERA_CONTROL_MODE_DAC == cameraControlConfig()->mode) {
        // @todo not yet implemented
    }
}

void cameraControlProcess(uint32_t currentTimeUs)
{
    if (endTimeMillis && currentTimeUs >= 1000 * endTimeMillis) {
        if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
            *cameraControlRuntime.channel.ccr = cameraControlRuntime.period;
        }

        endTimeMillis = 0;
    }
}

static const int buttonResistanceValues[] = { 45000, 27000, 15000, 6810, 0 };

static float calculateKeyPressVoltage(const cameraControlKey_e key)
{
    const int buttonResistance = buttonResistanceValues[key];
    return 1.0e-2f * cameraControlConfig()->refVoltage * buttonResistance / (100 * cameraControlConfig()->internalResistance + buttonResistance);
}

#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE)
static float calculatePWMDutyCycle(const cameraControlKey_e key)
{
    const float voltage = calculateKeyPressVoltage(key);

    return voltage / ADC_VOLTAGE;
}
#endif

void cameraControlKeyPress(cameraControlKey_e key, uint32_t holdDurationMs)
{
    if (!cameraControlRuntime.enabled)
        return;

    if (key >= CAMERA_CONTROL_KEYS_COUNT)
        return;

#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE)
    const float dutyCycle = calculatePWMDutyCycle(key);
#else
    (void) holdDurationMs;
#endif

#ifdef USE_OSD
    // Force OSD timeout so we are alone on the display.
    resumeRefreshAt = 0;
#endif

    if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
#ifdef CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
        *cameraControlRuntime.channel.ccr = lrintf(dutyCycle * cameraControlRuntime.period);
        endTimeMillis = millis() + cameraControlConfig()->keyDelayMs + holdDurationMs;
#endif
    } else if (CAMERA_CONTROL_MODE_DAC == cameraControlConfig()->mode) {
        // @todo not yet implemented
    }
}

#endif
