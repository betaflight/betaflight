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

#include <math.h>

#include "platform.h"
#include "platform/camera_control.h"

#include "drivers/io.h"
#include "drivers/time.h"

#ifdef CURRENT_TARGET_CPU_VOLTAGE
#define ADC_VOLTAGE CURRENT_TARGET_CPU_VOLTAGE
#else
#define ADC_VOLTAGE 3.3f
#endif

#ifdef CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
static timerChannel_t channel;
#endif

static cameraControlRuntime_t *cameraControlRuntime;

#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
void cameraControlHi(void)
{
    if (cameraControlRuntime->inverted) {
        IOLo(cameraControlRuntime->io);
    } else {
        IOHi(cameraControlRuntime->io);
    }
}

void cameraControlLo(void)
{
    if (cameraControlRuntime->inverted) {
        IOHi(cameraControlRuntime->io);
    } else {
        IOLo(cameraControlRuntime->io);
    }
}
#endif

void cameraControlInitImpl(cameraControlRuntime_t *runtime)
{
    cameraControlRuntime = runtime;
    if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
#ifdef CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
        const timerHardware_t *timerHardware = timerAllocate(cameraControlConfig()->ioTag, OWNER_CAMERA_CONTROL, 0);

        if (!timerHardware) {
            cameraControlRuntime->enabled = false;
            return;
        }

        IOConfigGPIOAF(cameraControlRuntime->io, IOCFG_AF_PP, timerHardware->alternateFunction);

        cameraControlHardwarePwmInit(&channel, timerHardware, cameraControlRuntime->inverted);

        cameraControlRuntime->period = CAMERA_CONTROL_PWM_RESOLUTION;
        *channel.ccr = cameraControlRuntime->period;
        cameraControlRuntime->enabled = true;
#endif
    } else if (CAMERA_CONTROL_MODE_SOFTWARE_PWM == cameraControlConfig()->mode) {
#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE

        IOConfigGPIO(cameraControlRuntime->io, IOCFG_OUT_PP);
        cameraControlHi();

        cameraControlRuntime->period = CAMERA_CONTROL_SOFT_PWM_RESOLUTION;
        cameraControlRuntime->enabled = true;

        cameraControlSoftwarePwmInit();
#endif
    } else if (CAMERA_CONTROL_MODE_DAC == cameraControlConfig()->mode) {
        // @todo not yet implemented
    }
}

#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE) || defined(CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE)

static float calculateKeyPressVoltage(const cameraControlKey_e key)
{
    const int buttonResistance = cameraControlConfig()->buttonResistanceValues[key] * 100;
    return 1.0e-2f * cameraControlConfig()->refVoltage * buttonResistance / (100 * cameraControlConfig()->internalResistance + buttonResistance);
}

static float calculatePWMDutyCycle(const cameraControlKey_e key)
{
    const float voltage = calculateKeyPressVoltage(key);

    return voltage / ADC_VOLTAGE;
}
#endif

void cameraControlProcessImpl(void)
{
    if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE)
        *channel.ccr = cameraControlRuntime->period;
#else
        UNUSED(cameraControlRuntime);
#endif
    } else if (CAMERA_CONTROL_MODE_SOFTWARE_PWM == cameraControlConfig()->mode) {

    }
}

void cameraControlKeyPressImpl(cameraControlKey_e key, uint32_t holdDurationMs)
{

#if defined(CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE) || defined(CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE)
    const float dutyCycle = calculatePWMDutyCycle(key);
#else
    UNUSED(holdDurationMs);
    UNUSED(key);
#endif

    if (CAMERA_CONTROL_MODE_HARDWARE_PWM == cameraControlConfig()->mode) {
#ifdef CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
        *channel.ccr = lrintf(dutyCycle * cameraControlRuntime->period);
        cameraControlRuntime->endTimeMillis = millis() + cameraControlConfig()->keyDelayMs + holdDurationMs;
#endif
    } else if (CAMERA_CONTROL_MODE_SOFTWARE_PWM == cameraControlConfig()->mode) {
#ifdef CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
        const uint32_t hiTime = lrintf(dutyCycle * cameraControlRuntime->period);

        if (0 == hiTime) {
            cameraControlLo();
            delay(cameraControlConfig()->keyDelayMs + holdDurationMs);
            cameraControlHi();
        } else {
            cameraControlSoftwarePwmEnable(hiTime, cameraControlRuntime->period);

            const uint32_t endTime = millis() + cameraControlConfig()->keyDelayMs + holdDurationMs;

            // Wait to give the camera a chance at registering the key press
            while (millis() < endTime);

            // Disable timers and interrupt generation
            cameraControlSoftwarePwmDisable();

            // Reset to idle state
            IOHi(cameraControlRuntime->io);
        }
#endif
    } else if (CAMERA_CONTROL_MODE_DAC == cameraControlConfig()->mode) {
        // @todo not yet implemented
    }
}