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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/timer.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_output.h"

#include "sound_beeper.h"


#ifdef BEEPER

static IO_t beeperIO = DEFIO_IO(NONE);
static bool beeperInverted = false;
static bool beeperState = false;

#endif

void systemBeep(bool onoff)
{
#if !defined(BEEPER)
    UNUSED(onoff);
#elif defined(BEEPER_PWM)
    TIM_CtrlPWMOutputs(BEEPER_PWM_TIMER, onoff ? ENABLE : DISABLE);
    beeperState = onoff;
#else
    IOWrite(beeperIO, beeperInverted ? onoff : !onoff);
    beeperState = onoff;
#endif
}

void systemBeepToggle(void)
{
#if defined(BEEPER)
    systemBeep(!beeperState);
#endif
}

#if defined(BEEPER_PWM)
static void configBeeperPWMTimer(const beeperDevConfig_t *config)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = (1000000 / BEEPER_PWM_FREQUENCY) * 50 / 100; // 50% duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = config->isInverted ? TIM_OCPolarity_High : TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = config->isInverted ? TIM_OCIdleState_Reset : TIM_OCIdleState_Set;

    configTimeBase(BEEPER_PWM_TIMER, 1000000 / BEEPER_PWM_FREQUENCY, PWM_TIMER_MHZ);
    TIM_Cmd(BEEPER_PWM_TIMER, ENABLE);

    switch (BEEPER_PWM_TIMER_CH) {
    case TIM_Channel_1:
        TIM_OC1Init(BEEPER_PWM_TIMER, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(BEEPER_PWM_TIMER, TIM_OCPreload_Enable);
        break;
    case TIM_Channel_2:
        TIM_OC2Init(BEEPER_PWM_TIMER, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(BEEPER_PWM_TIMER, TIM_OCPreload_Enable);
        break;
    case TIM_Channel_3:
        TIM_OC3Init(BEEPER_PWM_TIMER, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(BEEPER_PWM_TIMER, TIM_OCPreload_Enable);
        break;
    case TIM_Channel_4:
        TIM_OC4Init(BEEPER_PWM_TIMER, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(BEEPER_PWM_TIMER, TIM_OCPreload_Enable);
        break;
    }
    TIM_CtrlPWMOutputs(BEEPER_PWM_TIMER, DISABLE);
}
#endif

void beeperInit(const beeperDevConfig_t *config)
{
#if !defined(BEEPER)
    UNUSED(config);
#else
    beeperIO = IOGetByTag(config->ioTag);
    beeperInverted = config->isInverted;

    if (beeperIO) {
        IOInit(beeperIO, OWNER_BEEPER, RESOURCE_OUTPUT, 0);
#if defined(BEEPER_PWM)
        configBeeperPWMTimer(config);
        IOConfigGPIO(beeperIO, config->isOD ? IOCFG_AF_OD : IOCFG_AF_PP);
#else
        IOConfigGPIO(beeperIO, config->isOD ? IOCFG_OUT_OD : IOCFG_OUT_PP);
#endif
    }

    systemBeep(false);
#endif
}
