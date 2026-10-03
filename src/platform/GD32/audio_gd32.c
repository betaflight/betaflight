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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"

#include "drivers/audio.h"

void audioSetupIO(void)
{
    rcu_periph_clock_enable(RCU_DAC);
    rcu_periph_clock_enable(RCU_TIMER5);

    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_4);
}

void audioGenerateWhiteNoise(void)
{
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);

    timer_deinit(TIMER5);
    timer_parameter_struct timer_initpara;
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER5, &timer_initpara);

    timer_master_slave_mode_config(TIMER5, TIMER_MASTER_SLAVE_MODE_ENABLE);
    timer_master_output_trigger_source_select(TIMER5, TIMER_TRI_OUT_SRC_UPDATE);

    timer_enable(TIMER5);

    dac_deinit(DAC0);
    dac_trigger_source_config(DAC0, DAC_OUT0, DAC_TRIGGER_T5_TRGO);
    dac_trigger_enable(DAC0, DAC_OUT0);
    dac_output_buffer_enable(DAC0, DAC_OUT0);

    dac_wave_mode_config(DAC0, DAC_OUT0, DAC_WAVE_MODE_LFSR);
    dac_lfsr_noise_config(DAC0, DAC_OUT0, DAC_LFSR_BITS10_0);
    dac_data_set(DAC0, DAC_OUT0, DAC_ALIGN_12B_R, 0xD00);

    dac_enable(DAC0, DAC_OUT0);
}

#define TONE_SPREAD 8

void audioPlayTone(uint8_t tone)
{
    uint32_t autoreload_value = 64 + (MAX(TONE_MIN,MIN(tone, TONE_MAX)) * TONE_SPREAD);
    timer_autoreload_value_config(TIMER5, autoreload_value);
}

void audioSilence(void)
{
    dac_disable(DAC0, DAC_OUT0);
    timer_disable(TIMER5);
}
