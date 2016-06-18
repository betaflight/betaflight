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

#pragma once

#define DP_5G_MASK          0x7000
#define PA5G_BS_MASK        0x0E00
#define PA5G_PW_MASK        0x0180
#define PD_Q5G_MASK         0x0040
#define QI_5G_MASK          0x0038
#define PA_BS_MASK          0x0007

#define PA_CONTROL_DEFAULT  0x4FBD

#define CHANNELS_PER_BAND   8
#define BANDS_NUMBER        5

extern uint16_t const vtx_freq[];
extern uint16_t current_vtx_channel;

void rtc6705_soft_spi_init(void);
void rtc6705_soft_spi_set_channel(uint16_t channel_freq);
void rtc6705_soft_spi_set_rf_power(uint8_t reduce_power);

