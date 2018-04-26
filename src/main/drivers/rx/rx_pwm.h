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

#pragma once

typedef enum {
    INPUT_FILTERING_DISABLED = 0,
    INPUT_FILTERING_ENABLED
} inputFilteringMode_e;

#define PPM_RCVR_TIMEOUT            0
#define PWM_INPUT_PORT_COUNT        8

struct ppmConfig_s;
void ppmRxInit(const struct ppmConfig_s *ppmConfig);
struct pwmConfig_s;
void pwmRxInit(const struct pwmConfig_s *pwmConfig);

uint16_t pwmRead(uint8_t channel);
uint16_t ppmRead(uint8_t channel);

bool isPPMDataBeingReceived(void);
void resetPPMDataReceivedState(void);

bool isPWMDataBeingReceived(void);
