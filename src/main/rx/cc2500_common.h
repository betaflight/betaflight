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

#include "rx/rx_spi.h"

typedef struct cc2500RegisterConfigElement_s {
    uint8_t registerID;
    uint8_t registerValue;
} cc2500RegisterConfigElement_t;

uint16_t cc2500getRssiDbm(void);
void cc2500setRssiDbm(uint8_t value);
#if defined(USE_RX_CC2500_SPI_PA_LNA) && defined(USE_RX_CC2500_SPI_DIVERSITY)
void cc2500switchAntennae(void);
#endif
#if defined(USE_RX_CC2500_SPI_PA_LNA)
void cc2500TxEnable(void);
void cc2500TxDisable(void);
#endif
bool cc2500SpiInit(void);
void cc2500ApplyRegisterConfig(const cc2500RegisterConfigElement_t *configArrayPtr, int configSize);
