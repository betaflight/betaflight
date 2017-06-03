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

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    NRF24RX_V202_250K = 0,
    NRF24RX_V202_1M,
    NRF24RX_SYMA_X,
    NRF24RX_SYMA_X5C,
    NRF24RX_CX10,
    NRF24RX_CX10A,
    NRF24RX_H8_3D,
    NRF24RX_INAV,
    RFM22_ELERES,
    NRF24RX_PROTOCOL_COUNT
} rx_spi_protocol_e;

typedef enum {
    RX_SPI_RECEIVED_NONE = 0,
    RX_SPI_RECEIVED_BIND,
    RX_SPI_RECEIVED_DATA
} rx_spi_received_e;

// RC channels in AETR order
typedef enum {
    RC_SPI_ROLL = 0,
    RC_SPI_PITCH,
    RC_SPI_THROTTLE,
    RC_SPI_YAW,
    RC_SPI_AUX1,
    RC_SPI_AUX2,
    RC_SPI_AUX3,
    RC_SPI_AUX4,
    RC_SPI_AUX5,
    RC_SPI_AUX6,
    RC_SPI_AUX7,
    RC_SPI_AUX8,
    RC_SPI_AUX9,
    RC_SPI_AUX10,
    RC_SPI_AUX11,
    RC_SPI_AUX12,
    RC_SPI_AUX13,
    RC_SPI_AUX14
} rc_spi_aetr_e;

// RC channels as used by deviation
#define RC_CHANNEL_RATE        RC_SPI_AUX1
#define RC_CHANNEL_FLIP        RC_SPI_AUX2
#define RC_CHANNEL_PICTURE     RC_SPI_AUX3
#define RC_CHANNEL_VIDEO       RC_SPI_AUX4
#define RC_CHANNEL_HEADLESS    RC_SPI_AUX5
#define RC_CHANNEL_RTH         RC_SPI_AUX6 // return to home

uint8_t rxSpiFrameStatus(void);
bool rxSpiInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
