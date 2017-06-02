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

#include <stdint.h>

typedef enum {
    RX_SPI_SOFTSPI,
    RX_SPI_HARDSPI,
} rx_spi_type_e;

#define RX_SPI_MAX_PAYLOAD_SIZE 32

void rxSpiDeviceInit(rx_spi_type_e spiType);
uint8_t rxSpiTransferByte(uint8_t data);
uint8_t rxSpiWriteByte(uint8_t data);
uint8_t rxSpiWriteCommand(uint8_t command, uint8_t data);
uint8_t rxSpiWriteCommandMulti(uint8_t command, const uint8_t *data, uint8_t length);
uint8_t rxSpiReadCommand(uint8_t command, uint8_t commandData);
uint8_t rxSpiReadCommandMulti(uint8_t command, uint8_t commandData, uint8_t *retData, uint8_t length);
bool    rxSpiCheckIrq();

