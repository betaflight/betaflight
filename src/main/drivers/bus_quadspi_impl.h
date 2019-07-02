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
 *
 * Author: Dominic Clifton
 */

#pragma once


typedef struct quadSpiPinDef_s {
    ioTag_t pin;
#if defined(STM32H7)
    uint8_t af;
#endif
} quadSpiPinDef_t;

#if defined(STM32H7)
#define MAX_QUADSPI_PIN_SEL 3
#endif

typedef struct quadSpiHardware_s {
    QUADSPIDevice device;
    QUADSPI_TypeDef *reg;
    quadSpiPinDef_t clkPins[MAX_QUADSPI_PIN_SEL];
    quadSpiPinDef_t bk1IO0Pins[MAX_QUADSPI_PIN_SEL];
    quadSpiPinDef_t bk1IO1Pins[MAX_QUADSPI_PIN_SEL];
    quadSpiPinDef_t bk1IO2Pins[MAX_QUADSPI_PIN_SEL];
    quadSpiPinDef_t bk1IO3Pins[MAX_QUADSPI_PIN_SEL];
    quadSpiPinDef_t bk1CSPins[MAX_QUADSPI_PIN_SEL];
    quadSpiPinDef_t bk2IO0Pins[MAX_QUADSPI_PIN_SEL];
    quadSpiPinDef_t bk2IO1Pins[MAX_QUADSPI_PIN_SEL];
    quadSpiPinDef_t bk2IO2Pins[MAX_QUADSPI_PIN_SEL];
    quadSpiPinDef_t bk2IO3Pins[MAX_QUADSPI_PIN_SEL];
    quadSpiPinDef_t bk2CSPins[MAX_QUADSPI_PIN_SEL];

    rccPeriphTag_t rcc;
} quadSpiHardware_t;

extern const quadSpiHardware_t quadSpiHardware[];

typedef struct QUADSPIDevice_s {
    QUADSPI_TypeDef *dev;
    ioTag_t clk;
    ioTag_t bk1IO0;
    ioTag_t bk1IO1;
    ioTag_t bk1IO2;
    ioTag_t bk1IO3;
    ioTag_t bk1CS;
    ioTag_t bk2IO0;
    ioTag_t bk2IO1;
    ioTag_t bk2IO2;
    ioTag_t bk2IO3;
    ioTag_t bk2CS;
#if defined(STM32H7)
    uint8_t bk1IO0AF;
    uint8_t bk1IO1AF;
    uint8_t bk1IO2AF;
    uint8_t bk1IO3AF;
    uint8_t bk1CSAF;
    uint8_t bk2IO0AF;
    uint8_t bk2IO1AF;
    uint8_t bk2IO2AF;
    uint8_t bk2IO3AF;
    uint8_t bk2CSAF;
#endif
    rccPeriphTag_t rcc;
    volatile uint16_t errorCount;
#if defined(USE_HAL_DRIVER)
    QSPI_HandleTypeDef hquadSpi;
#endif
} quadSpiDevice_t;

extern quadSpiDevice_t quadSpiDevice[QUADSPIDEV_COUNT];

void quadSpiInitDevice(QUADSPIDevice device);
uint32_t quadSpiTimeoutUserCallback(QUADSPI_TypeDef *instance);
