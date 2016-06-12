/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "build_config.h"

#include "bus_spi.h"

#include "barometer.h"
#include "barometer_bmp280.h"

#define DISABLE_BMP280       GPIO_SetBits(BMP280_CS_GPIO,   BMP280_CS_PIN)
#define ENABLE_BMP280        GPIO_ResetBits(BMP280_CS_GPIO, BMP280_CS_PIN)

extern int32_t bmp280_up;
extern int32_t bmp280_ut;

bool bmp280WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_BMP280;
    spiTransferByte(BMP280_SPI_INSTANCE, reg & 0x7F);
    spiTransferByte(BMP280_SPI_INSTANCE, data);
    DISABLE_BMP280;

    return true;
}

bool bmp280ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_BMP280;
    spiTransferByte(BMP280_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(BMP280_SPI_INSTANCE, data, NULL, length);
    DISABLE_BMP280;

    return true;
}

void bmp280SpiInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return;
    }

#ifdef STM32F303
    RCC_AHBPeriphClockCmd(BMP280_CS_GPIO_CLK_PERIPHERAL, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = BMP280_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(BMP280_CS_GPIO, &GPIO_InitStructure);
#endif

#ifdef STM32F10X
    RCC_APB2PeriphClockCmd(BMP280_CS_GPIO_CLK_PERIPHERAL, ENABLE);

    gpio_config_t gpio;
    gpio.mode = Mode_Out_PP;
    gpio.pin = BMP280_CS_PIN;
    gpio.speed = Speed_50MHz;
    gpioInit(BMP280_CS_GPIO, &gpio);
#endif

    GPIO_SetBits(BMP280_CS_GPIO, BMP280_CS_PIN);

    spiSetDivisor(BMP280_SPI_INSTANCE, SPI_9MHZ_CLOCK_DIVIDER);

    hardwareInitialised = true;
}

void bmp280_spi_start_up(void)
{
    // start measurement
    // set oversampling + power mode (forced), and start sampling
    bmp280WriteRegister(BMP280_CTRL_MEAS_REG, BMP280_MODE);
}

void bmp280_spi_get_up(void)
{
    uint8_t data[BMP280_DATA_FRAME_SIZE];

    // read data from sensor
    bmp280ReadRegister(BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);
    bmp280_up = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
    bmp280_ut = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
}
