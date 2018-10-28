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

#include "platform.h"

#ifdef USE_SPI

#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "bus_spi.h"

typedef struct spiDefaultConfig_s {
    SPIDevice device;
    ioTag_t sck;
    ioTag_t miso;
    ioTag_t mosi;
} spiDefaultConfig_t;

const spiDefaultConfig_t spiDefaultConfig[] = {
#ifdef USE_SPI_DEVICE_1
    { SPIDEV_1, IO_TAG(SPI1_SCK_PIN), IO_TAG(SPI1_MISO_PIN), IO_TAG(SPI1_MOSI_PIN) },
#endif
#ifdef USE_SPI_DEVICE_2
    { SPIDEV_2, IO_TAG(SPI2_SCK_PIN), IO_TAG(SPI2_MISO_PIN), IO_TAG(SPI2_MOSI_PIN) },
#endif
#ifdef USE_SPI_DEVICE_3
    { SPIDEV_3, IO_TAG(SPI3_SCK_PIN), IO_TAG(SPI3_MISO_PIN), IO_TAG(SPI3_MOSI_PIN) },
#endif
#ifdef USE_SPI_DEVICE_4
    { SPIDEV_4, IO_TAG(SPI4_SCK_PIN), IO_TAG(SPI4_MISO_PIN), IO_TAG(SPI4_MOSI_PIN) },
#endif
};

PG_REGISTER_ARRAY_WITH_RESET_FN(spiPinConfig_t, SPIDEV_COUNT, spiPinConfig, PG_SPI_PIN_CONFIG, 1);

void pgResetFn_spiPinConfig(spiPinConfig_t *spiPinConfig)
{
    for (size_t i = 0 ; i < ARRAYLEN(spiDefaultConfig) ; i++) {
        const spiDefaultConfig_t *defconf = &spiDefaultConfig[i];
        spiPinConfig[defconf->device].ioTagSck = defconf->sck;
        spiPinConfig[defconf->device].ioTagMiso = defconf->miso;
        spiPinConfig[defconf->device].ioTagMosi = defconf->mosi;
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(spiCs_t, SPI_PREINIT_IPU_COUNT, spiPreinitIPUConfig, PG_SPI_PREINIT_IPU_CONFIG, 0);
PG_REGISTER_ARRAY(spiCs_t, SPI_PREINIT_OPU_COUNT, spiPreinitOPUConfig, PG_SPI_PREINIT_OPU_CONFIG, 0);

// Initialization values for input pull-up are listed here.
// Explicit output with pull-up should handled in target dependent config.c.
// Generic target will be specifying both values by resource commands.

ioTag_t preinitIPUList[SPI_PREINIT_IPU_COUNT] = {
#ifdef GYRO_1_CS_PIN
    IO_TAG(GYRO_1_CS_PIN),
#endif
#ifdef GYRO_2_CS_PIN
    IO_TAG(GYRO_2_CS_PIN),
#endif
#ifdef L3GD20_CS_PIN
    IO_TAG(L3GD20_CS_PIN),
#endif
#ifdef SDCARD_SPI_CS_PIN
    IO_TAG(SDCARD_SPI_CS_PIN),
#endif
#ifdef BMP280_CS_PIN
    IO_TAG(BMP280_CS_PIN),
#endif
#ifdef MS5611_CS_PIN
    IO_TAG(MS5611_CS_PIN),
#endif
#ifdef LPS_CS_PIN
    IO_TAG(LPS_CS_PIN),
#endif
#ifdef HMC5883_CS_PIN
    IO_TAG(HMC5883_CS_PIN),
#endif
#ifdef AK8963_CS_PIN
    IO_TAG(AK8963_CS_PIN),
#endif
#if defined(RTC6705_CS_PIN) && !defined(USE_VTX_RTC6705_SOFTSPI) // RTC6705 soft SPI initialisation handled elsewhere.
    IO_TAG(RTC6705_CS_PIN),
#endif
#ifdef FLASH_CS_PIN
    IO_TAG(FLASH_CS_PIN),
#endif
#if defined(USE_RX_SPI) && !defined(USE_RX_SOFTSPI)
    IO_TAG(RX_NSS_PIN),
#endif
#if defined(MAX7456_SPI_CS_PIN)
    IO_TAG(MAX7456_SPI_CS_PIN),
#endif
    IO_TAG(NONE)
};

void pgResetFn_spiPreinitIPUConfig(spiCs_t *config)
{
    int puPins = 0;

    for (int i = 0 ; i < SPI_PREINIT_IPU_COUNT ; i++) {
        for (int j = 0 ; j < i ; j++) {
            if (config[j].csnTag == preinitIPUList[i]) {
                goto next;
            }
        }
        config[puPins++].csnTag = preinitIPUList[i];
    next:;
    }
}
#endif
