/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "drivers/system.h"
#include "drivers/bus_spi.h"
#include "drivers/sensor.h"
#include "drivers/io.h"
#include "drivers/timer.h"

#include "hardware_revision.h"

static const char * const hardwareRevisionNames[] = {
    "Unknown",
    "OmnibusF4 V1",
    "OmnibusF4 V2"
};

uint8_t hardwareRevision = UNKNOWN;

#define DISABLE_SPI_CS       IOHi(omnibusf4SpiCsPin)
#define ENABLE_SPI_CS        IOLo(omnibusf4SpiCsPin)

#define SPI_DEVICE_NONE (0)
#define SPI_DEVICE_FLASH (1)

#define M25P16_INSTRUCTION_RDID        0x9F
#define FLASH_M25P16_ID                0x20ba18
#define JEDEC_ID_WINBOND_W25Q128       0xEF4018

static IO_t omnibusf4SpiCsPin = IO_NONE;

uint8_t detectSpiDevice(void)
{
    spiInit(SPIDEV_3);
#ifdef M25P16_CS_PIN
    omnibusf4SpiCsPin = IOGetByTag(IO_TAG(M25P16_CS_PIN));
    IOInit(omnibusf4SpiCsPin, OWNER_FLASH_CS, 0);
    IOConfigGPIO(omnibusf4SpiCsPin, IOCFG_OUT_PP);
#endif

    uint8_t out[] = { M25P16_INSTRUCTION_RDID, 0, 0, 0 };
    uint8_t in[4];
    uint32_t flash_id;

    // try autodetect flash chip
    delay(50); // short delay required after initialisation of SPI device instance.
    ENABLE_SPI_CS;
    spiTransfer(M25P16_SPI_INSTANCE, in, out, sizeof(out));
    DISABLE_SPI_CS;

    flash_id = in[1] << 16 | in[2] << 8 | in[3];
    if (flash_id == FLASH_M25P16_ID || flash_id == JEDEC_ID_WINBOND_W25Q128)
        return SPI_DEVICE_FLASH;

    return SPI_DEVICE_NONE;
}

void detectHardwareRevision(void) {
    uint8_t detectedSpiDevice = detectSpiDevice();

    if (detectedSpiDevice == SPI_DEVICE_FLASH)
        hardwareRevision = OMNIBUSF4V1;
    else
        hardwareRevision = OMNIBUSF4V2;
}

void updateHardwareRevision(void) {
}
