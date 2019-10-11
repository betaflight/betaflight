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

#ifdef USE_QUADSPI

#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "bus_quadspi.h"

typedef struct quadSpiDefaultConfig_s {
    QUADSPIDevice device;
    ioTag_t clk;

    // Note: Either or both CS pin may be used in DUAL_FLASH mode, any unused pins should be IO_NONE
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

    uint8_t mode;

    // CS pins can be under software control, useful when using BK1CS as the CS pin for BK2 in non-DUAL-FLASH mode.
    uint8_t csFlags;
} quadSpiDefaultConfig_t;

const quadSpiDefaultConfig_t quadSpiDefaultConfig[] = {
#ifdef USE_QUADSPI_DEVICE_1
    {
        QUADSPIDEV_1,
        IO_TAG(QUADSPI1_SCK_PIN),
        IO_TAG(QUADSPI1_BK1_IO0_PIN), IO_TAG(QUADSPI1_BK1_IO1_PIN), IO_TAG(QUADSPI1_BK1_IO2_PIN), IO_TAG(QUADSPI1_BK1_IO3_PIN), IO_TAG(QUADSPI1_BK1_CS_PIN),
        IO_TAG(QUADSPI1_BK2_IO0_PIN), IO_TAG(QUADSPI1_BK2_IO1_PIN), IO_TAG(QUADSPI1_BK2_IO2_PIN), IO_TAG(QUADSPI1_BK2_IO3_PIN), IO_TAG(QUADSPI1_BK2_CS_PIN),
        QUADSPI1_MODE,
        QUADSPI1_CS_FLAGS
    },
#endif
};

PG_REGISTER_ARRAY_WITH_RESET_FN(quadSpiConfig_t, QUADSPIDEV_COUNT, quadSpiConfig, PG_QUADSPI_CONFIG, 1);

void pgResetFn_quadSpiConfig(quadSpiConfig_t *quadSpiConfig)
{
    for (size_t i = 0 ; i < ARRAYLEN(quadSpiDefaultConfig) ; i++) {
        const quadSpiDefaultConfig_t *defconf = &quadSpiDefaultConfig[i];
        quadSpiConfig[defconf->device].ioTagClk = defconf->clk;

        quadSpiConfig[defconf->device].ioTagBK1IO0 = defconf->bk1IO0;
        quadSpiConfig[defconf->device].ioTagBK1IO1 = defconf->bk1IO1;
        quadSpiConfig[defconf->device].ioTagBK1IO2 = defconf->bk1IO2;
        quadSpiConfig[defconf->device].ioTagBK1IO3 = defconf->bk1IO3;
        quadSpiConfig[defconf->device].ioTagBK1CS = defconf->bk1CS;

        quadSpiConfig[defconf->device].ioTagBK2IO0 = defconf->bk2IO0;
        quadSpiConfig[defconf->device].ioTagBK2IO1 = defconf->bk2IO1;
        quadSpiConfig[defconf->device].ioTagBK2IO2 = defconf->bk2IO2;
        quadSpiConfig[defconf->device].ioTagBK2IO3 = defconf->bk2IO3;
        quadSpiConfig[defconf->device].ioTagBK2CS = defconf->bk2CS;

        quadSpiConfig[defconf->device].mode = defconf->mode;
        quadSpiConfig[defconf->device].csFlags = defconf->csFlags;
    }
}
#endif
