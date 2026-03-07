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
    quadSpiDevice_e device;
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
    /* Provide safe defaults when some per-target macros are missing */
    #ifdef QUADSPI1_SCK_PIN
        #define QSPI1_CLK_TAG IO_TAG(QUADSPI1_SCK_PIN)
    #else
        #define QSPI1_CLK_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_BK1_IO0_PIN
        #define QSPI1_BK1_IO0_TAG IO_TAG(QUADSPI1_BK1_IO0_PIN)
    #else
        #define QSPI1_BK1_IO0_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_BK1_IO1_PIN
        #define QSPI1_BK1_IO1_TAG IO_TAG(QUADSPI1_BK1_IO1_PIN)
    #else
        #define QSPI1_BK1_IO1_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_BK1_IO2_PIN
        #define QSPI1_BK1_IO2_TAG IO_TAG(QUADSPI1_BK1_IO2_PIN)
    #else
        #define QSPI1_BK1_IO2_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_BK1_IO3_PIN
        #define QSPI1_BK1_IO3_TAG IO_TAG(QUADSPI1_BK1_IO3_PIN)
    #else
        #define QSPI1_BK1_IO3_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_BK1_CS_PIN
        #define QSPI1_BK1_CS_TAG IO_TAG(QUADSPI1_BK1_CS_PIN)
    #else
        #define QSPI1_BK1_CS_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_BK2_IO0_PIN
        #define QSPI1_BK2_IO0_TAG IO_TAG(QUADSPI1_BK2_IO0_PIN)
    #else
        #define QSPI1_BK2_IO0_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_BK2_IO1_PIN
        #define QSPI1_BK2_IO1_TAG IO_TAG(QUADSPI1_BK2_IO1_PIN)
    #else
        #define QSPI1_BK2_IO1_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_BK2_IO2_PIN
        #define QSPI1_BK2_IO2_TAG IO_TAG(QUADSPI1_BK2_IO2_PIN)
    #else
        #define QSPI1_BK2_IO2_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_BK2_IO3_PIN
        #define QSPI1_BK2_IO3_TAG IO_TAG(QUADSPI1_BK2_IO3_PIN)
    #else
        #define QSPI1_BK2_IO3_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_BK2_CS_PIN
        #define QSPI1_BK2_CS_TAG IO_TAG(QUADSPI1_BK2_CS_PIN)
    #else
        #define QSPI1_BK2_CS_TAG IO_TAG_NONE
    #endif
    #ifdef QUADSPI1_MODE
        #define QSPI1_MODE_VAL QUADSPI1_MODE
    #else
        #define QSPI1_MODE_VAL 0
    #endif
    #ifdef QUADSPI1_CS_FLAGS
        #define QSPI1_CS_FLAGS_VAL QUADSPI1_CS_FLAGS
    #else
        #define QSPI1_CS_FLAGS_VAL 0
    #endif

    {
        QUADSPIDEV_1,
        QSPI1_CLK_TAG,
        QSPI1_BK1_IO0_TAG, QSPI1_BK1_IO1_TAG, QSPI1_BK1_IO2_TAG, QSPI1_BK1_IO3_TAG, QSPI1_BK1_CS_TAG,
        QSPI1_BK2_IO0_TAG, QSPI1_BK2_IO1_TAG, QSPI1_BK2_IO2_TAG, QSPI1_BK2_IO3_TAG, QSPI1_BK2_CS_TAG,
        QSPI1_MODE_VAL,
        QSPI1_CS_FLAGS_VAL
    },
    /* Undefine helper macros to avoid bleeding into other units */
    #undef QSPI1_CLK_TAG
    #undef QSPI1_BK1_IO0_TAG
    #undef QSPI1_BK1_IO1_TAG
    #undef QSPI1_BK1_IO2_TAG
    #undef QSPI1_BK1_IO3_TAG
    #undef QSPI1_BK1_CS_TAG
    #undef QSPI1_BK2_IO0_TAG
    #undef QSPI1_BK2_IO1_TAG
    #undef QSPI1_BK2_IO2_TAG
    #undef QSPI1_BK2_IO3_TAG
    #undef QSPI1_BK2_CS_TAG
    #undef QSPI1_MODE_VAL
    #undef QSPI1_CS_FLAGS_VAL
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
