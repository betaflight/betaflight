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

#if defined(USE_SDCARD_SDIO)

#include "pg/pg_ids.h"
#include "pg/sdio.h"

PG_REGISTER_WITH_RESET_TEMPLATE(sdioConfig_t, sdioConfig, PG_SDIO_CONFIG, 0);

PG_RESET_TEMPLATE(sdioConfig_t, sdioConfig,
    .clockBypass = 0,
    .useCache = 0,
    .use4BitWidth = true,
    .dmaopt = SDCARD_SDIO_DMA_OPT,
);

#endif
