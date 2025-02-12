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


#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx_neros.h"

PG_REGISTER_WITH_RESET_TEMPLATE(nelrsConfig_t, nelrsConfig, PG_NELRS_CONFIG, 0);

PG_RESET_TEMPLATE(nelrsConfig_t, nelrsConfig,
    .bindPhraseLow = "testtesttest",
    .bindPhraseHigh = "testtesttest",
    .startFrequencyLow = 9035,
    .midFrequencyLow = 9150,
    .endFrequencyLow = 9269,
    .numChannelsLow = 40,
    .startFrequencyHigh = 9035,
    .midFrequencyHigh = 9150,
    .endFrequencyHigh = 9269,
    .numChannelsHigh = 40
);

