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

#include <string.h>
#include "common/printf.h"

#include "platform.h"

#ifdef USE_VTX_TABLE

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/vtx_table.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

PG_REGISTER_WITH_RESET_FN(vtxTableConfig_t, vtxTableConfig, PG_VTX_TABLE_CONFIG, 0);

void pgResetFn_vtxTableConfig(vtxTableConfig_t *config)
{
    // Clear band names, letters and frequency values

    config->bands = 0;
    config->channels = 0;

    for (int band = 0; band < VTX_TABLE_MAX_BANDS; band++) {
        vtxTableConfigClearBand(config, band);
    }

    // Clear power values and labels

    config->powerLevels = 0;
    vtxTableConfigClearPowerValues(config, 0);
    vtxTableConfigClearPowerLabels(config, 0);
}
#endif
