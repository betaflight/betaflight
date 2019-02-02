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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"


PG_REGISTER_WITH_RESET_TEMPLATE(featureConfig_t, featureConfig, PG_FEATURE_CONFIG, 0);

PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
    .enabledFeatures = DEFAULT_FEATURES | DEFAULT_RX_FEATURE | FEATURE_DYNAMIC_FILTER | FEATURE_ANTI_GRAVITY | FEATURE_AIRMODE,
);

void featureSet(const uint32_t mask, uint32_t *features)
{
    *features |= mask;
}

void featureClear(const uint32_t mask, uint32_t *features)
{
    *features &= ~(mask);
}

bool featureIsEnabled(const uint32_t mask)
{
    return featureConfig()->enabledFeatures & mask;
}

void featureEnable(const uint32_t mask)
{
    featureSet(mask, &featureConfigMutable()->enabledFeatures);
}

void featureDisable(const uint32_t mask)
{
    featureClear(mask, &featureConfigMutable()->enabledFeatures);
}

void featureDisableAll(void)
{
    featureConfigMutable()->enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return featureConfig()->enabledFeatures;
}
