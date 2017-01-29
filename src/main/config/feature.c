/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "config/feature.h"

static uint32_t activeFeaturesLatch = 0;

void latchActiveFeatures()
{
    activeFeaturesLatch = featureConfig()->enabledFeatures;
}

bool featureConfigured(uint32_t mask)
{
    return featureConfig()->enabledFeatures & mask;
}

bool feature(uint32_t mask)
{
    return activeFeaturesLatch & mask;
}

void featureSet(uint32_t mask)
{
    featureConfigMutable()->enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    featureConfigMutable()->enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    featureConfigMutable()->enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return featureConfig()->enabledFeatures;
}
