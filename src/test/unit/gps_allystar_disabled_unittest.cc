/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 */

#include <stddef.h>

extern "C" {
#include "platform.h"
#include "pg/gps.h"
}

#include "gtest/gtest.h"

TEST(GpsAllystarDisabled, GpsConfigHasNoAllystarField)
{
    EXPECT_EQ(offsetof(gpsConfig_t, gps_baud) + sizeof(((gpsConfig_t *)nullptr)->gps_baud), sizeof(gpsConfig_t));
}
