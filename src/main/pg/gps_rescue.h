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

#pragma once

#include <stdint.h>

#include "pg/pg.h"

typedef struct gpsRescue_s {

    uint16_t maxRescueAngle; // degrees
    uint16_t returnAltitudeM; // meters
    uint16_t descentDistanceM; // meters
    uint16_t groundSpeedCmS; // centimeters per second
    uint8_t  throttleP, throttleI, throttleD;
    uint8_t  yawP;
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint8_t  minSats;
    uint8_t  velP, velI, velD;
    uint16_t minStartDistM; // meters
    uint8_t  sanityChecks;
    uint8_t  allowArmingWithoutFix;
    uint8_t  useMag;
    uint8_t  altitudeMode;
    uint16_t ascendRate;
    uint16_t descendRate;
    uint16_t initialClimbM; // meters
    uint8_t  rollMix;
    uint8_t  disarmThreshold;
    uint8_t  pitchCutoffHz;
    uint8_t  imuYawGain;
} gpsRescueConfig_t;

PG_DECLARE(gpsRescueConfig_t, gpsRescueConfig);
