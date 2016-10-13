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

typedef enum {
    FC_STATE_ARM = 0,
    FC_STATE_ANGLE,
    FC_STATE_HORIZON,
    FC_STATE_BARO,
    FC_STATE_MAG,
    FC_STATE_HEADFREE,
    FC_STATE_HEADADJ,
    FC_STATE_CAMSTAB,
    FC_STATE_CAMTRIG,
    FC_STATE_GPSHOME,
    FC_STATE_GPSHOLD,
    FC_STATE_PASSTHRU,
    FC_STATE_BEEPERON,
    FC_STATE_LEDMAX,
    FC_STATE_LEDLOW,
    FC_STATE_LLIGHTS,
    FC_STATE_CALIB,
    FC_STATE_GOV,
    FC_STATE_OSD,
    FC_STATE_TELEMETRY,
    FC_STATE_GTUNE,
    FC_STATE_SONAR,
    FC_STATE_SERVO1,
    FC_STATE_SERVO2,
    FC_STATE_SERVO3,
    FC_STATE_BLACKFC_STATE_,
    FC_STATE_FAILSAFE,
    FC_STATE_AIRMODE
} fcStateId_e;


typedef struct fcStatus_s {
    // MSP_STATUS
    uint16_t cycleTime;
    uint16_t i2cErrors;
    uint16_t sensors;
    uint32_t fcState;                // bitmask, see fcStateId_e
    uint8_t profile;

    // MSP_ANALOG
    uint8_t vbat;                    // voltage in 0.1V steps, 168 = 16.8v
    uint16_t rssi;                   // rssi in 0.1% steps, 505 = 50.5%
    uint16_t amperage;               // amperage in 0.01A steps, 12575 = 125.75A
    uint16_t mAhDrawn;               // milliampere hours, 1300mAh

    // calculated
    uint32_t armedDuration;
} fcStatus_t;

extern fcStatus_t fcStatus;

#define OSD_MAX_MOTORS 8
extern uint16_t fcMotors[OSD_MAX_MOTORS];
