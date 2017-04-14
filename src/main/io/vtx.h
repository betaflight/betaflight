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

#pragma once

#include "fc/rc_controls.h"

#define VTX_BAND_MIN                            1
#define VTX_BAND_MAX                            5
#define VTX_CHANNEL_MIN                         1
#define VTX_CHANNEL_MAX                         8
#define MAX_CHANNEL_ACTIVATION_CONDITION_COUNT  10

typedef struct vtxChannelActivationCondition_s {
    uint8_t auxChannelIndex;
    uint8_t band;
    uint8_t channel;
    channelRange_t range;
} vtxChannelActivationCondition_t;

typedef struct vtxConfig_s {
    uint8_t vtx_power;
    uint8_t vtx_channel; //1-8
    uint8_t vtx_band; //1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
    uint8_t vtx_mode; //0=ch+band 1=mhz
    uint16_t vtx_mhz; //5740
    vtxChannelActivationCondition_t vtxChannelActivationConditions[MAX_CHANNEL_ACTIVATION_CONDITION_COUNT];
} vtxConfig_t;

PG_DECLARE(vtxConfig_t, vtxConfig);

void vtxInit(void);
bool canUpdateVTX(void);
void vtxIncrementBand(void);
void vtxDecrementBand(void);
void vtxIncrementChannel(void);
void vtxDecrementChannel(void);
void vtxUpdateActivatedChannel(void);

