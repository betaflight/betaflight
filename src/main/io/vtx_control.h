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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "pg/pg.h"
#include "fc/rc_modes.h"

#define MAX_CHANNEL_ACTIVATION_CONDITION_COUNT  14

typedef struct vtxChannelActivationCondition_s {
    uint8_t auxChannelIndex;
    uint8_t band;
    uint8_t channel;
    uint8_t power;
    channelRange_t range;
} vtxChannelActivationCondition_t;

typedef struct vtxConfig_s {
    vtxChannelActivationCondition_t vtxChannelActivationConditions[MAX_CHANNEL_ACTIVATION_CONDITION_COUNT];
    uint8_t halfDuplex;
} vtxConfig_t;

PG_DECLARE(vtxConfig_t, vtxConfig);

void vtxControlInit(void);
void vtxControlInputPoll(void);

void vtxIncrementBand(void);
void vtxDecrementBand(void);
void vtxIncrementChannel(void);
void vtxDecrementChannel(void);

void vtxCyclePower(const uint8_t powerStep);
void vtxCycleBandOrChannel(const uint8_t bandStep, const uint8_t channelStep);

void vtxUpdateActivatedChannel(void);

void handleVTXControlButton(void);
