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

typedef struct vtxConfig_s {
    uint8_t channel;         // 0 based index, 0 = lowest.  maximum depends on driver implementation
    uint8_t band;            // 0 based index, 0 = lowest.  maximum depends on driver implementation
    uint8_t rfPower;         // 0 based index, 0 = lowest.  maximum depends on driver implementation
    uint8_t enabledOnBoot;
} vtxConfig_t;

PG_DECLARE(vtxConfig_t, vtxConfig);

// runtime state.
typedef struct vtxState_s {
    uint8_t channel;
    uint8_t band;
    uint8_t rfPower;
    uint8_t enabled;
} vtxState_t;

extern vtxState_t vtxState;

// VTX Control
void initVTXState(void);
bool isUsingVTXSwitch(void);
void updateVTXState(void);


// VTX API

// common methods
void vtxInit(void);
void vtxIOInit(void);
void vtxTogglePower(void);
void handleVTXControlButton(void);
void vtxSaveState(void);

// hardware specific implementation methods
void vtxEnable(void);
void vtxDisable(void);
void vtxCycleChannel(void);
void vtxCycleBand(void);
void vtxCycleRFPower(void); // 0 = lowest

