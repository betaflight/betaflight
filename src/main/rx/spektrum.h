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

//VTX enums and struct
typedef enum
{
	FATSHARK,
	RACEBAND,
	E,
	B,
	A
} VTX_BAND;

typedef enum
{
	POWER_25MW,
	POWER_250MW,
	POWER_500MW
} VTX_POWER;

typedef enum
{
	US,
	EU
} VTX_REGION;

typedef enum
{
	ACTIVE,      // turn on power
	PIT          // low power mode while in pit
} VTX_PIT;

typedef struct
{
	uint8_t vtxChannel;
	VTX_BAND vtxBand;
	VTX_POWER vtxPower;
	VTX_REGION vtxRegion;
	VTX_PIT vtxPit;
} SPM_VTX_DATA;

#define SPEKTRUM_SAT_BIND_DISABLED 0
#define SPEKTRUM_SAT_BIND_MAX 10

uint8_t spektrumFrameStatus(void);
struct rxConfig_s;
void spektrumBind(struct rxConfig_s *rxConfig);

