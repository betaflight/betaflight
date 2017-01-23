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
} DSMX_VTX_BAND;

typedef enum
{
	POWER_25MW,
	POWER_250MW,
	POWER_500MW
} DSMX_VTX_POWER;

typedef enum
{
	US,
	EU
} DSMX_VTX_REGION;

typedef enum
{
	ACTIVE,      // turn on power
	PIT          // low power mode while in pit
} DSMX_VTX_PIT;

typedef struct
{
	uint8_t vtxChannel;
	DSMX_VTX_BAND vtxBand;
	DSMX_VTX_POWER vtxPower;
	DSMX_VTX_REGION vtxRegion;
	DSMX_VTX_PIT vtxPit;
} DSMX_VTX_DATA;
extern DSMX_VTX_DATA vtxData;

#define SPEKTRUM_SAT_BIND_DISABLED     0
#define SPEKTRUM_SAT_BIND_MAX         10

#define SPEK_FRAME_SIZE             16
#define SRXL_FRAME_OVERHEAD         5
#define SRXL_FRAME_SIZE_MAX         (SPEK_FRAME_SIZE + SRXL_FRAME_OVERHEAD)

void spektrumBind(rxConfig_t *rxConfig);
bool spektrumInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);

void srxlRxWriteTelemetryData(const void *data, int len);
bool srxlRxIsActive(void);
