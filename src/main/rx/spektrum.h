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

#define SPEKTRUM_SAT_BIND_DISABLED     0
#define SPEKTRUM_SAT_BIND_MAX         10

#define SPEK_FRAME_SIZE             16
#define SRXL_FRAME_OVERHEAD         5
#define SRXL_FRAME_SIZE_MAX         (SPEK_FRAME_SIZE + SRXL_FRAME_OVERHEAD)

// Spektrum system type values
#define SPEKTRUM_DSM2_22 0x01
#define SPEKTRUM_DSM2_11 0x12
#define SPEKTRUM_DSMX_22 0xa2
#define SPEKTRUM_DSMX_11 0xb2

// Spektrum RSSI signal strength range, in dBm
#define SPEKTRUM_RSSI_MAX         (-42)
#define SPEKTRUM_RSSI_MIN         (-92)

// Spektrum RSSI reported value limit at or below which signals a fade instead of a real RSSI
#define SPEKTRUM_RSSI_FADE_LIMIT (-100)

typedef struct
{
  int8_t  dBm;
  uint8_t reportAs;
} stru_dbm_table;

void spektrumBind(rxConfig_t *rxConfig);
bool spektrumInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);

void srxlRxWriteTelemetryData(const void *data, int len);
bool srxlRxIsActive(void);
