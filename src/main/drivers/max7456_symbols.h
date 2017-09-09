/* @file max7456_symbols.h
 * @brief max7456 symbols for the mwosd font set
 *
 * @author Nathan Tsoi nathan@vertile.com
 *
 * Copyright (C) 2016 Nathan Tsoi
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#pragma once

#ifdef USE_MAX7456

// Character Symbols
#define SYM_BLANK 0x20

// Satellite Graphics
#define SYM_SAT_L 0x1E
#define SYM_SAT_R 0x1F
//#define SYM_SAT 0x0F  // Not used

// Degrees Icon for HEADING/DIRECTION HOME
#define SYM_DEGREES 0xBD

// Direction arrows
#define SYM_ARROW_UP    0x60
#define SYM_ARROW_2     0x61
#define SYM_ARROW_3     0x62
#define SYM_ARROW_4     0x63
#define SYM_ARROW_RIGHT 0x64
#define SYM_ARROW_6     0x65
#define SYM_ARROW_7     0x66
#define SYM_ARROW_8     0x67
#define SYM_ARROW_DOWN  0x68
#define SYM_ARROW_10    0x69
#define SYM_ARROW_11    0x6A
#define SYM_ARROW_12    0x6B
#define SYM_ARROW_LEFT  0x6C
#define SYM_ARROW_14    0x6D
#define SYM_ARROW_15    0x6E
#define SYM_ARROW_16    0x6F

// Heading Graphics
#define SYM_HEADING_N             0x18
#define SYM_HEADING_S             0x19
#define SYM_HEADING_E             0x1A
#define SYM_HEADING_W             0x1B
#define SYM_HEADING_DIVIDED_LINE  0x1C
#define SYM_HEADING_LINE          0x1D

// FRSKY HUB
#define SYM_CELL0      0xF0
#define SYM_CELL1      0xF1
#define SYM_CELL2      0xF2
#define SYM_CELL3      0xF3
#define SYM_CELL4      0xF4
#define SYM_CELL5      0xF5
#define SYM_CELL6      0xF6
#define SYM_CELL7      0xF7
#define SYM_CELL8      0xF8
#define SYM_CELL9      0xF9
#define SYM_CELLA      0xFA
#define SYM_CELLB      0xFB
#define SYM_CELLC      0xFC
#define SYM_CELLD      0xFD
#define SYM_CELLE      0xFE
#define SYM_CELLF      0xC3

// Map mode
#define SYM_HOME       0x04
#define SYM_AIRCRAFT   0x05
#define SYM_RANGE_100  0x21
#define SYM_RANGE_500  0x22
#define SYM_RANGE_2500 0x23
#define SYM_RANGE_MAX  0x24
#define SYM_DIRECTION  0x72

// GPS Coordinates and Altitude
#define SYM_LAT 0xCA
#define SYM_LON 0xCB
#define SYM_ALT 0xAA

// GPS Mode and Autopilot
#define SYM_3DFIX     0xDF
#define SYM_HOLD      0xEF
#define SYM_G_HOME    0xFF
#define SYM_GHOME     0x9D
#define SYM_GHOME1    0x9E
#define SYM_GHOLD     0xCD
#define SYM_GHOLD1    0xCE
#define SYM_GMISSION  0xB5
#define SYM_GMISSION1 0xB6
#define SYM_GLAND     0xB7
#define SYM_GLAND1    0xB8

// Gimbal active Mode
#define SYM_GIMBAL  0x16
#define SYM_GIMBAL1 0x17

// Sensor´s Presence
#define SYM_ACC     0xA0
#define SYM_MAG     0xA1
#define SYM_BAR     0xA2
#define SYM_GPS     0xA3
#define SYM_MAN     0xC0
#define SYM_MAN1    0xC1
#define SYM_MAN2    0xC2
#define SYM_CHECK   0xBE
#define SYM_BARO10  0xB7
#define SYM_BARO11  0xB8
#define SYM_MAG10   0xB5
#define SYM_MAG11   0xB6

// AH Center screen Graphics
#define SYM_AH_CENTER_LINE        0x26
#define SYM_AH_CENTER_LINE_RIGHT  0x27
#define SYM_AH_CENTER             0x7E
#define SYM_AH_RIGHT              0x02
#define SYM_AH_LEFT               0x03
#define SYM_AH_DECORATION_UP      0xC9
#define SYM_AH_DECORATION_DOWN    0xCF

// AH Bars
#define SYM_AH_BAR9_0 0x80

// Temperature
#define SYM_TEMP_F 0x0D
#define SYM_TEMP_C 0x0E

// Batt evolution
#define SYM_BATT_FULL   0x90
#define SYM_BATT_5      0x91
#define SYM_BATT_4      0x92
#define SYM_BATT_3      0x93
#define SYM_BATT_2      0x94
#define SYM_BATT_1      0x95
#define SYM_BATT_EMPTY  0x96

// Vario
#define SYM_VARIO 0x7F

// Glidescope
#define SYM_GLIDESCOPE 0xE0

// Batt Icon´s
#define SYM_MAIN_BATT 0x97
#define SYM_VID_BAT   0xBF

// Unit Icon´s (Metric)
#define SYM_MS          0x9F
#define SYM_KMH         0xA1
#define SYM_ALTM        0xA7
#define SYM_DISTHOME_M  0xBB
#define SYM_M           0x0C

// Unit Icon´s (Imperial)
#define SYM_FTS         0x99
#define SYM_MPH         0xB0
#define SYM_ALTFT       0xA8
#define SYM_DISTHOME_FT 0xB9
#define SYM_FT          0x0F

// Voltage and amperage
#define SYM_VOLT  0x06
#define SYM_AMP   0x9A
#define SYM_MAH   0x07
#define SYM_WATT  0x57

// Flying Mode
#define SYM_ACRO      0xAE
#define SYM_ACROGY    0x98
#define SYM_ACRO1     0xAF
#define SYM_STABLE    0xAC
#define SYM_STABLE1   0xAD
#define SYM_HORIZON   0xC4
#define SYM_HORIZON1  0xC5
#define SYM_PASS      0xAA
#define SYM_PASS1     0xAB
#define SYM_AIR       0xEA
#define SYM_AIR1      0xEB
#define SYM_PLUS      0x89

// Note, these change with scrolling enabled (scrolling is TODO)
//#define SYM_AH_DECORATION_LEFT 0x13
//#define SYM_AH_DECORATION_RIGHT 0x13
#define SYM_AH_DECORATION 0x13

// Time
#define SYM_ON_M  0x9B
#define SYM_FLY_M 0x9C
#define SYM_ON_H  0x70
#define SYM_FLY_H 0x71

// Throttle Position (%)
#define SYM_THR   0x04
#define SYM_THR1  0x05

// RSSI
#define SYM_RSSI 0x01

// Menu cursor
#define SYM_CURSOR SYM_AH_LEFT

//Misc
#define SYM_COLON 0x2D

//sport
#define SYM_MIN 0xB3
#define SYM_AVG 0xB4

#endif // USE_MAX7456
