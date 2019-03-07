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

//                                  0x00 // Blank
#define SYM_RSSI                    0x01 // 001 RSSI
#define SYM_AH_RIGHT                0x02 // 002 Arrow left
#define SYM_AH_LEFT                 0x03 // 003 Arrow right
#define SYM_THR                     0x04 // 004 Throttle
#define SYM_THR1                    0x05 // 005 Arrow up AHI
#define SYM_VOLT                    0x06 // 006 V
#define SYM_MAH                     0x07 // 007 MAH

#define SYM_STICK_OVERLAY_SPRITE_HIGH 0x08 // 008 Stick overlay
#define SYM_STICK_OVERLAY_SPRITE_MID  0x09 // 009 Stick overlay
#define SYM_STICK_OVERLAY_SPRITE_LOW  0x0A // 010 Stick overlay
#define SYM_STICK_OVERLAY_CENTER      0x0B // 011 Stick overlay

#define SYM_M                       0x0C // 012 m
#define SYM_F                       0x0D // 013 °F
#define SYM_C                       0x0E // 014 °C
#define SYM_FT                      0x0F // 015 ft

//                                  0x10 // 016 Scrolling bar
//                                  0x11 // 017 Scrolling bar
//                                  0x12 // 018 Scrolling bar
#define SYM_AH_DECORATION           0x13 // 013 -
//                                  0x14 // 020 Scrolling bar
//                                  0x15 // 021 Scrolling bar

#define SYM_STICK_OVERLAY_VERTICAL    0x16 // 022 Stick overlay 
#define SYM_STICK_OVERLAY_HORIZONTAL  0x17 // 023 Stick overlay

#define SYM_HEADING_N               0x18 // 024 Compass
#define SYM_HEADING_S               0x19 // 025 Compass
#define SYM_HEADING_E               0x1A // 026 Compass
#define SYM_HEADING_W               0x1B // 027 Compass
#define SYM_HEADING_DIVIDED_LINE    0x1C // 028 Compass
#define SYM_HEADING_LINE            0x1D // 029 Compass

#define SYM_SAT_L                   0x1E // 030 Sats left
#define SYM_SAT_R                   0x1F // 031 Sats right

#define SYM_BLANK                   0x20 // 032 Blank (space)

//                                  0x21 // 033 -----------
//                                  0x22 // 034 -----------
//                                  0x23 // 035 -----------
//                                  0x24 // 036 -----------

//                                  0x25 // 037 Ascii %

#define SYM_AH_CENTER_LINE          0x26 // 038 Crossair left
#define SYM_AH_CENTER_LINE_RIGHT    0x27 // 039 Crossair right

//                                  0x28 // 040 to 062 ASCII

#define SYM_COLON                   0x2D

//                                  0x3F // 063 -----------

//                                  0x40 // 064 to 095 ASCII

#define SYM_ARROW_SOUTH             0x60 // 096 Directional arrow
#define SYM_ARROW_2                 0x61 // 097 Directional arrow
#define SYM_ARROW_3                 0x62 // 098 Directional arrow
#define SYM_ARROW_4                 0x63 // 099 Directional arrow
#define SYM_ARROW_EAST              0x64 // 100 Directional arrow
#define SYM_ARROW_6                 0x65 // 101 Directional arrow
#define SYM_ARROW_7                 0x66 // 102 Directional arrow
#define SYM_ARROW_8                 0x67 // 103 Directional arrow
#define SYM_ARROW_NORTH             0x68 // 104 Directional arrow
#define SYM_ARROW_10                0x69 // 105 Directional arrow
#define SYM_ARROW_11                0x6A // 106 Directional arrow
#define SYM_ARROW_12                0x6B // 107 Directional arrow
#define SYM_ARROW_WEST              0x6C // 108 Directional arrow
#define SYM_ARROW_14                0x6D // 109 Directional arrow
#define SYM_ARROW_15                0x6E // 110 Directional arrow
#define SYM_ARROW_16                0x6F // 111 Directional arrow

//                                  0x70 // 112 -----------
//                                  0x71 // 113 -----------

#define SYM_DIRECTION               0x72 // 114 to 121, directional little arrows

//                                  0x7A // 122 -----------

//                                  0x7B // 123 to 125 ASCII

#define SYM_AH_CENTER               0x7E // 126 Crossair center

//                                  0x7F // 127 -----------

#define SYM_AH_BAR9_0               0x80 // 128 AHI
#define SYM_AH_BAR9_1               0x81 // 129 AHI
#define SYM_AH_BAR9_2               0x82 // 130 AHI
#define SYM_AH_BAR9_3               0x83 // 131 AHI
#define SYM_AH_BAR9_4               0x84 // 132 AHI
#define SYM_AH_BAR9_5               0x85 // 133 AHI
#define SYM_AH_BAR9_6               0x86 // 134 AHI
#define SYM_AH_BAR9_7               0x87 // 135 AHI
#define SYM_AH_BAR9_8               0x88 // 136 AHI

#define SYM_HOME                    0x89 // 137 Home flag NEW IN 4.0

#define SYM_PB_START                0x8A // 138 Progress bar
#define SYM_PB_FULL                 0x8B // 139 Progress bar
#define SYM_PB_HALF                 0x8C // 140 Progress bar
#define SYM_PB_EMPTY                0x8D // 141 Progress bar
#define SYM_PB_END                  0x8E // 142 Progress bar
#define SYM_PB_CLOSE                0x8F // 143 Progress bar

#define SYM_BATT_FULL               0x90 // 144 Battery full
#define SYM_BATT_5                  0x91 // 145 Battery
#define SYM_BATT_4                  0x92 // 146 Battery
#define SYM_BATT_3                  0x93 // 147 Battery
#define SYM_BATT_2                  0x94 // 148 Battery
#define SYM_BATT_1                  0x95 // 149 Battery
#define SYM_BATT_EMPTY              0x96 // 150 Battery empty

#define SYM_MAIN_BATT               0x97 // 151 Battery 

//                                  0x98 // 152 -----------

#define SYM_FTS                     0x99 // 153 FT/S NEW IN 4.0
#define SYM_AMP                     0x9A // 154 A
#define SYM_ON_M                    0x9B // 155 On MN
#define SYM_FLY_M                   0x9C // 156 FL MN

#define SYM_KMH                     0x9D // 157 KM/H -- NEW IN 4.0
#define SYM_MPH                     0x9E // 158 MP/H -- NEW IN 4.0
#define SYM_MS                      0x9F // 159 M/S

#define SYM_LOGO_START	            0xA0 // 160 to 256 BF logo
#define SYM_LOGO_END                0xFF // 255 End of the logo
