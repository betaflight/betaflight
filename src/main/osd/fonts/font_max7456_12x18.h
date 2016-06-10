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

// generated with MAX7456 Glyph Editor

#define FONT_MAX7456_12x18_BYTES_PER_CHARACTER 64 // FIXME actually only the first 54 bytes are used.  data is currently stored this way because of the way the font generators generate there c source.
extern const uint8_t font_max7456_12x18[16384];

const uint8_t font_max7456_12x18_asciiToFontMapping[128];

#define FONT_CHARACTER_CF_LOGO_W3xH2__1x1 0x01
#define FONT_CHARACTER_CF_LOGO_W3xH2__1x2 0x02
#define FONT_CHARACTER_CF_LOGO_W3xH2__1x3 0x03
#define FONT_CHARACTER_CF_LOGO_W3xH2__2x1 0x11
#define FONT_CHARACTER_CF_LOGO_W3xH2__2x2 0x12
#define FONT_CHARACTER_CF_LOGO_W3xH2__2x3 0x13

//
// small circles, in 1/8th increments.
//
#define FONT_CHARACTER_MOTOR_8_8 0x04 // complete circle
#define FONT_CHARACTER_MOTOR_7_8 0x05
#define FONT_CHARACTER_MOTOR_6_8 0x06
#define FONT_CHARACTER_MOTOR_5_8 0x07
#define FONT_CHARACTER_MOTOR_4_8 0x08 // semi-circle
#define FONT_CHARACTER_MOTOR_3_8 0x09
#define FONT_CHARACTER_MOTOR_2_8 0x0a
#define FONT_CHARACTER_MOTOR_1_8 0x0b
#define FONT_CHARACTER_MOTOR_0_8 0x0c // line
#define FONT_CHARACTER_MOTOR_OFF 0x0d // circle outline


#define FONT_VERSION 1
