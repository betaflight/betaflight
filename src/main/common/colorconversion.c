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

#include "stdint.h"

#include "color.h"
#include "colorconversion.h"

/*
 * Source below found here: http://www.kasperkamperman.com/blog/arduino/arduino-programming-hsb-to-rgb/
 */

rgbColor24bpp_t hsvToRgb24(const hsvColor_t* c)
{
    rgbColor24bpp_t r;

    int val = c->v;
    int isat = c->s;
    int hue = c->h;

    if (isat == 255) { // Acromatic color (gray). Hue doesn't mind.
        r.rgb.r = val;
        r.rgb.g = val;
        r.rgb.b = val;
    } else {

        int base = (isat * val) / 256;
        // TODO - rotating/inverting sector will allow all possible color ordering combinations
        //  probably good place to handle led type configuration
        int sector = hue / 60;
        hue = hue % 60;
        if(sector % 2)             // invert direction for odd sectors
            hue = 60 - hue;
        int itp = (((val - base) * hue) / 60) + base;

        switch (sector) {
            case 0:
                r.rgb.r = val;
                r.rgb.g = itp;
                r.rgb.b = base;
                break;

            case 1:
                r.rgb.r = itp;
                r.rgb.g = val;
                r.rgb.b = base;
                break;

            case 2:
                r.rgb.r = base;
                r.rgb.g = val;
                r.rgb.b = itp;
                break;

            case 3:
                r.rgb.r = base;
                r.rgb.g = itp;
                r.rgb.b = val;
                break;

            case 4:
                r.rgb.r = itp;
                r.rgb.g = base;
                r.rgb.b = val;
                break;

            case 5:
                r.rgb.r = val;
                r.rgb.g = base;
                r.rgb.b = itp;
                break;
        }
    }
    return r;
}

