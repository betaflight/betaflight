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

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "common/printf.h"
#include "platform.h"
#include "version.h"

#ifdef USE_MAX7456

#include "drivers/bus_spi.h"
#include "drivers/light_led.h"
#include "drivers/system.h"

#include "max7456.h"
#include "max7456_symbols.h"

#define DISABLE_MAX7456       IOHi(max7456CsPin)
#define ENABLE_MAX7456        IOLo(max7456CsPin)

/** Artificial Horizon limits **/
#define AHIPITCHMAX 200             // Specify maximum AHI pitch value displayed. Default 200 = 20.0 degrees
#define AHIROLLMAX  400             // Specify maximum AHI roll value displayed. Default 400 = 40.0 degrees
#define AHISIDEBARWIDTHPOSITION 7
#define AHISIDEBARHEIGHTPOSITION 3

uint16_t max_screen_size;
char max7456_screen[VIDEO_BUFFER_CHARS_PAL];

static uint8_t  video_signal_type   = 0;
static uint8_t  max7456_lock        = 0;
static IO_t max7456CsPin            = IO_NONE;

uint8_t max7456_send(uint8_t add, uint8_t data) {
    spiTransferByte(MAX7456_SPI_INSTANCE, add);
    return spiTransferByte(MAX7456_SPI_INSTANCE, data);
}


void max7456_init(uint8_t video_system) 
{
    uint8_t max_screen_rows;
    uint8_t srdata = 0;
    uint16_t x;

#ifdef MAX7456_SPI_CS_PIN
    max7456CsPin = IOGetByTag(IO_TAG(MAX7456_SPI_CS_PIN));
#endif
    IOInit(max7456CsPin, OWNER_SYSTEM, RESOURCE_SPI);
    IOConfigGPIO(max7456CsPin, SPI_IO_CS_CFG);

    //Minimum spi clock period for max7456 is 100ns (10Mhz)
    spiSetDivisor(MAX7456_SPI_INSTANCE, SPI_CLOCK_STANDARD);

    delay(1000);
    // force soft reset on Max7456
    ENABLE_MAX7456;
    max7456_send(VM0_REG, MAX7456_RESET);
    delay(100);

    srdata = max7456_send(0xA0, 0xFF);
    if ((0x01 & srdata) == 0x01) {     //PAL
          video_signal_type = VIDEO_MODE_PAL;
    }
    else if ((0x02 & srdata) == 0x02) { //NTSC
        video_signal_type = VIDEO_MODE_NTSC;
    }

    // Override detected type: 0-AUTO, 1-PAL, 2-NTSC
    switch(video_system) {
        case PAL:
            video_signal_type = VIDEO_MODE_PAL;
            break;
        case NTSC:
            video_signal_type = VIDEO_MODE_NTSC;
            break;
    }

    if (video_signal_type) { //PAL
        max_screen_size = VIDEO_BUFFER_CHARS_PAL;
        max_screen_rows = VIDEO_LINES_PAL;
    } else {                 // NTSC
        max_screen_size = VIDEO_BUFFER_CHARS_NTSC;
        max_screen_rows = VIDEO_LINES_NTSC;
    }

    // set all rows to same charactor black/white level
    for(x = 0; x < max_screen_rows; x++) {
        max7456_send(MAX7456ADD_RB0 + x, BWBRIGHTNESS);
    }

    // make sure the Max7456 is enabled
    max7456_send(VM0_REG, OSD_ENABLE | video_signal_type);

    DISABLE_MAX7456;
    delay(100);
}

// Copy string from ram into screen buffer
void max7456_write_string(const char *string, int16_t address) {
    char *dest;

    if (address >= 0)
        dest  = max7456_screen + address;
    else
        dest  = max7456_screen + (max_screen_size + address);

    while(*string && dest < (max7456_screen + max_screen_size))
        *dest++ = *string++;
}


// Write the artifical horizon to the screen buffer
void max7456_artificial_horizon(int rollAngle, int pitchAngle, uint8_t show_sidebars) {
    uint16_t position = 194;

    if(pitchAngle>AHIPITCHMAX) pitchAngle=AHIPITCHMAX;
    if(pitchAngle<-AHIPITCHMAX) pitchAngle=-AHIPITCHMAX;
    if(rollAngle>AHIROLLMAX) rollAngle=AHIROLLMAX;
    if(rollAngle<-AHIROLLMAX) rollAngle=-AHIROLLMAX;

    for(uint8_t X=0; X<=8; X++) {
      if (X==4) X=5;
      int Y = (rollAngle * (4-X)) / 64;
      Y -= pitchAngle / 8;
      Y += 41;
      if(Y >= 0 && Y <= 81) {
        uint16_t pos = position -7 + LINE*(Y/9) + 3 - 4*LINE + X;
        max7456_screen[pos] = SYM_AH_BAR9_0+(Y%9);
      }
    }
    max7456_screen[position-1] = SYM_AH_CENTER_LINE;
    max7456_screen[position+1] = SYM_AH_CENTER_LINE_RIGHT;
    max7456_screen[position] =   SYM_AH_CENTER;

    if (show_sidebars) {
      // Draw AH sides
      int8_t hudwidth =  AHISIDEBARWIDTHPOSITION;
      int8_t hudheight = AHISIDEBARHEIGHTPOSITION;
      for(int8_t X=-hudheight; X<=hudheight; X++) {
        max7456_screen[position-hudwidth+(X*LINE)] = SYM_AH_DECORATION;
        max7456_screen[position+hudwidth+(X*LINE)] = SYM_AH_DECORATION;
      }
      // AH level indicators
      max7456_screen[position-hudwidth+1] =  SYM_AH_LEFT;
      max7456_screen[position+hudwidth-1] =  SYM_AH_RIGHT;
    }
}

void max7456_draw_screen(void) {
    uint16_t xx;
    if (!max7456_lock) {
        ENABLE_MAX7456;
        for (xx = 0; xx < max_screen_size; ++xx) {
            max7456_send(MAX7456ADD_DMAH, xx>>8);
            max7456_send(MAX7456ADD_DMAL, xx);
            max7456_send(MAX7456ADD_DMDI, max7456_screen[xx]);
            max7456_screen[xx] = ' ';
        }
        DISABLE_MAX7456;
    }
}

void max7456_draw_screen_fast(void) {
    uint16_t xx;
    if (!max7456_lock) {
        ENABLE_MAX7456;
        max7456_send(MAX7456ADD_DMAH, 0);
        max7456_send(MAX7456ADD_DMAL, 0);
        max7456_send(MAX7456ADD_DMM, 1);
        for (xx = 0; xx < max_screen_size; ++xx) {
           max7456_send(MAX7456ADD_DMDI, max7456_screen[xx]);
            max7456_screen[xx] = ' ';
        }
        max7456_send(MAX7456ADD_DMDI, 0xFF);
        max7456_send(MAX7456ADD_DMM, 0);
        DISABLE_MAX7456;
    }
}


void max7456_write_nvm(uint8_t char_address, uint8_t *font_data) {
    uint8_t x;

    max7456_lock = 1;
    ENABLE_MAX7456;

    // disable display
    max7456_send(VM0_REG, video_signal_type);

    max7456_send(MAX7456ADD_CMAH, char_address); // set start address high

    for(x = 0; x < 54; x++) {
        max7456_send(MAX7456ADD_CMAL, x); //set start address low
        max7456_send(MAX7456ADD_CMDI, font_data[x]);
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        LED1_TOGGLE;
#endif
    }

    // transfer 54 bytes from shadow ram to NVM
    max7456_send(MAX7456ADD_CMM, WRITE_NVR);

    // wait until bit 5 in the status register returns to 0 (12ms)
    while ((spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456ADD_STAT) & STATUS_REG_NVR_BUSY) != 0);

    max7456_send(VM0_REG, video_signal_type | 0x0C);
    DISABLE_MAX7456;
    max7456_lock = 0;
}


#endif
