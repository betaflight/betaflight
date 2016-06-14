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
#include "drivers/system.h"

#include "max7456.h"

#define DISABLE_MAX7456       GPIO_SetBits(MAX7456_CS_GPIO,   MAX7456_CS_PIN)
#define ENABLE_MAX7456        GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN)


uint16_t max_screen_size;
uint8_t  video_signal_type = 0;
uint8_t  max7456_lock = 0;
char screen[480];


uint8_t max7456_send(uint8_t add, uint8_t data) {
    spiTransferByte(MAX7456_SPI_INSTANCE, add);
    return spiTransferByte(MAX7456_SPI_INSTANCE, data);
}


void max7456_init(uint8_t system) {
    uint8_t max_screen_rows;
    uint8_t srdata = 0;
    uint16_t x;
    char buf[30];

    //Minimum spi clock period for max7456 is 100ns (10Mhz)
    spiSetDivisor(MAX7456_SPI_INSTANCE, SPI_9MHZ_CLOCK_DIVIDER);

    delay(1000);
    // force soft reset on Max7456
    ENABLE_MAX7456;
    max7456_send(VM0_REG, MAX7456_RESET);
    delay(100);

    srdata = max7456_send(0xA0, 0xFF);
    if ((0x01 & srdata) == 0x01){     //PAL
          video_signal_type = VIDEO_MODE_PAL;
    }
    else if((0x02 & srdata) == 0x02){ //NTSC
        video_signal_type = VIDEO_MODE_NTSC;
    }

    // Override detected type: 0-AUTO, 1-PAL, 2-NTSC
    switch(system) {
        case 1:
            video_signal_type = VIDEO_MODE_PAL;
            break;
        case 2:
            video_signal_type = VIDEO_MODE_NTSC;
            break;
    }

    if (video_signal_type) { //PAL
        max_screen_size = 480;
        max_screen_rows = 16;
    } else {                 // NTSC
        max_screen_size = 390;
        max_screen_rows = 13;
    }

    // set all rows to same charactor black/white level
    for(x = 0; x < max_screen_rows; x++) {
        max7456_send(MAX7456ADD_RB0 + x, BWBRIGHTNESS);
    }

    // make sure the Max7456 is enabled
    max7456_send(VM0_REG, OSD_ENABLE | video_signal_type);

    DISABLE_MAX7456;
    delay(100);

    x =  160;
    for (int i = 1; i < 5; i++) {
        for (int j = 3; j < 27; j++)
            screen[i * 30 + j] = (char)x++;
    }
    tfp_sprintf(buf, "BF VERSION: %s", FC_VERSION_STRING);
    max7456_write_string(buf, 5*30+5);
    max7456_write_string("MENU: THRT MID", 6*30+7);
    max7456_write_string("YAW RIGHT", 7*30+13);
    max7456_write_string("PITCH UP", 8*30+13);
    max7456_draw_screen();
}

// Copy string from ram into screen buffer
void max7456_write_string(const char *string, int16_t address) {
    char *dest;

    if (address >= 0)
        dest  = screen + address;
    else
        dest  = screen + (max_screen_size + address);

    while(*string)
        *dest++ = *string++;
}

void max7456_draw_screen(void) {
    uint16_t xx;
    if (!max7456_lock) {
        ENABLE_MAX7456;
        for (xx = 0; xx < max_screen_size; ++xx) {
            max7456_send(MAX7456ADD_DMAH, xx>>8);
            max7456_send(MAX7456ADD_DMAL, xx);
            max7456_send(MAX7456ADD_DMDI, screen[xx]);
            screen[xx] = ' ';
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
           max7456_send(MAX7456ADD_DMDI, screen[xx]);
            screen[xx] = ' ';
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
