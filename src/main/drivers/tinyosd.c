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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifndef USE_MAX7456  // MAX7456 and tinyOSD video buffers will not fit into memory

#include "common/printf.h"
#include "common/maths.h"
#include "io/osd.h"
#include "io/displayport_tinyosd.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "drivers/bus_spi.h"
#include "drivers/light_led.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/dma.h"
#include "drivers/serial.h"
#include "drivers/vcd.h"
#include "tinyosd.h"

#include "io/serial.h"

#define TINYOSD_STICKSIZE_X  96.0f
#define TINYOSD_STICKSIZE_Y 128.0f

#define VIDEO_SIGNAL_DEBOUNCE_MS    100 // Time to wait for input to stabilize

// STAT register bits

#define VIDEO_MODE_PAL              0x40
#define VIDEO_MODE_NTSC             0x00
#define VIDEO_MODE_MASK             0x40
#define VIDEO_MODE_IS_PAL(val)      (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_PAL)
#define VIDEO_MODE_IS_NTSC(val)     (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_NTSC)

#define STAT_PAL      0x01
#define STAT_NTSC     0x02
#define STAT_LOS      0x04
#define STAT_NVR_BUSY 0x20

#define STAT_IS_PAL(val)  ((val) & STAT_PAL)
#define STAT_IS_NTSC(val) ((val) & STAT_NTSC)
#define STAT_IS_LOS(val)  ((val) & STAT_LOS)

#define VIN_IS_PAL(val)  (!STAT_IS_LOS(val) && STAT_IS_PAL(val))
#define VIN_IS_NTSC(val)  (!STAT_IS_LOS(val) && STAT_IS_NTSC(val))

#define CHARS_PER_LINE      35


uint16_t tinyOSD_maxScreenSize = TINYOSD_VIDEO_BUFFER_CHARS_PAL;
static const uint8_t tinyosd_crc8_table[256];


static uint8_t screenBuffer[TINYOSD_VIDEO_BUFFER_SIZE];
static uint8_t screenBufferDirty[TINYOSD_VIDEO_BUFFER_DIRTY_SIZE];




//Max chars to update in one idle
//#define MAX_CHARS2UPDATE    100
#define TINYOSD_PROTOCOL_FRAME_BUFFER_SIZE 32

//static uint8_t  videoSignalCfg;
static uint8_t  video_system;

static uint8_t  hosRegValue; // HOS (Horizontal offset register) value
static uint8_t  vosRegValue; // VOS (Vertical offset register) value

static bool  tinyOSDLock        = false;
static bool fontIsLoading       = false;

static serialPort_t *tinyOSDPort;
#define TINYOSD_PROTOCOL_HEADER 0x80


int tinyOSDGrab(displayPort_t * displayPort)
{
    UNUSED(displayPort);
    osdResetAlarms();
    resumeRefreshAt = 0;
    return 0;
}

int tinyOSDRelease(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static uint8_t tinyOSDSendBuffer(uint8_t cmd, uint8_t *data, uint8_t len)
{
    if (!tinyOSDPort) {
        // no port opened, return zero
        return 0;
    }

    // check for space in buffer
    uint16_t buf_free = serialTxBytesFree(tinyOSDPort);
    if (buf_free < (3 + len + 1)) {
        // no space to send
        return 0;
    }


    // very simple protocol:
    // [HEADER] LEN ADDR <n DATA> [CSUM]
    uint8_t header[3];
    header[0] = TINYOSD_PROTOCOL_HEADER;
    header[1] = len;
    header[2] = cmd;

    // send header
    serialWriteBuf(tinyOSDPort,header, 3);

    // send data
    serialWriteBuf(tinyOSDPort, data, len);

    // calculate checksum:
    uint8_t crc = 0;
    TINYOSD_CRC8_INIT(crc, 0);
    TINYOSD_CRC8_UPDATE(crc, header[0]);
    TINYOSD_CRC8_UPDATE(crc, header[1]);
    TINYOSD_CRC8_UPDATE(crc, header[2]);

    for (uint8_t idx=0; idx < len; idx++){
        TINYOSD_CRC8_UPDATE(crc, data[idx]);
    }

    // send checksum
    serialWrite(tinyOSDPort, crc);

    // expect a single byte reply:
    return 1; //serialRead(tinyOSDPort);
}

void tinyOSDReInit(void)
{
    //uint8_t maxScreenRows;
    //uint8_t srdata = 0;
    //uint16_t x;
    static bool firstInit = true;
/*
    ENABLE_MAX7456;

    switch(videoSignalCfg) {
        case VIDEO_SYSTEM_PAL:
            videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
            break;

        case VIDEO_SYSTEM_NTSC:
            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
            break;

        case VIDEO_SYSTEM_AUTO:
            srdata = max7456Send(MAX7456ADD_STAT, 0x00);

            if (VIN_IS_NTSC(srdata)) {
                videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
            } else if (VIN_IS_PAL(srdata)) {
                videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
            } else {
                // No valid input signal, fallback to default (XXX NTSC for now)
                videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
            }
            break;
    }
*/

    if (video_system == VIDEO_SYSTEM_AUTO) {
        // fetch video mode from tinyOSD FIXME
        video_system = VIDEO_SYSTEM_NTSC;
    }

    if (video_system == VIDEO_SYSTEM_PAL) {
        tinyOSD_maxScreenSize = TINYOSD_VIDEO_BUFFER_CHARS_PAL;
    } else {              // NTSC
        tinyOSD_maxScreenSize = TINYOSD_VIDEO_BUFFER_CHARS_NTSC;
    }

    // enable osd
    tinyOSDSendBuffer(TINYOSD_COMMAND_SET_STATUS, (uint8_t *)"\x01", 1);

    // set dirty flag to force redraw all screen in non-dma mode.
    memset(screenBufferDirty, 0xFF, sizeof(screenBufferDirty));

    if (firstInit)
    {
        tinyOSDRefreshAll();
        firstInit = false;
    }
}


bool tinyOSDInit(const vcdProfile_t *pVcdProfile)
{
    // Setup values to write to registers
    video_system = pVcdProfile->video_system;
    hosRegValue = 32 - pVcdProfile->h_offset;
    vosRegValue = 16 - pVcdProfile->v_offset;

    // find tinyosd serial port
    serialPortConfig_t *osdPortConfig = findSerialPortConfig(FUNCTION_TINYOSD);

    if (!osdPortConfig) {
        // could not get port -> abort
        return false;
    }

    // we will do fw upgrades in the feature so reserve rx and tx
    portMode_t mode = MODE_RXTX;

    baudRate_e baudRateIndex =  osdPortConfig->blackbox_baudrateIndex;
    uint32_t baudrate = baudRates[baudRateIndex];

    // open assigned serial port (no callback for RX right now)
    tinyOSDPort = openSerialPort(osdPortConfig->identifier, FUNCTION_NONE, NULL, baudrate, mode, SERIAL_NOT_INVERTED);

    // verify opening
    if (!tinyOSDPort) {
        return false;
    }

    // init with empty chars (blankspace)
    memset(screenBuffer, ' ', sizeof(screenBuffer));

    // fill whole screen on device with ' ' as well by using the FILL command
    tinyOSDSendBuffer(TINYOSD_COMMAND_FILL_SCREEN, (uint8_t *)"x", 1);

    // mark screen buffer as non dirty:
    memset(screenBufferDirty, 0x00, sizeof(screenBufferDirty));

    return true;
}

//just fill with spaces
int tinyOSDClearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    // clear screen buffer & mark non ' ' bytes as dirty:
    for(uint16_t i=0; i<TINYOSD_VIDEO_BUFFER_SIZE; i++) {
        if (SCREEN_BUFFER_GET(i) != ' ') {
            // clear pixel & set to changed:
            SCREEN_BUFFER_SET(i, ' ');
        }
    }

    return 0;
}

uint8_t* tinyOSDGetScreenBuffer(void) {
    return screenBuffer;
}

int tinyOSDWriteChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t c)
{
    UNUSED(displayPort);
    SCREEN_BUFFER_SET(y * CHARS_PER_LINE + x, c);
    return 0;
}

int tinyOSDWriteString(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *buff)
{
    UNUSED(displayPort);
    uint8_t *data_ptr = (uint8_t *)buff;
    uint8_t  i = 0;

    // start position of write
    uint16_t fb_pos = y * CHARS_PER_LINE + x;

    for (i = x; i < CHARS_PER_LINE; i++) {
        if (*data_ptr == 0) {
            // end of char -> finish
            break;
        }

        // copy to frame buffer
        SCREEN_BUFFER_SET(fb_pos, *data_ptr);

        // prepare for next char:
        fb_pos++;
        data_ptr++;
    }
    return 0;
}

#include "build/debug.h"

static void tinyOSDDrawSticks(void) {
    uint8_t data[5];
    data[0] = TINYOSD_STICKSIZE_X/2 + (TINYOSD_STICKSIZE_X/2 * rcCommand[ROLL]) / 500.0f;
    data[1] = TINYOSD_STICKSIZE_Y/2 - (TINYOSD_STICKSIZE_Y/2 * rcCommand[PITCH]) / 500.0f;
    // throttle is 1000 - 2000, rescale to match out STICK_Y resolution
    data[2] = TINYOSD_STICKSIZE_Y - (TINYOSD_STICKSIZE_Y * (rcCommand[THROTTLE]-1000.0f))/1000.0f; //(TINYOSD_STICKSIZE_Y/2 * (rcCommand[THROTTLE]-1.0)) / 1000.0;
    data[3] = TINYOSD_STICKSIZE_X/2 - (TINYOSD_STICKSIZE_X/2 * rcCommand[YAW]) / 500.0f;

    // armed?
    data[4] = armingFlags;

    tinyOSDSendBuffer(TINYOSD_COMMAND_WRITE_STICKDATA, &data[0], 5);
}

int tinyOSDDrawScreen(displayPort_t *displayPortProfile)
{
    UNUSED(displayPortProfile);
    //uint8_t stallCheck;
    //uint8_t videoSense;
    //static uint32_t lastSigCheckMs = 0;
    //uint32_t nowMs;
    //static uint32_t videoDetectTimeMs = 0;
    static uint16_t pos = 0;
    int k = 0;

    if (!tinyOSDLock && !fontIsLoading) {
        tinyOSDLock = true;
        //nowMs = millis();
        //------------   end of (re)init-------------------------------------

        // send stick and armed state data
        tinyOSDDrawSticks();

        // transfer as many new chars as possible:
        uint16_t allowed_charcount = TINYOSD_PROTOCOL_FRAME_BUFFER_SIZE - 1; // minus length
        uint8_t buffer[TINYOSD_PROTOCOL_FRAME_BUFFER_SIZE];

        while (allowed_charcount) {
            // we are allowed to send more updated chars
            // search for next updated char
            while ((!SCREEN_BUFFER_GET_DIRTY_FLAG(pos)) && (allowed_charcount)) {
                pos++;
                if (pos >= tinyOSD_maxScreenSize) {
                   // end of screen, restart from zero
                   pos = 0;
                   if (!SCREEN_BUFFER_GET_DIRTY_FLAG(pos)){
                       // no updated char, exit this iteration alltogether
                       // this makes sure that we do not iterate over unchanged frames
                       allowed_charcount = 0;
                   }
                }
            }

            // start new data set
            k = 0;

            // we ended here because we found an updated char or
            // there is no more data tx allowed
            if (allowed_charcount) {
                // we know that this byte differs (loop above exited)
                // add this and all following that differ to buf
                // but first of all add start address to packet
                uint8_t page = (pos>>8) & 0x03;  // write commands 0x00, 0x01, 0x02, 0x03 write to pages
                buffer[k++] = pos & 0xFF;

                while ((k < TINYOSD_PROTOCOL_FRAME_BUFFER_SIZE) && (allowed_charcount)){
                    if (!SCREEN_BUFFER_GET_DIRTY_FLAG(pos) && !SCREEN_BUFFER_GET_DIRTY_FLAG(pos+1)) {
                        // no changed byte within the next 2 chars -> finish & send buffer
                        break;
                    } else {
                        // different byte in buffer OR this is only 1 single equal byte
                        // -> add byte in both cases (it is more eficient to tx gaps as well)
                        buffer[k++] = SCREEN_BUFFER_GET(pos);
                        allowed_charcount--;

                        // mark as sent
                        SCREEN_BUFFER_CLEAR_DIRTY_FLAG(pos);

                        // increment
                        pos++;

                        // take care of counters
                        if (pos >= tinyOSD_maxScreenSize) {
                            pos = 0;
                            break;
                        }
                    }
                }

                // we added k bytes to the buffer, send it!
                if (k) {
                    if (!tinyOSDSendBuffer(TINYOSD_COMMAND_WRITE_PAGE_0 + page, buffer, k)){
                        // there should be no send failures as we checked the free beuf in advance
                    }
                }
            }
        }

        tinyOSDLock = false;
    }
    return 0;
}

bool tinyOSDIsTransferInProgress(const displayPort_t *displayPort){
    UNUSED(displayPort);
    return false;
}

int tinyOSDHeartbeat(displayPort_t *displayPort){
    UNUSED(displayPort);
    return 0;
}

// This funcktion refresh all and should not be used when copter is armed
void tinyOSDRefreshAll(void)
{
    if (!tinyOSDLock) {
        uint16_t xx = 0;
        uint32_t timeout;

        tinyOSDLock = true;
        bool timed_out = false;

        // abort after 500ms
        timeout = millis() + 500;

        while((!timed_out) && (xx < tinyOSD_maxScreenSize)) {
            uint16_t buf_free = 0;
            // wait until we have enough space in buffer for a full
            // data frame (header<3> + data<len> + csum<1>)
            while (buf_free < 3 + TINYOSD_PROTOCOL_FRAME_BUFFER_SIZE + 1) {
                // fetch free byte count
                buf_free = serialTxBytesFree(tinyOSDPort);

                // timeout ?
                if (millis() > timeout) {
                    timed_out = true;
                    break;
                }
            }
            if (timed_out) {
                break;
            }

            // build packet
            uint8_t page = xx>>8;
            uint8_t data[TINYOSD_PROTOCOL_FRAME_BUFFER_SIZE];
            uint8_t data_idx = 0;

            // address offset
            data[data_idx++] = xx & 0xFF;

            while ((xx < tinyOSD_maxScreenSize) && (data_idx<TINYOSD_PROTOCOL_FRAME_BUFFER_SIZE)) {
               data[data_idx++] = SCREEN_BUFFER_GET(xx);
               SCREEN_BUFFER_CLEAR_DIRTY_FLAG(xx);
               xx++;
            }

            while((!timed_out) && (!tinyOSDSendBuffer(TINYOSD_COMMAND_WRITE_PAGE_0 + page, data, data_idx - 1))) {
                // retry until sucessfull or timeout occured
                if (millis() > timeout) {
                    timed_out = true;
                }
            }
        }

        tinyOSDLock = false;
    }
}

void tinyOSDResync(displayPort_t *displayPort)
{
    tinyOSDRefreshAll();

    if (video_system == VIDEO_SYSTEM_PAL) {
        displayPort->rowCount = TINYOSD_VIDEO_LINES_PAL;
    } else {
        displayPort->rowCount = TINYOSD_VIDEO_LINES_NTSC;
    }

    displayPort->colCount = TINYOSD_VIDEO_COLS;
}

uint32_t tinyOSDTxBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

void tinyOSDWriteNvm(uint8_t char_address, const uint8_t *font_data)
{
    UNUSED(char_address);
    UNUSED(font_data);
/*    uint8_t x;

#ifdef MAX7456_DMA_CHANNEL_TX
    while (dmaTransactionInProgress);
#endif
    while (max7456Lock);
    max7456Lock = true;

    ENABLE_MAX7456;
    // disable display
    fontIsLoading = true;
    max7456Send(MAX7456ADD_VM0, 0);

    max7456Send(MAX7456ADD_CMAH, char_address); // set start address high

    for(x = 0; x < 54; x++) {
        max7456Send(MAX7456ADD_CMAL, x); //set start address low
        max7456Send(MAX7456ADD_CMDI, font_data[x]);
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        LED1_TOGGLE;
#endif
    }

    // Transfer 54 bytes from shadow ram to NVM

    max7456Send(MAX7456ADD_CMM, WRITE_NVR);

    // Wait until bit 5 in the status register returns to 0 (12ms)

    while ((max7456Send(MAX7456ADD_STAT, 0x00) & STAT_NVR_BUSY) != 0x00);

    DISABLE_MAX7456;

    max7456Lock = false;
*/
}


// dvb s2 crc8 (poly 0xD5) lookup table
static const uint8_t tinyosd_crc8_table[256] = {
    0x0, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54,
    0x29, 0xfc, 0x56, 0x83, 0xd7, 0x2, 0xa8, 0x7d,
    0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x6,
    0x7b, 0xae, 0x4, 0xd1, 0x85, 0x50, 0xfa, 0x2f,
    0xa4, 0x71, 0xdb, 0xe, 0x5a, 0x8f, 0x25, 0xf0,
    0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0xc, 0xd9,
    0xf6, 0x23, 0x89, 0x5c, 0x8, 0xdd, 0x77, 0xa2,
    0xdf, 0xa, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b,
    0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9,
    0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0,
    0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b,
    0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2,
    0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d,
    0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44,
    0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f,
    0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16,
    0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb,
    0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92,
    0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9,
    0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0,
    0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f,
    0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36,
    0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d,
    0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64,
    0x72, 0xa7, 0xd, 0xd8, 0x8c, 0x59, 0xf3, 0x26,
    0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0xf,
    0x20, 0xf5, 0x5f, 0x8a, 0xde, 0xb, 0xa1, 0x74,
    0x9, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d,
    0xd6, 0x3, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82,
    0xff, 0x2a, 0x80, 0x55, 0x1, 0xd4, 0x7e, 0xab,
    0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x5, 0xd0,
    0xad, 0x78, 0xd2, 0x7, 0x53, 0x86, 0x2c, 0xf9,
};



#endif
