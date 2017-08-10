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

#include "build/debug.h"
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
#include "sensors/gyroanalyse.h"
#include "tinyosd.h"

#include "io/serial.h"

static bool  tinyOSDLock        = false;
static serialPort_t *tinyOSDPort = 0;

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



typedef enum {
    OSD_WRITE_MODE_VERTICAL = 0x0,
    OSD_WRITE_MODE_HORIZONTAL
} openTCOCommandOSDWriteMode_e;

#define OPENTCO_PROTOCOL_HEADER 0x80

#define OPENTCO_MAX_DATA_LENGTH 60

#define OPENTCO_DEVICE_OSD 0x00
#define OPENTCO_DEVICE_VTX 0x01
#define OPENTCO_DEVICE_CAM 0x02

#define OPENTCO_OSD_COMMAND_SET_REGISTER    0x00
#define OPENTCO_OSD_COMMAND_FILL_REGION     0x01
#define OPENTCO_OSD_COMMAND_WRITE           0x02
#define OPENTCO_OSD_COMMAND_WRITE_BUFFER_H  0x08
#define OPENTCO_OSD_COMMAND_WRITE_BUFFER_V  0x09
#define OPENTCO_OSD_COMMAND_SPECIAL         0x0F

#define OPENTCO_OSD_REGISTER_STATUS         0x00
#define OPENTCO_OSD_REGISTER_VIDEO_FORMAT   0x01

#define OPENTCO_OSD_COMMAND_SPECIAL_SUB_STICKSTATUS 0x00
#define OPENTCO_OSD_COMMAND_SPECIAL_SUB_SPECTRUM    0x01

static void openTCOCommandOSDSetRegister(const uint8_t reg, const uint8_t value);
static void openTCOCommandOSDFillRegion(const uint8_t x, const uint8_t y, const uint8_t width, const uint8_t height, const uint8_t value);
static void openTCOCommandOSDWrite(const uint8_t x, const uint8_t y, const uint8_t value);
static void openTCOCommandOSDWriteBuffer(const uint8_t x, const uint8_t y, const openTCOCommandOSDWriteMode_e mode, const uint8_t len, const uint8_t *data);
static void openTCOCommandOSDSpecialCommand(const uint8_t len, const uint8_t *data);
static void openTCOCommandOSDFillScreen(const uint8_t value);
static void openTCOEncode(const uint8_t device, const uint8_t command, const uint8_t datalen, const uint8_t *data);

static void openTCOCommandOSDSetRegister(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;
    openTCOEncode(OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_SET_REGISTER, 2, buffer);
}

static void openTCOCommandOSDFillRegion(const uint8_t x, const uint8_t y, const uint8_t width, const uint8_t height, const uint8_t value)
{
    uint8_t buffer[5];
    buffer[0] = x;
    buffer[1] = y;
    buffer[2] = width;
    buffer[3] = height;
    buffer[4] = value;

    // fill a region with given char
    openTCOEncode(OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_FILL_REGION, 5, buffer);
}

static void openTCOCommandOSDWrite(const uint8_t x, const uint8_t y, const uint8_t value)
{
    uint8_t buffer[3];
    buffer[0] = x;
    buffer[1] = y;
    buffer[2] = value;
    openTCOEncode(OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_WRITE, 3, buffer);
}


static void openTCOCommandOSDWriteBuffer(const uint8_t x, const uint8_t y, const openTCOCommandOSDWriteMode_e mode, const uint8_t len, const uint8_t *data)
{
    uint8_t buffer[OPENTCO_MAX_DATA_LENGTH];
    if (len >= OPENTCO_MAX_DATA_LENGTH - 3) {
        // invalid data length! abort
        return;
    }

    buffer[0] = len + 2;
    buffer[1] = x;
    buffer[2] = y;
    memcpy(buffer + 3, data, len);

    if (mode == OSD_WRITE_MODE_VERTICAL) {
        openTCOEncode(OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_WRITE_BUFFER_V, len + 3, buffer);
    } else {
        openTCOEncode(OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_WRITE_BUFFER_H, len + 3, buffer);
    }
}

static void openTCOCommandOSDSpecialCommand(const uint8_t len, const uint8_t *data)
{
    // send anything you like, data[0] is treated as subcommand
    openTCOEncode(OPENTCO_DEVICE_OSD, OPENTCO_OSD_COMMAND_SPECIAL, len, data);
}

static void openTCOCommandOSDFillScreen(const uint8_t value) {
    openTCOCommandOSDFillRegion(0, 0, 255, 255, value);
}

static void openTCOEncode(const uint8_t device, const uint8_t command, const uint8_t datalen, const uint8_t *data){
    uint8_t crc = 0;
    uint8_t header[2];

    if (!tinyOSDPort) return;
    // lock access
    // FIXME: once this is used from multiple drivers we should
    // add a wait with a timeout
    if (!tinyOSDLock) {
        tinyOSDLock = true;
    } else {
        return;
    }

    // create header
    header[0] = OPENTCO_PROTOCOL_HEADER;
    header[1] = ((device & 0x0F)<<4) | (command & 0x0F);

    // calculate checksum:
    TINYOSD_CRC8_INIT(crc, 0);
    TINYOSD_CRC8_UPDATE(crc, header[0]);
    TINYOSD_CRC8_UPDATE(crc, header[1]);

    // send header
    serialWriteBuf(tinyOSDPort, header, 2);

    // send data
    serialWriteBuf(tinyOSDPort, data, datalen);

    // calc crc over data
    for (uint8_t idx=0; idx < datalen; idx++){
        TINYOSD_CRC8_UPDATE(crc, data[idx]);
    }

    // send checksum
    serialWriteBuf(tinyOSDPort, &crc, 1);

    // unlock access
    tinyOSDLock = false;
}


//static uint8_t  videoSignalCfg;
static uint8_t  video_system;

static uint8_t  hosRegValue; // HOS (Horizontal offset register) value
static uint8_t  vosRegValue; // VOS (Vertical offset register) value


static bool fontIsLoading       = false;



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

void tinyOSDReInit(void)
{
    //uint8_t maxScreenRows;
    //uint8_t srdata = 0;
    //uint16_t x;
    static bool firstInit = true;

    if (video_system == VIDEO_SYSTEM_AUTO) {
        // fetch video mode from tinyOSD FIXME
        video_system = VIDEO_SYSTEM_NTSC;
    } else {
        // set video system
        openTCOCommandOSDSetRegister(OPENTCO_OSD_REGISTER_VIDEO_FORMAT, video_system);
    }

    // FIXME: feth this info from device
    if (video_system == VIDEO_SYSTEM_PAL) {
        tinyOSD_maxScreenSize = TINYOSD_VIDEO_BUFFER_CHARS_PAL;
    } else {              // NTSC
        tinyOSD_maxScreenSize = TINYOSD_VIDEO_BUFFER_CHARS_NTSC;
    }

    // enable osd
    openTCOCommandOSDSetRegister(OPENTCO_OSD_REGISTER_STATUS, 1);

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
    //hosRegValue = 32 - pVcdProfile->h_offset;
    //vosRegValue = 16 - pVcdProfile->v_offset;

    // find tinyosd serial port
    serialPortConfig_t *osdPortConfig = findSerialPortConfig(FUNCTION_TINYOSD);

    if (!osdPortConfig) {
        // could not get port -> abort
        return false;
    }

    // we will do fw upgrades in the feature so reserve rx and tx
    portMode_e mode = MODE_RXTX;

    baudRate_e baudRateIndex =  osdPortConfig->blackbox_baudrateIndex;
    uint32_t baudrate = baudRates[baudRateIndex];

    // open assigned serial port (no callback for RX right now)
    tinyOSDPort = openSerialPort(osdPortConfig->identifier, FUNCTION_NONE, NULL, baudrate, mode, SERIAL_NOT_INVERTED);

    // verify opening
    if (!tinyOSDPort) {
        return false;
    }

    // fill whole screen on device with ' '
    openTCOCommandOSDFillScreen(' ');

    return true;
}

// fill the whole screen with spaces
int tinyOSDClearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    openTCOCommandOSDFillScreen(' ');
    return 0;
}

int tinyOSDFillRegion(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t value){
    UNUSED(displayPort);
    openTCOCommandOSDFillRegion(x, y, width, height, value);
    return 0;
}


uint8_t* tinyOSDGetScreenBuffer(void) {
    return screenBuffer;
}

static void tinyOSDDrawSticks(void) {
    uint8_t buffer[7];

    // assemble special subcommand
    buffer[0] = 6;
    buffer[1] = OPENTCO_OSD_COMMAND_SPECIAL_SUB_STICKSTATUS;
    buffer[2] = TINYOSD_STICKSIZE_X/2 + (TINYOSD_STICKSIZE_X/2 * rcCommand[ROLL]) / 500.0f;
    buffer[3] = TINYOSD_STICKSIZE_Y/2 - (TINYOSD_STICKSIZE_Y/2 * rcCommand[PITCH]) / 500.0f;
    // throttle is 1000 - 2000, rescale to match out STICK_Y resolution
    buffer[4] = TINYOSD_STICKSIZE_Y - (TINYOSD_STICKSIZE_Y * (rcCommand[THROTTLE]-1000.0f))/1000.0f; //(TINYOSD_STICKSIZE_Y/2 * (rcCommand[THROTTLE]-1.0)) / 1000.0;
    buffer[5] = TINYOSD_STICKSIZE_X/2 - (TINYOSD_STICKSIZE_X/2 * rcCommand[YAW]) / 500.0f;
    // armed?
    buffer[6] = armingFlags;

    openTCOCommandOSDSpecialCommand(7, buffer);
}

static void tinyOSDDrawFFT(void) {
    uint8_t buffer[3 + GYRO_FFT_BIN_COUNT];
    uint8_t axis = 0;

    const gyroFftData_t *gyro_fft = gyroFftData(axis);

    // assemble special subcommand
    buffer[0] = 2+GYRO_FFT_BIN_COUNT;
    buffer[1] = OPENTCO_OSD_COMMAND_SPECIAL_SUB_SPECTRUM;
    buffer[2] = axis;

    static float maxgyro[3];

    // add scaled data:
    float maxval = gyro_fft->maxVal;
    // simple LPF  y[i] = y[i-1] + (a) * ( x[i] - y[i-1] )
    maxgyro[axis] = maxgyro[axis] + 0.01 * (maxval - maxgyro[axis]);

    for(uint32_t i=0; i < GYRO_FFT_BIN_COUNT; i++) {
        // make sure to limit this to 255.0 in any case
        buffer[3 + i] = MAX(255.0, ((255.0 * gyro_fft->bin2[i]) / maxgyro[axis]));
    }

    openTCOCommandOSDSpecialCommand(3 + GYRO_FFT_BIN_COUNT, buffer);
}

int tinyOSDWriteChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t c)
{
    UNUSED(displayPort);
    openTCOCommandOSDWrite(x, y, c);
    return 0;
}

int tinyOSDWriteString(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *buff)
{
    UNUSED(displayPort);
    uint8_t len = strlen(buff);
    openTCOCommandOSDWriteBuffer(x, y, OSD_WRITE_MODE_HORIZONTAL, len, (const uint8_t *)buff);
    return 0;
}

int tinyOSDDrawScreen(displayPort_t *displayPortProfile)
{
    UNUSED(displayPortProfile);
    return 0;
}

bool tinyOSDIsTransferInProgress(const displayPort_t *displayPort){
    UNUSED(displayPort);
    return false;
}

int tinyOSDHeartbeat(displayPort_t *displayPort){
    UNUSED(displayPort);

    // FIXME: this should not be sent with 100hz...
    tinyOSDDrawFFT();
    tinyOSDDrawSticks();

    return 0;
}

// This funcktion refresh all and should not be used when copter is armed
void tinyOSDRefreshAll(void)
{
    // FIXME: with successive osd item drawing this has to be handled differently
}

void tinyOSDResync(displayPort_t *displayPort)
{
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
