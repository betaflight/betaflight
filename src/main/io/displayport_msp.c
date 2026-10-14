/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_MSP_DISPLAYPORT

#include "build/debug.h"

#include "cli/cli.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/osd.h"
#include "drivers/time.h"

#include "io/displayport_msp.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "osd/osd.h"

#include "pg/vcd.h"

// A busy screen sent whole, as the OSD draws it, needs more than a 115200 baud link carries at the
// frame rate of the OSD, and what does not fit in the serial buffer is dropped without a trace. The
// goggles keep what they hold between draws, so the driver sends only the writes of the OSD that
// changed. It keeps no copy of the screen, only a table of the writes the goggles hold: where each
// one is, its length and attribute, and a hash of its characters, and two bits a cell for the limit
// of DJI O3 Air Unit + Goggles 2.
//
// The screen is the 53x20 default HD canvas, or smaller. A canvas that does not fit, or the setting
// switched off, gets the commands of the OSD as they come.
#define MSP_DP_COLS OSD_HD_COLS
#define MSP_DP_ROWS OSD_HD_ROWS

// Writes the table holds. A screen with more is still shown, and cleared and sent whole now and then.
#define MSP_DP_WRITES 96

// MSP_DP_WRITE_STRING carries at most this many characters. DJI O3/Goggles 2 accept 32, other
// receivers may be sized for 30.
#define MSP_DP_WRITE_MAX 30

// Bytes a write costs besides its characters: the MSP v1 frame (6) and the write header (4)
#define MSP_DP_WRITE_OVERHEAD 10
#define MSP_DP_COMMAND_BYTES 7

// The goggles can miss a write, or be switched off and on, and nothing tells the flight controller.
// Switched on again, DJI Goggles 2 draw nothing until a CLEAR_SCREEN. So the goggles are cleared and
// sent the whole screen this often, in one draw, which they show at once.
#define MSP_DP_REFRESH_MS 1000

// The CMS draws its page again after a clear, in a few passes. The draw waits this long at most for it.
#define MSP_DP_REPAINT_MS 300

// DJI O3 Air Unit + Goggles 2 throw away a whole draw when the screen they hold at the draw is too big
// in their encoding: the 53x20 cells, a byte each and 0 for a blank one, read as 265 little-endian
// 32 bit words, each coded in LEB128. The goggles show the screen at up to 902 bytes. This fits every
// bench measurement to the byte, and random screens of 392 to 459 cells came out shown or dropped as
// it said. A space is a character like any other; a cell blanked with 0 costs nothing, and DJI shows
// 0 as a blank cell where it is in a write, so a DJI air unit gets 0 for a space.
//
// So a word takes one to four bytes, as far as its last cell that is not blank, one more when that
// cell holds at least 0x80, 0x40, 0x20 or 0x10 as the first to fourth cell of the word, and a blank
// word takes one byte.
#define MSP_DP_WORDS (MSP_DP_COLS * MSP_DP_ROWS / 4)

// A DJI O3 Air Unit polls the flight controller five times a second over MSP for, among others,
// FILTER_CONFIG, and never for OSD_CONFIG or ATTITUDE. The DJI air unit before it (V1, Vista),
// which draws the OSD itself, asks for both, and so does Configurator. An O4 was not measured.
#define MSP_DP_DJI_POLLS 5

// Flags of a write in the table, in bits of the attribute that are never sent
#define MSP_DP_TOUCHED DISPLAYPORT_MSP_ATTR_VERSION     // Written in this frame
#define MSP_DP_STALE   BIT(5)                           // To be sent again

typedef struct mspDpWrite_s {
    uint8_t row;
    uint8_t col;
    uint8_t len;                                // 0 for a free entry
    uint8_t attr;
    uint32_t hash;
} mspDpWrite_t;

static displayPort_t mspDisplayPort;
static serialPortIdentifier_e displayPortSerial;
static int8_t djiPolls;                         // FILTER_CONFIG polls on the display's port, -1 when it is not DJI

static struct {
    mspDpWrite_t writes[MSP_DP_WRITES];         // What the goggles hold
    uint64_t held[MSP_DP_ROWS];                 // Cells the goggles hold a character in, a space sent as one too
    uint64_t wide[MSP_DP_ROWS];                 // Of those, the ones that make their word a byte longer (DJI)
    uint16_t load;                              // Bytes of the screen the goggles hold, in their encoding (DJI)
    uint32_t refreshAt;
    uint32_t repaintUntil;
    uint16_t repaintCells;                      // Cells of the page the CMS draws again, before the draw
    uint16_t bytesSent;                         // In the frame
    uint16_t cellsSent;
    bool frameCleared;                          // The OSD cleared the screen to begin the frame
    bool needClear;                             // The goggles are to be cleared at the start of the next frame
    bool fresh;                                 // The goggles were cleared at the start of this frame
    bool full;                                  // The screen does not fit, what does not is left out until the next clear
    bool drawPending;
} mspScreen;

typedef struct displayPortMspCommand_s {
    uint8_t command;
    uint8_t row;
    uint8_t col;
    uint8_t attribute;
    uint8_t data[OSD_CHAR_BYTES];
} __attribute__((packed)) displayPortMspCommand_t;

static int output(displayPort_t *displayPort, uint8_t cmd, uint8_t *buf, int len)
{
    UNUSED(displayPort);

    return mspSerialPush(displayPortSerial, cmd, buf, len, MSP_DIRECTION_REPLY, MSP_V1);
}

static int heartbeat(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { MSP_DP_HEARTBEAT };

    // heartbeat is used to:
    // a) ensure display is not released by MW OSD software
    // b) prevent OSD Slave boards from displaying a 'disconnected' status.
    output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));

    return 0;
}

static int grab(displayPort_t *displayPort)
{
    return heartbeat(displayPort);
}

static bool shadowed(void)
{
    return displayPortProfileMsp()->sendChanges && mspDisplayPort.cols <= MSP_DP_COLS && mspDisplayPort.rows <= MSP_DP_ROWS;
}

// A DJI O3 Air Unit polls on the display's port
static bool dji(void)
{
    return djiPolls >= MSP_DP_DJI_POLLS;
}

void displayPortMspPolled(mspDescriptor_t srcDesc, int16_t cmd)
{
    if (djiPolls < 0 || dji() || (cmd != MSP_FILTER_CONFIG && cmd != MSP_OSD_CONFIG && cmd != MSP_ATTITUDE)
        || srcDesc != getMspSerialPortDescriptor(displayPortSerial)) {
        return;
    }
    if (cmd != MSP_FILTER_CONFIG) {
        djiPolls = -1;
    } else if (++djiPolls == MSP_DP_DJI_POLLS) {
        mspScreen.needClear = true;                     // Send the screen again with blank cells
    }
}

static uint8_t cellByte(char c)
{
    return (c == ' ' && dji()) ? 0 : c;
}

// Bytes of a word of the screen the goggles hold, in their encoding
static int wordLoad(int word)
{
    for (int pos = 3; pos >= 0; pos--) {
        const int row = (word * 4 + pos) / MSP_DP_COLS;
        const int col = (word * 4 + pos) % MSP_DP_COLS;

        if ((mspScreen.held[row] >> col) & 1) {
            return pos + 1 + ((mspScreen.wide[row] >> col) & 1);
        }
    }

    return 1;
}

// Note the cells of a write as the goggles hold them, spaces for no string
static void putCells(int row, int col, const char *string, int len)
{
    const int first = (row * MSP_DP_COLS + col) / 4;
    const int last = (row * MSP_DP_COLS + col + len - 1) / 4;
    int load = mspScreen.load;

    for (int word = first; word <= last; word++) {
        load -= wordLoad(word);
    }
    for (int i = 0; i < len; i++) {
        const uint8_t c = cellByte(string ? string[i] : ' ');
        const uint64_t bit = (uint64_t)1 << (col + i);

        mspScreen.held[row] = c ? mspScreen.held[row] | bit : mspScreen.held[row] & ~bit;
        mspScreen.wide[row] = c >= (0x80 >> ((row * MSP_DP_COLS + col + i) & 3)) ? mspScreen.wide[row] | bit : mspScreen.wide[row] & ~bit;
    }
    for (int word = first; word <= last; word++) {
        load += wordLoad(word);
    }
    mspScreen.load = load;
}

static uint64_t span(int col, int len)
{
    return (((uint64_t)1 << len) - 1) << col;
}

static int writeBytes(int len)
{
    return len + (len + MSP_DP_WRITE_MAX - 1) / MSP_DP_WRITE_MAX * MSP_DP_WRITE_OVERHEAD;
}

// Room in the serial buffer, as a command that finds none is dropped. Stop waiting after a while, until
// there is room again, rather than leave the OSD waiting for a port that does not drain.
static bool room(int bytes)
{
    static uint32_t waitingSince;
    static bool waiting;

    if (mspSerialTxBytesFree(displayPortSerial) >= (uint32_t)bytes) {
        waiting = false;
        return true;
    }
    if (!waiting) {
        waiting = true;
        waitingSince = millis();
    }

    return cmp32(millis(), waitingSince) > 250;
}

// Send a write, spaces for no string, in commands of at most MSP_DP_WRITE_MAX characters
static void sendWrite(displayPort_t *displayPort, int row, int col, uint8_t mspAttr, const char *string, int len)
{
    for (int done = 0; done < len; done += MSP_DP_WRITE_MAX) {
        const int n = MIN(len - done, MSP_DP_WRITE_MAX);
        uint8_t buf[MSP_DP_WRITE_MAX + 4] = { MSP_DP_WRITE_STRING, row, col + done, mspAttr };

        for (int i = 0; i < n; i++) {
            buf[4 + i] = cellByte(string ? string[done + i] : ' ');
        }
        mspScreen.bytesSent += output(displayPort, MSP_DISPLAYPORT, buf, n + 4);
    }
    mspScreen.cellsSent += len;
    mspScreen.drawPending = true;
}

static void sendClear(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { MSP_DP_CLEAR_SCREEN };

    mspScreen.bytesSent += output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
    memset(mspScreen.writes, 0, sizeof(mspScreen.writes));
    memset(mspScreen.held, 0, sizeof(mspScreen.held));
    memset(mspScreen.wide, 0, sizeof(mspScreen.wide));
    mspScreen.load = MSP_DP_WORDS;
    mspScreen.needClear = mspScreen.full = false;
    mspScreen.fresh = mspScreen.drawPending = true;
    mspScreen.repaintCells = 0;
    mspScreen.refreshAt = millis() + MSP_DP_REFRESH_MS;
}

// The goggles or the table take no more: clear at the start of the next frame, unless this one
// started with a clear, then what does not fit is left out until the next clear
static void overflow(void)
{
    if (mspScreen.fresh || mspScreen.full) {
        mspScreen.full = true;
    } else {
        mspScreen.needClear = true;
    }
}

// The write the goggles hold at this place and the frame has not written yet, the one with the same
// characters if there are more
static mspDpWrite_t *findWrite(int row, int col, int len, uint32_t hash)
{
    mspDpWrite_t *found = NULL;

    for (mspDpWrite_t *w = mspScreen.writes; w < ARRAYEND(mspScreen.writes); w++) {
        if (w->len && w->row == row && w->col == col && !(w->attr & MSP_DP_TOUCHED)) {
            if (w->len == len && w->hash == hash) {
                return w;
            }
            found = found ? found : w;
        }
    }

    return found;
}

static mspDpWrite_t *newWrite(void)
{
    mspDpWrite_t *w;

    for (w = mspScreen.writes; w < ARRAYEND(mspScreen.writes) && w->len; w++);
    if (w == ARRAYEND(mspScreen.writes)) {
        overflow();
        return NULL;
    }

    return w;
}

// Cells of the writes the table holds
static int heldCells(void)
{
    int cells = 0;

    for (const mspDpWrite_t *w = mspScreen.writes; w < ARRAYEND(mspScreen.writes); w++) {
        cells += w->len;
    }

    return cells;
}

// Cells of a row that the writes of this frame cover. Where one of them covers cells that another
// write had drawn over, the goggles may show the other one there, so it is sent again in the next frame.
static uint64_t covered(int row, uint64_t over)
{
    uint64_t bits = 0;

    for (mspDpWrite_t *w = mspScreen.writes; w < ARRAYEND(mspScreen.writes); w++) {
        if (w->len && w->row == row && (w->attr & MSP_DP_TOUCHED)) {
            bits |= span(w->col, w->len);
            if (span(w->col, w->len) & over) {
                w->attr |= MSP_DP_STALE;
            }
        }
    }

    return bits;
}

static int release(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { MSP_DP_RELEASE };

    // The goggles blank their OSD
    mspScreen.needClear = true;

    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static int clearScreen(displayPort_t *displayPort, displayClearOption_e options)
{
    if (!shadowed()) {
        uint8_t subcmd[] = { MSP_DP_CLEAR_SCREEN };

        return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
    }

    // A screen of its own, as a menu page or the stats, starts on clear goggles, and a frame of the OSD
    // replaces the screen at the draw, which blanks what the frame leaves out
    if ((options & DISPLAY_CLEAR_WAIT) || cmp32(millis(), mspScreen.refreshAt) >= 0) {
        mspScreen.needClear = true;
    }
    mspScreen.frameCleared = true;
    mspScreen.fresh = false;
    if (mspScreen.needClear && mspSerialTxBytesFree(displayPortSerial) >= MSP_DP_COMMAND_BYTES) {
        sendClear(displayPort);
    }

    return 0;
}

static bool drawScreen(displayPort_t *displayPort)
{
    uint8_t draw[] = { MSP_DP_DRAW_SCREEN };

    if (!shadowed()) {
        output(displayPort, MSP_DISPLAYPORT, draw, sizeof(draw));

        return false;
    }

    // The CMS draws without clearing, so the once a second clear is done here, and the CMS asked to
    // draw its page again. The goggles get the draw once the page is back, as they show it at the draw.
    if (!mspScreen.frameCleared && displayIsGrabbed(displayPort) && cmp32(millis(), mspScreen.refreshAt) >= 0
        && mspSerialTxBytesFree(displayPortSerial) >= MSP_DP_COMMAND_BYTES) {
        const int page = heldCells();

        sendClear(displayPort);
        mspScreen.repaintCells = page;
        mspScreen.repaintUntil = millis() + MSP_DP_REPAINT_MS;
        displayPort->cleared = true;
        displayPort->cursorRow = -1;
    }

    int writes = 0;

    for (mspDpWrite_t *w = mspScreen.writes; w < ARRAYEND(mspScreen.writes); w++) {
        if (w->len && !(w->attr & MSP_DP_TOUCHED) && mspScreen.frameCleared) {
            // Blank what the frame did not write again
            const uint64_t cells = span(w->col, w->len);

            for (uint64_t blank = cells & ~covered(w->row, cells) & mspScreen.held[w->row]; blank; ) {
                const int col = __builtin_ctzll(blank);
                const int len = MIN(__builtin_ctzll(~(blank >> col)), MSP_DP_WRITE_MAX);

                if (!room(len + MSP_DP_WRITE_OVERHEAD)) {
                    return true;
                }
                sendWrite(displayPort, w->row, col, 0, NULL, len);
                putCells(w->row, col, NULL, len);
                blank &= ~span(col, len);
                w->len -= col + len - w->col;               // What is left, if the draw has to wait
                w->col = col + len;
            }
            w->len = 0;
        }
    }
    for (mspDpWrite_t *w = mspScreen.writes; w < ARRAYEND(mspScreen.writes); w++) {
        w->attr &= ~MSP_DP_TOUCHED;
        writes += w->len != 0;
    }
    mspScreen.frameCleared = false;

    if (mspScreen.repaintCells && (heldCells() >= mspScreen.repaintCells || cmp32(millis(), mspScreen.repaintUntil) >= 0)) {
        mspScreen.repaintCells = 0;
    }
    if (mspScreen.drawPending && !mspScreen.repaintCells) {
        if (!room(sizeof(draw))) {
            return true;
        }
        mspScreen.bytesSent += output(displayPort, MSP_DISPLAYPORT, draw, sizeof(draw));
        mspScreen.drawPending = false;
    }

    DEBUG_SET(DEBUG_MSP_DISPLAYPORT, 0, mspScreen.bytesSent);  //!< Bytes Sent [unit:bytes]
    DEBUG_SET(DEBUG_MSP_DISPLAYPORT, 1, mspScreen.cellsSent);  //!< Cells Sent
    DEBUG_SET(DEBUG_MSP_DISPLAYPORT, 2, dji() ? mspScreen.load : 0);  //!< DJI Screen Load [unit:bytes]
    DEBUG_SET(DEBUG_MSP_DISPLAYPORT, 3, writes);  //!< Writes Held
    mspScreen.bytesSent = 0;
    mspScreen.cellsSent = 0;

    return false;
}

static int screenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

static int writeString(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t attr, const char *string)
{
    uint8_t mspAttr = displayPortProfileMsp()->fontSelection[attr & (DISPLAYPORT_SEVERITY_COUNT - 1)] & DISPLAYPORT_MSP_ATTR_FONT;

    if (attr & DISPLAYPORT_BLINK) {
        mspAttr |= DISPLAYPORT_MSP_ATTR_BLINK;
    }

    if (!shadowed()) {
#define MSP_OSD_MAX_STRING_LENGTH 30 // FIXME move this
        uint8_t buf[MSP_OSD_MAX_STRING_LENGTH + 4];
        const int len = MIN((int)strlen(string), MSP_OSD_MAX_STRING_LENGTH);

        buf[0] = MSP_DP_WRITE_STRING;
        buf[1] = row;
        buf[2] = col;
        buf[3] = mspAttr;
        memcpy(&buf[4], string, len);

        return output(displayPort, MSP_DISPLAYPORT, buf, len + 4);
    }

    if (row >= MSP_DP_ROWS || col >= MSP_DP_COLS || !*string) {
        return 0;
    }

    const int len = MIN((int)strlen(string), MSP_DP_COLS - col);
    const uint32_t hash = fnv_update(FNV_OFFSET_BASIS, string, len);
    mspDpWrite_t *w = findWrite(row, col, len, hash);

    // A write that blinks is sent every time, as the goggles blink it on their own
    if (w && w->len == len && w->hash == hash && (w->attr & ~MSP_DP_TOUCHED) == mspAttr && !(mspAttr & DISPLAYPORT_MSP_ATTR_BLINK)) {
        w->attr |= MSP_DP_TOUCHED;

        return 0;
    }

    // Spaces at the ends need not go to cells that are blank on the goggles
    const uint64_t held = mspScreen.held[row];
    int first = 0;
    int last = len - 1;

    while (first <= last && string[first] == ' ' && !((held >> (col + first)) & 1)) {
        first++;
    }
    while (last >= first && string[last] == ' ' && !((held >> (col + last)) & 1)) {
        last--;
    }

    if (first <= last) {
        const int n = last - first + 1;
        const uint64_t cells = span(col + first, n);

        if (mspSerialTxBytesFree(displayPortSerial) < (uint32_t)writeBytes(n)) {
            if (w) {
                w->attr |= MSP_DP_TOUCHED | MSP_DP_STALE;
            }

            return 0;
        }

        const uint64_t wide = mspScreen.wide[row];
        const uint16_t load = mspScreen.load;

        putCells(row, col + first, string + first, n);
        if (dji() && displayPortProfileMsp()->djiBudget && mspScreen.load > displayPortProfileMsp()->djiBudget) {
            mspScreen.held[row] = held;
            mspScreen.wide[row] = wide;
            mspScreen.load = load;
            overflow();
            if (w) {
                w->attr |= MSP_DP_TOUCHED;                  // The goggles show what was there
            }

            return 0;
        }
        sendWrite(displayPort, row, col + first, mspAttr, string + first, n);

        // What the goggles held under it and the frame has not written yet is not there any more
        for (mspDpWrite_t *e = mspScreen.writes; e < ARRAYEND(mspScreen.writes); e++) {
            if (e != w && e->len && e->row == row && !(e->attr & MSP_DP_TOUCHED) && (span(e->col, e->len) & cells)) {
                e->attr |= MSP_DP_STALE;
            }
        }
    }

    if (w && w->len > len) {
        // The cells the write no longer covers are blanked at the draw, unless the frame writes them
        mspDpWrite_t *tail = newWrite();

        if (tail) {
            *tail = (mspDpWrite_t){ row, col + len, w->len - len, MSP_DP_STALE, 0 };
        }
    }
    if (!w) {
        w = newWrite();
    }
    if (w) {
        *w = (mspDpWrite_t){ row, col, len, mspAttr | MSP_DP_TOUCHED, hash };
    }

    return 0;
}

static int writeSys(displayPort_t *displayPort, uint8_t col, uint8_t row, displayPortSystemElement_e systemElement)
{
    uint8_t syscmd[4];

    syscmd[0] = MSP_DP_SYS;
    syscmd[1] = row;
    syscmd[2] = col;
    syscmd[3] = systemElement;

    return output(displayPort, MSP_DISPLAYPORT, syscmd, sizeof(syscmd));
}

static int writeChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t attr, uint8_t c)
{
    char buf[2];

    buf[0] = c;
    buf[1] = 0;
    return writeString(displayPort, col, row, attr, buf);
}

// The serial transmit buffer drains at the baud rate and a frame that finds it full is dropped without
// a trace, so the OSD waits for some room in the buffer of the display's port before it sends anything,
// and gives up after a while. Room for the largest write is enough: one more to follow can wait in turn.
// Switched off, the commands of the OSD go out as they come, as they always have.
#define MSP_DP_TX_ROOM 64

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return shadowed() && !room(MSP_DP_TX_ROOM);
}

static bool isSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true;
}

static void redraw(displayPort_t *displayPort)
{
    mspScreen.drawPending = true;
    drawScreen(displayPort);
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return mspSerialTxBytesFree(displayPortSerial);
}

#ifdef USE_MSP_DISPLAYPORT_FONT
static bool writeFontCharacter(displayPort_t *displayPort, uint16_t addr, const osdCharacter_t *chr)
{
    displayPortMspCommand_t displayPortCommand;

    if (!chr) {
        return false;
    }

    displayPortCommand.command = MSP_DP_FONTCHAR_WRITE;
    displayPortCommand.row = addr & 0xff;
    displayPortCommand.col = (addr >> 8) & 0xff;
    displayPortCommand.attribute = 0;

    memcpy(displayPortCommand.data, chr->data, OSD_CHAR_BYTES);

    int res = output(displayPort, MSP_DISPLAYPORT, (uint8_t*)&displayPortCommand, OSD_CHAR_BYTES + 4);

    // 80ms delay needed to ensure the MSP display has enough time to process the font data
    delay(80);

    return res > 0;
}

static bool checkReady(displayPort_t *displayPort, bool rescan)
{
    UNUSED(rescan);
    if (
        !displayPort ||
        displayPort->deviceType != DISPLAYPORT_DEVICE_TYPE_MSP
    ) {
        return false;
    }

    return true;
}
#endif

static const displayPortVTable_t mspDisplayPortVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeSys = writeSys,
    .writeString = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .redraw = redraw,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree,
#ifdef USE_MSP_DISPLAYPORT_FONT
    .writeFontCharacter = writeFontCharacter,
    .checkReady = checkReady,
#endif
    .layerSupported = NULL,
    .layerSelect = NULL,
    .layerCopy = NULL,
};

displayPort_t *displayPortMspInit(void)
{
    displayInit(&mspDisplayPort, &mspDisplayPortVTable, DISPLAYPORT_DEVICE_TYPE_MSP);

    if (displayPortProfileMsp()->useDeviceBlink) {
        mspDisplayPort.useDeviceBlink = true;
    }

#ifndef USE_OSD_SD
    if (vcdProfile()->video_system != VIDEO_SYSTEM_HD) {
        vcdProfileMutable()->video_system = VIDEO_SYSTEM_HD;
    }
#endif
#ifndef USE_OSD_HD
    if (vcdProfile()->video_system == VIDEO_SYSTEM_HD) {
        vcdProfileMutable()->video_system = VIDEO_SYSTEM_AUTO;
    }
#endif

    if (vcdProfile()->video_system == VIDEO_SYSTEM_HD) {
        mspDisplayPort.rows = osdConfig()->canvas_rows;
        mspDisplayPort.cols = osdConfig()->canvas_cols;
    } else {
        const uint8_t displayRows = (vcdProfile()->video_system == VIDEO_SYSTEM_PAL) ? VIDEO_LINES_PAL : VIDEO_LINES_NTSC;
        mspDisplayPort.rows = displayRows + displayPortProfileMsp()->rowAdjust;
        mspDisplayPort.cols = OSD_SD_COLS + displayPortProfileMsp()->colAdjust;
    }

    memset(&mspScreen, 0, sizeof(mspScreen));
    djiPolls = 0;
    mspScreen.needClear = true;                     // What the goggles hold is not known
    redraw(&mspDisplayPort);

    return &mspDisplayPort;
}

void displayPortMspSetSerial(serialPortIdentifier_e serialPort)
{
    displayPortSerial = serialPort;
}

serialPortIdentifier_e displayPortMspGetSerial(void) {
    return displayPortSerial;
}
#endif // USE_MSP_DISPLAYPORT
