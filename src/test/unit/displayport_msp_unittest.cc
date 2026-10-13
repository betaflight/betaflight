/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// The MSP displayport driver sends the writes of the OSD that changed. The tests run it against a
// model of the goggles and check what they show.

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"
    #include "common/maths.h"
    #include "common/utils.h"
    #include "drivers/display.h"
    #include "drivers/osd.h"
    #include "io/displayport_msp.h"
    #include "msp/msp_protocol.h"
    #include "msp/msp_serial.h"
    #include "osd/osd.h"
    #include "pg/displayport_profiles.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/vcd.h"

    PG_REGISTER(osdConfig_t, osdConfig, PG_OSD_CONFIG, 0);
    PG_REGISTER(vcdProfile_t, vcdProfile, PG_VCD_CONFIG, 0);
    PG_REGISTER(displayPortProfile_t, displayPortProfileMsp, PG_DISPLAY_PORT_MSP_CONFIG, 0);

    uint8_t debugMode;
    int16_t debug[DEBUG16_VALUE_COUNT];
}

#include "gtest/gtest.h"

#define COLS OSD_HD_COLS
#define ROWS OSD_HD_ROWS

typedef std::vector<std::string> screen_t;

struct packet_t {
    uint8_t subcmd;
    uint8_t row;
    uint8_t col;
    uint8_t attr;
    std::string text;
};

// The goggles: a screen drawn by the writes, shown at the draw, and the cells written since a clear
static struct {
    screen_t cells;
    screen_t shown;
    bool sent[ROWS][COLS];
    bool blind;                 // Switched on again: nothing is drawn until a clear
    uint32_t maxLoad;           // Of the cells written since a clear, at a draw, in thousandths of a cell
} goggles;

static std::vector<packet_t> packets;
static uint32_t bytes;
static uint32_t now;            // millis()
static uint32_t txFree;         // Room in the serial buffer

static screen_t blank(void)
{
    return screen_t(ROWS, std::string(COLS, ' '));
}

static void gogglesOff(void)
{
    goggles.cells = blank();
    memset(goggles.sent, 0, sizeof(goggles.sent));
    goggles.blind = false;
}

// What DJI O3/Goggles 2 count against their limit: a cell each, 0.75 more for a cell two columns
// on from the one before it, 1.3 further, in screen order
static uint32_t gogglesLoad(void)
{
    uint32_t load = 0;
    int last = -1;

    for (int p = 0; p < ROWS * COLS; p++) {
        if (goggles.sent[p / COLS][p % COLS]) {
            load += 1000 + (p - last == 2 ? 750 : (p - last > 2 ? 1300 : 0));
            last = p;
        }
    }

    return load;
}

extern "C" {
    int mspSerialPush(serialPortIdentifier_e, uint8_t cmd, uint8_t *data, int datalen, mspDirection_e, mspVersion_e)
    {
        EXPECT_EQ(MSP_DISPLAYPORT, cmd);
        packet_t p = { data[0], 0, 0, 0, "" };

        switch (data[0]) {
        case MSP_DP_WRITE_STRING:
            p = { data[0], data[1], data[2], data[3], std::string((const char *)&data[4], datalen - 4) };
            EXPECT_LE(p.text.size(), 30u);
            for (size_t i = 0; i < p.text.size() && p.col + i < COLS && p.row < ROWS; i++) {
                goggles.cells[p.row][p.col + i] = p.text[i];
                goggles.sent[p.row][p.col + i] = true;
            }
            break;
        case MSP_DP_CLEAR_SCREEN:
            gogglesOff();
            break;
        case MSP_DP_DRAW_SCREEN:
            if (!goggles.blind) {
                goggles.shown = goggles.cells;
            }
            goggles.maxLoad = MAX(goggles.maxLoad, gogglesLoad());
            break;
        }
        packets.push_back(p);
        bytes += datalen + 6;
        if (txFree != UINT32_MAX) {
            txFree = txFree > (uint32_t)datalen + 6 ? txFree - (datalen + 6) : 0;
        }

        return datalen + 6;
    }

    uint32_t mspSerialTxBytesFree(serialPortIdentifier_e) { return txFree; }
    uint32_t millis(void) { return now; }
}

static displayPort_t *init(uint16_t budget = 0)
{
    vcdProfileMutable()->video_system = VIDEO_SYSTEM_HD;
    osdConfigMutable()->canvas_cols = COLS;
    osdConfigMutable()->canvas_rows = ROWS;
    displayPortProfileMspMutable()->cellBudget = budget;
    displayPortProfileMspMutable()->sendChanges = true;
    for (int i = 0; i < DISPLAYPORT_SEVERITY_COUNT; i++) {
        displayPortProfileMspMutable()->fontSelection[i] = i;
    }
    debugMode = DEBUG_MSP_DISPLAYPORT;
    now = 1000;
    txFree = UINT32_MAX;
    gogglesOff();
    goggles.cells[0] = "what the goggles held before";
    goggles.shown = goggles.cells;
    goggles.maxLoad = 0;

    packets.clear();
    displayPort_t *dp = displayPortMspInit();
    packets.clear();

    return dp;
}

static packet_t at(int row, int col, const std::string &text, uint8_t attr = DISPLAYPORT_SEVERITY_NORMAL)
{
    return { 0, (uint8_t)row, (uint8_t)col, attr, text };
}

static screen_t drawn(const std::vector<packet_t> &writes)
{
    screen_t s = blank();

    for (const auto &w : writes) {
        for (size_t i = 0; i < w.text.size() && w.col + i < COLS; i++) {
            s[w.row][w.col + i] = w.text[i];
        }
    }

    return s;
}

// What the OSD does for a frame: clear, write, draw. Each frame takes 80 ms.
static void frame(displayPort_t *dp, const std::vector<packet_t> &writes)
{
    packets.clear();
    bytes = 0;
    now += 80;
    displayClearScreen(dp, DISPLAY_CLEAR_NONE);
    for (const auto &w : writes) {
        displayWrite(dp, w.col, w.row, w.attr, w.text.c_str());
    }
    while (displayDrawScreen(dp)) {
        txFree = UINT32_MAX;
    }
}

static std::vector<packet_t> writes(void)
{
    std::vector<packet_t> w;

    for (const auto &p : packets) {
        if (p.subcmd == MSP_DP_WRITE_STRING) {
            w.push_back(p);
        }
    }

    return w;
}

static int count(uint8_t subcmd)
{
    int n = 0;

    for (const auto &p : packets) {
        n += p.subcmd == subcmd;
    }

    return n;
}

// A screen of n writes of 10 characters
static std::vector<packet_t> busy(int n)
{
    std::vector<packet_t> s;

    for (int i = 0; i < n; i++) {
        char text[11];
        snprintf(text, sizeof(text), "W%02d=%06d", i, i * 7919);
        s.push_back(at(i % ROWS, (i / ROWS) * 11, text));
    }

    return s;
}

TEST(DisplayPortMspTest, TheFirstFrameClearsTheGoggles)
{
    displayPort_t *dp = init();

    frame(dp, { at(3, 2, "ABC"), at(5, 10, "D E") });
    ASSERT_EQ(4u, packets.size());
    EXPECT_EQ(MSP_DP_CLEAR_SCREEN, packets[0].subcmd);
    EXPECT_EQ("ABC", packets[1].text);
    EXPECT_EQ("D E", packets[2].text);
    EXPECT_EQ(MSP_DP_DRAW_SCREEN, packets[3].subcmd);
    EXPECT_EQ(drawn({ at(3, 2, "ABC"), at(5, 10, "D E") }), goggles.shown);
}

TEST(DisplayPortMspTest, OnlyTheWritesThatChangedAreSent)
{
    displayPort_t *dp = init();
    auto s = busy(40);

    frame(dp, s);
    s[7].text = "changed!";
    frame(dp, s);
    EXPECT_EQ(0, count(MSP_DP_CLEAR_SCREEN));
    EXPECT_EQ(1, count(MSP_DP_DRAW_SCREEN));
    bool found = false;
    for (const auto &w : writes()) {
        found |= w.text == "changed!";
    }
    EXPECT_TRUE(found);
    EXPECT_LE(bytes, 40u + 18 + 2 * 20 + 7);            // The change and the refresh, the draw
    EXPECT_EQ(drawn(s), goggles.shown);
}

TEST(DisplayPortMspTest, TheSameScreenIsSentWholeOnceASecond)
{
    displayPort_t *dp = init();
    const auto s = busy(40);
    int clears = 0;

    frame(dp, s);
    for (int n = 0; n < 50; n++) {                      // 4 s
        frame(dp, s);
        clears += count(MSP_DP_CLEAR_SCREEN);
        if (count(MSP_DP_CLEAR_SCREEN)) {
            EXPECT_EQ(40u, writes().size());
            EXPECT_EQ(MSP_DP_CLEAR_SCREEN, packets.front().subcmd);
            EXPECT_EQ(MSP_DP_DRAW_SCREEN, packets.back().subcmd);
        } else {
            EXPECT_TRUE(packets.empty());
        }
        EXPECT_EQ(drawn(s), goggles.shown);
    }
    EXPECT_EQ(3, clears);                               // The first frame of each second at 80 ms a frame
}

TEST(DisplayPortMspTest, TheGogglesGetTheScreenBackWithinASecond)
{
    displayPort_t *dp = init();
    const auto s = busy(60);

    frame(dp, s);
    gogglesOff();                                       // The goggles restart, the flight controller is not told
    goggles.shown = blank();
    goggles.blind = true;
    int frames = 0;
    while (goggles.shown != drawn(s) && frames < 100) {
        frame(dp, s);
        frames++;
    }
    EXPECT_LE(frames, 13);                              // 80 ms each
    EXPECT_EQ(drawn(s), goggles.shown);
}

TEST(DisplayPortMspTest, WhatTheFrameLeavesOutIsBlanked)
{
    displayPort_t *dp = init();

    frame(dp, { at(3, 2, "ABC"), at(8, 0, "XY"), at(9, 0, "12345") });
    frame(dp, { at(3, 2, "ABC"), at(9, 0, "12") });
    EXPECT_EQ(drawn({ at(3, 2, "ABC"), at(9, 0, "12") }), goggles.shown);
    EXPECT_EQ(0, count(MSP_DP_CLEAR_SCREEN));
}

TEST(DisplayPortMspTest, SpacesAreNotSentToBlankCells)
{
    displayPort_t *dp = init();

    frame(dp, { at(3, 2, "  AB  ") });
    auto w = writes();
    ASSERT_EQ(1u, w.size());
    EXPECT_EQ(4, w[0].col);
    EXPECT_EQ("AB", w[0].text);

    frame(dp, { at(3, 2, "ABCD  ") });
    EXPECT_EQ(drawn({ at(3, 2, "ABCD") }), goggles.shown);
    frame(dp, { at(3, 2, "      ") });
    EXPECT_EQ(blank(), goggles.shown);
}

TEST(DisplayPortMspTest, OverlappingWritesShowAsTheOsdDrawsThem)
{
    displayPort_t *dp = init();
    const std::vector<packet_t> s = { at(4, 0, "ABCDEFGH"), at(4, 3, "xy"), at(5, 0, "LONG TEXT"), at(5, 0, "ab") };

    for (int n = 0; n < 5; n++) {
        frame(dp, s);
        EXPECT_EQ(drawn(s), goggles.shown);
    }
    // The one on top goes: the goggles may keep it for a frame, as what was under it is not kept here
    frame(dp, { at(4, 0, "ABCDEFGH"), at(5, 0, "LONG TEXT") });
    frame(dp, { at(4, 0, "ABCDEFGH"), at(5, 0, "LONG TEXT") });
    EXPECT_EQ(drawn({ at(4, 0, "ABCDEFGH"), at(5, 0, "LONG TEXT") }), goggles.shown);
    frame(dp, s);
    EXPECT_EQ(drawn(s), goggles.shown);
    frame(dp, { at(4, 3, "xy") });                      // The one below goes
    EXPECT_EQ(drawn({ at(4, 3, "xy") }), goggles.shown);
}

TEST(DisplayPortMspTest, AWriteIsSentWhenOnlyItsAttributeChanges)
{
    displayPort_t *dp = init();

    frame(dp, { at(3, 2, "ABC") });
    frame(dp, { at(3, 2, "ABC", DISPLAYPORT_SEVERITY_WARNING) });
    auto w = writes();
    ASSERT_EQ(1u, w.size());
    EXPECT_EQ(DISPLAYPORT_SEVERITY_WARNING, w[0].attr);
}

TEST(DisplayPortMspTest, AWriteThatBlinksIsSentEveryTime)
{
    displayPort_t *dp = init();
    auto s = busy(40);

    s.push_back(at(19, 40, "AB", DISPLAYPORT_SEVERITY_NORMAL | DISPLAYPORT_BLINK));
    frame(dp, s);
    for (int n = 0; n < 3; n++) {
        frame(dp, s);
        bool found = false;
        for (const auto &w : writes()) {
            found |= w.text == "AB" && w.attr == DISPLAYPORT_MSP_ATTR_BLINK;
        }
        EXPECT_TRUE(found);
    }
}

TEST(DisplayPortMspTest, ALongWriteIsSplit)
{
    displayPort_t *dp = init();
    const std::string s = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789ABCD";

    frame(dp, { at(2, 0, s) });
    auto w = writes();
    ASSERT_EQ(2u, w.size());
    EXPECT_EQ(s.substr(0, 30), w[0].text);
    EXPECT_EQ(s.substr(30), w[1].text);
    EXPECT_EQ(30, w[1].col);
}

TEST(DisplayPortMspTest, UpdatesInPlaceKeepWhatTheyDoNotWrite)
{
    displayPort_t *dp = init();

    // The CMS clears for a page and then writes over parts of it
    packets.clear();
    displayClearScreen(dp, DISPLAY_CLEAR_WAIT);
    displayWrite(dp, 2, 3, DISPLAYPORT_SEVERITY_NORMAL, "MENU");
    displayWrite(dp, 2, 4, DISPLAYPORT_SEVERITY_NORMAL, "ITEM 1");
    displayDrawScreen(dp);
    EXPECT_EQ(1, count(MSP_DP_CLEAR_SCREEN));

    packets.clear();
    displayWrite(dp, 7, 4, DISPLAYPORT_SEVERITY_NORMAL, "2");
    displayDrawScreen(dp);
    EXPECT_EQ(0, count(MSP_DP_CLEAR_SCREEN));
    ASSERT_EQ(1u, writes().size());
    EXPECT_EQ(drawn({ at(3, 2, "MENU"), at(4, 2, "ITEM 2") }), goggles.shown);
}

TEST(DisplayPortMspTest, AScreenOfItsOwnStartsOnClearGoggles)
{
    displayPort_t *dp = init();

    frame(dp, busy(10));
    packets.clear();
    displayClearScreen(dp, DISPLAY_CLEAR_WAIT);
    displayWrite(dp, 2, 3, DISPLAYPORT_SEVERITY_NORMAL, "STATS");
    displayRedraw(dp);
    EXPECT_EQ(3u, packets.size());
    EXPECT_EQ(drawn({ at(3, 2, "STATS") }), goggles.shown);
}

TEST(DisplayPortMspTest, AWriteWithNoRoomIsSentInTheNextFrame)
{
    displayPort_t *dp = init();

    frame(dp, { at(3, 0, "ABC"), at(5, 0, "DEF") });
    packets.clear();
    displayClearScreen(dp, DISPLAY_CLEAR_NONE);
    txFree = 12;
    displayWrite(dp, 0, 3, DISPLAYPORT_SEVERITY_NORMAL, "ABCDEFGHIJ");     // 20 bytes
    txFree = UINT32_MAX;
    displayWrite(dp, 0, 5, DISPLAYPORT_SEVERITY_NORMAL, "DEF");
    displayDrawScreen(dp);
    EXPECT_EQ(drawn({ at(3, 0, "ABC"), at(5, 0, "DEF") }), goggles.shown);   // What was there stays

    frame(dp, { at(3, 0, "ABCDEFGHIJ"), at(5, 0, "DEF") });
    EXPECT_EQ(drawn({ at(3, 0, "ABCDEFGHIJ"), at(5, 0, "DEF") }), goggles.shown);
}

TEST(DisplayPortMspTest, TheDrawWaitsForRoomInTheSerialBuffer)
{
    displayPort_t *dp = init();

    frame(dp, { at(3, 0, "ABCDEFGHIJ"), at(5, 0, "KLMNOPQRST") });
    packets.clear();
    displayClearScreen(dp, DISPLAY_CLEAR_NONE);

    txFree = 19;                                        // Blanking 10 cells takes 20 bytes
    EXPECT_TRUE(displayDrawScreen(dp));
    EXPECT_TRUE(packets.empty());

    txFree = 30;                                        // Room for one of them
    EXPECT_TRUE(displayDrawScreen(dp));
    EXPECT_EQ(1u, packets.size());

    txFree = UINT32_MAX;
    EXPECT_FALSE(displayDrawScreen(dp));
    ASSERT_EQ(3u, packets.size());
    EXPECT_EQ(MSP_DP_DRAW_SCREEN, packets[2].subcmd);
    EXPECT_EQ(blank(), goggles.shown);
}

TEST(DisplayPortMspTest, GivesUpWaitingForARoomThatDoesNotCome)
{
    displayPort_t *dp = init();

    frame(dp, { at(3, 0, "ABC") });
    displayClearScreen(dp, DISPLAY_CLEAR_NONE);
    txFree = 0;
    packets.clear();
    EXPECT_TRUE(displayDrawScreen(dp));
    EXPECT_TRUE(packets.empty());
    now += 300;
    EXPECT_FALSE(displayDrawScreen(dp));                // Everything goes, to be dropped by the port
    EXPECT_EQ(1, count(MSP_DP_DRAW_SCREEN));
}

TEST(DisplayPortMspTest, ReleaseClearsTheGogglesInTheNextFrame)
{
    displayPort_t *dp = init();

    frame(dp, { at(3, 2, "ABC") });
    displayRelease(dp);
    frame(dp, { at(3, 2, "ABC") });
    EXPECT_EQ(1, count(MSP_DP_CLEAR_SCREEN));
    EXPECT_EQ(1u, writes().size());
}

// The OSD as it flies: values that change now and then, and change length, an element that comes and
// goes, one that blinks, the artificial horizon moving, overlapping elements
static std::vector<packet_t> flying(uint32_t &seed)
{
    auto rnd = [&seed](int n) { seed = seed * 1103515245 + 12345; return (int)((seed >> 16) % n); };
    static int values[16];
    static int horizon, warning, frame;
    std::vector<packet_t> s;
    char text[16];

    for (int i = 0; i < 16; i++) {
        if (!rnd(4)) {
            values[i] = rnd(i % 3 ? 1000 : 1000000);
        }
        snprintf(text, sizeof(text), "%c%d", 'A' + i, values[i]);
        s.push_back(at(1 + i % 8 + (i / 8) * 10, i % 2 ? 40 : 1, text));
    }
    if (!rnd(20)) {
        warning = rnd(3);
    }
    if (warning) {
        s.push_back(at(17, 20, warning == 1 ? "LOW BATTERY" : "RSSI"));
    }
    s.push_back(at(15, 30, "BLINK", DISPLAYPORT_SEVERITY_NORMAL | DISPLAYPORT_BLINK));
    if (!rnd(3)) {
        horizon = rnd(7);
    }
    for (int c = 0; c < 9; c++) {
        s.push_back(at(6 + (horizon + c * (horizon - 3) / 4 + 9) % 9, 22 + c, "-"));
    }
    s.push_back(at(10, 26, "+"));                       // Crosshairs over the horizon
    s.push_back(at(12, 10, "FRAME TOP"));
    s.push_back(at(12, 12, ++frame % 20 < 10 ? "ab" : "abc"));

    return s;
}

// Cells that more than one write of either frame covers
static screen_t overlaps(const std::vector<packet_t> &a, const std::vector<packet_t> &b)
{
    screen_t over = blank();

    for (const auto *frame : { &a, &b }) {
        std::vector<std::vector<int>> n(ROWS, std::vector<int>(COLS, 0));
        for (const auto &w : *frame) {
            for (size_t i = 0; i < w.text.size() && w.col + i < COLS; i++) {
                if (++n[w.row][w.col + i] > 1) {
                    over[w.row][w.col + i] = '#';
                }
            }
        }
    }

    return over;
}

// Cells the goggles show wrong, but where elements overlap and one of them went: what was under it
// is back a frame later
static int wrongCells(const std::vector<packet_t> &s, const std::vector<packet_t> &last)
{
    const screen_t over = overlaps(s, last);
    const screen_t want = drawn(s);
    int wrong = 0;

    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            wrong += want[r][c] != goggles.shown[r][c] && over[r][c] == ' ';
        }
    }

    return wrong;
}

TEST(DisplayPortMspTest, ShowsWhatTheOsdDraws)
{
    displayPort_t *dp = init();
    uint32_t seed = 1;
    uint32_t total = 0;
    int clears = 0;
    int lagging = 0;
    std::vector<packet_t> last;
    uint32_t whole = 0;

    for (int n = 0; n < 500; n++) {
        const auto s = flying(seed);
        frame(dp, s);
        ASSERT_EQ(0, wrongCells(s, last)) << "frame " << n;
        lagging += drawn(s) != goggles.shown;
        clears += count(MSP_DP_CLEAR_SCREEN);
        whole = n ? whole : bytes;
        total += n ? bytes : 0;
        last = s;
    }
    printf("whole frame %u bytes, then %u a frame, %d frames lagging\n", whole, total / 499, lagging);
    EXPECT_EQ(39, clears);                              // At the start and once a second
    EXPECT_LT(lagging, 50);
    EXPECT_LT(total / 499, whole / 2);
}

// Load: a cell each, plus 0.75 for a cell two columns after the one before it and 1.3 further, in screen
// order. The screen of the OSD here is at 125 to 145 after a clear and builds up as the horizon moves.
TEST(DisplayPortMspTest, TheGogglesAreClearedBeforeTheyWouldTakeNoMore)
{
    displayPort_t *dp = init(200);
    uint32_t seed = 7;
    int clears = 0;
    int behind = 0;
    std::vector<packet_t> last;

    for (int n = 0; n < 500; n++) {
        const auto s = flying(seed);
        frame(dp, s);
        clears += count(MSP_DP_CLEAR_SCREEN);
        behind += wrongCells(s, last) != 0;
        last = s;
    }
    EXPECT_LE(goggles.maxLoad, 200000u);
    EXPECT_GT(clears, 3);
    EXPECT_LE(behind, clears);                          // A frame left behind for each clear at most
}

TEST(DisplayPortMspTest, RewritingCellsDoesNotUseTheBudget)
{
    displayPort_t *dp = init(20);

    int clears = 0;

    frame(dp, { at(2, 4, "ALT     0 M") });
    for (int n = 0; n < 50; n++) {
        char s[16];
        snprintf(s, sizeof(s), "ALT %5d M", n * 37);
        frame(dp, { at(2, 4, s) });
        clears += count(MSP_DP_CLEAR_SCREEN);
        EXPECT_EQ(drawn({ at(2, 4, s) }), goggles.shown);
    }
    EXPECT_EQ(3, clears);                               // Only those once a second
}

TEST(DisplayPortMspTest, AScreenTooBigIsClearedOnlyOnceASecond)
{
    displayPort_t *dp = init(100);
    const auto s = busy(20);                            // 200 cells
    int clears = 0;

    for (int n = 0; n < 125; n++) {                     // 10 s
        frame(dp, s);
        clears += count(MSP_DP_CLEAR_SCREEN);
        EXPECT_LE(goggles.maxLoad, 100000u);
    }
    EXPECT_LE(clears, 11);
}

TEST(DisplayPortMspTest, AScreenTooBigForTheTableIsShown)
{
    displayPort_t *dp = init();
    std::vector<packet_t> s;

    for (int i = 0; i < 120; i++) {
        s.push_back(at(i / 20 * 3, (i % 20) * 2, std::string(1, 'A' + i % 26)));
    }
    int clears = 0;
    for (int n = 0; n < 125; n++) {
        frame(dp, s);
        clears += count(MSP_DP_CLEAR_SCREEN);
        EXPECT_EQ(drawn(s), goggles.shown);
    }
    EXPECT_LE(clears, 11);
}

TEST(DisplayPortMspTest, NoBudgetOnAnotherCanvas)
{
    displayPort_t *dp = init(32);
    osdConfigMutable()->canvas_cols = 50;               // What HDZero announces
    osdConfigMutable()->canvas_rows = 18;

    frame(dp, { at(0, 0, "123456789012345678901234"), at(1, 0, "123456789012345678901234") });
    EXPECT_EQ(2u, writes().size());
}

TEST(DisplayPortMspTest, TheOsdWaitsForRoomOnlyForSoLong)
{
    displayPort_t *dp = init();

    txFree = 63;
    EXPECT_TRUE(displayIsTransferInProgress(dp));
    now += 300;
    EXPECT_FALSE(displayIsTransferInProgress(dp));      // A port that does not drain does not stop the OSD
    txFree = 64;
    EXPECT_FALSE(displayIsTransferInProgress(dp));
    txFree = 0;
    EXPECT_TRUE(displayIsTransferInProgress(dp));       // Waiting again once there was room

    displayPortProfileMspMutable()->sendChanges = false;
    EXPECT_FALSE(displayIsTransferInProgress(dp));      // Switched off, nothing waits
}

TEST(DisplayPortMspTest, SwitchedOffTheOsdCommandsGoThrough)
{
    displayPort_t *dp = init();
    displayPortProfileMspMutable()->sendChanges = false;

    frame(dp, { at(3, 2, "ABC") });
    frame(dp, { at(3, 2, "ABC") });
    EXPECT_EQ(1, count(MSP_DP_CLEAR_SCREEN));
    EXPECT_EQ(1u, writes().size());
    EXPECT_EQ(1, count(MSP_DP_DRAW_SCREEN));
}
