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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "drivers/display.h"

    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "osd/osd.h"
    #include "osd/osd_screensaver.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "rx/rx.h"

    PG_REGISTER(osdConfig_t, osdConfig, PG_OSD_CONFIG, 0);

    uint8_t armingFlags;
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState_t rxRuntimeState;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static bool grabbed;
static bool failsafe;
static bool modeOn;
static timeUs_t now;
static timeUs_t osdFrameUs = 83333;
static char screen[OSD_HD_ROWS][OSD_HD_COLS + 1];
static int writes;

static displayPort_t port;

static const timeUs_t S = 1000000;

static void setUpScreensaver(uint16_t timeout)
{
    memset(&port, 0, sizeof(port));
    port.rows = OSD_HD_ROWS;
    port.cols = OSD_HD_COLS;
    osdConfigMutable()->screensaver_timeout = timeout;
    osdConfigMutable()->framerate_hz = 12;
    rxRuntimeState.channelCount = 8;
    for (int i = 0; i < 8; i++) {
        rcData[i] = 1500;
    }
    rcData[2] = 1000;
    grabbed = failsafe = modeOn = false;
    now = 0;
    osdFrameUs = 83333;
    osdConfigMutable()->screensaver_effect = OSD_SCREENSAVER_MATRIX;
    osdConfigMutable()->screensaver_cycle = OSD_SCREENSAVER_CYCLE_DEFAULT_S;
    writes = 0;
    // Start from a woken screensaver that has taken the channels as they are, at time 0
    ENABLE_ARMING_FLAG(ARMED);
    osdScreensaverActive(&port, 0);
    osdScreensaverActive(&port, 0);
    DISABLE_ARMING_FLAG(ARMED);
}

// One frame, drawn in as many calls as it takes; returns the calls
static int drawFrame(bool msp)
{
    int calls = 1;

    now += osdFrameUs;
    while (osdScreensaverDraw(&port, msp, now)) {
        calls++;
    }

    return calls;
}

static int glyphCells(void)
{
    int n = 0;

    for (int r = 0; r < OSD_HD_ROWS; r++) {
        for (int c = 0; c < OSD_HD_COLS; c++) {
            n += screen[r][c] != ' ';
        }
    }

    return n;
}

TEST(OsdScreensaverTest, OffByDefault)
{
    setUpScreensaver(0);
    EXPECT_FALSE(osdScreensaverActive(&port, 3600 * S));
}

TEST(OsdScreensaverTest, StartsAfterTheTimeout)
{
    setUpScreensaver(10);
    EXPECT_FALSE(osdScreensaverActive(&port, 9 * S));
    EXPECT_TRUE(osdScreensaverActive(&port, 11 * S));
}

TEST(OsdScreensaverTest, StickMovementStopsItAndRestartsTheTimer)
{
    setUpScreensaver(10);
    EXPECT_TRUE(osdScreensaverActive(&port, 11 * S));

    rcData[0] = 1600;
    EXPECT_FALSE(osdScreensaverActive(&port, 12 * S));
    EXPECT_FALSE(osdScreensaverActive(&port, 21 * S));
    EXPECT_TRUE(osdScreensaverActive(&port, 23 * S));
}

TEST(OsdScreensaverTest, StaysOnPastTheTimeWrap)
{
    setUpScreensaver(10);
    EXPECT_TRUE(osdScreensaverActive(&port, 11 * S));
    EXPECT_TRUE(osdScreensaverActive(&port, 2200 * S));     // 2^31 us on
    EXPECT_TRUE(osdScreensaverActive(&port, 4300 * S));     // past the wrap of 2^32 us
}

TEST(OsdScreensaverTest, TheModeShowsItAtOnceAndHoldsItThroughMovement)
{
    setUpScreensaver(0);                                    // no timeout at all
    modeOn = true;
    EXPECT_TRUE(osdScreensaverActive(&port, 1 * S));
    rcData[0] = 1700;
    EXPECT_TRUE(osdScreensaverActive(&port, 2 * S));
    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_FALSE(osdScreensaverActive(&port, 3 * S));
    DISABLE_ARMING_FLAG(ARMED);
    EXPECT_TRUE(osdScreensaverActive(&port, 4 * S));
    modeOn = false;
    EXPECT_FALSE(osdScreensaverActive(&port, 5 * S));
}

TEST(OsdScreensaverTest, NoiseIsNotMovement)
{
    setUpScreensaver(10);
    rcData[1] = 1510;
    rcData[3] = 1490;
    EXPECT_TRUE(osdScreensaverActive(&port, 11 * S));
}

TEST(OsdScreensaverTest, ArmingFailsafeAndMenuStopIt)
{
    setUpScreensaver(10);
    EXPECT_TRUE(osdScreensaverActive(&port, 11 * S));
    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_FALSE(osdScreensaverActive(&port, 12 * S));
    DISABLE_ARMING_FLAG(ARMED);
    EXPECT_FALSE(osdScreensaverActive(&port, 13 * S));

    EXPECT_TRUE(osdScreensaverActive(&port, 23 * S));
    failsafe = true;
    EXPECT_FALSE(osdScreensaverActive(&port, 24 * S));
    failsafe = false;

    EXPECT_TRUE(osdScreensaverActive(&port, 35 * S));
    grabbed = true;
    EXPECT_FALSE(osdScreensaverActive(&port, 36 * S));
}

TEST(OsdScreensaverTest, DrawsEveryRowOnceAFrameAndTheRainMoves)
{
    setUpScreensaver(10);
    ASSERT_TRUE(osdScreensaverActive(&port, 11 * S));
    now = 11 * S;

    char before[OSD_HD_ROWS][OSD_HD_COLS + 1];
    int changed = 0;

    for (int f = 0; f < 60; f++) {
        memcpy(before, screen, sizeof(screen));
        writes = 0;
        EXPECT_EQ(4, drawFrame(false));                     // 5 rows a call
        EXPECT_EQ(OSD_HD_ROWS, writes);
        changed += memcmp(before, screen, sizeof(screen)) != 0;
    }
    EXPECT_GT(changed, 50);
    EXPECT_GT(glyphCells(), 100);
}

// New frames a second of an effect when the OSD runs at 30 Hz
static int newFramesASecond(osdScreensaverEffect_e effect)
{
    setUpScreensaver(10);
    osdConfigMutable()->screensaver_effect = effect;
    osdFrameUs = 1000000 / 30;
    EXPECT_TRUE(osdScreensaverActive(&port, 11 * S));
    now = 11 * S;

    char before[OSD_HD_ROWS][OSD_HD_COLS + 1];
    int changed = 0;
    drawFrame(false);
    for (int f = 0; f < 90; f++) {                          // 3 s
        memcpy(before, screen, sizeof(screen));
        writes = 0;
        drawFrame(false);
        EXPECT_EQ(OSD_HD_ROWS, writes);                     // the rows are written every OSD frame all the same
        changed += memcmp(before, screen, sizeof(screen)) != 0;
    }

    return changed / 3;
}

TEST(OsdScreensaverTest, EachEffectKeepsToItsFrameRate)
{
    EXPECT_LE(newFramesASecond(OSD_SCREENSAVER_TUNNEL), 12);
    EXPECT_GE(newFramesASecond(OSD_SCREENSAVER_TUNNEL), 11);
    EXPECT_LE(newFramesASecond(OSD_SCREENSAVER_TERRAIN), 12);
    EXPECT_LE(newFramesASecond(OSD_SCREENSAVER_WAVES), 20);
    EXPECT_GE(newFramesASecond(OSD_SCREENSAVER_WAVES), 18);
}

static bool horizonDashesShown(void)
{
    for (int r = 0; r < OSD_HD_ROWS; r++) {
        for (int c = 0; c < OSD_HD_COLS; c++) {
            if ((uint8_t)screen[r][c] >= 0x80) {
                return true;
            }
        }
    }

    return false;
}

TEST(OsdScreensaverTest, CycleGoesOnToTheNextEffect)
{
    setUpScreensaver(10);
    osdConfigMutable()->screensaver_effect = OSD_SCREENSAVER_CYCLE;
    ASSERT_TRUE(osdScreensaverActive(&port, 11 * S));
    now = 11 * S;

    // The rain first: letters, no dashes of the horizon
    bool dashes = false;
    for (int f = 0; f < 12 * (OSD_SCREENSAVER_CYCLE_DEFAULT_S - 1); f++) {
        drawFrame(false);
        dashes |= horizonDashesShown();
    }
    EXPECT_FALSE(dashes);

    // Then the tunnel, drawn with them
    for (int f = 0; f < 12 * 3; f++) {
        drawFrame(false);
        dashes |= horizonDashesShown();
    }
    EXPECT_TRUE(dashes);
}

TEST(OsdScreensaverTest, LeavesTheDjiWidgetsBlankOnTheMspCanvas)
{
    setUpScreensaver(10);
    ASSERT_TRUE(osdScreensaverActive(&port, 11 * S));
    now = 11 * S;

    bool timers = false, status = false;

    for (int f = 0; f < 200; f++) {
        drawFrame(true);
        for (int c = 44; c <= 49; c++) {
            EXPECT_EQ(' ', screen[5][c]);
            EXPECT_EQ(' ', screen[6][c]);
        }
        for (int c = 35; c <= 48; c++) {
            EXPECT_EQ(' ', screen[18][c]);
        }
    }
    for (int f = 0; f < 200 && !(timers && status); f++) {
        drawFrame(false);
        for (int c = 44; c <= 49; c++) {
            timers |= screen[5][c] != ' ' || screen[6][c] != ' ';
        }
        for (int c = 35; c <= 48; c++) {
            status |= screen[18][c] != ' ';
        }
    }
    EXPECT_TRUE(timers);    // another display draws there
    EXPECT_TRUE(status);
}

// STUBS

extern "C" {
    uint32_t micros(void) { return 12345; }
    bool failsafeIsActive(void) { return failsafe; }
    bool IS_RC_MODE_ACTIVE(boxId_e boxId) { return boxId == BOXSCREENSAVER && modeOn; }
    bool displayIsGrabbed(const displayPort_t *) { return grabbed; }

    int displayWrite(displayPort_t *, uint8_t x, uint8_t y, uint8_t, const char *text)
    {
        writes++;
        if (y < OSD_HD_ROWS) {
            strncpy(&screen[y][x], text, OSD_HD_COLS - x);
            screen[y][OSD_HD_COLS] = '\0';
        }
        return 0;
    }
}
