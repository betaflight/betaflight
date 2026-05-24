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
#include <string.h>

extern "C" {

#include "platform.h"
#include "target.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_ist8310.h"
#include "drivers/bus.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// ---------------------------------------------------------------------------
// Register addresses and bit values mirrored from compass_ist8310.c (which
// keeps these as file-local #defines). Naming them here makes assertion
// failures self-describing instead of "expected 0x02, got 0x03".
// ---------------------------------------------------------------------------
namespace {
constexpr uint8_t IST8310_REG_STAT1           = 0x02;
constexpr uint8_t IST8310_REG_DATA            = 0x03;
constexpr uint8_t IST8310_REG_CNTRL1          = 0x0A;
constexpr uint8_t IST8310_REG_WAI_VALID       = 0x10;
constexpr uint8_t IST8310_DRDY_MASK           = 0x01;
constexpr uint8_t IST8310_ODR_SINGLE          = 0x01;
constexpr uint8_t IST8310_I2C_ADDRESS_DEFAULT = 0x0E;
constexpr uint8_t IST8310_DATA_BUF_LEN        = 6;
} // namespace

// ---------------------------------------------------------------------------
// Mock infrastructure for bus operations
// ---------------------------------------------------------------------------

// Controls what busReadRegisterBuffer returns (used by deviceDetect)
static bool mock_busReadRegisterBuffer_ret = true;
static uint8_t mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;

// Controls what busReadRegisterBufferStart returns and writes into the buffer
static bool mock_busReadRegisterBufferStart_ret = true;
static uint8_t mock_busReadRegisterBufferStart_reg = 0;
static uint8_t mock_busReadRegisterBufferStart_buf[8];
static uint8_t mock_busReadRegisterBufferStart_len = 0;

// Controls what busWriteRegisterStart returns
static bool mock_busWriteRegisterStart_ret = true;
static uint8_t mock_busWriteRegisterStart_lastReg = 0;
static uint8_t mock_busWriteRegisterStart_lastVal = 0;

// Call counters
static int busReadRegisterBufferStart_callCount = 0;
static int busWriteRegisterStart_callCount = 0;

static void resetMocks(void)
{
    mock_busReadRegisterBuffer_ret = true;
    mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;
    mock_busReadRegisterBufferStart_ret = true;
    mock_busReadRegisterBufferStart_reg = 0;
    memset(mock_busReadRegisterBufferStart_buf, 0, sizeof(mock_busReadRegisterBufferStart_buf));
    mock_busReadRegisterBufferStart_len = 0;
    mock_busWriteRegisterStart_ret = true;
    mock_busWriteRegisterStart_lastReg = 0;
    mock_busWriteRegisterStart_lastVal = 0;
    busReadRegisterBufferStart_callCount = 0;
    busWriteRegisterStart_callCount = 0;
}

extern "C" {

void delay(uint32_t) {}
bool busBusy(const extDevice_t*, bool*) { return false; }

bool busReadRegisterBuffer(const extDevice_t*, uint8_t, uint8_t *buf, uint8_t len)
{
    if (buf && len > 0) {
        buf[0] = mock_busReadRegisterBuffer_value;
    }
    return mock_busReadRegisterBuffer_ret;
}

bool busReadRegisterBufferStart(const extDevice_t*, uint8_t reg, uint8_t *buf, uint8_t len)
{
    busReadRegisterBufferStart_callCount++;
    mock_busReadRegisterBufferStart_reg = reg;
    mock_busReadRegisterBufferStart_len = len;
    if (buf && len > 0) {
        // Simulate async completion: copy prepared data into caller's buffer
        memcpy(buf, mock_busReadRegisterBufferStart_buf, len);
    }
    return mock_busReadRegisterBufferStart_ret;
}

bool busWriteRegister(const extDevice_t*, uint8_t, uint8_t) { return true; }

bool busWriteRegisterStart(const extDevice_t*, uint8_t reg, uint8_t val)
{
    busWriteRegisterStart_callCount++;
    mock_busWriteRegisterStart_lastReg = reg;
    mock_busWriteRegisterStart_lastVal = val;
    return mock_busWriteRegisterStart_ret;
}

void busDeviceRegister(const extDevice_t*) {}

uint16_t spiCalculateDivider() { return 2; }
void spiSetClkDivisor() {}
void ioPreinitByIO() {}
void IOConfigGPIO() {}
void IOHi() {}
void IOInit() {}
void IORelease() {}

} // extern "C"

// ---------------------------------------------------------------------------
// Helpers
//
// ist8310Read uses static local variables for its state machine. State
// persists across calls within the same process. syncToCheckStatusBaseline()
// drives the state machine through a full cycle until it returns true
// (STATE_TRIGGER_MEASUREMENT succeeded), which resets state to
// STATE_CHECK_STATUS with status=0 and retries=0. Each state-machine test
// calls this at the top so tests are order-independent.
// ---------------------------------------------------------------------------

static void syncToCheckStatusBaseline(magDev_t *mag, int16_t *magData)
{
    for (int i = 0; i < 20; i++) {
        resetMocks();
        mock_busReadRegisterBufferStart_ret = true;
        mock_busReadRegisterBufferStart_buf[0] = IST8310_DRDY_MASK; // DRDY set when polled
        mock_busWriteRegisterStart_ret = true;
        if (mag->read(mag, magData)) {
            resetMocks();
            return;
        }
    }
    FAIL() << "Unable to synchronize IST8310 state machine to CHECK_STATUS baseline";
}

// ---------------------------------------------------------------------------
// Test: ist8310Detect sets up function pointers and default I2C address
// ---------------------------------------------------------------------------
TEST(Ist8310DetectTest, DetectSuccess)
{
    magDev_t mag;
    memset(&mag, 0, sizeof(mag));
    busDevice_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.busType = BUS_TYPE_I2C;
    mag.dev.bus = &bus;
    mag.dev.busType_u.i2c.address = 0;

    mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;
    mock_busReadRegisterBuffer_ret = true;

    bool result = ist8310Detect(&mag);
    EXPECT_TRUE(result);
    EXPECT_NE(nullptr, mag.init);
    EXPECT_NE(nullptr, mag.read);
    EXPECT_EQ(IST8310_I2C_ADDRESS_DEFAULT, mag.dev.busType_u.i2c.address);
}

TEST(Ist8310DetectTest, DetectFailWrongWAI)
{
    magDev_t mag;
    memset(&mag, 0, sizeof(mag));
    busDevice_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.busType = BUS_TYPE_I2C;
    mag.dev.bus = &bus;

    mock_busReadRegisterBuffer_value = 0xFF; // wrong WAI
    mock_busReadRegisterBuffer_ret = true;

    bool result = ist8310Detect(&mag);
    EXPECT_FALSE(result);
}

TEST(Ist8310DetectTest, DetectFailBusError)
{
    magDev_t mag;
    memset(&mag, 0, sizeof(mag));
    busDevice_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.busType = BUS_TYPE_I2C;
    mag.dev.bus = &bus;

    mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;
    mock_busReadRegisterBuffer_ret = false; // bus error

    bool result = ist8310Detect(&mag);
    EXPECT_FALSE(result);
}

TEST(Ist8310DetectTest, DetectPreservesExistingAddress)
{
    magDev_t mag;
    memset(&mag, 0, sizeof(mag));
    busDevice_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.busType = BUS_TYPE_I2C;
    mag.dev.bus = &bus;
    mag.dev.busType_u.i2c.address = 0x0D; // custom address

    mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;
    mock_busReadRegisterBuffer_ret = true;

    ist8310Detect(&mag);
    EXPECT_EQ(0x0D, mag.dev.busType_u.i2c.address);
}

// ---------------------------------------------------------------------------
// Full state machine integration test
//
// Because ist8310Read uses static locals, we exercise the entire state machine
// in a single test, calling read repeatedly and verifying transitions.
//
// State machine flow:
//   STATE_CHECK_STATUS (status=0) -> polls STAT1 -> returns false
//   STATE_CHECK_STATUS (status has DRDY) -> starts data read -> STATE_READ_DATA, returns false
//   STATE_READ_DATA -> parses mag data -> STATE_TRIGGER_MEASUREMENT, returns false
//   STATE_TRIGGER_MEASUREMENT -> writes CNTRL1 -> STATE_CHECK_STATUS, returns true
// ---------------------------------------------------------------------------
TEST(Ist8310StateMachineTest, FullCycleDRDYReady)
{
    magDev_t mag;
    memset(&mag, 0, sizeof(mag));
    busDevice_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.busType = BUS_TYPE_I2C;
    mag.dev.bus = &bus;
    mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;
    bool detected = ist8310Detect(&mag);
    ASSERT_TRUE(detected);
    ASSERT_NE(nullptr, mag.read);

    int16_t magData[3] = {0, 0, 0};
    syncToCheckStatusBaseline(&mag, magData);

    // --- Call 1: STATE_CHECK_STATUS, status=0, retries=0 ---
    // Should poll STAT1 register. We simulate DRDY not set yet.
    mock_busReadRegisterBufferStart_buf[0] = 0x00; // DRDY not set
    mock_busReadRegisterBufferStart_ret = true;

    bool result = mag.read(&mag, magData);
    EXPECT_FALSE(result);
    EXPECT_EQ(1, busReadRegisterBufferStart_callCount);
    EXPECT_EQ(IST8310_REG_STAT1, mock_busReadRegisterBufferStart_reg);

    // --- Call 2: STATE_CHECK_STATUS, status now has DRDY set ---
    // The mock wrote 0x00 into status in call 1. But our mock copies the
    // prepared buffer immediately, so status was set to 0x00.
    // We now prepare DRDY=1 for the next poll.
    resetMocks();
    mock_busReadRegisterBufferStart_buf[0] = IST8310_DRDY_MASK; // DRDY set
    mock_busReadRegisterBufferStart_ret = true;

    result = mag.read(&mag, magData);
    EXPECT_FALSE(result);
    // Still in CHECK_STATUS, polled STAT1 again (status was 0 from last call)
    EXPECT_EQ(1, busReadRegisterBufferStart_callCount);

    // --- Call 3: STATE_CHECK_STATUS, status=0x01 (DRDY set from previous read) ---
    // Now status has DRDY. Should start reading data registers.
    resetMocks();
    // Prepare 6 bytes of mag data in the read buffer:
    // X = 0x0100 = 256, Y = 0x0200 = 512, Z = 0x0300 = 768 (little-endian)
    mock_busReadRegisterBufferStart_buf[0] = 0x00; // X low
    mock_busReadRegisterBufferStart_buf[1] = 0x01; // X high
    mock_busReadRegisterBufferStart_buf[2] = 0x00; // Y low
    mock_busReadRegisterBufferStart_buf[3] = 0x02; // Y high
    mock_busReadRegisterBufferStart_buf[4] = 0x00; // Z low
    mock_busReadRegisterBufferStart_buf[5] = 0x03; // Z high
    mock_busReadRegisterBufferStart_ret = true;

    result = mag.read(&mag, magData);
    EXPECT_FALSE(result);
    EXPECT_EQ(IST8310_REG_DATA, mock_busReadRegisterBufferStart_reg);
    EXPECT_EQ(IST8310_DATA_BUF_LEN, mock_busReadRegisterBufferStart_len);

    // --- Call 4: STATE_READ_DATA ---
    // Should parse the buffer and transition to STATE_TRIGGER_MEASUREMENT
    resetMocks();
    result = mag.read(&mag, magData);
    EXPECT_FALSE(result);
    // Verify mag data conversion: raw * LSB2FSV(3)
    // X = 256 * 3 = 768
    EXPECT_EQ(768, magData[0]);
    // Y = -(512) * 3 = -1536 (Y axis inverted)
    EXPECT_EQ(-1536, magData[1]);
    // Z = 768 * 3 = 2304
    EXPECT_EQ(2304, magData[2]);

    // --- Call 5: STATE_TRIGGER_MEASUREMENT ---
    // Should write CNTRL1 register and return true
    resetMocks();
    mock_busWriteRegisterStart_ret = true;

    result = mag.read(&mag, magData);
    EXPECT_TRUE(result);
    EXPECT_EQ(1, busWriteRegisterStart_callCount);
    EXPECT_EQ(IST8310_REG_CNTRL1, mock_busWriteRegisterStart_lastReg);
    EXPECT_EQ(IST8310_ODR_SINGLE, mock_busWriteRegisterStart_lastVal);
}

// ---------------------------------------------------------------------------
// Test: DRDY timeout triggers re-measurement
// ---------------------------------------------------------------------------
TEST(Ist8310StateMachineTest, DRDYTimeoutRetriggersMeasurement)
{
    magDev_t mag;
    memset(&mag, 0, sizeof(mag));
    busDevice_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.busType = BUS_TYPE_I2C;
    mag.dev.bus = &bus;
    mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;
    bool detected = ist8310Detect(&mag);
    ASSERT_TRUE(detected);

    int16_t magData[3] = {0, 0, 0};
    syncToCheckStatusBaseline(&mag, magData);

    // Poll 15 times (retries 0..14, timeout at retries >= 15 = 16th call)
    for (int i = 0; i < 15; i++) {
        resetMocks();
        mock_busReadRegisterBufferStart_buf[0] = 0x00; // DRDY never set
        mock_busReadRegisterBufferStart_ret = true;

        bool result = mag.read(&mag, magData);
        EXPECT_FALSE(result);
    }

    // 16th call: retries (now 15) >= IST8310_DRDY_MAX_RETRIES (15) => timeout
    // Should transition to STATE_TRIGGER_MEASUREMENT
    resetMocks();
    mock_busReadRegisterBufferStart_buf[0] = 0x00;

    bool result = mag.read(&mag, magData);
    EXPECT_FALSE(result);
    // Now in STATE_TRIGGER_MEASUREMENT

    // Next call should write the trigger register
    resetMocks();
    mock_busWriteRegisterStart_ret = true;

    result = mag.read(&mag, magData);
    EXPECT_TRUE(result);
    EXPECT_EQ(IST8310_REG_CNTRL1, mock_busWriteRegisterStart_lastReg);
    EXPECT_EQ(IST8310_ODR_SINGLE, mock_busWriteRegisterStart_lastVal);
}

// ---------------------------------------------------------------------------
// Test: a STAT1 poll start that fails (bus busy) must NOT consume a retry,
// otherwise a stuck bus fast-paths the driver into a bogus timeout without
// ever talking to the chip.
// ---------------------------------------------------------------------------
TEST(Ist8310StateMachineTest, FailedPollStartDoesNotConsumeRetry)
{
    magDev_t mag;
    memset(&mag, 0, sizeof(mag));
    busDevice_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.busType = BUS_TYPE_I2C;
    mag.dev.bus = &bus;
    mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;
    ASSERT_TRUE(ist8310Detect(&mag));

    int16_t magData[3] = {0, 0, 0};
    syncToCheckStatusBaseline(&mag, magData);

    // Bus is stuck — every poll start fails. Call the read path well past
    // IST8310_DRDY_MAX_RETRIES. None of these may transition to TRIGGER.
    for (int i = 0; i < 30; i++) {
        resetMocks();
        mock_busReadRegisterBufferStart_ret = false;
        mock_busReadRegisterBufferStart_buf[0] = 0x00;

        bool result = mag.read(&mag, magData);
        EXPECT_FALSE(result);
        EXPECT_EQ(0, busWriteRegisterStart_callCount)
            << "Failed poll starts incorrectly advanced the retry counter "
               "and triggered a measurement re-arm at iteration " << i;
    }

    // Bus recovers and a poll succeeds with DRDY set — should arm a data read
    // on the following call (not a re-trigger, which would prove the retry
    // counter was incorrectly bumped to the limit).
    resetMocks();
    mock_busReadRegisterBufferStart_ret = true;
    mock_busReadRegisterBufferStart_buf[0] = IST8310_DRDY_MASK;
    mag.read(&mag, magData);

    resetMocks();
    mock_busReadRegisterBufferStart_ret = true;
    mag.read(&mag, magData);
    EXPECT_EQ(IST8310_REG_DATA, mock_busReadRegisterBufferStart_reg);
}

// ---------------------------------------------------------------------------
// Test: busWriteRegisterStart failure in TRIGGER state keeps retrying
// ---------------------------------------------------------------------------
TEST(Ist8310StateMachineTest, TriggerWriteFailureRetries)
{
    magDev_t mag;
    memset(&mag, 0, sizeof(mag));
    busDevice_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.busType = BUS_TYPE_I2C;
    mag.dev.bus = &bus;
    mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;
    ASSERT_TRUE(ist8310Detect(&mag));

    int16_t magData[3] = {0, 0, 0};
    syncToCheckStatusBaseline(&mag, magData);

    // Drive state machine to STATE_TRIGGER_MEASUREMENT via DRDY ready path.
    // Poll once to read status (DRDY set)
    resetMocks();
    mock_busReadRegisterBufferStart_buf[0] = IST8310_DRDY_MASK; // DRDY
    mag.read(&mag, magData);

    // Now status=0x01, next call starts data read
    resetMocks();
    mock_busReadRegisterBufferStart_ret = true;
    mag.read(&mag, magData);

    // STATE_READ_DATA - parse data
    resetMocks();
    mag.read(&mag, magData);

    // Now in STATE_TRIGGER_MEASUREMENT. Simulate write failure.
    resetMocks();
    mock_busWriteRegisterStart_ret = false;

    bool result = mag.read(&mag, magData);
    EXPECT_FALSE(result); // write failed, stays in TRIGGER state

    // Try again with success
    resetMocks();
    mock_busWriteRegisterStart_ret = true;

    result = mag.read(&mag, magData);
    EXPECT_TRUE(result); // now succeeds
}

// ---------------------------------------------------------------------------
// Test: Data conversion with negative values
// ---------------------------------------------------------------------------
TEST(Ist8310StateMachineTest, NegativeMagDataConversion)
{
    magDev_t mag;
    memset(&mag, 0, sizeof(mag));
    busDevice_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.busType = BUS_TYPE_I2C;
    mag.dev.bus = &bus;
    mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;
    ASSERT_TRUE(ist8310Detect(&mag));

    int16_t magData[3] = {0, 0, 0};
    syncToCheckStatusBaseline(&mag, magData);

    // Poll status with DRDY set
    resetMocks();
    mock_busReadRegisterBufferStart_buf[0] = IST8310_DRDY_MASK;
    mag.read(&mag, magData);

    // Start data read: X = 0xFFFE = -2, Y = 0xFFFD = -3, Z = 0xFFFC = -4
    resetMocks();
    mock_busReadRegisterBufferStart_buf[0] = 0xFE;
    mock_busReadRegisterBufferStart_buf[1] = 0xFF;
    mock_busReadRegisterBufferStart_buf[2] = 0xFD;
    mock_busReadRegisterBufferStart_buf[3] = 0xFF;
    mock_busReadRegisterBufferStart_buf[4] = 0xFC;
    mock_busReadRegisterBufferStart_buf[5] = 0xFF;
    mag.read(&mag, magData);

    // Parse data in STATE_READ_DATA
    resetMocks();
    mag.read(&mag, magData);

    // X = -2 * 3 = -6
    EXPECT_EQ(-6, magData[0]);
    // Y = -(-3) * 3 = 9 (Y inverted)
    EXPECT_EQ(9, magData[1]);
    // Z = -4 * 3 = -12
    EXPECT_EQ(-12, magData[2]);
}

// ---------------------------------------------------------------------------
// Test: busReadRegisterBufferStart failure in CHECK_STATUS when DRDY is set
// ---------------------------------------------------------------------------
TEST(Ist8310StateMachineTest, DataReadStartFailureStaysInCheckStatus)
{
    magDev_t mag;
    memset(&mag, 0, sizeof(mag));
    busDevice_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.busType = BUS_TYPE_I2C;
    mag.dev.bus = &bus;
    mock_busReadRegisterBuffer_value = IST8310_REG_WAI_VALID;
    ASSERT_TRUE(ist8310Detect(&mag));

    int16_t magData[3] = {0, 0, 0};
    syncToCheckStatusBaseline(&mag, magData);

    // Poll status, get DRDY
    resetMocks();
    mock_busReadRegisterBufferStart_buf[0] = IST8310_DRDY_MASK;
    mag.read(&mag, magData);

    // Next call: status has DRDY, but busReadRegisterBufferStart fails
    resetMocks();
    mock_busReadRegisterBufferStart_ret = false;

    bool result = mag.read(&mag, magData);
    EXPECT_FALSE(result);
    // Should stay in STATE_CHECK_STATUS, will retry on next call

    // Now succeed
    resetMocks();
    mock_busReadRegisterBufferStart_ret = true;
    mock_busReadRegisterBufferStart_buf[0] = 0x01;
    mock_busReadRegisterBufferStart_buf[1] = 0x00;
    mock_busReadRegisterBufferStart_buf[2] = 0x01;
    mock_busReadRegisterBufferStart_buf[3] = 0x00;
    mock_busReadRegisterBufferStart_buf[4] = 0x01;
    mock_busReadRegisterBufferStart_buf[5] = 0x00;

    result = mag.read(&mag, magData);
    EXPECT_FALSE(result);
    // Should have transitioned to STATE_READ_DATA
    EXPECT_EQ(IST8310_REG_DATA, mock_busReadRegisterBufferStart_reg);
}
