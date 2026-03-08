/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "common/bitarray.h"
    #include "common/utils.h"

    #include "config/feature.h"

    #include "drivers/io.h"
    #include "drivers/dshot_command.h"
    #include "drivers/pwm_output.h"
    #include "drivers/sound_beeper.h"
    #include "drivers/system.h"
    #include "drivers/time.h"
    #include "drivers/usb_io.h"

    #include "flight/failsafe.h"
    #include "flight/mixer.h"

    #include "config/config.h"
    #include "fc/core.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "io/statusindicator.h"
    #include "io/vtx_control.h"

#ifdef USE_GPS
    #include "io/gps.h"
#endif

    #include "msp/msp_serial.h"

    #include "pg/beeper.h"

    #include "scheduler/scheduler.h"

    #include "sensors/battery.h"
    #include "sensors/sensors.h"

    #include "io/beeper.h"

    PG_REGISTER_WITH_RESET_TEMPLATE(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 0);
    PG_RESET_TEMPLATE(beeperConfig_t, beeperConfig,
        .beeper_off_flags = 0,
        .dshotBeaconTone = 1,
        .dshotBeaconOffFlags = 0,
    );

    // Simulation variables
    static bool simulatorMspConfiguratorActive = false;
    static bool simulatorBoxBeeperOn = false;
    static bool simulatorBoxBeeperMute = false;
    static bool simulatorFailsafeRxDataReceived = true;
    static bool simulatorMotorsRunning = false;
    static timeUs_t simulatorLastDisarmTimeUs = 0;
    static bool simulatorTryingToArm = false;
    static uint8_t simulatorMotorCount = 4;
    static timeUs_t simulatorCurrentTimeUs = 10000000;  // 10 seconds after boot
    static batteryState_e simulatorBatteryState = BATTERY_OK;
    static int dshotCommandWriteCount = 0;
    static bool beeperOnState = false;
}

#include "gtest/gtest.h"

class BeeperTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all simulation state
        simulatorMspConfiguratorActive = false;
        simulatorBoxBeeperOn = false;
        simulatorBoxBeeperMute = false;
        simulatorFailsafeRxDataReceived = true;
        simulatorMotorsRunning = false;
        simulatorLastDisarmTimeUs = 0;
        simulatorTryingToArm = false;
        simulatorMotorCount = 4;
        simulatorCurrentTimeUs = 10000000;
        simulatorBatteryState = BATTERY_OK;
        dshotCommandWriteCount = 0;
        beeperOnState = false;

        // Reset beeper config
        beeperConfigMutable()->beeper_off_flags = 0;
        beeperConfigMutable()->dshotBeaconTone = 1;
        beeperConfigMutable()->dshotBeaconOffFlags = 0;

        // Ensure beeper is silent
        beeperSilence();
    }
};

// =============================================================================
// Gap 1 Tests: beeper() piezo path — BEEPER_USB with mspSerialIsConfiguratorActive
// =============================================================================

TEST_F(BeeperTest, BeeperUsbFlagOff_ConfiguratorActive_BeeperSounds)
{
    // BEEPER_USB not set in off_flags → beeper should sound regardless of configurator
    beeperConfigMutable()->beeper_off_flags = 0;
    simulatorMspConfiguratorActive = true;

    beeper(BEEPER_RX_SET);
    // After calling beeper(), the beeper should be active (not silenced)
    // We verify indirectly: if beeper() returns without calling beeperSilence(),
    // meaning the entry was accepted. We call beeperUpdate to drive it.
    beeperUpdate(simulatorCurrentTimeUs);
    EXPECT_TRUE(isBeeperOn());
}

TEST_F(BeeperTest, BeeperUsbFlagOn_ConfiguratorNotActive_BeeperSounds)
{
    // BEEPER_USB set in off_flags but configurator NOT active → should sound
    beeperConfigMutable()->beeper_off_flags = BEEPER_GET_FLAG(BEEPER_USB);
    simulatorMspConfiguratorActive = false;

    beeper(BEEPER_RX_SET);
    beeperUpdate(simulatorCurrentTimeUs);
    EXPECT_TRUE(isBeeperOn());
}

TEST_F(BeeperTest, BeeperUsbFlagOn_ConfiguratorActive_BeeperSilent)
{
    // BEEPER_USB set in off_flags AND configurator active → should be silenced
    beeperConfigMutable()->beeper_off_flags = BEEPER_GET_FLAG(BEEPER_USB);
    simulatorMspConfiguratorActive = true;

    beeper(BEEPER_RX_SET);
    EXPECT_FALSE(isBeeperOn());
}

TEST_F(BeeperTest, BeeperUsbFlagOn_ConfiguratorActive_BatteryPresent_BeeperSilent)
{
    // Key regression: BEEPER_USB + configurator active + battery present → must be silent
    // Previously this would have sounded because only BATTERY_NOT_PRESENT was checked
    beeperConfigMutable()->beeper_off_flags = BEEPER_GET_FLAG(BEEPER_USB);
    simulatorMspConfiguratorActive = true;
    simulatorBatteryState = BATTERY_OK;  // battery present!

    beeper(BEEPER_RX_SET);
    EXPECT_FALSE(isBeeperOn());
}

TEST_F(BeeperTest, BeeperMute_SilencesBeeper)
{
    // BOXBEEPERMUTE active → always silent
    simulatorBoxBeeperMute = true;

    beeper(BEEPER_RX_SET);
    EXPECT_FALSE(isBeeperOn());
}

// =============================================================================
// Gap 2 Tests: DShot beacon RX_SET path — BEEPER_USB guard
// =============================================================================

TEST_F(BeeperTest, DshotBeaconRxSet_NoUsbFlag_Sounds)
{
    // AUX switch active, RX healthy, no USB flag → beacon should be requested
    simulatorBoxBeeperOn = true;
    simulatorFailsafeRxDataReceived = true;
    simulatorMotorsRunning = false;
    simulatorLastDisarmTimeUs = 0;  // disarmed long ago
    beeperConfigMutable()->dshotBeaconOffFlags = 0;

    // First trigger the beeper via AUX
    beeper(BEEPER_RX_SET);
    // Use unique timestamp far from any prior lastDshotBeaconCommandTimeUs
    simulatorCurrentTimeUs = 100000000;
    beeperUpdate(simulatorCurrentTimeUs);

    EXPECT_GT(dshotCommandWriteCount, 0);
}

TEST_F(BeeperTest, DshotBeaconRxSet_UsbFlagOn_ConfiguratorActive_Silent)
{
    // AUX switch active, RX healthy, USB flag in dshotBeaconOffFlags,
    // configurator active → beacon must NOT be requested
    simulatorBoxBeeperOn = true;
    simulatorFailsafeRxDataReceived = true;
    simulatorMotorsRunning = false;
    simulatorLastDisarmTimeUs = 0;
    beeperConfigMutable()->dshotBeaconOffFlags = BEEPER_GET_FLAG(BEEPER_USB);

    beeper(BEEPER_RX_SET);
    simulatorCurrentTimeUs = 200000000;
    simulatorMspConfiguratorActive = true;
    beeperUpdate(simulatorCurrentTimeUs);

    EXPECT_EQ(dshotCommandWriteCount, 0);
}

TEST_F(BeeperTest, DshotBeaconRxSet_UsbFlagOn_ConfiguratorNotActive_Sounds)
{
    // AUX switch active, RX healthy, USB flag set but configurator NOT active → beacon sounds
    simulatorBoxBeeperOn = true;
    simulatorFailsafeRxDataReceived = true;
    simulatorMotorsRunning = false;
    simulatorLastDisarmTimeUs = 0;
    beeperConfigMutable()->dshotBeaconOffFlags = BEEPER_GET_FLAG(BEEPER_USB);

    beeper(BEEPER_RX_SET);
    simulatorCurrentTimeUs = 300000000;
    simulatorMspConfiguratorActive = false;
    beeperUpdate(simulatorCurrentTimeUs);

    EXPECT_GT(dshotCommandWriteCount, 0);
}

TEST_F(BeeperTest, DshotBeaconRxLost_ConfiguratorActive_Silent)
{
    // Verify the existing #14869 fix: RX_LOST + configurator active → no DShot beacon
    simulatorMotorsRunning = false;
    simulatorLastDisarmTimeUs = 0;
    beeperConfigMutable()->dshotBeaconOffFlags = 0;
    simulatorMspConfiguratorActive = true;

    beeper(BEEPER_RX_LOST);
    simulatorCurrentTimeUs = 400000000;
    beeperUpdate(simulatorCurrentTimeUs);

    EXPECT_EQ(dshotCommandWriteCount, 0);
}

TEST_F(BeeperTest, DshotBeaconRxLost_ConfiguratorNotActive_Sounds)
{
    // RX_LOST + configurator NOT active → DShot beacon should sound
    simulatorMotorsRunning = false;
    simulatorLastDisarmTimeUs = 0;
    beeperConfigMutable()->dshotBeaconOffFlags = 0;
    simulatorMspConfiguratorActive = false;

    beeper(BEEPER_RX_LOST);
    simulatorCurrentTimeUs = 500000000;
    beeperUpdate(simulatorCurrentTimeUs);

    EXPECT_GT(dshotCommandWriteCount, 0);
}

// =============================================================================
// Extern "C" stubs for unresolved symbols
// =============================================================================
extern "C" {

// -- MSP serial --
bool mspSerialIsConfiguratorActive(void) {
    return simulatorMspConfiguratorActive;
}

// -- RC modes --
bool IS_RC_MODE_ACTIVE(boxId_e boxId) {
    switch (boxId) {
        case BOXBEEPERON:   return simulatorBoxBeeperOn;
        case BOXBEEPERMUTE: return simulatorBoxBeeperMute;
        default:            return false;
    }
}

// -- Failsafe --
bool failsafeIsReceivingRxData(void) {
    return simulatorFailsafeRxDataReceived;
}

// -- Motor / mixer --
bool areMotorsRunning(void) {
    return simulatorMotorsRunning;
}

uint8_t getMotorCount(void) {
    return simulatorMotorCount;
}

// -- DShot --
void dshotCommandWrite(uint8_t index, uint8_t motorCount, uint8_t command, dshotCommandType_e commandType) {
    UNUSED(index);
    UNUSED(motorCount);
    UNUSED(command);
    UNUSED(commandType);
    dshotCommandWriteCount++;
}

// -- Core --
timeUs_t getLastDisarmTimeUs(void) {
    return simulatorLastDisarmTimeUs;
}

bool isTryingToArm(void) {
    return simulatorTryingToArm;
}

// -- Battery --
batteryState_e getBatteryState(void) {
    return simulatorBatteryState;
}

// -- Beeper hardware --
void systemBeep(bool onOff) {
    beeperOnState = onOff;
}

// -- Feature --
bool featureIsEnabled(uint32_t mask) {
    UNUSED(mask);
    return false;
}

// -- Warning LED / status indicator --
void warningLedEnable(void) {}
void warningLedDisable(void) {}
void warningLedRefresh(void) {}

// -- VTX --
void vtxCycleBandOrChannel(const uint8_t bandStep, const uint8_t channelStep) {
    UNUSED(bandStep);
    UNUSED(channelStep);
}
void vtxCyclePower(const uint8_t powerStep) {
    UNUSED(powerStep);
}

// -- Scheduler --
void schedulerIgnoreTaskExecTime(void) {}

// -- Time --
timeUs_t micros(void) {
    return simulatorCurrentTimeUs;
}

// cmpTimeUs and cmp32 are static inline in headers — no stubs needed

// -- IO / sound --
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index) {
    UNUSED(io); UNUSED(owner); UNUSED(index);
}
void IOConfigGPIO(IO_t io, ioConfig_t cfg) {
    UNUSED(io); UNUSED(cfg);
}
IO_t IOGetByTag(ioTag_t tag) {
    UNUSED(tag);
    return IO_NONE;
}

#ifdef USE_OSD
void osdSetVisualBeeperState(bool state) {
    UNUSED(state);
}
#endif

// -- Runtime config --
uint16_t flightModeFlags = 0;
uint8_t stateFlags = 0;

#ifdef USE_GPS
// -- GPS --
gpsSolutionData_t gpsSol;
#endif

} // extern "C"
