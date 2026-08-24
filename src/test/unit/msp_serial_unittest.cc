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

// Regression coverage for the non-MSP byte scanner in mspSerialProcess().
//
// History of the pendingRequest reset logic:
//   #13940  reset moved INSIDE the per-byte loop  -> masked stray-byte fire,
//           but broke '#\r' CLI entry (#14169/#15071)   [shipped 2025.12.2]
//   #15193  reset moved to a per-burst pre-loop guard -> fixed '#\r', but a
//           non-'$' byte no longer cancels a stale BOOTLOADER_ROM/CLI request
//           (#15315 GPS-save config wipe)               [shipped 2025.12.4]
//   #15323  reset on '$' only + per-byte lastActivityMs -> fixes both
//
// StrayRebootChar_BeforeMspFrame_DoesNotReboot and
// StrayHash_BeforeMspFrame_DoesNotEnterCli FAIL on the #15193 logic (current
// master) and PASS once #15323 is applied. The remaining cases guard the
// behaviours that must hold either way.

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "drivers/serial.h"
    #include "drivers/system.h"
    #include "io/serial.h"
    #include "io/serial_feature_map.h"
    #include "cli/cli.h"
    #include "msp/msp.h"
    #include "msp/msp_serial.h"
    #include "pg/msp.h"

    // --- controllable time ---
    static uint32_t fakeMillis = 1000;
    uint32_t millis(void) { return fakeMillis; }

    // --- faked serial RX queue ---
    #define FAKE_RX_CAP 256
    #define SERIAL_PORT_DUMMY_IDENTIFIER  (serialPortIdentifier_e)0x12
    static uint8_t  fakeRxBuf[FAKE_RX_CAP];
    static unsigned fakeRxHead;
    static unsigned fakeRxTail;

    uint32_t serialRxBytesWaiting(const serialPort_t *) { return fakeRxTail - fakeRxHead; }
    uint8_t  serialRead(serialPort_t *) { return fakeRxBuf[fakeRxHead++ % FAKE_RX_CAP]; }

    // --- observable side effects ---
    static int                     rebootCount;
    static bootloaderRequestType_e lastRebootRequest;
    void systemResetToBootloader(bootloaderRequestType_e requestType) {
        rebootCount++;
        lastRebootRequest = requestType;
    }

    static int  cliEnterCount;
    static bool lastCliInteractive;
    void cliEnter(serialPort_t *, bool interactive) {
        cliEnterCount++;
        lastCliInteractive = interactive;
    }
    bool cliProcess(void) { return false; }

    // --- port allocation mocks (drive mspSerialAllocatePorts -> one MSP port) ---
    // Each open hands back a distinct port carrying its identifier, so which
    // UARTs were opened, and in what order, is observable.
    #define FAKE_PORT_CAP 8
    static serialPort_t fakePorts[FAKE_PORT_CAP];
    static unsigned openedCount;

    serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e,
                                 serialReceiveCallbackPtr, void *, uint32_t,
                                 portMode_e, portOptions_e) {
        if (openedCount >= FAKE_PORT_CAP) {
            return NULL;
        }
        serialPort_t *port = &fakePorts[openedCount++];
        port->identifier = identifier;
        return port;
    }
    bool isSerialPortShared(serialPortIdentifier_e, uint16_t, serialPortFunction_e) { return false; }
    serialType_e serialType(serialPortIdentifier_e) { return SERIALTYPE_UART; }

    // Ports the test has declared as carrying an MSP-transport sensor.
    static serialPortIdentifier_e fakeSensorPorts[IMPLIED_MSP_SENSOR_PORT_COUNT];
    static unsigned fakeSensorPortCount;

    uint8_t serialDefaultPortBaud(serialBaudClass_e) { return BAUD_115200; }

    unsigned serialImpliedMspPorts(serialPortIdentifier_e *ports, unsigned maxPorts) {
        unsigned count = 0;
        for (unsigned i = 0; i < fakeSensorPortCount && count < maxPorts; i++) {
            ports[count++] = fakeSensorPorts[i];
        }
        return count;
    }

    uint32_t serialSynthesizeFunctionMask(serialPortIdentifier_e identifier) {
        for (unsigned i = 0; i < fakeSensorPortCount; i++) {
            if (fakeSensorPorts[i] == identifier) {
                return FUNCTION_MSP | FUNCTION_LIDAR;
            }
        }
        return FUNCTION_MSP;
    }

    // --- inert serial/system/msp stubs (off the tested path) ---
    uint32_t serialTxBytesFree(const serialPort_t *) { return FAKE_RX_CAP; }
    void serialWriteBufNoFlush(serialPort_t *, const uint8_t *, int) {}
    bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
    void serialBeginWrite(serialPort_t *) {}
    void serialEndWrite(serialPort_t *) {}
    void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
    void closeSerialPort(serialPort_t *) {}
    mspDescriptor_t mspDescriptorAlloc(void) { return 0; }
    // Never the test port's identifier, so non-MSP bytes are always evaluated.
    serialPortIdentifier_e displayPortMspGetSerial(void) { return (serialPortIdentifier_e)-1; }

    // --- PG backing storage + tables referenced by the unit under test ---
    serialConfig_t serialConfig_System;
    mspConfig_t mspConfig_System;
    const uint32_t baudRates[BAUD_COUNT] = { 0 };

    // --- MSP command/reply sinks ---
    static mspResult_e fakeCmd(mspDescriptor_t, mspPacket_t *, mspPacket_t *, mspPostProcessFnPtr *) {
        return MSP_RESULT_ACK;
    }
    static void fakeReply(mspPacket_t *) {}
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Minimal, well-formed MSP v1 command frame: '$' 'M' '<' size=0 cmd=1 crc=(0^1).
// Consumed end-to-end so the port returns to PORT_IDLE after processing.
static const uint8_t VALID_V1_FRAME[] = { '$', 'M', '<', 0x00, 0x01, 0x01 };

class MspSerialPendingRequestTest : public ::testing::Test {
protected:
    void SetUp() override {
        fakeRxHead = fakeRxTail = 0;
        rebootCount = 0;
        cliEnterCount = 0;
        lastCliInteractive = false;
        lastRebootRequest = BOOTLOADER_REQUEST_ROM;
        fakeMillis = 1000;

        memset(fakePorts, 0, sizeof(fakePorts));
        openedCount = 0;
        fakeSensorPortCount = 0;
        memset(&serialConfig_System, 0, sizeof(serialConfig_System));
        memset(&mspConfig_System, 0, sizeof(mspConfig_System));
        for (unsigned slot = 0; slot < MAX_MSP_PORT_COUNT; slot++) {
            mspConfig_System.msp_uart[slot] = SERIAL_PORT_NONE;
        }
        mspConfig_System.msp_uart[0] = SERIAL_PORT_DUMMY_IDENTIFIER;
        serialConfig_System.reboot_character = 'R';

        // memset all ports + re-allocate one from the mocked config above.
        mspSerialInit();
    }

    static void feed(const uint8_t *bytes, size_t len) {
        for (size_t i = 0; i < len; i++) {
            fakeRxBuf[fakeRxTail++ % FAKE_RX_CAP] = bytes[i];
        }
    }
    static void feedStr(const char *s) { feed(reinterpret_cast<const uint8_t *>(s), strlen(s)); }
    static void feedByte(uint8_t b) { feed(&b, 1); }

    static void process() { mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA, fakeCmd, fakeReply); }
    static void advanceMs(uint32_t ms) { fakeMillis += ms; }
};

// Preserved behaviour (the #15193 fix): '#' followed by CR must still enter the
// CLI — the trailing '\r' must not wipe MSP_PENDING_CLI before the guard fires.
TEST_F(MspSerialPendingRequestTest, CliEntry_HashThenCR_StillEntersCli)
{
    feedStr("#\r");
    process();                          // pending CLI set; 100ms guard not yet met
    EXPECT_EQ(0, cliEnterCount);

    advanceMs(150);
    process();                          // guard elapsed, no new bytes -> enter CLI
    EXPECT_EQ(1, cliEnterCount);
    EXPECT_TRUE(lastCliInteractive);
}

// Core fix (#15323): a stray reboot character immediately ahead of a real MSP
// frame must be cancelled by the '$', so no reboot fires afterwards. On the
// #15193 per-burst logic the request survives the frame and reboots once the
// port goes quiet — the GPS-save config-wipe hazard from #15315.
TEST_F(MspSerialPendingRequestTest, StrayRebootChar_BeforeMspFrame_DoesNotReboot)
{
    feedByte('R');
    feed(VALID_V1_FRAME, sizeof(VALID_V1_FRAME));
    process();                          // 'R' then a full frame; port back to idle

    advanceMs(150);
    process();                          // quiet period after the frame
    EXPECT_EQ(0, rebootCount);
}

// Same hazard for the CLI character.
TEST_F(MspSerialPendingRequestTest, StrayHash_BeforeMspFrame_DoesNotEnterCli)
{
    feedByte('#');
    feed(VALID_V1_FRAME, sizeof(VALID_V1_FRAME));
    process();

    advanceMs(150);
    process();
    EXPECT_EQ(0, cliEnterCount);
}

// Preserved behaviour: a deliberate, standalone reboot character followed by a
// quiet period still reboots to the ROM bootloader after the 100ms guard.
TEST_F(MspSerialPendingRequestTest, LoneRebootChar_ThenQuiet_DoesReboot)
{
    feedByte('R');
    process();
    EXPECT_EQ(0, rebootCount);          // guard not yet elapsed

    advanceMs(150);
    process();
    EXPECT_EQ(1, rebootCount);
    EXPECT_EQ(BOOTLOADER_REQUEST_ROM, lastRebootRequest);
}

// Preserved behaviour: a lone '#' followed by a quiet period enters the CLI.
TEST_F(MspSerialPendingRequestTest, LoneHash_ThenQuiet_EntersCli)
{
    feedByte('#');
    process();
    EXPECT_EQ(0, cliEnterCount);

    advanceMs(150);
    process();
    EXPECT_EQ(1, cliEnterCount);
    EXPECT_TRUE(lastCliInteractive);
}

// Sanity: a clean MSP frame produces no reboot and no CLI entry.
TEST_F(MspSerialPendingRequestTest, CleanMspFrame_NoSideEffects)
{
    feed(VALID_V1_FRAME, sizeof(VALID_V1_FRAME));
    process();
    advanceMs(150);
    process();
    EXPECT_EQ(0, rebootCount);
    EXPECT_EQ(0, cliEnterCount);
}

#if IMPLIED_MSP_SENSOR_PORT_COUNT > 0

#define SENSOR_PORT_IDENTIFIER  (serialPortIdentifier_e)0x21
#define SENSOR_PORT_IDENTIFIER2 (serialPortIdentifier_e)0x22

class MspSerialAllocationTest : public ::testing::Test {
protected:
    void SetUp() override {
        fakeRxHead = fakeRxTail = 0;
        fakeMillis = 1000;

        memset(fakePorts, 0, sizeof(fakePorts));
        openedCount = 0;
        fakeSensorPortCount = 0;

        memset(&mspConfig_System, 0, sizeof(mspConfig_System));
        for (unsigned slot = 0; slot < MAX_MSP_PORT_COUNT; slot++) {
            mspConfig_System.msp_uart[slot] = SERIAL_PORT_NONE;
        }
    }

    static void declareSensorPort(serialPortIdentifier_e identifier) {
        fakeSensorPorts[fakeSensorPortCount++] = identifier;
    }

    static void feedFrame() {
        for (size_t i = 0; i < sizeof(VALID_V1_FRAME); i++) {
            fakeRxBuf[fakeRxTail++ % FAKE_RX_CAP] = VALID_V1_FRAME[i];
        }
        mspSerialProcess(MSP_EVALUATE_NON_MSP_DATA, fakeCmd, fakeReply);
    }

    static bool wasOpened(serialPortIdentifier_e identifier) {
        for (unsigned i = 0; i < openedCount; i++) {
            if (fakePorts[i].identifier == identifier) {
                return true;
            }
        }
        return false;
    }
};

// An MSP-transport sensor is heard because a port is opened on the UART it
// declares, without the user ever assigning MSP there.
TEST_F(MspSerialAllocationTest, DeclaredSensorPortGetsAnMspPort)
{
    mspConfig_System.msp_uart[0] = SERIAL_PORT_DUMMY_IDENTIFIER;
    declareSensorPort(SENSOR_PORT_IDENTIFIER);

    mspSerialInit();

    EXPECT_EQ(2u, openedCount);
    EXPECT_TRUE(wasOpened(SERIAL_PORT_DUMMY_IDENTIFIER));
    EXPECT_TRUE(wasOpened(SENSOR_PORT_IDENTIFIER));
}

// The user is free to put MSP on the sensor port as well; it is one port either
// way, and openSerialPort() would refuse the second claim.
TEST_F(MspSerialAllocationTest, SensorPortAlreadyCarryingMspIsOpenedOnce)
{
    mspConfig_System.msp_uart[0] = SENSOR_PORT_IDENTIFIER;
    declareSensorPort(SENSOR_PORT_IDENTIFIER);

    mspSerialInit();

    EXPECT_EQ(1u, openedCount);
    EXPECT_TRUE(wasOpened(SENSOR_PORT_IDENTIFIER));
}

// The implied ports have their own budget, so declaring a sensor cannot cost
// the user a configurator link even with every msp_uart[] slot spoken for.
TEST_F(MspSerialAllocationTest, ImpliedPortsDoNotSpendMspSlots)
{
    for (unsigned slot = 0; slot < MAX_MSP_PORT_COUNT; slot++) {
        mspConfig_System.msp_uart[slot] = (serialPortIdentifier_e)(0x30 + slot);
    }
    declareSensorPort(SENSOR_PORT_IDENTIFIER);

    mspSerialInit();

    EXPECT_EQ(MAX_MSP_PORT_COUNT + 1u, openedCount);
    for (unsigned slot = 0; slot < MAX_MSP_PORT_COUNT; slot++) {
        EXPECT_TRUE(wasOpened((serialPortIdentifier_e)(0x30 + slot)));
    }
    EXPECT_TRUE(wasOpened(SENSOR_PORT_IDENTIFIER));
}

#if IMPLIED_MSP_SENSOR_PORT_COUNT > 1
// Two modules on separate UARTs need a port each.
TEST_F(MspSerialAllocationTest, SeparateSensorPortsEachGetAPort)
{
    declareSensorPort(SENSOR_PORT_IDENTIFIER);
    declareSensorPort(SENSOR_PORT_IDENTIFIER2);

    mspSerialInit();

    EXPECT_EQ(2u, openedCount);
    EXPECT_TRUE(wasOpened(SENSOR_PORT_IDENTIFIER));
    EXPECT_TRUE(wasOpened(SENSOR_PORT_IDENTIFIER2));
}
#endif

// An MT module streams for the whole flight, so its traffic must not read as a
// configurator being attached - failsafe consumes that.
TEST_F(MspSerialAllocationTest, SensorTrafficIsNotAConfigurator)
{
    declareSensorPort(SENSOR_PORT_IDENTIFIER);
    mspSerialInit();
    ASSERT_EQ(1u, openedCount);

    feedFrame();

    EXPECT_FALSE(mspSerialIsConfiguratorActive());
}

// The same traffic on an ordinary MSP port does.
TEST_F(MspSerialAllocationTest, OrdinaryMspTrafficIsAConfigurator)
{
    mspConfig_System.msp_uart[0] = SERIAL_PORT_DUMMY_IDENTIFIER;
    mspSerialInit();
    ASSERT_EQ(1u, openedCount);

    feedFrame();

    EXPECT_TRUE(mspSerialIsConfiguratorActive());
}

#endif // IMPLIED_MSP_SENSOR_PORT_COUNT
