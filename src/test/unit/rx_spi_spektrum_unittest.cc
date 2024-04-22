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

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "platform.h"

    #include "drivers/io.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx_spi.h"

    #include "rx/rx_spi.h"
    #include "rx/cyrf6936_spektrum.h"

    typedef enum {
        DSM2_22     = 0x01,
        DSM2_11     = 0x02,
        DSM2_11_DX8 = 0x12,
        DSMX_22     = 0xA2,
        DSMX_11     = 0xB2,
    } dsm_protocol_e;

    #define IS_DSM2(x) (x == DSM2_22 || x == DSM2_11 || x == DSM2_11_DX8)
    #define IS_DSMX(x) (!IS_DSM2(x))

    typedef enum {
        DSM_RECEIVER_BIND = 0,
        DSM_RECEIVER_BIND2,
        DSM_RECEIVER_SYNC_A,
        DSM_RECEIVER_SYNC_B,
        DSM_RECEIVER_RECV,
    #ifdef USE_RX_SPEKTRUM_TELEMETRY
        DSM_RECEIVER_TLM,
    #endif
    } dsm_receiver_status_e;

    typedef struct dsmReceiver_s {
        dsm_receiver_status_e status;
        dsm_protocol_e protocol;

        uint8_t mfgId[4];

        uint8_t rfChannel;
        uint8_t rfChannelIdx;
        uint8_t rfChannels[23];

        uint8_t sopCol;
        uint8_t dataCol;
        uint16_t crcSeed;

        uint8_t missedPackets;
        uint8_t numChannels;

        bool bound;

        uint32_t timeout;
        uint32_t timeLastPacket;

        uint16_t bindPackets;

    #ifdef USE_RX_SPEKTRUM_TELEMETRY
        uint32_t timeLastTelemetry;
        bool sendTelemetry;
    #endif
    } dsmReceiver_t;

    extern dsmReceiver_t dsmReceiver;
    bool isError = false;

    static const dsmReceiver_t empty = dsmReceiver_t();
    static rxRuntimeState_t config = rxRuntimeState_t();
    static rxSpiExtiConfig_t extiConfig;
    static uint8_t packetLen;
    static uint8_t packet[16];
    static uint16_t rssi = 0;
    static uint8_t cyrfRssi;

    /* DeviationTx code */
    #define CHAN_MAX_VALUE 10000
    #define CHAN_MIN_VALUE -10000
    static void buildDataPacket(bool upper, const int16_t *sticks, uint8_t *pkt)
    {
        const uint8_t chMap7[] = {1, 5, 2, 3, 0, 4, 6};
        const uint8_t chMap12[] = {1, 5, 2, 4, 6, 10, 0xff, 0, 7, 3, 8, 9, 11, 0xff};
        const uint8_t chMap10[] = {1, 5, 2, 3, 4, 6, 8, 1, 5, 2, 3, 0, 7, 9};
        const uint8_t *chMap;

        uint8_t bits = IS_DSMX(dsmReceiver.protocol) ? 11 : 10;
        if (dsmReceiver.numChannels < 8) {
            chMap = chMap7;
        } else if (dsmReceiver.numChannels < 11) {
            chMap = chMap10;
        } else {
            chMap = chMap12;
        }
        uint16_t max = 1 << bits;
        uint16_t pct100 = (uint32_t) max * 100 / 150;
        for (uint32_t i = 0; i < 7; i++) {
            uint8_t idx = chMap[upper * 7 + i];
            int32_t value;
            if ((chMap[upper * 7 + i] == 0xff) || ((dsmReceiver.numChannels > 7) && (chMap[upper * 7 + i] > dsmReceiver.numChannels - 1))) {
                value = 0xffff;
            } else {
                value = (int32_t) sticks[idx] * (pct100 / 2) / CHAN_MAX_VALUE + (max / 2);
                if (value >= max) {
                    value = max - 1;
                } else if (value < 0) {
                    value = 0;
                }
                value = (upper && i == 0 ? 0x8000 : 0) | (chMap[upper * 7 + i] << bits) | value;
            }
            pkt[i * 2] = (value >> 8) & 0xff;
            pkt[i * 2 + 1] = (value >> 0) & 0xff;
        }
    }

    static const rxSpiConfig_t injectedConfig = {
        .extiIoTag = IO_TAG(PA0),
    };
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

//make clean test_rx_spi_spektrum_unittest
TEST(RxSpiSpektrumUnitTest, TestInitUnbound)
{
    dsmReceiver = empty;
    spektrumSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_FALSE(dsmReceiver.bound);
    EXPECT_EQ(DSM_RECEIVER_BIND, dsmReceiver.status);
    EXPECT_EQ(DSM_INITIAL_BIND_CHANNEL, dsmReceiver.rfChannel);
}

TEST(RxSpiSpektrumUnitTest, TestInitBound)
{
    const uint8_t validMfgId[4] = {0xd4, 0x62, 0xd6, 0xad};
    const uint16_t validCrcSeed = (uint16_t) ~((validMfgId[0] << 8) + validMfgId[1]);
    const uint16_t changedSeed = ~validCrcSeed;

    dsmReceiver = empty;
    memcpy(spektrumConfigMutable()->mfgId, validMfgId, 4);
    spektrumConfigMutable()->numChannels = 7;

    spektrumConfigMutable()->protocol = DSMX_11;

    bool result = spektrumSpiInit(&injectedConfig, &config, &extiConfig);

    EXPECT_TRUE(result);
    EXPECT_TRUE(dsmReceiver.bound);
    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(validMfgId[i], dsmReceiver.mfgId[i]);
    }
    EXPECT_EQ(7, dsmReceiver.numChannels);
    EXPECT_EQ(DSMX_11, dsmReceiver.protocol);
    EXPECT_EQ(DSM_RECEIVER_SYNC_A, dsmReceiver.status);
    EXPECT_EQ(changedSeed, dsmReceiver.crcSeed);
    EXPECT_EQ(6, dsmReceiver.sopCol);
    EXPECT_EQ(1, dsmReceiver.dataCol);
    EXPECT_EQ(0, dsmReceiver.rfChannelIdx);
    EXPECT_TRUE(dsmReceiver.rfChannels[22] != 0);
    EXPECT_EQ(dsmReceiver.rfChannels[dsmReceiver.rfChannelIdx], dsmReceiver.rfChannel);

    dsmReceiver = empty;
    spektrumConfigMutable()->protocol = DSM2_11;

    spektrumSpiInit(&injectedConfig, &config, &extiConfig);

    EXPECT_TRUE(dsmReceiver.bound);
    EXPECT_EQ(DSM2_11, dsmReceiver.protocol);
    EXPECT_TRUE(dsmReceiver.rfChannels[22] == 0); //uninitialized for dsm2
    EXPECT_EQ(1, dsmReceiver.rfChannel);
}

TEST(RxSpiSpektrumUnitTest, TestDecodeBindPacket)
{
    const uint8_t bindPacket[] = {0xD7, 0xA1, 0x54, 0xB1, 0xD7, 0xA1, 0x54, 0xB1, 0x06, 0x6A, 0x01, 7, DSMX_22, 0x00, 0x07, 0x84};
    packetLen = sizeof(packet);
    memcpy(packet, bindPacket, packetLen);
    dsmReceiver = empty;
    dsmReceiver.status = DSM_RECEIVER_BIND;

    rx_spi_received_e result = spektrumSpiDataReceived(nullptr);
    EXPECT_EQ(RX_SPI_RECEIVED_BIND, result);
    EXPECT_EQ(7, dsmReceiver.numChannels);
    EXPECT_EQ(DSMX_22, dsmReceiver.protocol);
    EXPECT_EQ(0x4E, dsmReceiver.mfgId[3]);
    EXPECT_EQ(spektrumConfig()->numChannels, dsmReceiver.numChannels);
    EXPECT_EQ(spektrumConfig()->protocol, dsmReceiver.protocol);
    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(spektrumConfig()->mfgId[i], dsmReceiver.mfgId[i]);
    }
}

TEST(RxSpiSpektrumUnitTest, TestReceiverSyncLogic)
{
    const uint8_t mfgId[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    packetLen = 2;

    //DSMX SYNCA -> RECV
    dsmReceiver = empty;
    memcpy(dsmReceiver.mfgId, mfgId, 4);
    dsmReceiver.status = DSM_RECEIVER_SYNC_A;
    dsmReceiver.protocol = DSMX_11;
    dsmReceiver.crcSeed = 12345;
    dsmReceiver.missedPackets = 1;
    dsmReceiver.rfChannel = 10;
    dsmReceiver.rfChannelIdx = 1;
    dsmReceiver.rfChannels[2] = 5;
    packet[0] = mfgId[2];
    packet[1] = mfgId[3];

    spektrumSpiDataReceived(nullptr);
    EXPECT_EQ(DSM_RECEIVER_RECV, dsmReceiver.status);
    EXPECT_EQ(53190, dsmReceiver.crcSeed);
    EXPECT_EQ(0, dsmReceiver.missedPackets);
    EXPECT_EQ(2, dsmReceiver.rfChannelIdx);
    EXPECT_EQ(5, dsmReceiver.rfChannel);

    //DSM2 SYNCA -> SYNCB
    dsmReceiver = empty;
    memcpy(dsmReceiver.mfgId, mfgId, 4);
    dsmReceiver.status = DSM_RECEIVER_SYNC_A;
    dsmReceiver.protocol = DSM2_22;
    dsmReceiver.crcSeed = 12345;
    dsmReceiver.missedPackets = 1;
    dsmReceiver.rfChannel = 10;
    dsmReceiver.rfChannelIdx = 1;
    packet[0] = (~mfgId[2]&0xFF);
    packet[1] = (~mfgId[3]&0xFF);

    spektrumSpiDataReceived(nullptr);
    EXPECT_EQ(DSM_RECEIVER_SYNC_B, dsmReceiver.status);
    EXPECT_EQ(53190, dsmReceiver.crcSeed);
    EXPECT_EQ(dsmReceiver.rfChannel, dsmReceiver.rfChannels[0] + 1);
    EXPECT_EQ(dsmReceiver.rfChannel, dsmReceiver.rfChannels[1] + 1);
    EXPECT_EQ(1, dsmReceiver.missedPackets);
    EXPECT_EQ(1, dsmReceiver.rfChannelIdx);

    //DSM2 SYNCB -> RECV
    dsmReceiver.rfChannel = dsmReceiver.rfChannel+1;
    spektrumSpiDataReceived((uint8_t *) packet);
    EXPECT_EQ(DSM_RECEIVER_RECV, dsmReceiver.status);
    EXPECT_EQ(12345, dsmReceiver.crcSeed);
    EXPECT_EQ(0, dsmReceiver.missedPackets);
    EXPECT_EQ(0, dsmReceiver.rfChannelIdx);
    EXPECT_EQ(12, dsmReceiver.rfChannel);
}

TEST(RxSpiSpektrumUnitTest, TestReceiveData)
{
    const uint8_t mfgId[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    cyrfRssi = 31;
    packetLen = 16;
    dsmReceiver = empty;
    memcpy(dsmReceiver.mfgId, mfgId, 4);
    dsmReceiver.status = DSM_RECEIVER_RECV;
    dsmReceiver.protocol = DSMX_11;
    dsmReceiver.crcSeed = 12345;
    dsmReceiver.numChannels = 14;
    dsmReceiver.missedPackets = 1;
    dsmReceiver.rfChannelIdx = 22;
    dsmReceiver.rfChannels[0] = 5;
    const uint8_t dataPacket[16] = {mfgId[2], mfgId[3], 0x01, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE};
    memcpy(packet, dataPacket, packetLen);

    uint8_t data[16];
    rx_spi_received_e result = spektrumSpiDataReceived((uint8_t *) data);
    EXPECT_EQ(RX_SPI_RECEIVED_DATA, result);
    EXPECT_EQ(0, dsmReceiver.missedPackets);
    EXPECT_EQ(0, dsmReceiver.rfChannelIdx);
    EXPECT_EQ(5, dsmReceiver.rfChannel);
    EXPECT_EQ(53190, dsmReceiver.crcSeed);
    EXPECT_EQ(1023, rssi);
    for (int i = 0; i < 14; i++) {
        EXPECT_EQ(packet[i+2], data[i]);
    }
}

TEST(RxSpiSpektrumUnitTest, TestConvertDataPacket)
{
    dsmReceiver = empty;

    const int16_t sticks[12] = {CHAN_MAX_VALUE, CHAN_MIN_VALUE, 0, -7500, 7500, -5000, 5000, -2500, 2500, 6666, -3333, 250};
    const uint16_t bfSticks[12] = {1841, 1159, 1500, 1244, 1755, 1329, 1670, 1415, 1585, 1727, 1386, 1508};
    const uint8_t testNumChan[3] = {7, 10, 12};

    uint8_t dataPacket[16];
    uint16_t rcData[16];

    for (int i = 0; i < 3; i++) {
        dsmReceiver.numChannels = testNumChan[i];
        memset(rcData, 0, sizeof(rcData));
        memset(dataPacket, 0, sizeof(dataPacket));

        if (dsmReceiver.numChannels > 7) { //we need two packets to update all channels
            buildDataPacket(false, sticks, dataPacket);
            spektrumSpiSetRcDataFromPayload((uint16_t *) rcData, (uint8_t *) dataPacket);
            memset(dataPacket, 0, sizeof(dataPacket));
            buildDataPacket(true, sticks, dataPacket);
            spektrumSpiSetRcDataFromPayload((uint16_t *) rcData, (uint8_t *) dataPacket);
        } else { // we need only one packet
            buildDataPacket(false, sticks, dataPacket);
            spektrumSpiSetRcDataFromPayload((uint16_t *) rcData, (uint8_t *) dataPacket);
        }
        for (int k = 0; k < 16; k++) {
            if (k<dsmReceiver.numChannels) {
                EXPECT_NEAR(bfSticks[k], rcData[k], 1); //conversion error +-1
            } else {
                EXPECT_EQ(0, rcData[k]);
            }
        }
    }
}

// STUBS

extern "C" {

    int16_t *debug;
    uint8_t debugMode;

    rssiSource_e rssiSource;
    void setRssi(uint16_t newRssi, rssiSource_e )
    {
        rssi = newRssi;
    }
    void setRssiDirect(uint16_t , rssiSource_e ) {}

    uint32_t micros(void) { return 0; }
    uint32_t millis(void) { return 0; }

    bool IORead(IO_t ) { return true; }
    IO_t IOGetByTag(ioTag_t ) { return (IO_t)1; }
    void IOHi(IO_t ) {}
    void IOLo(IO_t ) {}
    void writeEEPROM(void) {}

    bool cyrf6936Init(IO_t ) { return true; }
    bool cyrf6936RxFinished (uint32_t *timestamp)
    {
        *timestamp = 0;
        return true;
    }
    void cyrf6936WriteRegister(const uint8_t , const uint8_t ) {}
    uint8_t cyrf6936ReadRegister(const uint8_t reg)
    {
        if (reg == 0x09) {//CYRF6936_RX_COUNT
            return packetLen;
        }
        return 0;
    }
    uint8_t cyrf6936GetRxStatus(void) { return 0; }
    void cyrf6936SetConfigLen(const uint8_t , const uint8_t ) {}
    void cyrf6936SetChannel(const uint8_t ) {}
    void cyrf6936SetMode(const uint8_t , const bool ) {}
    void cyrf6936SetCrcSeed(const uint16_t ) {}
    void cyrf6936SetSopCode(const uint8_t ) {}
    void cyrf6936SetDataCode(const uint8_t ) {}
    void cyrf6936StartRecv(void) {}
    void cyrf6936SendLen(uint8_t *, const uint8_t ) {}
    void cyrf6936RecvLen(uint8_t *data, const uint8_t length)
    {
        if (length == packetLen) {
            memcpy(data, packet, packetLen);
        }
    }
    uint8_t cyrf6936GetRssi(void)
    {
        return cyrfRssi;
    }

    void rxSpiCommonIOInit(const rxSpiConfig_t *) {}
    void rxSpiLedBlinkRxLoss(rx_spi_received_e ) {}
    void rxSpiLedBlinkBind(void) {};
    bool rxSpiCheckBindRequested(bool)
    {
        return false;
    }
    bool rxSpiExtiConfigured(void) { return true; }
}
