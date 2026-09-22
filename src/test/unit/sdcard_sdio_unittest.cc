/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <cstring>
#include <limits>

extern "C" {
#include "pg/bus_spi.h"
#include "drivers/sdcard_impl.h"
#include "drivers/sdmmc_sdio.h"
#include "pg/sdio.h"
}

#include "gtest/gtest.h"

extern "C" {

sdcard_t sdcard;
sdioConfig_t sdioConfig_System;
sdioConfig_t sdioConfig_Copy;
sdioPinConfig_t sdioPinConfig_System;
sdioPinConfig_t sdioPinConfig_Copy;
SD_CardInfo_t SD_CardInfo;
SD_CardType_t SD_CardType;

static timeMs_t mockTimeMs;
static SD_Error_t mockInitStatus;
static SD_Error_t mockReadStatus;
static SD_Error_t mockWriteStatus;
static bool mockCardReady;
static unsigned initCalls;
static unsigned readCheckCalls;
static unsigned writeCheckCalls;
static unsigned stateCalls;
static unsigned readCompletesAfterCalls;
static unsigned cardReadyAfterCalls;
static unsigned callbackCalls;
static uint8_t *callbackBuffer;

timeMs_t millis(void)
{
    return mockTimeMs;
}

timeUs_t micros(void)
{
    return mockTimeMs * 1000;
}

bool sdcard_isInserted(void)
{
    return true;
}

bool SD_InitialiseHardware(dmaResource_t *)
{
    return true;
}

SD_Error_t SD_Init(void)
{
    initCalls++;
    return mockInitStatus;
}

bool SD_IsDetected(void)
{
    return true;
}

bool SD_GetState(void)
{
    stateCalls++;
    return mockCardReady || stateCalls > cardReadyAfterCalls;
}

SD_Error_t SD_GetCardInfo(void)
{
    return SD_OK;
}

SD_Error_t SD_ReadBlocks_DMA(uint64_t, uint32_t *, uint32_t, uint32_t)
{
    return SD_OK;
}

SD_Error_t SD_CheckRead(void)
{
    readCheckCalls++;
    return readCheckCalls > readCompletesAfterCalls ? SD_OK : mockReadStatus;
}

SD_Error_t SD_WriteBlocks_DMA(uint64_t, uint32_t *, uint32_t, uint32_t)
{
    return SD_OK;
}

SD_Error_t SD_CheckWrite(void)
{
    writeCheckCalls++;
    return mockWriteStatus;
}

static void operationComplete(sdcardBlockOperation_e, uint32_t, uint8_t *buffer, uint32_t)
{
    callbackCalls++;
    callbackBuffer = buffer;
}

}

class SdcardSdioTest : public ::testing::Test {
protected:
    uint8_t buffer[512];

    void SetUp() override
    {
        std::memset(&sdcard, 0, sizeof(sdcard));
        std::memset(buffer, 0, sizeof(buffer));
        sdcard.enabled = true;
        sdcard.pendingOperation.buffer = buffer;
        sdcard.pendingOperation.blockIndex = 7;
        sdcard.pendingOperation.callback = operationComplete;
        mockTimeMs = 1000;
        mockInitStatus = SD_OK;
        mockReadStatus = SD_OK;
        mockWriteStatus = SD_OK;
        mockCardReady = true;
        initCalls = 0;
        readCheckCalls = 0;
        writeCheckCalls = 0;
        stateCalls = 0;
        readCompletesAfterCalls = std::numeric_limits<unsigned>::max();
        cardReadyAfterCalls = std::numeric_limits<unsigned>::max();
        callbackCalls = 0;
        callbackBuffer = nullptr;
    }
};

TEST_F(SdcardSdioTest, RejectsCompletedReadWhenCardIsNotReady)
{
    sdcard.state = SDCARD_STATE_READING;
    sdcard.operationStartTime = mockTimeMs;
    mockCardReady = false;
    cardReadyAfterCalls = 32;

    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());

    EXPECT_EQ(1u, readCheckCalls);
    EXPECT_EQ(1u, stateCalls);
    EXPECT_EQ(1u, initCalls);
    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(1u, callbackCalls);
    EXPECT_EQ(nullptr, callbackBuffer);
}

TEST_F(SdcardSdioTest, TimesOutBusyReadWithoutRetryingInSamePoll)
{
    sdcard.state = SDCARD_STATE_READING;
    sdcard.operationStartTime = mockTimeMs - SDCARD_TIMEOUT_READ_MSEC - 1;
    mockReadStatus = SD_BUSY;
    readCompletesAfterCalls = 32;

    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());

    EXPECT_EQ(1u, readCheckCalls);
    EXPECT_EQ(1u, initCalls);
    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(1u, callbackCalls);
    EXPECT_EQ(nullptr, callbackBuffer);
}

TEST_F(SdcardSdioTest, LeavesBusyReadPendingBeforeTimeout)
{
    sdcard.state = SDCARD_STATE_READING;
    sdcard.operationStartTime = mockTimeMs;
    mockReadStatus = SD_BUSY;

    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());

    EXPECT_EQ(1u, readCheckCalls);
    EXPECT_EQ(0u, initCalls);
    EXPECT_EQ(SDCARD_STATE_READING, sdcard.state);
    EXPECT_EQ(0u, callbackCalls);
}

TEST_F(SdcardSdioTest, SuccessfulReadCompletesNormally)
{
    sdcard.state = SDCARD_STATE_READING;
    sdcard.operationStartTime = mockTimeMs;

    EXPECT_TRUE(sdcardSdioVTable.sdcard_poll());

    EXPECT_EQ(1u, readCheckCalls);
    EXPECT_EQ(SDCARD_STATE_READY, sdcard.state);
    EXPECT_EQ(1u, callbackCalls);
    EXPECT_EQ(buffer, callbackBuffer);
}

TEST_F(SdcardSdioTest, SuccessfulReinitChangesWriteRecoveryState)
{
    sdcard.state = SDCARD_STATE_WAITING_FOR_WRITE;
    sdcard.operationStartTime = mockTimeMs - SDCARD_TIMEOUT_WRITE_MSEC - 1;
    mockCardReady = false;
    cardReadyAfterCalls = 32;

    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());

    EXPECT_EQ(1u, initCalls);
    EXPECT_EQ(SDCARD_STATE_CARD_INIT_IN_PROGRESS, sdcard.state);
    EXPECT_EQ(2u, stateCalls);
}

TEST_F(SdcardSdioTest, TimesOutBusyWriteAndReportsFailure)
{
    sdcard.state = SDCARD_STATE_SENDING_WRITE;
    sdcard.operationStartTime = mockTimeMs - SDCARD_TIMEOUT_WRITE_MSEC - 1;
    mockWriteStatus = SD_BUSY;

    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());

    EXPECT_EQ(1u, writeCheckCalls);
    EXPECT_EQ(1u, initCalls);
    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(1u, callbackCalls);
    EXPECT_EQ(nullptr, callbackBuffer);
}

TEST_F(SdcardSdioTest, LeavesBusyWritePendingBeforeTimeout)
{
    sdcard.state = SDCARD_STATE_SENDING_WRITE;
    sdcard.operationStartTime = mockTimeMs;
    mockWriteStatus = SD_BUSY;

    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());

    EXPECT_EQ(1u, writeCheckCalls);
    EXPECT_EQ(0u, initCalls);
    EXPECT_EQ(SDCARD_STATE_SENDING_WRITE, sdcard.state);
    EXPECT_EQ(0u, callbackCalls);
}
