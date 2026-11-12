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

/*
 * SCMI agent for the System Manager on the Cortex-M33, which owns the clocks and pads.
 * Calls are polled on A2P channel 0: a 128 byte shared memory buffer in the NXP_SCMI_MU SRAM, doorbell GCR bit 0.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/system.h"

#include "scmi_imx95.h"

#define SCMI_PROTOCOL_SYSTEM        0x12
#define SCMI_PROTOCOL_CLOCK         0x14
#define SCMI_PROTOCOL_SENSOR        0x15
#define SCMI_PROTOCOL_PINCTRL       0x19
#define SCMI_PROTOCOL_MISC          0x84

#define SCMI_CLOCK_RATE_SET         0x5
#define SCMI_CLOCK_RATE_GET         0x6
#define SCMI_CLOCK_CONFIG_SET       0x7
#define SCMI_CLOCK_PARENT_SET       0xd
#define SCMI_CLOCK_PARENT_GET       0xe
#define SCMI_PINCTRL_CONFIG_SET     0x6
#define SCMI_SYSTEM_POWER_STATE_SET 0x3
#define SCMI_MISC_SI_INFO           0xb
#define SCMI_SENSOR_READING_GET     0x6
#define SCMI_SENSOR_CONFIG_SET      0xa

#define SCMI_SYSTEM_FORCEFUL        0
#define SCMI_SYSTEM_WARM_RESET      2

#define SCMI_PINCTRL_TYPE_MUX       192
#define SCMI_PINCTRL_TYPE_CONFIG    193
#define SCMI_PINCTRL_TYPE_DAISY_ID  194
#define SCMI_PINCTRL_TYPE_DAISY_CFG 195

#define SCMI_PROTOCOL_ERROR         (-10)

#define SMT_FREE                    (1u << 0)
#define SMT_ERROR                   (1u << 1)
#define SMT_CRC32                   3

#define IOMUXC_CONFIG_OFFSET        0x204
#define IOMUXC_DAISY_OFFSET         0x408

// Rounds a rate request to the nearest rate the clock can make.
#define SCMI_CLOCK_ROUND_AUTO       (2u << 2)

// About half a second of polling the uncached transport buffer.
#define SCMI_WAIT_LOOPS             10000000
// The same half second for a posted sensor read.
#define SCMI_PENDING_US             500000

typedef struct {
    uint32_t reserved;
    volatile uint32_t channelStatus;
    volatile uint32_t impStatus;
    volatile uint32_t impCrc;
    volatile uint32_t channelFlags;
    volatile uint32_t length;
    volatile uint32_t header;
    volatile uint32_t payload[25];
} scmiSmt_t;

#define SCMI_SMT ((scmiSmt_t *)((uintptr_t)NXP_SCMI_MU + 0x1000))

static uint32_t scmiToken;

// Header of the posted sensor read whose reply is still to be collected, zero for none.
static uint32_t scmiPendingHeader;
static uint32_t scmiPendingSince;

uint32_t scmiCallCount;
uint32_t scmiCallCycles;
uint32_t scmiSensorReplyCount;
uint32_t scmiSensorPostCycles;
uint32_t scmiSensorReplyCycles;

// IEEE CRC-32 over header and payload, required when impStatus is SMT_CRC32.
static uint32_t scmiCrc32(const volatile uint32_t *words, uint32_t bytes)
{
    uint32_t crc = 0xffffffff;
    for (uint32_t i = 0; i < bytes; i++) {
        crc ^= (words[i / 4] >> (8 * (i % 4))) & 0xff;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
        }
    }
    return ~crc;
}

static bool scmiWaitFree(const scmiSmt_t *smt)
{
    for (int i = 0; i < SCMI_WAIT_LOOPS; i++) {
        if (smt->channelStatus & SMT_FREE) {
            return true;
        }
    }
    return false;
}

// Writes a message to the free channel and rings the doorbell; returns its header.
static uint32_t scmiSend(uint32_t protocol, uint32_t message, const uint32_t *tx, unsigned txWords)
{
    scmiSmt_t *smt = SCMI_SMT;

    const uint32_t header = message | (protocol << 10) | ((scmiToken++ & 0x3ff) << 18);
    smt->reserved = 0;
    smt->channelFlags = 0;
    smt->length = 4 * (1 + txWords);
    smt->header = header;
    for (unsigned i = 0; i < txWords; i++) {
        smt->payload[i] = tx[i];
    }
    if (smt->impStatus == SMT_CRC32) {
        smt->impCrc = scmiCrc32(&smt->header, smt->length);
    }
    smt->channelStatus = 0;
    __DSB();
    NXP_SCMI_MU->GCR |= 1;

    return header;
}

// Checks and copies the reply to the message with this header; the channel must be free.
static int32_t scmiReceive(uint32_t header, uint32_t *rx, unsigned rxWords)
{
    const scmiSmt_t *smt = SCMI_SMT;

    if ((smt->channelStatus & SMT_ERROR) || smt->header != header || smt->length < 8
        || smt->length > 4 + sizeof(smt->payload)
        || (smt->impStatus == SMT_CRC32 && smt->impCrc != scmiCrc32(&smt->header, smt->length))) {
        return SCMI_PROTOCOL_ERROR;
    }

    const int32_t status = (int32_t)smt->payload[0];
    if (status != SCMI_SUCCESS) {
        return status;
    }
    if (smt->length < 4 * (2 + rxWords)) {
        return SCMI_PROTOCOL_ERROR;
    }
    for (unsigned i = 0; i < rxWords; i++) {
        rx[i] = smt->payload[1 + i];
    }
    return SCMI_SUCCESS;
}

static int32_t scmiTransfer(uint32_t protocol, uint32_t message, const uint32_t *tx, unsigned txWords,
                            uint32_t *rx, unsigned rxWords)
{
    if (!scmiWaitFree(SCMI_SMT)) {
        return SCMI_TIMEOUT;
    }

    // The message below overwrites the reply to a posted sensor read.
    scmiPendingHeader = 0;
    const uint32_t header = scmiSend(protocol, message, tx, txWords);

    if (!scmiWaitFree(SCMI_SMT)) {
        return SCMI_TIMEOUT;
    }
    return scmiReceive(header, rx, rxWords);
}

static int32_t scmiCall(uint32_t protocol, uint32_t message, const uint32_t *tx, unsigned txWords,
                        uint32_t *rx, unsigned rxWords)
{
    // The transport buffer is not reentrant and a call may poll for half a second.
    if (__get_IPSR() != 0) {
        return SCMI_BUSY;
    }

    const uint32_t start = getCycleCounter();
    const int32_t status = scmiTransfer(protocol, message, tx, txWords, rx, rxWords);
    scmiCallCycles += getCycleCounter() - start;
    scmiCallCount++;

    return status;
}

int32_t scmiSystemReset(void)
{
    const uint32_t tx[] = { SCMI_SYSTEM_FORCEFUL, SCMI_SYSTEM_WARM_RESET };
    return scmiCall(SCMI_PROTOCOL_SYSTEM, SCMI_SYSTEM_POWER_STATE_SET, tx, 2, NULL, 0);
}

int32_t scmiClockEnable(uint32_t clockId)
{
    const uint32_t tx[] = { clockId, 1, 0 };
    return scmiCall(SCMI_PROTOCOL_CLOCK, SCMI_CLOCK_CONFIG_SET, tx, 3, NULL, 0);
}

int32_t scmiClockRateGet(uint32_t clockId, uint64_t *rateHz)
{
    uint32_t rx[2];
    const int32_t status = scmiCall(SCMI_PROTOCOL_CLOCK, SCMI_CLOCK_RATE_GET, &clockId, 1, rx, 2);
    if (status == SCMI_SUCCESS) {
        *rateHz = rx[0] | ((uint64_t)rx[1] << 32);
    }
    return status;
}

int32_t scmiClockRateSet(uint32_t clockId, uint64_t rateHz)
{
    const uint32_t tx[] = { SCMI_CLOCK_ROUND_AUTO, clockId, (uint32_t)rateHz, (uint32_t)(rateHz >> 32) };
    return scmiCall(SCMI_PROTOCOL_CLOCK, SCMI_CLOCK_RATE_SET, tx, 4, NULL, 0);
}

int32_t scmiClockParentSet(uint32_t clockId, uint32_t parentId)
{
    const uint32_t tx[] = { clockId, parentId };
    return scmiCall(SCMI_PROTOCOL_CLOCK, SCMI_CLOCK_PARENT_SET, tx, 2, NULL, 0);
}

int32_t scmiClockParentGet(uint32_t clockId, uint32_t *parentId)
{
    return scmiCall(SCMI_PROTOCOL_CLOCK, SCMI_CLOCK_PARENT_GET, &clockId, 1, parentId, 1);
}

int32_t scmiSiNameGet(char name[SCMI_SI_NAME_LENGTH + 1])
{
    // Device id, silicon revision and part number, then the name.
    uint32_t rx[3 + SCMI_SI_NAME_LENGTH / 4];
    const int32_t status = scmiCall(SCMI_PROTOCOL_MISC, SCMI_MISC_SI_INFO, NULL, 0, rx, sizeof(rx) / sizeof(rx[0]));
    if (status == SCMI_SUCCESS) {
        memcpy(name, &rx[3], SCMI_SI_NAME_LENGTH);
        name[SCMI_SI_NAME_LENGTH] = '\0';
    }
    return status;
}

int32_t scmiSensorEnable(uint32_t sensorId)
{
    const uint32_t tx[] = { sensorId, 1 };
    return scmiCall(SCMI_PROTOCOL_SENSOR, SCMI_SENSOR_CONFIG_SET, tx, 2, NULL, 0);
}

int32_t scmiSensorReadStart(uint32_t sensorId)
{
    if (__get_IPSR() != 0 || !(SCMI_SMT->channelStatus & SMT_FREE)) {
        return SCMI_BUSY;
    }

    const uint32_t start = getCycleCounter();
    const uint32_t tx[] = { sensorId, 0 };
    scmiPendingHeader = scmiSend(SCMI_PROTOCOL_SENSOR, SCMI_SENSOR_READING_GET, tx, 2);
    scmiPendingSince = getCycleCounter();
    scmiSensorPostCycles += scmiPendingSince - start;

    return SCMI_SUCCESS;
}

static bool scmiPendingExpired(void)
{
    return getCycleCounter() - scmiPendingSince >= clockMicrosToCycles(SCMI_PENDING_US);
}

bool scmiSensorReadBusy(void)
{
    return scmiPendingHeader && !(SCMI_SMT->channelStatus & SMT_FREE) && !scmiPendingExpired();
}

int32_t scmiSensorReadResult(int64_t *value)
{
    if (__get_IPSR() != 0) {
        return SCMI_BUSY;
    }
    if (!scmiPendingHeader) {
        return SCMI_NOTHING_PENDING;
    }
    if (!(SCMI_SMT->channelStatus & SMT_FREE)) {
        if (!scmiPendingExpired()) {
            return SCMI_BUSY;
        }
        scmiPendingHeader = 0;
        return SCMI_TIMEOUT;
    }

    const uint32_t start = getCycleCounter();
    uint32_t rx[2];
    const int32_t status = scmiReceive(scmiPendingHeader, rx, 2);
    scmiPendingHeader = 0;
    if (status == SCMI_SUCCESS) {
        *value = (int64_t)(rx[0] | ((uint64_t)rx[1] << 32));
    }
    scmiSensorReplyCycles += getCycleCounter() - start;
    scmiSensorReplyCount++;

    return status;
}

int32_t scmiPadConfigure(uint32_t configRegister, uint32_t configValue)
{
    const uint32_t tx[] = {
        (configRegister - (IOMUXC_BASE + IOMUXC_CONFIG_OFFSET)) / 4, 0, 1 << 2,
        SCMI_PINCTRL_TYPE_CONFIG, configValue,
    };
    return scmiCall(SCMI_PROTOCOL_PINCTRL, SCMI_PINCTRL_CONFIG_SET, tx, 5, NULL, 0);
}

// Mux and daisy go in one PINCTRL_CONFIG_SET on the mux register, the pad config in a second.
int32_t scmiPinConfigure(uint32_t muxRegister, uint32_t muxMode, uint32_t inputRegister,
                         uint32_t inputDaisy, uint32_t configRegister, uint32_t configValue)
{
    uint32_t tx[9] = {
        (muxRegister - IOMUXC_BASE) / 4, 0, 0,
        SCMI_PINCTRL_TYPE_MUX, muxMode,
        SCMI_PINCTRL_TYPE_DAISY_ID, (inputRegister - (IOMUXC_BASE + IOMUXC_DAISY_OFFSET)) / 4,
        SCMI_PINCTRL_TYPE_DAISY_CFG, inputDaisy,
    };
    const unsigned configs = inputRegister ? 3 : 1;
    tx[2] = configs << 2;
    const int32_t status = scmiCall(SCMI_PROTOCOL_PINCTRL, SCMI_PINCTRL_CONFIG_SET, tx, 3 + 2 * configs, NULL, 0);
    if (status != SCMI_SUCCESS) {
        return status;
    }

    return scmiPadConfigure(configRegister, configValue);
}
