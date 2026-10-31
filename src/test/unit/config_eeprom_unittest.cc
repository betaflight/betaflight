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

#include <cstddef>
#include <cstdint>
#include <cstring>

extern "C" {
#include "platform.h"

#include "common/crc.h"

#include "config/config_eeprom.h"

#include "drivers/system.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

extern uint8_t eepromData[EEPROM_SIZE];

typedef struct testVersionedConfig_s {
    uint32_t value;
} testVersionedConfig_t;

typedef struct testNeighborConfig_s {
    uint32_t value;
} testNeighborConfig_t;

PG_DECLARE(testVersionedConfig_t, testVersionedConfig);
PG_DECLARE(testNeighborConfig_t, testNeighborConfig);

PG_REGISTER_WITH_RESET_FN(testVersionedConfig_t, testVersionedConfig, PG_RESERVED_FOR_TESTING_1, 2);
PG_REGISTER_WITH_RESET_FN(testNeighborConfig_t, testNeighborConfig, PG_RESERVED_FOR_TESTING_2, 1);

static void pgResetFn_testVersionedConfig(testVersionedConfig_t *config)
{
    config->value = 0x11223344;
}

static void pgResetFn_testNeighborConfig(testNeighborConfig_t *config)
{
    config->value = 0x55667788;
}

static int failureModeCall = -1;

void failureMode(failureMode_e mode)
{
    failureModeCall = mode;
}
}

#include "gtest/gtest.h"

namespace {

constexpr uint16_t crcStartValue = 0xFFFF;

typedef struct testConfigHeader_s {
    uint8_t eepromConfigVersion;
    uint8_t magicBe;
} PG_PACKED testConfigHeader_t;

typedef struct testConfigRecord_s {
    uint16_t size;
    pgn_t pgn;
    uint8_t version;
    uint8_t flags;
    uint8_t pg[];
} PG_PACKED testConfigRecord_t;

testConfigRecord_t *findStoredRecord(pgn_t pgn)
{
    uint8_t *cursor = eepromData + sizeof(testConfigHeader_t);
    const uint8_t *const end = eepromData + EEPROM_SIZE;

    while (cursor + sizeof(testConfigRecord_t) < end) {
        testConfigRecord_t *record = reinterpret_cast<testConfigRecord_t *>(cursor);
        if (record->size == 0 || record->size < sizeof(testConfigRecord_t) || cursor + record->size >= end) {
            return nullptr;
        }
        if (record->pgn == pgn) {
            return record;
        }
        cursor += record->size;
    }

    return nullptr;
}

uint16_t storedConfigSize()
{
    const uint8_t *cursor = eepromData + sizeof(testConfigHeader_t);
    const uint8_t *const end = eepromData + EEPROM_SIZE;

    while (cursor + sizeof(testConfigRecord_t) < end) {
        const testConfigRecord_t *record = reinterpret_cast<const testConfigRecord_t *>(cursor);
        if (record->size == 0) {
            return static_cast<uint16_t>(cursor - eepromData + sizeof(uint16_t) + sizeof(uint16_t));
        }
        if (record->size < sizeof(testConfigRecord_t) || cursor + record->size >= end) {
            return 0;
        }
        cursor += record->size;
    }

    return 0;
}

void rewriteStoredCrc(uint16_t configSize)
{
    const size_t crcOffset = configSize - sizeof(uint16_t);
    const uint16_t crc = crc16_ccitt_update(crcStartValue, eepromData, crcOffset);
    const uint16_t invertedBigEndianCrc = static_cast<uint16_t>(~(((crc & 0xFF) << 8) | (crc >> 8)));
    std::memcpy(eepromData + crcOffset, &invertedBigEndianCrc, sizeof(invertedBigEndianCrc));
}

class ConfigEepromTest : public testing::Test {
protected:
    void SetUp() override
    {
        std::memset(eepromData, 0, EEPROM_SIZE);
        pgResetAll();
        failureModeCall = -1;
    }

    void writeCurrentConfig()
    {
        writeConfigToEEPROM();
        ASSERT_EQ(-1, failureModeCall);
        ASSERT_TRUE(isEEPROMVersionValid());
        ASSERT_TRUE(isEEPROMStructureValid());
    }

    void downgradeVersionedRecord()
    {
        ASSERT_TRUE(isEEPROMStructureValid());
        const uint16_t configSize = storedConfigSize();
        ASSERT_NE(0, configSize);
        testConfigRecord_t *record = findStoredRecord(PG_RESERVED_FOR_TESTING_1);
        ASSERT_NE(nullptr, record);
        ASSERT_EQ(2, record->version);
        record->version = 1;
        rewriteStoredCrc(configSize);
        ASSERT_TRUE(isEEPROMStructureValid());
    }
};

TEST_F(ConfigEepromTest, VersionMismatchForcesDefaultOnlyConfigRewrite)
{
    writeCurrentConfig();
    downgradeVersionedRecord();

    EXPECT_FALSE(loadEEPROM());
    EXPECT_EQ(0x11223344U, testVersionedConfig()->value);
    EXPECT_EQ(0x55667788U, testNeighborConfig()->value);
    const uint32_t resetHash = fnv_update(FNV_OFFSET_BASIS, testVersionedConfig(), sizeof(*testVersionedConfig()));
    EXPECT_EQ(~resetHash, testVersionedConfig_fnv_hash);

    // initPhase1 responds to a failed readEEPROM() with resetEEPROM(); these are the
    // persistent operations performed by that reset path.
    pgResetAll();
    writeCurrentConfig();

    const testConfigRecord_t *rewrittenRecord = findStoredRecord(PG_RESERVED_FOR_TESTING_1);
    ASSERT_NE(nullptr, rewrittenRecord);
    EXPECT_EQ(2, rewrittenRecord->version);

    testVersionedConfigMutable()->value = 0;
    testNeighborConfigMutable()->value = 0;
    EXPECT_TRUE(loadEEPROM());
    EXPECT_EQ(0x11223344U, testVersionedConfig()->value);
    EXPECT_EQ(0x55667788U, testNeighborConfig()->value);
    EXPECT_EQ(resetHash, testVersionedConfig_fnv_hash);
}

TEST_F(ConfigEepromTest, VersionMismatchRejectsBlobAndResetsUserConfig)
{
    testVersionedConfigMutable()->value = 0xDEADBEEF;
    testNeighborConfigMutable()->value = 0xCAFEBABE;
    writeCurrentConfig();
    downgradeVersionedRecord();

    EXPECT_FALSE(loadEEPROM());
    EXPECT_EQ(0x11223344U, testVersionedConfig()->value);
    EXPECT_EQ(0xCAFEBABEU, testNeighborConfig()->value);

    pgResetAll();
    writeCurrentConfig();

    testVersionedConfigMutable()->value = 0;
    testNeighborConfigMutable()->value = 0;
    EXPECT_TRUE(loadEEPROM());
    EXPECT_EQ(0x11223344U, testVersionedConfig()->value);
    EXPECT_EQ(0x55667788U, testNeighborConfig()->value);
}

} // namespace
