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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/utils.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/rmt_ll.h"
#include "hal/gpio_ll.h"
#pragma GCC diagnostic pop

#include "soc/rmt_struct.h"
#include "soc/gpio_struct.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/motor_impl.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/time.h"

#include "pg/motor.h"

#define DSHOT_MAX_MOTORS     4
#define DSHOT_FRAME_BITS     16
#define DSHOT_RMT_CLK_DIV    4      // 80MHz APB / 4 = 20MHz (50ns per tick)

// DShot600 timing in RMT ticks (at 20MHz = 50ns per tick)
#define DSHOT600_T1H_TICKS   25     // 1.25us high for '1'
#define DSHOT600_T1L_TICKS   8      // 0.42us low for '1'
#define DSHOT600_T0H_TICKS   12     // 0.625us high for '0'
#define DSHOT600_T0L_TICKS   21     // 1.04us low for '0'

// RMT memory base address for ESP32-S3 (from linker script: RMTMEM = 0x60016800)
#define RMT_MEM_BASE         0x60016800
#define RMT_MEM_ITEM_SIZE    sizeof(rmt_symbol_word_t)
#define RMT_MEM_BLOCK_WORDS  SOC_RMT_MEM_WORDS_PER_CHANNEL  // 48 words per channel block

typedef struct {
    IO_t io;
    uint8_t rmtChannel;
    bool enabled;
} dshotMotor_t;

static dshotMotor_t dshotMotors[DSHOT_MAX_MOTORS];
static uint8_t esp32DshotMotorCount = 0;
static bool dshotInitialized = false;

// DShot packet buffer per motor: 16 data bits + 1 end marker
static rmt_symbol_word_t dshotBuffer[DSHOT_MAX_MOTORS][DSHOT_FRAME_BITS + 1];

static void dshotEncodeBit(rmt_symbol_word_t *item, bool bit)
{
    if (bit) {
        item->duration0 = DSHOT600_T1H_TICKS;
        item->level0 = 1;
        item->duration1 = DSHOT600_T1L_TICKS;
        item->level1 = 0;
    } else {
        item->duration0 = DSHOT600_T0H_TICKS;
        item->level0 = 1;
        item->duration1 = DSHOT600_T0L_TICKS;
        item->level1 = 0;
    }
}

static void dshotEncodePacket(uint8_t motorIndex, uint16_t packet)
{
    for (int i = 0; i < DSHOT_FRAME_BITS; i++) {
        dshotEncodeBit(&dshotBuffer[motorIndex][i], (packet >> (DSHOT_FRAME_BITS - 1 - i)) & 1);
    }
    // End marker: zero duration terminates RMT transmission
    dshotBuffer[motorIndex][DSHOT_FRAME_BITS].val = 0;
}

static volatile rmt_symbol_word_t *dshotGetRmtMemory(uint8_t channel)
{
    return (volatile rmt_symbol_word_t *)(RMT_MEM_BASE + channel * RMT_MEM_BLOCK_WORDS * RMT_MEM_ITEM_SIZE);
}

// Motor VTable functions

static void dshotPostInit(void)
{
}

static bool dshotEnable(void)
{
    for (int i = 0; i < esp32DshotMotorCount; i++) {
        dshotMotors[i].enabled = true;
    }
    return true;
}

static void dshotDisable(void)
{
    for (int i = 0; i < esp32DshotMotorCount; i++) {
        dshotMotors[i].enabled = false;
    }
}

static bool dshotIsMotorEnabled(unsigned index)
{
    return index < esp32DshotMotorCount && dshotMotors[index].enabled;
}

static bool dshotDecodeTelemetry(void)
{
    return true;
}

static bool dshotTelemetryWait(void)
{
    return true;
}

static void dshotUpdateInit(void)
{
}

static void dshotWriteInt(uint8_t index, uint16_t value)
{
    if (index >= esp32DshotMotorCount || !dshotMotors[index].enabled) {
        return;
    }

    dshotProtocolControl_t pcb = { .value = value, .requestTelemetry = false };
    uint16_t packet = prepareDshotPacket(&pcb);

    dshotEncodePacket(index, packet);

    // Copy encoded items to RMT peripheral memory
    uint8_t ch = dshotMotors[index].rmtChannel;
    volatile rmt_symbol_word_t *rmtMem = dshotGetRmtMemory(ch);
    for (int i = 0; i <= DSHOT_FRAME_BITS; i++) {
        rmtMem[i].val = dshotBuffer[index][i].val;
    }
}

static void dshotWrite(uint8_t index, float value)
{
    dshotWriteInt(index, lrintf(value));
}

static void dshotUpdateComplete(void)
{
    // Start all enabled channels
    for (int i = 0; i < esp32DshotMotorCount; i++) {
        if (dshotMotors[i].enabled) {
            uint8_t ch = dshotMotors[i].rmtChannel;
            rmt_ll_tx_reset_pointer(&RMT, ch);
            rmt_ll_tx_start(&RMT, ch);
        }
    }
}

static void dshotShutdown(void)
{
    for (int i = 0; i < esp32DshotMotorCount; i++) {
        rmt_ll_tx_stop(&RMT, dshotMotors[i].rmtChannel);
        dshotMotors[i].enabled = false;
    }
}

static bool dshotIsMotorIdle(unsigned index)
{
    UNUSED(index);
    return false;
}

static IO_t dshotGetMotorIO(unsigned index)
{
    if (index >= esp32DshotMotorCount) {
        return IO_NONE;
    }
    return dshotMotors[index].io;
}

static void dshotRequestTelemetry(unsigned index)
{
    UNUSED(index);
}

static const motorVTable_t dshotVTable = {
    .postInit = dshotPostInit,
    .convertExternalToMotor = dshotConvertFromExternal,
    .convertMotorToExternal = dshotConvertToExternal,
    .enable = dshotEnable,
    .disable = dshotDisable,
    .isMotorEnabled = dshotIsMotorEnabled,
    .telemetryWait = dshotTelemetryWait,
    .decodeTelemetry = dshotDecodeTelemetry,
    .updateInit = dshotUpdateInit,
    .write = dshotWrite,
    .writeInt = dshotWriteInt,
    .updateComplete = dshotUpdateComplete,
    .shutdown = dshotShutdown,
    .isMotorIdle = dshotIsMotorIdle,
    .getMotorIO = dshotGetMotorIO,
    .requestTelemetry = dshotRequestTelemetry,
};

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    // The rmt_ll_enable_bus_clock and rmt_ll_reset_register macros require
    // a local __DECLARE_RCC_ATOMIC_ENV variable for critical section usage
    int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));

    // Enable RMT peripheral clock and reset
    rmt_ll_enable_bus_clock(0, true);
    rmt_ll_reset_register(0);
    rmt_ll_enable_periph_clock(&RMT, true);

    // Enable non-FIFO (direct memory) access mode
    rmt_ll_enable_mem_access_nonfifo(&RMT, true);

    // Force power on the RMT memory block
    rmt_ll_mem_force_power_on(&RMT);

    // Select APB clock source (80MHz) with divider=1 (no group-level division)
    rmt_ll_set_group_clock_src(&RMT, 0, RMT_CLK_SRC_APB, 1, 0, 0);
    rmt_ll_enable_group_clock(&RMT, true);

    esp32DshotMotorCount = 0;

    for (int i = 0; i < DSHOT_MAX_MOTORS && i < MAX_SUPPORTED_MOTORS; i++) {
        const ioTag_t tag = motorConfig->ioTags[i];
        if (!tag) {
            break;
        }

        IO_t io = IOGetByTag(tag);
        if (!io) {
            break;
        }

        IOInit(io, OWNER_MOTOR, i);
        IOConfigGPIO(io, IOCFG_OUT_PP);

        uint32_t pin = IO_Pin(io);
        uint8_t ch = i;  // RMT TX channel 0-3

        dshotMotors[i].io = io;
        dshotMotors[i].rmtChannel = ch;
        dshotMotors[i].enabled = false;

        // Set per-channel clock divider: APB 80MHz / 4 = 20MHz (50ns per tick)
        rmt_ll_tx_set_channel_clock_div(&RMT, ch, DSHOT_RMT_CLK_DIV);

        // Allocate 1 memory block (48 items) per channel - sufficient for 16-bit DShot + end marker
        rmt_ll_tx_set_mem_blocks(&RMT, ch, 1);

        // Set idle output level low when not transmitting
        rmt_ll_tx_fix_idle_level(&RMT, ch, 0, true);

        // Disable carrier modulation (not needed for DShot)
        rmt_ll_tx_enable_carrier_modulation(&RMT, ch, false);

        // Disable continuous/loop transmission - each frame is a single shot
        rmt_ll_tx_enable_loop(&RMT, ch, false);

        // Connect RMT TX channel output to GPIO pin via the GPIO matrix
        esp_rom_gpio_pad_select_gpio(pin);
        gpio_ll_output_enable(&GPIO, pin);
        esp_rom_gpio_connect_out_signal(pin, RMT_SIG_OUT0_IDX + ch, false, false);

        esp32DshotMotorCount++;
    }

    if (esp32DshotMotorCount == 0) {
        return false;
    }

    device->vTable = &dshotVTable;
    device->count = esp32DshotMotorCount;
    device->initialized = true;

    dshotInitialized = true;

    return true;
}
