/*
 * Macronix MX66UW1G45G — 1 Gb octal STR/DTR flash driver for the
 * Betaflight N6 OpenBootloader.
 *
 * Targets the wiring on the STM32N6570-DK reference design and pin-
 * compatible boards: XSPI2 + XSPIM_P2 + GPIOM AF11 + NCS1. Boards with
 * different XSPI controllers, IO ports, or chip selects need their own
 * driver source (or this one parameterised via the OBL_FLASH_DRIVER
 * config knob).
 *
 * XSPI bring-up + 1S-1S-1S soft reset + memory-mapped engagement is
 * lifted from lib/main/STM32/n6_fsbl/main.c (proven across multiple boots
 * during the FSBL stub work). The new pieces here are the indirect-mode
 * write primitives (write enable / sector erase / block erase / page
 * program) and a memory-mapped-mode-off hook so the driver can flip
 * between read-via-memmap and write-via-indirect cleanly.
 */

#include <string.h>

#include "stm32n6xx_hal.h"

#include "flash_iface.h"

/* ---------- chip command set (1S-1S-1S, post-reset state) ----------- */
#define MX66_CMD_RESET_ENABLE       0x66U
#define MX66_CMD_RESET              0x99U
#define MX66_CMD_RDSR               0x05U   /* read status register      */
#define MX66_SR_WIP                 0x01U   /* write-in-progress bit     */
#define MX66_CMD_WREN               0x06U   /* write enable              */
#define MX66_CMD_READ_4B            0x0CU   /* 4-byte FAST_READ          */
#define MX66_CMD_READ_4B_DUMMY      8U
#define MX66_CMD_PP_4B              0x12U   /* 4-byte page program       */
#define MX66_CMD_SE_4B              0x21U   /* 4-byte 4 KiB sector erase */
#define MX66_CMD_BE_4B              0xDCU   /* 4-byte 64 KiB block erase */

#define MX66_PAGE_SIZE              256U
#define MX66_SECTOR_SIZE            (4U * 1024U)
#define MX66_TOTAL_SIZE             (128U * 1024U * 1024U) /* 1 Gb = 128 MiB */

#define MX66_RETRIES                3
#define SETTLE_DELAY_MS             600
#define MX66_TIMEOUT_PROGRAM_MS     50
#define MX66_TIMEOUT_SECTOR_MS      400
#define MX66_TIMEOUT_BLOCK_MS       2000
#define MX66_TIMEOUT_GENERIC_MS     1000

#define MX66_MEMMAP_BASE            0x70000000U

static XSPI_HandleTypeDef hxspi2;
static const flash_geometry_t geometry = {
    .page_size_bytes   = MX66_PAGE_SIZE,
    .sector_size_bytes = MX66_SECTOR_SIZE,
    .total_size_bytes  = MX66_TOTAL_SIZE,
};
static bool memmap_active;

/* HAL_XSPI_MspInit (peripheral + IO manager + GPIO N clocks, GPION pins
 * to AF9_XSPIM_P2) lives in stm32n6xx_hal_msp.c — board-level config
 * shared with whatever flash chip happens to be wired to XSPI2. A
 * manufacturer using a different XSPI controller or pin map provides
 * their own MSP file under the per-config build. */

static HAL_StatusTypeDef mx66_wait_ready(uint32_t timeout_ms)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    uint8_t status;
    uint32_t tickstart = HAL_GetTick();

    cmd.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.Instruction        = MX66_CMD_RDSR;
    cmd.DataMode           = HAL_XSPI_DATA_1_LINE;
    cmd.DataLength         = 1;
    cmd.AddressMode        = HAL_XSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DummyCycles        = 0;

    do {
        if (HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return HAL_ERROR;
        }
        if (HAL_XSPI_Receive(&hxspi2, &status, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return HAL_ERROR;
        }
        if ((status & MX66_SR_WIP) == 0) {
            return HAL_OK;
        }
    } while ((HAL_GetTick() - tickstart) < timeout_ms);

    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef mx66_simple_cmd_1s(uint8_t opcode)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.IOSelect           = HAL_XSPI_SELECT_IO_7_0;
    cmd.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Instruction        = opcode;
    cmd.AddressMode        = HAL_XSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode           = HAL_XSPI_DATA_NONE;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_XSPI_DQS_DISABLE;
    return HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

static HAL_StatusTypeDef mx66_cmd_8dtr(uint8_t opcode)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.IOSelect           = HAL_XSPI_SELECT_IO_7_0;
    cmd.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
    cmd.InstructionWidth   = HAL_XSPI_INSTRUCTION_16_BITS;
    cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    cmd.Instruction        = ((uint16_t)opcode << 8) | (uint8_t)(~opcode);
    cmd.AddressMode        = HAL_XSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode           = HAL_XSPI_DATA_NONE;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_XSPI_DQS_DISABLE;
    return HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

static HAL_StatusTypeDef mx66_software_reset(void)
{
    for (int i = 0; i < MX66_RETRIES; i++) {
        (void)mx66_cmd_8dtr(MX66_CMD_RESET_ENABLE);
        (void)mx66_cmd_8dtr(MX66_CMD_RESET);
        HAL_Delay(1);

        if (mx66_simple_cmd_1s(MX66_CMD_RESET_ENABLE) == HAL_OK) {
            if (mx66_simple_cmd_1s(MX66_CMD_RESET) == HAL_OK) {
                HAL_Delay(1);
                return mx66_wait_ready(MX66_TIMEOUT_GENERIC_MS);
            }
        }
    }
    return HAL_ERROR;
}

static HAL_StatusTypeDef mx66_write_enable(void)
{
    HAL_StatusTypeDef st = mx66_simple_cmd_1s(MX66_CMD_WREN);
    return st;
}

static HAL_StatusTypeDef mx66_addr_cmd_1s(uint8_t opcode, uint32_t address,
                                          uint32_t timeout_ms)
{
    if (mx66_write_enable() != HAL_OK) {
        return HAL_ERROR;
    }

    XSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.IOSelect           = HAL_XSPI_SELECT_IO_7_0;
    cmd.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Instruction        = opcode;
    cmd.AddressMode        = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
    cmd.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
    cmd.Address            = address;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode           = HAL_XSPI_DATA_NONE;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_XSPI_DQS_DISABLE;

    if (HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return HAL_ERROR;
    }
    return mx66_wait_ready(timeout_ms);
}

static HAL_StatusTypeDef mx66_page_program(uint32_t address, const uint8_t *data,
                                           uint32_t length)
{
    if (length == 0 || length > MX66_PAGE_SIZE) {
        return HAL_ERROR;
    }
    /* Page-program straddles must be split by the caller — chip wraps the
     * write at the page boundary, which would corrupt earlier bytes in
     * the same page. */
    if (((address & (MX66_PAGE_SIZE - 1)) + length) > MX66_PAGE_SIZE) {
        return HAL_ERROR;
    }

    if (mx66_write_enable() != HAL_OK) {
        return HAL_ERROR;
    }

    XSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.IOSelect           = HAL_XSPI_SELECT_IO_7_0;
    cmd.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Instruction        = MX66_CMD_PP_4B;
    cmd.AddressMode        = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
    cmd.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
    cmd.Address            = address;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode           = HAL_XSPI_DATA_1_LINE;
    cmd.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
    cmd.DataLength         = length;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_XSPI_DQS_DISABLE;

    if (HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_XSPI_Transmit(&hxspi2, (uint8_t *)data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return HAL_ERROR;
    }
    return mx66_wait_ready(MX66_TIMEOUT_PROGRAM_MS);
}

static HAL_StatusTypeDef xspi2_init_controller(void)
{
    XSPIM_CfgTypeDef mgr = {0};

    hxspi2.Instance                  = XSPI2;
    hxspi2.Init.FifoThresholdByte    = 4;
    hxspi2.Init.MemoryMode           = HAL_XSPI_SINGLE_MEM;
    hxspi2.Init.MemoryType           = HAL_XSPI_MEMTYPE_MACRONIX;
    hxspi2.Init.MemorySize           = HAL_XSPI_SIZE_1GB;
    hxspi2.Init.ChipSelectHighTimeCycle = 2;
    hxspi2.Init.FreeRunningClock     = HAL_XSPI_FREERUNCLK_DISABLE;
    hxspi2.Init.ClockMode            = HAL_XSPI_CLOCK_MODE_0;
    hxspi2.Init.WrapSize             = HAL_XSPI_WRAP_NOT_SUPPORTED;
    hxspi2.Init.ClockPrescaler       = 0;
    hxspi2.Init.SampleShifting       = HAL_XSPI_SAMPLE_SHIFT_NONE;
    hxspi2.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
    hxspi2.Init.ChipSelectBoundary   = HAL_XSPI_BONDARYOF_NONE;
    hxspi2.Init.MaxTran              = 0;
    hxspi2.Init.Refresh              = 0;
    hxspi2.Init.MemorySelect         = HAL_XSPI_CSSEL_NCS1;
    if (HAL_XSPI_Init(&hxspi2) != HAL_OK) {
        return HAL_ERROR;
    }

    mgr.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
    mgr.IOPort      = HAL_XSPIM_IOPORT_2;
    mgr.Req2AckTime = 1;
    return HAL_XSPIM_Config(&hxspi2, &mgr, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

bool flash_init(void)
{
    if (xspi2_init_controller() != HAL_OK) {
        return false;
    }
    HAL_Delay(SETTLE_DELAY_MS);

    if (mx66_software_reset() != HAL_OK) {
        return false;
    }
    HAL_Delay(SETTLE_DELAY_MS);

    memmap_active = false;
    return true;
}

void flash_deinit(void)
{
    /* Leave XSPI2 fully configured + memory-mapped so the post-reset
     * boot decision can read nor0 0x0 directly without re-init churn.
     * Callers wanting a true peripheral teardown should HAL_XSPI_DeInit
     * themselves; OBL never does. */
}

const flash_geometry_t *flash_get_geometry(void)
{
    return &geometry;
}

bool flash_memmap_on(void)
{
    if (memmap_active) {
        return true;
    }

    XSPI_RegularCmdTypeDef cmd = {0};
    cmd.IOSelect           = HAL_XSPI_SELECT_IO_7_0;
    cmd.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.AddressMode        = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth       = HAL_XSPI_ADDRESS_32_BITS;
    cmd.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode           = HAL_XSPI_DATA_1_LINE;
    cmd.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
    cmd.DummyCycles        = 0;
    cmd.DQSMode            = HAL_XSPI_DQS_DISABLE;

    cmd.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
    cmd.Instruction   = MX66_CMD_READ_4B;
    cmd.DummyCycles   = MX66_CMD_READ_4B_DUMMY;
    if (HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return false;
    }
    cmd.DummyCycles = 0;

    cmd.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
    cmd.Instruction   = MX66_CMD_PP_4B;     /* placeholder — never issued */
    if (HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return false;
    }

    XSPI_MemoryMappedTypeDef mm = {0};
    mm.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;
    /* Prefetch off — speculative reads against a chip in an unexpected
     * mode would stall the AXI bus indefinitely. Bounded stalls only. */
    mm.NoPrefetchData    = HAL_XSPI_AUTOMATIC_PREFETCH_DISABLE;
    mm.NoPrefetchAXI     = HAL_XSPI_AXI_PREFETCH_DISABLE;
    if (HAL_XSPI_MemoryMapped(&hxspi2, &mm) != HAL_OK) {
        return false;
    }

    memmap_active = true;
    return true;
}

bool flash_memmap_off(void)
{
    if (!memmap_active) {
        return true;
    }
    /* Aborting an in-flight memory-mapped session is the documented way
     * to drop the controller back to indirect command mode. */
    if (HAL_XSPI_Abort(&hxspi2) != HAL_OK) {
        return false;
    }
    memmap_active = false;
    return true;
}

bool flash_erase_sector(uint32_t offset)
{
    if (!flash_memmap_off()) {
        return false;
    }
    return mx66_addr_cmd_1s(MX66_CMD_SE_4B, offset, MX66_TIMEOUT_SECTOR_MS) == HAL_OK;
}

bool flash_program_page(uint32_t offset, const uint8_t *data, uint32_t length)
{
    if (!flash_memmap_off()) {
        return false;
    }
    return mx66_page_program(offset, data, length) == HAL_OK;
}

bool flash_erase_range(uint32_t offset, uint32_t length)
{
    const uint32_t mask = MX66_SECTOR_SIZE - 1;
    if (offset & mask) {
        return false;
    }
    uint32_t end = offset + length;
    /* Round end up to the next sector boundary. */
    end = (end + mask) & ~mask;

    for (uint32_t a = offset; a < end; a += MX66_SECTOR_SIZE) {
        if (!flash_erase_sector(a)) {
            return false;
        }
    }
    return true;
}

bool flash_program(uint32_t offset, const uint8_t *data, uint32_t length)
{
    while (length) {
        uint32_t chunk = MX66_PAGE_SIZE - (offset & (MX66_PAGE_SIZE - 1));
        if (chunk > length) {
            chunk = length;
        }
        if (!flash_program_page(offset, data, chunk)) {
            return false;
        }
        offset += chunk;
        data   += chunk;
        length -= chunk;
    }
    return true;
}

bool flash_verify(uint32_t offset, const uint8_t *data, uint32_t length)
{
    if (!flash_memmap_on()) {
        return false;
    }
    const uint8_t *flash = (const uint8_t *)(MX66_MEMMAP_BASE + offset);
    return memcmp(flash, data, length) == 0;
}
