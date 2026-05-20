/*
 * STM32N6 FSBL stub for the Betaflight N6 platform.
 *
 * Boot flow (BOOT0=USER, BOOT1=2):
 *   1. Boot ROM verifies the signed FSBL header at XSPI 0x70000000,
 *      copies the payload to AXISRAM2 secure (0x34180400), branches to
 *      Reset_Handler.
 *   2. Reset_Handler -> SystemInit -> .data copy / .bss zero -> main().
 *   3. main() initialises XSPI2, walks MX66UW1G45G into 1S-1S-1S
 *      4-byte addressing, engages memory-mapped mode, then jumps directly
 *      to the betaflight XIP entry at 0x70100000 (vector table SP / PC
 *      validated against the AXI-mapped flash window). Betaflight is
 *      linked execute-in-place via STM32N657XX_XIP.ld; only initialised
 *      data and explicit RAM-resident sections are copied to AXISRAM by
 *      Reset_Handler. The stub leaves XSPI2 fully configured + memory-
 *      mapped and does NOT deinit the peripheral before jumping.
 *
 * Signing recipe (output is committed under prebuilt/):
 *
 *   STM32_SigningTool_CLI \
 *       -bin n6_fsbl.bin \
 *       -nk -of 0x80000000 -t ssbl -hv 2.3 -align \
 *       -o prebuilt/n6_fsbl_signed.stm32 -s
 *
 *   -nk    : empty signature, accepted by boot ROM on dev chips.
 *   -align : pad payload to start at 0x400 in the .stm32.
 *   -t ssbl: emits Binary type 0x0 in the v2.3 header. -t fsbl emits
 *            type 0x10 which the N6 boot ROM rejects.
 */

#include <string.h>

#include "main.h"

#define APP_XIP_BASE         0x70100000U
#define MX66_RETRIES         3
/* Inter-XSPI-step delay so the MX66 leaves the controller cleanly idle
 * between commands. Empirically 250-600 ms is enough; keep generous. */
#define SETTLE_DELAY_MS      600

/* MX66UW1G45G command set (1S-1S-1S, post-reset state). */
#define MX66_CMD_RESET_ENABLE   0x66U
#define MX66_CMD_RESET          0x99U
/* 4-byte FAST_READ + 8 dummy cycles. Plain 0x13 4READ at 0 dummy only
 * specs to 86 MHz; the XSPI2 kernel comes out of the boot ROM faster
 * than that and 0x13 mis-samples the first data byte after the address
 * phase. 0x0C with 8 dummy cycles is reliable to 133 MHz. */
#define MX66_CMD_READ_4B        0x0CU
#define MX66_CMD_READ_4B_DUMMY  8U
#define MX66_CMD_RDSR           0x05U   /* Read Status Register */
#define MX66_SR_WIP             0x01U   /* Write In Progress bit */

XSPI_HandleTypeDef hxspi2;

/*
 * Poll the flash status register until the WIP bit is cleared.
 */
static HAL_StatusTypeDef mx66_wait_ready(uint32_t timeout_ms)
{
    XSPI_RegularCmdTypeDef sCommand = {0};
    uint8_t status;
    uint32_t tickstart = HAL_GetTick();

    sCommand.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    sCommand.Instruction        = MX66_CMD_RDSR;
    sCommand.DataMode           = HAL_XSPI_DATA_1_LINE;
    sCommand.DataLength         = 1;
    sCommand.AddressMode        = HAL_XSPI_ADDRESS_NONE;
    sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    sCommand.DummyCycles        = 0;

    do {
        if (HAL_XSPI_Command(&hxspi2, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
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

/* Bring up XSPI2 in 1S-1S-1S 4-byte addressing mode. MSP_Init enables
 * peripheral + XSPIM + GPION clocks and configures GPIO N pins to AF11
 * (XSPIM_P2). */
static void MX_XSPI2_Init(void)
{
    XSPIM_CfgTypeDef sXspiManagerCfg = {0};

    hxspi2.Instance = XSPI2;
    hxspi2.Init.FifoThresholdByte = 4;
    hxspi2.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
    hxspi2.Init.MemoryType = HAL_XSPI_MEMTYPE_MACRONIX;
    hxspi2.Init.MemorySize = HAL_XSPI_SIZE_1GB;
    hxspi2.Init.ChipSelectHighTimeCycle = 2;
    hxspi2.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
    hxspi2.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
    hxspi2.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
    hxspi2.Init.ClockPrescaler = 0;
    hxspi2.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
    hxspi2.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
    hxspi2.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
    hxspi2.Init.MaxTran = 0;
    hxspi2.Init.Refresh = 0;
    hxspi2.Init.MemorySelect = HAL_XSPI_CSSEL_NCS1;
    if (HAL_XSPI_Init(&hxspi2) != HAL_OK) {
        Error_Handler();
    }

    sXspiManagerCfg.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
    sXspiManagerCfg.IOPort = HAL_XSPIM_IOPORT_2;
    sXspiManagerCfg.Req2AckTime = 1;
    if (HAL_XSPIM_Config(&hxspi2, &sXspiManagerCfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        Error_Handler();
    }
}

/* Send a single 1S-1S-1S no-data command. */
static HAL_StatusTypeDef mx66_cmd_1s(uint8_t opcode)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Instruction = opcode;
    cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode = HAL_XSPI_DATA_NONE;
    cmd.DummyCycles = 0;
    cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    return HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

/* Send a single 8-line DTR no-data command. The chip in 8D-DTR mode
 * expects a 16-bit instruction = opcode | (~opcode << 8) on all eight
 * IOs; harmless if the chip is currently in 1S mode (IO[7:1] aren't
 * being watched), required if it isn't. */
static HAL_StatusTypeDef mx66_cmd_8dtr(uint8_t opcode)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    cmd.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    cmd.Instruction = ((uint16_t)opcode << 8) | (uint8_t)(~opcode);
    cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode = HAL_XSPI_DATA_NONE;
    cmd.DummyCycles = 0;
    cmd.DQSMode = HAL_XSPI_DQS_DISABLE;
    return HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

/* Walk MX66UW1G45G to a known 1S-1S-1S state regardless of how the boot
 * ROM left it. The 8D-DTR pair drops the chip to 1S if it's there, then
 * the 1S pair issues the actual reset. Datasheet requires >= 30 us
 * between RESET and the next command. */
static HAL_StatusTypeDef mx66_software_reset(void)
{
    for (int i = 0; i < MX66_RETRIES; i++) {
        (void)mx66_cmd_8dtr(MX66_CMD_RESET_ENABLE);
        (void)mx66_cmd_8dtr(MX66_CMD_RESET);
        HAL_Delay(1);

        if (mx66_cmd_1s(MX66_CMD_RESET_ENABLE) == HAL_OK) {
            if (mx66_cmd_1s(MX66_CMD_RESET) == HAL_OK) {
                HAL_Delay(1);
                return mx66_wait_ready(100);
            }
        }
    }
    return HAL_ERROR;
}

/* Latch the READ_CFG and a placeholder WRITE_CFG into the XSPI handle.
 * Both sides are required to advance the handle's state to CMD_CFG;
 * without the write side HAL_XSPI_MemoryMapped() returns
 * INVALID_SEQUENCE. The write opcode is never issued — memory-mapped
 * XSPI is read-only here — so 0x12 (4-byte PAGE PROGRAM) is just a
 * legal placeholder. */
static HAL_StatusTypeDef xspi2_latch_read_cmd(void)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    cmd.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    cmd.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd.DummyCycles = 0;
    cmd.DQSMode = HAL_XSPI_DQS_DISABLE;

    cmd.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
    cmd.Instruction = MX66_CMD_READ_4B;       /* 4-byte FAST_READ */
    cmd.DummyCycles = MX66_CMD_READ_4B_DUMMY;
    if (HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return HAL_ERROR;
    }
    cmd.DummyCycles = 0;

    cmd.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
    cmd.Instruction = 0x12U;                  /* 4-byte PAGE PROGRAM (placeholder) */
    return HAL_XSPI_Command(&hxspi2, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

static HAL_StatusTypeDef xspi2_engage_mem_mapped(void)
{
    XSPI_MemoryMappedTypeDef memMappedCfg = {0};
    memMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;
    /* Disable prefetch — speculative reads against a chip in an
     * unexpected mode (e.g. still 8D-DTR while we use 1S) would stall
     * the AXI bus indefinitely. With prefetch off, stalls are bounded
     * to a single explicit CPU read. */
    memMappedCfg.NoPrefetchData = HAL_XSPI_AUTOMATIC_PREFETCH_DISABLE;
    memMappedCfg.NoPrefetchAXI  = HAL_XSPI_AXI_PREFETCH_DISABLE;
    return HAL_XSPI_MemoryMapped(&hxspi2, &memMappedCfg);
}

static __attribute__((noreturn)) void jump_to_app(uint32_t app_base)
{
    const uint32_t app_sp = *(volatile uint32_t *)(app_base);
    const uint32_t app_pc = *(volatile uint32_t *)(app_base + 4U);

    /* Sanity gate: SP must point into AXISRAM (0x2xxxxxxx). An erased flash
     * slot reads 0xFFFFFFFF and would fail this check. */
    if ((app_sp & 0xF8000000U) != 0x20000000U) {
        Error_Handler();
    }

    HAL_SuspendTick();
    __disable_irq();

    /* Leave XSPI2 fully configured + memory-mapped. The app runs XIP from
     * 0x70100000 — any DeInit / FORCE_RESET here would un-map the window
     * and the next instruction fetch would bus-fault. */

    SCB_DisableICache();
    SCB_DisableDCache();

    SCB->VTOR = app_base;
    __DSB();
    __ISB();
    __set_MSP(app_sp);
    /* Re-enable IRQs so the app inherits PRIMASK=0. App startup_*.s files
     * that don't call cpsie would otherwise run with interrupts masked
     * and SysTick never fires (HAL_Delay / PLL-lock waits would hang). */
    __enable_irq();
    ((void (*)(void))app_pc)();

    while (1) {
        __NOP();
    }
}

int main(void)
{
    HAL_Init();

    /* Leave RIFSC alone. The boot ROM has already set up master/slave
     * tagging for the OPEN-lifecycle dev path; the registers are
     * clamped immediately after that and any writes here would either
     * clobber working tagging or silently drop. */

    MX_XSPI2_Init();
    HAL_Delay(SETTLE_DELAY_MS);

    if (mx66_software_reset() != HAL_OK) {
        Error_Handler();
    }
    HAL_Delay(SETTLE_DELAY_MS);

    if (xspi2_latch_read_cmd() != HAL_OK) {
        Error_Handler();
    }
    HAL_Delay(SETTLE_DELAY_MS);

    if (xspi2_engage_mem_mapped() != HAL_OK) {
        Error_Handler();
    }

    /* "STM2" little-endian = 0x324D5453. */
    if (*(volatile uint32_t *)0x70000000U != 0x324D5453U) {
        Error_Handler();
    }

    /* The app at APP_XIP_BASE is XIP-linked: vector table SP/PC are read
     * straight from the memory-mapped XSPI window. Validate the head of
     * the table before handing off — SP in AXISRAM (0x2xxxxxxx),
     * Reset_Handler within the 0x70100000 XIP slot with thumb bit set.
     * An erased slot reads 0xFFFFFFFF and fails the SP check. */
    const uint32_t app_sp = *(volatile uint32_t *)APP_XIP_BASE;
    const uint32_t app_pc = *(volatile uint32_t *)(APP_XIP_BASE + 4U);
    if ((app_sp & 0xF8000000U) != 0x20000000U
            || (app_pc & 0xFFF00000U) != APP_XIP_BASE
            || (app_pc & 1U) == 0U) {
        Error_Handler();
    }

    jump_to_app(APP_XIP_BASE);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
