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
 * XSPI support for STM32N6.
 *
 * The STM32N6 has three XSPI peripherals. XSPI2 is memory-mapped at
 * 0x70000000 and is used for booting from external flash.
 *
 * This implementation assumes the bootloader has already configured
 * XSPI2 in memory-mapped mode, matching the approach used for OCTOSPI
 * on the STM32H7 series.
 *
 * The XSPI peripheral registers are register-compatible with the H7
 * OCTOSPI peripheral, using XSPI_ prefixed defines instead of OCTOSPI_.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OCTOSPI

#include "drivers/system.h"

#include "drivers/bus_octospi.h"
#include "drivers/bus_octospi_impl.h"

// Provide platform-specific hardware table for STM32N6
const octoSpiHardware_t octoSpiHardware[OCTOSPIDEV_COUNT] = {
    {
        .device = OCTOSPIDEV_1,
        .reg = (octoSpiResource_t *)XSPI2,
    }
};

#define XSPI_INTERFACE_COUNT         1

#define XSPI_FUNCTIONAL_MODE_INDIRECT_WRITE ((uint32_t)0x00000000)
#define XSPI_FUNCTIONAL_MODE_INDIRECT_READ  ((uint32_t)XSPI_CR_FMODE_0)
#define XSPI_FUNCTIONAL_MODE_AUTO_POLLING   ((uint32_t)XSPI_CR_FMODE_1)
#define XSPI_FUNCTIONAL_MODE_MEMORY_MAPPED  ((uint32_t)XSPI_CR_FMODE)

#define XSPI_DHQC_DISABLE                ((uint32_t)0x00000000U)
#define XSPI_DHQC_ENABLE                 ((uint32_t)XSPI_TCR_DHQC)

#define XSPI_OPTYPE_COMMON_CFG           ((uint32_t)0x00000000U)

#define XSPI_OPTYPE_READ_CFG             ((uint32_t)0x00000001U)
#define XSPI_OPTYPE_WRITE_CFG            ((uint32_t)0x00000002U)
#define XSPI_OPTYPE_WRAP_CFG             ((uint32_t)0x00000003U)

#define XSPI_INSTRUCTION_NONE            ((uint32_t)0x00000000U)
#define XSPI_INSTRUCTION_1_LINE          ((uint32_t)XSPI_CCR_IMODE_0)
#define XSPI_INSTRUCTION_2_LINES         ((uint32_t)XSPI_CCR_IMODE_1)
#define XSPI_INSTRUCTION_4_LINES         ((uint32_t)(XSPI_CCR_IMODE_0 | XSPI_CCR_IMODE_1))
#define XSPI_INSTRUCTION_8_LINES         ((uint32_t)XSPI_CCR_IMODE_2)

#define XSPI_INSTRUCTION_8_BITS          ((uint32_t)0x00000000U)
#define XSPI_INSTRUCTION_16_BITS         ((uint32_t)XSPI_CCR_ISIZE_0)
#define XSPI_INSTRUCTION_24_BITS         ((uint32_t)XSPI_CCR_ISIZE_1)
#define XSPI_INSTRUCTION_32_BITS         ((uint32_t)XSPI_CCR_ISIZE)

#define XSPI_INSTRUCTION_DTR_DISABLE     ((uint32_t)0x00000000U)
#define XSPI_INSTRUCTION_DTR_ENABLE      ((uint32_t)XSPI_CCR_IDTR)

#define XSPI_ADDRESS_NONE                ((uint32_t)0x00000000U)
#define XSPI_ADDRESS_1_LINE              ((uint32_t)XSPI_CCR_ADMODE_0)
#define XSPI_ADDRESS_2_LINES             ((uint32_t)XSPI_CCR_ADMODE_1)
#define XSPI_ADDRESS_4_LINES             ((uint32_t)(XSPI_CCR_ADMODE_0 | XSPI_CCR_ADMODE_1))
#define XSPI_ADDRESS_8_LINES             ((uint32_t)XSPI_CCR_ADMODE_2)

#define XSPI_ADDRESS_8_BITS              ((uint32_t)0x00000000U)
#define XSPI_ADDRESS_16_BITS             ((uint32_t)XSPI_CCR_ADSIZE_0)
#define XSPI_ADDRESS_24_BITS             ((uint32_t)XSPI_CCR_ADSIZE_1)
#define XSPI_ADDRESS_32_BITS             ((uint32_t)XSPI_CCR_ADSIZE)

#define XSPI_ADDRESS_DTR_DISABLE         ((uint32_t)0x00000000U)
#define XSPI_ADDRESS_DTR_ENABLE          ((uint32_t)XSPI_CCR_ADDTR)

#define XSPI_DATA_NONE                   ((uint32_t)0x00000000U)
#define XSPI_DATA_1_LINE                 ((uint32_t)XSPI_CCR_DMODE_0)
#define XSPI_DATA_2_LINES                ((uint32_t)XSPI_CCR_DMODE_1)
#define XSPI_DATA_4_LINES               ((uint32_t)(XSPI_CCR_DMODE_0 | XSPI_CCR_DMODE_1))
#define XSPI_DATA_8_LINES                ((uint32_t)XSPI_CCR_DMODE_2)

#define XSPI_DATA_DTR_DISABLE            ((uint32_t)0x00000000U)
#define XSPI_DATA_DTR_ENABLE             ((uint32_t)XSPI_CCR_DDTR)

#define XSPI_ALTERNATE_BYTES_NONE        ((uint32_t)0x00000000U)
#define XSPI_ALTERNATE_BYTES_1_LINE      ((uint32_t)XSPI_CCR_ABMODE_0)
#define XSPI_ALTERNATE_BYTES_2_LINES     ((uint32_t)XSPI_CCR_ABMODE_1)
#define XSPI_ALTERNATE_BYTES_4_LINES     ((uint32_t)(XSPI_CCR_ABMODE_0 | XSPI_CCR_ABMODE_1))
#define XSPI_ALTERNATE_BYTES_8_LINES     ((uint32_t)XSPI_CCR_ABMODE_2)

#define XSPI_ALTERNATE_BYTES_8_BITS      ((uint32_t)0x00000000U)
#define XSPI_ALTERNATE_BYTES_16_BITS     ((uint32_t)XSPI_CCR_ABSIZE_0)
#define XSPI_ALTERNATE_BYTES_24_BITS     ((uint32_t)XSPI_CCR_ABSIZE_1)
#define XSPI_ALTERNATE_BYTES_32_BITS     ((uint32_t)XSPI_CCR_ABSIZE)

#define XSPI_ALTERNATE_BYTES_DTR_DISABLE ((uint32_t)0x00000000U)
#define XSPI_ALTERNATE_BYTES_DTR_ENABLE  ((uint32_t)XSPI_CCR_ABDTR)

#define XSPI_DQS_DISABLE                 ((uint32_t)0x00000000U)
#define XSPI_DQS_ENABLE                  ((uint32_t)XSPI_CCR_DQSE)

// XSPI does not have SIOO (Send Instruction Only Once) like OCTOSPI
#define XSPI_SIOO_INST_EVERY_CMD         ((uint32_t)0x00000000U)

static MMFLASH_CODE_NOINLINE void Error_Handler(void) {
    while (1) {
        NOOP;
    }
}

#define __XSPI_GET_FLAG(__INSTANCE__, __FLAG__)           ((READ_BIT((__INSTANCE__)->SR, (__FLAG__)) != 0U) ? SET : RESET)
#define __XSPI_CLEAR_FLAG(__INSTANCE__, __FLAG__)           WRITE_REG((__INSTANCE__)->FCR, (__FLAG__))
#define __XSPI_ENABLE(__INSTANCE__)                       SET_BIT((__INSTANCE__)->CR, XSPI_CR_EN)
#define __XSPI_DISABLE(__INSTANCE__)                      CLEAR_BIT((__INSTANCE__)->CR, XSPI_CR_EN)
#define __XSPI_IS_ENABLED(__INSTANCE__)                   (READ_BIT((__INSTANCE__)->CR, XSPI_CR_EN) != 0U)

MMFLASH_CODE_NOINLINE static void xspiAbort(XSPI_TypeDef *instance)
{
    SET_BIT(instance->CR, XSPI_CR_ABORT);
}

MMFLASH_CODE_NOINLINE static void xspiWaitStatusFlags(XSPI_TypeDef *instance, uint32_t mask, FlagStatus flagStatus)
{
    uint32_t regval;

    switch (flagStatus) {
    case SET:
        while (!((regval = READ_REG(instance->SR)) & mask))
            {}
        break;
    case RESET:
        while (((regval = READ_REG(instance->SR)) & mask))
            {}
        break;
    }

    (void)regval;
}

typedef struct {
    uint32_t OperationType;

    uint32_t Instruction;
    uint32_t InstructionMode;
    uint32_t InstructionSize;
    uint32_t InstructionDtrMode;

    uint32_t Address;
    uint32_t AddressMode;
    uint32_t AddressSize;
    uint32_t AddressDtrMode;

    uint32_t AlternateBytes;
    uint32_t AlternateBytesMode;
    uint32_t AlternateBytesSize;
    uint32_t AlternateBytesDtrMode;

    uint32_t DummyCycles; // 0-31

    uint32_t DataMode;
    uint32_t DataDtrMode;
    uint32_t NbData;

    uint32_t DQSMode;
    uint32_t SIOOMode;
} XSPI_Command_t;

MMFLASH_CODE_NOINLINE static ErrorStatus xspiConfigureCommand(XSPI_TypeDef *instance, XSPI_Command_t *cmd)
{
    ErrorStatus status = SUCCESS;

    MODIFY_REG(instance->CR, XSPI_CR_FMODE, 0U);

    instance->CCR = (cmd->DQSMode | cmd->SIOOMode);

    if (cmd->AlternateBytesMode != XSPI_ALTERNATE_BYTES_NONE)
    {
        instance->ABR = cmd->AlternateBytes;

        MODIFY_REG(
            instance->ABR,
            (XSPI_CCR_ABMODE | XSPI_CCR_ABDTR | XSPI_CCR_ABSIZE),
            (cmd->AlternateBytesMode | cmd->AlternateBytesDtrMode | cmd->AlternateBytesSize)
        );
    }

    MODIFY_REG(instance->TCR, XSPI_TCR_DCYC, cmd->DummyCycles);

    if (cmd->DataMode != XSPI_DATA_NONE)
    {
      if (cmd->OperationType == XSPI_OPTYPE_COMMON_CFG)
      {
        instance->DLR = (cmd->NbData - 1U);
      }
    }

    if (cmd->InstructionMode != XSPI_INSTRUCTION_NONE)
    {
      if (cmd->AddressMode != XSPI_ADDRESS_NONE)
      {
        if (cmd->DataMode != XSPI_DATA_NONE)
        {
          // instruction, address and data

          MODIFY_REG(instance->CCR, (XSPI_CCR_IMODE  | XSPI_CCR_IDTR  | XSPI_CCR_ISIZE  |
                                  XSPI_CCR_ADMODE | XSPI_CCR_ADDTR | XSPI_CCR_ADSIZE |
                                  XSPI_CCR_DMODE  | XSPI_CCR_DDTR),
                                 (cmd->InstructionMode | cmd->InstructionDtrMode | cmd->InstructionSize |
                                  cmd->AddressMode     | cmd->AddressDtrMode     | cmd->AddressSize     |
                                  cmd->DataMode        | cmd->DataDtrMode));
        }
        else
        {
          // instruction and address

          MODIFY_REG(instance->CCR, (XSPI_CCR_IMODE  | XSPI_CCR_IDTR  | XSPI_CCR_ISIZE  |
                                  XSPI_CCR_ADMODE | XSPI_CCR_ADDTR | XSPI_CCR_ADSIZE),
                                 (cmd->InstructionMode | cmd->InstructionDtrMode | cmd->InstructionSize |
                                  cmd->AddressMode     | cmd->AddressDtrMode     | cmd->AddressSize));

          // DHQC bit is linked with DDTR
          if (((instance->TCR & XSPI_TCR_DHQC_Msk) == XSPI_DHQC_ENABLE) &&
              (cmd->InstructionDtrMode == XSPI_INSTRUCTION_DTR_ENABLE))
          {
            MODIFY_REG(instance->CCR, XSPI_CCR_DDTR, XSPI_DATA_DTR_ENABLE);
          }
        }

        instance->IR = cmd->Instruction;

        instance->AR = cmd->Address;
      }
      else
      {
        if (cmd->DataMode != XSPI_DATA_NONE)
        {
          // instruction and data

          MODIFY_REG(instance->CCR, (XSPI_CCR_IMODE | XSPI_CCR_IDTR | XSPI_CCR_ISIZE |
                                  XSPI_CCR_DMODE | XSPI_CCR_DDTR),
                                 (cmd->InstructionMode | cmd->InstructionDtrMode | cmd->InstructionSize |
                                  cmd->DataMode        | cmd->DataDtrMode));
        }
        else
        {
          // instruction only

          MODIFY_REG(instance->CCR, (XSPI_CCR_IMODE | XSPI_CCR_IDTR | XSPI_CCR_ISIZE),
                                 (cmd->InstructionMode | cmd->InstructionDtrMode | cmd->InstructionSize));

          // DHQC bit is linked with DDTR
          if (((instance->TCR & XSPI_TCR_DHQC_Msk) == XSPI_DHQC_ENABLE) &&
              (cmd->InstructionDtrMode == XSPI_INSTRUCTION_DTR_ENABLE))
          {
            MODIFY_REG(instance->CCR, XSPI_CCR_DDTR, XSPI_DATA_DTR_ENABLE);
          }
        }

        instance->IR = cmd->Instruction;

      }
    }
    else
    {
      if (cmd->AddressMode != XSPI_ADDRESS_NONE)
      {
        if (cmd->DataMode != XSPI_DATA_NONE)
        {
          // address and data

          MODIFY_REG(instance->CCR, (XSPI_CCR_ADMODE | XSPI_CCR_ADDTR | XSPI_CCR_ADSIZE |
                                  XSPI_CCR_DMODE  | XSPI_CCR_DDTR),
                                 (cmd->AddressMode | cmd->AddressDtrMode | cmd->AddressSize     |
                                  cmd->DataMode    | cmd->DataDtrMode));
        }
        else
        {
          // address only

          MODIFY_REG(instance->CCR, (XSPI_CCR_ADMODE | XSPI_CCR_ADDTR | XSPI_CCR_ADSIZE),
                                 (cmd->AddressMode | cmd->AddressDtrMode | cmd->AddressSize));
        }

        instance->AR = cmd->Address;
      }
      else
      {
        // no instruction, no address
        status = ERROR;
      }
    }

    return status;
}

static MMFLASH_CODE_NOINLINE ErrorStatus xspiCommand(XSPI_TypeDef *instance, XSPI_Command_t *cmd)
{
    xspiWaitStatusFlags(instance, XSPI_SR_BUSY, RESET);

    ErrorStatus status = xspiConfigureCommand(instance, cmd);
    if (status == SUCCESS) {
        if (cmd->DataMode == XSPI_DATA_NONE)
        {
            // transfer is already started, wait now.
            xspiWaitStatusFlags(instance, XSPI_SR_TCF, SET);
            __XSPI_CLEAR_FLAG(instance, XSPI_SR_TCF);
        }
    }

    return status;
}

/*
 * Transmit
 *
 * Call xspiCommand first to configure the transaction stages.
 */
static MMFLASH_CODE_NOINLINE ErrorStatus xspiTransmit(XSPI_TypeDef *instance, uint8_t *data)
{
    if (data == NULL) {
        return ERROR;
    }

    __IO uint32_t              XferCount = READ_REG(instance->DLR) + 1U;
    uint8_t                    *pBuffPtr = data;

    MODIFY_REG(instance->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

    __IO uint32_t *data_reg = &instance->DR;
    do
    {
      xspiWaitStatusFlags(instance, XSPI_SR_FTF, SET);

      *((__IO uint8_t *)data_reg) = *pBuffPtr;
      pBuffPtr++;
      XferCount--;
    } while (XferCount > 0U);

    xspiWaitStatusFlags(instance, XSPI_SR_TCF, SET);
    __XSPI_CLEAR_FLAG(instance, XSPI_SR_TCF);

    return SUCCESS;
}

/*
 * Receive
 *
 * Call xspiCommand first to configure the transaction stages.
 */
static MMFLASH_CODE_NOINLINE ErrorStatus xspiReceive(XSPI_TypeDef *instance, uint8_t *data)
{
    if (data == NULL) {
        return ERROR;
    }

    __IO uint32_t              XferCount = READ_REG(instance->DLR) + 1U;
    uint8_t                    *pBuffPtr = data;

    MODIFY_REG(instance->CR, XSPI_CR_FMODE, XSPI_FUNCTIONAL_MODE_INDIRECT_READ);

    uint32_t addr_reg = instance->AR;
    uint32_t ir_reg = instance->IR;

    // Trigger the transfer by re-writing the address or instruction register
    if (READ_BIT(instance->CCR, XSPI_CCR_ADMODE) != XSPI_ADDRESS_NONE)
    {
      WRITE_REG(instance->AR, addr_reg);
    }
    else
    {
      WRITE_REG(instance->IR, ir_reg);
    }

    __IO uint32_t *data_reg = &instance->DR;

    do
    {
      xspiWaitStatusFlags(instance, XSPI_SR_FTF | XSPI_SR_TCF, SET);

      *pBuffPtr = *((__IO uint8_t *)data_reg);
      pBuffPtr++;
      XferCount--;
    } while(XferCount > 0U);

    xspiWaitStatusFlags(instance, XSPI_SR_TCF, SET);
    __XSPI_CLEAR_FLAG(instance, XSPI_SR_TCF);

    return SUCCESS;
}

typedef struct
{
    // CR register contains the all-important FMODE bits.
    uint32_t CR;

    // flash chip specific configuration set by the bootloader
    uint32_t CCR;
    uint32_t TCR;
    uint32_t IR;
    uint32_t ABR;
    // address register - no meaning.
    // data length register no meaning.

} xspiMemoryMappedModeConfigurationRegisterBackup_t;

xspiMemoryMappedModeConfigurationRegisterBackup_t xspiMMMCRBackups[XSPI_INTERFACE_COUNT];

static void xspiBackupMemoryMappedModeConfiguration(XSPI_TypeDef *instance)
{
    octoSpiDevice_e device = octoSpiDeviceByInstance((octoSpiResource_t *)instance);
    if (device == OCTOSPIINVALID) {
        return;
    }

    xspiMemoryMappedModeConfigurationRegisterBackup_t *xspiMMMCRBackup = &xspiMMMCRBackups[device];

    // backup all the registers used by memory mapped mode that:
    // a) the bootloader configured.
    // b) that other code in this implementation may have modified when memory mapped mode is disabled.

    xspiMMMCRBackup->CR = instance->CR;
    xspiMMMCRBackup->IR = instance->IR;
    xspiMMMCRBackup->CCR = instance->CCR;
    xspiMMMCRBackup->TCR = instance->TCR;
    xspiMMMCRBackup->ABR = instance->ABR;
}

static MMFLASH_CODE_NOINLINE void xspiRestoreMemoryMappedModeConfiguration(XSPI_TypeDef *instance)
{
    octoSpiDevice_e device = octoSpiDeviceByInstance((octoSpiResource_t *)instance);
    if (device == OCTOSPIINVALID) {
        return;
    }

    xspiMemoryMappedModeConfigurationRegisterBackup_t *xspiMMMCRBackup = &xspiMMMCRBackups[device];

    xspiWaitStatusFlags(instance, XSPI_SR_BUSY, RESET);

    instance->ABR = xspiMMMCRBackup->ABR;
    instance->TCR = xspiMMMCRBackup->TCR;

    instance->DLR = 0; // "no meaning" in MM mode.

    instance->CCR = xspiMMMCRBackup->CCR;

    instance->IR = xspiMMMCRBackup->IR;
    instance->AR = 0; // "no meaning" in MM mode.

    xspiAbort(instance);
    xspiWaitStatusFlags(instance, XSPI_SR_BUSY, RESET);

    instance->CR = xspiMMMCRBackup->CR;
}

/*
 * Disable memory mapped mode.
 *
 * @See octoSpiEnableMemoryMappedMode
 * @See MMFLASH_CODE_NOINLINE
 *
 * Once this is called any code or data in the memory mapped region cannot be accessed.
 * Thus, this function itself must be in RAM, and the caller's code and data should all be in RAM
 * and this requirement continues until octoSpiEnableMemoryMappedMode is called.
 * This applies to ISR code that runs from the memory mapped region, so likely the caller should
 * also disable IRQs before calling this.
 */
MMFLASH_CODE_NOINLINE void octoSpiDisableMemoryMappedMode(octoSpiResource_t *instance_)
{
    XSPI_TypeDef *instance = (XSPI_TypeDef *)instance_;
    if (READ_BIT(instance->CR, XSPI_CR_FMODE) != XSPI_CR_FMODE) {
        failureMode(FAILURE_DEVELOPER); // likely not booted with memory mapped mode enabled, or mismatched calls to enable/disable memory map mode.
    }

    xspiAbort(instance);
    if (__XSPI_GET_FLAG(instance, XSPI_SR_BUSY) == SET) {

        __XSPI_DISABLE(instance);
        xspiAbort(instance);
    }
    xspiWaitStatusFlags(instance, XSPI_SR_BUSY, RESET);

    uint32_t fmode = 0x0;  // b00 = indirect write, see XSPI->CR->FMODE
    MODIFY_REG(instance->CR, XSPI_CR_FMODE, fmode);

    uint32_t regval = READ_REG(instance->CR);
    if ((regval & XSPI_CR_FMODE) != fmode) {
        Error_Handler();
    }

    if (!__XSPI_IS_ENABLED(instance)) {
        __XSPI_ENABLE(instance);
    }
}

/*
 * Enable memory mapped mode.
 *
 * @See octoSpiDisableMemoryMappedMode
 * @See MMFLASH_CODE_NOINLINE
 */

MMFLASH_CODE_NOINLINE void octoSpiEnableMemoryMappedMode(octoSpiResource_t *instance_)
{
    XSPI_TypeDef *instance = (XSPI_TypeDef *)instance_;
    xspiAbort(instance);
    xspiWaitStatusFlags(instance, XSPI_SR_BUSY, RESET);

    xspiRestoreMemoryMappedModeConfiguration(instance);
}

static MMFLASH_CODE_NOINLINE void xspiTestEnableDisableMemoryMappedMode(octoSpiDevice_t *octoSpi)
{
    octoSpiResource_t *instance = octoSpi->dev;

    __disable_irq();
    octoSpiDisableMemoryMappedMode(instance);
    octoSpiEnableMemoryMappedMode(instance);
    __enable_irq();
}

static const uint32_t xspi_addressSizeMap[] = {
    XSPI_ADDRESS_8_BITS,
    XSPI_ADDRESS_16_BITS,
    XSPI_ADDRESS_24_BITS,
    XSPI_ADDRESS_32_BITS
};

MMFLASH_CODE static uint32_t xspi_addressSizeFromValue(uint8_t addressSize)
{
    return xspi_addressSizeMap[((addressSize + 1) / 8) - 1]; // rounds to nearest XSPI_ADDRESS_* value that will hold the address.
}

MMFLASH_CODE_NOINLINE bool octoSpiTransmit1LINE(octoSpiResource_t *instance_, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{
    XSPI_TypeDef *instance = (XSPI_TypeDef *)instance_;
    XSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = XSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = XSPI_INSTRUCTION_8_BITS;

    cmd.AddressDtrMode     = XSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = XSPI_ADDRESS_NONE;
    cmd.AddressSize        = XSPI_ADDRESS_32_BITS;

    cmd.DummyCycles        = dummyCycles;

    cmd.AlternateBytesMode = XSPI_ALTERNATE_BYTES_NONE;

    cmd.DataDtrMode        = XSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = XSPI_DATA_NONE;
    cmd.NbData             = length;

    cmd.DQSMode            = XSPI_DQS_DISABLE;
    cmd.SIOOMode           = XSPI_SIOO_INST_EVERY_CMD;

    if (out) {
        cmd.DataMode       = XSPI_DATA_1_LINE;
    }

    ErrorStatus status = xspiCommand(instance, &cmd);

    if (status == SUCCESS && out && length > 0) {
        status = xspiTransmit(instance, (uint8_t *)out);
    }
    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceive1LINE(octoSpiResource_t *instance_, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    XSPI_TypeDef *instance = (XSPI_TypeDef *)instance_;
    XSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = XSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = XSPI_INSTRUCTION_8_BITS;

    cmd.AddressDtrMode     = XSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = XSPI_ADDRESS_NONE;
    cmd.AddressSize        = XSPI_ADDRESS_32_BITS;

    cmd.AlternateBytesMode = XSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = XSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = XSPI_DATA_1_LINE;
    cmd.NbData             = length;

    cmd.DQSMode            = XSPI_DQS_DISABLE;
    cmd.SIOOMode           = XSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = xspiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = xspiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceive4LINES(octoSpiResource_t *instance_, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    XSPI_TypeDef *instance = (XSPI_TypeDef *)instance_;
    XSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = XSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = XSPI_INSTRUCTION_4_LINES;
    cmd.InstructionSize    = XSPI_INSTRUCTION_8_BITS;

    cmd.AddressDtrMode     = XSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = XSPI_ADDRESS_NONE;
    cmd.AddressSize        = XSPI_ADDRESS_32_BITS;

    cmd.AlternateBytesMode = XSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = XSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = XSPI_DATA_4_LINES;
    cmd.NbData             = length;

    cmd.DQSMode            = XSPI_DQS_DISABLE;
    cmd.SIOOMode           = XSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = xspiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = xspiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceiveWithAddress1LINE(octoSpiResource_t *instance_, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    XSPI_TypeDef *instance = (XSPI_TypeDef *)instance_;
    XSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = XSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = XSPI_INSTRUCTION_8_BITS;

    cmd.Address            = address;
    cmd.AddressDtrMode     = XSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = XSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = xspi_addressSizeFromValue(addressSize);

    cmd.AlternateBytesMode = XSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = XSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = XSPI_DATA_1_LINE;
    cmd.NbData             = length;

    cmd.SIOOMode           = XSPI_SIOO_INST_EVERY_CMD;
    cmd.DQSMode            = XSPI_DQS_DISABLE;

    ErrorStatus status = xspiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = xspiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceiveWithAddress4LINES(octoSpiResource_t *instance_, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    XSPI_TypeDef *instance = (XSPI_TypeDef *)instance_;
    XSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = XSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = XSPI_INSTRUCTION_8_BITS;

    cmd.Address            = address;
    cmd.AddressDtrMode     = XSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = XSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = xspi_addressSizeFromValue(addressSize);

    cmd.AlternateBytesMode = XSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = XSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = XSPI_DATA_4_LINES;
    cmd.NbData             = length;

    cmd.DQSMode            = XSPI_DQS_DISABLE;
    cmd.SIOOMode           = XSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = xspiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = xspiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiTransmitWithAddress1LINE(octoSpiResource_t *instance_, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    XSPI_TypeDef *instance = (XSPI_TypeDef *)instance_;
    XSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = XSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = XSPI_INSTRUCTION_8_BITS;

    cmd.Address            = address;
    cmd.AddressDtrMode     = XSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = XSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = xspi_addressSizeFromValue(addressSize);

    cmd.AlternateBytesMode = XSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = XSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = XSPI_DATA_1_LINE;
    cmd.NbData             = length;

    cmd.DQSMode            = XSPI_DQS_DISABLE;
    cmd.SIOOMode           = XSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = xspiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = xspiTransmit(instance, (uint8_t *)out);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiTransmitWithAddress4LINES(octoSpiResource_t *instance_, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    XSPI_TypeDef *instance = (XSPI_TypeDef *)instance_;
    XSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = XSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = XSPI_INSTRUCTION_8_BITS;

    cmd.Address            = address;
    cmd.AddressDtrMode     = XSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = XSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = xspi_addressSizeFromValue(addressSize);

    cmd.AlternateBytesMode = XSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = XSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = XSPI_DATA_4_LINES;
    cmd.NbData             = length;

    cmd.DQSMode            = XSPI_DQS_DISABLE;
    cmd.SIOOMode           = XSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = xspiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = xspiTransmit(instance, (uint8_t *)out);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiInstructionWithAddress1LINE(octoSpiResource_t *instance_, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize)
{
    XSPI_TypeDef *instance = (XSPI_TypeDef *)instance_;
    XSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = XSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = XSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = XSPI_INSTRUCTION_8_BITS;

    cmd.Address            = address;
    cmd.AddressDtrMode     = XSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = XSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = xspi_addressSizeFromValue(addressSize);

    cmd.AlternateBytesMode = XSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = XSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = XSPI_DATA_NONE;
    cmd.NbData             = 0;

    cmd.DQSMode            = XSPI_DQS_DISABLE;
    cmd.SIOOMode           = XSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = xspiCommand(instance, &cmd);

    return status == SUCCESS;
}

void octoSpiInitDevice(octoSpiDevice_e device)
{
    octoSpiDevice_t *octoSpi = &(octoSpiDevice[device]);

    if (isMemoryMappedModeEnabledOnBoot()) {
        // Bootloader has already configured the IO, clocks and peripherals.
        xspiBackupMemoryMappedModeConfiguration((XSPI_TypeDef *)octoSpi->dev);

        xspiTestEnableDisableMemoryMappedMode(octoSpi);
    } else {
        failureMode(FAILURE_DEVELOPER); // trying to use this implementation when memory mapped mode is not already enabled by a bootloader
    }
}

#endif
