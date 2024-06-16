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
 *
 * Author: Dominic Clifton
 */

/*
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OCTOSPI

#include "drivers/system.h"

#include "drivers/bus_octospi.h"
#include "drivers/bus_octospi_impl.h"

#if !(defined(STM32H730xx) || defined(STM32H723xx))
#error MCU not supported.
#endif

#define OCTOSPI_INTERFACE_COUNT         1

#define OSPI_FUNCTIONAL_MODE_INDIRECT_WRITE ((uint32_t)0x00000000)
#define OSPI_FUNCTIONAL_MODE_INDIRECT_READ  ((uint32_t)OCTOSPI_CR_FMODE_0)
#define OSPI_FUNCTIONAL_MODE_AUTO_POLLING   ((uint32_t)OCTOSPI_CR_FMODE_1)
#define OSPI_FUNCTIONAL_MODE_MEMORY_MAPPED  ((uint32_t)OCTOSPI_CR_FMODE)

#define OSPI_DHQC_DISABLE                ((uint32_t)0x00000000U)
#define OSPI_DHQC_ENABLE                 ((uint32_t)OCTOSPI_TCR_DHQC)

#define OSPI_OPTYPE_COMMON_CFG           ((uint32_t)0x00000000U)

#define OSPI_OPTYPE_READ_CFG             ((uint32_t)0x00000001U)
#define OSPI_OPTYPE_WRITE_CFG            ((uint32_t)0x00000002U)
#define OSPI_OPTYPE_WRAP_CFG             ((uint32_t)0x00000003U)

#define OSPI_INSTRUCTION_NONE            ((uint32_t)0x00000000U)
#define OSPI_INSTRUCTION_1_LINE          ((uint32_t)OCTOSPI_CCR_IMODE_0)
#define OSPI_INSTRUCTION_2_LINES         ((uint32_t)OCTOSPI_CCR_IMODE_1)
#define OSPI_INSTRUCTION_4_LINES         ((uint32_t)(OCTOSPI_CCR_IMODE_0 | OCTOSPI_CCR_IMODE_1))
#define OSPI_INSTRUCTION_8_LINES         ((uint32_t)OCTOSPI_CCR_IMODE_2)

#define OSPI_INSTRUCTION_8_BITS          ((uint32_t)0x00000000U)
#define OSPI_INSTRUCTION_16_BITS         ((uint32_t)OCTOSPI_CCR_ISIZE_0)
#define OSPI_INSTRUCTION_24_BITS         ((uint32_t)OCTOSPI_CCR_ISIZE_1)
#define OSPI_INSTRUCTION_32_BITS         ((uint32_t)OCTOSPI_CCR_ISIZE)

#define OSPI_INSTRUCTION_DTR_DISABLE     ((uint32_t)0x00000000U)
#define OSPI_INSTRUCTION_DTR_ENABLE      ((uint32_t)OCTOSPI_CCR_IDTR)

#define OSPI_ADDRESS_NONE                ((uint32_t)0x00000000U)                                         /*!< No address               */
#define OSPI_ADDRESS_1_LINE              ((uint32_t)OCTOSPI_CCR_ADMODE_0)                                /*!< Address on a single line */
#define OSPI_ADDRESS_2_LINES             ((uint32_t)OCTOSPI_CCR_ADMODE_1)                                /*!< Address on two lines     */
#define OSPI_ADDRESS_4_LINES             ((uint32_t)(OCTOSPI_CCR_ADMODE_0 | OCTOSPI_CCR_ADMODE_1))       /*!< Address on four lines    */
#define OSPI_ADDRESS_8_LINES             ((uint32_t)OCTOSPI_CCR_ADMODE_2)                                /*!< Address on eight lines   */

#define OSPI_ADDRESS_8_BITS              ((uint32_t)0x00000000U)                                         /*!< 8-bit address  */
#define OSPI_ADDRESS_16_BITS             ((uint32_t)OCTOSPI_CCR_ADSIZE_0)                                /*!< 16-bit address */
#define OSPI_ADDRESS_24_BITS             ((uint32_t)OCTOSPI_CCR_ADSIZE_1)                                /*!< 24-bit address */
#define OSPI_ADDRESS_32_BITS             ((uint32_t)OCTOSPI_CCR_ADSIZE)                                  /*!< 32-bit address */

#define OSPI_ADDRESS_DTR_DISABLE         ((uint32_t)0x00000000U)                                         /*!< DTR mode disabled for address phase */
#define OSPI_ADDRESS_DTR_ENABLE          ((uint32_t)OCTOSPI_CCR_ADDTR)

#define OSPI_DATA_NONE                   ((uint32_t)0x00000000U)
#define OSPI_DATA_1_LINE                 ((uint32_t)OCTOSPI_CCR_DMODE_0)
#define OSPI_DATA_2_LINES                ((uint32_t)OCTOSPI_CCR_DMODE_1)
#define OSPI_DATA_4_LINES                ((uint32_t)(OCTOSPI_CCR_DMODE_0 | OCTOSPI_CCR_DMODE_1))
#define OSPI_DATA_8_LINES                ((uint32_t)OCTOSPI_CCR_DMODE_2)

#define OSPI_DATA_DTR_DISABLE            ((uint32_t)0x00000000U)
#define OSPI_DATA_DTR_ENABLE             ((uint32_t)OCTOSPI_CCR_DDTR)

#define OSPI_ALTERNATE_BYTES_NONE        ((uint32_t)0x00000000U)
#define OSPI_ALTERNATE_BYTES_1_LINE      ((uint32_t)OCTOSPI_CCR_ABMODE_0)
#define OSPI_ALTERNATE_BYTES_2_LINES     ((uint32_t)OCTOSPI_CCR_ABMODE_1)
#define OSPI_ALTERNATE_BYTES_4_LINES     ((uint32_t)(OCTOSPI_CCR_ABMODE_0 | OCTOSPI_CCR_ABMODE_1))
#define OSPI_ALTERNATE_BYTES_8_LINES     ((uint32_t)OCTOSPI_CCR_ABMODE_2)

#define OSPI_ALTERNATE_BYTES_8_BITS      ((uint32_t)0x00000000U)
#define OSPI_ALTERNATE_BYTES_16_BITS     ((uint32_t)OCTOSPI_CCR_ABSIZE_0)
#define OSPI_ALTERNATE_BYTES_24_BITS     ((uint32_t)OCTOSPI_CCR_ABSIZE_1)
#define OSPI_ALTERNATE_BYTES_32_BITS     ((uint32_t)OCTOSPI_CCR_ABSIZE)

#define OSPI_ALTERNATE_BYTES_DTR_DISABLE ((uint32_t)0x00000000U)
#define OSPI_ALTERNATE_BYTES_DTR_ENABLE  ((uint32_t)OCTOSPI_CCR_ABDTR)

#define OSPI_DQS_DISABLE                 ((uint32_t)0x00000000U)
#define OSPI_DQS_ENABLE                  ((uint32_t)OCTOSPI_CCR_DQSE)

#define OSPI_SIOO_INST_EVERY_CMD         ((uint32_t)0x00000000U)
#define OSPI_SIOO_INST_ONLY_FIRST_CMD    ((uint32_t)OCTOSPI_CCR_SIOO)

MMFLASH_CODE_NOINLINE static void Error_Handler(void) {
    while (1) {
        NOOP;
    }
}


#define __OSPI_GET_FLAG(__INSTANCE__, __FLAG__)           ((READ_BIT((__INSTANCE__)->SR, (__FLAG__)) != 0U) ? SET : RESET)
#define __OSPI_CLEAR_FLAG(__INSTANCE__, __FLAG__)           WRITE_REG((__INSTANCE__)->FCR, (__FLAG__))
#define __OSPI_ENABLE(__INSTANCE__)                       SET_BIT((__INSTANCE__)->CR, OCTOSPI_CR_EN)
#define __OSPI_DISABLE(__INSTANCE__)                      CLEAR_BIT((__INSTANCE__)->CR, OCTOSPI_CR_EN)
#define __OSPI_IS_ENABLED(__INSTANCE__)                   (READ_BIT((__INSTANCE__)->CR, OCTOSPI_CR_EN) != 0U)

MMFLASH_CODE_NOINLINE static void octoSpiAbort(OCTOSPI_TypeDef *instance)
{
    SET_BIT(instance->CR, OCTOSPI_CR_ABORT);
}

MMFLASH_CODE_NOINLINE static void octoSpiWaitStatusFlags(OCTOSPI_TypeDef *instance, uint32_t mask, FlagStatus flagStatus)
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
} OSPI_Command_t;

// TODO rename cmd to command
MMFLASH_CODE_NOINLINE static ErrorStatus octoSpiConfigureCommand(OCTOSPI_TypeDef *instance, OSPI_Command_t *cmd)
{
    ErrorStatus status = SUCCESS;

    MODIFY_REG(instance->CR, OCTOSPI_CR_FMODE, 0U);

    instance->CCR = (cmd->DQSMode | cmd->SIOOMode);

    if (cmd->AlternateBytesMode != OSPI_ALTERNATE_BYTES_NONE)
    {
        instance->ABR = cmd->AlternateBytes;

        MODIFY_REG(
            instance->ABR,
            (OCTOSPI_CCR_ABMODE | OCTOSPI_CCR_ABDTR | OCTOSPI_CCR_ABSIZE),
            (cmd->AlternateBytesMode | cmd->AlternateBytesDtrMode | cmd->AlternateBytesSize)
        );
    }

    MODIFY_REG(instance->TCR, OCTOSPI_TCR_DCYC, cmd->DummyCycles);

    if (cmd->DataMode != OSPI_DATA_NONE)
    {
      if (cmd->OperationType == OSPI_OPTYPE_COMMON_CFG)
      {
        instance->DLR = (cmd->NbData - 1U);
      }
    }


    if (cmd->InstructionMode != OSPI_INSTRUCTION_NONE)
    {
      if (cmd->AddressMode != OSPI_ADDRESS_NONE)
      {
        if (cmd->DataMode != OSPI_DATA_NONE)
        {
          // instruction, address and data

          MODIFY_REG(instance->CCR, (OCTOSPI_CCR_IMODE  | OCTOSPI_CCR_IDTR  | OCTOSPI_CCR_ISIZE  |
                                  OCTOSPI_CCR_ADMODE | OCTOSPI_CCR_ADDTR | OCTOSPI_CCR_ADSIZE |
                                  OCTOSPI_CCR_DMODE  | OCTOSPI_CCR_DDTR),
                                 (cmd->InstructionMode | cmd->InstructionDtrMode | cmd->InstructionSize |
                                  cmd->AddressMode     | cmd->AddressDtrMode     | cmd->AddressSize     |
                                  cmd->DataMode        | cmd->DataDtrMode));
        }
        else
        {
          // instruction and address

          MODIFY_REG(instance->CCR, (OCTOSPI_CCR_IMODE  | OCTOSPI_CCR_IDTR  | OCTOSPI_CCR_ISIZE  |
                                  OCTOSPI_CCR_ADMODE | OCTOSPI_CCR_ADDTR | OCTOSPI_CCR_ADSIZE),
                                 (cmd->InstructionMode | cmd->InstructionDtrMode | cmd->InstructionSize |
                                  cmd->AddressMode     | cmd->AddressDtrMode     | cmd->AddressSize));

          // DHQC bit is linked with DDTR
          if (((instance->TCR & OCTOSPI_TCR_DHQC_Msk) == OSPI_DHQC_ENABLE) &&
              (cmd->InstructionDtrMode == OSPI_INSTRUCTION_DTR_ENABLE))
          {
            MODIFY_REG(instance->CCR, OCTOSPI_CCR_DDTR, OSPI_DATA_DTR_ENABLE);
          }
        }

        instance->IR = cmd->Instruction;

        instance->AR = cmd->Address;
      }
      else
      {
        if (cmd->DataMode != OSPI_DATA_NONE)
        {
          // instruction and data

          MODIFY_REG(instance->CCR, (OCTOSPI_CCR_IMODE | OCTOSPI_CCR_IDTR | OCTOSPI_CCR_ISIZE |
                                  OCTOSPI_CCR_DMODE | OCTOSPI_CCR_DDTR),
                                 (cmd->InstructionMode | cmd->InstructionDtrMode | cmd->InstructionSize |
                                  cmd->DataMode        | cmd->DataDtrMode));
        }
        else
        {
          // instruction only

          MODIFY_REG(instance->CCR, (OCTOSPI_CCR_IMODE | OCTOSPI_CCR_IDTR | OCTOSPI_CCR_ISIZE),
                                 (cmd->InstructionMode | cmd->InstructionDtrMode | cmd->InstructionSize));

          // DHQC bit is linked with DDTR
          if (((instance->TCR & OCTOSPI_TCR_DHQC_Msk) == OSPI_DHQC_ENABLE) &&
              (cmd->InstructionDtrMode == OSPI_INSTRUCTION_DTR_ENABLE))
          {
            MODIFY_REG(instance->CCR, OCTOSPI_CCR_DDTR, OSPI_DATA_DTR_ENABLE);
          }
        }

        instance->IR = cmd->Instruction;

      }
    }
    else
    {
      if (cmd->AddressMode != OSPI_ADDRESS_NONE)
      {
        if (cmd->DataMode != OSPI_DATA_NONE)
        {
          // address and data

          MODIFY_REG(instance->CCR, (OCTOSPI_CCR_ADMODE | OCTOSPI_CCR_ADDTR | OCTOSPI_CCR_ADSIZE |
                                  OCTOSPI_CCR_DMODE  | OCTOSPI_CCR_DDTR),
                                 (cmd->AddressMode | cmd->AddressDtrMode | cmd->AddressSize     |
                                  cmd->DataMode    | cmd->DataDtrMode));
        }
        else
        {
          // address only

          MODIFY_REG(instance->CCR, (OCTOSPI_CCR_ADMODE | OCTOSPI_CCR_ADDTR | OCTOSPI_CCR_ADSIZE),
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

MMFLASH_CODE_NOINLINE ErrorStatus octoSpiCommand(OCTOSPI_TypeDef *instance, OSPI_Command_t *cmd)
{
    octoSpiWaitStatusFlags(instance, OCTOSPI_SR_BUSY, RESET);

    ErrorStatus status = octoSpiConfigureCommand(instance, cmd);
    if (status == SUCCESS) {
        if (cmd->DataMode == OSPI_DATA_NONE)
        {
            // transfer is already started, wait now.
            octoSpiWaitStatusFlags(instance, OCTOSPI_SR_TCF, SET);
            __OSPI_CLEAR_FLAG(instance, OCTOSPI_SR_TCF);
        }
    }

    return status;
}

/*
 * Transmit
 *
 * Call optoSpiCommand first to configure the transaction stages.
 */
MMFLASH_CODE_NOINLINE ErrorStatus octoSpiTransmit(OCTOSPI_TypeDef *instance, uint8_t *data)
{
    if (data == NULL) {
        return ERROR;
    }


    __IO uint32_t              XferCount = READ_REG(instance->DLR) + 1U;
    uint8_t                    *pBuffPtr = data;

    MODIFY_REG(instance->CR, OCTOSPI_CR_FMODE, OSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

    __IO uint32_t *data_reg = &instance->DR;
    do
    {
      octoSpiWaitStatusFlags(instance, OCTOSPI_SR_FTF, SET);

      *((__IO uint8_t *)data_reg) = *pBuffPtr;
      pBuffPtr++;
      XferCount--;
    } while (XferCount > 0U);

    octoSpiWaitStatusFlags(instance, OCTOSPI_SR_TCF, SET);
    __OSPI_CLEAR_FLAG(instance, OCTOSPI_SR_TCF);

    return SUCCESS;
}

/*
 * Receive
 *
 * Call optoSpiCommand first to configure the transaction stages.
 */
MMFLASH_CODE_NOINLINE ErrorStatus octoSpiReceive(OCTOSPI_TypeDef *instance, uint8_t *data)
{
    if (data == NULL) {
        return ERROR;
    }

    __IO uint32_t              XferCount = READ_REG(instance->DLR) + 1U;
    uint8_t                    *pBuffPtr = data;

    MODIFY_REG(instance->CR, OCTOSPI_CR_FMODE, OSPI_FUNCTIONAL_MODE_INDIRECT_READ);

    uint32_t addr_reg = instance->AR;
    uint32_t ir_reg = instance->IR;

    // Trigger the transfer by re-writing the address or instruction register
    if (READ_BIT(instance->CCR, OCTOSPI_CCR_ADMODE) != OSPI_ADDRESS_NONE)
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
      octoSpiWaitStatusFlags(instance, OCTOSPI_SR_FTF | OCTOSPI_SR_TCF, SET);

      *pBuffPtr = *((__IO uint8_t *)data_reg);
      pBuffPtr++;
      XferCount--;
    } while(XferCount > 0U);

    octoSpiWaitStatusFlags(instance, OCTOSPI_SR_TCF, SET);
    __OSPI_CLEAR_FLAG(instance, OCTOSPI_SR_TCF);

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

} octoSpiMemoryMappedModeConfigurationRegisterBackup_t;

octoSpiMemoryMappedModeConfigurationRegisterBackup_t ospiMMMCRBackups[OCTOSPI_INTERFACE_COUNT];

void octoSpiBackupMemoryMappedModeConfiguration(OCTOSPI_TypeDef *instance)
{
    OCTOSPIDevice device = octoSpiDeviceByInstance(instance);
    if (device == OCTOSPIINVALID) {
        return;
    }

    octoSpiMemoryMappedModeConfigurationRegisterBackup_t *ospiMMMCRBackup = &ospiMMMCRBackups[device];

    // backup all the registers used by memory mapped mode that:
    // a) the bootloader configured.
    // b) that other code in this implementation may have modified when memory mapped mode is disabled.

    ospiMMMCRBackup->CR = instance->CR;
    ospiMMMCRBackup->IR = instance->IR;
    ospiMMMCRBackup->CCR = instance->CCR;
    ospiMMMCRBackup->TCR = instance->TCR;
    ospiMMMCRBackup->ABR = instance->ABR;
}

MMFLASH_CODE_NOINLINE void octoSpiRestoreMemoryMappedModeConfiguration(OCTOSPI_TypeDef *instance)
{
    OCTOSPIDevice device = octoSpiDeviceByInstance(instance);
    if (device == OCTOSPIINVALID) {
        return;
    }

    octoSpiMemoryMappedModeConfigurationRegisterBackup_t *ospiMMMCRBackup = &ospiMMMCRBackups[device];

    octoSpiWaitStatusFlags(instance, OCTOSPI_SR_BUSY, RESET);

    instance->ABR = ospiMMMCRBackup->ABR;
    instance->TCR = ospiMMMCRBackup->TCR;

    instance->DLR = 0; // "no meaning" in MM mode.

    instance->CCR = ospiMMMCRBackup->CCR;

    instance->IR = ospiMMMCRBackup->IR;
    instance->AR = 0; // "no meaning" in MM mode.

    octoSpiAbort(instance);
    octoSpiWaitStatusFlags(instance, OCTOSPI_SR_BUSY, RESET);

    instance->CR = ospiMMMCRBackup->CR;
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
MMFLASH_CODE_NOINLINE void octoSpiDisableMemoryMappedMode(OCTOSPI_TypeDef *instance)
{
    if (READ_BIT(OCTOSPI1->CR, OCTOSPI_CR_FMODE) != OCTOSPI_CR_FMODE) {
        failureMode(FAILURE_DEVELOPER); // likely not booted with memory mapped mode enabled, or mismatched calls to enable/disable memory map mode.
    }

    octoSpiAbort(instance);
    if (__OSPI_GET_FLAG(instance, OCTOSPI_SR_BUSY) == SET) {

        __OSPI_DISABLE(instance);
        octoSpiAbort(instance);
    }
    octoSpiWaitStatusFlags(instance, OCTOSPI_SR_BUSY, RESET);

    uint32_t fmode = 0x0;  // b00 = indirect write, see OCTOSPI->CR->FMODE
    MODIFY_REG(instance->CR, OCTOSPI_CR_FMODE, fmode);

    uint32_t regval = READ_REG(instance->CR);
    if ((regval & OCTOSPI_CR_FMODE) != fmode) {
        Error_Handler();
    }

    if (!__OSPI_IS_ENABLED(instance)) {
        __OSPI_ENABLE(instance);
    }
}

/*
 * Enable memory mapped mode.
 *
 * @See octoSpiDisableMemoryMappedMode
 * @See MMFLASH_CODE_NOINLINE
 */

MMFLASH_CODE_NOINLINE void octoSpiEnableMemoryMappedMode(OCTOSPI_TypeDef *instance)
{
    octoSpiAbort(instance);
    octoSpiWaitStatusFlags(instance, OCTOSPI_SR_BUSY, RESET);

    octoSpiRestoreMemoryMappedModeConfiguration(instance);
}

MMFLASH_CODE_NOINLINE void octoSpiTestEnableDisableMemoryMappedMode(octoSpiDevice_t *octoSpi)
{
    OCTOSPI_TypeDef *instance = octoSpi->dev;

    __disable_irq();
    octoSpiDisableMemoryMappedMode(instance);
    octoSpiEnableMemoryMappedMode(instance);
    __enable_irq();
}

MMFLASH_DATA static const uint32_t octoSpi_addressSizeMap[] = {
    OSPI_ADDRESS_8_BITS,
    OSPI_ADDRESS_16_BITS,
    OSPI_ADDRESS_24_BITS,
    OSPI_ADDRESS_32_BITS
};

MMFLASH_CODE static uint32_t octoSpi_addressSizeFromValue(uint8_t addressSize)
{
    return octoSpi_addressSizeMap[((addressSize + 1) / 8) - 1]; // rounds to nearest OSPI_ADDRESS_* value that will hold the address.
}


MMFLASH_CODE_NOINLINE bool octoSpiTransmit1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{
    OSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = OSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = OSPI_INSTRUCTION_8_BITS;

    cmd.AddressDtrMode     = OSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = OSPI_ADDRESS_NONE;
    cmd.AddressSize        = OSPI_ADDRESS_32_BITS;

    cmd.DummyCycles        = dummyCycles;

    cmd.AlternateBytesMode = OSPI_ALTERNATE_BYTES_NONE;

    cmd.DataDtrMode        = OSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = OSPI_DATA_NONE;
    cmd.NbData             = length;

    cmd.DQSMode            = OSPI_DQS_DISABLE;
    cmd.SIOOMode           = OSPI_SIOO_INST_EVERY_CMD;

    if (out) {
        cmd.DataMode       = OSPI_DATA_1_LINE;
    }

    ErrorStatus status = octoSpiCommand(instance, &cmd);

    if (status == SUCCESS && out && length > 0) {
        status = octoSpiTransmit(instance, (uint8_t *)out);
    }
    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceive1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    OSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = OSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = OSPI_INSTRUCTION_8_BITS;

    cmd.AddressDtrMode     = OSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = OSPI_ADDRESS_NONE;
    cmd.AddressSize        = OSPI_ADDRESS_32_BITS;

    cmd.AlternateBytesMode = OSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = OSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = OSPI_DATA_1_LINE;
    cmd.NbData             = length;

    cmd.DQSMode            = OSPI_DQS_DISABLE;
    cmd.SIOOMode           = OSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = octoSpiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = octoSpiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceive4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    OSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = OSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = OSPI_INSTRUCTION_4_LINES;
    cmd.InstructionSize    = OSPI_INSTRUCTION_8_BITS;

    cmd.AddressDtrMode     = OSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = OSPI_ADDRESS_NONE;
    cmd.AddressSize        = OSPI_ADDRESS_32_BITS;

    cmd.AlternateBytesMode = OSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = OSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = OSPI_DATA_4_LINES;
    cmd.NbData             = length;

    cmd.DQSMode            = OSPI_DQS_DISABLE;
    cmd.SIOOMode           = OSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = octoSpiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = octoSpiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceiveWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    OSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = OSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = OSPI_INSTRUCTION_8_BITS;

    cmd.Address            = address;
    cmd.AddressDtrMode     = OSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = octoSpi_addressSizeFromValue(addressSize);

    cmd.AlternateBytesMode = OSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = OSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = OSPI_DATA_1_LINE;
    cmd.NbData             = length;

    cmd.SIOOMode           = OSPI_SIOO_INST_EVERY_CMD;
    cmd.DQSMode            = OSPI_DQS_DISABLE;

    ErrorStatus status = octoSpiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = octoSpiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceiveWithAddress4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    OSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = OSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = OSPI_INSTRUCTION_8_BITS;

    cmd.Address            = address;
    cmd.AddressDtrMode     = OSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = octoSpi_addressSizeFromValue(addressSize);

    cmd.AlternateBytesMode = OSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = OSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = OSPI_DATA_4_LINES;
    cmd.NbData             = length;

    cmd.DQSMode            = OSPI_DQS_DISABLE;
    cmd.SIOOMode           = OSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = octoSpiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = octoSpiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiTransmitWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    OSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = OSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = OSPI_INSTRUCTION_8_BITS;

    cmd.Address            = address;
    cmd.AddressDtrMode     = OSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = octoSpi_addressSizeFromValue(addressSize);

    cmd.AlternateBytesMode = OSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = OSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = OSPI_DATA_1_LINE;
    cmd.NbData             = length;

    cmd.DQSMode            = OSPI_DQS_DISABLE;
    cmd.SIOOMode           = OSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = octoSpiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = octoSpiTransmit(instance, (uint8_t *)out);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiTransmitWithAddress4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    OSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = OSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = OSPI_INSTRUCTION_8_BITS;

    cmd.Address            = address;
    cmd.AddressDtrMode     = OSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = octoSpi_addressSizeFromValue(addressSize);

    cmd.AlternateBytesMode = OSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = OSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = OSPI_DATA_4_LINES;
    cmd.NbData             = length;

    cmd.DQSMode            = OSPI_DQS_DISABLE;
    cmd.SIOOMode           = OSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = octoSpiCommand(instance, &cmd);

    if (status == SUCCESS) {
        status = octoSpiTransmit(instance, (uint8_t *)out);
    }


    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiInstructionWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize)
{
    OSPI_Command_t cmd; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd.OperationType      = OSPI_OPTYPE_COMMON_CFG;

    cmd.Instruction        = instruction;
    cmd.InstructionDtrMode = OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.InstructionMode    = OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = OSPI_INSTRUCTION_8_BITS;

    cmd.Address            = address;
    cmd.AddressDtrMode     = OSPI_ADDRESS_DTR_DISABLE;
    cmd.AddressMode        = OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = octoSpi_addressSizeFromValue(addressSize);

    cmd.AlternateBytesMode = OSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles        = dummyCycles;

    cmd.DataDtrMode        = OSPI_DATA_DTR_DISABLE;
    cmd.DataMode           = OSPI_DATA_NONE;
    cmd.NbData             = 0;

    cmd.DQSMode            = OSPI_DQS_DISABLE;
    cmd.SIOOMode           = OSPI_SIOO_INST_EVERY_CMD;

    ErrorStatus status = octoSpiCommand(instance, &cmd);

    return status == SUCCESS;
}


void octoSpiInitDevice(OCTOSPIDevice device)
{
    octoSpiDevice_t *octoSpi = &(octoSpiDevice[device]);

#if defined(STM32H730xx) || defined(STM32H723xx)
    if (isMemoryMappedModeEnabledOnBoot()) {
        // Bootloader has already configured the IO, clocks and peripherals.
        octoSpiBackupMemoryMappedModeConfiguration(octoSpi->dev);

        octoSpiTestEnableDisableMemoryMappedMode(octoSpi);
    } else {
        failureMode(FAILURE_DEVELOPER); // trying to use this implementation when memory mapped mode is not already enabled by a bootloader

        // Here is where we would configure the OCTOSPI1/2 and OCTOSPIM peripherals for the non-memory-mapped use case.
    }
#else
#error MCU not supported.
#endif

}

#endif
