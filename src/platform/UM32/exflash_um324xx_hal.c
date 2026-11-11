/**
  ******************************************************************************
  * @file     um324xx_hal_exflash.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-05-26
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "um324xx_hal.h"
#include "exflash_um324xx_hal.h"
#include "common/maths.h"
/** @addtogroup UM324xF_HAL_Driver
  * @{
  */

/** @defgroup GT25QXX GT25QXX
  * @brief HAL GT25QXX module driver
  * @{
  */

/** @defgroup GT25QXX_functions
  * @{
  */

#define QSPI_CONTROLLER_MAX_POLLS       100000U
#define QSPI_SR1_BUSY                   (1U << 0)
#define QSPI_SR2_QUAD_ENABLE            (1U << 1)
#define QSPI_STATUS_POLLS_PER_MS        10000U
#define QSPI_WRITE_TIMEOUT_MS           100U
#define QSPI_SECTOR_ERASE_TIMEOUT_MS    1000U
#define QSPI_CHIP_ERASE_TIMEOUT_MS      30000U
#define QSPI_STIG_MAX_BYTES             8U
#define QSPI_FLASH_PAGE_SIZE            256U

static HAL_StatusTypeDef qspiWaitRegister(const volatile uint32_t *reg, uint32_t mask, uint32_t expected)
{
    for (uint32_t poll = 0; poll < QSPI_CONTROLLER_MAX_POLLS; ++poll) {
        if ((*reg & mask) == expected) {
            return HAL_OK;
        }
    }
    return HAL_TIMEOUT;
}

// Issue a STIG command with bounded controller waits, including during XIP startup.
static HAL_StatusTypeDef qspiStigCommand(QSPI_HandleTypeDef *hqspi, uint8_t opcode, uint32_t flags,
    uint32_t address, const uint8_t *writeData, uint8_t *readData, uint32_t length)
{
    if (hqspi->State != HAL_QSPI_STATE_READY) {
        return HAL_BUSY;
    }

    __HAL_LOCK(hqspi);
    hqspi->State = HAL_QSPI_STATE_BUSY;

    QSPI_TypeDef *qspi = hqspi->Instance;
    const uint32_t busyMask = QSPI_FCR_CMDT | QSPI_FCR_CMDS;
    HAL_StatusTypeDef status = qspiWaitRegister(&qspi->CR, QSPI_CR_IDLES, QSPI_CR_IDLES);
    if (status == HAL_OK) {
        status = qspiWaitRegister(&qspi->FCR, busyMask, 0);
    }
    if (status == HAL_OK) {
        WRITE_REG(qspi->FCAR, address);
        if (writeData) {
            // The first byte occupies the least significant byte of the low word.
            uint32_t words[2] = {0};
            memcpy(words, writeData, length);
            WRITE_REG(qspi->FCWLR, words[0]);
            if (length > sizeof(words[0])) {
                WRITE_REG(qspi->FCWHR, words[1]);
            }
            flags |= QSPI_WREN_ENABLE | ((length - 1U) << QSPI_FCR_WD_NUM_Pos);
        }
        if (readData) {
            flags |= QSPI_RDEN_ENABLE | ((length - 1U) << QSPI_FCR_RD_NUM_Pos);
        }
        WRITE_REG(qspi->FCR, ((uint32_t)opcode << QSPI_FCR_OPCODE_Pos) | flags | QSPI_FCR_CMDT);
        status = qspiWaitRegister(&qspi->FCR, busyMask, 0);
        if (status == HAL_OK && readData) {
            const uint32_t words[2] = {
                READ_REG(qspi->FCRLR),
                length > sizeof(uint32_t) ? READ_REG(qspi->FCRHR) : 0,
            };
            memcpy(readData, words, length);
        }
    }

    hqspi->ErrorCode = status == HAL_OK ? HAL_QSPI_ERROR_NONE : HAL_QSPI_ERROR_TIMEOUT;
    hqspi->State = HAL_QSPI_STATE_READY;
    __HAL_UNLOCK(hqspi);
    return status;
}

/** @addtogroup GT25QXX_Exported_Functions GT25QXX Exported Functions
  * @{
  */

HAL_StatusTypeDef QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
    return qspiStigCommand(hqspi, CMD_WREN, 0, 0, NULL, NULL, 0);
}

HAL_StatusTypeDef QSPI_ReadSr(QSPI_HandleTypeDef *hqspi, uint8_t *flashStatus)
{
    return qspiStigCommand(hqspi, CMD_RDSR, 0, 0, NULL, flashStatus, 1);
}

HAL_StatusTypeDef QSPI_WaitUntilIdle(QSPI_HandleTypeDef *hqspi, uint32_t timeoutMs)
{
    const uint32_t started = HAL_GetTick();
    // Bound the number of status reads even if the HAL tick stops advancing.
    const uint64_t maxPolls = (uint64_t)timeoutMs * QSPI_STATUS_POLLS_PER_MS + 1U;

    for (uint64_t poll = 0; poll < maxPolls; ++poll) {
        uint8_t flashStatus;
        const HAL_StatusTypeDef status = QSPI_ReadSr(hqspi, &flashStatus);
        if (status != HAL_OK) {
            return status;
        }
        if (!(flashStatus & QSPI_SR1_BUSY)) {
            return HAL_OK;
        }
        if (HAL_GetTick() - started >= timeoutMs) {
            break;
        }
    }

    hqspi->ErrorCode |= HAL_QSPI_ERROR_TIMEOUT;
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef qspiWriteAndWait(QSPI_HandleTypeDef *hqspi, uint8_t opcode, uint32_t flags,
    uint32_t address, const uint8_t *data, uint32_t length, uint32_t timeoutMs)
{
    HAL_StatusTypeDef status = QSPI_WriteEnable(hqspi);
    if (status != HAL_OK) {
        return status;
    }

    status = qspiStigCommand(hqspi, opcode, flags, address, data, NULL, length);
    if (status != HAL_OK) {
        return status;
    }

    return QSPI_WaitUntilIdle(hqspi, timeoutMs);
}

HAL_StatusTypeDef QSPI_WriteSr(QSPI_HandleTypeDef *hqspi, uint8_t sr)
{
    return qspiWriteAndWait(hqspi, CMD_WRSR2, 0, 0, &sr, 1, QSPI_WRITE_TIMEOUT_MS);
}

HAL_StatusTypeDef QSPI_QuadEn(QSPI_HandleTypeDef *hqspi)
{
    return QSPI_WriteSr(hqspi, QSPI_SR2_QUAD_ENABLE);
}

HAL_StatusTypeDef QSPI_QuadDisable(QSPI_HandleTypeDef *hqspi)
{
    return QSPI_WriteSr(hqspi, 0);
}

/**
 * @brief  read flash device id
 * @param  hqspi           QSPI handle
 * @return none
 */
uint32_t QSPI_ReadDeviceId(QSPI_HandleTypeDef *hqspi)
{
	uint32_t device_id;
	uint16_t temp16;

  hqspi->Command.Instruction      = READ_DEVICE_ID; //0x90
  hqspi->Command.AddressEn        = QSPI_ADDREN_ENABLE;
  hqspi->Command.AddressByte      = QSPI_ADNUM_BYTES_3;
  hqspi->Command.DummyCycles      = QSPI_DUMMY_CLKS_0;
  hqspi->Command.ReadEn           = QSPI_RDEN_ENABLE;
  hqspi->Command.ReadDataNum      = QSPI_RD_NUM_2;

  HAL_QSPI_Command(hqspi, (uint32_t)NULL, (uint32_t)NULL, 5000);

	temp16 = __HAL_QSPI_GET_FCRLR(hqspi);

  /* The size end converts high and low bytes	*/
	device_id = (((temp16 & 0xFF)<<8) | (temp16>>8));

	return device_id;
}

/**
 * @brief  read flash device id by 0x9F command
 * @param  hqspi           QSPI handle
 * @return none
 */
uint32_t QSPI_ReadDeviceId_9F(QSPI_HandleTypeDef *hqspi)
{
	uint32_t device_id;
	uint32_t temp32;

  hqspi->Command.Instruction      = FLASH_ID; // 0x9F
  hqspi->Command.AddressEn        = QSPI_ADDREN_ENABLE;
  hqspi->Command.AddressByte      = QSPI_ADNUM_BYTES_3;
  hqspi->Command.DummyCycles      = QSPI_DUMMY_CLKS_0;
  hqspi->Command.ReadEn           = QSPI_RDEN_ENABLE;
  hqspi->Command.ReadDataNum      = QSPI_RD_NUM_3;

  HAL_QSPI_Command(hqspi, (uint32_t)NULL, (uint32_t)NULL, 5000);

	temp32 = __HAL_QSPI_GET_FCRLR(hqspi);

  /* The size end converts high and low bytes	*/
	device_id = (((temp32 & 0xFF)<<16) | ((temp32 & 0xFF00)<<0) | ((temp32 & 0xFF0000)>>16));

	return device_id;
}

/**
 * @brief  read flash unique id by 0x4B command
 * @param  hqspi           QSPI handle
 * @return none
 */
//uint32_t QSPI_ReadUniqueId(QSPI_HandleTypeDef *hqspi)
void QSPI_ReadUniqueId(QSPI_HandleTypeDef *hqspi, uint32_t arr[2])
{
	// uint32_t device_id;
	uint32_t temp32_low, temp32_High;
	uint32_t reverse_FCRLR_reg, reverse_FCRHR_reg;

  hqspi->Command.Instruction      = CMD_UNIQUEID; // 0x4B
  hqspi->Command.AddressEn        = QSPI_ADDREN_ENABLE;
  hqspi->Command.AddressByte      = QSPI_ADNUM_BYTES_3;
  hqspi->Command.DummyCycles      = QSPI_DUMMY_CLKS_4;
  hqspi->Command.ReadEn           = QSPI_RDEN_ENABLE;
  hqspi->Command.ReadDataNum      = QSPI_RD_NUM_8;


  HAL_QSPI_Command(hqspi, (uint32_t)NULL, (uint32_t)NULL, 5000);


	temp32_low = __HAL_QSPI_GET_FCRLR(hqspi);
	temp32_High = __HAL_QSPI_GET_FCRHR(hqspi);

	reverse_FCRLR_reg = (((temp32_low & 0xFF)<<24) | ((temp32_low & 0xFF00)<<8)
					| ((temp32_low & 0xFF0000)>>8) | ((temp32_low & 0xFF000000)>>24));

	reverse_FCRHR_reg = (((temp32_High & 0xFF)<<24) | ((temp32_High & 0xFF00)<<8)
					| ((temp32_High & 0xFF0000)>>8) | ((temp32_High & 0xFF000000)>>24));

	arr[0] = reverse_FCRLR_reg;
	arr[1] = reverse_FCRHR_reg;

	// device_id = reverse_FCRHR_reg;
	// return device_id;
}

HAL_StatusTypeDef QSPI_EraseSector(QSPI_HandleTypeDef *hqspi, uint32_t addr)
{
    const uint32_t flags = QSPI_ADDREN_ENABLE | QSPI_ADNUM_BYTES_3;
    return qspiWriteAndWait(hqspi, CMD_SE, flags, addr, NULL, 0, QSPI_SECTOR_ERASE_TIMEOUT_MS);
}

HAL_StatusTypeDef QSPI_EraseChip(QSPI_HandleTypeDef *hqspi, uint32_t addr)
{
    return qspiWriteAndWait(hqspi, CMD_CE, QSPI_ADNUM_BYTES_3, addr, NULL, 0, QSPI_CHIP_ERASE_TIMEOUT_MS);
}

HAL_StatusTypeDef QSPI_PageWrite(QSPI_HandleTypeDef *hqspi, uint8_t *buf, uint32_t write_addr, uint16_t len)
{
    const uint32_t flags = QSPI_ADDREN_ENABLE | QSPI_ADNUM_BYTES_3;
    while (len) {
        const uint32_t pageRemaining = QSPI_FLASH_PAGE_SIZE - (write_addr % QSPI_FLASH_PAGE_SIZE);
        const uint32_t chunk = MIN(MIN(len, QSPI_STIG_MAX_BYTES), pageRemaining);
        const HAL_StatusTypeDef status = qspiWriteAndWait(hqspi, CMD_PP, flags, write_addr, buf, chunk, QSPI_WRITE_TIMEOUT_MS);
        if (status != HAL_OK) {
            return status;
        }
        buf += chunk;
        write_addr += chunk;
        len -= chunk;
    }
    return HAL_OK;
}

HAL_StatusTypeDef QSPI_PageRead(QSPI_HandleTypeDef *hqspi, uint8_t *buf, uint32_t read_addr, uint16_t len)
{
    const uint32_t flags = QSPI_ADDREN_ENABLE | QSPI_ADNUM_BYTES_3;
    while (len) {
        const uint32_t chunk = MIN(len, QSPI_STIG_MAX_BYTES);
        const HAL_StatusTypeDef status = qspiStigCommand(hqspi, CMD_READ, flags, read_addr, NULL, buf, chunk);
        if (status != HAL_OK) {
            return status;
        }
        buf += chunk;
        read_addr += chunk;
        len -= chunk;
    }
    return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
