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

#include "sdcard.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/gpio.h"
#include "drivers/sdcard.h"

void SD_Detect_LowLevel_DeInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Configure SD_SPI_INSTANCE_DETECT_PIN pin: SD Card detect pin */
    GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
    GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);
}

void SD_Detect_LowLevel_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Configure SD_SPI_INSTANCE_DETECT_PIN pin: SD Card detect pin */
    GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  DeInitializes the SD/SD communication.
  * @param  None
  * @retval None
  */
void SD_DeInit(void)
{
  //SD_LowLevel_DeInit();
}

/**
  * @brief  Initializes the SD/SD communication.
  * @param  None
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_Init(void)
{
  uint32_t i = 0;

  /*!< Initialize SD_SPI_INSTANCE */
  //SD_LowLevel_Init();

  /*!< SD chip select high */
  SD_CS_HIGH();

  /*!< Send dummy byte 0xFF, 10 times with CS high */
  /*!< Rise CS and MOSI for 80 clocks cycles */
  for (i = 0; i <= 9; i++)
  {
    /*!< Send dummy byte 0xFF */
    SD_WriteByte(SD_DUMMY_BYTE);
  }

  /*------------Put SD in SPI mode--------------*/
  /*!< SD initialized and set to SPI mode properly */
  return (SD_GoIdleState());
}

/**
 * @brief  Detect if SD card is correctly plugged in the memory slot.
 * @param  None
 * @retval Return if SD is detected or not
 */
uint8_t SD_Detect(void)
{
  /*!< Check GPIO to detect SD */
    if (GPIO_ReadInputData(SD_DETECT_GPIO_PORT) & SD_DETECT_PIN)
    {
#ifdef SD_DETECT_INVERTED
        return SD_NOT_PRESENT;
#else
        return SD_PRESENT;
#endif
    } else {
#ifdef SD_DETECT_INVERTED
        return SD_PRESENT;
#else
        return SD_NOT_PRESENT;
#endif
    }
}


/**
  * @brief  Returns information about specific card.
  * @param  cardinfo: pointer to a SD_CardInfo structure that contains all SD
  *         card information.
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
  SD_Error status = SD_RESPONSE_FAILURE;

  SD_GetCSDRegister(&(cardinfo->SD_csd));
  status = SD_GetCIDRegister(&(cardinfo->SD_cid));
  cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
  cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
  cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
  cardinfo->CardCapacity *= cardinfo->CardBlockSize;

  /*!< Returns the reponse */
  return status;
}

/**
  * @brief  Reads a block of data from the SD.
  * @param  pBuffer: pointer to the buffer that receives the data read from the
  *                  SD.
  * @param  ReadAddr: SD's internal address to read from.
  * @param  BlockSize: the SD card Data block size.
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_ReadBlock(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t BlockSize)
{
  uint32_t i = 0;
  SD_Error rvalue = SD_RESPONSE_FAILURE;

  /*!< SD chip select low */
  SD_CS_LOW();

  /*!< Send CMD17 (SD_CMD_READ_SINGLE_BLOCK) to read one block */
  SD_SendCmd(SD_CMD_READ_SINGLE_BLOCK, ReadAddr, 0xFF);

  /*!< Check if the SD acknowledged the read block command: R1 response (0x00: no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
  {
    /*!< Now look for the data token to signify the start of the data */
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
    {
      /*!< Read the SD block data : read NumByteToRead data */
      for (i = 0; i < BlockSize; i++)
      {
        /*!< Save the received data */
        *pBuffer = SD_ReadByte();

        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
      }
      /*!< Get CRC bytes (not really needed by us, but required by SD) */
      SD_ReadByte();
      SD_ReadByte();
      /*!< Set response value to success */
      rvalue = SD_RESPONSE_NO_ERROR;
    }
  }
  /*!< SD chip select high */
  SD_CS_HIGH();

  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);

  /*!< Returns the reponse */
  return rvalue;
}

/**
  * @brief  Reads multiple block of data from the SD.
  * @param  pBuffer: pointer to the buffer that receives the data read from the
  *                  SD.
  * @param  ReadAddr: SD's internal address to read from.
  * @param  BlockSize: the SD card Data block size.
  * @param  NumberOfBlocks: number of blocks to be read.
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_ReadMultiBlocks(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  uint32_t i = 0, Offset = 0;
  SD_Error rvalue = SD_RESPONSE_FAILURE;

  /*!< SD chip select low */
  SD_CS_LOW();
  /*!< Data transfer */
  while (NumberOfBlocks--)
  {
    /*!< Send CMD17 (SD_CMD_READ_SINGLE_BLOCK) to read one block */
    SD_SendCmd (SD_CMD_READ_SINGLE_BLOCK, ReadAddr + Offset, 0xFF);
    /*!< Check if the SD acknowledged the read block command: R1 response (0x00: no errors) */
    if (SD_GetResponse(SD_RESPONSE_NO_ERROR))
    {
      return  SD_RESPONSE_FAILURE;
    }
    /*!< Now look for the data token to signify the start of the data */
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
    {
      /*!< Read the SD block data : read NumByteToRead data */
      for (i = 0; i < BlockSize; i++)
      {
        /*!< Read the pointed data */
        *pBuffer = SD_ReadByte();
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
      }
      /*!< Set next read address*/
      Offset += 512;
      /*!< get CRC bytes (not really needed by us, but required by SD) */
      SD_ReadByte();
      SD_ReadByte();
      /*!< Set response value to success */
      rvalue = SD_RESPONSE_NO_ERROR;
    }
    else
    {
      /*!< Set response value to failure */
      rvalue = SD_RESPONSE_FAILURE;
    }
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);
  /*!< Returns the reponse */
  return rvalue;
}

/**
  * @brief  Writes a block on the SD
  * @param  pBuffer: pointer to the buffer containing the data to be written on
  *                  the SD.
  * @param  WriteAddr: address to write on.
  * @param  BlockSize: the SD card Data block size.
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_WriteBlock(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t BlockSize)
{
  uint32_t i = 0;
  SD_Error rvalue = SD_RESPONSE_FAILURE;

  /*!< SD chip select low */
  SD_CS_LOW();

  /*!< Send CMD24 (SD_CMD_WRITE_SINGLE_BLOCK) to write multiple block */
  SD_SendCmd(SD_CMD_WRITE_SINGLE_BLOCK, WriteAddr, 0xFF);

  /*!< Check if the SD acknowledged the write block command: R1 response (0x00: no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
  {
    /*!< Send a dummy byte */
    SD_WriteByte(SD_DUMMY_BYTE);

    /*!< Send the data token to signify the start of the data */
    SD_WriteByte(0xFE);

    /*!< Write the block data to SD : write count data by block */
    for (i = 0; i < BlockSize; i++)
    {
      /*!< Send the pointed byte */
      SD_WriteByte(*pBuffer);
      /*!< Point to the next location where the byte read will be saved */
      pBuffer++;
    }
    /*!< Put CRC bytes (not really needed by us, but required by SD) */
    SD_ReadByte();
    SD_ReadByte();
    /*!< Read data response */
    if (SD_GetDataResponse() == SD_DATA_OK)
    {
      rvalue = SD_RESPONSE_NO_ERROR;
    }
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);

  /*!< Returns the reponse */
  return rvalue;
}

/**
  * @brief  Writes many blocks on the SD
  * @param  pBuffer: pointer to the buffer containing the data to be written on
  *                  the SD.
  * @param  WriteAddr: address to write on.
  * @param  BlockSize: the SD card Data block size.
  * @param  NumberOfBlocks: number of blocks to be written.
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_WriteMultiBlocks(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  uint32_t i = 0, Offset = 0;
  SD_Error rvalue = SD_RESPONSE_FAILURE;

  /*!< SD chip select low */
  SD_CS_LOW();
  /*!< Data transfer */
  while (NumberOfBlocks--)
  {
    /*!< Send CMD24 (SD_CMD_WRITE_SINGLE_BLOCK) to write blocks */
    SD_SendCmd(SD_CMD_WRITE_SINGLE_BLOCK, WriteAddr + Offset, 0xFF);
    /*!< Check if the SD acknowledged the write block command: R1 response (0x00: no errors) */
    if (SD_GetResponse(SD_RESPONSE_NO_ERROR))
    {
      return SD_RESPONSE_FAILURE;
    }
    /*!< Send dummy byte */
    SD_WriteByte(SD_DUMMY_BYTE);
    /*!< Send the data token to signify the start of the data */
    SD_WriteByte(SD_START_DATA_SINGLE_BLOCK_WRITE);
    /*!< Write the block data to SD : write count data by block */
    for (i = 0; i < BlockSize; i++)
    {
      /*!< Send the pointed byte */
      SD_WriteByte(*pBuffer);
      /*!< Point to the next location where the byte read will be saved */
      pBuffer++;
    }
    /*!< Set next write address */
    Offset += 512;
    /*!< Put CRC bytes (not really needed by us, but required by SD) */
    SD_ReadByte();
    SD_ReadByte();
    /*!< Read data response */
    if (SD_GetDataResponse() == SD_DATA_OK)
    {
      /*!< Set response value to success */
      rvalue = SD_RESPONSE_NO_ERROR;
    }
    else
    {
      /*!< Set response value to failure */
      rvalue = SD_RESPONSE_FAILURE;
    }
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);
  /*!< Returns the reponse */
  return rvalue;
}

/**
  * @brief  Read the CSD card register.
  *         Reading the contents of the CSD register in SPI mode is a simple
  *         read-block transaction.
  * @param  SD_csd: pointer on an SCD register structure
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_GetCSDRegister(SD_CSD* SD_csd)
{
  uint32_t i = 0;
  SD_Error rvalue = SD_RESPONSE_FAILURE;
  uint8_t CSD_Tab[16];

  /*!< SD chip select low */
  SD_CS_LOW();
  /*!< Send CMD9 (CSD register) or CMD10(CSD register) */
  SD_SendCmd(SD_CMD_SEND_CSD, 0, 0xFF);
  /*!< Wait for response in the R1 format (0x00 is no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
  {
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
    {
      for (i = 0; i < 16; i++)
      {
        /*!< Store CSD register value on CSD_Tab */
        CSD_Tab[i] = SD_ReadByte();
      }
    }
    /*!< Get CRC bytes (not really needed by us, but required by SD) */
    SD_WriteByte(SD_DUMMY_BYTE);
    SD_WriteByte(SD_DUMMY_BYTE);
    /*!< Set response value to success */
    rvalue = SD_RESPONSE_NO_ERROR;
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);

  /*!< Byte 0 */
  SD_csd->CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
  SD_csd->SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
  SD_csd->Reserved1 = CSD_Tab[0] & 0x03;

  /*!< Byte 1 */
  SD_csd->TAAC = CSD_Tab[1];

  /*!< Byte 2 */
  SD_csd->NSAC = CSD_Tab[2];

  /*!< Byte 3 */
  SD_csd->MaxBusClkFrec = CSD_Tab[3];

  /*!< Byte 4 */
  SD_csd->CardComdClasses = CSD_Tab[4] << 4;

  /*!< Byte 5 */
  SD_csd->CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
  SD_csd->RdBlockLen = CSD_Tab[5] & 0x0F;

  /*!< Byte 6 */
  SD_csd->PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
  SD_csd->WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
  SD_csd->RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
  SD_csd->DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
  SD_csd->Reserved2 = 0; /*!< Reserved */

  SD_csd->DeviceSize = (CSD_Tab[6] & 0x03) << 10;

  /*!< Byte 7 */
  SD_csd->DeviceSize |= (CSD_Tab[7]) << 2;

  /*!< Byte 8 */
  SD_csd->DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;

  SD_csd->MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
  SD_csd->MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);

  /*!< Byte 9 */
  SD_csd->MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
  SD_csd->MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
  SD_csd->DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
  /*!< Byte 10 */
  SD_csd->DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;

  SD_csd->EraseGrSize = (CSD_Tab[10] & 0x40) >> 6;
  SD_csd->EraseGrMul = (CSD_Tab[10] & 0x3F) << 1;

  /*!< Byte 11 */
  SD_csd->EraseGrMul |= (CSD_Tab[11] & 0x80) >> 7;
  SD_csd->WrProtectGrSize = (CSD_Tab[11] & 0x7F);

  /*!< Byte 12 */
  SD_csd->WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
  SD_csd->ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
  SD_csd->WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
  SD_csd->MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;

  /*!< Byte 13 */
  SD_csd->MaxWrBlockLen |= (CSD_Tab[13] & 0xC0) >> 6;
  SD_csd->WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
  SD_csd->Reserved3 = 0;
  SD_csd->ContentProtectAppli = (CSD_Tab[13] & 0x01);

  /*!< Byte 14 */
  SD_csd->FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
  SD_csd->CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
  SD_csd->PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
  SD_csd->TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
  SD_csd->FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
  SD_csd->ECC = (CSD_Tab[14] & 0x03);

  /*!< Byte 15 */
  SD_csd->CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
  SD_csd->Reserved4 = 1;

  /*!< Return the reponse */
  return rvalue;
}

/**
  * @brief  Read the CID card register.
  *         Reading the contents of the CID register in SPI mode is a simple
  *         read-block transaction.
  * @param  SD_cid: pointer on an CID register structure
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_GetCIDRegister(SD_CID* SD_cid)
{
  uint32_t i = 0;
  SD_Error rvalue = SD_RESPONSE_FAILURE;
  uint8_t CID_Tab[16];

  /*!< SD chip select low */
  SD_CS_LOW();

  /*!< Send CMD10 (CID register) */
  SD_SendCmd(SD_CMD_SEND_CID, 0, 0xFF);

  /*!< Wait for response in the R1 format (0x00 is no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
  {
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
    {
      /*!< Store CID register value on CID_Tab */
      for (i = 0; i < 16; i++)
      {
        CID_Tab[i] = SD_ReadByte();
      }
    }
    /*!< Get CRC bytes (not really needed by us, but required by SD) */
    SD_WriteByte(SD_DUMMY_BYTE);
    SD_WriteByte(SD_DUMMY_BYTE);
    /*!< Set response value to success */
    rvalue = SD_RESPONSE_NO_ERROR;
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);

  /*!< Byte 0 */
  SD_cid->ManufacturerID = CID_Tab[0];

  /*!< Byte 1 */
  SD_cid->OEM_AppliID = CID_Tab[1] << 8;

  /*!< Byte 2 */
  SD_cid->OEM_AppliID |= CID_Tab[2];

  /*!< Byte 3 */
  SD_cid->ProdName1 = CID_Tab[3] << 24;

  /*!< Byte 4 */
  SD_cid->ProdName1 |= CID_Tab[4] << 16;

  /*!< Byte 5 */
  SD_cid->ProdName1 |= CID_Tab[5] << 8;

  /*!< Byte 6 */
  SD_cid->ProdName1 |= CID_Tab[6];

  /*!< Byte 7 */
  SD_cid->ProdName2 = CID_Tab[7];

  /*!< Byte 8 */
  SD_cid->ProdRev = CID_Tab[8];

  /*!< Byte 9 */
  SD_cid->ProdSN = CID_Tab[9] << 24;

  /*!< Byte 10 */
  SD_cid->ProdSN |= CID_Tab[10] << 16;

  /*!< Byte 11 */
  SD_cid->ProdSN |= CID_Tab[11] << 8;

  /*!< Byte 12 */
  SD_cid->ProdSN |= CID_Tab[12];

  /*!< Byte 13 */
  SD_cid->Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
  SD_cid->ManufactDate = (CID_Tab[13] & 0x0F) << 8;

  /*!< Byte 14 */
  SD_cid->ManufactDate |= CID_Tab[14];

  /*!< Byte 15 */
  SD_cid->CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
  SD_cid->Reserved2 = 1;

  /*!< Return the reponse */
  return rvalue;
}

/**
  * @brief  Send 5 bytes command to the SD card.
  * @param  Cmd: The user expected command to send to SD card.
  * @param  Arg: The command argument.
  * @param  Crc: The CRC.
  * @retval None
  */
void SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc)
{
  uint32_t i = 0x00;

  uint8_t Frame[6];

  Frame[0] = (Cmd | 0x40); /*!< Construct byte 1 */

  Frame[1] = (uint8_t)(Arg >> 24); /*!< Construct byte 2 */

  Frame[2] = (uint8_t)(Arg >> 16); /*!< Construct byte 3 */

  Frame[3] = (uint8_t)(Arg >> 8); /*!< Construct byte 4 */

  Frame[4] = (uint8_t)(Arg); /*!< Construct byte 5 */

  Frame[5] = (Crc); /*!< Construct CRC: byte 6 */

  for (i = 0; i < 6; i++)
  {
    SD_WriteByte(Frame[i]); /*!< Send the Cmd bytes */
  }
}

/**
  * @brief  Get SD card data response.
  * @param  None
  * @retval The SD status: Read data response xxx0<status>1
  *         - status 010: Data accepted
  *         - status 101: Data rejected due to a crc error
  *         - status 110: Data rejected due to a Write error.
  *         - status 111: Data rejected due to other error.
  */
uint8_t SD_GetDataResponse(void)
{
  uint32_t i = 0;
  uint8_t response, rvalue;

  while (i <= 64)
  {
    /*!< Read response */
    response = SD_ReadByte();
    /*!< Mask unused bits */
    response &= 0x1F;
    switch (response)
    {
      case SD_DATA_OK:
      {
        rvalue = SD_DATA_OK;
        break;
      }
      case SD_DATA_CRC_ERROR:
        return SD_DATA_CRC_ERROR;
      case SD_DATA_WRITE_ERROR:
        return SD_DATA_WRITE_ERROR;
      default:
      {
        rvalue = SD_DATA_OTHER_ERROR;
        break;
      }
    }
    /*!< Exit loop in case of data ok */
    if (rvalue == SD_DATA_OK)
      break;
    /*!< Increment loop counter */
    i++;
  }

  /*!< Wait null data */
  while (SD_ReadByte() == 0);

  /*!< Return response */
  return response;
}

/**
  * @brief  Returns the SD response.
  * @param  None
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_GetResponse(uint8_t Response)
{
  uint32_t Count = 0xFFF;

  /* Check if response is got or a timeout is happen */
  while ((SD_ReadByte() != Response) && Count)
  {
    Count--;
  }

  if (Count == 0)
  {
    /* After time out */
    return SD_RESPONSE_FAILURE;
  }
  else
  {
    /* Right response got */
    return SD_RESPONSE_NO_ERROR;
  }
}

/**
  * @brief  Returns the SD status.
  * @param  None
  * @retval The SD status.
  */
uint16_t SD_GetStatus(void)
{
  uint16_t Status = 0;

  /*!< SD chip select low */
  SD_CS_LOW();

  /*!< Send CMD13 (SD_SEND_STATUS) to get SD status */
  SD_SendCmd(SD_CMD_SEND_STATUS, 0, 0xFF);

  Status = SD_ReadByte();
  Status |= (uint16_t)(SD_ReadByte() << 8);

  /*!< SD chip select high */
  SD_CS_HIGH();

  /*!< Send dummy byte 0xFF */
  SD_WriteByte(SD_DUMMY_BYTE);

  return Status;
}

/**
  * @brief  Put SD in Idle state.
  * @param  None
  * @retval The SD Response:
  *         - SD_RESPONSE_FAILURE: Sequence failed
  *         - SD_RESPONSE_NO_ERROR: Sequence succeed
  */
SD_Error SD_GoIdleState(void)
{
  /*!< SD chip select low */
  SD_CS_LOW();

  /*!< Send CMD0 (SD_CMD_GO_IDLE_STATE) to put SD in SPI mode */
  SD_SendCmd(SD_CMD_GO_IDLE_STATE, 0, 0x95);

  /*!< Wait for In Idle State Response (R1 Format) equal to 0x01 */
  if (SD_GetResponse(SD_IN_IDLE_STATE))
  {
    /*!< No Idle State Response: return response failue */
    return SD_RESPONSE_FAILURE;
  }

//  SD_SendCmd(SD_CMD_SEND_IF_COND, 0, 0x65);
//  /*!< Wait for In Idle State Response (R1 Format) equal to 0x01 */
//  if (SD_GetResponse(SD_IN_IDLE_STATE))
//  {
//    /*!< No Idle State Response: return response failue */
//    return SD_RESPONSE_FAILURE;
//  }


  /*----------Activates the card initialization process-----------*/
  do
  {
    /*!< SD chip select high */
    SD_CS_HIGH();

    /*!< Send Dummy byte 0xFF */
    SD_WriteByte(SD_DUMMY_BYTE);

    /*!< SD chip select low */
    SD_CS_LOW();

    /*!< Send CMD1 (Activates the card process) until response equal to 0x0 */
    SD_SendCmd(SD_CMD_SEND_OP_COND, 0, 0xFF);
    /*!< Wait for no error Response (R1 Format) equal to 0x00 */
  }
  while (SD_GetResponse(SD_RESPONSE_NO_ERROR));

  /*!< SD chip select high */
  SD_CS_HIGH();

  /*!< Send dummy byte 0xFF */
  SD_WriteByte(SD_DUMMY_BYTE);

  return SD_RESPONSE_NO_ERROR;
}

/**
  * @brief  Write a byte on the SD.
  * @param  Data: byte to send.
  * @retval None
  */
uint8_t SD_WriteByte(uint8_t Data)
{
  /*!< Wait until the transmit buffer is empty */
  while(SPI_I2S_GetFlagStatus(SD_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET)
  {
  }

  /*!< Send the byte */
  SPI_SendData8(SD_SPI_INSTANCE, Data);

  /*!< Wait to receive a byte*/
  while(SPI_I2S_GetFlagStatus(SD_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == RESET)
  {
  }

  /*!< Return the byte read from the SPI bus */
  return SPI_ReceiveData8(SD_SPI_INSTANCE);
}

/**
  * @brief  Read a byte from the SD.
  * @param  None
  * @retval The received byte.
  */
uint8_t SD_ReadByte(void)
{
  uint8_t Data = 0;

  /*!< Wait until the transmit buffer is empty */
  while (SPI_I2S_GetFlagStatus(SD_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET)
  {
  }
  /*!< Send the byte */
  SPI_SendData8(SD_SPI_INSTANCE, SD_DUMMY_BYTE);

  /*!< Wait until a data is received */
  while (SPI_I2S_GetFlagStatus(SD_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == RESET)
  {
  }
  /*!< Get the received data */
  Data = SPI_ReceiveData8(SD_SPI_INSTANCE);

  /*!< Return the shifted data */
  return Data;
}
#include "drivers/bus_spi.h"
#include "drivers/system.h"

#include "sdcard_standard.h"

#ifdef USE_SDCARD

#define SET_CS_HIGH          GPIO_SetBits(SDCARD_SPI_CS_GPIO,   SDCARD_SPI_CS_PIN)
#define SET_CS_LOW           GPIO_ResetBits(SDCARD_SPI_CS_GPIO, SDCARD_SPI_CS_PIN)

#define SDCARD_INIT_NUM_DUMMY_BYTES 10
#define SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY 8
// Chosen so that CMD8 will have the same CRC as CMD0:
#define SDCARD_IF_COND_CHECK_PATTERN 0xAB

#define STATIC_ASSERT(condition, name ) \
    typedef char assert_failed_ ## name [(condition) ? 1 : -1 ]

typedef enum {
    SDCARD_STATE_NOT_PRESENT = 0,
    SDCARD_STATE_INITIALIZATION,
    SDCARD_STATE_INITIALIZATION_RECEIVE_CID,
    SDCARD_STATE_READY,
    SDCARD_STATE_READING,
    SDCARD_STATE_WRITING,
} sdcardState_e;

typedef struct sdcard_t {
    struct {
        uint8_t *buffer;
        int error;
        uint32_t blockIndex;

        sdcard_operationCompleteCallback_c callback;
        uint32_t callbackData;
    } pendingOperation;

    uint8_t version;
    bool highCapacity;

    sdcardMetadata_t metadata;
    sdcardCSD_t csd;

    sdcardState_e state;
} sdcard_t;


static sdcard_t sdcard;

STATIC_ASSERT(sizeof(sdcardCSD_t) == 16, sdcard_csd_bitfields_didnt_pack_properly);

static void sdcard_select()
{
    SET_CS_LOW;
}

static void sdcard_deselect()
{
    // As per the SD-card spec, give the card 8 dummy clocks so it can finish its operation
    //spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);

    while (spiIsBusBusy(SDCARD_SPI_INSTANCE)) {
    }

    SET_CS_HIGH;
}


/**
 * The SD card spec requires 8 clock cycles to be sent by us on the bus after most commands so it can finish its
 * processing of that command. The easiest way for us to do this is to just wait for the bus to become idle before
 * we transmit a command, sending at least 8-bits onto the bus when we do so.
 */
static bool sdcard_waitForIdle(int maxBytesToWait)
{
    while (maxBytesToWait > 0) {
        uint8_t b = spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
        if (b == 0xFF) {
            return true;
        }
        maxBytesToWait--;
    }

    return false;
}

/**
 * Wait for up to maxDelay 0xFF idle bytes to arrive from the card, returning the first non-idle byte found.
 *
 * Returns 0xFF on failure.
 */
static uint8_t sdcard_waitForNonIdleByte(int maxDelay)
{
    for (int i = 0; i < maxDelay + 1; i++) { // + 1 so we can wait for maxDelay '0xFF' bytes before reading a response byte afterwards
        uint8_t response = spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);

        if (response != 0xFF)
            return response;
    }

    return 0xFF;
}

/**
 * Waits up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes for the card to become ready, send a command to the card
 * with the given argument, waits up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes for a reply, and returns the
 * first non-0xFF byte of the reply.
 *
 * You must select the card first with sdcard_select() and deselect it afterwards with sdcard_deselect().
 *
 * Upon failure, 0xFF is returned.
 */
static uint8_t sdcard_sendCommand(uint8_t commandCode, uint32_t commandArgument)
{
    uint8_t command[6] = {
        0x40 | commandCode,
        commandArgument >> 24,
        commandArgument >> 16,
        commandArgument >> 8,
        commandArgument,
        0x95 /* Static CRC. This CRC is valid for CMD0 with a 0 argument, and CMD8 with 0x1AB argument, which are the only
        commands that require a CRC */
    };

    // Go ahead and send the command even if the card isn't idle if this is the reset command
    if (!sdcard_waitForIdle(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY) && commandCode != SDCARD_COMMAND_GO_IDLE_STATE)
        return 0xFF;

    spiTransfer(SDCARD_SPI_INSTANCE, NULL, command, sizeof(command));

    /*
     * The card can take up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes to send the response, in the meantime
     * it'll transmit 0xFF filler bytes.
     */
    return sdcard_waitForNonIdleByte(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY);
}

static uint8_t sdcard_sendAppCommand(uint8_t commandCode, uint32_t commandArgument) {
    sdcard_sendCommand(SDCARD_COMMAND_APP_CMD, 0);

    return sdcard_sendCommand(commandCode, commandArgument);
}

/**
 * Sends an IF_COND message to the card to check its version and validate its voltage requirements. Sets the global
 * sdCardVersion with the detected version (0, 1, or 2) and returns true if the card is compatbile.
 */
static bool sdcard_validateInterfaceCondition()
{
    uint8_t ifCondReply[4];

    sdcard.version = 0;

    sdcard_select();

    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_SEND_IF_COND, (SDCARD_VOLTAGE_ACCEPTED_2_7_to_3_6 << 8) | SDCARD_IF_COND_CHECK_PATTERN);

    // Don't deselect the card right away, because we'll want to read the rest of its reply if it's a V2 card

    if (status == (SDCARD_R1_STATUS_BIT_ILLEGAL_COMMAND | SDCARD_R1_STATUS_BIT_IDLE)) {
        // V1 cards don't support this command
        sdcard.version = 1;
    } else if (status == SDCARD_R1_STATUS_BIT_IDLE) {
        spiTransfer(SDCARD_SPI_INSTANCE, ifCondReply, NULL, sizeof(ifCondReply));

        /*
         * We don't bother to validate the SDCard's operating voltage range since the spec requires it to accept our
         * 3.3V, but do check that it echoed back our check pattern properly.
         */
        if (ifCondReply[3] == SDCARD_IF_COND_CHECK_PATTERN) {
            sdcard.version = 2;
        }
    }

    sdcard_deselect();

    return sdcard.version > 0;
}

static bool sdcard_readOCRRegister(uint32_t *result)
{
    sdcard_select();

    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_READ_OCR, 0);

    uint8_t response[4];

    spiTransfer(SDCARD_SPI_INSTANCE, response, NULL, sizeof(response));

    if (status == 0) {
        sdcard_deselect();

        *result = (response[0] << 24) | (response[1] << 16) | (response[2] << 8) | response[3];

        return true;
    } else {
        sdcard_deselect();

        return false;
    }
}

typedef enum {
    SDCARD_RECEIVE_SUCCESS,
    SDCARD_RECEIVE_BLOCK_IN_PROGRESS,
    SDCARD_RECEIVE_ERROR,
} sdcardReceiveBlockStatus_e;

/**
 * Attempt to receive a data block from the SD card.
 *
 * Return true on success, otherwise the card has not responded yet and you should retry later.
 */
static sdcardReceiveBlockStatus_e sdcard_receiveDataBlock(uint8_t *buffer, int count)
{
    uint8_t dataToken = sdcard_waitForNonIdleByte(8);

    if (dataToken == 0xFF) {
        return SDCARD_RECEIVE_BLOCK_IN_PROGRESS;
    }

    if (dataToken != SDCARD_SINGLE_BLOCK_READ_START_TOKEN) {
        return SDCARD_RECEIVE_ERROR;
    }

    spiTransfer(SDCARD_SPI_INSTANCE, buffer, NULL, count);

    // Discard trailing CRC, we don't care
    spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
    spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);

    return SDCARD_RECEIVE_SUCCESS;
}

/**
 * Write the buffer of `count` bytes to the SD card.
 *
 * Returns true if the card accepted the write (card will enter a busy state).
 */
static bool sdcard_sendDataBlock(uint8_t *buffer, int count)
{
    // Card wants 8 dummy clock cycles after the command response to become ready
    spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);

    spiTransferByte(SDCARD_SPI_INSTANCE, SDCARD_SINGLE_BLOCK_WRITE_START_TOKEN);
    spiTransfer(SDCARD_SPI_INSTANCE, NULL, buffer, count);

    // Send a dummy CRC
    spiTransferByte(SDCARD_SPI_INSTANCE, 0x00);
    spiTransferByte(SDCARD_SPI_INSTANCE, 0x00);

    uint8_t dataResponseToken = spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);

    /*
     * Check if the card accepted the write (no CRC error / no address error)
     *
     * The lower 5 bits are structured as follows:
     * | 0 | Status  | 1 |
     * | 0 | x  x  x | 1 |
     *
     * Statuses:
     * 010 - Data accepted
     * 101 - CRC error
     * 110 - Write error
     */
    return (dataResponseToken & 0x1F) == 0x05;
}

static bool sdcard_receiveCID()
{
    uint8_t cid[16];

    if (sdcard_receiveDataBlock(cid, sizeof(cid)) != SDCARD_RECEIVE_SUCCESS) {
        sdcard_deselect();
        return false;
    }

    sdcard.metadata.manufacturerID = cid[0];
    sdcard.metadata.oemID = (cid[1] << 8) | cid[2];
    sdcard.metadata.productName[0] = cid[3];
    sdcard.metadata.productName[1] = cid[4];
    sdcard.metadata.productName[2] = cid[5];
    sdcard.metadata.productName[3] = cid[6];
    sdcard.metadata.productName[4] = cid[7];
    sdcard.metadata.productRevisionMajor = cid[8] >> 4;
    sdcard.metadata.productRevisionMinor = cid[8] & 0x0F;
    sdcard.metadata.productSerial = (cid[9] << 24) | (cid[10] << 16) | (cid[11] << 8) | cid[12];
    sdcard.metadata.productionYear = (((cid[13] & 0x0F) << 4) | (cid[14] >> 4)) + 2000;
    sdcard.metadata.productionMonth = cid[14] & 0x0F;

    sdcard_deselect();

    return true;
}

static bool sdcard_fetchCSD()
{
    uint32_t readBlockLen, blockCount, blockCountMult, capacityBytes;

    sdcard_select();

    // The CSD command's data block will arrive within 8 idle clock cycles (SD card spec)
    bool success =
        sdcard_sendCommand(SDCARD_COMMAND_SEND_CSD, 0) == 0
        && sdcard_receiveDataBlock((uint8_t*) &sdcard.csd, sizeof(sdcard.csd)) == SDCARD_RECEIVE_SUCCESS
        && SDCARD_GET_CSD_FIELD(sdcard.csd, 1, TRAILER) == 1;

    if (success) {
        switch (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSD_STRUCTURE_VER)) {
            case SDCARD_CSD_STRUCTURE_VERSION_1:
                // Block size in bytes (doesn't have to be 512)
                readBlockLen = 1 << SDCARD_GET_CSD_FIELD(sdcard.csd, 1, READ_BLOCK_LEN);
                blockCountMult = 1 << (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSIZE_MULT) + 2);
                blockCount = (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSIZE) + 1) * blockCountMult;
                capacityBytes = blockCount * readBlockLen;

                // Re-express that capacity (max 2GB) in our standard 512-byte block size
                sdcard.metadata.numBlocks = capacityBytes / SDCARD_BLOCK_SIZE;
            break;
            case SDCARD_CSD_STRUCTURE_VERSION_2:
                sdcard.metadata.numBlocks = (SDCARD_GET_CSD_FIELD(sdcard.csd, 2, CSIZE) + 1) * 1024;
            break;
            default:
                success = false;
        }
    }

    sdcard_deselect();

    return success;
}

/**
 * Call after the CID and CSD data have been read to set our preferred settings into the card (frequency and blocksize).
 *
 * Returns true on success, false on card init failure.
 */
static bool sdcard_setConfigurationAndFinalClock()
{
    /* The spec is a little iffy on what the default block size is for Standard Size cards (it can be changed on
     * standard size cards) so let's just set it to 512 explicitly so we don't have a problem.
     */
    if (!sdcard.highCapacity) {
        sdcard_select();

        if (sdcard_sendCommand(SDCARD_COMMAND_SET_BLOCKLEN, SDCARD_BLOCK_SIZE) != 0) {
            sdcard_deselect();
            return false;
        }

        sdcard_deselect();
    }

    spiSetDivisor(SDCARD_SPI_INSTANCE, SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER);

    return true;
}

/**
 * Check if the SD Card has completed its startup sequence. Must be called with sdcard.state == SDCARD_STATE_INITIALIZATION.
 *
 * Returns true if the card has finished its init process.
 */
static bool sdcard_checkInitDone() {
    sdcard_select();

    uint8_t status = sdcard_sendAppCommand(SDCARD_ACOMMAND_SEND_OP_COND, sdcard.version == 2 ? 1 << 30 /* We support high capacity cards */ : 0);

    sdcard_deselect();

    // When card init is complete, the idle bit in the response becomes zero.
    return status == 0x00;
}

bool sdcard_init()
{
    // Max frequency is initially 400kHz
    spiSetDivisor(SDCARD_SPI_INSTANCE, SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER);

    // SDCard wants 1ms minimum delay after power is applied to it
    delay(1000);

    // Transmit at least 74 dummy clock cycles with CS high so the SD card can start up
    SET_CS_HIGH;

    spiTransfer(SDCARD_SPI_INSTANCE, NULL, NULL, SDCARD_INIT_NUM_DUMMY_BYTES);

    // Wait for that transmission to finish before we enable the SDCard, so it receives the required number of cycles:
    while (spiIsBusBusy(SDCARD_SPI_INSTANCE)) {
    }

    sdcard_select();

    uint8_t initStatus = sdcard_sendCommand(SDCARD_COMMAND_GO_IDLE_STATE, 0);

    sdcard_deselect();

    if (initStatus != SDCARD_R1_STATUS_BIT_IDLE)
        return false;

    // Check card voltage and version
    if (!sdcard_validateInterfaceCondition())
        return false;

    uint32_t ocr;

    sdcard_readOCRRegister(&ocr);

    /*
     * Now the SD card will perform its startup, which can take hundreds of milliseconds. We won't wait for this to
     * avoid slowing down system startup. Instead we'll periodically poll with sdcard_checkInitDone() later on.
     */
    sdcard.state = SDCARD_STATE_INITIALIZATION;

    return true;
}

/**
 * Call periodically for the SD card to perform in-progress transfers.
 */
void sdcard_poll()
{
    doMore:
    switch (sdcard.state) {
        case SDCARD_STATE_INITIALIZATION:
            if (sdcard_checkInitDone()) {
                if (sdcard.version == 2) {
                    // Check for high capacity card
                    uint32_t ocr;

                    if (!sdcard_readOCRRegister(&ocr)) {
                        break;
                    }

                    sdcard.highCapacity = (ocr & (1 << 30)) != 0;
                } else {
                    // Version 1 cards are always low-capacity
                    sdcard.highCapacity = false;
                }

                if (sdcard_fetchCSD()) {
                    sdcard_select();

                    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_SEND_CID, 0);

                    if (status == 0) {
                        sdcard.state = SDCARD_STATE_INITIALIZATION_RECEIVE_CID;
                        goto doMore;
                    } else {
                        sdcard_deselect();
                    }
                }
            }
        break;
        case SDCARD_STATE_INITIALIZATION_RECEIVE_CID:
            if (sdcard_receiveCID()) {
                if (sdcard_setConfigurationAndFinalClock()) {
                    sdcard.state = SDCARD_STATE_READY;
                } else {
                    // TODO we could reset the card here and try again
                }
            }
        break;
        case SDCARD_STATE_WRITING:
            if (sdcard_waitForIdle(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY)) {
                sdcard_deselect();
                sdcard.state = SDCARD_STATE_READY;
            }

        break;
        case SDCARD_STATE_READING:
            switch (sdcard_receiveDataBlock(sdcard.pendingOperation.buffer, SDCARD_BLOCK_SIZE)) {
                case SDCARD_RECEIVE_SUCCESS:
                    sdcard_deselect();

                    sdcard.state = SDCARD_STATE_READY;

                    if (sdcard.pendingOperation.callback) {
                        sdcard.pendingOperation.callback(
                            SDCARD_BLOCK_OPERATION_READ,
                            sdcard.pendingOperation.blockIndex,
                            sdcard.pendingOperation.buffer,
                            sdcard.pendingOperation.callbackData
                        );
                    }
                break;
                case SDCARD_RECEIVE_ERROR:
                    sdcard_deselect();

                    sdcard.state = SDCARD_STATE_READY;

                    if (sdcard.pendingOperation.callback) {
                        sdcard.pendingOperation.callback(
                            SDCARD_BLOCK_OPERATION_READ,
                            sdcard.pendingOperation.blockIndex,
                            NULL,
                            sdcard.pendingOperation.callbackData
                        );
                    }
                break;
                case SDCARD_RECEIVE_BLOCK_IN_PROGRESS:
                    ;
                break;
            }
        break;
        default:
            ;
    }
}

/**
 * Write the 512-byte block from the given buffer into the block with the given index.
 *
 * Returns true if the write was successfully sent to the card, or false if the operation could
 * not be started due to the card being busy (try again later), or because the write was invalid (bad address).
 *
 * The buffer is not copied anywhere, you must keep the pointer to the buffer valid until the operation completes!
 */
bool sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer)
{
    if (sdcard.state != SDCARD_STATE_READY)
        return false;

    sdcard_select();

    // Standard size cards use byte addressing, high capacity cards use block addressing
    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_WRITE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE);

    if (status == 0 && sdcard_sendDataBlock(buffer, SDCARD_BLOCK_SIZE)) {
        sdcard.state = SDCARD_STATE_WRITING;

        // Leave the card selected while the write is in progress
        return true;
    } else {
        sdcard_deselect();
        return false;
    }
}

/**
 * Read the 512-byte block with the given index into the given 512-byte buffer.
 *
 * Returns true if the operation was successfully queued for later completion, or false if the operation could
 * not be started due to the card being busy (try again later).
 *
 * You must keep the pointer to the buffer valid until the operation completes!
 */
bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    if (sdcard.state != SDCARD_STATE_READY)
        return false;

    sdcard_select();

    // Standard size cards use byte addressing, high capacity cards use block addressing
    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_READ_SINGLE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE);

    if (status == 0) {
        sdcard.pendingOperation.buffer = buffer;
        sdcard.pendingOperation.blockIndex = blockIndex;
        sdcard.pendingOperation.callback = callback;
        sdcard.pendingOperation.callbackData = callbackData;
        
        sdcard.state = SDCARD_STATE_READING;
        // Leave the card selected for the whole transaction

        return true;
    } else {
        sdcard_deselect();

        return false;
    }
}

bool sdcard_isReady() {
    return sdcard.state == SDCARD_STATE_READY;
}

#endif
>>>>>>> 98133e5... Basic SDCard block read / write (minimal timeout/error handling)
