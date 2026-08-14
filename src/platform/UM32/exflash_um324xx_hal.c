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
#include "um324xx_hal.h"
#include "exflash_um324xx_hal.h" 
#include "drivers/io.h"
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup GT25QXX_Exported_Functions GT25QXX Exported Functions
  * @{
  */

/**
 * @brief  This function send a Write Enable and wait it is effective.
 * @param  hqspi           QSPI handle	 
 * @return none
 */
void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
    hqspi->Command.Instruction = (uint32_t)CMD_WREN; 
    hqspi->Command.AddressByte = QSPI_ADNUM_BYTES_1; 
    hqspi->Command.DummyCycles = QSPI_DUMMY_CLKS_0;
    
    HAL_QSPI_Command(hqspi, 0, 0, 500);  
    
    /* Wait for the write function to take effect */
    // __HAL_QSPI_WAIT_COMMAND_IDLE(hqspi);	 			
}

/**
 * @brief  Enable flash's qe mode
 * @param  hqspi           QSPI handle		 
 * @return none
 */
void QSPI_QuadEn(QSPI_HandleTypeDef *hqspi)
{
  /* enable to write */
	QSPI_WriteEnable(hqspi);
	
  /* enable the QE mode */
	__HAL_QSPI_SET_FCWLR(hqspi, 0x02);

  hqspi->Command.Instruction       = CMD_WRSR2; //0x31, write status reg2
  hqspi->Command.DummyCycles       = QSPI_DUMMY_CLKS_0;
  hqspi->Command.WriteDataNum      = QSPI_WD_NUM_1; 
  hqspi->Command.WriteEn           = QSPI_WREN_ENABLE;
    
  HAL_QSPI_Command(hqspi, 0, 0, 500);   
    
  /* wait for the flash to free */   
	QSPI_WaitUntilIdle(hqspi);                         
}

/**
 * @brief  Disable flash's qe mode
 * @param  hqspi           QSPI handle		 
 * @return none
 */
void QSPI_QuadDisable(QSPI_HandleTypeDef *hqspi)
{
	uint8_t flash_status = 0;
	
	flash_status = QSPI_ReadSr(hqspi);		
    
  /* enable to write	*/
	QSPI_WriteEnable(hqspi);
    
	/* disable the QE mode 	*/
	flash_status = 0;							
	QSPI_WriteSr(hqspi,flash_status);				
	
	QSPI_WaitUntilIdle(hqspi);
}

/**
 * @brief  Write the flash status register2
 * @param  hqspi           QSPI handle	
 * @param  sr
 * @return none
 */
void QSPI_WriteSr(QSPI_HandleTypeDef *hqspi,uint8_t sr)
{
	__HAL_QSPI_SET_FCWLR(hqspi,sr);
    	
  hqspi->Command.Instruction         = (uint32_t)CMD_WRSR2;
  hqspi->Command.AddressByte         = QSPI_ADNUM_BYTES_1;
  hqspi->Command.DummyCycles         = QSPI_DUMMY_CLKS_0;
  hqspi->Command.WriteDataNum         = QSPI_WD_NUM_1;
  hqspi->Command.WriteEn              = QSPI_WREN_ENABLE;
    
  HAL_QSPI_Command(hqspi, 0, 0, 500);     
}

/**
 * @brief  Read the flash status register1
 * @param  hqspi           QSPI handle	 
 * @return none
 */
uint8_t QSPI_ReadSr(QSPI_HandleTypeDef *hqspi)
{
	uint8_t read_sr = 0;
    		
  hqspi->Command.Instruction         = (uint32_t)CMD_RDSR; 
  hqspi->Command.AddressByte         = QSPI_ADNUM_BYTES_1; 
  hqspi->Command.DummyCycles         = QSPI_DUMMY_CLKS_0;
  hqspi->Command.ReadDataNum         = QSPI_RD_NUM_1; 
  hqspi->Command.ReadEn              = QSPI_RDEN_ENABLE;
    
  HAL_QSPI_Command(hqspi, 0, 0, 500);
	
	read_sr = __HAL_QSPI_GET_FCRLR(hqspi); 
	
	return read_sr;
}

/**
 * @brief  Wait for flash to free
 * @param  hqspi           QSPI handle	 
 * @return none
 */
void QSPI_WaitUntilIdle(QSPI_HandleTypeDef *hqspi)
{
  /* Waitting for the BUSY bit to clear */
	while((QSPI_ReadSr(hqspi) & 0x1));
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

/**
 * @brief  flash erase sector
 * @param  hqspi           QSPI handle
 * @param  addr     Address to be erased 	 
 * @return none
 */
void QSPI_EraseSector(QSPI_HandleTypeDef *hqspi,uint32_t addr)
{
	QSPI_WriteEnable(hqspi); // send write enable cmd before sending sector erase cmd	
	
  hqspi->Command.Instruction      = CMD_SE; // 0x20
  hqspi->Command.AddressEn        = QSPI_ADDREN_ENABLE;
  hqspi->Command.AddressByte      = QSPI_ADNUM_BYTES_3;    
  hqspi->Command.DummyCycles      = QSPI_DUMMY_CLKS_0;
    
  HAL_QSPI_Command(hqspi, addr, 0, 5000);         
    
	QSPI_WaitUntilIdle(hqspi);
}

/**
 * @brief  flash erase full chip
 * @param  hqspi           QSPI handle
 * @param  addr     Address to be erased 	 
 * @return none
 */
void QSPI_EraseChip(QSPI_HandleTypeDef *hqspi,uint32_t addr)
{
	QSPI_WriteEnable(hqspi); // send write enable cmd before sending sector erase cmd	
	
  hqspi->Command.Instruction      = CMD_CE; // 0x60
  hqspi->Command.AddressEn        = QSPI_ADDREN_DISABLE;
  hqspi->Command.AddressByte      = QSPI_ADNUM_BYTES_3;    
  hqspi->Command.DummyCycles      = QSPI_DUMMY_CLKS_0;
    
  HAL_QSPI_Command(hqspi, addr, 0, 5000);         
    
	QSPI_WaitUntilIdle(hqspi);
}

/**
 * @brief  flash page write in qspi stig mode
 * @param  hqspi             QSPI handle 
 * @param  *buf 	         Array pointer waiting for operation
 * @param  write_addr        The starting address of the operation.
 * @param  len               The length of the operation.
 * @return none
 */
void QSPI_PageWrite(QSPI_HandleTypeDef *hqspi,uint8_t* buf,uint32_t write_addr,uint16_t len)
{
	
	while(len)
	{
    /*  enable to write */
		QSPI_WriteEnable(hqspi);
        
    hqspi->Command.Instruction      = CMD_PP; // 0x02 page program
    hqspi->Command.AddressEn        = QSPI_ADDREN_ENABLE;
    hqspi->Command.AddressByte      = QSPI_ADNUM_BYTES_3; 
    hqspi->Command.WriteEn          = QSPI_WREN_ENABLE;
    hqspi->Command.WriteDataNum     = QSPI_WD_NUM_1; 

    HAL_QSPI_Command(hqspi,write_addr,*buf,5000);

		len--;
		buf++;
		write_addr++;
		
    QSPI_WaitUntilIdle(hqspi); 
	}
	
}

/**
 * @brief  flash read in qspi stig mode
 * @param  hqspi            QSPI handle 
 * @param  *buf  	        Array pointer waiting for operation
 * @param  read_addr 	    The starting address of the operation.
 * @param  len 	            The length of the operation.
 * @return none
 */
void QSPI_PageRead(QSPI_HandleTypeDef *hqspi,uint8_t* buf,uint32_t read_addr,uint16_t len)
{
	while(len)
	{
    hqspi->Command.Instruction      = CMD_READ; // 0x03
    hqspi->Command.AddressEn        = QSPI_ADDREN_ENABLE;
    hqspi->Command.AddressByte      = QSPI_ADNUM_BYTES_3; 
    hqspi->Command.ReadEn           = QSPI_RDEN_ENABLE;
    hqspi->Command.ReadDataNum      = QSPI_RD_NUM_1; 
    hqspi->Command.DummyCycles      = QSPI_DUMMY_CLKS_0;

    HAL_QSPI_Command(hqspi,read_addr, (uint32_t)NULL,5000);
    *buf = __HAL_QSPI_GET_FCRLR(hqspi); 
        
		len--;
		read_addr++;
		buf++;
  }         
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
