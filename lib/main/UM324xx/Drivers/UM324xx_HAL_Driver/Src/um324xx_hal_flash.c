/**
  ******************************************************************************
  * @file     um324xx_hal_flash.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-04-11
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

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @defgroup FLASH FLASH
  * @brief FLASH HAL module driver
  * @{
  */
#ifdef HAL_FLASH_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup FLASH_Private_Constants
  * @{
  */
#define FLASH_TIMEOUT_VALUE       50000U /* 50 s */


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup FLASH_Private_Variables
  * @{
  */
/* Variable used for Erase sectors under interruption */
FLASH_ProcessTypeDef pFlash;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @addtogroup FLASH_Private_Functions
  * @{
  */
/* Program operations */
static void   FLASH_Program_Word(uint32_t Address, uint32_t Data);
static void   FLASH_Program_HalfWord(uint32_t Address, uint16_t Data);
static void   FLASH_Program_Byte(uint32_t Address, uint8_t Data);
static void   FLASH_Continue_Program_Word(uint32_t Address, uint32_t *Data);
static void   FLASH_SetErrorCode(void);
static void   FLASH_MassErase(void);

HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup FLASH_Exported_Functions FLASH Exported Functions
  * @{
  */
  
/** @defgroup FLASH_Exported_Functions_Group1 Programming operation functions 
 *  @brief   Programming operation functions 
 *
@verbatim   
 ===============================================================================
                  ##### Programming operation functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to manage the FLASH 
    program operations.

@endverbatim
  * @{
  */

/**
  * @brief  Program byte, halfword or word at a specified address
  * @param  TypeProgram  Indicate the way to program at a specified address.
  *                           This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address  specifies the address to be programmed.
  * @param  Data specifies the data to be programmed
  * 
  * @retval HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint32_t Data)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    /* Process Locked */
    __HAL_LOCK(&pFlash);    

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  
    if(status == HAL_OK)
    {
        if(TypeProgram == FLASH_TYPEPROGRAM_BYTE)
        {
            /*Program byte (8-bit) at a specified address.*/
            FLASH_Program_Byte(Address, (uint8_t) Data);
        }
        if(TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
        {
            /*Program halfword (16-bit) at a specified address.*/
            FLASH_Program_HalfWord(Address, (uint16_t) Data);
        }
        if(TypeProgram == FLASH_TYPEPROGRAM_WORD)
        {
            /*Program word (32-bit) at a specified address.*/
            FLASH_Program_Word(Address, (uint32_t) Data);
        }
        
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);         
    }
    
    /* Process Unlocked */
    __HAL_UNLOCK(&pFlash);

    return status;    
}

/**
  * @brief  Continuously program 512 bytes (32 bits) in the specified address space.
  * @note   Operating this function will operate the flash address space with 
  *         a minimum unit of 128 words at one time.
  * @param  Address  specifies the address to be programmed.
  * @param  *Data    specifies the data to be programmed
  * @param  data_len     The length of the *Data(Unit: word)
  * @note   This parameter must be a multiple of 128 words.
  * @retval HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Program_Continue(uint32_t Address, uint32_t *Data,uint32_t data_len)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    /* Process Locked */
    __HAL_LOCK(&pFlash);    

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  
    if(status == HAL_OK)
    {
 
        while(data_len)
        {
            /*Program word (32-bit) at a specified address.*/
            FLASH_Continue_Program_Word(Address, Data);
            Address+=512;
            Data+=128;
            data_len-=128;              
        }
    }
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);  
    /* Process Unlocked */
    __HAL_UNLOCK(&pFlash);

    return status;    
}

/**
  * @brief  Program byte, halfword or word at a specified address  with interrupt enabled.
  * @param  TypeProgram  Indicate the way to program at a specified address.
  *                           This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address  specifies the address to be programmed.
  * @param  Data specifies the data to be programmed
  * 
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint32_t Data)
{
    HAL_StatusTypeDef status = HAL_OK;
     
    /* Process Locked */
    __HAL_LOCK(&pFlash);
    
    /* Enable Programming check bit */
    EFC->CTRL |=(1<<4);
    
    /* Enable Erasure check bit */
    EFC->CTRL |=(1<<5);   
    
    /* Enable the Interrupt source */
    __HAL_FLASH_ENABLE_IT(FLASH_IT_PVFE);
    __HAL_FLASH_ENABLE_IT(FLASH_IT_EVFE);
    
    pFlash.ProcedureOnGoing = FLASH_PROC_PROGRAM;
    pFlash.Address = Address;
    
    if(TypeProgram == FLASH_TYPEPROGRAM_BYTE)
    {
        /*Program byte (8-bit) at a specified address.*/
        FLASH_Program_Byte(Address, (uint8_t) Data);
    }
    if(TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
    {
        /*Program halfword (16-bit) at a specified address.*/
        FLASH_Program_HalfWord(Address, (uint16_t) Data);
    }
    if(TypeProgram == FLASH_TYPEPROGRAM_WORD)
    {
        /*Program word (32-bit) at a specified address.*/
        FLASH_Program_Word(Address, (uint32_t) Data);
    }
    
    return status;
}
  
/**
  * @brief  Flash writes back a word.
  * @note   If both erase and program operations are requested,
  *         Erase operation needs to be performed before programming.
  * @param  Address    Address to be written back.
  * @param  Data       Data to be written back.
  * @return 1
  */
#if(0)
HAL_StatusTypeDef HAL_FLASH_RewriteWord(uint32_t Address, uint32_t Data)
{
    uint32_t buff[FLASH_PAGE_SIZE/4];
    uint32_t *dst;
    uint32_t dst_addr;
    uint32_t page_addr;                         // Page base address 
    uint32_t i;
    
    HAL_StatusTypeDef status = HAL_OK;
    
    if(*(__IO uint32_t *)Address==Data)
    {
        return status;
    } 

    page_addr = Address&0xFFFFF000;                // for page size
    dst = (uint32_t *)(page_addr);
	for(i=0;i<(FLASH_PAGE_SIZE/4); i++)
	{
		buff[i]=*dst++;
	}
    
    buff[(Address-page_addr)/4] = Data;           // Change the target word content 

    HAL_FLASH_Erase_Page(page_addr);
    
    dst_addr = page_addr;
    for(i=0;i<(FLASH_PAGE_SIZE/4); i++)
    {
        FLASH_Program_Word(dst_addr,buff[i]);
        
        dst_addr +=4;
    } 
    return status; 
}
#endif

/**
  * @brief  Erases a specified page of the FLASH addr.
  * @param  Address    The page with the address will be erased.
  *                 This parameter can be a value of @ref FLASHEx_Pages  
  * @return none
  */
HAL_StatusTypeDef HAL_FLASH_Erase_Page(uint32_t Address)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    /* Turn off the total interrupt.*/
    __set_PRIMASK(1); 

    /* Enable page erasure. */
    EFC->CTRL = (1<<2);	
    
     /* Unlock FLASH erase write protection */
    HAL_FLASH_Unlock(); 
    *((__IO uint32_t *)(Address)) = 0;
    
    /* Wait for flash programming to complete. */    
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDYS) == RESET);  
    EFC->CTRL = 0;  

    /* Turn on the total interrupt. */
    __set_PRIMASK(0);                 
    
    /* Lock FLASH erase write protection */
    HAL_FLASH_Lock();
     
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

    return status; 
}

/**
  * @brief  Perform a mass erase or erase the specified FLASH memory pages
  * @param[in]  pEraseInit pointer to an FLASH_EraseInitTypeDef structure that
  *         contains the configuration information for the erasing.
  *
  * @param[out]  PageError pointer to variable  that
  *         contains the configuration information on faulty sector in case of error
  *         (0xFFFFFFFFU means that all the sectors have been correctly erased)
  *
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    __IO uint32_t index;
    
    /* Process Locked */
    __HAL_LOCK(&pFlash);    
        
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
    
    if (status == HAL_OK)
    {  
        /*Initialization of SectorError variable*/
        *PageError = 0xFFFFFFFFU;
              
        if (pEraseInit->TypeErase == FLASH_TYPEERASE_PAGES)
        {
            /* Erase from the initial page address to the end page address*/
            for (index=pEraseInit->Page_Address; index <= pEraseInit->NbPages_Address;)
            {
                
                HAL_FLASH_Erase_Page(index);
                index =index+FLASH_PAGE_SIZE;
                
                /* Wait for last operation to be completed */
                status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
    
                if (status != HAL_OK)
                {
                    /* In case of error, stop erase procedure and return the faulty sector*/
                    *PageError = index;
                    break;
                }
            }        
        }
        if (pEraseInit->TypeErase == FLASH_TYPEERASE_MASSERASE)
        {
            /*Mass erase to be done*/
            FLASH_MassErase();
    
            /* Wait for last operation to be completed */
            status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

        }          
    }
    
    /* Process Unlocked */
    __HAL_UNLOCK(&pFlash);    
    
    return status;    
}

/**
  * @brief  Erases all of the FLASH.
  * @param  none   
  * @return none
  */
static void FLASH_MassErase(void)
{
    /* Turn off the total interrupt */
    __set_PRIMASK(1);            
    /* Enable full erase of main area. */   
	EFC->CTRL = (1<<3);	
    
    /* Unlock FLASH erase write protection */
    HAL_FLASH_Unlock();
    
    *((__IO uint32_t *)(0x00000000)) = 0;
    
    /* Wait for flash programming to complete. */    
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDYS) == RESET){;} 
	EFC->CTRL = 0;
    
    /* Turn on the total interrupt.*/
    __set_PRIMASK(0);    
    
    /* Lock FLASH erase write protection */
    HAL_FLASH_Lock();    
}

/**
  * @brief  eflash_set_keyctrl
  * @param  Sha_icv                 location of SHA_ICV
  * @param  Aes_key2_position       location to read AES_KEY2
  * @param  Aes_key1_position       location to read AES_KEY1
  * @return note
  */
void HAL_FLASH_SetKeyctrl(uint8_t Sha_icv,uint8_t Aes_key2_position,uint8_t Aes_key1_position)
{
    uint32_t temp =0;
    
    /* Select the location of SHA_IC */
    temp |=(Sha_icv<<6);   
    
    /* Select the location to read AES_KEY2 */
    temp |=(Aes_key2_position<<2);       
    
    /* Select the location to read AES_KEY1 */
    temp |=(Aes_key1_position<<0);          


    /* Wait for flash programming to complete */    
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDYS) == RESET); 
}

/**
  * @brief This function handles FLASH interrupt request.
  * @retval None
  */
void HAL_FLASH_IRQHandler(void)
{
    uint32_t addresstmp = 0U;
    
    /* Check FLASH operation error flags */
    if(__HAL_FLASH_GET_ITFLAG((HAL_FLASH_ERROR_PVFS | HAL_FLASH_ERROR_EVFS)) != RESET)
    {
        if(pFlash.ProcedureOnGoing == FLASH_PROC_ERASE)
        {
            /*return the faulty page*/
            addresstmp = pFlash.Page;
            pFlash.Page = 0xFFFFFFFFU;
        }
        else
        {
            /*return the faulty address*/
            addresstmp = pFlash.Address;
        }
        
        /*Save the Error code*/
        FLASH_SetErrorCode();
        
        /* FLASH error interrupt user callback */
        HAL_FLASH_OperationErrorCallback(addresstmp);
        
        /*Stop the procedure ongoing*/
        pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
    }
    /* Check FLASH End of Operation flag  */
    if(__HAL_FLASH_GET_ITFLAG(FLASH_FLAG_OPDS) != RESET)
    {  
        /* Clear FLASH End of Operation pending bit */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPDS);  

        if(pFlash.ProcedureOnGoing == FLASH_PROC_ERASE)
        {
            /*Nb of page to erased can be decreased*/
            pFlash.NbPagesToErase--;
            
            /* Check if there are still pages to erase*/
            if(pFlash.NbPagesToErase != 0U)
            {
                addresstmp = pFlash.Page;
                /*Indicate user which page has been erased*/
                HAL_FLASH_EndOfOperationCallback(addresstmp);
                
                /*Increment page number*/
                pFlash.Page++;
                addresstmp = pFlash.Page;
                HAL_FLASH_Erase_Page(addresstmp);
            }
            else
            {
                /*No more pages to Erase, user callback can be called.*/
                /*Reset Page and stop Erase sectors procedure*/
                pFlash.Page = addresstmp = 0xFFFFFFFFU;
                pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
                        
                /* FLASH EOP interrupt user callback */
                HAL_FLASH_EndOfOperationCallback(addresstmp);
            }
        }      
    }        
    if(pFlash.ProcedureOnGoing == FLASH_PROC_NONE)
    {
        CLEAR_BIT(EFC->INTSTATUS, (FLASH_FLAG_OPDS | FLASH_FLAG_PVFS | FLASH_FLAG_EVFS));

        /* Disable all the Interrupt of FLASH interrupt */
        __HAL_FLASH_DISABLE_IT(FLASH_IT_ALL);

        /* Process Unlocked */
        __HAL_UNLOCK(&pFlash);
    }
}

/**
  * @brief  FLASH end of operation interrupt callback
  * @param  ReturnValue The value saved in this parameter depends on the ongoing procedure
  *                  Mass Erase: Bank number which has been requested to erase
  *                  Pages Erase: Page which has been erased 
  *                    (if 0xFFFFFFFFU, it means that all the selected pages have been erased)
  *                  Program: Address which was selected for data program
  * @retval None
  */
__weak void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
    /* Prevent unused argument(s) compilation warning */
  UNUSED(ReturnValue);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_FLASH_EndOfOperationCallback could be implemented in the user file
   */
}

/**
  * @brief  FLASH operation error interrupt callback
  * @param  ReturnValue The value saved in this parameter depends on the ongoing procedure
  *                 Mass Erase: Bank number which has been requested to erase
  *                 Pages Erase: Page number which returned an error
  *                 Program: Address which was selected for data program
  * @retval None
  */
__weak void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
      /* Prevent unused argument(s) compilation warning */
  UNUSED(ReturnValue);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_FLASH_OperationErrorCallback could be implemented in the user file
   */ 
}

/**
  * @}
  */
/** @defgroup FLASH_Exported_Functions_Group2 Peripheral Control functions 
 *  @brief   management functions 
 *
@verbatim   
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the FLASH 
    memory operations.

@endverbatim
  * @{
  */

/**
  * @brief  Unlock the FLASH control register access
  * @note   This register is relocked when programming or erasing is started.
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Unlock(void)
{ 
    HAL_StatusTypeDef status = HAL_OK;
    
    EFC->SEC = 0x55AAAA55;			        // Unlock FLASH erase write protection 
   
    return status;
}
/**
  * @brief  Locks the FLASH control register access
  * @note   This register is relocked when programming or erasing is started.
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Lock(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    EFC->SEC = 0x55aa0000;			        // Lock FLASH erase write protection

    return status;
}

/**
  * @}
  */

/** @defgroup FLASH_Exported_Functions_Group3 Peripheral State and Errors functions 
 *  @brief   Peripheral Errors functions 
 *
@verbatim   
 ===============================================================================
                ##### Peripheral Errors functions #####
 ===============================================================================  
    [..]
    This subsection permits to get in run-time Errors of the FLASH peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Get the specific FLASH error flag.
  * @retval FLASH_ErrorCode: The returned value can be a combination of:
  *         @arg HAL_FLASH_ERROR_PVFS    FLASH Write check error 
  *         @arg HAL_FLASH_ERROR_EVFS    FLASH Erasure check error 
  *         @arg FLASH_FLAG_RDYS         FLASH End of Operation flag 
  *         @arg FLASH_FLAG_CONPROGRDY   FLASH is in the state of waiting for the next continuous programming. 
  *         @arg FLASH_FLAG_VDDS         FLASH Low voltage warning flag  
  *         @arg FLASH_FLAG_LPSPD        FLASH Power down mode flag 
  *         @arg FLASH_FLAG_LPSLEEP      FLASH Sleeping mode flag   
  *         @arg FLASH_FLAG_LPSLVDD      FLASH Low voltage operation flag  
  *         @arg FLASH_FLAG_VDDLS        VDD voltage low interrupt status bit flag  
  *         @arg FLASH_FLAG_OPDS         FLASH operation or erase successfully flag
  */
uint32_t HAL_FLASH_GetError(void)
{ 
     return pFlash.ErrorCode;
}

/**
  * @brief  Wait for a FLASH operation to complete.
  * @param  Timeout maximum flash operationtimeout
  * @retval HAL Status
  */
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout)
{ 
    uint32_t tickstart = 0U;
  
    /* Clear Error Code */
    pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;
 
    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
       Even if the FLASH operation fails, the BUSY flag will be reset and an error
       flag will be set */
    /* Get tick */    
    tickstart = HAL_GetTick();
  
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDYS) == RESET) 
    { 
        if(Timeout != HAL_MAX_DELAY)
        {
            if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
            {
                return HAL_TIMEOUT;
            }
        } 
    }
   if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_CONPROGRDY) != RESET)     
    {    
        /*Save the error code*/
        FLASH_SetErrorCode();
        return HAL_ERROR;
    }  
    /* If there is no error flag set */
    return HAL_OK;    
}

/**
  * @brief  Program word (32-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         1.8V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_Word(uint32_t Address, uint32_t Data)
{
    /* Turn off the total interrupt. */
    __set_PRIMASK(1); 

    /* Enable flash's one-time programming mode. */   
	EFC->CTRL = (1<<0);	
    
    /* Turn on the check enable bit. */   
    EFC->CTRL |= (1<<4);
    
    /* Unlock FLASH erase write protection */
    HAL_FLASH_Unlock();
    
    *((__IO uint32_t *)(Address)) = Data;
    
    /* Wait for flash programming to complete. */    
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDYS) == RESET); 
	EFC->CTRL = 0;
    
    /* Turn on the total interrupt */
	__set_PRIMASK(0); 
    
    /* Lock FLASH erase write protection */
    HAL_FLASH_Lock(); 
}

/**
  * @brief  Program a half-word (16-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         1.8V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_HalfWord(uint32_t Address, uint16_t Data)
{
      
    /* Turn off the total interrupt. */
    __set_PRIMASK(1); 

    /* Enable flash's one-time programming mode. */   
	EFC->CTRL = (1<<0);	
    
    /* Turn on the check enable bit. */   
    EFC->CTRL |= (1<<4);  		

    /* Unlock FLASH erase write protection */
    HAL_FLASH_Unlock();
    
    *((__IO uint16_t *)(Address)) = Data;
    
    /* Wait for flash programming to complete. */    
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDYS) == RESET); 
	EFC->CTRL = 0;
    
    /* Turn on the total interrupt */
	__set_PRIMASK(0);     

    /* Lock FLASH erase write protection */
    HAL_FLASH_Lock();
}

/**
  * @brief  Program byte (8-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         1.8V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_Byte(uint32_t Address, uint8_t Data)
{
    /* Turn off the total interrupt. */
    __set_PRIMASK(1); 

    /* Enable flash's one-time programming mode. */   
	EFC->CTRL = (1<<0);
    
    /* Turn on the check enable bit. */   
    EFC->CTRL |= (1<<4);  
    
    /* Unlock FLASH erase write protection*/
    HAL_FLASH_Unlock();
    
    *((__IO uint8_t *)(Address)) = Data;
    
    /* Wait for flash programming to complete. */    
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDYS) == RESET); 
	EFC->CTRL = 0;
    
    /* Turn on the total interrupt */
	__set_PRIMASK(0);  
                         
    /* Lock FLASH erase write protection */
    HAL_FLASH_Lock(); 
}

/**
  * @brief  Continuously program 512 bytes (32 bits) in the specified address space.
  * @note   If both erase and program operations are requested,
  *         erase operation needs to be performed before programming.
  * @param  Address            start address to be continuously programmed.
  *                         This parameter can be any address in main flash zone.  
  * @param  *Data     specify the data to be programmed continuously.
  * @return none
  */
static void FLASH_Continue_Program_Word(uint32_t Address, uint32_t *Data)
{
    uint32_t i;
    
    /*Enable the continuous programming mode of flash. */
	EFC->CTRL = (1<<1);		

	for(i=0; i<=127; i++)
	{
        /* Unlock FLASH erase write protection*/
        HAL_FLASH_Unlock();
		*((__IO uint32_t *)(Address+i*4)) = Data[i];
	}
    /* write 0 to end the continuous programming */
	EFC->STATUS  = 0x0;                    
	EFC->CTRL = 0;

    /* Lock FLASH erase write protection */
    HAL_FLASH_Lock();
}

/**
  * @brief  Set the specific FLASH error flag.
  * @retval None
  */
static void FLASH_SetErrorCode(void)
{
     /* FLASH is in the state of waiting for the next continuous programming. */
    if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_CONPROGRDY) != RESET)
    {
        pFlash.ErrorCode |= FLASH_FLAG_CONPROGRDY;
    }  
    /* FLASH Write check error flag */
    if(__HAL_FLASH_GET_ITFLAG(FLASH_FLAG_PVFS) != RESET)
    {
        pFlash.ErrorCode |= HAL_FLASH_ERROR_PVFS;
    
        /* Clear FLASH Write check error pending bit */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PVFS);
    } 
    /*FLASH Erasure check error flag */
    if(__HAL_FLASH_GET_ITFLAG(FLASH_FLAG_EVFS) != RESET)
    {
        pFlash.ErrorCode |= HAL_FLASH_ERROR_EVFS;
    
        /* Clear FLASH Erasure check error pending bit */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EVFS);
    }   
}

/**
  * @}
  */
#endif /* HAL_FLASH_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/


