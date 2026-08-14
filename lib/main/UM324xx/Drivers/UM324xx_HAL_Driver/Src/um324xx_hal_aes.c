/**
  ******************************************************************************
  * @file    um324xx_hal_aes.c
  * @author  MCU Team
  * @version V1.00
  * @date    10-February-2023
  * @brief   AES HAL module driver.
  ****************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023 Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup AES AES
  * @brief AES HAL module driver
  * @{
  */
#ifdef HAL_AES_MODULE_ENABLED  
/* Private typedef -----------------------------------------------------------*/
/** @defgroup AES_Private_typedefs AES Private Typedefs
  * @{
  */


/**
  * @}
  */ 
  
/* Private define ------------------------------------------------------------*/
/** @defgroup AES_Private_define AES Private Define
  * @{
  */


/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @defgroup AES_Private_macro AES Private Macro
  * @{
  */ 
  
  
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup AES_Private_variables AES Private Variables
  * @{
  */ 


/**
  * @}
  */
  

/* Private function prototypes -----------------------------------------------*/
/** @addtogroup AES_function_prototypes  AES function prototypes
  * @{
  */

static void HAL_AES_Config(AES_HandleTypeDef *haes);

/**
  * @}
  */
  


/* Exported functions ---------------------------------------------------------*/
/** @defgroup AES_Exported_Functions AES Exported Functions
  * @{
  */  

/**
  * @brief  AES Initial configuration
  * @param  haes Pointer to the AES_HandleTypeDef structure that contains 
  *              configuration information for the specified AES module
  * @return None
  */
HAL_StatusTypeDef HAL_AES_Init(AES_HandleTypeDef *haes)
{
    /* Check Null pointer */
    if((haes == NULL)||(haes->Instance == NULL))
    {
        return HAL_ERROR;
    }

    /* AES Basic configuration */
    HAL_AES_Config(haes);
    
    /* AES interrupt configuration */
    HAL_AES_IrqConfig(haes);
    
    __HAL_AES_CLEAR_STATUS(AES_CBCDONE_STATE | AES_KEYDONE_STATE | AES_CRYPTDONE_STATE);

    return HAL_OK;
}

/**
  * @brief  AES Basic configuration
  * @param  haes Pointer to the AES_HandleTypeDef structure that contains 
  *              configuration information for the specified AES module
  * @return None
  */
static void HAL_AES_Config(AES_HandleTypeDef *haes)
{
    uint32_t tmpctrl;
    uint32_t temp1;    
    
    temp1 = (uint32_t)(AES_CONTROL_ALL_ROUND_Msk | AES_CONTROL_VAES_EN_Msk | \
                AES_CONTROL_ECB_Msk | AES_CONTROL_SWAP_Msk);
    
    tmpctrl = (uint32_t)(haes->Init.CalNum | haes->Init.VaesEn | haes->Init.Mode | \
               haes->Init.DateSwap);
    
    MODIFY_REG(haes->Instance->CONTROL,temp1,tmpctrl);

}

/**
  * @brief  AES Indicates the interrupt configuration
  * @param  haes Pointer to the AES_HandleTypeDef structure that contains 
  *              configuration information for the specified AES module
  * @return HAL_StatusTypeDef
  *         @retval HAL_OK    nothing wrong
  *         @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_IrqConfig(AES_HandleTypeDef *haes)
{
    
    /* Check Null pointer */
    if((haes == NULL)||(haes->Instance == NULL))
    {
        return HAL_ERROR;
    }        
    
    if(haes->Init.IntEn == AES_INT_ENABLE)
    {
        haes->Instance->CONTROL |= AES_INT_ENABLE;
        NVIC_ClearPendingIRQ(AES_IRQn);
        NVIC_EnableIRQ(AES_IRQn);
    
    }

    return HAL_OK;
}

/**
  * @brief  AES Write key
  * @param  haes          Pointer to the AES_HandleTypeDef structure that contains 
  *                       configuration information for the specified AES module
  * @param  pAeskey       Key value
  * @param  KeyLength     Key length
  *         @arg AES_KEY_LENGTH_128: The AES key length mode is 128
  *         @arg AES_KEY_LENGTH_192: The AES key length mode is 192
  *         @arg AES_KEY_LENGTH_256: The AES key length mode is 256
  * @return HAL_StatusTypeDef
  *         @retval HAL_OK    nothing wrong
  *         @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_Write_Key(AES_HandleTypeDef *haes, uint8_t* pAeskey, uint32_t KeyLength)
{
    uint32_t aes_key_value[8];
        /* Check Null pointer */
    if((haes == NULL)||(haes->Instance == NULL))
    {
        return HAL_ERROR;
    }    
    
    if(KeyLength == AES_KEY_LENGTH_128)
    {
        
        MODIFY_REG(haes->Instance->CONTROL,AES_CONTROL_KEY_MODE_Msk,AES_KEY_LENGTH_128);
        
        for(uint8_t i = 0;i < 4;i++)
        {
            aes_key_value[i] = (((uint32_t)pAeskey[0+i*4]) | ((uint32_t)pAeskey[1+i*4]<<8) | \
                                ((uint32_t)pAeskey[2+i*4]<<16) | ((uint32_t)pAeskey[3+i*4]<<24));
            
            haes->Instance->KEYIN = aes_key_value[i];
        }
        
        /**< Initiating key extension*/
        SET_BIT(haes->Instance->CONTROL,AES_KEY_START);
        
        /**< The AES key extension is complete*/
        while ((haes->Instance->STATE&(0x02)) != 0x02);
        /**< Clear flag bit*/
        SET_BIT(haes->Instance->CONTROL,0x02);
    }
    else if(KeyLength == AES_KEY_LENGTH_192)
    {
        
        MODIFY_REG(haes->Instance->CONTROL,AES_CONTROL_KEY_MODE_Msk,AES_KEY_LENGTH_192);
        
        for(uint8_t i = 0;i < 6;i++)
        {
            aes_key_value[i] = (((uint32_t)pAeskey[0+i*6]) | ((uint32_t)pAeskey[1+i*6]<<8) | \
                            ((uint32_t)pAeskey[2+i*6]<<16) | ((uint32_t)pAeskey[3+i*6]<<24));
            
            haes->Instance->KEYIN = aes_key_value[i];
        }
        
        /**< Initiating key extension*/
        SET_BIT(haes->Instance->CONTROL,AES_KEY_START);
        /**< The AES key extension is complete*/
        while ((haes->Instance->STATE&(0x02)) != 0x02);
        /**< Clear flag bit*/
        SET_BIT(haes->Instance->CONTROL,0x02);
    }
    else if(KeyLength == AES_KEY_LENGTH_256)
    {
        
        MODIFY_REG(haes->Instance->CONTROL,AES_CONTROL_KEY_MODE_Msk,AES_KEY_LENGTH_256);
        
        for(uint8_t i = 0;i < 8;i++)
        {
            aes_key_value[i] = (((uint32_t)pAeskey[0+i*8]) | ((uint32_t)pAeskey[1+i*8]<<8) | \
                            ((uint32_t)pAeskey[2+i*8]<<16) | ((uint32_t)pAeskey[3+i*8]<<24));
            
            haes->Instance->KEYIN = aes_key_value[i];
        }

        /**< Initiating key extension*/
        SET_BIT(haes->Instance->CONTROL,AES_KEY_START);
        
        /**< The AES key extension is complete*/
        while ((haes->Instance->STATE&(0x02)) != 0x02);
        
        /**< Clear flag bit*/
        SET_BIT(haes->Instance->CONTROL,0x02);
    }    
    else
    {
        return HAL_ERROR;
    }
	
    return HAL_OK;
}

/**
  * @brief  AES Write plaintext data
  * @param  haes Pointer to the AES_HandleTypeDef structure that contains 
  *              configuration information for the specified AES module
  * @param  data Plaintext data
  * @return HAL_StatusTypeDef
  *         @retval HAL_OK    nothing wrong
  *         @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_Write_Data(AES_HandleTypeDef *haes, uint8_t* data)
{
    uint32_t aes_data_value[4];
    
    
    /* Check Null pointer */
    if((haes == NULL)||(haes->Instance == NULL))
    {
        return HAL_ERROR;
    }    
    
    for(uint8_t i = 0 ; i <4 ; i++)
    {
         aes_data_value[i] = ((uint32_t)data[3+4*i]<<24) | ((uint32_t)data[2+4*i]<<16) | \
            ((uint32_t)data[1+4*i]<<8) | (uint32_t)data[0+4*i];
        
        haes->Instance->DATAIN = aes_data_value[i];
    }
    
    return HAL_OK;  
}

/**
  * @brief  AES Read ciphertext/plaintext data
  * @param  haes Pointer to the AES_HandleTypeDef structure that contains 
  *              configuration information for the specified AES module
  * @param  data Ciphertext/Plaintext data
  * @return HAL_StatusTypeDef
  *         @retval HAL_OK    nothing wrong
  *         @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_Read_Data(AES_HandleTypeDef *haes, uint8_t* data)
{
    uint32_t temp;

    /* Check Null pointer */
    if((haes == NULL)||(haes->Instance == NULL))
    {
        return HAL_ERROR;
    }

    for(uint8_t i = 0 ; i <4 ; i++)
    {
        temp = haes->Instance->DATAOUT;
        data[0+4*i] = temp;
        data[1+4*i] = temp>>8;
        data[2+4*i] = temp>>16;
        data[3+4*i] = temp>>24;
    }

    return HAL_OK;
    
}

/**
  * @brief  AES Write the initial vector data
  * @param  haes     Pointer to the AES_HandleTypeDef structure that contains 
  *                  configuration information for the specified AES module
  * @param  pAesivin Initial vector data
  * @return HAL_StatusTypeDef
  *         @retval HAL_OK    nothing wrong
  *         @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_Write_Ivin(AES_HandleTypeDef *haes, uint8_t* pAesivin)
{
    /* Check Null pointer */
    if((haes == NULL)||(haes->Instance == NULL))
    {
        return HAL_ERROR;
    }

    uint32_t aes_ivin_value[4];
    
    for(uint8_t i = 0;i < 4;i++)
    {
        aes_ivin_value[i] = (((uint32_t)pAesivin[0+i*4]) | ((uint32_t)pAesivin[1+i*4]<<8) | \
                    ((uint32_t)pAesivin[2+i*4]<<16) | ((uint32_t)pAesivin[3+i*4]<<24));
        
        AES->IVIN = aes_ivin_value[i];
    }

    return HAL_OK;  
}

/**
  * @brief  AES Start encryption/decryption operations
  * @param  haes Pointer to the AES_HandleTypeDef structure that contains 
  *              configuration information for the specified AES module
  * @param  CRYPT_Type Operation type
  *         @arg AES_CRYPT_DECRYP: Decryption operation
  *         @arg AES_CRYPT_ENCRYP: Encryption operation
  * @return HAL_StatusTypeDef
  *         @retval HAL_OK    nothing wrong
  *         @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_CRYPT_Start(AES_HandleTypeDef *haes, uint8_t CRYPT_Type)
{
    /* Check Null pointer */
    if((haes == NULL)||(haes->Instance == NULL))
    {
        return HAL_ERROR;
    }    

    /**< encrypt*/
    if(CRYPT_Type == AES_CRYPT_ENCRYP)
    {
        CLEAR_BIT(haes->Instance->CONTROL,AES_CONTROL_CRYPT_Msk);
    }
    /**< decrypt*/
    else if(CRYPT_Type == AES_CRYPT_DECRYP)
    {
        SET_BIT(haes->Instance->CONTROL,AES_CONTROL_CRYPT_Msk);
    }
    else
    {
        return HAL_ERROR;
    }
    
    /**< Start encryption/decryption operations*/
    SET_BIT(haes->Instance->CONTROL,AES_CRYPT_START);
    
    return HAL_OK;
}

/**
  * @brief  Reference interrupt function
  * @param  haes Pointer to the AES_HandleTypeDef structure that contains 
  *              configuration information for the specified AES module
  * @return None
  */
void HAL_AES_IRQHandler(AES_HandleTypeDef *haes)
{
   /**< Clear the completed flag bit*/
    __HAL_AES_CLEAR_STATUS(AES_CRYPTDONE_STATE);  
    
    if((__HAL_AES_GET_STATUS(haes) & AES_CRYPTDONE_STATE) == AES_CRYPTDONE_STATE)
    {
        /*If the callback function is registered*/
        if(haes->AES_Callback != NULL)  
        {
            /*Callback interface to perform registration*/
            haes->AES_Callback();
        }
    }  
}

/**
  * @}
  */

#endif /* HAL_AES_MODULE_ENABLED */

/**
  * @}
  */


/**
  * @}
  */


/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/

