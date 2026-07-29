/**
  ******************************************************************************
  * @file    um324xx_hal_aes.h
  * @author  MCU Team
  * @version V1.00 
  * @date    10-February-2023  
  * @brief   Header file of AES HAL module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023 Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/ 
#ifndef __UM324XX_HAL_AES_H__
#define __UM324XX_HAL_AES_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */


/** @addtogroup AES
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup AES_Exported_Types AES Exported Types
  * @{
  */

/**
  * @brief AES Init structure definition
  */
typedef struct
{
    uint32_t CalNum;             /**< The total number of rounds when pseudo-AES arithmetic is turned on
                                 This parameter can be a value of @ref AES_CALNUM_define */

    uint32_t VaesEn;            /**< Pseudo-aes operation enable bit
                                 This parameter can be a value of @ref AES_VAES_EN_define */

    uint32_t  Mode;              /**< AES mode indicator bit
                                 This parameter can be a value of @ref AES_MODE_define */

    uint32_t  DateSwap;          /**< Data Input/Output SWAP mode
                                 This parameter can be a value of @ref AES_SWAP_define */

    uint32_t  IntEn;             /**< Interrupt enablement
                                 This parameter can be a value of @ref AES_INT_define */

}AES_InitTypeDef;

/**
  * @brief  DIV Handle Structure definition
  */
typedef struct
{
  AES_TypeDef                 *Instance;    /*!< Register base address        */

  AES_InitTypeDef             Init;         /*!< DIV Init object           */

  void (*AES_Callback)(void);               /**< DIV overflow interrupt callback */
    
} AES_HandleTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup AES_Exported_Constants AES Exported Constants
  * @{
  */

/** @defgroup AES_CALNUM_define AES CALNUM define
  * @{
  */
#define AES_ALL_ROUND_2             0x00000000UL
#define AES_ALL_ROUND_4             AES_CONTROL_ALL_ROUND_1
#define AES_ALL_ROUND_8             AES_CONTROL_ALL_ROUND_2
#define AES_ALL_ROUND_16            AES_CONTROL_ALL_ROUND_3
#define AES_ALL_ROUND_32            AES_CONTROL_ALL_ROUND_4
#define AES_ALL_ROUND_64            AES_CONTROL_ALL_ROUND_5

/**
  * @}
  */

/** @defgroup AES_VAES_EN_define AES VAES EN define
  * @{
  */
#define AES_VAES_ENABLE             AES_CONTROL_VAES_EN
#define AES_VAES_DISABLE            0x00000000UL

/**
  * @}
  */

/** @defgroup AES_KEY_LENGTH_define AES KEY LENGTH define
  * @{
  */
#define AES_KEY_LENGTH_128          0x00000000UL
#define AES_KEY_LENGTH_192          AES_CONTROL_KEY_MODE_0
#define AES_KEY_LENGTH_256          AES_CONTROL_KEY_MODE_1

/**
  * @}
  */

/** @defgroup AES_MODE_define AES MODE define
  * @{
  */
#define AES_MODE_CBC                AES_CONTROL_ECB
#define AES_MODE_ECB                0x00000000UL
/**
  * @}
  */

/** @defgroup AES_SWAP_define AES SWAP define
  * @{
  */
#define AES_DATA_SWAP_ENABLE        AES_CONTROL_SWAP
#define AES_DATA_SWAP_DISABLE       0x00000000UL

/**
  * @}
  */

/** @defgroup AES_INT_define AES INT define
  * @{
  */
#define AES_INT_ENABLE              AES_CONTROL_INT_EN
#define AES_INT_DISABLE             0x00000000UL

/**
  * @}
  */

/** @defgroup AES_CAL_TYPE_define AES CAL TYPE define
  * @{
  */
#define AES_CRYPT_ENCRYP            0x00000000UL
#define AES_CRYPT_DECRYP            AES_CONTROL_CRYPT
/**
  * @}
  */

/** @defgroup AES_CAL_EN_define AES CAL EN define
  * @{
  */
  
#define AES_KEY_START               AES_CONTROL_KEY_START
#define AES_CRYPT_START             AES_CONTROL_CRYPT_START

/**
  * @}
  */

/** @defgroup AES_STATE_define AES STATE define
  * @{
  */
#define AES_CBCDONE_STATE           AES_STATE_CBCDONE
#define AES_KEYDONE_STATE           AES_STATE_KEY_DONE
#define AES_CRYPTDONE_STATE         AES_STATE_CRYPT_DONE

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup AES_Exported_Macros AES Exported Macros
  * @{
  */

/**
  * @brief  Get Status
  * @param  __HANDLE__ AESHandleTypeDef pointer
  * @return Aes Status
  */
#define __HAL_AES_GET_STATUS(__HANDLE__)            READ_REG((__HANDLE__)->Instance->STATE)

/**
  * @brief  Clear Status
  * @param  __HANDLE__ AES_HandleTypeDef pointer
  * @return None
  */
#define __HAL_AES_CLEAR_STATUS(__STATUS__)            WRITE_REG(AES->STATE,__STATUS__)

/**
  * @brief  Wait for the calculation to complete
  * @param  __HANDLE__ AES_HandleTypeDef pointer
  * @return None
  */
#define __HAL_AES_WAIT_CALCOMPLETE(__HANDLE__)  do { \
                                                     \
                                     } while(((__HANDLE__)->Instance->STATE&AES_CRYPTDONE_STATE) != AES_CRYPTDONE_STATE)


/**
  * @brief  Wait for the CBC to complete
  * @param  __HANDLE__ AES_HandleTypeDef pointer
  * @return None
  */
#define __HAL_AES_WAIT_CBCDONE(__HANDLE__)   do {     \
                                                    \
                                    } while(((__HANDLE__)->Instance->STATE&AES_CBCDONE_STATE) != AES_CBCDONE_STATE)


/**
  * @brief  Wait for the STATE to complete
  * @param  __HANDLE__ AES_HandleTypeDef pointer
  * @return None
  */
#define __HAL_AES_WAIT_STATE_COMPLETE(__STATE__)   do {     \
                                                    \
                                    } while((AES->STATE&(__STATE__)) != (__STATE__))

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/

/** @addtogroup AES_Exported_Functions
  * @{
  */


/** @addtogroup AES_Exported_Functions_Group1
  * @{
  */

/**< Initialization and de-initialization functions *****************************/
/**
  * @brief  AES Initial configuration
  * @param  haes Pointer to the AES_HandleTypeDef structure that contains 
  *                 configuration information for the specified AES module
  * @return None
  */
HAL_StatusTypeDef HAL_AES_Init(AES_HandleTypeDef *haes);

/**
  * @}
  */

/* Peripheral Control functions ***********************************************/
/** @defgroup AES_Exported_Functions_Group2 Peripheral Control functions
  * @{
  */

/**
  * @brief  AES Indicates the interrupt configuration
  * @param  haes Pointer to the AES_HandleTypeDef structure that contains 
  *              configuration information for the specified AES module
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_IrqConfig(AES_HandleTypeDef *haes);

/**
  * @brief  AES Write key
  * @param  haes          Pointer to the AES_HandleTypeDef structure that contains 
  *                       configuration information for the specified AES module
  * @param  pAeskey       Key value
  * @param  KeyLength     Key length
  *          @arg AES_KEY_LENGTH_128: The AES key length mode is 128
  *          @arg AES_KEY_LENGTH_192: The AES key length mode is 192
  *          @arg AES_KEY_LENGTH_256: The AES key length mode is 256
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_Write_Key(AES_HandleTypeDef *haes, uint8_t* pAeskey, uint32_t KeyLength);

/**
  * @brief  AES Write plaintext data
  * @param  haes          Pointer to the AES_HandleTypeDef structure that contains 
  *                       configuration information for the specified AES module
  * @param  data          Plaintext data
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_Write_Data(AES_HandleTypeDef *haes, uint8_t* data);

/**
  * @brief  AES Write the initial vector data
  * @param  haes          Pointer to the AES_HandleTypeDef structure that contains 
  *                       configuration information for the specified AES module
  * @param  pAesivin      Initial vector data
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_Write_Ivin(AES_HandleTypeDef *haes, uint8_t* pAesivin);


/**
  * @brief  AES Start encryption/decryption operations
  * @param  haes          Pointer to the AES_HandleTypeDef structure that contains 
  *                       configuration information for the specified AES module
  * @param  CRYPT_Type    Operation type
  *            @arg AES_CRYPT_DECRYP: Decryption operation
  *            @arg AES_CRYPT_ENCRYP: Encryption operation
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_CRYPT_Start(AES_HandleTypeDef *haes, uint8_t CRYPT_Type);

/**
  * @brief  AES Read ciphertext/plaintext data
  * @param  haes          Pointer to the AES_HandleTypeDef structure that contains 
  *                       configuration information for the specified AES module
  * @param  data          Ciphertext/Plaintext data
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_AES_Read_Data(AES_HandleTypeDef *haes, uint8_t* data);


/**
  * @brief  Reference interrupt function
  * @param  haes Pointer to the AES_HandleTypeDef structure that contains 
  *              configuration information for the specified AES module
  * @return None
  */
void HAL_AES_IRQHandler(AES_HandleTypeDef *haes);


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */





#ifdef __cplusplus
}
#endif

#endif /* __UM32x42x_HAL_AES_H__ */


