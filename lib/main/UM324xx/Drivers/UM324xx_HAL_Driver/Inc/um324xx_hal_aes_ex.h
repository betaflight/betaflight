 /**
  ******************************************************************************
  * @file     um324xx_hal_aes_ex.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-03-21  
  * @brief   
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_AES_EX_H__
#define __UM324XX_HAL_AES_EX_H__

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"
#include "um324xx.h"
/** @addtogroup UM324xx_HAL_Driver
  * @{
  */
//#if defined (AES_EX)
/** @addtogroup AES_EX
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup AES_EX_Exported_Types AES_EX Exported Types
  * @{
  */

/**
  * @brief AES_EX Init Structure definition
  */

typedef struct
{
    uint32_t DataType;                   /*!< 32-bit data, 16-bit data, 8-bit data or 1-bit string.
                                            This parameter can be a value of @ref CRYP_Data_Type */
    uint32_t KeySize;                    /*!< Used only in AES mode : 128, 256 bit key length in AES.
                                            128 or 256 bit key length in TinyAES This parameter can be a value of @ref AES_Key_Size */
    uint32_t *pKey;                      /*!< The key used for encryption/decryption */
    uint32_t *pInitVect;                 /*!< The initialization vector used also as initialization
                                            counter in CTR mode */
    uint32_t Algorithm;                  /*!<  DES/ TDES Algorithm ECB/CBC
                                            AES Algorithm ECB/CBC/CTR/GCM or CCM
                                            This parameter can be a value of @ref AES_Algorithm_Mode */
    uint32_t *Header;                    /*!< used only in AES GCM and CCM Algorithm for authentication,
                                            GCM : also known as Additional Authentication Data
                                            CCM : named B1 composed of the associated data length and Associated Data. */
    uint32_t HeaderSize;                /*!< The size of header buffer in word  */
    uint32_t *B0;                       /*!< B0 is first authentication block used only  in AES CCM mode */
    uint32_t DataWidthUnit;             /*!< Data With Unit, this parameter can be value of @ref AES_Data_Width_Unit*/
    uint32_t HeaderWidthUnit;           /*!< Header Width Unit, this parameter can be value of @ref AES_Header_Width_Unit*/
    uint32_t KeyIVConfigSkip;           /*!< AES peripheral Key and IV configuration skip, to config Key and Initialization
                                            Vector only once and to skip configuration for consecutive processings.
                                            This parameter can be a value of @ref AES_Configuration_Skip */
    uint32_t KeyCtxKeySel;              /*!< key source is CPU or OTP KEY1 or OTP KEY2.
                                            This parameter can be a value of @ref AES_Data_Type */
    uint32_t DmaIngLen;                 /*!< intput data lenth. */                                    
    
} AES_EX_ConfigTypeDef;


/**
  * @brief  AES_EX State Structure definition
  */
typedef enum
{
  HAL_AES_EX_STATE_RESET             = 0x00U,  /*!< CRYP not yet initialized or disabled  */
  HAL_AES_EX_STATE_READY             = 0x01U,  /*!< CRYP initialized and ready for use    */
  HAL_AES_EX_STATE_BUSY              = 0x02U  /*!< CRYP BUSY, internal processing is ongoing  */
} HAL_AES_EX_STATETypeDef;


/**
  * @brief  AES_EX handle Structure definition
  */

typedef struct __AES_EX_HandleTypeDef
{
    AES_EX_TypeDef                       *Instance;            /*!< AES Register base address */

    AES_EX_ConfigTypeDef                Init;             /*!< CRYP required parameters */

    FunctionalState                   AutoKeyDerivation;   /*!< Used only in TinyAES to allows to bypass or not key write-up before decryption.
                                                         This parameter can be a value of ENABLE/DISABLE */

    uint32_t                          *pCrypInBuffPtr;  /*!< Pointer to AES processing (encryption, decryption,...) buffer */

    uint32_t                          *pCrypOutBuffPtr; /*!< Pointer to AES processing (encryption, decryption,...) buffer */

    __IO uint16_t                     CrypHeaderCount;   /*!< Counter of header data */

    __IO uint16_t                     CrypInCount;      /*!< Counter of input data */

    __IO uint16_t                     CrypOutCount;     /*!< Counter of output data */

    uint16_t                          Size;           /*!< length of input data in word */

    uint32_t                          Phase;            /*!< AES peripheral phase */

    DMA_HandleTypeDef                 *hdmain;          /*!< AES In DMA handle parameters */

    DMA_HandleTypeDef                 *hdmaout;         /*!< AES Out DMA handle parameters */

    HAL_LockTypeDef                   Lock;             /*!< AES locking object */

    __IO  HAL_AES_EX_STATETypeDef       State;            /*!< AES peripheral state */

    __IO uint32_t                     ErrorCode;        /*!< AES peripheral error code */

    uint32_t                          KeyIVConfig;      /*!< AES peripheral Key and IV configuration flag, used when
                                                           configuration can be skipped */

    uint32_t                          SizesSum;         /*!< Sum of successive payloads lengths (in bytes), stored
                                                           for a single signature computation after several
                                                           messages processing */

} AES_EX_HandleTypeDef;


/**
  * @}
  */


/* Exported constants --------------------------------------------------------*/
/** @defgroup AES_EX_Exported_Constants AES_EX Exported Constants
  * @{
  */

/** @defgroup AES_EX_Error_Definition   AES_EX Error Definition
  * @{
  */
#define HAL_AES_EX_ERROR_NONE              0x00000000U  /*!< No error        */
#define HAL_AES_EX_ERROR_WRITE             0x00000001U  /*!< Write error     */
#define HAL_AES_EX_ERROR_READ              0x00000002U  /*!< Read error      */
#define HAL_AES_EX_ERROR_DMA               0x00000004U  /*!< DMA error       */
#define HAL_AES_EX_ERROR_BUSY              0x00000008U  /*!< Busy flag error */
#define HAL_AES_EX_ERROR_TIMEOUT           0x00000010U  /*!< Timeout error */
#define HAL_AES_EX_ERROR_NOT_SUPPORTED     0x00000020U  /*!< Not supported mode */
#define HAL_AES_EX_ERROR_AUTH_TAG_SEQUENCE 0x00000040U  /*!< Sequence are not respected only for GCM or CCM */
#if (USE_HAL_AES_EX_REGISTER_CALLBACKS == 1)
#define  HAL_AES_EX_ERROR_INVALID_CALLBACK ((uint32_t)0x00000080U)    /*!< Invalid Callback error  */
#endif /* USE_HAL_AES_EX_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup AES_EX_Data_Width_Unit CRYP Data Width Unit
  * @{
  */

#define AES_EX_DATAWIDTHUNIT_WORD   0x00000000U  /*!< By default, size unit is word */
#define AES_EX_DATAWIDTHUNIT_BYTE   0x00000001U  /*!< By default, size unit is word */

/**
  * @}
  */

/** @defgroup AES_EX_Header_Width_Unit CRYP Header Width Unit
  * @{
  */

#define AES_EX_HEADERWIDTHUNIT_WORD   0x00000000U  /*!< By default, header size unit is word */
#define AES_EX_HEADERWIDTHUNIT_BYTE   0x00000001U  /*!< By default, header size unit is byte */

/**
  * @}
  */
  
/** @defgroup AES_EX_Algorithm_Mode AES_EX Algorithm Mode
  * @{
  */
/* AES_EX*/
#define AES_EX_ECB            (0x0U<<AES_EX_MSGCFG_ALG_MODE_Pos) /*!< Electronic codebook chaining algorithm                     */
#define AES_EX_CBC            (0x1U<<AES_EX_MSGCFG_ALG_MODE_Pos) /*!< Cipher block chaining algorithm                            */
#define AES_EX_CTR            (0x2U<<AES_EX_MSGCFG_ALG_MODE_Pos) /*!< Counter mode chaining algorithm                            */
#define AES_EX_CCM            (0x3U<<AES_EX_MSGCFG_ALG_MODE_Pos) /*!< Counter with Cipher Mode                                   */
#define AES_EX_CMAC           (0x4U<<AES_EX_MSGCFG_ALG_MODE_Pos) /*!< Galois counter mode - Galois message authentication code */
#define AES_EX_GCM            (0x5U<<AES_EX_MSGCFG_ALG_MODE_Pos) /*!< Galois counter mode - Galois message authentication code */
                               

/**
  * @}
  */

/** @defgroup AES_EX_Key_Size AES_EX Key Size
  * @{
  */
/* AES_EX*/
#define AES_EX_KEYSIZE_128B           0x00000000U                   /*!< 128-bit long key */
#define AES_EX_KEYSIZE_256B          AES_EX_MSGCFG_KEY_SIZE_1         /*!< 256-bit long key */

/**
  * @}
  */
  
/** @defgroup AES_EX_CTXKEYSEL AES_EX KEY SEL
  * @{
  */
#define AES_EX_KEYSEL_CPU             AES_EX_CTXKEYSEL_KEY_SEL_0         /*!< CPU key */
#define AES_EX_KEYSEL_OPT_KEY1        AES_EX_CTXKEYSEL_KEY_SEL_1         /*!< OPT key1 */
#define AES_EX_KEYSEL_OPT_KEY2        AES_EX_CTXKEYSEL_KEY_SEL_2         /*!< OPT key2 */

/**
  * @}
  */  



/** @defgroup MSGCFG  MSG BEGIN
  * @{
  */
#define AES_EX_BEGIN_DISABLE          0x00000000U                     /*!< BEGIN disable */
#define AES_EX_BEGIN_ENABLE           AES_EX_MSGCFG_MSG_BEGIN            /*!< BEGIN enable */

/**
  * @}
  */ 


/** @defgroup MSGCFG  MSG END
  * @{
  */
#define AES_EX_END_DISABLE          0x00000000U                       /*!< END disable */
#define AES_EX_END_ENABLE           AES_EX_MSGCFG_MSG_END                /*!< END enable */

/**
  * @}
  */ 

/** @defgroup AES_EX_Data_Type AES_EX Data Type
  * @{
  */

#define AES_EX_DATATYPE_32B         0x00000000U  /*!< 32-bit data type (no swapping)        */
#define AES_EX_DATATYPE_16B         AES_EX_CR_DATATYPE_0       /*!< 16-bit data type (half-word swapping) */
#define AES_EX_DATATYPE_8B          AES_EX_CR_DATATYPE_1       /*!< 8-bit data type (byte swapping)       */
#define AES_EX_DATATYPE_1B          AES_EX_CR_DATATYPE         /*!< 1-bit data type (bit swapping)        */


/**
  * @}
  */

/** @defgroup AES_EX_Interrupt  AES_EX Interrupt
  * @{
  */
#define AES_EX_IT_CCFIE     AES_EX_CR_CCFIE /*!< Computation Complete interrupt enable */
#define AES_EX_IT_ERRIE     AES_EX_CR_ERRIE /*!< Error interrupt enable                */
#define AES_EX_IT_WRERR     AES_EX_SR_WRERR  /*!< Write Error           */
#define AES_EX_IT_RDERR     AES_EX_SR_RDERR  /*!< Read Error            */
#define AES_EX_IT_CCF       AES_EX_SR_CCF    /*!< Computation completed */

/**
  * @}
  */

/** @defgroup AES_EX_Flags AES_EX Flags
  * @{
  */

/* status flags */
#define AES_EX_FLAG_BUSY    AES_EX_SR_BUSY   /*!< GCM process suspension forbidden */
#define AES_EX_FLAG_WRERR   AES_EX_SR_WRERR  /*!< Write Error                      */
#define AES_EX_FLAG_RDERR   AES_EX_SR_RDERR  /*!< Read error                       */
#define AES_EX_FLAG_CCF     AES_EX_SR_CCF    /*!< Computation completed            */
/* clearing flags */
#define AES_EX_CCF_CLEAR    AES_EX_CR_CCFC   /*!< Computation Complete Flag Clear */
#define AES_EX_ERR_CLEAR    AES_EX_CR_ERRC   /*!< Error Flag Clear  */


/**
  * @}
  */

/** @defgroup AES_EX_Configuration_Skip AES_EX Key and IV Configuration Skip Mode
  * @{
  */

#define AES_EX_KEYIVCONFIG_ALWAYS        0x00000000U            /*!< Peripheral Key and IV configuration to do systematically */
#define AES_EX_KEYIVCONFIG_ONCE          0x00000001U            /*!< Peripheral Key and IV configuration to do only once      */

/**
  * @}
  */


/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup AES_EX_Exported_Macros AES_EX Exported Macros
  * @{
  */

/** @brief ResetAES_EX handle state
  * @param  __HANDLE__ specifies the AES_EX handle.
  * @retval None
  */

#define __HAL_AES_EX_RESET_HANDLE_STATE(__HANDLE__) ( (__HANDLE__)->State = HAL_AES_EX_STATE_RESET)


/**
  * @brief  Enable/Disable the AES_EX peripheral.
  * @param  __HANDLE__: specifies the AES_EX handle.
  * @retval None
  */
#define __HAL_AES_EX_ENABLE(__HANDLE__)  ((__HANDLE__)->Instance->MSGCFG |=  AES_EX_MSGCFG_AES_EX_GO)
#define __HAL_AES_EX_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->MSGCFG &=  ~AES_EX_MSGCFG_AES_EX_GO)


/**
  * @}
  */
#if defined (AES_EX_CR_ALGOMODE_AES_EX_GCM)|| defined (AES_EX)
/* Include AES_EX HAL Extended module */

#endif /* AES_EX or GCM CCM defined*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup AES_EX_Exported_Functions AES_EX Exported Functions
  * @{
  */

/** @addtogroup AES_EX_Exported_Functions_Group1
  * @{
  */
HAL_StatusTypeDef HAL_AES_EX_Init(AES_EX_HandleTypeDef *haes);
HAL_StatusTypeDef HAL_AES_EX_DeInit(AES_EX_HandleTypeDef *haes);
HAL_StatusTypeDef HAL_AES_EX_SetConfig(AES_EX_HandleTypeDef *haes, AES_EX_ConfigTypeDef *pConf);
HAL_StatusTypeDef HAL_AES_EX_GetConfig(AES_EX_HandleTypeDef *haes, AES_EX_ConfigTypeDef *pConf);

/**
  * @}
  */

/** @addtogroup AES_EX_Exported_Functions_Group2
  * @{
  */

/* encryption/decryption ***********************************/
HAL_StatusTypeDef HAL_AES_EX_Encrypt(AES_EX_HandleTypeDef *haes, uint32_t *Input, uint16_t Size, uint32_t *Output,
                                   uint32_t Timeout);
HAL_StatusTypeDef HAL_AES_EX_Decrypt(AES_EX_HandleTypeDef *haes, uint32_t *Input, uint16_t Size, uint32_t *Output,
                                   uint32_t Timeout);



/**
  * @}
  */


/** @addtogroup AES_EX_Exported_Functions_Group3
  * @{
  */
/* Interrupt Handler functions  **********************************************/
//void HAL_AES_EX_IRQHandler(AES_HandleTypeDef *haes);
HAL_AES_EX_STATETypeDef HAL_AES_GetState(AES_EX_HandleTypeDef *haes);
void HAL_AES_EX_InCpltCallback(AES_EX_HandleTypeDef *haes);
void HAL_AES_EX_OutCpltCallback(AES_EX_HandleTypeDef *haes);
void HAL_AES_EX_ErrorCallback(AES_EX_HandleTypeDef *haes);
uint32_t HAL_AES_EX_GetError(AES_EX_HandleTypeDef *haes);
void HAL_AES_EX_MspInit(void);
void HAL_AES_EX_MspDeInit(void);
/**
  * @}
  */

/**
  * @}
  */

/* Private macros --------------------------------------------------------*/
/** @defgroup AES_EX_Private_Macros   AES_EX Private Macros
  * @{
  */


/**
  * @}
  */


/* Private constants ---------------------------------------------------------*/
/** @defgroup AES_EX_Private_Constants AES_EX Private Constants
  * @{
  */

/**
  * @}
  */
/* Private defines -----------------------------------------------------------*/
/** @defgroup AES_EX_Private_Defines AES_EX Private Defines
  * @{
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup AES_EX_Private_Variables AES_EX Private Variables
  * @{
  */

/**
  * @}
  */
/* Private functions prototypes ----------------------------------------------*/
/** @defgroup AES_EX_Private_Functions_Prototypes AES_EX Private Functions Prototypes
  * @{
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup AES_EX_Private_Functions AES Private Functions
  * @{
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
//#endif /* TinyAES or AES_EX*/

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __UM324xx_HAL_AES_EX_H */

