/**
  *
  * @file    apm32f4xx_dal_cryp.h
  * @author  MCD Application Team
  * @brief   Header file of CRYP DAL module.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_CRYP_H
#define APM32F4xx_DAL_CRYP_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */
#if defined (AES)  || defined (CRYP)
/** @addtogroup CRYP
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup CRYP_Exported_Types CRYP Exported Types
  * @{
  */

/**
  * @brief CRYP Init Structure definition
  */

typedef struct
{
  uint32_t DataType;                   /*!< 32-bit data, 16-bit data, 8-bit data or 1-bit string.
                                        This parameter can be a value of @ref CRYP_Data_Type */
  uint32_t KeySize;                    /*!< Used only in AES mode : 128, 192 or 256 bit key length in CRYP1.
                                        128 or 256 bit key length in TinyAES This parameter can be a value of @ref CRYP_Key_Size */
  uint32_t *pKey;                      /*!< The key used for encryption/decryption */
  uint32_t *pInitVect;                 /*!< The initialization vector used also as initialization
                                         counter in CTR mode */
  uint32_t Algorithm;                  /*!<  DES/ TDES Algorithm ECB/CBC
                                        AES Algorithm ECB/CBC/CTR/GCM or CCM
                                        This parameter can be a value of @ref CRYP_Algorithm_Mode */
  uint32_t *Header;                    /*!< used only in AES GCM and CCM Algorithm for authentication,
                                        GCM : also known as Additional Authentication Data
                                        CCM : named B1 composed of the associated data length and Associated Data. */
  uint32_t HeaderSize;                /*!< The size of header buffer in word  */
  uint32_t *B0;                       /*!< B0 is first authentication block used only  in AES CCM mode */
  uint32_t DataWidthUnit;              /*!< Data With Unit, this parameter can be value of @ref CRYP_Data_Width_Unit*/
  uint32_t HeaderWidthUnit;            /*!< Header Width Unit, this parameter can be value of @ref CRYP_Header_Width_Unit*/
  uint32_t KeyIVConfigSkip;            /*!< CRYP peripheral Key and IV configuration skip, to config Key and Initialization
                                           Vector only once and to skip configuration for consecutive processings.
                                           This parameter can be a value of @ref CRYP_Configuration_Skip */

} CRYP_ConfigTypeDef;


/**
  * @brief  CRYP State Structure definition
  */

typedef enum
{
  DAL_CRYP_STATE_RESET             = 0x00U,  /*!< CRYP not yet initialized or disabled  */
  DAL_CRYP_STATE_READY             = 0x01U,  /*!< CRYP initialized and ready for use    */
  DAL_CRYP_STATE_BUSY              = 0x02U  /*!< CRYP BUSY, internal processing is ongoing  */
} DAL_CRYP_STATETypeDef;


/**
  * @brief  CRYP handle Structure definition
  */

typedef struct __CRYP_HandleTypeDef
{
#if defined (CRYP)
  CRYP_TypeDef                      *Instance;            /*!< CRYP registers base address */
#else /* AES*/
  AES_TypeDef                       *Instance;            /*!< AES Register base address */
#endif /* End AES or CRYP */

  CRYP_ConfigTypeDef                Init;             /*!< CRYP required parameters */

  FunctionalState                   AutoKeyDerivation;   /*!< Used only in TinyAES to allows to bypass or not key write-up before decryption.
                                                         This parameter can be a value of ENABLE/DISABLE */

  uint32_t                          *pCrypInBuffPtr;  /*!< Pointer to CRYP processing (encryption, decryption,...) buffer */

  uint32_t                          *pCrypOutBuffPtr; /*!< Pointer to CRYP processing (encryption, decryption,...) buffer */

  __IO uint16_t                     CrypHeaderCount;   /*!< Counter of header data */

  __IO uint16_t                     CrypInCount;      /*!< Counter of input data */

  __IO uint16_t                     CrypOutCount;     /*!< Counter of output data */

  uint16_t                          Size;           /*!< length of input data in word */

  uint32_t                          Phase;            /*!< CRYP peripheral phase */

  DMA_HandleTypeDef                 *hdmain;          /*!< CRYP In DMA handle parameters */

  DMA_HandleTypeDef                 *hdmaout;         /*!< CRYP Out DMA handle parameters */

  DAL_LockTypeDef                   Lock;             /*!< CRYP locking object */

  __IO  DAL_CRYP_STATETypeDef       State;            /*!< CRYP peripheral state */

  __IO uint32_t                     ErrorCode;        /*!< CRYP peripheral error code */

  uint32_t                          KeyIVConfig;      /*!< CRYP peripheral Key and IV configuration flag, used when
                                                           configuration can be skipped */

  uint32_t                          SizesSum;         /*!< Sum of successive payloads lengths (in bytes), stored
                                                           for a single signature computation after several
                                                           messages processing */

#if (USE_DAL_CRYP_REGISTER_CALLBACKS == 1)
  void (*InCpltCallback)(struct __CRYP_HandleTypeDef *hcryp);      /*!< CRYP Input FIFO transfer completed callback  */
  void (*OutCpltCallback)(struct __CRYP_HandleTypeDef *hcryp);     /*!< CRYP Output FIFO transfer completed callback */
  void (*ErrorCallback)(struct __CRYP_HandleTypeDef *hcryp);       /*!< CRYP Error callback */

  void (* MspInitCallback)(struct __CRYP_HandleTypeDef *hcryp);    /*!< CRYP Msp Init callback  */
  void (* MspDeInitCallback)(struct __CRYP_HandleTypeDef *hcryp);  /*!< CRYP Msp DeInit callback  */

#endif /* (USE_DAL_CRYP_REGISTER_CALLBACKS) */
} CRYP_HandleTypeDef;


/**
  * @}
  */

#if (USE_DAL_CRYP_REGISTER_CALLBACKS == 1)
/** @defgroup DAL_CRYP_Callback_ID_enumeration_definition DAL CRYP Callback ID enumeration definition
  * @brief  DAL CRYP Callback ID enumeration definition
  * @{
  */
typedef enum
{
  DAL_CRYP_INPUT_COMPLETE_CB_ID    = 0x01U,    /*!< CRYP Input FIFO transfer completed callback ID */
  DAL_CRYP_OUTPUT_COMPLETE_CB_ID   = 0x02U,    /*!< CRYP Output FIFO transfer completed callback ID */
  DAL_CRYP_ERROR_CB_ID             = 0x03U,    /*!< CRYP Error callback ID           */

  DAL_CRYP_MSPINIT_CB_ID        = 0x04U,    /*!< CRYP MspInit callback ID         */
  DAL_CRYP_MSPDEINIT_CB_ID      = 0x05U     /*!< CRYP MspDeInit callback ID       */

} DAL_CRYP_CallbackIDTypeDef;
/**
  * @}
  */

/** @defgroup DAL_CRYP_Callback_pointer_definition DAL CRYP Callback pointer definition
  * @brief  DAL CRYP Callback pointer definition
  * @{
  */

typedef  void (*pCRYP_CallbackTypeDef)(CRYP_HandleTypeDef *hcryp);    /*!< pointer to a common CRYP callback function */

/**
  * @}
  */

#endif /* USE_DAL_CRYP_REGISTER_CALLBACKS */

/* Exported constants --------------------------------------------------------*/
/** @defgroup CRYP_Exported_Constants CRYP Exported Constants
  * @{
  */

/** @defgroup CRYP_Error_Definition   CRYP Error Definition
  * @{
  */
#define DAL_CRYP_ERROR_NONE              0x00000000U  /*!< No error        */
#define DAL_CRYP_ERROR_WRITE             0x00000001U  /*!< Write error     */
#define DAL_CRYP_ERROR_READ              0x00000002U  /*!< Read error      */
#define DAL_CRYP_ERROR_DMA               0x00000004U  /*!< DMA error       */
#define DAL_CRYP_ERROR_BUSY              0x00000008U  /*!< Busy flag error */
#define DAL_CRYP_ERROR_TIMEOUT           0x00000010U  /*!< Timeout error */
#define DAL_CRYP_ERROR_NOT_SUPPORTED     0x00000020U  /*!< Not supported mode */
#define DAL_CRYP_ERROR_AUTH_TAG_SEQUENCE 0x00000040U  /*!< Sequence are not respected only for GCM or CCM */
#if (USE_DAL_CRYP_REGISTER_CALLBACKS == 1)
#define  DAL_CRYP_ERROR_INVALID_CALLBACK ((uint32_t)0x00000080U)    /*!< Invalid Callback error  */
#endif /* USE_DAL_CRYP_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup CRYP_Data_Width_Unit CRYP Data Width Unit
  * @{
  */

#define CRYP_DATAWIDTHUNIT_WORD   0x00000000U  /*!< By default, size unit is word */
#define CRYP_DATAWIDTHUNIT_BYTE   0x00000001U  /*!< By default, size unit is word */

/**
  * @}
  */

/** @defgroup CRYP_Header_Width_Unit CRYP Header Width Unit
  * @{
  */

#define CRYP_HEADERWIDTHUNIT_WORD   0x00000000U  /*!< By default, header size unit is word */
#define CRYP_HEADERWIDTHUNIT_BYTE   0x00000001U  /*!< By default, header size unit is byte */

/**
  * @}
  */
  
/** @defgroup CRYP_Algorithm_Mode CRYP Algorithm Mode
  * @{
  */
#if defined(CRYP)

#define CRYP_DES_ECB     CRYP_CTRL_ALGOMSEL_DES_ECB
#define CRYP_DES_CBC     CRYP_CTRL_ALGOMSEL_DES_CBC
#define CRYP_TDES_ECB    CRYP_CTRL_ALGOMSEL_TDES_ECB
#define CRYP_TDES_CBC    CRYP_CTRL_ALGOMSEL_TDES_CBC
#define CRYP_AES_ECB     CRYP_CTRL_ALGOMSEL_AES_ECB
#define CRYP_AES_CBC     CRYP_CTRL_ALGOMSEL_AES_CBC
#define CRYP_AES_CTR     CRYP_CTRL_ALGOMSEL_AES_CTR
#if defined (CRYP_CTRL_ALGOMSEL_AES_GCM)
#define CRYP_AES_GCM     CRYP_CTRL_ALGOMSEL_AES_GCM
#define CRYP_AES_CCM     CRYP_CTRL_ALGOMSEL_AES_CCM
#endif /* GCM CCM defined*/
#else /* AES*/
#define CRYP_AES_ECB            0x00000000U                       /*!< Electronic codebook chaining algorithm                   */
#define CRYP_AES_CBC            AES_CR_CHMOD_0                    /*!< Cipher block chaining algorithm                          */
#define CRYP_AES_CTR            AES_CR_CHMOD_1                    /*!< Counter mode chaining algorithm                          */
#define CRYP_AES_GCM_GMAC       (AES_CR_CHMOD_0 | AES_CR_CHMOD_1) /*!< Galois counter mode - Galois message authentication code */
#define CRYP_AES_CCM            AES_CR_CHMOD_2                    /*!< Counter with Cipher Mode                                 */
#endif /* End AES or CRYP */

/**
  * @}
  */

/** @defgroup CRYP_Key_Size CRYP Key Size
  * @{
  */
#if defined(CRYP)
#define CRYP_KEYSIZE_128B         0x00000000U
#define CRYP_KEYSIZE_192B         CRYP_CTRL_KSIZESEL_0
#define CRYP_KEYSIZE_256B         CRYP_CTRL_KSIZESEL_1
#else /* AES*/
#define CRYP_KEYSIZE_128B         0x00000000U          /*!< 128-bit long key */
#define CRYP_KEYSIZE_256B         AES_CR_KEYSIZE       /*!< 256-bit long key */
#endif /* End AES or CRYP */
/**
  * @}
  */

/** @defgroup CRYP_Data_Type CRYP Data Type
  * @{
  */
#if defined(CRYP)
#define CRYP_DATATYPE_32B         0x00000000U
#define CRYP_DATATYPE_16B         CRYP_CTRL_DTSEL_0
#define CRYP_DATATYPE_8B          CRYP_CTRL_DTSEL_1
#define CRYP_DATATYPE_1B          CRYP_CTRL_DTSEL
#else /* AES*/
#define CRYP_DATATYPE_32B         0x00000000U  /*!< 32-bit data type (no swapping)        */
#define CRYP_DATATYPE_16B         AES_CR_DATATYPE_0       /*!< 16-bit data type (half-word swapping) */
#define CRYP_DATATYPE_8B          AES_CR_DATATYPE_1       /*!< 8-bit data type (byte swapping)       */
#define CRYP_DATATYPE_1B          AES_CR_DATATYPE         /*!< 1-bit data type (bit swapping)        */
#endif /* End AES or CRYP */

/**
  * @}
  */

/** @defgroup CRYP_Interrupt  CRYP Interrupt
  * @{
  */
#if defined (CRYP)
#define CRYP_IT_INI       CRYP_INTMASK_INIMASK   /*!< Input FIFO Interrupt */
#define CRYP_IT_OUTI      CRYP_INTMASK_OUTIMASK  /*!< Output FIFO Interrupt */
#else /* AES*/
#define CRYP_IT_CCFIE     AES_CR_CCFIE /*!< Computation Complete interrupt enable */
#define CRYP_IT_ERRIE     AES_CR_ERRIE /*!< Error interrupt enable                */
#define CRYP_IT_WRERR     AES_SR_WRERR  /*!< Write Error           */
#define CRYP_IT_RDERR     AES_SR_RDERR  /*!< Read Error            */
#define CRYP_IT_CCF       AES_SR_CCF    /*!< Computation completed */
#endif /* End AES or CRYP */

/**
  * @}
  */

/** @defgroup CRYP_Flags CRYP Flags
  * @{
  */
#if defined (CRYP)
/* Flags in the SR register */
#define CRYP_FLAG_IFEM    CRYP_STS_IFEMPT  /*!< Input FIFO is empty */
#define CRYP_FLAG_IFNF    CRYP_STS_IFFULL  /*!< Input FIFO is not Full */
#define CRYP_FLAG_OFNE    CRYP_STS_OFEMPT  /*!< Output FIFO is not empty */
#define CRYP_FLAG_OFFU    CRYP_STS_OFFULL  /*!< Output FIFO is Full */
#define CRYP_FLAG_BUSY    CRYP_STS_BUSY  /*!< The CRYP core is currently processing a block of data 
                                             or a key preparation (for AES decryption). */
/* Flags in the RISR register */
#define CRYP_FLAG_OUTRIS  0x01000002U  /*!< Output FIFO service raw interrupt status */
#define CRYP_FLAG_INRIS   0x01000001U  /*!< Input FIFO service raw interrupt status*/
#else /* AES*/
/* status flags */
#define CRYP_FLAG_BUSY    AES_SR_BUSY   /*!< GCM process suspension forbidden */
#define CRYP_FLAG_WRERR   AES_SR_WRERR  /*!< Write Error                      */
#define CRYP_FLAG_RDERR   AES_SR_RDERR  /*!< Read error                       */
#define CRYP_FLAG_CCF     AES_SR_CCF    /*!< Computation completed            */
/* clearing flags */
#define CRYP_CCF_CLEAR    AES_CR_CCFC   /*!< Computation Complete Flag Clear */
#define CRYP_ERR_CLEAR    AES_CR_ERRC   /*!< Error Flag Clear  */
#endif /* End AES or CRYP  */

/**
  * @}
  */

/** @defgroup CRYP_Configuration_Skip CRYP Key and IV Configuration Skip Mode
  * @{
  */

#define CRYP_KEYIVCONFIG_ALWAYS        0x00000000U            /*!< Peripheral Key and IV configuration to do systematically */
#define CRYP_KEYIVCONFIG_ONCE          0x00000001U            /*!< Peripheral Key and IV configuration to do only once      */

/**
  * @}
  */


/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup CRYP_Exported_Macros CRYP Exported Macros
  * @{
  */

/** @brief Reset CRYP handle state
  * @param  __HANDLE__ specifies the CRYP handle.
  * @retval None
  */
#if (USE_DAL_CRYP_REGISTER_CALLBACKS == 1)
#define __DAL_CRYP_RESET_HANDLE_STATE(__HANDLE__) do{\
                                                      (__HANDLE__)->State = DAL_CRYP_STATE_RESET;\
                                                      (__HANDLE__)->MspInitCallback = NULL;\
                                                      (__HANDLE__)->MspDeInitCallback = NULL;\
                                                     }while(0)
#else
#define __DAL_CRYP_RESET_HANDLE_STATE(__HANDLE__) ( (__HANDLE__)->State = DAL_CRYP_STATE_RESET)
#endif /* USE_DAL_CRYP_REGISTER_CALLBACKS */

/**
  * @brief  Enable/Disable the CRYP peripheral.
  * @param  __HANDLE__: specifies the CRYP handle.
  * @retval None
  */
#if defined(CRYP)
#define __DAL_CRYP_ENABLE(__HANDLE__)  ((__HANDLE__)->Instance->CTRL |=  CRYP_CTRL_CRYPEN)
#define __DAL_CRYP_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL &=  ~CRYP_CTRL_CRYPEN)
#else /* AES*/
#define __DAL_CRYP_ENABLE(__HANDLE__)  ((__HANDLE__)->Instance->CTRL |=  AES_CR_EN)
#define __DAL_CRYP_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL &=  ~AES_CR_EN)
#endif /* End AES or CRYP  */

/** @brief  Check whether the specified CRYP status flag is set or not.
  * @param  __FLAG__: specifies the flag to check.
  *         This parameter can be one of the following values for TinyAES:
  *            @arg @ref CRYP_FLAG_BUSY GCM process suspension forbidden
  *            @arg @ref CRYP_IT_WRERR Write Error
  *            @arg @ref CRYP_IT_RDERR Read Error
  *            @arg @ref CRYP_IT_CCF Computation Complete
  *         This parameter can be one of the following values for CRYP:
  *            @arg CRYP_FLAG_BUSY: The CRYP core is currently processing a block of data
  *                                 or a key preparation (for AES decryption).
  *            @arg CRYP_FLAG_IFEM: Input FIFO is empty
  *            @arg CRYP_FLAG_IFNF: Input FIFO is not full
  *            @arg CRYP_FLAG_INRIS: Input FIFO service raw interrupt is pending
  *            @arg CRYP_FLAG_OFNE: Output FIFO is not empty
  *            @arg CRYP_FLAG_OFFU: Output FIFO is full
  *            @arg CRYP_FLAG_OUTRIS: Input FIFO service raw interrupt is pending
  * @retval The state of __FLAG__ (TRUE or FALSE).
  */
#define CRYP_FLAG_MASK  0x0000001FU
#if defined(CRYP)
#define __DAL_CRYP_GET_FLAG(__HANDLE__, __FLAG__) ((((uint8_t)((__FLAG__) >> 24)) == 0x01U)?((((__HANDLE__)->Instance->INTSTS) & ((__FLAG__) & CRYP_FLAG_MASK)) == ((__FLAG__) & CRYP_FLAG_MASK)): \
                                                   ((((__HANDLE__)->Instance->INTSTS) & ((__FLAG__) & CRYP_FLAG_MASK)) == ((__FLAG__) & CRYP_FLAG_MASK)))
#else /* AES*/
#define __DAL_CRYP_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->STS & (__FLAG__)) == (__FLAG__))
#endif /* End AES or CRYP */

/** @brief  Clear the CRYP pending status flag.
  * @param  __FLAG__: specifies the flag to clear.
  *         This parameter can be one of the following values:
  *            @arg @ref CRYP_ERR_CLEAR Read (RDERR) or Write Error (WRERR) Flag Clear
  *            @arg @ref CRYP_CCF_CLEAR Computation Complete Flag (CCF) Clear
  * @param  __HANDLE__: specifies the CRYP handle.
  * @retval None
  */

#if defined(AES)
#define __DAL_CRYP_CLEAR_FLAG(__HANDLE__, __FLAG__) SET_BIT((__HANDLE__)->Instance->CTRL, (__FLAG__))


/** @brief  Check whether the specified CRYP interrupt source is enabled or not.
  * @param __INTERRUPT__: CRYP interrupt source to check
  *         This parameter can be one of the following values for TinyAES:
  *            @arg @ref CRYP_IT_ERRIE Error interrupt (used for RDERR and WRERR)
  *            @arg @ref CRYP_IT_CCFIE Computation Complete interrupt
  * @param  __HANDLE__: specifies the CRYP handle.
  * @retval State of interruption (TRUE or FALSE).
  */

#define __DAL_CRYP_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CTRL\
                                                              & (__INTERRUPT__)) == (__INTERRUPT__))

#endif /* AES */

/** @brief  Check whether the specified CRYP interrupt is set or not.
  * @param  __INTERRUPT__: specifies the interrupt to check.
  *         This parameter can be one of the following values for TinyAES:
  *            @arg @ref CRYP_IT_WRERR Write Error
  *            @arg @ref CRYP_IT_RDERR Read Error
  *            @arg @ref CRYP_IT_CCF  Computation Complete
  *         This parameter can be one of the following values for CRYP:
  *            @arg CRYP_IT_INI: Input FIFO service masked interrupt status
  *            @arg CRYP_IT_OUTI: Output FIFO service masked interrupt status
  * @param  __HANDLE__: specifies the CRYP handle.
  * @retval The state of __INTERRUPT__ (TRUE or FALSE).
  */
#if defined(CRYP)
#define __DAL_CRYP_GET_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->MINTSTS\
                                                       & (__INTERRUPT__)) == (__INTERRUPT__))
#else /* AES*/
#define __DAL_CRYP_GET_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->STS & (__INTERRUPT__)) == (__INTERRUPT__))
#endif /* End AES or CRYP */

/**
  * @brief  Enable the CRYP interrupt.
  * @param  __INTERRUPT__: CRYP Interrupt.
  *         This parameter can be one of the following values for TinyAES:
  *            @arg @ref CRYP_IT_ERRIE Error interrupt (used for RDERR and WRERR)
  *            @arg @ref CRYP_IT_CCFIE Computation Complete interrupt
  *         This parameter can be one of the following values for CRYP:
  *            @ CRYP_IT_INI : Input FIFO service interrupt mask.
  *            @ CRYP_IT_OUTI : Output FIFO service interrupt mask.CRYP interrupt.
  * @param  __HANDLE__: specifies the CRYP handle.
  * @retval None
  */
#if defined(CRYP)
#define __DAL_CRYP_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->INTMASK) |= (__INTERRUPT__))
#else /* AES*/
#define __DAL_CRYP_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CTRL) |= (__INTERRUPT__))
#endif /* End AES or CRYP */

/**
  * @brief  Disable the CRYP interrupt.
  * @param  __INTERRUPT__: CRYP Interrupt.
  *         This parameter can be one of the following values for TinyAES:
  *            @arg @ref CRYP_IT_ERRIE Error interrupt (used for RDERR and WRERR)
  *            @arg @ref CRYP_IT_CCFIE Computation Complete interrupt
  *         This parameter can be one of the following values for CRYP:
  *            @ CRYP_IT_INI : Input FIFO service interrupt mask.
  *            @ CRYP_IT_OUTI : Output FIFO service interrupt mask.CRYP interrupt.
  * @param  __HANDLE__: specifies the CRYP handle.
  * @retval None
  */
#if defined(CRYP)
#define __DAL_CRYP_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->INTMASK) &= ~(__INTERRUPT__))
#else /* AES*/
#define __DAL_CRYP_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CTRL) &= ~(__INTERRUPT__))
#endif /* End AES or CRYP */

/**
  * @}
  */
#if defined (CRYP_CTRL_ALGOMSEL_AES_GCM)|| defined (AES)
/* Include CRYP DAL Extended module */
#include "apm32f4xx_dal_cryp_ex.h"
#endif /* AES or GCM CCM defined*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup CRYP_Exported_Functions CRYP Exported Functions
  * @{
  */

/** @addtogroup CRYP_Exported_Functions_Group1
  * @{
  */
DAL_StatusTypeDef DAL_CRYP_Init(CRYP_HandleTypeDef *hcryp);
DAL_StatusTypeDef DAL_CRYP_DeInit(CRYP_HandleTypeDef *hcryp);
void DAL_CRYP_MspInit(CRYP_HandleTypeDef *hcryp);
void DAL_CRYP_MspDeInit(CRYP_HandleTypeDef *hcryp);
DAL_StatusTypeDef DAL_CRYP_SetConfig(CRYP_HandleTypeDef *hcryp, CRYP_ConfigTypeDef *pConf);
DAL_StatusTypeDef DAL_CRYP_GetConfig(CRYP_HandleTypeDef *hcryp, CRYP_ConfigTypeDef *pConf);
#if (USE_DAL_CRYP_REGISTER_CALLBACKS == 1)
DAL_StatusTypeDef DAL_CRYP_RegisterCallback(CRYP_HandleTypeDef *hcryp, DAL_CRYP_CallbackIDTypeDef CallbackID,
                                            pCRYP_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_CRYP_UnRegisterCallback(CRYP_HandleTypeDef *hcryp, DAL_CRYP_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_CRYP_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup CRYP_Exported_Functions_Group2
  * @{
  */

/* encryption/decryption ***********************************/
DAL_StatusTypeDef DAL_CRYP_Encrypt(CRYP_HandleTypeDef *hcryp, uint32_t *Input, uint16_t Size, uint32_t *Output,
                                   uint32_t Timeout);
DAL_StatusTypeDef DAL_CRYP_Decrypt(CRYP_HandleTypeDef *hcryp, uint32_t *Input, uint16_t Size, uint32_t *Output,
                                   uint32_t Timeout);
DAL_StatusTypeDef DAL_CRYP_Encrypt_IT(CRYP_HandleTypeDef *hcryp, uint32_t *Input, uint16_t Size, uint32_t *Output);
DAL_StatusTypeDef DAL_CRYP_Decrypt_IT(CRYP_HandleTypeDef *hcryp, uint32_t *Input, uint16_t Size, uint32_t *Output);
DAL_StatusTypeDef DAL_CRYP_Encrypt_DMA(CRYP_HandleTypeDef *hcryp, uint32_t *Input, uint16_t Size, uint32_t *Output);
DAL_StatusTypeDef DAL_CRYP_Decrypt_DMA(CRYP_HandleTypeDef *hcryp, uint32_t *Input, uint16_t Size, uint32_t *Output);

/**
  * @}
  */


/** @addtogroup CRYP_Exported_Functions_Group3
  * @{
  */
/* Interrupt Handler functions  **********************************************/
void DAL_CRYP_IRQHandler(CRYP_HandleTypeDef *hcryp);
DAL_CRYP_STATETypeDef DAL_CRYP_GetState(CRYP_HandleTypeDef *hcryp);
void DAL_CRYP_InCpltCallback(CRYP_HandleTypeDef *hcryp);
void DAL_CRYP_OutCpltCallback(CRYP_HandleTypeDef *hcryp);
void DAL_CRYP_ErrorCallback(CRYP_HandleTypeDef *hcryp);
uint32_t DAL_CRYP_GetError(CRYP_HandleTypeDef *hcryp);

/**
  * @}
  */

/**
  * @}
  */

/* Private macros --------------------------------------------------------*/
/** @defgroup CRYP_Private_Macros   CRYP Private Macros
  * @{
  */

/** @defgroup CRYP_IS_CRYP_Definitions CRYP Private macros to check input parameters
  * @{
  */
#if defined(CRYP)
#if defined (CRYP_CTRL_ALGOMSEL_AES_GCM)
#define IS_CRYP_ALGORITHM(ALGORITHM) (((ALGORITHM) == CRYP_DES_ECB)   || \
                                      ((ALGORITHM)  == CRYP_DES_CBC)   || \
                                      ((ALGORITHM)  == CRYP_TDES_ECB)  || \
                                      ((ALGORITHM)  == CRYP_TDES_CBC)  || \
                                      ((ALGORITHM)  == CRYP_AES_ECB)   || \
                                      ((ALGORITHM)  == CRYP_AES_CBC)   || \
                                      ((ALGORITHM)  == CRYP_AES_CTR)   || \
                                      ((ALGORITHM)  == CRYP_AES_GCM)   || \
                                      ((ALGORITHM)  == CRYP_AES_CCM))
#else /*NO GCM CCM */
#define IS_CRYP_ALGORITHM(ALGORITHM) (((ALGORITHM) == CRYP_DES_ECB)   || \
                                      ((ALGORITHM)  == CRYP_DES_CBC)   || \
                                      ((ALGORITHM)  == CRYP_TDES_ECB)  || \
                                      ((ALGORITHM)  == CRYP_TDES_CBC)  || \
                                      ((ALGORITHM)  == CRYP_AES_ECB)   || \
                                      ((ALGORITHM)  == CRYP_AES_CBC)   || \
                                      ((ALGORITHM)  == CRYP_AES_CTR))
#endif /* GCM CCM defined*/
#define IS_CRYP_KEYSIZE(KEYSIZE)(((KEYSIZE) == CRYP_KEYSIZE_128B)   || \
                                 ((KEYSIZE) == CRYP_KEYSIZE_192B)   || \
                                 ((KEYSIZE) == CRYP_KEYSIZE_256B))
#else /* AES*/
#define IS_CRYP_ALGORITHM(ALGORITHM) (((ALGORITHM) == CRYP_AES_ECB)   || \
                                      ((ALGORITHM)  == CRYP_AES_CBC)   || \
                                      ((ALGORITHM)  == CRYP_AES_CTR)  || \
                                      ((ALGORITHM)  == CRYP_AES_GCM_GMAC)|| \
                                      ((ALGORITHM)  == CRYP_AES_CCM))


#define IS_CRYP_KEYSIZE(KEYSIZE)(((KEYSIZE) == CRYP_KEYSIZE_128B)   || \
                                 ((KEYSIZE) == CRYP_KEYSIZE_256B))
#endif /* End AES or CRYP */

#define IS_CRYP_DATATYPE(DATATYPE)(((DATATYPE) == CRYP_DATATYPE_32B)   || \
                                   ((DATATYPE) == CRYP_DATATYPE_16B) || \
                                   ((DATATYPE) == CRYP_DATATYPE_8B) || \
                                   ((DATATYPE) == CRYP_DATATYPE_1B))

#define IS_CRYP_INIT(CONFIG)(((CONFIG) == CRYP_KEYIVCONFIG_ALWAYS) || \
                             ((CONFIG) == CRYP_KEYIVCONFIG_ONCE))
/**
  * @}
  */

/**
  * @}
  */


/* Private constants ---------------------------------------------------------*/
/** @defgroup CRYP_Private_Constants CRYP Private Constants
  * @{
  */

/**
  * @}
  */
/* Private defines -----------------------------------------------------------*/
/** @defgroup CRYP_Private_Defines CRYP Private Defines
  * @{
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup CRYP_Private_Variables CRYP Private Variables
  * @{
  */

/**
  * @}
  */
/* Private functions prototypes ----------------------------------------------*/
/** @defgroup CRYP_Private_Functions_Prototypes CRYP Private Functions Prototypes
  * @{
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup CRYP_Private_Functions CRYP Private Functions
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
#endif /* TinyAES or CRYP*/

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_CRYP_H */

