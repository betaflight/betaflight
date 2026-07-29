/**
  ******************************************************************************
  * @file     um324xF_hal_cryp.c
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

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324XX_HAL_Driver
  * @{
  */

/** @defgroup AES_EX
  * @brief AES_EX HAL module driver.
  * @{
  */

#ifdef HAL_AES_EX_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup AES_EX_Private_Defines
  * @{
  */
#define AES_EX_TIMEOUT_KEYPREPARATION      82U         /*The latency of key preparation operation is 82 clock cycles.*/
#define AES_EX_TIMEOUT_GCMCCMINITPHASE     299U        /*  The latency of  GCM/CCM init phase to prepare hash subkey is 299 clock cycles.*/
#define AES_EX_TIMEOUT_GCMCCMHEADERPHASE   290U        /*  The latency of  GCM/CCM header phase is 290 clock cycles.*/

#define AES_EX_PHASE_READY                0x00000001U /*!< AES_EX peripheral is ready for initialization. */
#define AES_EX_PHASE_PROCESS              0x00000002U /*!< AES_EX peripheral is in processing phase */

#define AES_EX_OPERATINGMODE_ENCRYPT                   AES_EX_MSGCFG_DIR     /*!< Encryption mode  */
#define AES_EX_OPERATINGMODE_DECRYPT                   0x00000000U          /*!< Decryption    */
#define AES_EX_OPERATINGMODE_KEYDERIVATION_DECRYPT     AES_EX_CR_MODE        /*!< Key derivation and decryption only used when performing ECB and CBC decryptions (Mode 4) */
#define AES_EX_PHASE_INIT                              0x00000000U          /*!< GCM/GMAC (or CCM) init phase */
#define AES_EX_PHASE_HEADER                            AES_EX_CR_GCMPH_0     /*!< GCM/GMAC or CCM header phase */
#define AES_EX_PHASE_PAYLOAD                           AES_EX_CR_GCMPH_1     /*!< GCM(/CCM) payload phase      */
#define AES_EX_PHASE_FINAL                             AES_EX_CR_GCMPH       /*!< GCM/GMAC or CCM  final phase */

/*  CTR1 information to use in CCM algorithm */
#define AES_EX_CCM_CTR1_0                  0x07FFFFFFU
#define AES_EX_CCM_CTR1_1                  0xFFFFFF00U
#define AES_EX_CCM_CTR1_2                  0x00000001U

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @addtogroup AES_EX_Private_Macros
  * @{
  */

#define AES_EX_SET_PHASE(__HANDLE__, __PHASE__)  do{(__HANDLE__)->Instance->CR &= (uint32_t)(~AES_EX_CR_GCMPH);\
                                                        (__HANDLE__)->Instance->CR |= (uint32_t)(__PHASE__);\
                                                       }while(0)

/**
  * @}
  */

/* Private struct -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup AES_EX_Private_Functions_prototypes
  * @{
  */

static void AES_EX_SetKey(AES_EX_HandleTypeDef *haes, uint32_t KeySize);
#if defined (AES_EX_CR_ALGOMODE_AES_EX_GCM)|| defined (AES_EX)

static HAL_StatusTypeDef AES_EX_AES_EXCCM_Process(AES_EX_HandleTypeDef *haes, uint32_t Timeout);

#endif /* AES_EX or GCM CCM defined*/
static void AES_EX_AES_EX_ProcessData(AES_EX_HandleTypeDef *hcrypt, uint32_t Timeout);
static HAL_StatusTypeDef AES_EX_AES_EX_Encrypt(AES_EX_HandleTypeDef *haes, uint32_t Timeout);
static HAL_StatusTypeDef AES_EX_AES_EX_Decrypt(AES_EX_HandleTypeDef *haes, uint32_t Timeout);

static HAL_StatusTypeDef AES_EX_WaitOnCCFlag(AES_EX_HandleTypeDef *haes, uint32_t Timeout);


/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup AES_EX_Exported_Functions AES_EX Exported Functions
  * @{
  */


/** @defgroup AES_EX_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions.
  *
@verbatim
  ========================================================================================
     ##### Initialization, de-initialization and Set and Get configuration functions #####
  ========================================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the AES_EX
      (+) DeInitialize the AES_EX
      (+) Initialize the AES_EX MSP
      (+) DeInitialize the AES_EX MSP
      (+) configure AES_EX (HAL_AES_EX_SetConfig) with the specified parameters in the AES_EX_ConfigTypeDef
          Parameters which are configured in This section are :
          (+) Key size
          (+) Data Type : 32,16, 8 or 1bit
          (+) AlgoMode :
              - for AES_EX1 IP :
                 ECB and CBC in DES/TDES Standard
                 ECB,CBC,CTR,GCM/GMAC and CCM in AES_EX Standard.
              - for Tiny AES_EX 2 IP, only ECB,CBC,CTR,GCM/GMAC and CCM in AES_EX Standard are supported.
      (+) Get AES_EX configuration (HAL_AES_EX_GetConfig) from the specified parameters in the AES_EX_HandleTypeDef


@endverbatim
  * @{
  */


/**
  * @brief  Initializes the AES_EX according to the specified
  *         parameters in the AES_EX_ConfigTypeDef and creates the associated handle.
  * @param  AES_EX: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AES_EX_Init(AES_EX_HandleTypeDef *haes)
{
    HAL_AES_EX_MspInit();
    
    /* Check the AES_EX handle allocation */
    if (haes == NULL)
    {
        return HAL_ERROR;
    }


    if (haes->State == HAL_AES_EX_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        haes->Lock = HAL_UNLOCKED;

    }


    /* Set the key size(This bit field is don't care in the DES or TDES modes) data type and Algorithm */

    MODIFY_REG(haes->Instance->MSGCFG, AES_EX_MSGCFG_KEY_SIZE | AES_EX_MSGCFG_ALG_MODE,
                                        haes->Init.KeySize | haes->Init.Algorithm);    
    
    /* Set the key source*/
    MODIFY_REG(haes->Instance->CTXKEYSEL, AES_EX_CTXKEYSEL_KEY_SEL, haes->Init.KeyCtxKeySel);
    
    haes->Instance->DMAINGLEN = haes->Init.DmaIngLen;
    haes->Instance->MSGAADBYTES = haes->Init.DmaIngLen;
    
    MODIFY_REG(haes->Instance->INGDBCFG, AES_EX_INGDBCFG_DMA_EN, 0);
    MODIFY_REG(haes->Instance->INGDBCFG, AES_EX_INGDBCFG_DST_MSIZE, AES_EX_INGDBCFG_DST_MSIZE_0);
    MODIFY_REG(haes->Instance->INGDBCFG, AES_EX_INGDBCFG_DST_TR_WIDTH, AES_EX_INGDBCFG_DST_TR_WIDTH_1);
    
    
    /* Reset Error Code field */
    haes->ErrorCode = HAL_AES_EX_ERROR_NONE;

    /* Change the AES_EX state */
    haes->State = HAL_AES_EX_STATE_READY;

    /* Set the default AES_EX phase */
    haes->Phase = AES_EX_PHASE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  De-Initializes the AES_EX peripheral.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AES_EX_DeInit(AES_EX_HandleTypeDef *haes)
{
    /* Check the AES_EX handle allocation */
    if (haes == NULL)
    {
        return HAL_ERROR;
    }

    /* Set the default AES_EX phase */
    haes->Phase = AES_EX_PHASE_READY;

    /* Reset CrypInCount and CrypOutCount */
    haes->CrypInCount = 0;
    haes->CrypOutCount = 0;
    haes->CrypHeaderCount = 0;

    /* Disable the AES_EX peripheral clock */
    __HAL_AES_EX_DISABLE(haes);

    /* DeInit the low level hardware: CLOCK, NVIC.*/
    HAL_AES_EX_MspDeInit();


    /* Change the AES_EX state */
    haes->State = HAL_AES_EX_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(haes);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Configure the AES_EX according to the specified
  *         parameters in the AES_EX_ConfigTypeDef
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure
  * @param  pConf: pointer to a AES_EX_ConfigTypeDef structure that contains
  *         the configuration information for AES_EX module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AES_EX_SetConfig(AES_EX_HandleTypeDef *haes, AES_EX_ConfigTypeDef *pConf)
{
    /* Check the AES_EX handle allocation */
    if ((haes == NULL) || (pConf == NULL))
    {
        return HAL_ERROR;
    }

    if (haes->State == HAL_AES_EX_STATE_READY)
    {
        /* Change the AES_EX state */
        haes->State = HAL_AES_EX_STATE_BUSY;

        /* Process locked */
        __HAL_LOCK(haes);

        /* Set  AES_EX parameters  */
        haes->Init.DataType   = pConf->DataType;
        haes->Init.pKey       = pConf->pKey;
        haes->Init.Algorithm  = pConf->Algorithm;
        haes->Init.KeySize    = pConf->KeySize;
        haes->Init.pInitVect  = pConf->pInitVect;
        haes->Init.Header     = pConf->Header;
        haes->Init.HeaderSize = pConf->HeaderSize;
        haes->Init.B0         = pConf->B0;
        haes->Init.DataWidthUnit = pConf->DataWidthUnit;
        haes->Init.KeyIVConfigSkip = pConf->KeyIVConfigSkip;
        haes->Init.HeaderWidthUnit = pConf->HeaderWidthUnit;
        haes->Init.KeyCtxKeySel = pConf->KeyCtxKeySel;
        haes->Init.DmaIngLen = pConf->DmaIngLen;
        

        MODIFY_REG(haes->Instance->MSGCFG, AES_EX_MSGCFG_KEY_SIZE |AES_EX_MSGCFG_ALG_MODE,
                                            haes->Init.KeySize | haes->Init.Algorithm);    
        
        /* Set the key source*/
        MODIFY_REG(haes->Instance->CTXKEYSEL, AES_EX_CTXKEYSEL_KEY_SEL, haes->Init.KeyCtxKeySel);
        
        haes->Instance->DMAINGLEN = haes->Init.DmaIngLen;
        haes->Instance->MSGAADBYTES = haes->Init.DmaIngLen;
        
        MODIFY_REG(haes->Instance->INGDBCFG, AES_EX_INGDBCFG_DMA_EN, 0);
        MODIFY_REG(haes->Instance->INGDBCFG, AES_EX_INGDBCFG_DST_MSIZE, AES_EX_INGDBCFG_DST_MSIZE_0);
        MODIFY_REG(haes->Instance->INGDBCFG, AES_EX_INGDBCFG_DST_TR_WIDTH, AES_EX_INGDBCFG_DST_TR_WIDTH_1);


        /* Process Unlocked */
        __HAL_UNLOCK(haes);

        /* Reset Error Code field */
        haes->ErrorCode = HAL_AES_EX_ERROR_NONE;

        /* Change the AES_EX state */
        haes->State = HAL_AES_EX_STATE_READY;

        /* Set the default AES_EX phase */
        haes->Phase = AES_EX_PHASE_READY;

        /* Return function status */
        return HAL_OK;
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(haes);

        /* Busy error code field */
        haes->ErrorCode |= HAL_AES_EX_ERROR_BUSY;
        return HAL_ERROR;
    }
}

/**
  * @brief  Get AES_EX Configuration parameters in associated handle.
  * @param  pConf: pointer to a AES_EX_ConfigTypeDef structure
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AES_EX_GetConfig(AES_EX_HandleTypeDef *haes, AES_EX_ConfigTypeDef *pConf)
{
    /* Check the AES_EX handle allocation */
    if ((haes == NULL) || (pConf == NULL))
    {
        return HAL_ERROR;
    }

    if (haes->State == HAL_AES_EX_STATE_READY)
    {
        /* Change the AES_EX state */
        haes->State = HAL_AES_EX_STATE_BUSY;

        /* Process locked */
        __HAL_LOCK(haes);

        /* Get AES_EX parameters  */
        pConf->DataType        = haes->Init.DataType;
        pConf->pKey            = haes->Init.pKey;
        pConf->Algorithm       = haes->Init.Algorithm;
        pConf->KeySize         = haes->Init.KeySize ;
        pConf->pInitVect       = haes->Init.pInitVect;
        pConf->Header          = haes->Init.Header ;
        pConf->HeaderSize      = haes->Init.HeaderSize;
        pConf->B0              = haes->Init.B0;
        pConf->DataWidthUnit   = haes->Init.DataWidthUnit;
        pConf->KeyIVConfigSkip = haes->Init.KeyIVConfigSkip;
        pConf->HeaderWidthUnit = haes->Init.HeaderWidthUnit;
        pConf->KeyCtxKeySel = haes->Init.KeyCtxKeySel;
        pConf->DmaIngLen = haes->Init.DmaIngLen;
        

        /* Process Unlocked */
        __HAL_UNLOCK(haes);

        /* Change the AES_EX state */
        haes->State = HAL_AES_EX_STATE_READY;

        /* Return function status */
        return HAL_OK;
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(haes);

        /* Busy error code field */
        haes->ErrorCode |= HAL_AES_EX_ERROR_BUSY;
        return HAL_ERROR;
    }
}

/**
  * @}
  */

/** @defgroup AES_EX_Exported_Functions_Group2  Encrypt Decrypt functions
  *  @brief   processing functions.
  *
@verbatim
  ==============================================================================
                      ##### Encrypt Decrypt  functions #####
  ==============================================================================
    [..]  This section provides API allowing to Encrypt/Decrypt Data following
          Standard DES/TDES or AES_EX, and Algorithm configured by the user:
      (+) Standard DES/TDES only supported by AES_EX IP, below list of Algorithm supported :
           - Electronic Code Book(ECB)
           - Cipher Block Chaining (CBC)
      (+) Standard AES_EX  supported by AES_EX IP & TinyAES_EX, list of Algorithm supported:
           - Electronic Code Book(ECB)
           - Cipher Block Chaining (CBC)
           - Counter mode (CTR)
           - Cipher Block Chaining (CBC)
           - Counter mode (CTR)
           - Galois/counter mode (GCM)
           - Counter with Cipher Block Chaining-Message(CCM)
    [..]  Three processing functions are available:
      (+) Polling mode : HAL_AES_EX_Encrypt & HAL_AES_EX_Decrypt


@endverbatim
  * @{
  */


/**
  * @brief  Encryption mode.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module
  * @param  Input: Pointer to the input buffer (plaintext)
  * @param  Size: Length of the plaintext buffer in word.
  * @param  Output: Pointer to the output buffer(ciphertext)
  * @param  Timeout: Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AES_EX_Encrypt(AES_EX_HandleTypeDef *haes, uint32_t *Input, uint16_t Size, uint32_t *Output,
                                   uint32_t Timeout)
{
    uint32_t algo;
    HAL_StatusTypeDef status;

    if (haes->State == HAL_AES_EX_STATE_READY)
    {
        /* Change state Busy */
        haes->State = HAL_AES_EX_STATE_BUSY;

        /* Process locked */
        __HAL_LOCK(haes);

        /*  Reset CrypInCount, CrypOutCount and Initialize pCrypInBuffPtr and pCrypOutBuffPtr parameters*/
        haes->CrypInCount = 0U;
        haes->CrypOutCount = 0U;
        haes->pCrypInBuffPtr = Input;
        haes->pCrypOutBuffPtr = Output;

        /*  Calculate Size parameter in Byte*/
        if (haes->Init.DataWidthUnit == AES_EX_DATAWIDTHUNIT_WORD)
        {
            haes->Size = Size * 4U;
        }
        else
        {
            haes->Size = Size;
        }

        /* Set the operating mode*/
        MODIFY_REG(haes->Instance->MSGCFG, AES_EX_MSGCFG_DIR, AES_EX_OPERATINGMODE_ENCRYPT);
        
        /* Set the operating mode*/
        MODIFY_REG(haes->Instance->MSGCFG, AES_EX_MSGCFG_MSG_BEGIN, AES_EX_BEGIN_ENABLE);
        
        /* Set the operating mode*/
        MODIFY_REG(haes->Instance->MSGCFG, AES_EX_MSGCFG_MSG_END, AES_EX_END_ENABLE);
        
        

        /* algo get algorithm selected */
        algo = haes->Instance->MSGCFG & AES_EX_MSGCFG_ALG_MODE_Msk;
        

        switch (algo)
        {
            case AES_EX_ECB:
            case AES_EX_CBC:
            case AES_EX_CTR:

                /* AES_EX encryption */
                status = AES_EX_AES_EX_Encrypt(haes, Timeout);
            break;

            case AES_EX_GCM:
            case AES_EX_CMAC:

                /* AES_EX GCM encryption */

                break;

            case AES_EX_CCM:

                /* AES_EX CCM encryption */
                status = AES_EX_AES_EXCCM_Process(haes, Timeout);
                break;

            default:
                haes->ErrorCode |= HAL_AES_EX_ERROR_NOT_SUPPORTED;
                /* Change the AES_EX peripheral state */
                haes->State = HAL_AES_EX_STATE_READY;
                /* Process unlocked */
                __HAL_UNLOCK(haes);
                return HAL_ERROR;
        }


        if (status == HAL_OK)
        {
            /* Change the AES_EX peripheral state */
            haes->State = HAL_AES_EX_STATE_READY;

            /* Process unlocked */
            __HAL_UNLOCK(haes);
        }
    }
    else
    {
        /* Busy error code field */
        haes->ErrorCode |= HAL_AES_EX_ERROR_BUSY;
        return HAL_ERROR;
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Decryption mode.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module
  * @param  Input: Pointer to the input buffer (ciphertext )
  * @param  Size: Length of the plaintext buffer in word.
  * @param  Output: Pointer to the output buffer(plaintext)
  * @param  Timeout: Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_AES_EX_Decrypt(AES_EX_HandleTypeDef *haes, uint32_t *Input, uint16_t Size, uint32_t *Output,
                                   uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t algo;

    if (haes->State == HAL_AES_EX_STATE_READY)
    {
        /* Change state Busy */
        haes->State = HAL_AES_EX_STATE_BUSY;

        /* Process locked */
        __HAL_LOCK(haes);

        /*  Reset CrypInCount, CrypOutCount and Initialize pCrypInBuffPtr and pCrypOutBuffPtr  parameters*/
        haes->CrypInCount = 0U;
        haes->CrypOutCount = 0U;
        haes->pCrypInBuffPtr = Input;
        haes->pCrypOutBuffPtr = Output;

        /*  Calculate Size parameter in Byte*/
        if (haes->Init.DataWidthUnit == AES_EX_DATAWIDTHUNIT_WORD)
        {
            haes->Size = Size * 4U;
        }
        else
        {
            haes->Size = Size;
        }


        /* Set the operating mode*/
        MODIFY_REG(haes->Instance->MSGCFG, AES_EX_MSGCFG_DIR, AES_EX_OPERATINGMODE_DECRYPT);
        
        /* Set the operating mode*/
        MODIFY_REG(haes->Instance->MSGCFG, AES_EX_MSGCFG_MSG_BEGIN, AES_EX_BEGIN_ENABLE);
        
        /* Set the operating mode*/
        MODIFY_REG(haes->Instance->MSGCFG, AES_EX_MSGCFG_MSG_END, AES_EX_END_ENABLE);
        
        /* algo get algorithm selected */
        algo = haes->Instance->MSGCFG & AES_EX_MSGCFG_ALG_MODE_Msk;

        switch (algo)
        {
            case AES_EX_ECB:
            case AES_EX_CBC:
            case AES_EX_CTR:

                /* AES_EX decryption */
                status = AES_EX_AES_EX_Decrypt(haes, Timeout);
                break;

            case AES_EX_GCM:
            case AES_EX_CMAC:

                /* AES_EX GCM decryption */

                break;
            
            case AES_EX_CCM:

                /* AES_EX CCM encryption */
                status = AES_EX_AES_EXCCM_Process(haes, Timeout);
                break;

            default:
                haes->ErrorCode |= HAL_AES_EX_ERROR_NOT_SUPPORTED;
                /* Change the AES_EX peripheral state */
                haes->State = HAL_AES_EX_STATE_READY;
                /* Process unlocked */
                __HAL_UNLOCK(haes);
                return HAL_ERROR;
        }

        if (status == HAL_OK)
        {
            /* Change the AES_EX peripheral state */
            haes->State = HAL_AES_EX_STATE_READY;

            /* Process unlocked */
            __HAL_UNLOCK(haes);
        }
    }
    else
    {
        /* Busy error code field */
        haes->ErrorCode |= HAL_AES_EX_ERROR_BUSY;
        return HAL_ERROR;
    }

    /* Return function status */
    return HAL_OK;
}


/**
  * @}
  */


/**
  * @brief  Return the AES_EX error code.
  * @param  haes : pointer to a AES_EX_HandleTypeDef structure that contains
  *                 the configuration information for the  AES_EX IP
  * @retval AES_EX error code
  */
uint32_t HAL_AES_EX_GetError(AES_EX_HandleTypeDef *haes)
{
  return haes->ErrorCode;
}

/**
  * @brief  Returns the AES_EX state.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module.
  * @retval HAL state
  */
HAL_AES_EX_STATETypeDef HAL_AES_EX_GetState(AES_EX_HandleTypeDef *haes)
{
  return haes->State;
}

/**
  * @brief  Input FIFO transfer completed callback.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module.
  * @retval None
  */
__weak void HAL_AES_EX_InCpltCallback(AES_EX_HandleTypeDef *haes)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(haes);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_AES_EX_InCpltCallback can be implemented in the user file
   */
}

/**
  * @brief  Output FIFO transfer completed callback.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module.
  * @retval None
  */
__weak void HAL_AES_EX_OutCpltCallback(AES_EX_HandleTypeDef *haes)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(haes);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_AES_EX_OutCpltCallback can be implemented in the user file
   */
}

/**
  * @brief  AES_EX error callback.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module.
  * @retval None
  */
__weak void HAL_AES_EX_ErrorCallback(AES_EX_HandleTypeDef *haes)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(haes);

    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_AES_EX_ErrorCallback could be implemented in the user file
    */
}
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @addtogroup AES_EX_Private_Functions
  * @{
  */


/**
  * @brief  Encryption in ECB/CBC & CTR Algorithm with AES_EX Standard
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure
  * @param  Timeout: specify Timeout value
  * @retval HAL status
  */
static HAL_StatusTypeDef AES_EX_AES_EX_Encrypt(AES_EX_HandleTypeDef *haes, uint32_t Timeout)
{
    uint16_t outcount;  /* Temporary CrypOutCount Value */
    uint32_t DoKeyIVConfig = 1U; /* By default, carry out peripheral Key and IV configuration */
    

    if (haes->Init.KeyIVConfigSkip == AES_EX_KEYIVCONFIG_ONCE)
    {
        if (haes->KeyIVConfig == 1U)
        {
            /* If the Key and IV configuration has to be done only once
            and if it has already been done, skip it */
            DoKeyIVConfig = 0U;
        }
        else
        {
            /* If the Key and IV configuration has to be done only once
            and if it has not been done already, do it and set KeyIVConfig
            to keep track it won't have to be done again next time */
            haes->KeyIVConfig = 1U;
        }
    }

    if (DoKeyIVConfig == 1U)
    {
        /*  Set the Key*/
        AES_EX_SetKey(haes, haes->Init.KeySize);
        haes->Instance->CTXCFG = 0x03;

        if (haes->Init.Algorithm != AES_EX_ECB)
        {
            /* Set the Initialization Vector*/
            haes->Instance->CTXIV[0] = *(uint32_t *)(haes->Init.pInitVect);
            haes->Instance->CTXIV[1] = *(uint32_t *)(haes->Init.pInitVect + 1);
            haes->Instance->CTXIV[2] = *(uint32_t *)(haes->Init.pInitVect + 2);
            haes->Instance->CTXIV[3] = *(uint32_t *)(haes->Init.pInitVect + 3);
            
            haes->Instance->CTXCTR[0] = *(uint32_t *)(haes->Init.pInitVect);
            haes->Instance->CTXCTR[1] = *(uint32_t *)(haes->Init.pInitVect + 1);
            haes->Instance->CTXCTR[2] = *(uint32_t *)(haes->Init.pInitVect + 2);
            haes->Instance->CTXCTR[3] = *(uint32_t *)(haes->Init.pInitVect + 3);
            
            haes->Instance->CTXCBCKEY[0] = *(uint32_t *)(haes->Init.pInitVect);
            haes->Instance->CTXCBCKEY[1] = *(uint32_t *)(haes->Init.pInitVect + 1);
            haes->Instance->CTXCBCKEY[2] = *(uint32_t *)(haes->Init.pInitVect + 2);
            haes->Instance->CTXCBCKEY[3] = *(uint32_t *)(haes->Init.pInitVect + 3);
            
        }
    } /* if (DoKeyIVConfig == 1U) */

    /* Set the phase */
    haes->Phase = AES_EX_PHASE_PROCESS;


    /*Temporary CrypOutCount Value*/
    outcount = haes->CrypOutCount;
    
    while ((haes->CrypInCount < (haes->Size / 4U)) && (outcount < (haes->Size / 4U)))
    {
        /* Write plain Ddta and get cipher data */
        /* Enable AES_EX */
        __HAL_AES_EX_ENABLE(haes);

        AES_EX_AES_EX_ProcessData(haes, Timeout);
        /*Temporary CrypOutCount Value*/
        outcount = haes->CrypOutCount;
    }
       
    
  /* Disable AES_EX */
  __HAL_AES_EX_DISABLE(haes);

  /* Change the AES_EX state */
  haes->State = HAL_AES_EX_STATE_READY;

  /* Return function status */
  return HAL_OK;
}


/**
  * @brief  Decryption in ECB/CBC & CTR mode with AES_EX Standard
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure
  * @param  Timeout: Specify Timeout value
  * @retval HAL status
  */
static HAL_StatusTypeDef AES_EX_AES_EX_Decrypt(AES_EX_HandleTypeDef *haes, uint32_t Timeout)
{
    uint16_t outcount;  /* Temporary CrypOutCount Value */
    uint32_t DoKeyIVConfig = 1U; /* By default, carry out peripheral Key and IV configuration */

    if (haes->Init.KeyIVConfigSkip == AES_EX_KEYIVCONFIG_ONCE)
    {
        if (haes->KeyIVConfig == 1U)
        {
            /* If the Key and IV configuration has to be done only once
            and if it has already been done, skip it */
            DoKeyIVConfig = 0U;
        }
        else
        {
            /* If the Key and IV configuration has to be done only once
            and if it has not been done already, do it and set KeyIVConfig
            to keep track it won't have to be done again next time */
            haes->KeyIVConfig = 1U;
        }
    }

    if (DoKeyIVConfig == 1U)
    {
        /*  Key preparation for ECB/CBC */
        if (haes->Init.Algorithm != AES_EX_CTR)
        {
            if (haes->AutoKeyDerivation == DISABLE)/*Mode 2 Key preparation*/
            {
                /*  Set the Key*/
                AES_EX_SetKey(haes, haes->Init.KeySize);
                haes->Instance->CTXCFG = 0x03;
     
            }
            else /*Mode 4 : decryption & Key preparation*/
            {
                /*  Set the Key*/
                AES_EX_SetKey(haes, haes->Init.KeySize);
                haes->Instance->CTXCFG = 0x03;
                
            }

        }
        else  /*Algorithm CTR */
        {
            /*  Set the Key*/
            AES_EX_SetKey(haes, haes->Init.KeySize);
            haes->Instance->CTXCFG = 0x03;
        }

        /* Set IV */
        if (haes->Init.Algorithm != AES_EX_ECB)
        {
            /* Set the Initialization Vector*/
            haes->Instance->CTXIV[0] = *(uint32_t *)(haes->Init.pInitVect);
            haes->Instance->CTXIV[1] = *(uint32_t *)(haes->Init.pInitVect + 1);
            haes->Instance->CTXIV[2] = *(uint32_t *)(haes->Init.pInitVect + 2);
            haes->Instance->CTXIV[3] = *(uint32_t *)(haes->Init.pInitVect + 3);
            
            /* Set the Initialization Vector*/
            haes->Instance->CTXCBCKEY[0] = *(uint32_t *)(haes->Init.pInitVect);
            haes->Instance->CTXCBCKEY[1] = *(uint32_t *)(haes->Init.pInitVect + 1);
            haes->Instance->CTXCBCKEY[2] = *(uint32_t *)(haes->Init.pInitVect + 2);
            haes->Instance->CTXCBCKEY[3] = *(uint32_t *)(haes->Init.pInitVect + 3);
            
            /* Set the Initialization Vector*/
            haes->Instance->CTXCTR[0] = *(uint32_t *)(haes->Init.pInitVect);
            haes->Instance->CTXCTR[1] = *(uint32_t *)(haes->Init.pInitVect + 1);
            haes->Instance->CTXCTR[2] = *(uint32_t *)(haes->Init.pInitVect + 2);
            haes->Instance->CTXCTR[3] = *(uint32_t *)(haes->Init.pInitVect + 3);
            
        }
    } /* if (DoKeyIVConfig == 1U) */
    
    /* Set the phase */
    haes->Phase = AES_EX_PHASE_PROCESS;

    /* Enable AES_EX */
    __HAL_AES_EX_ENABLE(haes);

    /*Temporary CrypOutCount Value*/
    outcount = haes->CrypOutCount;

    while ((haes->CrypInCount < (haes->Size / 4U)) && (outcount < (haes->Size / 4U)))
    {
        /* Enable AES_EX */
        __HAL_AES_EX_ENABLE(haes);
        
        /* Write plain data and get cipher data */
        AES_EX_AES_EX_ProcessData(haes, Timeout);
        /*Temporary CrypOutCount Value*/
        outcount = haes->CrypOutCount;
    }

    /* Disable AES_EX */
    __HAL_AES_EX_DISABLE(haes);

    /* Change the AES_EX state */
    haes->State = HAL_AES_EX_STATE_READY;

    /* Return function status */
    return HAL_OK;
}


/**
  * @brief  Process Data: Write Input data in polling mode and used in AES_EX functions.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module
  * @param  Timeout: Specify Timeout value
  * @retval None
  */
static void AES_EX_AES_EX_ProcessData(AES_EX_HandleTypeDef *haes, uint32_t Timeout)
{
    uint32_t temp[4];  /* Temporary CrypOutBuff */
    uint32_t i;
    
    /* Write the input block in the IN FIFO */
    haes->Instance->INGRESSFIFO  = *(uint32_t *)(haes->pCrypInBuffPtr + haes->CrypInCount);
    haes->CrypInCount++;
    
    haes->Instance->INGRESSFIFO  = *(uint32_t *)(haes->pCrypInBuffPtr + haes->CrypInCount);
    haes->CrypInCount++;
    
    haes->Instance->INGRESSFIFO  = *(uint32_t *)(haes->pCrypInBuffPtr + haes->CrypInCount);
    haes->CrypInCount++;
    
    haes->Instance->INGRESSFIFO  = *(uint32_t *)(haes->pCrypInBuffPtr + haes->CrypInCount);
    haes->CrypInCount++;
    
    /* Wait for CCF flag to be raised */
    if (AES_EX_WaitOnCCFlag(haes, Timeout) != HAL_OK)
    {
        /* Disable the CRYP peripheral clock */
        __HAL_AES_EX_DISABLE(haes);

        /* Change state */
        haes->ErrorCode |= HAL_AES_EX_ERROR_TIMEOUT;
        haes->State = HAL_AES_EX_STATE_READY;

        /* Process unlocked */
        __HAL_UNLOCK(haes);
        #if (USE_HAL_AES_EX_REGISTER_CALLBACKS == 1)
            /*Call registered error callback*/
            haes->ErrorCallback(haes);
        #else
            /*Call legacy weak error callback*/
            HAL_AES_EX_ErrorCallback(haes);
        #endif /* USE_HAL_AES_EX_REGISTER_CALLBACKS */
    }


    /* Read the output block from the output FIFO and put them in temporary buffer then get CrypOutBuff from temporary buffer*/
    for (i = 0U; i < 4U; i++)
    {
        temp[i] = haes->Instance->ENGRESSFIFO;
    }
    
    i = 0U;
    
    while ((haes->CrypOutCount < ((haes->Size + 3U) / 4U)) && (i < 4U))
    {
        *(uint32_t *)(haes->pCrypOutBuffPtr + haes->CrypOutCount) = temp[i];
        haes->CrypOutCount++;
        i++;
    }

}



/**
  * @brief  Writes Key in Key registers.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module
  * @param  KeySize: Size of Key
  * @retval None
  */
static void AES_EX_SetKey(AES_EX_HandleTypeDef *haes, uint32_t KeySize)
{

    switch (KeySize)
    {
        case AES_EX_KEYSIZE_256B:
            haes->Instance->CTXKEY[0] = *(uint32_t *)(haes->Init.pKey);
            haes->Instance->CTXKEY[1] = *(uint32_t *)(haes->Init.pKey + 1);
            haes->Instance->CTXKEY[2] = *(uint32_t *)(haes->Init.pKey + 2);
            haes->Instance->CTXKEY[3] = *(uint32_t *)(haes->Init.pKey + 3);
            haes->Instance->CTXKEY[4] = *(uint32_t *)(haes->Init.pKey + 4);
            haes->Instance->CTXKEY[5] = *(uint32_t *)(haes->Init.pKey + 5);
            haes->Instance->CTXKEY[6] = *(uint32_t *)(haes->Init.pKey + 6);
            haes->Instance->CTXKEY[7] = *(uint32_t *)(haes->Init.pKey + 7);
        
            break;
        
        case AES_EX_KEYSIZE_128B:
            haes->Instance->CTXKEY[0] = *(uint32_t *)(haes->Init.pKey);
            haes->Instance->CTXKEY[1] = *(uint32_t *)(haes->Init.pKey + 1);
            haes->Instance->CTXKEY[2] = *(uint32_t *)(haes->Init.pKey + 2);
            haes->Instance->CTXKEY[3] = *(uint32_t *)(haes->Init.pKey + 3);
       
            break;
        
        default:
            break;
    }

}

#if defined (AES_EX_CR_ALGOMODE_AES_EX_GCM)|| defined (AES_EX)

/**
  * @brief  AES_EX CCM encryption/decryption processing in polling mode
  *         for TinyAES_EX IP, no encrypt/decrypt performed, only authentication preparation.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef AES_EX_AES_EXCCM_Process(AES_EX_HandleTypeDef *haes, uint32_t Timeout)
{
  uint32_t tickstart;
  uint32_t wordsize = (uint32_t)(haes->Size) / 4U;
  uint16_t outcount;  /* Temporary CrypOutCount Value */
  uint32_t DoKeyIVConfig = 1U; /* By default, carry out peripheral Key and IV configuration */

  uint32_t loopcounter;
  uint32_t npblb;
  uint32_t lastwordsize;


  if (haes->Init.KeyIVConfigSkip == AES_EX_KEYIVCONFIG_ONCE)
  {
    if (haes->KeyIVConfig == 1U)
    {
      /* If the Key and IV configuration has to be done only once
      and if it has already been done, skip it */
      DoKeyIVConfig = 0U;
      haes->SizesSum += haes->Size; /* Compute message total payload length */
    }
    else
    {
      /* If the Key and IV configuration has to be done only once
      and if it has not been done already, do it and set KeyIVConfig
      to keep track it won't have to be done again next time */
      haes->KeyIVConfig = 1U;
      haes->SizesSum = haes->Size; /* Merely store payload length */
    }
  }
  else
  {
    haes->SizesSum = haes->Size;
  }

  if (DoKeyIVConfig == 1U)
  {

    /*  Reset CrypHeaderCount */
    haes->CrypHeaderCount = 0U;


    /* Set the key */
    AES_EX_SetKey(haes, haes->Init.KeySize);


    /* Enable the CRYP peripheral */
    __HAL_AES_EX_ENABLE(haes);


    /*  wait until the end of computation */
    if (AES_EX_WaitOnCCFlag(haes, Timeout) != HAL_OK)
    {
      /* Change state */
      haes->ErrorCode |= HAL_AES_EX_ERROR_TIMEOUT;
      haes->State = HAL_AES_EX_STATE_READY;

      /* Process unlocked & return error */
      __HAL_UNLOCK(haes);
      return HAL_ERROR;
    }

    /* Set the phase */
    haes->Phase = AES_EX_PHASE_PROCESS;

    /* From that point the whole message must be processed, first the Header then the payload.
    First the  Header block(B1) : associated data length expressed in bytes concatenated with Associated Data (A)*/

    if (haes->Init.HeaderSize != 0U)
    {
      if ((haes->Init.HeaderSize % 4U) == 0U)
      {
        /* HeaderSize %4, no padding */
        for (loopcounter = 0U; (loopcounter < haes->Init.HeaderSize); loopcounter += 4U)
        {
          /* Write the Input block in the Data Input register */

          if (AES_EX_WaitOnCCFlag(haes, Timeout) != HAL_OK)
          {
            /* Disable the CRYP peripheral clock */
            __HAL_AES_EX_DISABLE(haes);

            /* Change state */
            haes->ErrorCode |= HAL_AES_EX_ERROR_TIMEOUT;
            haes->State = HAL_AES_EX_STATE_READY;

            /* Process unlocked */
            __HAL_UNLOCK(haes);
            return HAL_ERROR;
          }

        }
      }
      else
      {
        /*Write Header block in the IN FIFO without last block */
        for (loopcounter = 0U; (loopcounter < ((haes->Init.HeaderSize) - (haes->Init.HeaderSize % 4U))); loopcounter += 4U)
        {

          if (AES_EX_WaitOnCCFlag(haes, Timeout) != HAL_OK)
          {
            /* Disable the AES_EX peripheral clock */
            __HAL_AES_EX_DISABLE(haes);

            /* Change state */
            haes->ErrorCode |= HAL_AES_EX_ERROR_TIMEOUT;
            haes->State = HAL_AES_EX_STATE_READY;

            /* Process unlocked */
            __HAL_UNLOCK(haes);
            return HAL_ERROR;
          }

        }
        /*  Last block optionally pad the data with zeros*/
        for (loopcounter = 0U; (loopcounter < (haes->Init.HeaderSize % 4U)); loopcounter++)
        {
          haes->CrypHeaderCount++ ;
        }
        while (loopcounter < 4U)
        {
          loopcounter++;
        }

        if (AES_EX_WaitOnCCFlag(haes, Timeout) != HAL_OK)
        {
          /* Disable the AES_EX peripheral clock */
          __HAL_AES_EX_DISABLE(haes);

          /* Change state */
          haes->ErrorCode |= HAL_AES_EX_ERROR_TIMEOUT;
          haes->State = HAL_AES_EX_STATE_READY;

          /* Process unlocked */
          __HAL_UNLOCK(haes);
          return HAL_ERROR;
        }

      }
    }
  } /* if (DoKeyIVConfig == 1U) */
  /* Then the payload: cleartext payload (not the ciphertext payload).
  Write input Data, no output Data to get */
  if (haes->Size != 0U)
  {
    if ((haes->Size % 16U) != 0U)
    {
      /* recalculate  wordsize */
      wordsize = ((wordsize / 4U) * 4U) ;
    }

    /* Get tick */
    tickstart = HAL_GetTick();
    /*Temporary CrypOutCount Value*/
    outcount = haes->CrypOutCount;

    while ((haes->CrypInCount < wordsize) && (outcount < wordsize))
    {
      /* Write plain data and get cipher data */
      AES_EX_AES_EX_ProcessData(haes, Timeout);

      /*Temporary CrypOutCount Value*/
      outcount = haes->CrypOutCount;

      /* Check for the Timeout */
      if (Timeout != HAL_MAX_DELAY)
      {
        if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          /* Disable the AES_EX peripheral clock */
          __HAL_AES_EX_DISABLE(haes);

          /* Change state */
          haes->ErrorCode |= HAL_AES_EX_ERROR_TIMEOUT;
          haes->State = HAL_AES_EX_STATE_READY;

          /* Process unlocked */
          __HAL_UNLOCK(haes);
          return HAL_ERROR;
        }
      }
    }

    if ((haes->Size % 16U) != 0U)
    {
      /* Compute the number of padding bytes in last block of payload */
      npblb = ((((uint32_t)(haes->Size) / 16U) + 1U) * 16U) - (uint32_t)(haes->Size);

      /* Number of valid words (lastwordsize) in last block */
      if ((npblb % 4U) == 0U)
      {
        lastwordsize = (16U - npblb) / 4U;
      }
      else
      {
        lastwordsize = ((16U - npblb) / 4U) + 1U;
      }
      /*  Last block optionally pad the data with zeros*/
      for (loopcounter = 0U; loopcounter < lastwordsize; loopcounter ++)
      {
      }
      while (loopcounter < 4U)
      {
        loopcounter++;
      }
      /* Wait for CCF flag to be raised */
      if (AES_EX_WaitOnCCFlag(haes, Timeout) != HAL_OK)
      {
        /* Disable the AES_EX peripheral clock */
        __HAL_AES_EX_DISABLE(haes);

        /* Change state */
        haes->ErrorCode |= HAL_AES_EX_ERROR_TIMEOUT;
        haes->State = HAL_AES_EX_STATE_READY;

        /* Process unlocked */
        __HAL_UNLOCK(haes);
        return HAL_ERROR;
      }

    }
  }

  /* Return function status */
  return HAL_OK;
}


#endif /* AES_EX or GCM CCM defined*/



/**
  * @brief  Handle AES_EX hardware block Timeout when waiting for CCF flag to be raised.
  * @param  haes: pointer to a AES_EX_HandleTypeDef structure that contains
  *         the configuration information for AES_EX module.
  * @param  Timeout: Timeout duration.
  * @retval HAL status
  */
static HAL_StatusTypeDef AES_EX_WaitOnCCFlag(AES_EX_HandleTypeDef *haes, uint32_t Timeout)
{
    uint32_t tickstart;

    /* Get timeout */
    tickstart = HAL_GetTick();

    while (HAL_IS_BIT_CLR(haes->Instance->DONESTATUS, AES_EX_INGDMADONE_DMA_INGRESS_DONE))
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
            {
                return HAL_ERROR;
            }
        }
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

#endif /* HAL_AES_EX_MODULE_ENABLED */


/**
  * @}
  */
/**
  * @}
  */

