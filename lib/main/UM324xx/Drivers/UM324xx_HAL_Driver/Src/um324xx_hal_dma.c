/**
  ******************************************************************************
  * @file     um324xx_hal_dma.c
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

/** @defgroup DMA DMA
  * @brief DMA HAL module driver
  * @{
  */
#ifdef HAL_DMA_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/** @defgroup DMA_Private_Functions DMA Private Functions
  * @{
  */
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup DMA_Exported_Functions DMA Exported Functions
  * @{
  */
/** @defgroup DMA_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief   Initialization and de-initialization functions
  *
@verbatim
 ===============================================================================
             ##### Initialization and de-initialization functions  #####
 ===============================================================================
    [..]
    This section provides functions allowing to initialize the DMA Channel source
    and destination addresses, incrementation and data sizes, transfer direction,
    circular/normal mode selection, memory-to-memory mode selection and Channel priority value.
    [..]
    The HAL_DMA_Init() function follows the DMA configuration procedures as described in
    reference manual.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the DMA according to the specified
  *         parameters in the DMA_InitTypeDef and initialize the associated handle.
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma)
{
    uint32_t tmp;

    /* Check the DMA handle allocation */
    if (hdma == NULL||(hdma->Instance == NULL))
    {
        return HAL_ERROR;
    }

    /* Change DMA peripheral state */
    hdma->State = HAL_DMA_STATE_BUSY;

    /* Get the CTLx register value */
    tmp = *(&(hdma->Instance->CTL0)+ CHANNEL_OFFSET*hdma->DmaChannelSel);

    /* Clear PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR and MEM2MEM bits */
    tmp &= ((uint32_t)~(DMA_CTL0_TT_FC    | DMA_CTL0_SRC_MSIZE  | DMA_CTL0_DST_MSIZE  |
                        DMA_CTL0_SINC  | DMA_CTL0_DINC   | DMA_CTL0_SRC_TR_WIDTH   |
                        DMA_CTL0_DST_TR_WIDTH | DMA_CTL0_INT_EN));

    /* Prepare the DMA Channel configuration */
    tmp |=  hdma->Init.Direction   |
            hdma->Init.SrcInc      | hdma->Init.DstInc   |
            hdma->Init.SrcMSize    | hdma->Init.DstMSize |
            hdma->Init.SrcDataAlignment | hdma->Init.DstDataAlignment ;

    /* Write to DMA Channel CR register */
    *(&(hdma->Instance->CTL0)+ CHANNEL_OFFSET*hdma->DmaChannelSel) = tmp;

    /* Get the CFGHx register value */
    tmp = *(&(hdma->Instance->CFGH0)+ CHANNEL_OFFSET*hdma->DmaChannelSel);

    tmp &= ((uint32_t)~(  DMA_CFGH0_SRC_PER  |  DMA_CFGH0_DEST_PER | DMA_CFGH0_FIFO_MODE |
                          DMA_CFGH0_FCMODE));
    tmp |= hdma->Init.DstPer | hdma->Init.SrcPer | hdma->Init.FIFOMode | hdma->Init.FCMode;
    /* Write to DMA CFGHx register */
    *(&(hdma->Instance->CFGH0) + CHANNEL_OFFSET*hdma->DmaChannelSel) = tmp;


    /* Get the CFGx register value */
    tmp = *(&(hdma->Instance->CFG0)+ CHANNEL_OFFSET*hdma->DmaChannelSel);
    tmp &= ((uint32_t)~(  DMA_CFG0_HS_SEL_DST  |  DMA_CFG0_HS_SEL_SRC | DMA_CFG0_RELOAD_SRC |
                          DMA_CFG0_RELOAD_DST | DMA_CFG0_CH_PRIOR));

    tmp |= hdma->Init.DstHsSel | hdma->Init.SrcHsSel | hdma->Init.SrcReload | hdma->Init.DstReload | hdma->Init.Priority;
    /* Write to DMA CFGx register */
    *(&(hdma->Instance->CFG0)+ CHANNEL_OFFSET*hdma->DmaChannelSel) = tmp;

    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
        /*If it is memory to memory, it is not necessary to fill in the DMA handshake signal ID*/

    }
    else
    {
#if defined(UM32x42x) || defined(UM32x41x)
        /*Configure the dma peripheral handshake signal*/
        if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH)
        {
            hdma->DmamuxChannelSel = hdma->Init.DstPer >> DMA_CFGH0_DEST_PER_Pos;
            *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel) = (hdma->Init.DstRequest);
        }
        else if (hdma->Init.Direction == DMA_PERIPH_TO_MEMORY)
        {
            hdma->DmamuxChannelSel = hdma->Init.SrcPer >> DMA_CFGH0_SRC_PER_Pos;
            *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel) = (hdma->Init.SrcRequest);
        }
        else if (hdma->Init.Direction == DMA_PERIPH_TO_PERIPH)
        {
            hdma->DmamuxChannelSel = hdma->Init.SrcPer >> DMA_CFGH0_SRC_PER_Pos;
            *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel) = (hdma->Init.SrcRequest);

            hdma->DmamuxChannelSel = hdma->Init.DstPer >> DMA_CFGH0_DEST_PER_Pos;
            *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel) = (hdma->Init.DstRequest);
        }
#endif

#if defined(UM324xH)
        /*Configure the dma peripheral handshake signal*/
        if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH)
        {
            hdma->DmamuxChannelSel = hdma->Init.DstPer >> DMA_CFGH0_DEST_PER_Pos;
            *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel) = (hdma->Init.DstRequest);
        }
        else if (hdma->Init.Direction == DMA_PERIPH_TO_MEMORY)
        {
            hdma->DmamuxChannelSel = hdma->Init.SrcPer >> DMA_CFGH0_SRC_PER_Pos;
            *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel) = (hdma->Init.SrcRequest);
        }
        else if (hdma->Init.Direction == DMA_PERIPH_TO_PERIPH)
        {
            hdma->DmamuxChannelSel = hdma->Init.SrcPer >> DMA_CFGH0_SRC_PER_Pos;
            *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel) = (hdma->Init.SrcRequest);

            hdma->DmamuxChannelSel = hdma->Init.DstPer >> DMA_CFGH0_DEST_PER_Pos;
            *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel) = (hdma->Init.DstRequest);
        }
#endif

    }

    __HAL_DMA_ENABLE(hdma);
    /* Initialize the error code */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Initialize the DMA state*/
    hdma->State  = HAL_DMA_STATE_READY;

    /* Allocate lock resource and initialize it */
    hdma->Lock = HAL_UNLOCKED;

    return HAL_OK;
}

/**
  * @brief  DeInitialize the DMA peripheral.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma)
{
    /* Check the DMA handle allocation */
    if (NULL == hdma)
    {
        return HAL_ERROR;
    }

    __HAL_DMA_DISABLE(hdma);

    /* Clean all callbacks */
    hdma->XferTfrCallback  = NULL;
    hdma->XferBlockCallback  = NULL;
    hdma->XferSrcTranCallback = NULL;
    hdma->XferDstTranCallback = NULL;
    hdma->XferErrorCallback = NULL;

    /* Initialize the error code */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Initialize the DMA state */
    hdma->State = HAL_DMA_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hdma);

    return HAL_OK;
}

/**
  * @brief  Register a User DMA Callback
  *         To be used instead of the weak predefined callback
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *         @arg @ref  HAL_DMA_TFR_CB_ID
  *         @arg @ref  HAL_DMA_BLOCK_CB_ID
  *         @arg @ref  HAL_DMA_SRCTRAN_CB_ID
  *         @arg @ref  HAL_DMA_DSTTRAN_CB_ID
  *         @arg @ref  HAL_DMA_ERROR_CB_ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, pDMA_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        return HAL_ERROR;
    }

    switch (CallbackID)
    {
    case HAL_DMA_TFR_CB_ID :
        hdma->XferTfrCallback = pCallback;
        break;

    case HAL_DMA_BLOCK_CB_ID :
        hdma->XferBlockCallback = pCallback;
        break;

    case HAL_DMA_SRCTRAN_CB_ID :
        hdma->XferSrcTranCallback = pCallback;
        break;

    case HAL_DMA_DSTTRAN_CB_ID :
        hdma->XferDstTranCallback = pCallback;
        break;

    case HAL_DMA_ERROR_CB_ID :
        hdma->XferErrorCallback = pCallback;
        break;

    default :
        /* Return error status */
        status = HAL_ERROR;
        break;
    }
    return status;
}

/**
  * @brief  Unregister a DMA Callback
  *         ADC callback is redirected to the weak predefined callback
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *         the configuration information for the specified DMA Channel.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *         @arg @ref HAL_DMA_TFR_CB_ID
  *         @arg @ref HAL_DMA_BLOCK_CB_ID
  *         @arg @ref HAL_DMA_SRCTRAN_CB_ID
  *         @arg @ref HAL_DMA_DSTTRAN_CB_ID
  *         @arg @ref HAL_DMA_ERROR_CB_ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    switch (CallbackID)
    {
    case HAL_DMA_TFR_CB_ID :
        hdma->XferTfrCallback =  NULL;
        break;

    case HAL_DMA_BLOCK_CB_ID :
        hdma->XferBlockCallback = NULL;
        break;

    case HAL_DMA_SRCTRAN_CB_ID :
        hdma->XferSrcTranCallback = NULL;
        break;

    case HAL_DMA_DSTTRAN_CB_ID :
        hdma->XferDstTranCallback = NULL;
        break;

    case HAL_DMA_ERROR_CB_ID :
        hdma->XferErrorCallback = NULL;
        break;

    default :
        /* Return error status */
        status = HAL_ERROR;
        break;
    }
    return status;
}

#if defined(UM32x42x) || defined(UM32x41x)
/**
  * @brief  Configure the DMAMUX synchronization parameters for a given DMA channel (instance).
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA channel.
  * @param  pSyncConfig : pointer to HAL_DMA_MuxSyncConfigTypeDef : contains the DMAMUX synchronization parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_ConfigMuxSync(DMA_HandleTypeDef *hdma, HAL_DMA_MuxSyncConfigTypeDef *pSyncConfig)
{
    uint32_t tmp;
    /*Check if the DMA state is ready */
    if (hdma->State == HAL_DMA_STATE_READY)
    {
        /* Process Locked */
        __HAL_LOCK(hdma);

        tmp = (uint32_t) ((pSyncConfig->EventEnable << DMAMUX_C0CR_EGE_Pos) | (pSyncConfig->RequestNumber << DMAMUX_C0CR_NBREQ_Pos) |
                          (pSyncConfig->SyncEnable << DMAMUX_C0CR_SE_Pos) | pSyncConfig->SyncPolarity | (pSyncConfig->SyncSignalID << DMAMUX_C0CR_SYNC_ID_Pos));


        if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
        {
            MODIFY_REG( *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->Init.DstPer),
                        DMAMUX_C0CR_SYNC_ID | DMAMUX_C0CR_NBREQ | DMAMUX_C0CR_SPOL |
                        DMAMUX_C0CR_SE | DMAMUX_C0CR_EGE,0U);
        }
        else
        {
            if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH)
            {
                hdma->DmamuxChannelSel = hdma->Init.DstPer>>DMA_CFGH0_DEST_PER_Pos;
            }
            else if (hdma->Init.Direction == DMA_PERIPH_TO_MEMORY)
            {
                hdma->DmamuxChannelSel = hdma->Init.SrcPer>>DMA_CFGH0_SRC_PER_Pos;
            }
            else if (hdma->Init.Direction == DMA_PERIPH_TO_PERIPH)
            {
                hdma->DmamuxChannelSel = hdma->Init.SrcPer>>DMA_CFGH0_SRC_PER_Pos;
            }
            MODIFY_REG( *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel),
                        DMAMUX_C0CR_SYNC_ID | DMAMUX_C0CR_NBREQ | DMAMUX_C0CR_SPOL |
                        DMAMUX_C0CR_SE | DMAMUX_C0CR_EGE,tmp);
        }

        /* Process UnLocked */
        __HAL_UNLOCK(hdma);

        return HAL_OK;
    }
    else
    {
        /*DMA State not Ready*/
        return HAL_ERROR;
    }
}

/**
  * @brief  Configure the DMAMUX request generator block used by the given DMA channel (instance).
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA channel.
  * @param  pRequestGeneratorConfig : pointer to HAL_DMA_MuxRequestGeneratorConfigTypeDef :
  *         contains the request generator parameters.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_ConfigMuxRequestGenerator(DMA_HandleTypeDef *hdma,
        HAL_DMA_MuxRequestGeneratorConfigTypeDef *pRequestGeneratorConfig)
{
    uint32_t tmp;
    /* check if the DMA state is ready
       and DMA is using a DMAMUX request generator block
    */
    if (hdma->State == HAL_DMA_STATE_READY)
    {
        /* Process Locked */
        __HAL_LOCK(hdma);

        tmp = (uint32_t) ((pRequestGeneratorConfig->GeneratorEnable << DMAMUX_RG0CR_GE_Pos)| pRequestGeneratorConfig->Polarity|
                          (pRequestGeneratorConfig->RequestNumber << DMAMUX_RG0CR_GNBREQ_Pos) |
                          (pRequestGeneratorConfig->SignalID << DMAMUX_RG0CR_SIG_ID_Pos));

        if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
        {
            MODIFY_REG( *(&(hdma->DmamuxBaseAddress->RG0CR) + 1U*hdma->DmamuxChannelSel),
                        DMAMUX_RG0CR_GNBREQ | DMAMUX_RG0CR_GPOL | DMAMUX_RG0CR_GE |
                        DMAMUX_RG0CR_SIG_ID,0U);
        }
        else
        {
            if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH)
            {
                if (hdma->Init.DstRequest <= DMA_REQUEST_DMAMUX_REQ_GEN3)
                {
                    hdma->DmamuxReqGenSel = hdma->Init.DstRequest;
                }
                else
                {
                    return HAL_ERROR;
                }

            }
            else if (hdma->Init.Direction == DMA_PERIPH_TO_MEMORY)
            {
                if (hdma->Init.SrcRequest <= DMA_REQUEST_DMAMUX_REQ_GEN3)
                {
                    hdma->DmamuxReqGenSel = hdma->Init.SrcRequest;
                }
                else
                {
                    return HAL_ERROR;
                }
            }
            else if (hdma->Init.Direction == DMA_PERIPH_TO_PERIPH)
            {
                if (hdma->Init.SrcRequest <= DMA_REQUEST_DMAMUX_REQ_GEN3)
                {
                    hdma->DmamuxReqGenSel = hdma->Init.SrcRequest;
                }
                else
                {
                    return HAL_ERROR;
                }
            }

            MODIFY_REG( *(&(hdma->DmamuxBaseAddress->RG0CR) + 1U*hdma->DmamuxReqGenSel),
                        DMAMUX_RG0CR_GNBREQ | DMAMUX_RG0CR_GPOL | DMAMUX_RG0CR_GE |
                        DMAMUX_RG0CR_SIG_ID,tmp);

        }
        /* Process UnLocked */
        __HAL_UNLOCK(hdma);

        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}
#endif

#if defined(UM324xH)
/**
  * @brief  Configure the DMAMUX synchronization parameters for a given DMA channel (instance).
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA channel.
  * @param  pSyncConfig : pointer to HAL_DMA_MuxSyncConfigTypeDef : contains the DMAMUX synchronization parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_ConfigMuxSync(DMA_HandleTypeDef *hdma, HAL_DMA_MuxSyncConfigTypeDef *pSyncConfig)
{
    uint32_t tmp;
    /*Check if the DMA state is ready */
    if (hdma->State == HAL_DMA_STATE_READY)
    {
        /* Process Locked */
        __HAL_LOCK(hdma);

        tmp = (uint32_t) ((pSyncConfig->EventEnable << DMAMUX_C0CR_EGE_Pos) | (pSyncConfig->RequestNumber << DMAMUX_C0CR_NBREQ_Pos) |
                          (pSyncConfig->SyncEnable << DMAMUX_C0CR_SE_Pos) | pSyncConfig->SyncPolarity | (pSyncConfig->SyncSignalID << DMAMUX_C0CR_SYNC_ID_Pos));


        if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
        {
            MODIFY_REG( *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->Init.DstPer),
                        DMAMUX_C0CR_SYNC_ID | DMAMUX_C0CR_NBREQ | DMAMUX_C0CR_SPOL |
                        DMAMUX_C0CR_SE | DMAMUX_C0CR_EGE,0U);
        }
        else
        {
            if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH)
            {
                hdma->DmamuxChannelSel = hdma->Init.DstPer>>DMA_CFGH0_DEST_PER_Pos;
            }
            else if (hdma->Init.Direction == DMA_PERIPH_TO_MEMORY)
            {
                hdma->DmamuxChannelSel = hdma->Init.SrcPer>>DMA_CFGH0_SRC_PER_Pos;
            }
            else if (hdma->Init.Direction == DMA_PERIPH_TO_PERIPH)
            {
                hdma->DmamuxChannelSel = hdma->Init.SrcPer>>DMA_CFGH0_SRC_PER_Pos;
            }
            MODIFY_REG( *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel),
                        DMAMUX_C0CR_SYNC_ID | DMAMUX_C0CR_NBREQ | DMAMUX_C0CR_SPOL |
                        DMAMUX_C0CR_SE | DMAMUX_C0CR_EGE,tmp);
        }

        /* Process UnLocked */
        __HAL_UNLOCK(hdma);

        return HAL_OK;
    }
    else
    {
        /*DMA State not Ready*/
        return HAL_ERROR;
    }
}

/**
  * @brief  Configure the DMAMUX request generator block used by the given DMA channel (instance).
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA channel.
  * @param  pRequestGeneratorConfig : pointer to HAL_DMA_MuxRequestGeneratorConfigTypeDef :
  *         contains the request generator parameters.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_ConfigMuxRequestGenerator(DMA_HandleTypeDef *hdma,
        HAL_DMA_MuxRequestGeneratorConfigTypeDef *pRequestGeneratorConfig)
{
    uint32_t tmp;
    /* check if the DMA state is ready
       and DMA is using a DMAMUX request generator block
    */
    if (hdma->State == HAL_DMA_STATE_READY)
    {
        /* Process Locked */
        __HAL_LOCK(hdma);

        tmp = (uint32_t) ((pRequestGeneratorConfig->GeneratorEnable << DMAMUX_RG0CR_GE_Pos)| pRequestGeneratorConfig->Polarity|
                          (pRequestGeneratorConfig->RequestNumber << DMAMUX_RG0CR_GNBREQ_Pos) |
                          (pRequestGeneratorConfig->SignalID << DMAMUX_RG0CR_SIG_ID_Pos));


        if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
        {
            MODIFY_REG( *(&(hdma->DmamuxBaseAddress->RG0CR) + 1U*hdma->DmamuxChannelSel),
                        DMAMUX_RG0CR_GNBREQ | DMAMUX_RG0CR_GPOL | DMAMUX_RG0CR_GE |
                        DMAMUX_RG0CR_SIG_ID,0U);
        }
        else
        {
            if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH)
            {
                if (hdma->Init.DstRequest <= DMA_REQUEST_DMAMUX_REQ_GEN3)
                {
                    hdma->DmamuxReqGenSel = hdma->Init.DstRequest;
                }
                else
                {
                    return HAL_ERROR;
                }

            }
            else if (hdma->Init.Direction == DMA_PERIPH_TO_MEMORY)
            {
                if (hdma->Init.SrcRequest <= DMA_REQUEST_DMAMUX_REQ_GEN3)
                {
                    hdma->DmamuxReqGenSel = hdma->Init.SrcRequest;
                }
                else
                {
                    return HAL_ERROR;
                }
            }
            else if (hdma->Init.Direction == DMA_PERIPH_TO_PERIPH)
            {
                if (hdma->Init.SrcRequest <= DMA_REQUEST_DMAMUX_REQ_GEN3)
                {
                    hdma->DmamuxReqGenSel = hdma->Init.SrcRequest;
                }
                else
                {
                    return HAL_ERROR;
                }
            }

            MODIFY_REG( *(&(hdma->DmamuxBaseAddress->RG0CR) + 1U*hdma->DmamuxReqGenSel),
                        DMAMUX_RG0CR_GNBREQ | DMAMUX_RG0CR_GPOL | DMAMUX_RG0CR_GE |
                        DMAMUX_RG0CR_SIG_ID,tmp);

        }
        /* Process UnLocked */
        __HAL_UNLOCK(hdma);

        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}
#endif

/**
  * @}
  */

/** @defgroup DMA_Exported_Functions_Group2 Input and Output operation functions
  *  @brief   Input and Output operation functions
  *
@verbatim
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and Start DMA transfer
      (+) Configure the source, destination address and data length and
          Start DMA transfer with interrupt
      (+) Abort DMA transfer
      (+) Poll for transfer complete
      (+) Handle DMA interrupt request

@endverbatim
  * @{
  */

/**
  * @brief  Start the DMA Transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination (up to 256Kbytes-1)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hdma);

    if (HAL_DMA_STATE_READY == hdma->State)
    {
        /* Change DMA peripheral state */
        hdma->State = HAL_DMA_STATE_BUSY;
        hdma->ErrorCode = HAL_DMA_ERROR_NONE;

        /* Configure the source, destination address and the data length & clear flags*/
        DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);

        /*<channel  enable*/
        __HAL_DMA_CHANNEL_ENABLE(hdma,hdma->DmaChannelSel);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
        status = HAL_BUSY;
    }
    return status;
}

/**
  * @brief  Start the DMA Transfer with interrupt enabled.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination (up to 256Kbytes-1)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress,
                                   uint32_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hdma);

    __HAL_DMA_CHANNEL_DISABLE(hdma,hdma->DmaChannelSel);
    hdma->State = HAL_DMA_STATE_READY;
    if (HAL_DMA_STATE_READY == hdma->State)
    {
        /* Change DMA peripheral state */
        hdma->State = HAL_DMA_STATE_BUSY;
        hdma->ErrorCode = HAL_DMA_ERROR_NONE;

        /* Configure the source, destination address and the data length & clear flags*/
        DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);
        /*<channel  enable*/
        SET_BIT(*(&(hdma->Instance->CTL0)+ CHANNEL_OFFSET*hdma->DmaChannelSel),DMA_CTL0_INT_EN);

        /*Enable DMA various interrupts*/
        // __HAL_DMA_ENABLE_TFR_IT(hdma,hdma->DmaChannelSel);
        __HAL_DMA_ENABLE_BLOCK_IT(hdma,hdma->DmaChannelSel);
        // __HAL_DMA_ENABLE_SRCTRAN_IT(hdma,hdma->DmaChannelSel);
        // __HAL_DMA_ENABLE_DSTTRAN_IT(hdma,hdma->DmaChannelSel);
        // __HAL_DMA_ENABLE_ERROR_IT(hdma,hdma->DmaChannelSel);

#if defined(UM32x42x)
        if (((*(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel)) & DMAMUX_C0CR_SE) != 0U)
        {
            *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel) |=  DMAMUX_C0CR_SOIE;
        }

        if (((*(&(hdma->DmamuxBaseAddress->RG0CR) + 1U*hdma->DmamuxChannelSel)) & DMAMUX_RG0CR_GE) != 0U)
        {
            *(&(hdma->DmamuxBaseAddress->RG0CR) + 1U*hdma->DmamuxChannelSel) |=  DMAMUX_RG0CR_OIE;
        }
#endif

#if defined(UM324xH)
        if (((*(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel)) & DMAMUX_C0CR_SE) != 0U)
        {
            *(&(hdma->DmamuxBaseAddress->C0CR) + 1U*hdma->DmamuxChannelSel) |=  DMAMUX_C0CR_SOIE;
        }

        if (((*(&(hdma->DmamuxBaseAddress->RG0CR) + 1U*hdma->DmamuxChannelSel)) & DMAMUX_RG0CR_GE) != 0U)
        {
            *(&(hdma->DmamuxBaseAddress->RG0CR) + 1U*hdma->DmamuxChannelSel) |=  DMAMUX_RG0CR_OIE;
        }
#endif

        __HAL_DMA_CHANNEL_ENABLE(hdma,hdma->DmaChannelSel);

    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
        status = HAL_BUSY;
    }
    return status;
}

/**
  * @brief  Aborts the DMA Transfer.
  * @param  hdma   pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Stream.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
    /* Disable the stream */
    __HAL_DMA_CHANNEL_DISABLE(hdma,hdma->DmaChannelSel);

    /* Change the DMA state*/
    hdma->State = HAL_DMA_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hdma);

    return HAL_OK;
}

/**
  * @brief  Aborts the DMA Transfer in Interrupt mode.
  * @param  hdma   pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Stream.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
    __HAL_DMA_CHANNEL_DISABLE(hdma,hdma->DmaChannelSel);
    /* Disable all the transfer interrupts */
    __HAL_DMA_DISABLE_TFR_IT(hdma,hdma->DmaChannelSel);
    __HAL_DMA_DISABLE_BLOCK_IT(hdma,hdma->DmaChannelSel);
    __HAL_DMA_DISABLE_SRCTRAN_IT(hdma,hdma->DmaChannelSel);
    __HAL_DMA_DISABLE_DSTTRAN_IT(hdma,hdma->DmaChannelSel);
    __HAL_DMA_DISABLE_ERROR_IT(hdma,hdma->DmaChannelSel);

    CLEAR_BIT(*(&(hdma->Instance->CTL0)+ CHANNEL_OFFSET*hdma->DmaChannelSel),DMA_CTL0_INT_EN);

    /* Change the DMA state*/
    hdma->State = HAL_DMA_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hdma);

    return HAL_OK;
}

/**
  * @brief  Polling for transfer complete.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Channel.
  * @param  Timeout       Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma,
        uint32_t Timeout)
{
    uint32_t tickstart;

    if (HAL_DMA_STATE_BUSY != hdma->State)
    {
        /* no transfer ongoing */
        hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;
        __HAL_UNLOCK(hdma);
        return HAL_ERROR;
    }

    /* Get tick */
    tickstart = HAL_GetTick();

    while (hdma->Instance->CHENREG == (uint32_t)(1<<(hdma->DmaChannelSel)))
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
            {
                /* Update error code */
                hdma->ErrorCode = HAL_DMA_ERROR_TIMEOUT;

                /* Change the DMA state */
                hdma->State = HAL_DMA_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(hdma);

                return HAL_ERROR;
            }
        }
    }
    /* Change the DMA state */
    hdma->State = HAL_DMA_STATE_READY;
    /* Process Unlocked */
    __HAL_UNLOCK(hdma);

    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup DMA_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief    Peripheral State and Errors functions
  *
@verbatim
 ===============================================================================
            ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DMA state
      (+) Get error code

@endverbatim
  * @{
  */
/**
* @brief  Handle DMA interrupt request.
* @param  hdma pointer to a DMA_HandleTypeDef structure that contains
*               the configuration information for the specified DMA Channel.
* @retval None
*/
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
{
    uint32_t tmp_channel = hdma->DmaChannelSel;

    uint32_t flag_tfr = hdma->Instance->STATUSTFR;
    uint32_t source_tfr = hdma->Instance->MASKTFR;

    uint32_t flag_block = hdma->Instance->STATUSBLOCK;
    uint32_t source_block = hdma->Instance->MASKBLOCK;

    uint32_t flag_srctran = hdma->Instance->STATUSSRCTRAN;
    uint32_t source_srctran = hdma->Instance->MASKSRCTRAN;

    uint32_t flag_dsttran = hdma->Instance->STATUSDSTTRAN;
    uint32_t source_dsttran = hdma->Instance->MASKDSTTRAN;

    uint32_t flag_error = hdma->Instance->STATUSERR;
    uint32_t source_error = hdma->Instance->MASKERR;

    if ((0u != (source_tfr & (uint32_t)(1U << tmp_channel))) && (0u != (flag_tfr & (uint32_t)(1U << tmp_channel))))
    {
        __HAL_DMA_CLEAR_TFR_FLAG(hdma, tmp_channel);

        /* Change the DMA state */
        hdma->State = HAL_DMA_STATE_READY;

        if (hdma->XferTfrCallback != NULL)
        {
            hdma->XferTfrCallback(hdma);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
    }

    if ((0u != (source_block & (uint32_t)(1U << tmp_channel))) && (0u != (flag_block & (uint32_t)(1U << tmp_channel))))
    {
        __HAL_DMA_CLEAR_BLOCK_FLAG(hdma, tmp_channel);

        /* Change the DMA state */
        hdma->State = HAL_DMA_STATE_READY;

        if (hdma->XferBlockCallback != NULL)
        {
            hdma->XferBlockCallback(hdma);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
    }

    if ((0u != (source_srctran & (uint32_t)(1U << tmp_channel))) && (0u != (flag_srctran & (uint32_t)(1U << tmp_channel))))
    {
        __HAL_DMA_CLEAR_SRCTRAN_FLAG(hdma, tmp_channel);

        /* Change the DMA state */
        hdma->State = HAL_DMA_STATE_READY;
        /* Conversion complete callback */

        if (hdma->XferSrcTranCallback != NULL)
        {
            hdma->XferSrcTranCallback(hdma);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
    }

    if ((0u != (source_dsttran & (uint32_t)(1U << tmp_channel))) && (0u != (flag_dsttran & (uint32_t)(1U << tmp_channel))))
    {
        /* Clear regular group conversion flag */
        __HAL_DMA_CLEAR_DSTTRAN_FLAG(hdma, tmp_channel);

        /* Change the DMA state */
        hdma->State = HAL_DMA_STATE_READY;

        if (hdma->XferDstTranCallback != NULL)
        {
            hdma->XferDstTranCallback(hdma);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
    }

    if ((0u != (source_error & (uint32_t)(1U << tmp_channel))) && (0u != (flag_error & (uint32_t)(1U << tmp_channel))))
    {
        /* Clear regular group conversion flag */
        __HAL_DMA_CLEAR_ERROR_FLAG(hdma, tmp_channel);

        /* Change the DMA state */
        hdma->State = HAL_DMA_STATE_ERROR;

        if (hdma->XferErrorCallback != NULL)
        {
            hdma->XferErrorCallback(hdma);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
    }
}

/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group3
  *
@verbatim
 ===============================================================================
                    ##### State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DMA state
      (+) Get error code

@endverbatim
  * @{
  */

/**
  * @brief  Returns the DMA state.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.
  * @retval HAL state
  */
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma)
{
    return hdma->State;
}

/**
  * @brief  Return the DMA error code
  * @param  hdma  pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Stream.
  * @retval DMA Error Code
  */
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *hdma)
{
    return hdma->ErrorCode;
}

/**
  * @}
  */

/** @addtogroup DMA_Private_Functions
  * @{
  */

/**
  * @brief  Sets the DMA Transfer parameter.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval HAL status
  */
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
#if defined(UM32x42x)
    /* Clear the DMAMUX synchro overrun flag */
    hdma->DmamuxBaseAddress->CFR = DMAMUX_CFR_CSOF_Msk;

    /* Clear the DMAMUX request generator overrun flag */
    hdma->DmamuxBaseAddress->RGCFR = DMAMUX_RGCFR_COF_Msk;
#endif

#if defined(UM324xH)
    /* Clear the DMAMUX synchro overrun flag */
    hdma->DmamuxBaseAddress->CFR = DMAMUX_CFR_CSOF_Msk;

    /* Clear the DMAMUX request generator overrun flag */
    hdma->DmamuxBaseAddress->RGCFR = DMAMUX_RGCFR_COF_Msk;
#endif

    /* Clear all flags */
    hdma->Instance->CLEARTFR = DMA_CLEARTFR_CLEARTFR_Msk;
    hdma->Instance->CLEARBLOCK = DMA_CLEARBLOCK_CLEARBLOCK_Msk;
    hdma->Instance->CLEARSRCTRAN = DMA_CLEARSRCTRAN_CLEARSRCTRAN_Msk;
    hdma->Instance->CLEARDSTTRAN = DMA_CLEARSRCTRAN_CLEARSRCTRAN_Msk;
    hdma->Instance->CLEARERR = DMA_CLEARERR_CLEARERR_Msk;


    /* Configure DMA Channel data length Based on channel number*/
    *(&(hdma->Instance->CTLH0)+ CHANNEL_OFFSET*hdma->DmaChannelSel) = DataLength;

    /* Configure DMA Channel source address */
    *(&(hdma->Instance->SAR0)+ CHANNEL_OFFSET*hdma->DmaChannelSel) = SrcAddress;

    /* Configure DMA Channel destination address */
    *(&(hdma->Instance->DAR0)+ CHANNEL_OFFSET*hdma->DmaChannelSel) = DstAddress;

}
#endif /* HAL_DMA_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
