/**
  ******************************************************************************
  * @file     um324xF_hal_qspi.c 
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-05-17  
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
#include "string.h"

/** @addtogroup UM324xF_HAL_Driver
  * @{
  */
  
/** @defgroup QSPI QSPI
  * @brief HAL QSPI module driver
  * @{
  */
  
#ifdef HAL_QSPI_MODULE_ENABLED

/** @defgroup QSPI_functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void QSPI_DMARxCplt(DMA_HandleTypeDef *hdma);
static void QSPI_DMATxCplt(DMA_HandleTypeDef *hdma);
static void QSPI_DMAError(DMA_HandleTypeDef *hdma);
static HAL_StatusTypeDef QSPI_WaitFlagStateUntilTimeout(QSPI_HandleTypeDef *hqspi, uint32_t Flag, FlagStatus State, uint32_t Tickstart, uint32_t Timeout);
static void QSPI_Config(QSPI_HandleTypeDef *hqspi);

/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup QSPI_Exported_Functions QSPI Exported Functions
  * @{
  */

/** @defgroup QSPI_Exported_Functions_Group1 Initialization/de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
===============================================================================
            ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to :
      (+) Initialize the QSPI.
      (+) De-initialize the QSPI.

@endverbatim
  * @{
  */
/**
  * @brief Initialize the QSPI mode according to the specified parameters
  *        in the QSPI_InitTypeDef and initialize the associated handle.
  * @param hqspi QSPI handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_QSPI_Init(QSPI_HandleTypeDef *hqspi)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();
    
    /* Check the QSPI handle allocation */
    if(hqspi == NULL)
    {
        return HAL_ERROR;
    }
    
    if(hqspi->State == HAL_QSPI_STATE_RESET)
    { 
        /* Allocate lock resource and initialize it */
        hqspi->Lock = HAL_UNLOCKED;        

#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
        /* Reset Callback pointers in HAL_QSPI_STATE_RESET only */    
        hqspi->ErrorCallback                    = HAL_QSPI_ErrorCallback;
        hqspi->AbortCpltCallback                = HAL_QSPI_AbortCpltCallback;
        hqspi->CmdCpltCallback                  = HAL_QSPI_CmdCpltCallback;
        hqspi->UnderFlowCallback                = HAL_QSPI_UnderFlowCallback;
        hqspi->IndacOperationCallback           = HAL_QSPI_IndacOperationCallback;
        hqspi->AttemptToWriteProtectCallback    = HAL_QSPI_AttemptToWriteProtectCallback;   
        hqspi->IllegalAHBDetectedCallback       = HAL_QSPI_IllegalAHBDetectedCallback;
        hqspi->ReceiveFlowCallback              = HAL_QSPI_ReceiveFlowCallback;
        hqspi->MaxPollingCyclesCallback         = HAL_QSPI_MaxPollingCyclesCallback;        
        hqspi->RxCpltCallback                   = HAL_QSPI_RxCpltCallback;
        hqspi->TxCpltCallback                   = HAL_QSPI_TxCpltCallback;
        hqspi->RxNotEmptyCpltCallback           = HAL_QSPI_RxNotEmptyCpltCallback;
        hqspi->TxNotFullCpltCallback            = HAL_QSPI_TxNotFullCpltCallback;        
        hqspi->TimeOutCallback                  = HAL_QSPI_TimeOutCallback;
        
        if(hqspi->MspInitCallback == NULL)
        {
        hqspi->MspInitCallback = HAL_QSPI_MspInit;
        }
    
        /* Init the low level hardware */
        hqspi->MspInitCallback(hqspi);
#else
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_QSPI_MspInit(hqspi);
#endif
        /* Configure the default timeout for the QSPI memory access */
        HAL_QSPI_SetTimeout(hqspi, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);    
    }

    /* Wait till BUSY flag reset */
    status = QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_STATE_IDLE, SET, tickstart, hqspi->Timeout);

    if(status == HAL_OK)
    {
        /* Call the configuration function */        
         QSPI_Config(hqspi);
        
        /* Enable the QSPI peripheral */
        __HAL_QSPI_ENABLE(hqspi);
    
        /* Set QSPI error code to none */
        hqspi->ErrorCode = HAL_QSPI_ERROR_NONE;
    
        /* Initialize the QSPI state */
        hqspi->State = HAL_QSPI_STATE_READY;        
    }
  
    /* Release Lock */
    __HAL_UNLOCK(hqspi);

    /* Return function status */
    return status;    
}
    
/**
  * @brief De-Initialize the QSPI peripheral.
  * @param hqspi QSPI handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_QSPI_DeInit(QSPI_HandleTypeDef *hqspi)
{
    
    /* Check the QSPI handle allocation */
    if(hqspi == NULL)
    {
        return HAL_ERROR;
    }
    
    /* Disable the QSPI Peripheral Clock */
    __HAL_QSPI_DISABLE(hqspi);
    
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    if(hqspi->MspDeInitCallback == NULL)
    {
        hqspi->MspDeInitCallback = HAL_QSPI_MspDeInit;
    }

    /* DeInit the low level hardware */
    hqspi->MspDeInitCallback(hqspi);
#else
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    HAL_QSPI_MspDeInit(hqspi);
#endif    
    /* Set QSPI error code to none */
    hqspi->ErrorCode = HAL_QSPI_ERROR_NONE;

    /* Initialize the QSPI state */
    hqspi->State = HAL_QSPI_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hqspi);

    return HAL_OK;   
}

/**
  * @brief Initialize the QSPI MSP.
  * @param hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_QSPI_MspInit can be implemented in the user file
    */
}

/**
  * @brief DeInitialize the QSPI MSP.
  * @param hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_QSPI_MspDeInit can be implemented in the user file
    */
}

/**
  * @}
  */

/** @defgroup QSPI_Exported_Functions_Group2 Input and Output operation functions
  *  @brief QSPI Transmit/Receive functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to :
      (+) Handle the interrupts.
      (+) Transmit data in blocking, interrupt or DMA mode.
      (+) Receive data in blocking, interrupt or DMA mode.

@endverbatim
  * @{
  */
  
/**
  * @brief Handle QSPI interrupt request.
  * @param hqspi QSPI handle
  * @retval None
  */
void HAL_QSPI_IRQHandler(QSPI_HandleTypeDef *hqspi)
{
    uint32_t flag = READ_REG(hqspi->Instance->IFR);    
    uint32_t itsource = READ_REG(hqspi->Instance->IMR);  

    /* it's valid in standard mode */
    if(hqspi->Init.WorkMode == QSPI_WORKMODE_STANDARD)
    {
        /* QSPI Small capacity TXFIFO full interrupt occurred ----------------------------------*/
        if(((flag & QSPI_FLAG_STFFF) != 0U) && ((itsource & QSPI_IT_STFFF) != 0U))
        {  
            while(__HAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_STFFF) != RESET)
            {  
                /* Disable the QSPI Small capacity TXFIFO full Interrupt */
                __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_STFFF);
                
                /* Clear interrupt */
                __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_STFFF);
            }
            /* Change state of QSPI */
            hqspi->State = HAL_QSPI_STATE_READY;    
            
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->TxCpltCallback(hqspi);
#else
    HAL_QSPI_TxCpltCallback(hqspi);
#endif       
             
        }
        /* QSPI Small capacity TXFIFO non-empty interrupt occurred ----------------------------------*/
        if(((flag & QSPI_FLAG_STFNFF) != 0U) && ((itsource & QSPI_IT_STFNFF) != 0U))
        {
             while(__HAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_STFNFF) != RESET)
             {  
                 /* Disable the QSPI Small capacity TXFIFO non-empty Interrupt */
                 __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_STFNFF);
                 
                 /* Clear interrupt */
                __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_STFNFF);
             } 
             
            /* Change state of QSPI */
            hqspi->State = HAL_QSPI_STATE_READY; 
             
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->TxNotFullCpltCallback(hqspi);
#else
    HAL_QSPI_TxNotFullCpltCallback(hqspi);
#endif               
        }   
        
        /* QSPI Small capacity RXFIFO full interrupt occurred ----------------------------------*/
        if(((flag & QSPI_FLAG_SRFFF) != 0U) && ((itsource & QSPI_IT_SRFFF) != 0U))
        {
             while(__HAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_SRFFF) != RESET)
             {  
                 /* Disable the QSPI Small capacity RXFIFO full Interrupt */
                 __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_SRFFF);
                 
                 /* Clear interrupt */
                __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_SRFFF);
             }  
             
            /* Change state of QSPI */
            hqspi->State = HAL_QSPI_STATE_READY; 
             
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->RxCpltCallback(hqspi);
#else
    HAL_QSPI_RxCpltCallback(hqspi);
#endif               
        } 
        
        /* QSPI Small capacity RXFIFO non-empty interrupt occurred ----------------------------------*/
        if(((flag & QSPI_FLAG_SRFNEF) != 0U) && ((itsource & QSPI_IT_SRFNEF) != 0U))
        {
             while(__HAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_SRFNEF) != RESET)
             {  
                 /* Disable the QSPI Small capacity RXFIFO non-empty Interrupt */
                 __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_SRFNEF);
                 
                 /* Clear interrupt */
                __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_SRFNEF);
             }   
            /* Change state of QSPI */
            hqspi->State = HAL_QSPI_STATE_READY; 
             
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->RxNotEmptyCpltCallback(hqspi);
#else
    HAL_QSPI_RxNotEmptyCpltCallback(hqspi);
#endif                
        } 
        /* QSPI Receive overflow interrupt occurred ----------------------------------*/
        if(((flag & QSPI_FLAG_ROVF) != 0U) && ((itsource & QSPI_IT_ROVF) != 0U))
        {
             while(__HAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_ROVF) != RESET)
             {  
                 /* Disable the QSPI Small capacity RXFIFO full Interrupt */
                 __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_ROVF);
                 
                 /* Clear interrupt */
                __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_ROVF);
             }

            /* Change state of QSPI */
            hqspi->State = HAL_QSPI_STATE_READY; 
             
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->ReceiveFlowCallback(hqspi);
#else
    HAL_QSPI_ReceiveFlowCallback(hqspi);
#endif               
        }                
    }
    /* it's valid in indac mode or indac dma mode*/
    if((hqspi->Init.WorkMode == QSPI_WORKMODE_INDAC) || (hqspi->Init.WorkMode == QSPI_WORKMODE_INDACDMA))
    {
        /* QSPI The controller has completed the last indirect operation interrupt occurred -------------------------------*/
        if(((flag & QSPI_FLAG_INDCF) != 0U) && ((itsource & QSPI_IT_INDCF) != 0U))
        {
            /* Disable the QSPI controller has completed the last indirect operation Interrupt */
            __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_INDCF);
            
            /* Clear interrupt */
            __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_INDCF);      
                       
        } 
        /* QSPI Not received the indirect operation request interrupt occurred -------------------------------*/
        if(((flag & QSPI_FLAG_INDRRF) != 0U) && ((itsource & QSPI_IT_INDRRF) != 0U))
        {
            /* Disable the QSPI Not received the indirect operation request Interrupt */
            __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_INDRRF);
            
            /* Clear interrupt */
            __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_INDRRF);           
        }   
        /* QSPI Exceeds the indirect transmission depth threshold interrupt occurred -------------------------------*/
        if(((flag & QSPI_FLAG_INDTWF) != 0U) && ((itsource & QSPI_IT_INDTWF) != 0U))
        {
            /* Disable the QSPI Exceeds the indirect transmission depth threshold Interrupt */
            __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_INDTWF);
            
            /* Clear interrupt */
            __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_INDTWF);      
            
            /* Set error code */
            hqspi->ErrorCode |= HAL_QSPI_ERROR_TRANSFER;
           
            /* Change state of QSPI */
            hqspi->State = HAL_QSPI_STATE_READY;       
            
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->ErrorCallback(hqspi);
#else
    HAL_QSPI_ErrorCallback(hqspi);
#endif                       
        }       
        /* QSPI Exceeds the indirect transmission depth threshold interrupt occurred -------------------------------*/
        if(((flag & QSPI_FLAG_INDRSFF) != 0U) && ((itsource & QSPI_IT_INDRSFF) != 0U))
        {
            /* Disable the QSPI Exceeds the indirect transmission depth threshold Interrupt */
            __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_INDRSFF);
            
            /* Clear interrupt */
            __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_INDRSFF);    
            
            /* Set error code */
            hqspi->ErrorCode |= HAL_QSPI_ERROR_TRANSFER;
 
            /* Change state of QSPI */
            hqspi->State = HAL_QSPI_STATE_READY; 
            
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->ErrorCallback(hqspi);
#else
    HAL_QSPI_ErrorCallback(hqspi);
#endif              
        } 
        
        if(hqspi->Init.WorkMode == QSPI_WORKMODE_INDACDMA)
        {
            /* Disable the DMA channel */
            __HAL_DMA_DISABLE(hqspi->hdma_write);        
            /* Disable the DMA channel */
            __HAL_DMA_DISABLE(hqspi->hdma_read);               
        }         
        
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->IndacOperationCallback(hqspi);
#else
    HAL_QSPI_IndacOperationCallback(hqspi);
#endif           
    }        
    /* QSPI Check underflow interrupt occurred -------------------------------*/
    if(((flag & QSPI_FLAG_UDFF) != 0U) && ((itsource & QSPI_IT_UDFF) != 0U))
    {
        /* Disable the QSPI Check underflow Interrupt */
        __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_UDFF);
        
        /* Clear interrupt */
        __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_UDFF);    

        /* Change state of QSPI */
        hqspi->State = HAL_QSPI_STATE_READY; 
        
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->UnderFlowCallback(hqspi);
#else
    HAL_QSPI_UnderFlowCallback(hqspi);
#endif   
        
    }    
    
    /* QSPI Attempt to write protected area denied interrupt occurred -------------------------------*/
    if(((flag & QSPI_FLAG_WPAF) != 0U) && ((itsource & QSPI_IT_WPAF) != 0U))
    {
        /* Disable the QSPI Attempt to write protected area denied Interrupt */
        __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_WPAF);
        
        /* Clear interrupt */
        __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_WPAF);     

        /* Change state of QSPI */
        hqspi->State = HAL_QSPI_STATE_READY; 
        
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->AttemptToWriteProtectCallback(hqspi);
#else
    HAL_QSPI_AttemptToWriteProtectCallback(hqspi);
#endif          
    }     
    
    /* QSPI Illegal AHB access interrupt occurred -------------------------------*/
    if(((flag & QSPI_FLAG_AHBAEF) != 0U) && ((itsource & QSPI_IT_AHBAEF) != 0U))
    {
        /* Disable the QSPI Illegal AHB access Interrupt */
        __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_AHBAEF);
        
        /* Clear interrupt */
        __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_AHBAEF); 

        /* Change state of QSPI */
        hqspi->State = HAL_QSPI_STATE_READY; 
        
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->IllegalAHBDetectedCallback(hqspi);
#else
    HAL_QSPI_IllegalAHBDetectedCallback(hqspi);
#endif         
    }     

    /* QSPI Maximum number of polling cycles interrupt occurred -------------------------------*/
    if(((flag & QSPI_FLAG_POLLF) != 0U) && ((itsource & QSPI_IT_POLLF) != 0U))
    {
        /* Disable the QSPI Maximum number of polling cycles Interrupt */
        __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_POLLF);
        
        /* Clear interrupt */
        __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_POLLF);       
 
        /* Change state of QSPI */
        hqspi->State = HAL_QSPI_STATE_READY; 
        
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
    hqspi->MaxPollingCyclesCallback(hqspi);
#else
    HAL_QSPI_MaxPollingCyclesCallback(hqspi);
#endif          
    }   

   else
   {
        /* Nothing to do */
   }                                
}


/**
  * @brief Transmit an amount of data in blocking mode.
  * @param hqspi           QSPI handle
  * @param StartAddress    the start address of the transmit mode  
  * @param pData           pointer to data buffer
  * @param DataLen         amount of data to be sent
  * @param Timeout         Timeout duration
  * @note  This function is used only in Direct Write Modes
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_QSPI_DirectTransmit(QSPI_HandleTypeDef *hqspi, uint32_t StartAddress,uint8_t *pData, uint32_t DataLen, uint32_t Timeout)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t tickstart = HAL_GetTick();

    
    /* Process locked */
    __HAL_LOCK(hqspi);    
   
    /* Wait until IDLE flag is set to send data */
    status = QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_STATE_IDLE, SET, tickstart, Timeout);

    if (status == HAL_OK)
    {
        
    if(hqspi->State == HAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = HAL_QSPI_ERROR_NONE;
              
        /* Update state */
        hqspi->State = HAL_QSPI_STATE_BUSY_TX;
        
        /* Set the transaction information */
        hqspi->pTxBuffPtr  = (uint8_t *)pData;       
        hqspi->TxXAddress  = StartAddress;    
        hqspi->TxXferCount = DataLen;
        
        /*Init field not used in handle to zero */
        hqspi->pRxBuffPtr  = (uint8_t *)NULL;
        hqspi->RxXAddress  = 0U;
        hqspi->RxXferCount = 0U;

        while(hqspi->TxXferCount > 0U)
        {     
            /* Transmission in dac mode */
            if((hqspi->Init.WorkMode == QSPI_WORKMODE_DAC) && (pData != NULL))
            {
                if(hqspi->DataSize ==  QSPI_DATASIZE_32BIT)
                {
                    /* 32bit communication mode */
                    *(volatile uint32_t*)hqspi->TxXAddress = *((uint32_t *)hqspi->pTxBuffPtr);
                    hqspi->pTxBuffPtr += sizeof(uint32_t);
                    hqspi->TxXAddress = ((hqspi->TxXAddress) + sizeof(uint32_t));
                }  
                
                else if(hqspi->DataSize ==  QSPI_DATASIZE_16BIT)
                {
                    /* 16bit communication mode */
                   *(volatile uint16_t*)hqspi->TxXAddress=*((uint16_t *)hqspi->pTxBuffPtr);
                    hqspi->pTxBuffPtr += sizeof(uint16_t);
                    hqspi->TxXAddress = ((hqspi->TxXAddress) + sizeof(uint16_t));
                } 
                
                else
                {
                    /* 8bit communication mode */
                    (*(volatile uint8_t*)hqspi->TxXAddress)=*((uint8_t *)hqspi->pTxBuffPtr);                                     
                    hqspi->pTxBuffPtr += sizeof(uint8_t);
                    hqspi->TxXAddress = ((hqspi->TxXAddress) + sizeof(uint8_t));

                } 
            }
            else
            {
                hqspi->ErrorCode |= HAL_QSPI_ERROR_TRANSFER;
                status = HAL_ERROR;
                return status;
            }
            hqspi->TxXferCount--;                 
        }
    
        /* Update QSPI state */
        hqspi->State = HAL_QSPI_STATE_READY;
                   
    }
    else
    {
        status = HAL_BUSY;
    }   
 
    }
    else
    {
        return status;  
    }    
    
    /* Process unlocked */
    __HAL_UNLOCK(hqspi);

    return status;    
     
}


/**
  * @brief Receive an amount of data in blocking mode.
  * @param hqspi           QSPI handle
  * @param StartAddress    the start address of the transmit mode  
  * @param pData           pointer to data buffer
  * @param DataLen         amount of data to be sent
  * @param Timeout         Timeout duration
  * @note  This function is used only in Direct Read Modes.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_QSPI_DirectReceive(QSPI_HandleTypeDef *hqspi, uint32_t StartAddress,uint8_t *pData, uint32_t DataLen, uint32_t Timeout)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t tickstart = HAL_GetTick();
    
    /* Process locked */
    __HAL_LOCK(hqspi);    
        
    if(hqspi->State == HAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = HAL_QSPI_ERROR_NONE;
            
        /* Set the transaction information */
        hqspi->State       = HAL_QSPI_STATE_BUSY_RX;
        hqspi->ErrorCode   = HAL_QSPI_ERROR_NONE;
        hqspi->pRxBuffPtr  = (uint8_t *)pData;
        hqspi->RxXAddress  = StartAddress;
        hqspi->RxXferCount = DataLen;
        
        /*Init field not used in handle to zero */
        hqspi->pTxBuffPtr  = (uint8_t *)NULL;
        hqspi->TxXferCount = 0U;    
            
        while(hqspi->RxXferCount > 0U)
        {
            
            if(hqspi->Init.WorkMode == QSPI_WORKMODE_DAC)
            {
                if(hqspi->DataSize ==  QSPI_DATASIZE_32BIT)
                {
                      *((uint32_t *)hqspi->pRxBuffPtr) = (*(volatile uint32_t*)(hqspi->RxXAddress)) ;
                      hqspi->pRxBuffPtr += sizeof(uint32_t);
                      hqspi->RxXAddress = ((hqspi->RxXAddress) + sizeof(uint32_t));
                }  
                
                else if(hqspi->DataSize ==  QSPI_DATASIZE_16BIT)
                {
                      *((uint16_t *)hqspi->pRxBuffPtr) = (*(volatile uint16_t*)(hqspi->RxXAddress)) ;
                      hqspi->pRxBuffPtr += sizeof(uint16_t);
                      hqspi->RxXAddress = ((hqspi->RxXAddress) + sizeof(uint16_t));
                } 
                
                else
                {
                      *((uint8_t *)hqspi->pRxBuffPtr) = (*(volatile uint8_t*)(hqspi->RxXAddress)) ;
                      hqspi->pRxBuffPtr += sizeof(uint8_t);
                      hqspi->RxXAddress = ((hqspi->RxXAddress) + sizeof(uint8_t));
                } 
            }            
            else
            {
                hqspi->ErrorCode |= HAL_QSPI_ERROR_RECEIVE;
                status = HAL_ERROR;
                return status;
            }
            hqspi->RxXferCount--;      
        }
           
    }    
    else
    {
        status = HAL_BUSY;
        return status;
    }  
    
    /* Wait until IDLE flag is set to send data */
    status = QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_STATE_IDLE, SET, tickstart, Timeout);
    
    if (status != HAL_OK)
    {
        return status;  
    }  
    
    /* Process unlocked */
    __HAL_UNLOCK(hqspi);
    
    return status;

    
}

/**
  * @brief Set the command configuration.
  * @param hqspi    QSPI handle
  * @param Address  Address to be set
  * @param Timeout  Timeout duration
  * @note   This function is only valid in stig mode, and is used to set commands separately.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_QSPI_Command(QSPI_HandleTypeDef *hqspi, uint32_t Address, uint32_t Data, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();    
    __IO uint32_t tempg=0;
    
    /* Process locked */
    __HAL_LOCK(hqspi);    
    
    if(hqspi->State == HAL_QSPI_STATE_READY)
    { 
        hqspi->ErrorCode = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        hqspi->State = HAL_QSPI_STATE_BUSY; 
        
        // if(Address == 0)
        // {
        //     hqspi->TxXAddress  = 0;            
        // }
        // else
        {
            hqspi->TxXAddress  = Address; 
        }            
        
        /*Init field not used in handle to zero */
        hqspi->pRxBuffPtr  = (uint8_t *)NULL;
        hqspi->RxXAddress  = 0U;
        hqspi->RxXferCount = 0U;
        
        /* Wait till BUSY flag reset */
        status = QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_STATE_IDLE, SET, tickstart, Timeout);
        
        if (status == HAL_OK)
        {               
            /* Transmission in stig mode */

           hqspi->Instance->FCAR = hqspi->TxXAddress;
           if(Data != 0)
           {
                hqspi->Instance->FCWLR = Data;                               
           }

           tempg = 0;
           tempg = (hqspi->Command.AddressByte | (hqspi->Command.Instruction << 24) | hqspi->Command.AddressEn \
                   | hqspi->Command.ModeBitEn | hqspi->Command.WriteDataNum | (hqspi->Command.DummyCycles << 7) \
                   | hqspi->Command.WriteEn  | hqspi->Command.ReadEn | hqspi->Command.ReadDataNum | QSPI_FCR_CMDT);                        
                               
           WRITE_REG(hqspi->Instance->FCR , tempg);
            
          /* QSPI wait command run idle */  
           __HAL_QSPI_WAIT_COMMAND_IDLE(hqspi);         
                        
        }  
        else
        {
            hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
            status = HAL_ERROR;
        }   
        
        /* Update QSPI state */
        hqspi->State = HAL_QSPI_STATE_READY;        
        
    }
    else
    {
        status = HAL_BUSY;
    } 
    
    /* Process unlocked */
    __HAL_UNLOCK(hqspi);
  
    /* Initialize structural member variable to 0. */
    memset(&hqspi->Command, 0, sizeof(QSPI_CommandTypeDef)); 

    /* Return function status */
    return status;
    
}


/**
  * @brief  Send an amount of data in non-blocking mode with DMA.
  * @param  hqspi           QSPI handle
  * @param  pData           pointer to data buffer
  * @param  DataLen         amount of data to be sents
  * @param  Timeout         Duration of the timeout
  * @note   This function is used only in Indirect Write Modes and the DataSize is QSPI_DATASIZE_32BIT
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_QSPI_IndirectTransmit(QSPI_HandleTypeDef *hqspi,uint8_t *pData, uint32_t DataLen, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();

    /* Process locked */
    __HAL_LOCK(hqspi);    

    if(hqspi->State == HAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = HAL_QSPI_ERROR_NONE;
        
        if(pData != NULL)
        {    
            /* Update state */
            hqspi->State = HAL_QSPI_STATE_BUSY_TX;
            
            /* Set the transaction information */
            hqspi->pTxBuffPtr  = (uint8_t *)pData;
            hqspi->TxXAddress  = hqspi->IndacMode.WriteAddrStart;    
            hqspi->TxXferCount = DataLen;
        
            /*Init field not used in handle to zero */
            hqspi->pRxBuffPtr  = (uint8_t *)NULL;
            hqspi->RxXAddress  = 0U;
            hqspi->RxXferCount = 0U;            
            
            if(hqspi->Init.WorkMode == QSPI_WORKMODE_INDAC)
            {
                /* Indirect write settings */
                hqspi->Instance->IWTSAR = hqspi->IndacMode.WriteAddrStart;
                hqspi->Instance->IWTNR  = hqspi->IndacMode.WriteNum;
                hqspi->Instance->IATR   = hqspi->IndacMode.WriteTrigAddress;
                hqspi->Instance->ITARR  = hqspi->IndacMode.WriteTrigAddressRange;
           
                /* Start writing indirectly */
                hqspi->Instance->IWTR |= QSPI_IWTR_WR_ST;                
            }   
            else
            {
                hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
                status = HAL_ERROR;
                return status;
            }  
        
            /* Wait till BUSY flag reset */
            status = QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_STATE_IDLE, SET, tickstart, Timeout);
            if(status == HAL_OK)
            {                
                while(hqspi->TxXferCount > 0U)           
                {  
                    /* Transmission in indac mode */
                    if(hqspi->DataSize ==  QSPI_DATASIZE_32BIT)
                    {  
                        *(volatile uint32_t*)(hqspi->IndacMode.WriteTrigAddress) = *((uint32_t *)hqspi->pTxBuffPtr);                    
                        hqspi->pTxBuffPtr += sizeof(uint32_t);
                        hqspi->IndacMode.WriteTrigAddress = ((hqspi->IndacMode.WriteTrigAddress) + sizeof(uint32_t));                        
                    }
                    else
                    {
                        hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
                        status = HAL_ERROR;
                        return status;
                    }                
                    hqspi->TxXferCount--;                   
                }                       
                
                /* wait for the qpspi indac write completely */
                HAL_QSPI_WaitIndacWriteCmpl(hqspi);
            }
            else
            {
                hqspi->ErrorCode |= HAL_QSPI_ERROR_TIMEOUT;
            }             
        } 
        else
        {
            hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
            status = HAL_ERROR;
        }        
    }
    
    else
    {
        status = HAL_BUSY;
    }    

    /* Process unlocked */
    __HAL_UNLOCK(hqspi);
    
    /* Update QSPI state */
    hqspi->State = HAL_QSPI_STATE_READY;   
      
    /* Return function status */
    return status;    
    
}
/**
  * @brief  Send an amount of data in non-blocking mode with DMA.
  * @param  hqspi           QSPI handle  
  * @param  pData           pointer to data buffer
  * @param  DataLen         amount of data to be sents
  * @param  Timeout         Duration of the timeout
  * @note   This function is used only in Indirect Read Modes and the DataSize is QSPI_DATASIZE_32BIT.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_QSPI_IndirectReceive(QSPI_HandleTypeDef *hqspi,uint8_t *pData, uint32_t DataLen, uint32_t Timeout)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = HAL_GetTick();
  
    /* Process locked */
    __HAL_LOCK(hqspi);   
    
    if(hqspi->State == HAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = HAL_QSPI_ERROR_NONE;
             
        if(pData != NULL )
        { 
            /* Set the transaction information */
            hqspi->State       = HAL_QSPI_STATE_BUSY_RX;
            hqspi->ErrorCode   = HAL_QSPI_ERROR_NONE;
            hqspi->pRxBuffPtr  = (uint8_t *)pData;
            hqspi->RxXAddress  = hqspi->IndacMode.ReadAddrStart;
            hqspi->RxXferCount = DataLen;
            
            /*Init field not used in handle to zero */
            hqspi->pTxBuffPtr  = (uint8_t *)NULL;
            hqspi->TxXferCount = 0U;       
  
            /* Indirect write settings */
            hqspi->Instance->IRTSAR = hqspi->IndacMode.ReadAddrStart;
            hqspi->Instance->IRTNR  = hqspi->IndacMode.ReadNum;
            hqspi->Instance->IATR   = hqspi->IndacMode.ReadTrigAddress;
            hqspi->Instance->ITARR  = hqspi->IndacMode.ReadTrigAddressRange;
                          
            /* Start writing indirectly */
            hqspi->Instance->IRTR |= QSPI_IRTR_RDST;                
            /* Wait till BUSY flag reset */
            status = QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_STATE_IDLE, SET, tickstart, Timeout);
            if(status == HAL_OK)
            {
                while(hqspi->RxXferCount > 0U)
                { 
                    if(hqspi->Init.WorkMode == QSPI_WORKMODE_INDAC)
                    {
                        if(hqspi->DataSize ==  QSPI_DATASIZE_32BIT)
                        {  
                            (*(uint32_t *)hqspi->pRxBuffPtr) = (*(volatile uint32_t*)hqspi->IndacMode.ReadTrigAddress);     
                            hqspi->pRxBuffPtr += sizeof(uint32_t);
                            hqspi->IndacMode.ReadTrigAddress = ((hqspi->IndacMode.ReadTrigAddress) + sizeof(uint32_t));                        
                        }
                        else
                        {
                            hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
                            status = HAL_ERROR;
                            return status;
                        }    
                    }
                    else
                    {
                        hqspi->ErrorCode |= HAL_QSPI_ERROR_RECEIVE;
                        status = HAL_ERROR;
                        return status;
                    }                    
    
    
                    hqspi->RxXferCount--;                     
                }   
                
                /* wait for the qpspi indac read completely */
                HAL_QSPI_WaitIndacReadCmpl(hqspi);
            }
            else
            {
                hqspi->ErrorCode |= HAL_QSPI_ERROR_TIMEOUT;
                status = HAL_BUSY;              
            }
            /* Update QSPI state */
            hqspi->State = HAL_QSPI_STATE_READY;               
        }
        else
        {
            hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
            status = HAL_ERROR;            
        }        
    }   
    else
    {
        status = HAL_BUSY;
    }     
    /* Process unlocked */
    __HAL_UNLOCK(hqspi);
     
    /* Return function status */
    return status;     
}



/**
  * @brief  Send an amount of data in non-blocking mode with DMA.
  * @param  hqspi           QSPI handle
  * @param  StartAddress    the start address of the transmit mode  
  * @param  pData           pointer to data buffer
  * @param  DataLen         amount of data to be sents
  * @param  Timeout         Duration of the timeout
  * @note   This function is used only in Indirect Write DMA Mode and only supports the DataSize of 32bit.
  * @note   If DMA peripheral access is configured as halfword, the number
  *         of data and the fifo threshold should be aligned on halfword
  * @note   If DMA peripheral access is configured as word, the number
  *         of data and the fifo threshold should be aligned on word
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_QSPI_Transmit_DMA(QSPI_HandleTypeDef *hqspi, uint8_t *pData,uint32_t DataLen ,uint32_t Timeout)
{                                                                                                   
    HAL_StatusTypeDef status = HAL_OK;   
    uint32_t tickstart = HAL_GetTick();    
    __IO uint32_t tempg=0;
    
    /* Process locked */
    __HAL_LOCK(hqspi);

    if(hqspi->State == HAL_QSPI_STATE_READY)
    {
        /* Clear the error code */
        hqspi->ErrorCode = HAL_QSPI_ERROR_NONE;       
        
        /* Update state */
        hqspi->State = HAL_QSPI_STATE_BUSY_TX;
        
        /* Clear interrupt */
        __HAL_QSPI_CLEAR_FLAG(hqspi, (QSPI_FLAG_INDRRF | QSPI_FLAG_INDCF));
        
        if(pData != NULL )
        {
            /* Set the transaction information */
            hqspi->pTxBuffPtr  = (uint8_t *)pData;
            hqspi->TxXAddress  = hqspi->IndacMode.WriteAddrStart;    
            hqspi->TxXferCount = DataLen;
          
            /*Init field not used in handle to zero */
            hqspi->pRxBuffPtr  = (uint8_t *)NULL;
            hqspi->RxXAddress  = 0U;
            hqspi->RxXferCount = 0U;
            /* Wait till BUSY flag reset */
            status = QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_STATE_IDLE, SET, tickstart, Timeout);
            if(status == HAL_OK)
            {             
                if((hqspi->Init.WorkMode == QSPI_WORKMODE_INDACDMA) && (hqspi->DataSize == QSPI_DATASIZE_32BIT))
                {
                    
                    /* Indirect write settings */
                    hqspi->Instance->IWTSAR = hqspi->IndacMode.WriteAddrStart;
                    hqspi->Instance->IWTNR  = hqspi->IndacMode.WriteNum;
                    hqspi->Instance->IATR   = hqspi->IndacMode.WriteTrigAddress;
                    hqspi->Instance->ITARR  = hqspi->IndacMode.WriteTrigAddressRange;
                
                    /* Setting the QSPI_DMACR register */
                    tempg = 0;
                    /* Number of bytes of Burst and Single type in DMA request */
                    tempg = ((hqspi->IndacMode.DMA_BurstBytes<<8) | hqspi->IndacMode.DMA_SingleBytes);
                    WRITE_REG(hqspi->Instance->DMACR,tempg);
                        
                    /* Set the SPI TxDMA transfer complete callback */
                    hqspi->hdma_write->XferBlockCallback = QSPI_DMATxCplt;
                        
                    /* Set the DMA error callback */
                    hqspi->hdma_write->XferErrorCallback = QSPI_DMAError;                    
            
                }
                else
                {
                    hqspi->ErrorCode |=HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                    return status;
                }
            }
            else
            {
                 hqspi->ErrorCode |=HAL_QSPI_ERROR_TIMEOUT;
                 status = HAL_BUSY;
                 return status;            
            }
            /* Enable the QSPI transmit DMA Channel */
            if (HAL_DMA_Start_IT(hqspi->hdma_write, (uint32_t)hqspi->pTxBuffPtr, (uint32_t)(hqspi->IndacMode.WriteTrigAddress), hqspi->TxXferCount) == HAL_OK)
            {
                /* Process unlocked */
                __HAL_UNLOCK(hqspi);
        
                /* Enable the QSPI transfer error Interrupt */
                __HAL_QSPI_ENABLE_IT(hqspi, QSPI_IT_INDCF | QSPI_IT_INDRRF);
        
                /* Enable the DMA transfer by setting the DMAEN bit in the QSPI CR register */
                SET_BIT(hqspi->Instance->CR, QSPI_CR_DMA_EN);
               
                /* Start writing indirectly */
                hqspi->Instance->IWTR |= QSPI_IWTR_WR_ST;      

                /* wait for the qpspi indac write completely */
                HAL_QSPI_WaitIndacWriteCmpl(hqspi);                 
            }
            else
            {
                status = HAL_ERROR;
                hqspi->ErrorCode |= HAL_QSPI_ERROR_DMA;
                hqspi->State = HAL_QSPI_STATE_READY;
        
                /* Process unlocked */
                __HAL_UNLOCK(hqspi);
            }               
        }        
        else
        {
            hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
            status = HAL_ERROR;

            /* Process unlocked */
            __HAL_UNLOCK(hqspi);
        }        
    } 
    else
    {
        status = HAL_BUSY;
    }   

    /* Process unlocked */
    __HAL_UNLOCK(hqspi);  
    
    return status;
    
}

/**
  * @brief  Receive an amount of data in non-blocking mode with DMA.
  * @param  hqspi           QSPI handle
  * @param  StartAddress    the start address of the transmit mode  
  * @param  pData           pointer to data buffer
  * @param  DataLen         amount of data to be sent
  * @note   This function is used only in Indirect Read DMA Mode and only supports the DataSize of 32bit.
  * @note   If DMA peripheral access is configured as halfword, the number
  *         of data and the fifo threshold should be aligned on halfword
  * @note   If DMA peripheral access is configured as word, the number
  *         of data and the fifo threshold should be aligned on word
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_QSPI_Receive_DMA(QSPI_HandleTypeDef *hqspi, uint8_t *pData,uint32_t DataLen, uint32_t Timeout)
{
    HAL_StatusTypeDef status = HAL_OK;  
    uint32_t tickstart = HAL_GetTick();     
    __IO uint32_t tempg = 0;
    
    /* Process locked */
    __HAL_LOCK(hqspi);    
    
    if(hqspi->State == HAL_QSPI_STATE_READY)
    {
        /* Clear the error code */
        hqspi->ErrorCode = HAL_QSPI_ERROR_NONE;
        
        /* Update state */
        hqspi->State = HAL_QSPI_STATE_BUSY_RX;
        
        if(pData != NULL )
        {   
            /* Set the transaction information */
            hqspi->pTxBuffPtr  = (uint8_t *)NULL;
            hqspi->TxXAddress  = 0U;    
            hqspi->TxXferCount = 0U;
          
            /*Init field not used in handle to zero */
            hqspi->pRxBuffPtr  = (uint8_t *)pData;
            hqspi->RxXAddress  = hqspi->IndacMode.ReadAddrStart; ;
            hqspi->RxXferCount = DataLen;
            

            /* Wait till BUSY flag reset */
            status = QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_STATE_IDLE, SET, tickstart, Timeout);
            if(status == HAL_OK)
            {            
                /* Configure in the indac dma mode*/
                if((hqspi->Init.WorkMode == QSPI_WORKMODE_INDACDMA) && (hqspi->DataSize == QSPI_DATASIZE_32BIT))
                {
                    /* Indirect write settings */
                    hqspi->Instance->IRTSAR = hqspi->IndacMode.ReadAddrStart;
                    hqspi->Instance->IRTNR  = hqspi->IndacMode.ReadNum;
                    hqspi->Instance->IATR   = hqspi->IndacMode.ReadTrigAddress;
                    hqspi->Instance->ITARR  = hqspi->IndacMode.ReadTrigAddressRange;
                
                    /* Setting the QSPI_DMACR register */
                    tempg = 0;
                    /* Number of bytes of Burst and Single type in DMA request */
                    tempg = ((hqspi->IndacMode.DMA_BurstBytes<<8) | hqspi->IndacMode.DMA_SingleBytes);
                    WRITE_REG(hqspi->Instance->DMACR,tempg);
                    
                }
                else
                {
                    hqspi->ErrorCode |=HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                    return status;            
                }
            }
            else
            {
                hqspi->ErrorCode |=HAL_QSPI_ERROR_TIMEOUT;
                status = HAL_BUSY;
                return status;              
            }
            /* Clear interrupt */
            __HAL_QSPI_CLEAR_FLAG(hqspi, (QSPI_FLAG_INDCF | QSPI_FLAG_INDRRF));            
            
            /* Set the QSPI DMA transfer complete callback */
            hqspi->hdma_read->XferBlockCallback = QSPI_DMARxCplt;
            
            /* Set the DMA error callback */
            hqspi->hdma_read->XferErrorCallback = QSPI_DMAError;
            
            /* Enable the DMA Channel */
            if(HAL_DMA_Start_IT(hqspi->hdma_read, (uint32_t)hqspi->IndacMode.ReadTrigAddress, (uint32_t)pData, hqspi->RxXferCount) == HAL_OK)
            {
                /* Enable the DMA transfer by setting the DMAEN bit in the QSPI CR register */
                SET_BIT(hqspi->Instance->CR, QSPI_CR_DMA_EN);
    
                /* Process unlocked */
                __HAL_UNLOCK(hqspi);
    
                /* Enable the QSPI transfer error Interrupt */
                __HAL_QSPI_ENABLE_IT(hqspi, QSPI_IT_INDRSFF);
                
                /* wait for the qpspi indac read start  */
                hqspi->Instance->IRTR |= QSPI_IRTR_RDST;
                
                /* wait for the qpspi indac read completely */
                HAL_QSPI_WaitIndacReadCmpl(hqspi);                 
            }
            else
            {
                status = HAL_ERROR;
                hqspi->ErrorCode |= HAL_QSPI_ERROR_DMA;
                hqspi->State = HAL_QSPI_STATE_READY;
    
                /* Process unlocked */
                __HAL_UNLOCK(hqspi);
            } 
                        
        }
        else
        {
            hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
            status = HAL_ERROR;
    
            /* Process unlocked */
            __HAL_UNLOCK(hqspi);
        }        
    } 
    else
    {
        status = HAL_BUSY;

        /* Process unlocked */
        __HAL_UNLOCK(hqspi);
    }
    
    return status;    
}

/**
 * @brief  wait for the qpspi indac write completely 
 * @param  hqspi QSPI handle		
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_QSPI_WaitIndacWriteCmpl(QSPI_HandleTypeDef *hqspi)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t tickstart = HAL_GetTick();
    uint32_t Timeout   = 2000U;
    
    /* Process locked */
    __HAL_LOCK(hqspi); 

    /* This bit is set when the number of bytes set for the 
    completion of the indirect operation is fully written */   
    while(!(hqspi->Instance->IWTR & QSPI_IWTR_WRCS))
    {
        if((HAL_GetTick() - tickstart) > Timeout)
        {
            hqspi->State      = HAL_QSPI_STATE_ERROR;
            hqspi->ErrorCode |= HAL_QSPI_ERROR_TIMEOUT;
            status = HAL_ERROR;
            
            return status;
        }
    }
    
    /* Clear the status of indirect read transmission completion */
    hqspi->Instance->IWTR |= QSPI_IWTR_WRCS;

    return status;    
}

/**
 * @brief  wait for the qpspi indac read completely 
 * @param  hqspi QSPI handle		
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_QSPI_WaitIndacReadCmpl(QSPI_HandleTypeDef *hqspi)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t tickstart = HAL_GetTick();
    
    /*  Delay two seconds for timeout */
    uint32_t Timeout   = 2000U;
    
    /* Process locked */
    __HAL_LOCK(hqspi); 
    
    /* This bit is set when the number of bytes set for the 
    completion of the indirect operation is fully written */   
    while(!(hqspi->Instance->IRTR & QSPI_IRTR_RDCS))
    {
        if((HAL_GetTick() - tickstart) > Timeout)
        {
            hqspi->State      = HAL_QSPI_STATE_ERROR;
            hqspi->ErrorCode |= HAL_QSPI_ERROR_TIMEOUT;
            status = HAL_ERROR;
            
            return status;
        }        

    }
    
    /* Clear the status of indirect read transmission completion */
    hqspi->Instance->IRTR |= QSPI_IRTR_RDCS;
    
    return status;     
}


/**
  * @brief  Transfer Error callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_QSPI_ErrorCallback could be implemented in the user file
    */
}

/**
  * @brief  Abort completed callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_AbortCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_AbortCpltCallback could be implemented in the user file
    */
}

/**
  * @brief  Command completed callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_CmdCpltCallback could be implemented in the user file
    */
}

/**
  * @brief  Command underflow callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_UnderFlowCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_UnderFlowCallback could be implemented in the user file
    */
}

/**
  * @brief  Command indac mode operation callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_IndacOperationCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_IndacOperationCallback could be implemented in the user file
    */
}

/**
  * @brief  Command attempt to write protect operation callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_AttemptToWriteProtectCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_AttemptToWriteProtectCallback could be implemented in the user file
    */
}

/**
  * @brief  Command Illegal AHB access detected callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_IllegalAHBDetectedCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_IllegalAHBDetectedCallback could be implemented in the user file
    */
}

/**
  * @brief  Command receive flow callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_ReceiveFlowCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_ReceiveFlowCallback could be implemented in the user file
    */
}

/**
  * @brief  Command maximum number of polling cycles callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_MaxPollingCyclesCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_MaxPollingCyclesCallback could be implemented in the user file
    */
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_RxCpltCallback could be implemented in the user file
    */
}


/**
  * @brief  Tx Transfer completed callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_TxCpltCallback could be implemented in the user file
    */
}


/**
  * @brief  Rx Half Transfer completed callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_RxNotEmptyCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_RxNotEmptyCpltCallback could be implemented in the user file
    */
}

/**
  * @brief  Tx Half Transfer completed callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_TxNotFullCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_QSPI_TxNotFullCpltCallback could be implemented in the user file
    */
}

/**
  * @brief  Timeout callback.
  * @param  hqspi QSPI handle
  * @retval None
  */
__weak void HAL_QSPI_TimeOutCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_QSPI_TimeOutCallback could be implemented in the user file
    */
}

#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User QSPI Callback
  *         To be used instead of the weak (surcharged) predefined callback
  * @param hqspi QSPI handle
  * @param CallbackId ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_QSPI_ERROR_CB_ID                   QSPI Error Callback ID            
  *          @arg @ref HAL_QSPI_ABORT_CB_ID                   QSPI Abort Callback ID           
  *          @arg @ref HAL_QSPI_CMD_CPLT_CB_ID                QSPI Command Complete Callback ID 
  *          @arg @ref HAL_QSPI_UNDERFOLW_CB_ID               QSPI Underflow Callback ID 
  *          @arg @ref HAL_QSPI_INDACOPERATION_CB_ID          QSPI Operation of the indac mode Callback ID 
  *          @arg @ref HAL_QSPI_ATTEMPTTOWRITEPROTECT_CB_ID   QSPI Attempted to write protection Callback ID 
  *          @arg @ref HAL_QSPI_ILLEGALAHBDETECTED_CB_ID      QSPI Illegal AHB access detected Callback ID 
  *          @arg @ref HAL_QSPI_RECEIVEFLOW_CB_ID             QSPI Receive overflow Callback ID
  *          @arg @ref HAL_QSPI_MAXPOLLINGCYCLES_CB_ID        QSPI Maximum number of polling cycles Callback ID 
  *          @arg @ref HAL_QSPI_RX_CPLT_CB_ID                 QSPI Rx Complete Callback ID     
  *          @arg @ref HAL_QSPI_TX_CPLT_CB_ID                 QSPI Tx Complete Callback ID    
  *          @arg @ref HAL_QSPI_RX_NOTEMPTY_CPLT_CB_ID        QSPI Small capacity RXFIFO is not empty Callback ID   
  *          @arg @ref HAL_QSPI_TX_NOTFULL_CPLT_CB_ID         QSPI Small capacity TXFIFO is not full Callback ID 
  *          @arg @ref HAL_QSPI_TIMEOUT_CB_ID                 QSPI Timeout Callback ID          
  *          @arg @ref HAL_QSPI_MSP_INIT_CB_ID                QSPI MspInit Callback ID          
  *          @arg @ref HAL_QSPI_MSP_DEINIT_CB_ID              QSPI MspDeInit Callback ID                                                                     
  * @param pCallback pointer to the Callback function
  * @retval status
  */
HAL_StatusTypeDef HAL_QSPI_RegisterCallback (QSPI_HandleTypeDef *hqspi, HAL_QSPI_CallbackIDTypeDef CallbackId, pQSPI_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    if(pCallback == NULL)
    {
        /* Update the error code */
        hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_CALLBACK;
        return HAL_ERROR;
    }
    
    /* Process locked */
    __HAL_LOCK(hqspi);

    if(hqspi->State == HAL_QSPI_STATE_READY)
    {
        switch (CallbackId)
        {
            case  HAL_QSPI_ERROR_CB_ID :
                hqspi->ErrorCallback = pCallback;
                break; 
            case  HAL_QSPI_ABORT_CB_ID :
                hqspi->AbortCpltCallback = pCallback;
                break;     
            case  HAL_QSPI_CMD_CPLT_CB_ID :
                hqspi->CmdCpltCallback = pCallback;
                break;  
            case  HAL_QSPI_UNDERFOLW_CB_ID :
                hqspi->UnderFlowCallback = pCallback;
                break;  
            case  HAL_QSPI_INDACOPERATION_CB_ID :
                hqspi->IndacOperationCallback = pCallback;
                break;       
            case  HAL_QSPI_ATTEMPTTOWRITEPROTECT_CB_ID :
                hqspi->AttemptToWriteProtectCallback = pCallback;
                break;        
            case  HAL_QSPI_ILLEGALAHBDETECTED_CB_ID :
                hqspi->IllegalAHBDetectedCallback = pCallback;
                break;    
            case  HAL_QSPI_RECEIVEFLOW_CB_ID :
                hqspi->ReceiveFlowCallback = pCallback;
                break;    
            case  HAL_QSPI_MAXPOLLINGCYCLES_CB_ID :
                hqspi->MaxPollingCyclesCallback = pCallback;
                break;            
            case HAL_QSPI_RX_CPLT_CB_ID :
                hqspi->RxCpltCallback = pCallback;
                break;
            case HAL_QSPI_TX_CPLT_CB_ID :
                hqspi->TxCpltCallback = pCallback;
                break;   
            case HAL_QSPI_RX_NOTEMPTY_CPLT_CB_ID :
                hqspi->RxNotEmptyCpltCallback = pCallback;
                break;     
            case HAL_QSPI_TX_NOTFULL_CPLT_CB_ID :
                hqspi->TxNotFullCpltCallback = pCallback;
                break;   
            case HAL_QSPI_TIMEOUT_CB_ID :
                hqspi->TimeOutCallback = pCallback;
                break;              
            case HAL_QSPI_MSP_INIT_CB_ID :
                hqspi->MspInitCallback = pCallback;
                break;
            case HAL_QSPI_MSP_DEINIT_CB_ID :
                hqspi->MspDeInitCallback = pCallback;
                break;            
            default :
                /* Update the error code */
                hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_CALLBACK;
                /* update return status */
                status =  HAL_ERROR;
                break;        
        }
    }
    else if (hqspi->State == HAL_QSPI_STATE_RESET)
    {
        switch (CallbackId)
            {
                case HAL_QSPI_MSP_INIT_CB_ID :
                    hqspi->MspInitCallback = pCallback;
                    break;
            case HAL_QSPI_MSP_DEINIT_CB_ID :
                    hqspi->MspDeInitCallback = pCallback;
                    break;
            default :
                /* Update the error code */
                hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_CALLBACK;
                /* update return status */
                status =  HAL_ERROR;
                break;
            }        
    }  
    else
    {
        /* Update the error code */
        hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_CALLBACK;
        /* update return status */
        status =  HAL_ERROR;
    } 
    
    /* Release Lock */
    __HAL_UNLOCK(hqspi);
    return status;    
}


/**
  * @brief  Unregister a User QSPI Callback
  *         QSPI Callback is redirected to the weak (surcharged) predefined callback
  * @param hqspi QSPI handle
  * @param CallbackId ID of the callback to be unregistered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_QSPI_ERROR_CB_ID                   QSPI Error Callback ID            
  *          @arg @ref HAL_QSPI_ABORT_CB_ID                   QSPI Abort Callback ID           
  *          @arg @ref HAL_QSPI_CMD_CPLT_CB_ID                QSPI Command Complete Callback ID 
  *          @arg @ref HAL_QSPI_UNDERFOLW_CB_ID               QSPI Underflow Callback ID 
  *          @arg @ref HAL_QSPI_INDACOPERATION_CB_ID          QSPI Operation of the indac mode Callback ID 
  *          @arg @ref HAL_QSPI_ATTEMPTTOWRITEPROTECT_CB_ID   QSPI Attempted to write protection Callback ID 
  *          @arg @ref HAL_QSPI_ILLEGALAHBDETECTED_CB_ID      QSPI Illegal AHB access detected Callback ID 
  *          @arg @ref HAL_QSPI_RECEIVEFLOW_CB_ID             QSPI Receive overflow Callback ID
  *          @arg @ref HAL_QSPI_MAXPOLLINGCYCLES_CB_ID        QSPI Maximum number of polling cycles Callback ID 
  *          @arg @ref HAL_QSPI_RX_CPLT_CB_ID                 QSPI Rx Complete Callback ID     
  *          @arg @ref HAL_QSPI_TX_CPLT_CB_ID                 QSPI Tx Complete Callback ID    
  *          @arg @ref HAL_QSPI_RX_NOTEMPTY_CPLT_CB_ID        QSPI Small capacity RXFIFO is not empty Callback ID   
  *          @arg @ref HAL_QSPI_TX_NOTFULL_CPLT_CB_ID         QSPI Small capacity TXFIFO is not full Callback ID 
  *          @arg @ref HAL_QSPI_TIMEOUT_CB_ID                 QSPI Timeout Callback ID          
  *          @arg @ref HAL_QSPI_MSP_INIT_CB_ID                QSPI MspInit Callback ID          
  *          @arg @ref HAL_QSPI_MSP_DEINIT_CB_ID              QSPI MspDeInit Callback ID                                                
  * @retval status
  */
HAL_StatusTypeDef HAL_QSPI_UnRegisterCallback (QSPI_HandleTypeDef *hqspi, HAL_QSPI_CallbackIDTypeDef CallbackId)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    /* Process locked */
    __HAL_LOCK(hqspi);
    
    if(hqspi->State == HAL_QSPI_STATE_READY)
    {
        switch (CallbackId)
        {
            case  HAL_QSPI_ERROR_CB_ID :
                hqspi->ErrorCallback = HAL_QSPI_ErrorCallback;
                break;
            case HAL_QSPI_ABORT_CB_ID :
                hqspi->AbortCpltCallback = HAL_QSPI_AbortCpltCallback;
                break;            
            case HAL_QSPI_CMD_CPLT_CB_ID :
                hqspi->CmdCpltCallback = HAL_QSPI_CmdCpltCallback;
                break;     
            case HAL_QSPI_UNDERFOLW_CB_ID :
                hqspi->UnderFlowCallback = HAL_QSPI_UnderFlowCallback;
                break;       
            case HAL_QSPI_INDACOPERATION_CB_ID :
                hqspi->IndacOperationCallback = HAL_QSPI_IndacOperationCallback;
                break;    
            case HAL_QSPI_ATTEMPTTOWRITEPROTECT_CB_ID :
                hqspi->AttemptToWriteProtectCallback = HAL_QSPI_AttemptToWriteProtectCallback;
                break;        
            case HAL_QSPI_ILLEGALAHBDETECTED_CB_ID :
                hqspi->IllegalAHBDetectedCallback = HAL_QSPI_IllegalAHBDetectedCallback;
                break; 
            case HAL_QSPI_RECEIVEFLOW_CB_ID :
                hqspi->ReceiveFlowCallback = HAL_QSPI_ReceiveFlowCallback;
                break;   
            case HAL_QSPI_MAXPOLLINGCYCLES_CB_ID :
                hqspi->MaxPollingCyclesCallback = HAL_QSPI_MaxPollingCyclesCallback;
                break;            
            case HAL_QSPI_RX_CPLT_CB_ID :
                hqspi->RxCpltCallback = HAL_QSPI_RxCpltCallback;
                break;
            case HAL_QSPI_TX_CPLT_CB_ID :
                hqspi->TxCpltCallback = HAL_QSPI_TxCpltCallback;
                break; 
            case HAL_QSPI_RX_NOTEMPTY_CPLT_CB_ID :
                hqspi->RxNotEmptyCpltCallback = HAL_QSPI_RxNotEmptyCpltCallback;
                break;  
            case HAL_QSPI_TX_NOTFULL_CPLT_CB_ID :
                hqspi->TxNotFullCpltCallback = HAL_QSPI_TxNotFullCpltCallback;
                break; 
            case HAL_QSPI_TIMEOUT_CB_ID :
                hqspi->TimeOutCallback = HAL_QSPI_TimeOutCallback;
                break;               
            case HAL_QSPI_MSP_INIT_CB_ID :
                hqspi->MspInitCallback = HAL_QSPI_MspInit;
                break;
            case HAL_QSPI_MSP_DEINIT_CB_ID :
                hqspi->MspDeInitCallback = HAL_QSPI_MspDeInit;
                break;            
            default :
                /* Update the error code */
                hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_CALLBACK;
                /* update return status */
                status =  HAL_ERROR;
                break;            
        }        
        
        
    } 
    else if (hqspi->State == HAL_QSPI_STATE_RESET)
    {
        switch (CallbackId)
        {
            case HAL_QSPI_MSP_INIT_CB_ID :
                hqspi->MspInitCallback = HAL_QSPI_MspInit;
                break;
            case HAL_QSPI_MSP_DEINIT_CB_ID :
                hqspi->MspDeInitCallback = HAL_QSPI_MspDeInit;
                break;
            default :
                /* Update the error code */
                hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_CALLBACK;
                /* update return status */
                status =  HAL_ERROR;
                break;
        }        
        
    }   
    else
    {
        /* Update the error code */
        hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_CALLBACK;
        /* update return status */
        status =  HAL_ERROR;
    }  
 
    /* Release Lock */
    __HAL_UNLOCK(hqspi);
    return status;    
}

#endif





/**
  * @}
  */


/** @defgroup QSPI_Exported_Functions_Group3 Peripheral Control and State functions
  *  @brief   QSPI control and State functions
  *
@verbatim
 ===============================================================================
                  ##### Peripheral Control and State functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to :
      (+) Check in run-time the state of the driver.
      (+) Check the error code set during last operation.
      (+) Abort any operation.


@endverbatim
  * @{
  */

/**
  * @brief  Return the QSPI handle state.
  * @param  hqspi QSPI handle
  * @retval HAL state
  */
HAL_QSPI_StateTypeDef HAL_QSPI_GetState(QSPI_HandleTypeDef *hqspi)
{
    /* Return QSPI handle state */
    return hqspi->State;
}

/**
* @brief  Return the QSPI error code.
* @param  hqspi QSPI handle
* @retval QSPI Error Code
*/
uint32_t HAL_QSPI_GetError(QSPI_HandleTypeDef *hqspi)
{
    return hqspi->ErrorCode;
}

/**
* @brief  Abort the current transmission (non-blocking function)
* @param  hqspi QSPI handle
* @retval HAL status
*/
HAL_StatusTypeDef HAL_QSPI_Abort_IT(QSPI_HandleTypeDef *hqspi)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    /* Check if the state is in one of the busy states */    
    if (((uint32_t)hqspi->State & 0x2U) != 0U)
    { 
        /* Process unlocked */
        __HAL_UNLOCK(hqspi);

        /* Update QSPI state */
        hqspi->State = HAL_QSPI_STATE_ABORT;

        /* Disable all interrupts */
        __HAL_QSPI_DISABLE_IT(hqspi, (QSPI_IT_POLLF | QSPI_IT_INDRSFF | QSPI_IT_SRFFF | QSPI_IT_SRFNEF \
        | QSPI_IT_STFFF | QSPI_IT_STFNFF | QSPI_IT_ROVF | QSPI_IT_INDTWF | QSPI_IT_AHBAEF | QSPI_IT_WPAF \
        | QSPI_IT_INDRRF | QSPI_IT_INDCF | QSPI_IT_UDFF));
 
        if ((hqspi->Instance->CR & QSPI_CR_DMA_EN) != 0U)
        {
            /* Disable the DMA transfer by clearing the DMAEN bit in the QSPI CR register */
            CLEAR_BIT(hqspi->Instance->CR, QSPI_CR_DMA_EN);

            /* Abort DMA channel */
            if ((HAL_DMA_Abort_IT(hqspi->hdma_write) != HAL_OK) || (HAL_DMA_Abort_IT(hqspi->hdma_read) != HAL_OK))
            {
                /* Change state of QSPI */
                hqspi->State = HAL_QSPI_STATE_READY;

                /* Abort Complete callback */
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1)
                hqspi->AbortCpltCallback(hqspi);
#else
                HAL_QSPI_AbortCpltCallback(hqspi);
#endif
            }        
        }
        else
        {
            if (__HAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_INDCF) != RESET)
            {
                /* Clear interrupt */
                __HAL_QSPI_CLEAR_FLAG(hqspi, QSPI_FLAG_INDCF);
            
                /* Enable the QSPI Transfer Complete Interrupt */
                __HAL_QSPI_ENABLE_IT(hqspi, QSPI_FLAG_INDCF);
            
            }    
            else
            {
                /* Change state of QSPI */
                hqspi->State = HAL_QSPI_STATE_READY;
            }
        }        
    }
    
    return status;    
}

/** @brief Set QSPI timeout.
  * @param  hqspi QSPI handle.
  * @param  Timeout Timeout for the QSPI memory access.
  * @retval None
  */
void HAL_QSPI_SetTimeout(QSPI_HandleTypeDef *hqspi, uint32_t Timeout)
{
    hqspi->Timeout = Timeout;
}


/**
  * @}
  */
  
  
/**
  * @}
  */


/** @defgroup QSPI_Private_Functions QSPI Private Functions
  * @{
  */
/**
  * @brief  DMA QSPI receive process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void QSPI_DMARxCplt(DMA_HandleTypeDef *hdma)
{
    QSPI_HandleTypeDef* hqspi = (QSPI_HandleTypeDef*)(((DMA_HandleTypeDef *)hdma)->Parent);

     /* DMA Normal Mode */
    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == RESET)
    {
        /* Disable ERR interrupt */
        __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_INDCF | QSPI_IT_INDRRF);
       
        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        __HAL_QSPI_CLEAR_FLAG(hqspi,QSPI_FLAG_INDRRF | QSPI_FLAG_INDCF);
        
        hqspi->RxXferCount = 0U;  
        hqspi->State = HAL_QSPI_STATE_READY;     
        if (hqspi->ErrorCode != HAL_QSPI_ERROR_NONE)
        {
        /* Call user error callback */
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1U)
      hspi->ErrorCallback(hspi);
#else
      HAL_QSPI_ErrorCallback(hqspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
      return;
        }         
    }        
 
}

/**
  * @brief  DMA QSPI transmit process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void QSPI_DMATxCplt(DMA_HandleTypeDef *hdma)
{
    QSPI_HandleTypeDef* hqspi = (QSPI_HandleTypeDef*)(((DMA_HandleTypeDef *)hdma)->Parent);
    
    /* DMA Normal Mode */
    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == RESET)
    {
        /* Disable ERR interrupt */
        __HAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_INDCF | QSPI_IT_INDRRF);
       
        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        __HAL_QSPI_CLEAR_FLAG(hqspi,QSPI_FLAG_INDRRF | QSPI_FLAG_INDCF);
        
        hqspi->TxXferCount = 0U;  
        hqspi->State = HAL_QSPI_STATE_READY;        
        if (hqspi->ErrorCode != HAL_QSPI_ERROR_NONE)
        {
        /* Call user error callback */
#if (USE_HAL_QSPI_REGISTER_CALLBACKS == 1U)
      hspi->ErrorCallback(hspi);
#else
      HAL_QSPI_ErrorCallback(hqspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
      return;
        }        
    }  
  
}


/**
  * @brief  DMA QSPI communication error callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void QSPI_DMAError(DMA_HandleTypeDef *hdma)
{
    QSPI_HandleTypeDef* hqspi = ( QSPI_HandleTypeDef* )(hdma->Parent);
    
    /* if DMA error is FIFO error ignore it */
    if(HAL_DMA_GetError(hdma) != HAL_DMA_ERROR_TE)
    {
        hqspi->RxXferCount = 0U;
        hqspi->TxXferCount = 0U;
        hqspi->ErrorCode   |= HAL_QSPI_ERROR_DMA;
        
        /* Disable the DMA transfer by clearing the DMAEN bit in the QSPI CR register */
        CLEAR_BIT(hqspi->Instance->CR, QSPI_CR_DMA_EN);
        
        /* Abort the QSPI */
        (void)HAL_QSPI_Abort_IT(hqspi);       
    }    
}

/**
  * @brief  Wait for a flag state until timeout.
  * @param  hqspi QSPI handle
  * @param  Flag Flag checked
  * @param  State Value of the flag expected
  * @param  Tickstart Tick start value
  * @param  Timeout Duration of the timeout
  * @retval HAL status
  */
static HAL_StatusTypeDef QSPI_WaitFlagStateUntilTimeout(QSPI_HandleTypeDef *hqspi, uint32_t Flag,
                                                        FlagStatus State, uint32_t Tickstart, uint32_t Timeout)
{
    /* Wait until flag is in expected state */
    while((__HAL_QSPI_GET_IDLE(hqspi, Flag)) != State)
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if(((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
            {
                hqspi->State     = HAL_QSPI_STATE_ERROR;
                hqspi->ErrorCode |= HAL_QSPI_ERROR_TIMEOUT;
        
                return HAL_ERROR;
            }
        }
    }
    
    return HAL_OK; 
}

/**
  * @brief  Configure the communication registers.
  * @param  hqspi QSPI handle
  * @retval None
  */
static void QSPI_Config(QSPI_HandleTypeDef *hqspi)
{
	uint32_t tmpreg=0 , tmpvariable =0;    					
   
    /* Configure the qspi in the dac mode */ 
    if(hqspi->Init.WorkMode == QSPI_WORKMODE_DAC)
    {

        /* Configure the DRIR register */ 
        tmpvariable |= (hqspi->DacMode.ReadCommand <<0);
        tmpvariable |= hqspi->DacMode.ReadAddr_Type;
        tmpvariable |= hqspi->DacMode.ReadData_Type;
        tmpvariable |= (hqspi->DacMode.ReadData_Dummy<<24);
  
        WRITE_REG(hqspi->Instance->DRIR,tmpvariable);   
        
        /* Set the default value to tmpvariable */        
        tmpvariable =0;        
        
        /* Configure the DWIR register */ 
        tmpvariable |= (hqspi->DacMode.WriteCommand <<0);
        tmpvariable |= hqspi->DacMode.WriteAddr_Type;
        tmpvariable |= hqspi->DacMode.WriteData_Type;
        tmpvariable |= (hqspi->DacMode.WriteData_Dummy<<24);        
        
        WRITE_REG(hqspi->Instance->DWIR,tmpvariable);    
        
		tmpreg |= QSPI_CR_DAC_EN;	 
        tmpreg |= hqspi->Init.AddressRemap;     
        tmpreg |= hqspi->Init.AHB_Decoder;          
        
    }
    /* Set the default value to tmpvariable */        
    tmpvariable =0; 
    
    /* Configure the qspi in the indac mode */  
    if((hqspi->Init.WorkMode & 0x10000000U))
    {
        /* Configure the qspi in the indac mode */      
        if(hqspi->Init.WorkMode == QSPI_WORKMODE_INDAC)
        {
            tmpreg &= ~(QSPI_CR_DAC_EN);    
        } 
    
        /* Configure the qspi in the indac dma mode */     
        if(hqspi->Init.WorkMode == QSPI_WORKMODE_INDACDMA)
        {
            tmpreg |= QSPI_CR_DMA_EN;				
            tmpreg &= ~(QSPI_CR_DAC_EN);	    
        } 
    }
    /* Configure the qspi in the standard mode */     
    if(hqspi->Init.WorkMode == QSPI_WORKMODE_STANDARD)
    {
		tmpreg |= QSPI_CR_LIM_EN;  				
		tmpreg |= QSPI_CR_DAC_EN;      
    }    
    
    tmpreg |= hqspi->Init.ClockPrescaler;	
    tmpreg |= hqspi->Init.ClockMode;    
    tmpreg |= hqspi->Init.WriteProtectPin;  
    tmpreg |= hqspi->Init.Enter_XIPIM;
    tmpreg |= hqspi->Init.Enter_XIPNext;
    tmpreg |= hqspi->Init.DTR_Protocol;  

    /* Configure the CR register */    
    WRITE_REG(hqspi->Instance->CR,tmpreg);
    
    /* Set the default value to tmpreg */        
    tmpreg =0;
    
    tmpreg |= (hqspi->Init.AddrSizes<<0);    
    tmpreg |= (hqspi->Init.PageSizes<<4);     
    tmpreg |= (hqspi->Init.BlockSizes<<16); 
    
    if(hqspi->Init.CSSizes != QSPI_CSSIZES_512M)
    {
        tmpreg |= (hqspi->Init.CSSizes<<21);        
    }
    else
    {
        tmpreg |= hqspi->Init.CSSizes;       
    }
    /* Configure the DSCR register */    
    WRITE_REG(hqspi->Instance->DSCR,tmpreg);   

    /* Set the default value to tmpreg */        
    tmpreg =0;

    tmpreg |= hqspi->Init.ReadDelay;    
    tmpreg |= hqspi->Init.TransDelay;     
    tmpreg |= hqspi->Init.Sampling_Edge;     
    
    /* Configure the RDCR register */    
    MODIFY_REG(hqspi->Instance->RDCR,0x0,tmpreg); 

    /* Set the default value to tmpreg */        
    tmpreg =0;

    tmpreg |= (hqspi->Init.CSStartDelay<<0);    
    tmpreg |= (hqspi->Init.CSStopDelay<<8);     
    tmpreg |= (hqspi->Init.CSInvalidDelay<<24);     
 
    /* Configure the DDLR register */    
    WRITE_REG(hqspi->Instance->DDLR,tmpreg);   
}

/**
  * @}
  */

#endif /* HAL_QSPI_MODULE_ENABLED */
/**
  * @}
  */
  
/**
  * @}
  */  
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
