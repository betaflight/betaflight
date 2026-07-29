/**
  ******************************************************************************
  * @file     um324xx_hal_i2c_ex.c 
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-08-04  
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

/** @addtogroup UM324xx_HAL_Examples
  * @{
  */

/** @defgroup I2C_EX I2C_EX
  * @brief I2C_EX HAL module driver
  * @{
  */

#ifdef HAL_I2C_EX_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup I2C_EX_Private_Define
  * @{
  */
#define I2C_EX_TIMEOUT_FLAG          35U         /*!< Timeout 35 ms             */
#define I2C_EX_TIMEOUT_BUSY_FLAG     25U         /*!< Timeout 25 ms             */
#define I2C_EX_TIMEOUT_STOP_FLAG     5U          /*!< Timeout 5 ms              */
#define I2C_EX_NO_OPTION_FRAME       0xFFFF0000U /*!< XferOptions default value */


#define MAX_NBYTE_SIZE      255U
#define SlaveAddr_SHIFT     7U
#define SlaveAddr_MSK       0x06U


/* Private define for @ref PreviousState usage */
#define I2C_EX_STATE_MSK             ((uint32_t)((uint32_t)((uint32_t)HAL_I2C_EX_STATE_BUSY_TX | (uint32_t)HAL_I2C_EX_STATE_BUSY_RX) & (uint32_t)(~((uint32_t)HAL_I2C_EX_STATE_READY)))) /*!< Mask State define, keep only RX and TX bits            */
#define I2C_EX_STATE_NONE            ((uint32_t)(HAL_I2C_EX_MODE_NONE))                                                        /*!< Default Value                                          */
#define I2C_EX_STATE_MASTER_BUSY_TX  ((uint32_t)(((uint32_t)HAL_I2C_EX_STATE_BUSY_TX & I2C_EX_STATE_MSK) | (uint32_t)HAL_I2C_EX_MODE_MASTER))            /*!< Master Busy TX, combinaison of State LSB and Mode enum */
#define I2C_EX_STATE_MASTER_BUSY_RX  ((uint32_t)(((uint32_t)HAL_I2C_EX_STATE_BUSY_RX & I2C_EX_STATE_MSK) | (uint32_t)HAL_I2C_EX_MODE_MASTER))            /*!< Master Busy RX, combinaison of State LSB and Mode enum */
#define I2C_EX_STATE_SLAVE_BUSY_TX   ((uint32_t)(((uint32_t)HAL_I2C_EX_STATE_BUSY_TX & I2C_EX_STATE_MSK) | (uint32_t)HAL_I2C_EX_MODE_SLAVE))             /*!< Slave Busy TX, combinaison of State LSB and Mode enum  */
#define I2C_EX_STATE_SLAVE_BUSY_RX   ((uint32_t)(((uint32_t)HAL_I2C_EX_STATE_BUSY_RX & I2C_EX_STATE_MSK) | (uint32_t)HAL_I2C_EX_MODE_SLAVE))             /*!< Slave Busy RX, combinaison of State LSB and Mode enum  */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @defgroup I2C_EX_Private_Functions I2C_EX Private Functions
  * @{
  */
/* Private functions to handle DMA transfer */

/* Private functions for I2C transfer IRQ handler */
static HAL_StatusTypeDef I2C_EX_Master_ISR_IT(struct __I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef I2C_EX_Slave_ISR_IT(struct __I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef I2C_EX_Master_ISR_DMA(struct __I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef I2C_EX_Slave_ISR_DMA(struct __I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);

/* Private functions to handle flags during polling transfer */
static HAL_StatusTypeDef I2C_EX_WaitOnRawITFlagUntilTimeout(I2C_EX_HandleTypeDef *hi2c_ex, uint32_t RawITFlag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_EX_WaitOnFlagUntilTimeout(I2C_EX_HandleTypeDef *hi2c_ex, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
/* Private functions to handle IT transfer */
static void I2C_EX_ITMasterCplt(I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_EX_ITSlaveCplt(I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags);
static void I2C_EX_ITMasterSeqCplt(I2C_EX_HandleTypeDef *hi2c);
static void I2C_EX_ITSlaveSeqCplt(I2C_EX_HandleTypeDef *hi2c);
/* Private functions to handle DMA transfer */

static void I2C_EX_DMASlaveTransmitCplt(DMA_HandleTypeDef *hdma);
static void I2C_EX_DMASlaveReceiveCplt(DMA_HandleTypeDef *hdma);
static void I2C_EX_DMAMasterTransmitCplt(DMA_HandleTypeDef *hdma);
static void I2C_EX_DMAMasterReceiveCplt(DMA_HandleTypeDef *hdma);
static void I2C_EX_DMAMasterSrcTran(DMA_HandleTypeDef *hdma);


static HAL_StatusTypeDef I2C_EX_SpeedConfig(I2C_EX_HandleTypeDef *hi2c);

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/** @defgroup I2C_EX_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the I2C_EXx peripheral:

      (+) User must Implement HAL_I2C_EX_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC).

      (+) Call the function HAL_I2C_EX_Init() to configure the selected device with
          the selected configuration:
        (++) Communication Speed
        (++) Duty cycle
        (++) Addressing mode
        (++) Own Address 1
        (++) Dual Addressing mode
        (++) Own Address 2
        (++) General call mode
        (++) Nostretch mode

      (+) Call the function HAL_I2C_EX_DeInit() to restore the default configuration
          of the selected I2C_EXx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the I2C_EX according to the specified parameters
  *         in the I2C_EX_InitTypeDef and initialize the associated handle.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Init(I2C_EX_HandleTypeDef *hi2c_ex)
{

  /* Check the I2C_EX handle allocation */
  if (hi2c_ex == NULL)
  {
    return HAL_ERROR;
  }


  if (hi2c_ex->State == HAL_I2C_EX_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hi2c_ex->Lock = HAL_UNLOCKED;

#if (USE_HAL_I2C_EX_REGISTER_CALLBACKS == 1)
    /* Init the I2C_EX Callback settings */
    hi2c_ex->MasterTxCpltCallback = HAL_I2C_EX_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
    hi2c_ex->MasterRxCpltCallback = HAL_I2C_EX_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
    hi2c_ex->SlaveTxCpltCallback  = HAL_I2C_EX_SlaveTxCpltCallback;  /* Legacy weak SlaveTxCpltCallback  */
    hi2c_ex->SlaveRxCpltCallback  = HAL_I2C_EX_SlaveRxCpltCallback;  /* Legacy weak SlaveRxCpltCallback  */
    hi2c_ex->ListenCpltCallback   = HAL_I2C_EX_ListenCpltCallback;   /* Legacy weak ListenCpltCallback   */
    hi2c_ex->MemTxCpltCallback    = HAL_I2C_EX_MemTxCpltCallback;    /* Legacy weak MemTxCpltCallback    */
    hi2c_ex->MemRxCpltCallback    = HAL_I2C_EX_MemRxCpltCallback;    /* Legacy weak MemRxCpltCallback    */
    hi2c_ex->ErrorCallback        = HAL_I2C_EX_ErrorCallback;        /* Legacy weak ErrorCallback        */
    hi2c_ex->AbortCpltCallback    = HAL_I2C_EX_AbortCpltCallback;    /* Legacy weak AbortCpltCallback    */
    hi2c_ex->AddrCallback         = HAL_I2C_EX_AddrCallback;         /* Legacy weak AddrCallback         */

    if (hi2c_ex->MspInitCallback == NULL)
    {
      hi2c_ex->MspInitCallback = HAL_I2C_EX_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    hi2c_ex->MspInitCallback(hi2c_ex);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    HAL_I2C_EX_MspInit(hi2c_ex);
#endif /* USE_HAL_I2C_EX_REGISTER_CALLBACKS */
  }

  hi2c_ex->State = HAL_I2C_EX_STATE_BUSY;

  /* Disable the selected I2C_EX peripheral */
  __HAL_I2C_EX_DISABLE(hi2c_ex);

	/*Disable I2C Interrupt*/
  __HAL_I2C_EX_DISABLE_IT(hi2c_ex, I2C_EX_IT_MASK_ALL);
  __HAL_I2C_EX_SMBUS_DISABLE_IT(hi2c_ex, I2C_SMBUS_IT_MASK_ALL);
  
	if((hi2c_ex->Init.AddressingMode == I2C_EX_ADDRESSINGMODE_MASTER_7BIT) || (hi2c_ex->Init.AddressingMode == I2C_EX_ADDRESSINGMODE_MASTER_10BIT))
	{
		MODIFY_REG(hi2c_ex->Instance->CR, (I2C_EX_CR_MASTER_MODE | I2C_EX_CR_SLAVE_DISABLE | I2C_EX_CR_10ADDR_MASTER), hi2c_ex->Init.AddressingMode);
	
		MODIFY_REG(hi2c_ex->Instance->CR, (I2C_EX_CR_SPEED | I2C_EX_CR_RESTART_EN | I2C_EX_CR_BUS_CLEAR_FEATURE_CTRL), \
									   (hi2c_ex->Init.ClockSpeed | hi2c_ex->Init.MasterRestartMode | hi2c_ex->Init.MasterBusClearMode));	
		
		
		I2C_EX_SpeedConfig(hi2c_ex);
		
		MODIFY_REG(hi2c_ex->Instance->TAR, (I2C_EX_TAR_SPECIAL | I2C_EX_TAR_GC_OR_START), hi2c_ex->Init.GeneralCallMode);	

		
	}
	else
	{
		MODIFY_REG(hi2c_ex->Instance->CR, (I2C_EX_CR_MASTER_MODE | I2C_EX_CR_SLAVE_DISABLE | I2C_EX_CR_10ADDR_SLAVE), hi2c_ex->Init.AddressingMode);		
	
		WRITE_REG(hi2c_ex->Instance->SAR, hi2c_ex->Init.SlaveOwnAddress);
	}
	
	SET_BIT(hi2c_ex->Instance->CR,I2C_EX_CR_STOP_DET_IFADDRESSED);
	CLEAR_BIT(hi2c_ex->Instance->EN, I2C_EX_EN_TX_CMD_BLOCK);
	
  /* Enable the selected I2C_EX peripheral */
  __HAL_I2C_EX_ENABLE(hi2c_ex);

  hi2c_ex->ErrorCode = HAL_I2C_EX_ERROR_NONE;
  hi2c_ex->State = HAL_I2C_EX_STATE_READY;
  hi2c_ex->PreviousState = I2C_EX_STATE_NONE;
  hi2c_ex->Mode = HAL_I2C_EX_MODE_NONE;

  return HAL_OK;
}

/**
  * @brief  DeInitialize the I2C_EX peripheral.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C_EX.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_DeInit(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Check the I2C_EX handle allocation */
  if (hi2c_ex == NULL)
  {
    return HAL_ERROR;
  }


  hi2c_ex->State = HAL_I2C_EX_STATE_BUSY;

  /* Disable the I2C_EX Peripheral Clock */
  __HAL_I2C_EX_DISABLE(hi2c_ex);

#if (USE_HAL_I2C_EX_REGISTER_CALLBACKS == 1)
  if (hi2c_ex->MspDeInitCallback == NULL)
  {
    hi2c_ex->MspDeInitCallback = HAL_I2C_EX_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  hi2c_ex->MspDeInitCallback(hi2c_ex);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  HAL_I2C_EX_MspDeInit(hi2c_ex);
#endif /* USE_HAL_I2C_EX_REGISTER_CALLBACKS */

  hi2c_ex->ErrorCode     = HAL_I2C_EX_ERROR_NONE;
  hi2c_ex->State         = HAL_I2C_EX_STATE_RESET;
  hi2c_ex->PreviousState = I2C_EX_STATE_NONE;
  hi2c_ex->Mode          = HAL_I2C_EX_MODE_NONE;

  /* Release Lock */
  __HAL_UNLOCK(hi2c_ex);

  return HAL_OK;
}


/**
  * @brief  Configure the I2C Baude Set
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
static HAL_StatusTypeDef I2C_EX_SpeedConfig(I2C_EX_HandleTypeDef *hi2c)
{
    /* Check the I2C handle allocation */
  if(hi2c == NULL) 
  {
    return HAL_ERROR;
  }
	
	uint32_t pclk;
	uint32_t i2c_clk;
	uint32_t SCLLCNT;
	uint32_t SCLHCNT;
#if defined(UM324xF) 
  pclk = HAL_RCM_GetPCLK1Freq();
#endif
   
#if defined(UM32x42x) || defined(UM32x41x)
  pclk = HAL_RCM_GetPCLK0Freq();
#endif
	
  /*I2C rate = (I2C BUS frequency )/��LCNT+HCNT��*/
  if(hi2c->Init.ClockSpeed == I2C_EX_SPEED_HIGH_1M)
  {
		 i2c_clk = 1000000;
		 SCLLCNT = 500/(1000000000/pclk);
		 SCLHCNT = pclk/i2c_clk - SCLLCNT;
     MODIFY_REG(hi2c->Instance->CR,I2C_EX_CR_SPEED_Msk,I2C_EX_ClockSpeed_Fast); 
     MODIFY_REG(hi2c->Instance->FSSCLLCNT,I2C_EX_FSSCLLCNT_FS_SCL_LCNT_Msk,SCLLCNT);
     MODIFY_REG(hi2c->Instance->FSSCLHCNT,I2C_EX_FSSCLHCNT_FS_SCL_HCNT_Msk,SCLHCNT);
  }
	else if(hi2c->Init.ClockSpeed == I2C_EX_SPEED_FAST_400K)
  {
		 i2c_clk = 400000;
		 SCLLCNT = 1300/(1000000000/pclk);
		 SCLHCNT = pclk/i2c_clk - SCLLCNT;
     MODIFY_REG(hi2c->Instance->CR,I2C_EX_CR_SPEED_Msk,I2C_EX_ClockSpeed_Fast); 
     MODIFY_REG(hi2c->Instance->FSSCLLCNT,I2C_EX_FSSCLLCNT_FS_SCL_LCNT_Msk,SCLLCNT);
     MODIFY_REG(hi2c->Instance->FSSCLHCNT,I2C_EX_FSSCLHCNT_FS_SCL_HCNT_Msk,SCLHCNT);
  }
  else 
  {
		 i2c_clk = 100000;
		 SCLLCNT = 4700/(1000000000/pclk);
		 SCLHCNT = pclk/i2c_clk - SCLLCNT;
		 MODIFY_REG(hi2c->Instance->CR,I2C_EX_CR_SPEED_Msk,I2C_EX_ClockSpeed_Standard);  
     MODIFY_REG(hi2c->Instance->SSSCLLCNT,I2C_EX_SSSCLLCNT_SS_SCL_LCNT_Msk,SCLLCNT);
     MODIFY_REG(hi2c->Instance->SSSCLHCNT,I2C_EX_SSSCLHCNT_SS_SCL_HCNT_Msk,SCLHCNT);
     
  }

  return HAL_OK;
}
/**
  * @brief  Initialize the I2C_EX MSP.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_MspInit(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitialize the I2C_EX MSP.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_MspDeInit(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_MspDeInit could be implemented in the user file
   */
}

#if (USE_HAL_I2C_EX_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User I2C_EX Callback
  *         To be used instead of the weak predefined callback
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_I2C_EX_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_EX_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_EX_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_EX_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_EX_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref HAL_I2C_EX_MEM_TX_COMPLETE_CB_ID Memory Tx Transfer callback ID
  *          @arg @ref HAL_I2C_EX_MEM_RX_COMPLETE_CB_ID Memory Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_EX_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_I2C_EX_ABORT_CB_ID Abort callback ID
  *          @arg @ref HAL_I2C_EX_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_I2C_EX_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_RegisterCallback(I2C_EX_HandleTypeDef *hi2c_ex, HAL_I2C_EX_CallbackIDTypeDef CallbackID, pI2C_EX_CallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hi2c_ex->ErrorCode |= HAL_I2C_EX_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }
  /* Process locked */
  __HAL_LOCK(hi2c_ex);

  if (HAL_I2C_EX_STATE_READY == hi2c_ex->State)
  {
    switch (CallbackID)
    {
      case HAL_I2C_EX_MASTER_TX_COMPLETE_CB_ID :
        hi2c_ex->MasterTxCpltCallback = pCallback;
        break;

      case HAL_I2C_EX_MASTER_RX_COMPLETE_CB_ID :
        hi2c_ex->MasterRxCpltCallback = pCallback;
        break;

      case HAL_I2C_EX_SLAVE_TX_COMPLETE_CB_ID :
        hi2c_ex->SlaveTxCpltCallback = pCallback;
        break;

      case HAL_I2C_EX_SLAVE_RX_COMPLETE_CB_ID :
        hi2c_ex->SlaveRxCpltCallback = pCallback;
        break;

      case HAL_I2C_EX_LISTEN_COMPLETE_CB_ID :
        hi2c_ex->ListenCpltCallback = pCallback;
        break;

      case HAL_I2C_EX_MEM_TX_COMPLETE_CB_ID :
        hi2c_ex->MemTxCpltCallback = pCallback;
        break;

      case HAL_I2C_EX_MEM_RX_COMPLETE_CB_ID :
        hi2c_ex->MemRxCpltCallback = pCallback;
        break;

      case HAL_I2C_EX_ERROR_CB_ID :
        hi2c_ex->ErrorCallback = pCallback;
        break;

      case HAL_I2C_EX_ABORT_CB_ID :
        hi2c_ex->AbortCpltCallback = pCallback;
        break;

      case HAL_I2C_EX_MSPINIT_CB_ID :
        hi2c_ex->MspInitCallback = pCallback;
        break;

      case HAL_I2C_EX_MSPDEINIT_CB_ID :
        hi2c_ex->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hi2c_ex->ErrorCode |= HAL_I2C_EX_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (HAL_I2C_EX_STATE_RESET == hi2c_ex->State)
  {
    switch (CallbackID)
    {
      case HAL_I2C_EX_MSPINIT_CB_ID :
        hi2c_ex->MspInitCallback = pCallback;
        break;

      case HAL_I2C_EX_MSPDEINIT_CB_ID :
        hi2c_ex->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hi2c_ex->ErrorCode |= HAL_I2C_EX_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hi2c_ex->ErrorCode |= HAL_I2C_EX_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hi2c_ex);
  return status;
}

/**
  * @brief  Unregister an I2C_EX Callback
  *         I2C_EX callback is redirected to the weak predefined callback
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_I2C_EX_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_EX_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_EX_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_EX_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_EX_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref HAL_I2C_EX_MEM_TX_COMPLETE_CB_ID Memory Tx Transfer callback ID
  *          @arg @ref HAL_I2C_EX_MEM_RX_COMPLETE_CB_ID Memory Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_EX_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_I2C_EX_ABORT_CB_ID Abort callback ID
  *          @arg @ref HAL_I2C_EX_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_I2C_EX_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_UnRegisterCallback(I2C_EX_HandleTypeDef *hi2c_ex, HAL_I2C_EX_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hi2c_ex);

  if (HAL_I2C_EX_STATE_READY == hi2c_ex->State)
  {
    switch (CallbackID)
    {
      case HAL_I2C_EX_MASTER_TX_COMPLETE_CB_ID :
        hi2c_ex->MasterTxCpltCallback = HAL_I2C_EX_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
        break;

      case HAL_I2C_EX_MASTER_RX_COMPLETE_CB_ID :
        hi2c_ex->MasterRxCpltCallback = HAL_I2C_EX_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
        break;

      case HAL_I2C_EX_SLAVE_TX_COMPLETE_CB_ID :
        hi2c_ex->SlaveTxCpltCallback = HAL_I2C_EX_SlaveTxCpltCallback;   /* Legacy weak SlaveTxCpltCallback  */
        break;

      case HAL_I2C_EX_SLAVE_RX_COMPLETE_CB_ID :
        hi2c_ex->SlaveRxCpltCallback = HAL_I2C_EX_SlaveRxCpltCallback;   /* Legacy weak SlaveRxCpltCallback  */
        break;

      case HAL_I2C_EX_LISTEN_COMPLETE_CB_ID :
        hi2c_ex->ListenCpltCallback = HAL_I2C_EX_ListenCpltCallback;     /* Legacy weak ListenCpltCallback   */
        break;

      case HAL_I2C_EX_MEM_TX_COMPLETE_CB_ID :
        hi2c_ex->MemTxCpltCallback = HAL_I2C_EX_MemTxCpltCallback;       /* Legacy weak MemTxCpltCallback    */
        break;

      case HAL_I2C_EX_MEM_RX_COMPLETE_CB_ID :
        hi2c_ex->MemRxCpltCallback = HAL_I2C_EX_MemRxCpltCallback;       /* Legacy weak MemRxCpltCallback    */
        break;

      case HAL_I2C_EX_ERROR_CB_ID :
        hi2c_ex->ErrorCallback = HAL_I2C_EX_ErrorCallback;               /* Legacy weak ErrorCallback        */
        break;

      case HAL_I2C_EX_ABORT_CB_ID :
        hi2c_ex->AbortCpltCallback = HAL_I2C_EX_AbortCpltCallback;       /* Legacy weak AbortCpltCallback    */
        break;

      case HAL_I2C_EX_MSPINIT_CB_ID :
        hi2c_ex->MspInitCallback = HAL_I2C_EX_MspInit;                   /* Legacy weak MspInit              */
        break;

      case HAL_I2C_EX_MSPDEINIT_CB_ID :
        hi2c_ex->MspDeInitCallback = HAL_I2C_EX_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        hi2c_ex->ErrorCode |= HAL_I2C_EX_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (HAL_I2C_EX_STATE_RESET == hi2c_ex->State)
  {
    switch (CallbackID)
    {
      case HAL_I2C_EX_MSPINIT_CB_ID :
        hi2c_ex->MspInitCallback = HAL_I2C_EX_MspInit;                   /* Legacy weak MspInit              */
        break;

      case HAL_I2C_EX_MSPDEINIT_CB_ID :
        hi2c_ex->MspDeInitCallback = HAL_I2C_EX_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        hi2c_ex->ErrorCode |= HAL_I2C_EX_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hi2c_ex->ErrorCode |= HAL_I2C_EX_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hi2c_ex);
  return status;
}

/**
  * @brief  Register the Slave Address Match I2C_EX Callback
  *         To be used instead of the weak HAL_I2C_EX_AddrCallback() predefined callback
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @param  pCallback pointer to the Address Match Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_RegisterAddrCallback(I2C_EX_HandleTypeDef *hi2c_ex, pI2C_EX_AddrCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hi2c_ex->ErrorCode |= HAL_I2C_EX_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }
  /* Process locked */
  __HAL_LOCK(hi2c_ex);

  if (HAL_I2C_EX_STATE_READY == hi2c_ex->State)
  {
    hi2c_ex->AddrCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hi2c_ex->ErrorCode |= HAL_I2C_EX_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hi2c_ex);
  return status;
}

/**
  * @brief  UnRegister the Slave Address Match I2C_EX Callback
  *         Info Ready I2C_EX Callback is redirected to the weak HAL_I2C_EX_AddrCallback() predefined callback
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_UnRegisterAddrCallback(I2C_EX_HandleTypeDef *hi2c_ex)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hi2c_ex);

  if (HAL_I2C_EX_STATE_READY == hi2c_ex->State)
  {
    hi2c_ex->AddrCallback = HAL_I2C_EX_AddrCallback; /* Legacy weak AddrCallback  */
  }
  else
  {
    /* Update the error code */
    hi2c_ex->ErrorCode |= HAL_I2C_EX_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hi2c_ex);
  return status;
}

#endif /* USE_HAL_I2C_EX_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup I2C_EX_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the I2C_EX data
    transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode : The communication is performed in the polling mode.
            The status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode : The communication is performed using Interrupts
            or DMA. These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated I2C_EX IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.

    (#) Blocking mode functions are :
        (++) HAL_I2C_EX_Master_Transmit()
        (++) HAL_I2C_EX_Master_Receive()
        (++) HAL_I2C_EX_Slave_Transmit()
        (++) HAL_I2C_EX_Slave_Receive()
        (++) HAL_I2C_EX_Mem_Write()
        (++) HAL_I2C_EX_Mem_Read()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) HAL_I2C_EX_MasterTxCpltCallback()
        (++) HAL_I2C_EX_MasterRxCpltCallback()
        (++) HAL_I2C_EX_SlaveTxCpltCallback()
        (++) HAL_I2C_EX_SlaveRxCpltCallback()
        (++) HAL_I2C_EX_MemTxCpltCallback()
        (++) HAL_I2C_EX_MemRxCpltCallback()
        (++) HAL_I2C_EX_AddrCallback()
        (++) HAL_I2C_EX_ListenCpltCallback()
        (++) HAL_I2C_EX_ErrorCallback()
        (++) HAL_I2C_EX_AbortCpltCallback()

@endverbatim
  * @{
  */

/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Master_Transmit(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = HAL_GetTick();
	
  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if (I2C_EX_WaitOnFlagUntilTimeout(hi2c, I2C_EX_FLAG_MST_ACTIVITY, SET, I2C_EX_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK)
    {
		   return HAL_BUSY;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Check if the I2C_EX is already enabled */
    if ((hi2c->Instance->EN & I2C_EX_EN_ENABLE) == I2C_EX_EN_ENABLE)
    {
		/* Disable I2C_EX peripheral */
		__HAL_I2C_EX_DISABLE(hi2c);
    }	

    hi2c->State       = HAL_I2C_EX_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_EX_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
	  hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;

    /* Set Slave Address */
	if(hi2c->Init.AddressingMode == I2C_EX_ADDRESSINGMODE_MASTER_7BIT)
	{
		/* Set slave 7bit address */
		MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x7FUL));
	}
	else
	{
		/* Set slave 10bit address */
		MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x3FFUL)|I2C_EX_TAR_10BITADDR_MASTER);
	}

	/* Enable I2C_EX peripheral */
	__HAL_I2C_EX_ENABLE(hi2c);

	while (hi2c->XferSize > 0U)
	{
		/* Wait until TX_EMPTY flag is set */
		if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_TX_EMPTY, RESET, Timeout, tickstart) != HAL_OK)
		{
			return HAL_ERROR;
		}		
		
		if(hi2c->XferSize == 1)
		{
			/* Write data to DATACMD */
			hi2c->Instance->DATACMD = (*hi2c->pBuffPtr | I2C_EX_CMD_WRITE | I2C_EX_CMD_STOP);		
		}
		else
		{
			/* Write data to DATACMD */
			hi2c->Instance->DATACMD = (*hi2c->pBuffPtr | I2C_EX_CMD_WRITE);		
		}
		/* Increment Buffer pointer */
		hi2c->pBuffPtr++;

		/* Update counter */
		hi2c->XferCount--;
		hi2c->XferSize--;

	  }
	
    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->Mode = HAL_I2C_EX_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receives in master mode an amount of data in blocking mode.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Master_Receive(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  UNUSED(Timeout);
  /* Init tickstart for timeout management*/
  uint32_t tickstart = HAL_GetTick();

  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if (I2C_EX_WaitOnFlagUntilTimeout(hi2c, I2C_EX_FLAG_MST_ACTIVITY, SET, I2C_EX_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK)
    {
		return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Check if the I2C_EX is already enabled */
    if ((hi2c->Instance->EN & I2C_EX_EN_ENABLE) == I2C_EX_EN_ENABLE)
    {
		/* Disable I2C_EX peripheral */
		__HAL_I2C_EX_DISABLE(hi2c);
    }
	

    hi2c->State       = HAL_I2C_EX_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_EX_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;

	  /* Set Slave Address */
	  if (hi2c->Init.AddressingMode == I2C_EX_ADDRESSINGMODE_MASTER_7BIT)
	  {
	  	/* Set slave 7bit address */
	  	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x7FUL));
	  }
	  else
	  {
	  	/* Set slave 10bit address */
	  	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x3FFUL)|I2C_EX_TAR_10BITADDR_MASTER);
	  }
	  
	  /* Enable I2C_EX peripheral */
	  __HAL_I2C_EX_ENABLE(hi2c);
	  
    while (hi2c->XferSize > 0U)
    {
	     if(hi2c->XferSize == 1)
	     {
	     	/* Write cmd to DATACMD */
	     	hi2c->Instance->DATACMD |= (I2C_EX_CMD_READ | I2C_EX_CMD_STOP);		
	     }
	     else
	     {
	     	/* Write cmd to DATACMD */
	     	hi2c->Instance->DATACMD |= (I2C_EX_CMD_READ);		
	     }
	     
	     /* Wait until RFNE flag is set */
	     if (I2C_EX_WaitOnFlagUntilTimeout(hi2c, I2C_EX_FLAG_RFNE, RESET, I2C_EX_TIMEOUT_FLAG, tickstart) != HAL_OK)
	     {
	     	return HAL_ERROR;
	     }
	     
	     /* Read data from DATACMD */
	     *hi2c->pBuffPtr = ((uint8_t)hi2c->Instance->DATACMD & 0xFFUL);
	     
	     /* Increment Buffer pointer */
	     hi2c->pBuffPtr++;
	     
	     /* Update counter */
	     hi2c->XferCount--;
	     hi2c->XferSize--;
    }
	     
    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->Mode = HAL_I2C_EX_MODE_NONE;
	   
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
	   
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmits in slave mode an amount of data in blocking mode.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Slave_Transmit(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = HAL_GetTick();

  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Check if the I2C_EX is already enabled */
    if ((hi2c->Instance->EN & I2C_EX_EN_ENABLE) != I2C_EX_EN_ENABLE)
    {
      /* Enable I2C_EX peripheral */
      __HAL_I2C_EX_ENABLE(hi2c);
    }

    hi2c->State       = HAL_I2C_EX_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_EX_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;

    /* Wait until RD_REQ flag is set */
    if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_RD_REQ, RESET, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear RD_REQ flag */
    __HAL_I2C_EX_CLEAR_RD_REQ_ITFLAG(hi2c);

	  if(__HAL_I2C_EX_GET_RAW_IT_FLAG(hi2c, I2C_EX_RAW_ITFLAG_TX_ABRT) == SET)
	  {
	  	/* Clear TX_ABRT flag */
	  	__HAL_I2C_EX_CLEAR_TX_ABRT_ITFLAG(hi2c);	
	  }
	
    while (hi2c->XferSize > 0U)
    {
      /* Write data to DATACMD */
      hi2c->Instance->DATACMD = (*hi2c->pBuffPtr | I2C_EX_CMD_WRITE);

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferCount--;
      hi2c->XferSize--;

      /* Wait until TX_EMPTY flag is set */
	  if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_TX_EMPTY, RESET, Timeout, tickstart) != HAL_OK)
	  {
	    return HAL_ERROR;
	  }
    }
	
    /* Wait until STOP flag is set */
    if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_STOP_DET, RESET, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }
    /* Clear STOP flag */
    __HAL_I2C_EX_CLEAR_STOP_DET_ITFLAG(hi2c);

    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->Mode = HAL_I2C_EX_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in blocking mode
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C_EX.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Slave_Receive(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = HAL_GetTick();

  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    if ((pData == NULL) || (Size == (uint16_t)0))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Check if the I2C_EX is already enabled */
    if ((hi2c->Instance->EN & I2C_EX_EN_ENABLE) != I2C_EX_EN_ENABLE)
    {
      /* Enable I2C_EX peripheral */
      __HAL_I2C_EX_ENABLE(hi2c);
    }
	
    hi2c->State       = HAL_I2C_EX_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_EX_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;

    /* Wait until START flag is set */
    if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_START_DET, RESET, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear START flag */
    __HAL_I2C_EX_CLEAR_START_DET_ITFLAG(hi2c);

    while (hi2c->XferSize > 0U)
    {
      /* Wait until RX_FULL flag is set */
      if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_RX_FULL, RESET, Timeout, tickstart) != HAL_OK)
      {
		      return HAL_ERROR;
      }

      /* Read data from DATACMD */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATACMD;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferSize--;
      hi2c->XferCount--;

    }

    /* Wait until STOP flag is set */
    if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_STOP_DET, RESET, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }
	
    /* Clear STOP flag */
    __HAL_I2C_EX_CLEAR_STOP_DET_ITFLAG(hi2c);

    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->Mode = HAL_I2C_EX_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Mem_Write(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = HAL_GetTick();

  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if (I2C_EX_WaitOnFlagUntilTimeout(hi2c, I2C_EX_FLAG_MST_ACTIVITY, SET, I2C_EX_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK)
    {
		return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Check if the I2C_EX is already enabled */
    if ((hi2c->Instance->EN & I2C_EX_EN_ENABLE) == I2C_EX_EN_ENABLE)
    {
		/* Disable I2C_EX peripheral */
		__HAL_I2C_EX_DISABLE(hi2c);
    }	

    hi2c->State     = HAL_I2C_EX_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_EX_MODE_MEM;
    hi2c->ErrorCode = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;

       /* Set Slave Address */
	  if (hi2c->Init.AddressingMode == I2C_EX_ADDRESSINGMODE_MASTER_7BIT)
	  {
	  	/* Set slave 7bit address */
	  	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x7FUL));
	  }
	  else
	  {
	  	/* Set slave 10bit address */
	  	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x3FFUL)|I2C_EX_TAR_10BITADDR_MASTER);
	  }
	  
	  /* Enable I2C_EX peripheral */
	  __HAL_I2C_EX_ENABLE(hi2c);	
	  
	  /* Wait until TX_EMPTY flag is set */
	  if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_TX_EMPTY, RESET, Timeout, tickstart) != HAL_OK)
	  {
	  	return HAL_ERROR;
	  }	
	  
	  if(MemAddSize == I2C_EX_MEMADD_SIZE_8BIT)
	  {
	  	/* Write data to DATACMD */
	  	hi2c->Instance->DATACMD = (MemAddress | I2C_EX_CMD_WRITE);	
	  }
	  else
	  {
	  	/* Write data to DATACMD */
	  	hi2c->Instance->DATACMD = (((MemAddress & (uint16_t)0xFF00) >> 8)| I2C_EX_CMD_WRITE);	
	  	/* Wait until TX_EMPTY flag is set */
	  	if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_TX_EMPTY, RESET, Timeout, tickstart) != HAL_OK)
	  	{
	  		return HAL_ERROR;
	  	}		
	  	/* Write data to DATACMD */
	  	hi2c->Instance->DATACMD = ((MemAddress & (uint16_t)0x00FF) | I2C_EX_CMD_WRITE);			
	  }
	
    while (hi2c->XferSize > 0U)
    {
		/* Wait until TX_EMPTY flag is set */
		if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_TX_EMPTY, RESET, Timeout, tickstart) != HAL_OK)
		{
			return HAL_ERROR;
		}	

		if(hi2c->XferSize == 1)
		{
			/* Write data to DATACMD */
			hi2c->Instance->DATACMD = (*hi2c->pBuffPtr | I2C_EX_CMD_WRITE | I2C_EX_CMD_STOP);		
		}
		else
		{
			/* Write data to DATACMD */
			hi2c->Instance->DATACMD = (*hi2c->pBuffPtr | I2C_EX_CMD_WRITE);		
		}

		/* Increment Buffer pointer */
		hi2c->pBuffPtr++;

		/* Update counter */
		hi2c->XferSize--;
		hi2c->XferCount--;

    }

    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->Mode = HAL_I2C_EX_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Mem_Read(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = HAL_GetTick();

  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if (I2C_EX_WaitOnFlagUntilTimeout(hi2c, I2C_EX_FLAG_MST_ACTIVITY, SET, I2C_EX_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK)
    {
		    return HAL_BUSY;
    }	

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Check if the I2C_EX is already enabled */
    if ((hi2c->Instance->EN & I2C_EX_EN_ENABLE) == I2C_EX_EN_ENABLE)
    {
		/* Disable I2C_EX peripheral */
		__HAL_I2C_EX_DISABLE(hi2c);
    }	

    hi2c->State     = HAL_I2C_EX_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_EX_MODE_MEM;
    hi2c->ErrorCode = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;

      /* Set Slave Address */
	  if (hi2c->Init.AddressingMode == I2C_EX_ADDRESSINGMODE_MASTER_7BIT)
	  {
	  	/* Set slave 7bit address */
	  	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x7FUL));
	  }
	  else
	  {
	  	/* Set slave 10bit address */
	  	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x3FFUL)|I2C_EX_TAR_10BITADDR_MASTER);
	  }
	  
	  /* Enable I2C_EX peripheral */
	  __HAL_I2C_EX_ENABLE(hi2c);	
	  
	  /* Wait until TX_EMPTY flag is set */
	  if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_TX_EMPTY, RESET, Timeout, tickstart) != HAL_OK)
	  {
	  	return HAL_ERROR;
	  }	
	  
	  if(MemAddSize == I2C_EX_MEMADD_SIZE_8BIT)
	  {
	  	/* Write data to DATACMD */
	  	hi2c->Instance->DATACMD = (MemAddress | I2C_EX_CMD_WRITE);	
	  }
	  else
	  {
	  	/* Write data to DATACMD */
	  	hi2c->Instance->DATACMD = (((MemAddress & (uint16_t)0xFF00) >> 8)| I2C_EX_CMD_WRITE);	
	  	/* Wait until TX_EMPTY flag is set */
	  	if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_TX_EMPTY, RESET, Timeout, tickstart) != HAL_OK)
	  	{
	  		return HAL_ERROR;
	  	}		
	  	/* Write data to DATACMD */
	  	hi2c->Instance->DATACMD = ((MemAddress & (uint16_t)0x00FF) | I2C_EX_CMD_WRITE);	
	  }

    while (hi2c->XferSize > 0U)
    {
		if(hi2c->XferSize == 1)
		{
			/* Write cmd to DATACMD */
			hi2c->Instance->DATACMD |= (I2C_EX_CMD_READ | I2C_EX_CMD_STOP);		
		}
		else
		{
			/* Write cmd to DATACMD */
			hi2c->Instance->DATACMD |= (I2C_EX_CMD_READ);		
		}

		/* Wait until RX_FULL flag is set */
		if (I2C_EX_WaitOnRawITFlagUntilTimeout(hi2c, I2C_EX_RAW_ITFLAG_RX_FULL, RESET, Timeout, tickstart) != HAL_OK)
		{
			return HAL_ERROR;
		}
		
		/* Read data from DATACMD */
		*hi2c->pBuffPtr = ((uint8_t)hi2c->Instance->DATACMD & 0xFFUL);
		
		/* Increment Buffer pointer */
		hi2c->pBuffPtr++;

		/* Update counter */
		hi2c->XferCount--;
		hi2c->XferSize--;
    }

    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->Mode = HAL_I2C_EX_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}



/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Master_Transmit_IT(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size)
{
  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {

    hi2c->State       = HAL_I2C_EX_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_EX_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr   = pData;
    hi2c->XferCount  = Size;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_EX_Master_ISR_IT;
		
    __HAL_I2C_EX_DISABLE(hi2c);  
      /* Set Slave Address */
	  if (hi2c->Init.AddressingMode == I2C_EX_ADDRESSINGMODE_MASTER_7BIT)
	  {
	  	/* Set slave 7bit address */
	  	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x7FUL));
	  }
	  else
	  {
	  	/* Set slave 10bit address */
	  	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x3FFUL)|I2C_EX_TAR_10BITADDR_MASTER);
	  }

    __HAL_I2C_EX_ENABLE(hi2c);
      
    __HAL_I2C_EX_ENABLE_IT(hi2c, I2C_EX_IT_TX_EMPTY | I2C_EX_IT_STOP_DET);
      
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Master_Receive_IT(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{

  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_EX_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_EX_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_EX_Master_ISR_IT;
    
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
    
		__HAL_I2C_EX_DISABLE(hi2c);
      /* Set Slave Address */
	  if (hi2c->Init.AddressingMode == I2C_EX_ADDRESSINGMODE_MASTER_7BIT)
	  {
	  	/* Set slave 7bit address */
	  	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x7FUL));
	  }
	  else
	  {
	  	/* Set slave 10bit address */
	  	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x3FFUL)|I2C_EX_TAR_10BITADDR_MASTER);
	  }
    
    __HAL_I2C_EX_ENABLE(hi2c);
      
    if(hi2c->XferCount == 1)
    {
       hi2c->Instance->DATACMD = (I2C_EX_CMD_READ | I2C_EX_CMD_STOP);
    }
    else
    {
       hi2c->Instance->DATACMD = (I2C_EX_CMD_READ);
    }
    
    
		__HAL_I2C_EX_ENABLE_IT(hi2c, I2C_EX_IT_RX_FULL | I2C_EX_IT_STOP_DET);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Slave_Transmit_IT(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    hi2c->State       = HAL_I2C_EX_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_EX_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

   /* Prepare transfer parameters */
    hi2c->pBuffPtr     = pData;
    hi2c->XferCount    = Size;
    hi2c->XferOptions  = I2C_EX_NO_OPTION_FRAME;
    hi2c->XferISR      = I2C_EX_Slave_ISR_IT;
    /* Enable ERR, TC, STOP, NACK, RXI interrupt */
		__HAL_I2C_EX_ENABLE_IT(hi2c, I2C_EX_IT_START_DET | I2C_EX_IT_RD_REQ | I2C_EX_IT_STOP_DET);
    
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Slave_Receive_IT(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    hi2c->State       = HAL_I2C_EX_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_EX_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;
    
    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;
    hi2c->XferSize    = hi2c->XferCount;  
		hi2c->XferISR     = I2C_EX_Slave_ISR_IT;
		
		 /* Enable  Start,STOP,RXI interrupt */
		__HAL_I2C_EX_ENABLE_IT(hi2c, I2C_EX_IT_START_DET | I2C_EX_IT_RX_FULL | I2C_EX_IT_STOP_DET );    
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}





/**
  * @}
  */

/** @defgroup I2C_EX_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */

/**
  * @brief  This function handles I2C_EX interrupt request.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval None
  */
__attribute__((section(".tcm_code"))) __attribute__((optimize("-Ofast"))) \
void HAL_I2C_EX_IRQHandler(I2C_EX_HandleTypeDef *hi2c_ex)
{
	/* Get current IT Flags and IT sources value */
  uint32_t itflags   = READ_REG(hi2c_ex->Instance->ISR);
  uint32_t itsources = READ_REG(hi2c_ex->Instance->INTMASK);
/* I2C events treatment -------------------------------------*/
	
  if(hi2c_ex->XferISR != NULL)
  {
     hi2c_ex->XferISR(hi2c_ex, itflags, itsources);
  }
  
	
}


/**
  * @brief  Master Tx Transfer completed callback.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_MasterTxCpltCallback(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_MasterTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Master Rx Transfer completed callback.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_MasterRxCpltCallback(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_MasterRxCpltCallback could be implemented in the user file
   */
}

/** @brief  Slave Tx Transfer completed callback.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_SlaveTxCpltCallback(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_SlaveTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_SlaveRxCpltCallback(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_SlaveRxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Slave Address Match callback.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @param  TransferDirection Master request Transfer Direction (Write/Read), value of @ref I2C_EX_XferDirection_definition
  * @param  AddrMatchCode Address Match Code
  * @retval None
  */
__weak void HAL_I2C_EX_AddrCallback(I2C_EX_HandleTypeDef *hi2c_ex, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);
  UNUSED(TransferDirection);
  UNUSED(AddrMatchCode);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_AddrCallback() could be implemented in the user file
   */
}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_ListenCpltCallback(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_ListenCpltCallback() could be implemented in the user file
  */
}

/**
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_MemTxCpltCallback(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_MemTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_MemRxCpltCallback(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_MemRxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  I2C_EX error callback.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_ErrorCallback(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_ErrorCallback could be implemented in the user file
   */
}

/**
  * @brief  I2C_EX abort callback.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval None
  */
__weak void HAL_I2C_EX_AbortCpltCallback(I2C_EX_HandleTypeDef *hi2c_ex)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c_ex);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_EX_AbortCpltCallback could be implemented in the user file
   */
}





/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_EX_Master_ISR_IT(struct __I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources)
{
  /* Process Locked */
  __HAL_LOCK(hi2c);
   uint32_t tmpITFlags = ITFlags;

  if ((I2C_EX_CHECK_FLAG(tmpITFlags, I2C_EX_ITFLAG_STOP_DET) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources, I2C_EX_IT_STOP_DET) != RESET))
  {
    /* Call I2C Master complete process */

     I2C_EX_ITMasterCplt(hi2c, tmpITFlags);
     __HAL_I2C_EX_CLEAR_STOP_DET_ITFLAG(hi2c);
  }
  
  if((I2C_EX_CHECK_FLAG(tmpITFlags, I2C_EX_ITFLAG_RX_FULL) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources,I2C_EX_IT_RX_FULL)!= RESET))
  {
   
    if (hi2c->XferCount > 0U)
    {
       *hi2c->pBuffPtr = hi2c->Instance->DATACMD & 0xff;
         
       /* Increment Buffer pointer */
       hi2c->pBuffPtr++;
			 
       hi2c->XferCount--;
       hi2c->XferSize--;
         
       if(hi2c->XferCount == 1)
       {
          hi2c->Instance->DATACMD = (I2C_EX_CMD_READ | I2C_EX_CMD_STOP);      
       }
       else if(hi2c->XferCount > 1)
       {
          hi2c->Instance->DATACMD = (I2C_EX_CMD_READ);
       }
     }
   }
   else if((I2C_EX_CHECK_FLAG(tmpITFlags, I2C_EX_ITFLAG_TX_EMPTY) != RESET) && (I2C_EX_CHECK_FLAG(ITSources,I2C_EX_ITFLAG_TX_EMPTY)!= RESET))
   {             
      if (hi2c->XferCount > 0U)
      {
        if(hi2c->XferCount == 1)
        {
            hi2c->Instance->DATACMD = *hi2c->pBuffPtr | (I2C_EX_CMD_WRITE | I2C_EX_CMD_STOP); 
        }
        else
        {
            hi2c->Instance->DATACMD = *hi2c->pBuffPtr | (I2C_EX_CMD_WRITE);
        }
       
        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;
    
        hi2c->XferCount--;
        hi2c->XferSize--;
       }
    }
    else
    {     
        hi2c->State = HAL_I2C_EX_STATE_READY;
        hi2c->Mode  = HAL_I2C_EX_MODE_NONE;
    }
  
  /* Process Unlocked */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}


/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_EX_Slave_ISR_IT(struct __I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources)
{
    /* Process Locked */
  __HAL_LOCK(hi2c);
  if((I2C_EX_CHECK_FLAG(ITFlags, I2C_EX_ITFLAG_START_DET) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources, I2C_EX_IT_START_DET) != RESET))
	{
      if(__HAL_I2C_EX_GET_IT_FLAG(hi2c,I2C_EX_ITFLAG_RESTART_DET) != RESET )
      {
         __HAL_I2C_EX_CLEAR_RESTART_DET_ITFLAG(hi2c);
         /*stop dma transmission */
         if(hi2c->State == HAL_I2C_EX_STATE_BUSY_RX)
         {
					  hi2c->State = HAL_I2C_EX_STATE_READY;
            HAL_I2C_EX_SlaveRxCpltCallback(hi2c);
         }
         else if(hi2c->State == HAL_I2C_EX_STATE_BUSY_TX)
         {
					 hi2c->State = HAL_I2C_EX_STATE_READY;
           HAL_I2C_EX_SlaveTxCpltCallback(hi2c);
         }
         I2C_EX_ITSlaveSeqCplt(hi2c);
      }
      __HAL_I2C_EX_CLEAR_START_DET_ITFLAG(hi2c);		
	}	
	
  if ((I2C_EX_CHECK_FLAG(ITFlags, I2C_EX_ITFLAG_STOP_DET) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources, I2C_EX_IT_STOP_DET) != RESET))
  {
      /* Call I2C Slave complete process */
      I2C_EX_ITSlaveCplt(hi2c, ITFlags);
      __HAL_I2C_EX_CLEAR_STOP_DET_ITFLAG(hi2c);
  }
  
  if((I2C_EX_CHECK_FLAG(ITFlags, I2C_EX_ITFLAG_RX_FULL) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources,I2C_EX_IT_RX_FULL)!= RESET))
  {
    if (hi2c->XferCount > 0U)
    {
      *hi2c->pBuffPtr = hi2c->Instance->DATACMD & 0xff;
        
      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
		}	
  }
  
  else if((I2C_EX_CHECK_FLAG(ITFlags, I2C_EX_ITFLAG_RD_REQ) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources,I2C_EX_IT_RD_REQ)!= RESET))
  {
		  if(__HAL_I2C_EX_GET_RAW_IT_FLAG(hi2c,I2C_EX_RAW_ITFLAG_TX_EMPTY)!=RESET)
			{
        /* Read the clear register to clear the flag bit */ 
        if (hi2c->XferCount > 0U)
        {
          hi2c->Instance->DATACMD = *hi2c->pBuffPtr++ | (I2C_EX_CMD_WRITE);
          /* Increment Buffer pointer */
          hi2c->XferCount--;
          hi2c->XferSize--;
        }	
		  }
      __HAL_I2C_EX_CLEAR_RD_REQ_ITFLAG(hi2c);  
  }
  
  /* Process Unlocked */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}


/**
  * @brief  I2C Slave sequential complete process.
  * @param  hi2c I2C handle.
  * @retval None
  */

static void I2C_EX_ITMasterCplt(I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags)
{
  UNUSED(ITFlags);
     /* Call the corresponding callback to inform upper layer of End of Transfer */
  if (hi2c->State == HAL_I2C_EX_STATE_BUSY_RX)
  {
    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->PreviousState = I2C_EX_STATE_NONE;
      
    __HAL_I2C_EX_DISABLE_IT(hi2c, I2C_EX_IT_RX_FULL | I2C_EX_IT_STOP_DET);
    
    __HAL_I2C_EX_DISABLE(hi2c);
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    if(hi2c->MasterRxCpltCallback == NULL)
    {
        hi2c->MasterRxCpltCallback = HAL_I2C_MasterRxCpltCallback;
    }
    hi2c->MasterRxCpltCallback(hi2c);
#else
    HAL_I2C_EX_MasterRxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  else if(hi2c->State == HAL_I2C_EX_STATE_BUSY_TX)
  {
    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->PreviousState = I2C_EX_STATE_NONE;
      
    __HAL_I2C_EX_DISABLE_IT(hi2c, I2C_EX_IT_TX_EMPTY | I2C_EX_IT_STOP_DET);
    __HAL_I2C_EX_DISABLE(hi2c);
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
    
    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    if(hi2c->MasterTxCpltCallback == NULL)
    {
        hi2c->MasterTxCpltCallback = HAL_I2C_MasterTxCpltCallback;
    }
    hi2c->MasterTxCpltCallback(hi2c);
#else
    HAL_I2C_EX_MasterTxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  else
  {
    __HAL_I2C_EX_DISABLE_IT(hi2c,I2C_EX_IT_MASK_ALL);
    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->PreviousState = I2C_EX_STATE_NONE;
    __HAL_I2C_EX_DISABLE(hi2c);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
  }
}




/**
  * @brief  I2C Slave sequential complete process.
  * @param  hi2c I2C handle.
  * @retval None
  */

static void I2C_EX_ITSlaveCplt(I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags)
{
  UNUSED(ITFlags);

  if(hi2c->State== HAL_I2C_EX_STATE_BUSY_RX)
  {
    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->PreviousState = I2C_EX_STATE_NONE;
    hi2c->Mode  = HAL_I2C_EX_MODE_NONE;
		
   __HAL_I2C_EX_DISABLE_IT(hi2c, I2C_EX_IT_START_DET|I2C_EX_IT_RX_FULL|I2C_EX_IT_STOP_DET|I2C_EX_IT_TX_ABRT);
 
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    if(hi2c->SlaveRxCpltCallback == NULL)
    {
        hi2c->SlaveRxCpltCallback = HAL_I2C_SlaveRxCpltCallback;
    }
    hi2c->SlaveRxCpltCallback(hi2c);
#else
    HAL_I2C_EX_SlaveRxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  else if(hi2c->State == HAL_I2C_EX_STATE_BUSY_TX)
  {
    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->PreviousState = I2C_EX_STATE_NONE;
    hi2c->Mode  = HAL_I2C_EX_MODE_NONE;  
    __HAL_I2C_EX_DISABLE_IT(hi2c, I2C_EX_IT_START_DET|I2C_EX_IT_TX_ABRT|I2C_EX_IT_TX_EMPTY|I2C_EX_IT_RD_REQ|I2C_EX_IT_STOP_DET);
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    if(hi2c->SlaveTxCpltCallback == NULL)
    {
        hi2c->SlaveTxCpltCallback = HAL_I2C_SlaveTxCpltCallback;
    }
#else
    HAL_I2C_EX_SlaveTxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  else if(hi2c->State == HAL_I2C_EX_STATE_READY)
  {
     /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
  }
  else
  {
    __HAL_I2C_EX_DISABLE_IT(hi2c,I2C_EX_IT_MASK_ALL);
    hi2c->State = HAL_I2C_EX_STATE_READY;
    hi2c->PreviousState = I2C_EX_STATE_NONE;
    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
  }
}


/**
  * @brief  I2C Master sequential complete process.
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_EX_ITMasterSeqCplt(I2C_EX_HandleTypeDef *hi2c)
{
  uint32_t tmpcr1value = READ_REG(hi2c->Instance->DMACR);
  /* Reset I2C handle mode */
  hi2c->Mode = HAL_I2C_EX_MODE_NONE;

  /* If a DMA is ongoing, Update handle size context */
  if ((tmpcr1value & (I2C_EX_DMACR_RDMAE)) != 0U)
  {
    /* Disable DMA Request */
    hi2c->Instance->DMACR &= ~I2C_EX_DMACR_RDMAE;
  }
  else if ((tmpcr1value & (I2C_EX_DMACR_TDMAE)) != 0U)
  {
    /* Disable DMA Request */
   hi2c->Instance->DMACR &= ~I2C_EX_DMACR_TDMAE;
  }
  else
  {
    /* Do nothing */
  }
}

/**
  * @brief  I2C Slave sequential complete process.
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_EX_ITSlaveSeqCplt(I2C_EX_HandleTypeDef *hi2c)
{
  uint32_t tmpcr1value = READ_REG(hi2c->Instance->DMACR);

  /* Reset I2C handle mode */
  hi2c->Mode = HAL_I2C_EX_MODE_NONE;
  /* If a DMA is ongoing, Update handle size context */
  if ((tmpcr1value & (I2C_EX_DMACR_RDMAE)) != 0U)
  {
    /* Disable DMA Request */
    hi2c->Instance->DMACR &= ~I2C_EX_DMACR_RDMAE;
  }
  else if ((tmpcr1value & (I2C_EX_DMACR_TDMAE)) != 0U)
  {
    /* Disable DMA Request */
   hi2c->Instance->DMACR &= ~I2C_EX_DMACR_TDMAE;
  }
  else
  {
    /* Do nothing */
  }
}


/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Master_Transmit_DMA(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                              uint16_t Size)
{
  HAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_EX_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_EX_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_EX_Master_ISR_DMA;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
    }
    
    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmatx != NULL)
      {
				__HAL_I2C_EX_DISABLE(hi2c);  
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmatx->XferTfrCallback = I2C_EX_DMAMasterTransmitCplt;
        
           /* Set Slave Address */
	      if (hi2c->Init.AddressingMode == I2C_EX_ADDRESSINGMODE_MASTER_7BIT)
	      {
	      	/* Set slave 7bit address */
	      	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x7FUL));
	      }
	      else
	      {
	      	/* Set slave 10bit address */
	      	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x3FFUL)|I2C_EX_TAR_10BITADDR_MASTER);
	      }
         /* Init tickstart for timeout management*/
        __HAL_I2C_EX_ENABLE(hi2c);
         /* Enable the DMA channel */
        dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)hi2c->pBuffPtr, (uint32_t)&hi2c->Instance->DATACMD, hi2c->XferSize-1);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_EX_STATE_READY;
        hi2c->Mode      = HAL_I2C_EX_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_EX_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }

      if (dmaxferstatus == HAL_OK)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
         /* Enable  STOP, interrupt */
        __HAL_I2C_EX_ENABLE_IT(hi2c, I2C_EX_IT_STOP_DET);

        /* Enable DMA Request */
        hi2c->Instance->DMACR |= I2C_EX_DMACR_TDMAE;
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_EX_STATE_READY;
        hi2c->Mode      = HAL_I2C_EX_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_EX_ERROR_DMA;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Master_Receive_DMA(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size)
{
  HAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_EX_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_EX_MODE_MASTER;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_EX_Master_ISR_DMA;

    if (hi2c->XferCount > MAX_NBYTE_SIZE)
    {
      hi2c->XferSize = MAX_NBYTE_SIZE;    
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;      
    }

    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmarx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmarx->XferTfrCallback = I2C_EX_DMAMasterReceiveCplt; 
        hi2c->hdmarx->XferSrcTranCallback = I2C_EX_DMAMasterSrcTran;				
        /* Enable the DMA channel */
        dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->DATACMD, (uint32_t)hi2c->pBuffPtr, hi2c->XferSize); 
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_EX_STATE_READY;
        hi2c->Mode      = HAL_I2C_EX_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_EX_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }

      if (dmaxferstatus == HAL_OK)
      {
        /* Send Slave Address */

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        /* Enable DMA Request */
				__HAL_I2C_EX_DISABLE(hi2c);
				
        hi2c->Instance->DMACR |= I2C_EX_DMACR_RDMAE;
           /* Set Slave Address */
	      if (hi2c->Init.AddressingMode == I2C_EX_ADDRESSINGMODE_MASTER_7BIT)
	      {
	      	/* Set slave 7bit address */
	      	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x7FUL));
	      }
	      else
	      {
	      	/* Set slave 10bit address */
	      	MODIFY_REG(hi2c->Instance->TAR, I2C_EX_TAR_ADDR, (DevAddress & 0x3FFUL)|I2C_EX_TAR_10BITADDR_MASTER);
	      }
        /* Init tickstart for timeout management*/
        __HAL_I2C_EX_ENABLE(hi2c);
        
        hi2c->Instance->DATACMD = (I2C_EX_CMD_READ);
        
         __HAL_I2C_EX_ENABLE_IT(hi2c, I2C_EX_IT_STOP_DET);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = HAL_I2C_EX_STATE_READY;
        hi2c->Mode      = HAL_I2C_EX_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_EX_ERROR_DMA;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }   
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Slave_Transmit_DMA(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef dmaxferstatus;
  if (hi2c->State == HAL_I2C_EX_STATE_READY)
	{
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_EX_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_EX_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_EX_Slave_ISR_DMA;
    if (hi2c->hdmatx != NULL)
    {
       /* Set the I2C DMA transfer complete callback */
       hi2c->hdmatx->XferTfrCallback = I2C_EX_DMASlaveTransmitCplt;
      /* Enable the DMA channel */
       dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)hi2c->pBuffPtr, (uint32_t)&hi2c->Instance->DATACMD, hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_EX_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_EX_MODE_NONE;
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    if (dmaxferstatus == HAL_OK)
    {
      /* Enable Address Acknowledge */
      hi2c->Instance->SLVDATANACKONLY &= ~I2C_EX_SLVDATANACKONLY_NACK;
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Enable STOP interrupt */
     __HAL_I2C_EX_ENABLE_IT(hi2c,I2C_EX_IT_START_DET|I2C_EX_IT_TX_ABRT|I2C_EX_IT_STOP_DET|I2C_EX_IT_RD_REQ);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_EX_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_EX_MODE_NONE;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    return HAL_OK;
	}
	else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EX_Slave_Receive_DMA(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == HAL_I2C_EX_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_EX_STATE_BUSY_RX;
    hi2c->Mode        = HAL_I2C_EX_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_EX_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr   = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_EX_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_EX_Slave_ISR_DMA;

    if (hi2c->hdmarx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmarx->XferTfrCallback = I2C_EX_DMASlaveReceiveCplt;

      /* Set the DMA error callback */
      hi2c->hdmarx->XferErrorCallback = NULL;

      /* Enable the DMA channel */
      dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->DATACMD, (uint32_t)hi2c->pBuffPtr, hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_EX_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_EX_MODE_NONE;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    if (dmaxferstatus == HAL_OK)
    {
      /* Enable Address Acknowledge */
      hi2c->Instance->SLVDATANACKONLY &= ~I2C_EX_SLVDATANACKONLY_NACK;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Enable STOPinterrupts */
      __HAL_I2C_EX_ENABLE_IT(hi2c, I2C_EX_IT_START_DET|I2C_EX_IT_STOP_DET);
			
			hi2c->Instance->DMACR |= I2C_EX_DMACR_RDMAE;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = HAL_I2C_EX_STATE_LISTEN;
      hi2c->Mode      = HAL_I2C_EX_MODE_NONE;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with DMA.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_EX_Master_ISR_DMA(struct __I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources)
{
  uint32_t tmpoptions = hi2c->XferOptions;
  UNUSED(tmpoptions);

  /* Process locked */
  __HAL_LOCK(hi2c);

  if ((I2C_EX_CHECK_FLAG(ITFlags, I2C_EX_ITFLAG_STOP_DET) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources, I2C_EX_IT_STOP_DET) != RESET))
  {
    /* Call I2C Slave complete process */
      I2C_EX_ITMasterCplt(hi2c, ITFlags);
      __HAL_I2C_EX_CLEAR_STOP_DET_ITFLAG(hi2c);
  }
  /* Process Unlocked */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}


/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with DMA.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_EX_Slave_ISR_DMA(struct __I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources)
{
  if((I2C_EX_CHECK_FLAG(ITFlags, I2C_EX_ITFLAG_START_DET) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources, I2C_EX_IT_START_DET) != RESET))
  {
      if(__HAL_I2C_EX_GET_IT_FLAG(hi2c,I2C_EX_ITFLAG_RESTART_DET) != RESET )
      {
         __HAL_I2C_EX_CLEAR_RESTART_DET_ITFLAG(hi2c);
         /*stop dma transmission */
         if(hi2c->State == HAL_I2C_EX_STATE_BUSY_RX)
         {
            HAL_DMA_Abort_IT(hi2c->hdmarx);
         }
         else
         {
            HAL_DMA_Abort_IT(hi2c->hdmatx);   
         }
         I2C_EX_ITSlaveSeqCplt(hi2c);
      }
      __HAL_I2C_EX_CLEAR_START_DET_ITFLAG(hi2c);
      __HAL_I2C_EX_ENABLE_IT(hi2c,I2C_EX_IT_TX_ABRT|I2C_EX_IT_RD_REQ|I2C_EX_IT_RX_FULL); 
  }
  
  if((I2C_EX_CHECK_FLAG(ITFlags, I2C_EX_ITFLAG_TX_ABRT) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources, I2C_EX_IT_TX_ABRT) != RESET))
  {
      __HAL_I2C_EX_CLEAR_TX_ABRT_ITFLAG(hi2c);
  }
  
	if((I2C_EX_CHECK_FLAG(ITFlags, I2C_EX_ITFLAG_RD_REQ) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources,I2C_EX_IT_RD_REQ)!= RESET))
  {
      hi2c->Instance->DMACR |= I2C_EX_DMACR_TDMAE;
        
      __HAL_I2C_EX_CLEAR_RD_REQ_ITFLAG(hi2c);
      __HAL_I2C_EX_DISABLE_IT(hi2c,I2C_EX_IT_RD_REQ);
  }
            
  if ((I2C_EX_CHECK_FLAG(ITFlags, I2C_EX_ITFLAG_STOP_DET) != RESET) && (I2C_EX_CHECK_IT_SOURCE(ITSources, I2C_EX_IT_STOP_DET) != RESET))
  {
      __HAL_I2C_EX_CLEAR_STOP_DET_ITFLAG(hi2c);
      /*stop dma transmission */
      if(hi2c->State == HAL_I2C_EX_STATE_BUSY_RX)
      {
        HAL_DMA_Abort_IT(hi2c->hdmarx);
      }
      else  if(hi2c->State == HAL_I2C_EX_STATE_BUSY_TX)
      {
        HAL_DMA_Abort_IT(hi2c->hdmatx);
      }
      
      I2C_EX_ITSlaveSeqCplt(hi2c);
      /* Call I2C Slave complete process */
      I2C_EX_ITSlaveCplt(hi2c, ITFlags);
  }
  return HAL_OK;
}



/**
  * @brief  DMA I2C Master transmit process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_EX_DMAMasterTransmitCplt(DMA_HandleTypeDef *hdma)
{
    I2C_EX_HandleTypeDef *hi2c = (I2C_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); 
    uint32_t tmpoptions = hi2c->XferOptions;
    UNUSED(tmpoptions);

     hi2c->Instance->DATACMD = hi2c->pBuffPtr[hi2c->XferSize-1] | I2C_EX_CMD_STOP;
    /* Disable DMA Request */
    hi2c->Instance->DMACR &= ~I2C_EX_DMACR_TDMAE;

    /* Last Byte is Transmitted */
    /* Call I2C Slave Sequential complete process */
    I2C_EX_ITMasterSeqCplt(hi2c);
 
}



/**
  * @brief  DMA I2C Master receive  process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_EX_DMAMasterReceiveCplt(DMA_HandleTypeDef *hdma)
{
    I2C_EX_HandleTypeDef *hi2c = (I2C_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); 
    uint32_t tmpoptions = hi2c->XferOptions;
    UNUSED(tmpoptions);

    /* Disable DMA Request */
    hi2c->Instance->DMACR &= ~I2C_EX_DMACR_RDMAE;

    /* Last Byte is Transmitted */
    /* Call I2C Slave Sequential complete process */
    I2C_EX_ITMasterSeqCplt(hi2c);
 
}

/**
  * @brief  DMA I2C Master transmit process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_EX_DMAMasterSrcTran(DMA_HandleTypeDef *hdma)
{
    I2C_EX_HandleTypeDef *hi2c = (I2C_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
    uint32_t tmpoptions = hi2c->XferOptions;
    UNUSED(tmpoptions);

    hi2c->XferCount--;
        
    if(hi2c->XferCount == 1)
    {
      hi2c->Instance->DATACMD = (I2C_EX_CMD_READ | I2C_EX_CMD_STOP);
         
    }
    else if(hi2c->XferCount > 1)
    {
       hi2c->Instance->DATACMD = (I2C_EX_CMD_READ);
    }
    else
    {
      /* Do nothing */
    }
}
/**
  * @brief  DMA I2C slave transmit process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_EX_DMASlaveTransmitCplt(DMA_HandleTypeDef *hdma)
{
    I2C_EX_HandleTypeDef *hi2c = (I2C_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent);
    uint32_t tmpoptions = hi2c->XferOptions;
    UNUSED(tmpoptions);

    __HAL_I2C_EX_CLEAR_RD_REQ_ITFLAG(hi2c);    
    /* Disable DMA Request */
    hi2c->Instance->DMACR &= ~I2C_EX_DMACR_TDMAE;
    /* Last Byte is Transmitted */
    /* Call I2C Slave Sequential complete process */
    I2C_EX_ITSlaveSeqCplt(hi2c);
}


/**
  * @brief  DMA I2C slave receive process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_EX_DMASlaveReceiveCplt(DMA_HandleTypeDef *hdma)
{
    I2C_EX_HandleTypeDef *hi2c = (I2C_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); 
    uint32_t tmpoptions = hi2c->XferOptions;
    UNUSED(tmpoptions);
	
    /* Disable DMA Request */
    hi2c->Instance->DMACR &= ~I2C_EX_DMACR_RDMAE;
    /* Call I2C Slave Sequential complete process */
    I2C_EX_ITSlaveSeqCplt(hi2c);
}

/**
  * @}
  */

/** @defgroup I2C_EX_Exported_Functions_Group3 Peripheral State, Mode and Error functions
 *  @brief   Peripheral State, Mode and Error functions
  *
@verbatim
 ===============================================================================
            ##### Peripheral State, Mode and Error functions #####
 ===============================================================================
    [..]
    This subsection permit to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the I2C_EX handle state.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C_EX.
  * @retval HAL state
  */
HAL_I2C_EX_StateTypeDef HAL_I2C_EX_GetState(I2C_EX_HandleTypeDef *hi2c)
{
  /* Return I2C_EX handle state */
  return hi2c->State;
}

/**
  * @brief  Returns the I2C_EX Master, Slave, Memory or no mode.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *         the configuration information for I2C_EX module
  * @retval HAL mode
  */
HAL_I2C_EX_ModeTypeDef HAL_I2C_EX_GetMode(I2C_EX_HandleTypeDef *hi2c)
{
  return hi2c->Mode;
}

/**
  * @brief  Return the I2C_EX error code.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C_EX.
  * @retval I2C_EX Error Code
  */
uint32_t HAL_I2C_EX_GetError(I2C_EX_HandleTypeDef *hi2c)
{
  return hi2c->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup I2C_EX_Private_Functions
  * @{
  */

/**
  * @brief  This function handles I2C_EX Communication Timeout.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *         the configuration information for I2C_EX module
  * @param  Flag specifies the I2C_EX flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_EX_WaitOnRawITFlagUntilTimeout(I2C_EX_HandleTypeDef *hi2c, uint32_t RawITFlag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
  /* Wait until flag is set */
  while (__HAL_I2C_EX_GET_RAW_IT_FLAG(hi2c, RawITFlag) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->PreviousState     = I2C_EX_STATE_NONE;
        hi2c->State             = HAL_I2C_EX_STATE_READY;
        hi2c->Mode              = HAL_I2C_EX_MODE_NONE;
        hi2c->ErrorCode         |= HAL_I2C_EX_ERROR_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles I2C_EX Communication Timeout.
  * @param  hi2c_ex Pointer to a I2C_EX_HandleTypeDef structure that contains
  *         the configuration information for I2C_EX module
  * @param  Flag specifies the I2C_EX flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_EX_WaitOnFlagUntilTimeout(I2C_EX_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
  /* Wait until flag is set */
  while (__HAL_I2C_EX_GET_FLAG(hi2c, Flag) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->PreviousState     = I2C_EX_STATE_NONE;
        hi2c->State             = HAL_I2C_EX_STATE_READY;
        hi2c->Mode              = HAL_I2C_EX_MODE_NONE;
        hi2c->ErrorCode         |= HAL_I2C_EX_ERROR_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
  }
  return HAL_OK;
}	
	
/**
  * @}
  */

#endif /* HAL_I2C_EX_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
  
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/

