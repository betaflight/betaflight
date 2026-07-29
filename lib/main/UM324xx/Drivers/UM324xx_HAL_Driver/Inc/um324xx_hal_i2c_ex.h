 /**
  ******************************************************************************
  * @file     um324xx_hal_i2c_ex.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_I2C_EX_H__
#define __UM324XX_HAL_I2C_EX_H__
                            



#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup I2C_EX
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup I2C_EX_EX_Exported_typedefs I2C_EX_EX Exported Typedefs
  * @{
  */ 

/** @defgroup I2C_EX_EX_Configuration_Structure_definition I2C_EX_EX Configuration Structure definition
  * @brief  I2C_EX_EX Configuration Structure definition
  * @{
  */
typedef struct
{
  uint32_t ClockSpeed;       /*!< Specifies the clock frequency.
                                  This parameter must be set to a value lower than 400kHz */

  uint32_t SlaveOwnAddress;  /*!< Specifies the slave device own address.
                                  This parameter can be a 7-bit or 10-bit address. */

  uint32_t AddressingMode;   /*!< Specifies if 7-bit or 10-bit addressing mode is selected.
                                  This parameter can be a value of @ref I2C_EX_addressing_mode */

  uint32_t GeneralCallMode;  /*!< Specifies if general call mode is selected.
                                  This parameter can be a value of @ref I2C_EX_general_call_addressing_mode */

  uint32_t MasterRestartMode; /*!< Specifies if master restart mode is selected.
                                   This parameter can be a value of @ref I2C_EX_master_restart_mode */

  uint32_t MasterBusClearMode; /*!< Specifies if master bus clear mode is selected.
                                    This parameter can be a value of @ref I2C_EX_master_bus_clear_mode */
	
} I2C_EX_InitTypeDef;

/**
  * @}
  */

/** @defgroup HAL_state_structure_definition HAL state structure definition
  * @brief  HAL State structure definition
  * @note  HAL I2C_EX State value coding follow below described bitmap :
  *          b7-b6  Error information
  *             00 : No Error
  *             01 : Abort (Abort user request on going)
  *             10 : Timeout
  *             11 : Error
  *          b5     Peripheral initialization status
  *             0  : Reset (Peripheral not initialized)
  *             1  : Init done (Peripheral initialized and ready to use. HAL I2C_EX Init function called)
  *          b4     (not used)
  *             x  : Should be set to 0
  *          b3
  *             0  : Ready or Busy (No Listen mode ongoing)
  *             1  : Listen (Peripheral in Address Listen Mode)
  *          b2     Intrinsic process state
  *             0  : Ready
  *             1  : Busy (Peripheral busy with some configuration or internal operations)
  *          b1     Rx state
  *             0  : Ready (no Rx operation ongoing)
  *             1  : Busy (Rx operation ongoing)
  *          b0     Tx state
  *             0  : Ready (no Tx operation ongoing)
  *             1  : Busy (Tx operation ongoing)
  * @{
  */
typedef enum
{
  HAL_I2C_EX_STATE_RESET             = 0x00U,   /*!< Peripheral is not yet Initialized         */
  HAL_I2C_EX_STATE_READY             = 0x20U,   /*!< Peripheral Initialized and ready for use  */
  HAL_I2C_EX_STATE_BUSY              = 0x24U,   /*!< An internal process is ongoing            */
  HAL_I2C_EX_STATE_BUSY_TX           = 0x21U,   /*!< Data Transmission process is ongoing      */
  HAL_I2C_EX_STATE_BUSY_RX           = 0x22U,   /*!< Data Reception process is ongoing         */
  HAL_I2C_EX_STATE_LISTEN            = 0x28U,   /*!< Address Listen Mode is ongoing            */
  HAL_I2C_EX_STATE_BUSY_TX_LISTEN    = 0x29U,   /*!< Address Listen Mode and Data Transmission
                                                 process is ongoing                         */
  HAL_I2C_EX_STATE_BUSY_RX_LISTEN    = 0x2AU,   /*!< Address Listen Mode and Data Reception
                                                 process is ongoing                         */
  HAL_I2C_EX_STATE_ABORT             = 0x60U,   /*!< Abort user request ongoing                */
  HAL_I2C_EX_STATE_TIMEOUT           = 0xA0U,   /*!< Timeout state                             */
  HAL_I2C_EX_STATE_ERROR             = 0xE0U    /*!< Error                                     */

} HAL_I2C_EX_StateTypeDef;

/**
  * @}
  */

/** @defgroup HAL_mode_structure_definition HAL mode structure definition
  * @brief  HAL Mode structure definition
  * @note  HAL I2C_EX Mode value coding follow below described bitmap :\n
  *          b7     (not used)\n
  *             x  : Should be set to 0\n
  *          b6\n
  *             0  : None\n
  *             1  : Memory (HAL I2C_EX communication is in Memory Mode)\n
  *          b5\n
  *             0  : None\n
  *             1  : Slave (HAL I2C_EX communication is in Slave Mode)\n
  *          b4\n
  *             0  : None\n
  *             1  : Master (HAL I2C_EX communication is in Master Mode)\n
  *          b3-b2-b1-b0  (not used)\n
  *             xxxx : Should be set to 0000
  * @{
  */
typedef enum
{
  HAL_I2C_EX_MODE_NONE               = 0x00U,   /*!< No I2C_EX communication on going             */
  HAL_I2C_EX_MODE_MASTER             = 0x10U,   /*!< I2C_EX communication is in Master Mode       */
  HAL_I2C_EX_MODE_SLAVE              = 0x20U,   /*!< I2C_EX communication is in Slave Mode        */
  HAL_I2C_EX_MODE_MEM                = 0x40U    /*!< I2C_EX communication is in Memory Mode       */

} HAL_I2C_EX_ModeTypeDef;

/**
  * @}
  */

/** @defgroup I2C_EX_Error_Code_definition I2C_EX Error Code definition
  * @brief  I2C_EX Error Code definition
  * @{
  */
#define HAL_I2C_EX_ERROR_NONE              0x00000000U    /*!< No error              */
#define HAL_I2C_EX_ERROR_BERR              0x00000001U    /*!< BERR error            */
#define HAL_I2C_EX_ERROR_ARLO              0x00000002U    /*!< ARLO error            */
#define HAL_I2C_EX_ERROR_AF                0x00000004U    /*!< AF error              */
#define HAL_I2C_EX_ERROR_OVR               0x00000008U    /*!< OVR error             */
#define HAL_I2C_EX_ERROR_DMA               0x00000010U    /*!< DMA transfer error    */
#define HAL_I2C_EX_ERROR_TIMEOUT           0x00000020U    /*!< Timeout Error         */
#define HAL_I2C_EX_ERROR_SIZE              0x00000040U    /*!< Size Management error */
#define HAL_I2C_EX_ERROR_DMA_PARAM         0x00000080U    /*!< DMA Parameter Error   */
#define HAL_I2C_EX_WRONG_START             0x00000200U    /*!< Wrong start Error     */
#if (USE_HAL_I2C_EX_REGISTER_CALLBACKS == 1)
#define HAL_I2C_EX_ERROR_INVALID_CALLBACK  0x00000100U    /*!< Invalid Callback error */
#endif /* USE_HAL_I2C_EX_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup I2C_EX_handle_Structure_definition I2C_EX handle Structure definition
  * @brief  I2C_EX handle Structure definition
  * @{
  */
typedef struct __I2C_EX_HandleTypeDef
{
  I2C_EX_TypeDef                *Instance;      /*!< I2C_EX registers base address               */

  I2C_EX_InitTypeDef            Init;           /*!< I2C_EX communication parameters             */

  uint8_t                    *pBuffPtr;      /*!< Pointer to I2C_EX transfer buffer           */

  uint16_t                   XferSize;       /*!< I2C_EX transfer size                        */

  __IO uint16_t              XferCount;      /*!< I2C_EX transfer counter                     */

  __IO uint32_t              XferOptions;    /*!< I2C_EX transfer options                     */

  __IO uint32_t              PreviousState;  /*!< I2C_EX communication Previous state and mode
                                                  context for internal usage               */

  DMA_HandleTypeDef          *hdmatx;        /*!< I2C_EX Tx DMA handle parameters             */

  DMA_HandleTypeDef          *hdmarx;        /*!< I2C_EX Rx DMA handle parameters             */

  HAL_LockTypeDef            Lock;           /*!< I2C_EX locking object                       */

  __IO HAL_I2C_EX_StateTypeDef  State;          /*!< I2C_EX communication state                  */

  __IO HAL_I2C_EX_ModeTypeDef   Mode;           /*!< I2C_EX communication mode                   */

  __IO uint32_t              ErrorCode;      /*!< I2C_EX Error code                           */

  __IO uint32_t              Devaddress;     /*!< I2C_EX Target device address                */

  __IO uint32_t              Memaddress;     /*!< I2C_EX Target memory address                */

  __IO uint32_t              MemaddSize;     /*!< I2C_EX Target memory address  size          */

  __IO uint32_t              EventCount;     /*!< I2C_EX Event counter                        */
	
   HAL_StatusTypeDef(*XferISR)(struct __I2C_EX_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);/*!< I2C transfer IRQ handler function pointer */
  
#if (USE_HAL_I2C_EX_REGISTER_CALLBACKS == 1)
  void (* MasterTxCpltCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);           /*!< I2C_EX Master Tx Transfer completed callback */
  void (* MasterRxCpltCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);           /*!< I2C_EX Master Rx Transfer completed callback */
  void (* SlaveTxCpltCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);            /*!< I2C_EX Slave Tx Transfer completed callback  */
  void (* SlaveRxCpltCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);            /*!< I2C_EX Slave Rx Transfer completed callback  */
  void (* ListenCpltCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);             /*!< I2C_EX Listen Complete callback              */
  void (* MemTxCpltCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);              /*!< I2C_EX Memory Tx Transfer completed callback */
  void (* MemRxCpltCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);              /*!< I2C_EX Memory Rx Transfer completed callback */
  void (* ErrorCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);                  /*!< I2C_EX Error callback                        */
  void (* AbortCpltCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);              /*!< I2C_EX Abort callback                        */

  void (* AddrCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex, uint8_t TransferDirection, uint16_t AddrMatchCode);  /*!< I2C_EX Slave Address Match callback */

  void (* MspInitCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);                /*!< I2C_EX Msp Init callback                     */
  void (* MspDeInitCallback)(struct __I2C_EX_HandleTypeDef *hi2c_ex);              /*!< I2C_EX Msp DeInit callback                   */

#endif  /* USE_HAL_I2C_EX_REGISTER_CALLBACKS */
} I2C_EX_HandleTypeDef;

#if (USE_HAL_I2C_EX_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL I2C_EX Callback ID enumeration definition
  */
typedef enum
{
  HAL_I2C_EX_MASTER_TX_COMPLETE_CB_ID      = 0x00U,    /*!< I2C_EX Master Tx Transfer completed callback ID  */
  HAL_I2C_EX_MASTER_RX_COMPLETE_CB_ID      = 0x01U,    /*!< I2C_EX Master Rx Transfer completed callback ID  */
  HAL_I2C_EX_SLAVE_TX_COMPLETE_CB_ID       = 0x02U,    /*!< I2C_EX Slave Tx Transfer completed callback ID   */
  HAL_I2C_EX_SLAVE_RX_COMPLETE_CB_ID       = 0x03U,    /*!< I2C_EX Slave Rx Transfer completed callback ID   */
  HAL_I2C_EX_LISTEN_COMPLETE_CB_ID         = 0x04U,    /*!< I2C_EX Listen Complete callback ID               */
  HAL_I2C_EX_MEM_TX_COMPLETE_CB_ID         = 0x05U,    /*!< I2C_EX Memory Tx Transfer callback ID            */
  HAL_I2C_EX_MEM_RX_COMPLETE_CB_ID         = 0x06U,    /*!< I2C_EX Memory Rx Transfer completed callback ID  */
  HAL_I2C_EX_ERROR_CB_ID                   = 0x07U,    /*!< I2C_EX Error callback ID                         */
  HAL_I2C_EX_ABORT_CB_ID                   = 0x08U,    /*!< I2C_EX Abort callback ID                         */

  HAL_I2C_EX_MSPINIT_CB_ID                 = 0x09U,    /*!< I2C_EX Msp Init callback ID                      */
  HAL_I2C_EX_MSPDEINIT_CB_ID               = 0x0AU     /*!< I2C_EX Msp DeInit callback ID                    */

} HAL_I2C_EX_CallbackIDTypeDef;

/**
  * @brief  HAL I2C_EX Callback pointer definition
  */
typedef  void (*pI2C_EX_CallbackTypeDef)(I2C_EX_HandleTypeDef *hi2c_ex); /*!< pointer to an I2C_EX callback function */
typedef  void (*pI2C_EX_AddrCallbackTypeDef)(I2C_EX_HandleTypeDef *hi2c_ex, uint8_t TransferDirection, uint16_t AddrMatchCode); /*!< pointer to an I2C_EX Address Match callback function */

#endif /* USE_HAL_I2C_EX_REGISTER_CALLBACKS */
/**
  * @}
  */

/**
  * @}
  */   

/* Exported constants --------------------------------------------------------*/
/** @defgroup I2C_EX_Exported_constants I2C_EX Exported Constants
  * @{
  */ 
/** @defgroup HAL_Speed_structure_definition HAL mode structure definition
  * @brief I2C MODE enumeration
  * @{
  */
typedef enum
{
    I2C_EX_SPEED_STAD_100K = 0,
    I2C_EX_SPEED_FAST_400K,
    I2C_EX_SPEED_HIGH_1M
}i2c_ex_speed_typedef;

/** @defgroup I2C_EX_clock_speed I2C_EX clock speed
  * @{
  */
#define I2C_EX_ClockSpeed_Standard			I2C_EX_CR_SPEED_0
#define I2C_EX_ClockSpeed_Fast				  I2C_EX_CR_SPEED_1
/**
  * @}
  */

/** @defgroup I2C_EX_addressing_mode I2C_EX addressing mode
  * @{
  */
#define I2C_EX_ADDRESSINGMODE_MASTER_7BIT        0x00000041U
#define I2C_EX_ADDRESSINGMODE_MASTER_10BIT       0x00000051U
#define I2C_EX_ADDRESSINGMODE_SLAVE_7BIT         0x00000000U
#define I2C_EX_ADDRESSINGMODE_SLAVE_10BIT        0x00000008U
/**
  * @}
  */

/** @defgroup I2C_EX_general_call_addressing_mode I2C_EX general call addressing mode
  * @{
  */
#define I2C_EX_GENERALCALL_DISABLE        0x00000000U
#define I2C_EX_GENERALCALL_ENABLE         I2C_EX_TAR_SPECIAL
/**
  * @}
  */

/** @defgroup I2C_EX_master_restart_mode I2C_EX master restart mode
  * @{
  */
#define I2C_EX_MASTER_RESTART_DISABLE     0x00000000U
#define I2C_EX_MASTER_RESTART_ENABLE      I2C_EX_CR_RESTART_EN
/**
  * @}
  */
 
/** @defgroup I2C_EX_master_bus_clear_mode I2C_EX master bus_clear mode
  * @{
  */
#define I2C_EX_MASTER_BUS_CLEAR_DISABLE     I2C_EX_CR_BUS_CLEAR_FEATURE_CTRL
#define I2C_EX_MASTER_BUS_CLEAR_ENABLE      0x00000000U
/**
  * @}
  */
  
/** @defgroup I2C_EX_Memory_Address_Size I2C_EX Memory Address Size
  * @{
  */
#define I2C_EX_MEMADD_SIZE_8BIT            0x00000001U
#define I2C_EX_MEMADD_SIZE_16BIT           0x00000010U
/**
  * @}
  */

/** @defgroup I2C_EX_CMD_WRITE_OR_READ I2C_EX cmd write or read
  * @{
  */
#define I2C_EX_CMD_WRITE				0x00000000U
#define I2C_EX_CMD_READ				  I2C_EX_DATACMD_CMD
#define I2C_EX_CMD_STOP				  I2C_EX_DATACMD_STOP
/**
  * @}
  */
  
/** @defgroup I2C_EX_XferDirection_definition I2C_EX XferDirection definition
  * @{
  */
#define I2C_EX_DIRECTION_RECEIVE           0x00000000U
#define I2C_EX_DIRECTION_TRANSMIT          0x00000001U
/**
  * @}
  */

/** @defgroup I2C_EX_XferOptions_definition I2C_EX XferOptions definition
  * @{
  */
#define  I2C_EX_FIRST_FRAME                0x00000001U
#define  I2C_EX_FIRST_AND_NEXT_FRAME       0x00000002U
#define  I2C_EX_NEXT_FRAME                 0x00000004U
#define  I2C_EX_FIRST_AND_LAST_FRAME       0x00000008U
#define  I2C_EX_LAST_FRAME_NO_STOP         0x00000010U
#define  I2C_EX_LAST_FRAME                 0x00000020U

/* List of XferOptions in usage of :
 * 1- Restart condition in all use cases (direction change or not)
 */
#define  I2C_EX_OTHER_FRAME                (0x00AA0000U)
#define  I2C_EX_OTHER_AND_LAST_FRAME       (0xAA000000U)
/**
  * @}
  */

/** @defgroup I2C_EX_Interrupt_configuration_definition I2C_EX Interrupt configuration definition
  * @{
  */
#define I2C_EX_IT_CEN_CALL                 I2C_EX_INTMASK_GEN_CALL
#define I2C_EX_IT_START_DET                I2C_EX_INTMASK_START_DET
#define I2C_EX_IT_STOP_DET                 I2C_EX_INTMASK_STOP_DET
#define I2C_EX_IT_ACTIVITY                 I2C_EX_INTMASK_ACTIVITY
#define I2C_EX_IT_RX_DONE                  I2C_EX_INTMASK_RX_DONE
#define I2C_EX_IT_TX_ABRT                  I2C_EX_INTMASK_TX_ABRT
#define I2C_EX_IT_RD_REQ                   I2C_EX_INTMASK_RD_REQ
#define I2C_EX_IT_TX_EMPTY                 I2C_EX_INTMASK_TX_EMPTY
#define I2C_EX_IT_TX_OVER                  I2C_EX_INTMASK_TX_OVER
#define I2C_EX_IT_RX_FULL                  I2C_EX_INTMASK_RX_FULL
#define I2C_EX_IT_RX_OVER                  I2C_EX_INTMASK_RX_OVER
#define I2C_EX_IT_RX_UNDER                 I2C_EX_INTMASK_RX_UNDER
#define I2C_EX_IT_RESTART_DET              I2C_EX_INTMASK_RESTART_DET
#define I2C_EX_IT_MST_ON_HOLD              I2C_EX_INTMASK_MST_ON_HOLD
#define I2C_EX_IT_SCL_STUCK_AT_LOW         I2C_EX_INTMASK_SCL_STUCK_AT_LOW
#define I2C_EX_IT_MASK_ALL                 (0x00007FFFU)
/**
  * @}
  */

/** @defgroup I2C_EX_ITFlag_definition I2C_EX IT Flag definition
  * @{
  */
#define I2C_EX_ITFLAG_CEN_CALL              I2C_EX_ISR_GEN_CALL
#define I2C_EX_ITFLAG_START_DET             I2C_EX_ISR_START_DET
#define I2C_EX_ITFLAG_STOP_DET              I2C_EX_ISR_STOP_DET
#define I2C_EX_ITFLAG_ACTIVITY              I2C_EX_ISR_ACTIVITY
#define I2C_EX_ITFLAG_RX_DONE               I2C_EX_ISR_RX_DONE
#define I2C_EX_ITFLAG_TX_ABRT               I2C_EX_ISR_TX_ABRT
#define I2C_EX_ITFLAG_RD_REQ                I2C_EX_ISR_RD_REQ
#define I2C_EX_ITFLAG_TX_EMPTY              I2C_EX_ISR_TX_EMPTY
#define I2C_EX_ITFLAG_TX_OVER               I2C_EX_ISR_TX_OVER
#define I2C_EX_ITFLAG_RX_FULL               I2C_EX_ISR_RX_FULL
#define I2C_EX_ITFLAG_RX_OVER               I2C_EX_ISR_RX_OVER
#define I2C_EX_ITFLAG_RX_UNDER              I2C_EX_ISR_RX_UNDER
#define I2C_EX_ITFLAG_RESTART_DET           I2C_EX_ISR_RESTART_DET
#define I2C_EX_ITFLAG_I2C1_ISR_MST_ON_HOLD  I2C_EX_ISR_MST_ON_HOLD
#define I2C_EX_ITFLAG_SCL_STUCK_AT_LOW      I2C_EX_ISR_SCL_STUCK_AT_LOW
/**
  * @}
  */
  
/** @defgroup I2C_EX_ITFlag_definition I2C_EX RAW IT Flag definition
  * @{
  */

#define I2C_EX_RAW_ITFLAG_CEN_CALL          I2C_EX_RAWISR_GEN_CALL
#define I2C_EX_RAW_ITFLAG_START_DET         I2C_EX_RAWISR_START_DET
#define I2C_EX_RAW_ITFLAG_STOP_DET          I2C_EX_RAWISR_STOP_DET
#define I2C_EX_RAW_ITFLAG_ACTIVITY          I2C_EX_RAWISR_ACTIVITY
#define I2C_EX_RAW_ITFLAG_RX_DONE           I2C_EX_RAWISR_RX_DONE
#define I2C_EX_RAW_ITFLAG_TX_ABRT           I2C_EX_RAWISR_TX_ABRT
#define I2C_EX_RAW_ITFLAG_RD_REQ            I2C_EX_RAWISR_RD_REQ
#define I2C_EX_RAW_ITFLAG_TX_EMPTY          I2C_EX_RAWISR_TX_EMPTY
#define I2C_EX_RAW_ITFLAG_TX_OVER           I2C_EX_RAWISR_TX_OVER
#define I2C_EX_RAW_ITFLAG_RX_FULL           I2C_EX_RAWISR_RX_FULL
#define I2C_EX_RAW_ITFLAG_RX_OVER           I2C_EX_RAWISR_RX_OVER
#define I2C_EX_RAW_ITFLAG_RX_UNDER          I2C_EX_RAWISR_RX_UNDER
#define I2C_EX_RAW_ITFLAG__RESTART          I2C_EX_RAWISR_RESTART_DET
#define I2C_EX_RAW_ITFLAG__MST_ON_HOLD      I2C_EX_RAWISR_MST_ON_HOLD
#define I2C_EX_RAW_ITFLAG__SCL_STUCK_AT_LOW I2C_EX_RAWISR_SCL_STUCK_AT_LOW

/**
  * @}
  */

/** @defgroup I2C_EX_Flag_definition I2C_EX Flag definition
  * @{
  */
#define I2C_EX_FLAG_SMBUS_SLAVE_ADDR_RESOLVED	  I2C_EX_SR_SMBUS_SLAVE_ADDR_RESOLVED	
#define I2C_EX_FLAG_SMBUS_SLAVE_ADDR_VALID		  I2C_EX_SR_SMBUS_SLAVE_ADDR_VALID		
#define I2C_EX_FLAG_SMBUS_QUICK_CMD			        I2C_EX_SR_SMBUS_QUICK_CMD			
#define I2C_EX_FLAG_SDA_STUCK_NOT_RECOVERED	    I2C_EX_SR_SDA_STUCK_NOT_RECOVERED	
#define I2C_EX_FLAG_SLV_ACTIVITY				        I2C_EX_SR_SLV_ACTIVITY				
#define I2C_EX_FLAG_MST_ACTIVITY				        I2C_EX_SR_MST_ACTIVITY				
#define I2C_EX_FLAG_RFF						              I2C_EX_SR_RFF						
#define I2C_EX_FLAG_RFNE						            I2C_EX_SR_RFNE						
#define I2C_EX_FLAG_TFE						              I2C_EX_SR_TFE						
#define I2C_EX_FLAG_TFNF						            I2C_EX_SR_TFNF						
#define I2C_EX_FLAG_ACTIVITY					          I2C_EX_SR_ACTIVITY
/**
  * @}
  */
  
/** @defgroup I2C_EX_Flag_definition I2C_EX Flag definition
  * @{
  */	
#define I2C_SMBUS_IT_M_SLV_CLOCK_EXTND_TIMEOUT          I2C_EX_SMBUSINTMASK_M_SLV_CLOCK_EXTND_TIMEOUT
#define I2C_SMBUS_IT_M_MST_CLOCK_EXTND_TIMEOUT          I2C_EX_SMBUSINTMASK_M_MST_CLOCK_EXTND_TIMEOUT
#define I2C_SMBUS_IT_M_QUICK_CMD_DET                    I2C_EX_SMBUSINTMASK_M_QUICK_CMD_DET
#define I2C_SMBUS_IT_M_HOST_NOTIFY_MST_DET              I2C_EX_SMBUSINTMASK_M_HOST_NOTIFY_MST_DET
#define I2C_SMBUS_IT_M_ARP_PREPARE_CMD_DET              I2C_EX_SMBUSINTMASK_M_ARP_PREPARE_CMD_DET
#define I2C_SMBUS_IT_M_ARP_RST_CMD_DET                  I2C_EX_SMBUSINTMASK_M_ARP_RST_CMD_DET
#define I2C_SMBUS_IT_M_ARP_GET_UDID_CMD_DET             I2C_EX_SMBUSINTMASK_M_ARP_GET_UDID_CMD_DET
#define I2C_SMBUS_IT_M_ARP_ASSGN_ADDM_CMD_DET           I2C_EX_SMBUSINTMASK_M_ARP_ASSGN_ADDM_CMD_DET

#define I2C_SMBUS_IT_MASK_ALL                           (0x000001FFU)
/**
  * @}
  */
	
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup I2C_EX_Exported_macro I2C_EX Exported Macro
  * @{
  */ 

/** @brief Reset I2C_EX handle state.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#if (USE_HAL_I2C_EX_REGISTER_CALLBACKS == 1)
#define __HAL_I2C_EX_RESET_HANDLE_STATE(__HANDLE__)                do{                                                   \
                                                                    (__HANDLE__)->State = HAL_I2C_EX_STATE_RESET;       \
                                                                    (__HANDLE__)->MspInitCallback = NULL;            \
                                                                    (__HANDLE__)->MspDeInitCallback = NULL;          \
                                                                  } while(0)
#else
#define __HAL_I2C_EX_RESET_HANDLE_STATE(__HANDLE__)                ((__HANDLE__)->State = HAL_I2C_EX_STATE_RESET)
#endif

/** @brief  Enable or disable the specified I2C_EX interrupts.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to enable or disable.
  *         This parameter can be one of the following values:
  *            @arg I2C_EX_IT_CEN_CALL 
  *            @arg I2C_EX_IT_START_DET
  *            @arg I2C_EX_IT_STOP_DET 
  *            @arg I2C_EX_IT_ACTIVITY 
  *            @arg I2C_EX_IT_RX_DONE  
  *            @arg I2C_EX_IT_TX_ABRT   
  *            @arg I2C_EX_IT_RD_REQ   
  *            @arg I2C_EX_IT_TX_EMPTY 
  *            @arg I2C_EX_IT_TX_OVER   
  *            @arg I2C_EX_IT_RX_FULL  
  *            @arg I2C_EX_IT_RX_OVER  
  *            @arg I2C_EX_IT_RX_UNDER   
  * @retval None
  */
#define __HAL_I2C_EX_ENABLE_IT(__HANDLE__, __INTERRUPT__)   SET_BIT((__HANDLE__)->Instance->INTMASK,(__INTERRUPT__))
#define __HAL_I2C_EX_DISABLE_IT(__HANDLE__, __INTERRUPT__)  CLEAR_BIT((__HANDLE__)->Instance->INTMASK, (__INTERRUPT__))

/** @brief  Enable or disable the specified I2C_EX interrupts.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to enable or disable.
  *         This parameter can be one of the following values:
  *            @arg I2C_SMBUS_IT_M_SLV_CLOCK_EXTND_TIMEOUT
  *            @arg I2C_SMBUS_IT_M_MST_CLOCK_EXTND_TIMEOUT
  *            @arg I2C_SMBUS_IT_M_QUICK_CMD_DET          
  *            @arg I2C_SMBUS_IT_M_HOST_NOTIFY_MST_DET    
  *            @arg I2C_SMBUS_IT_M_ARP_PREPARE_CMD_DET    
  *            @arg I2C_SMBUS_IT_M_ARP_RST_CMD_DET        
  *            @arg I2C_SMBUS_IT_M_ARP_GET_UDID_CMD_DET   
  *            @arg I2C_SMBUS_IT_M_ARP_ASSGN_ADDM_CMD_DET 
  *            @arg I2C_SMBUS_IT_MASK_ALL                 
  * @retval None
  */
#define __HAL_I2C_EX_SMBUS_ENABLE_IT(__HANDLE__, __INTERRUPT__)   SET_BIT((__HANDLE__)->Instance->SMBUSINTMASK,(__INTERRUPT__))
#define __HAL_I2C_EX_SMBUS_DISABLE_IT(__HANDLE__, __INTERRUPT__)  CLEAR_BIT((__HANDLE__)->Instance->SMBUSINTMASK, (__INTERRUPT__))

/** @brief  Checks if the specified I2C_EX interrupt source is enabled or disabled.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @param  __INTERRUPT__ specifies the I2C_EX interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg I2C_EX_IT_CEN_CALL 
  *            @arg I2C_EX_IT_START_DET
  *            @arg I2C_EX_IT_STOP_DET 
  *            @arg I2C_EX_IT_ACTIVITY 
  *            @arg I2C_EX_IT_RX_DONE  
  *            @arg I2C_EX_IT_TX_ABRT   
  *            @arg I2C_EX_IT_RD_REQ   
  *            @arg I2C_EX_IT_TX_EMPTY 
  *            @arg I2C_EX_IT_TX_OVER   
  *            @arg I2C_EX_IT_RX_FULL  
  *            @arg I2C_EX_IT_RX_OVER  
  *            @arg I2C_EX_IT_RX_UNDER  
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __HAL_I2C_EX_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->INTMASK & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Checks whether the specified I2C_EX interrupt flag is set or not.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg I2C_EX_ITFLAG_CEN_CALL 
  *            @arg I2C_EX_ITFLAG_START_DET
  *            @arg I2C_EX_ITFLAG_STOP_DET 
  *            @arg I2C_EX_ITFLAG_ACTIVITY 
  *            @arg I2C_EX_ITFLAG_RX_DONE  
  *            @arg I2C_EX_ITFLAG_TX_ABRT  
  *            @arg I2C_EX_ITFLAG_RD_REQ   
  *            @arg I2C_EX_ITFLAG_TX_EMPTY 
  *            @arg I2C_EX_ITFLAG_TX_OVER  
  *            @arg I2C_EX_ITFLAG_RX_FULL  
  *            @arg I2C_EX_ITFLAG_RX_OVER  
  *            @arg I2C_EX_ITFLAG_RX_UNDER 
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_I2C_EX_GET_IT_FLAG(__HANDLE__, __FLAG__) (((((__HANDLE__)->Instance->ISR) & (__FLAG__)) == (__FLAG__)) ? SET : RESET)

/** @brief  Checks whether the specified I2C_EX raw interrupt flag is set or not.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg I2C_EX_RAW_ITFLAG_CEN_CALL 
  *            @arg I2C_EX_RAW_ITFLAG_START_DET
  *            @arg I2C_EX_RAW_ITFLAG_STOP_DET 
  *            @arg I2C_EX_RAW_ITFLAG_ACTIVITY 
  *            @arg I2C_EX_RAW_ITFLAG_RX_DONE  
  *            @arg I2C_EX_RAW_ITFLAG_TX_ABRT  
  *            @arg I2C_EX_RAW_ITFLAG_RD_REQ   
  *            @arg I2C_EX_RAW_ITFLAG_TX_EMPTY 
  *            @arg I2C_EX_RAW_ITFLAG_TX_OVER  
  *            @arg I2C_EX_RAW_ITFLAG_RX_FULL  
  *            @arg I2C_EX_RAW_ITFLAG_RX_OVER  
  *            @arg I2C_EX_RAW_ITFLAG_RX_UNDER 
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_I2C_EX_GET_RAW_IT_FLAG(__HANDLE__, __FLAG__)		(((((__HANDLE__)->Instance->RAWISR) & (__FLAG__)) == (__FLAG__)) ? SET : RESET)
												  
/** @brief  Checks whether the specified I2C_EX flag is set or not.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg I2C_EX_FLAG_SMBUS_SLAVE_ADDR_RESOLVED	
  *            @arg I2C_EX_FLAG_SMBUS_SLAVE_ADDR_VALID		
  *            @arg I2C_EX_FLAG_SMBUS_QUICK_CMD			
  *            @arg I2C_EX_FLAG_SDA_STUCK_NOT_RECOVERED	
  *            @arg I2C_EX_FLAG_SLV_ACTIVITY				
  *            @arg I2C_EX_FLAG_MST_ACTIVITY				
  *            @arg I2C_EX_FLAG_RFF						
  *            @arg I2C_EX_FLAG_RFNE						
  *            @arg I2C_EX_FLAG_TFE						
  *            @arg I2C_EX_FLAG_TFNF						
  *            @arg I2C_EX_FLAG_ACTIVITY					
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_I2C_EX_GET_FLAG(__HANDLE__, __FLAG__)		(((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__)) ? SET : RESET)

/** @brief  Clears the I2C_EX all interrupt flag.
			Do not clear interrupts that can be automatically cleared by hardware, 
			only clear interrupts that can be cleared by software.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  *         This parameter can be I2C_EX where x: 0, 1, or 2 to select the I2C_EX peripheral.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_ALL_ITFLAG(__HANDLE__)  \
  do{                                           \
    __IO uint32_t tmpreg = 0x00U;               \
    tmpreg = (__HANDLE__)->Instance->CLR;       \
    UNUSED(tmpreg);                             \
  } while(0)

/** @brief  Clears the I2C_EX RX_UNDER interrupt flag.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_RX_UNDER_ITFLAG(__HANDLE__)	   \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRRXUNDER;	   \
    UNUSED(tmpreg);                                    \
  } while(0)
  
/** @brief  Clears the I2C_EX RX_OVER interrupt flag.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_RX_OVER_ITFLAG(__HANDLE__)	   \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRRXOVER;	   \
    UNUSED(tmpreg);                                    \
  } while(0)

/** @brief  Clears the I2C_EX TX_OVER interrupt flag.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_TX_OVER_ITFLAG(__HANDLE__)	   \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRTXOVER;	   \
    UNUSED(tmpreg);                                    \
  } while(0)

/** @brief  Clears the I2C_EX RD_REQ interrupt flag.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_RD_REQ_ITFLAG(__HANDLE__)	   \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRRDREQ;	   \
    UNUSED(tmpreg);                                    \
  } while(0)

/** @brief  Clears the I2C_EX TX_ABRT interrupt flag.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_TX_ABRT_ITFLAG(__HANDLE__)	   \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRTXABRT;	   \
    UNUSED(tmpreg);                                    \
  } while(0)

/** @brief  Clears the I2C_EX RX_DONE interrupt flag.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_RX_DONE_ITFLAG(__HANDLE__)	   \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRRXDONE;	   \
    UNUSED(tmpreg);                                    \
  } while(0)  
  
/** @brief  Clears the I2C_EX ACTIVITY interrupt flag.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_ACTIVITY_ITFLAG(__HANDLE__)	   \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRACTIVITY;	   \
    UNUSED(tmpreg);                                    \
  } while(0)   
  
/** @brief  Clears the I2C_EX STOP_DET interrupt flag.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_STOP_DET_ITFLAG(__HANDLE__)	   \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRSTOPDET;	   \
    UNUSED(tmpreg);                                    \
  } while(0)    
  
/** @brief  Clears the I2C_EX START_DET interrupt flag.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_START_DET_ITFLAG(__HANDLE__)	   \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRSTARTDET;	   \
    UNUSED(tmpreg);                                    \
  } while(0)     
  
/** @brief  Clears the I2C_EX GEN_CALL interrupt flag.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_GEN_CALL_ITFLAG(__HANDLE__)	   \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRGENCALL;	   \
    UNUSED(tmpreg);                                    \
  } while(0)      
	
	/** @brief  Clears the I2C RESTART interrupt flag.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @retval None
  */
#define __HAL_I2C_EX_CLEAR_RESTART_DET_ITFLAG(__HANDLE__)     \
  do{                                                  \
    __IO uint32_t tmpreg = 0x00U;                      \
    tmpreg = (__HANDLE__)->Instance->CLRRD;     \
    UNUSED(tmpreg);                                    \
  } while(0)
  
/** @brief  Enable the specified I2C_EX peripheral.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_ENABLE(__HANDLE__)                  SET_BIT((__HANDLE__)->Instance->EN, I2C_EX_EN_ENABLE)

/** @brief  Disable the specified I2C_EX peripheral.
  * @param  __HANDLE__ specifies the I2C_EX Handle.
  * @retval None
  */
#define __HAL_I2C_EX_DISABLE(__HANDLE__)                 CLEAR_BIT((__HANDLE__)->Instance->EN, I2C_EX_EN_ENABLE)


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2C_EX_Exported_Functions
  * @{
  */ 

/** @addtogroup I2C_EX_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
/* Initialization and de-initialization functions******************************/
HAL_StatusTypeDef HAL_I2C_EX_Init(I2C_EX_HandleTypeDef *hi2c_ex);
HAL_StatusTypeDef HAL_I2C_EX_DeInit(I2C_EX_HandleTypeDef *hi2c_ex);
void HAL_I2C_EX_MspInit(I2C_EX_HandleTypeDef *hi2c_ex);
void HAL_I2C_EX_MspDeInit(I2C_EX_HandleTypeDef *hi2c_ex);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_HAL_I2C_EX_REGISTER_CALLBACKS == 1)
HAL_StatusTypeDef HAL_I2C_EX_RegisterCallback(I2C_EX_HandleTypeDef *hi2c_ex, HAL_I2C_EX_CallbackIDTypeDef CallbackID, pI2C_EX_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_I2C_EX_UnRegisterCallback(I2C_EX_HandleTypeDef *hi2c_ex, HAL_I2C_EX_CallbackIDTypeDef CallbackID);

HAL_StatusTypeDef HAL_I2C_EX_RegisterAddrCallback(I2C_EX_HandleTypeDef *hi2c_ex, pI2C_EX_AddrCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_I2C_EX_UnRegisterAddrCallback(I2C_EX_HandleTypeDef *hi2c_ex);
#endif /* USE_HAL_I2C_EX_REGISTER_CALLBACKS */
/**
  * @}
  */  
  
/** @addtogroup I2C_EX_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
/* IO operation functions  ****************************************************/
/******* Blocking mode: Polling */
HAL_StatusTypeDef HAL_I2C_EX_Master_Transmit(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_EX_Master_Receive(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_EX_Slave_Transmit(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_EX_Slave_Receive(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_EX_Mem_Write(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_EX_Mem_Read(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/******* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_I2C_EX_Master_Transmit_IT(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_I2C_EX_Master_Receive_IT(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                            uint16_t Size);
HAL_StatusTypeDef HAL_I2C_EX_Slave_Transmit_IT(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_EX_Slave_Receive_IT(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);

/******* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_I2C_EX_Master_Transmit_DMA(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                              uint16_t Size);
HAL_StatusTypeDef HAL_I2C_EX_Master_Receive_DMA(I2C_EX_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_I2C_EX_Slave_Transmit_DMA(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_EX_Slave_Receive_DMA(I2C_EX_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
/**
  * @}
  */

/** @addtogroup I2C_EX_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */
/******* I2C_EX IRQHandler and Callbacks used in non blocking modes (Interrupt and DMA) */
void HAL_I2C_EX_IRQHandler(I2C_EX_HandleTypeDef *hi2c);
void HAL_I2C_EX_MasterTxCpltCallback(I2C_EX_HandleTypeDef *hi2c);
void HAL_I2C_EX_MasterRxCpltCallback(I2C_EX_HandleTypeDef *hi2c);
void HAL_I2C_EX_SlaveTxCpltCallback(I2C_EX_HandleTypeDef *hi2c);
void HAL_I2C_EX_SlaveRxCpltCallback(I2C_EX_HandleTypeDef *hi2c);
void HAL_I2C_EX_AddrCallback(I2C_EX_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_EX_ListenCpltCallback(I2C_EX_HandleTypeDef *hi2c);
void HAL_I2C_EX_MemTxCpltCallback(I2C_EX_HandleTypeDef *hi2c);
void HAL_I2C_EX_MemRxCpltCallback(I2C_EX_HandleTypeDef *hi2c);
void HAL_I2C_EX_ErrorCallback(I2C_EX_HandleTypeDef *hi2c);
void HAL_I2C_EX_AbortCpltCallback(I2C_EX_HandleTypeDef *hi2c);
/**
  * @}
  */

/** @addtogroup I2C_EX_Exported_Functions_Group3 Peripheral State, Mode and Error functions
  * @{
  */
/* Peripheral State, Mode and Error functions  *********************************/
HAL_I2C_EX_StateTypeDef HAL_I2C_EX_GetState(I2C_EX_HandleTypeDef *hi2c);
HAL_I2C_EX_ModeTypeDef HAL_I2C_EX_GetMode(I2C_EX_HandleTypeDef *hi2c);
uint32_t HAL_I2C_EX_GetError(I2C_EX_HandleTypeDef *hi2c);

/**
  * @}
  */  
  
/**
  * @}
  */
  
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/


/* Private macros ------------------------------------------------------------*/
/** @defgroup I2C_EX_Private_Macros 
  * @{
  */
#define I2C_EX_7BIT_ADD_WRITE(__ADDRESS__)                    ((uint8_t)((__ADDRESS__) & (uint8_t)(~I2C_EX_OAR1_ADD0)))
#define I2C_EX_7BIT_ADD_READ(__ADDRESS__)                     ((uint8_t)((__ADDRESS__) | I2C_EX_OAR1_ADD0))

#define I2C_EX_10BIT_ADD_WRITE(__ADDRESS__)                ((uint8_t)((uint16_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)0x0300)) >> 7) | (uint16_t)0x00F0)))
#define I2C_EX_10BIT_ADD_READ(__ADDRESS__)                 ((uint8_t)((uint16_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)0x0300)) >> 7) | (uint16_t)(0x00F1))))


/** @defgroup I2C_EX_IS_RTC_Definitions I2C_EX Private macros to check input parameters
  * @{
  */
#define I2C_EX_CHECK_FLAG(__ISR__, __FLAG__)         ((((__ISR__) & (__FLAG__)) == (__FLAG__)) ? SET : RESET)
#define I2C_EX_CHECK_IT_SOURCE(__CR1__, __IT__)      ((((__CR1__) & (__IT__)) == (__IT__)) ? SET : RESET)
/**
  * @}
  */

/**
  * @}
  */ 

/* Private functions ---------------------------------------------------------*/
/** @defgroup UART_Private_Functions UART Private Functions
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

#ifdef __cplusplus
}
#endif

#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/

