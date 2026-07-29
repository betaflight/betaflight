/**
  ******************************************************************************
  * @file     um324xx_hal_i2c.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_I2C_H__
#define __UM324XX_HAL_I2C_H__



#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"


/** @addtogroup Um324xx_HAL_Driver
  * @{
  */

/** @addtogroup I2C
  * @{
  */
  
  /* Exported types ------------------------------------------------------------*/
/** @defgroup I2C_Exported_Types I2C Exported Types
  * @{
  */


/** @defgroup I2C_Configuration_Structure_definition I2C Configuration Structure definition
  * @brief  I2C Configuration Structure definition
  * @{
  */

typedef struct
{
  uint32_t AddressingMode;           /*!< Specifies if 7-bit or 10-bit addressing mode is selected.
                                  This parameter can be a value of @ref I2C_ADDRESSING_MODE */

  uint32_t SlaveAddress0;         /*!< Specifies the device own address.
                                  This parameter can be a 7-bit or 10-bit address. */

  uint32_t SlaveAddress1;         /*!< Specifies the device own address.
                                  This parameter can be a 7-bit or 10-bit address. */
    
  uint32_t SlaveAddress2;         /*!< Specifies the device own address.
                                  This parameter can be a 7-bit or 10-bit address. */
    
  uint32_t SlaveAddress3;         /*!< Specifies the device own address.
                                  This parameter can be a 7-bit or 10-bit address. */
  
  uint32_t BaudRate;              /*!< Set the communication rate of i2c. */
    
  uint32_t IrqType;               /*!< interrupt type */

} I2C_InitTypeDef;
/**
  * @}
  */

/** @defgroup HAL_state_structure_definition HAL state structure definition
  * @brief  HAL State structure definition
  * @note  HAL I2C State value coding follow below described bitmap :\n
  *          b7-b6  Error information\n
  *             00 : No Error\n
  *             01 : Abort (Abort user request on going)\n
  *             10 : Timeout\n
  *             11 : Error\n
  *          b5     Peripheral initialization status\n
  *             0  : Reset (peripheral not initialized)\n
  *             1  : Init done (peripheral initialized and ready to use. HAL I2C Init function called)\n
  *          b4     (not used)\n
  *             x  : Should be set to 0\n
  *          b3\n
  *             0  : Ready or Busy (No Listen mode ongoing)\n
  *             1  : Listen (peripheral in Address Listen Mode)\n
  *          b2     Intrinsic process state\n
  *             0  : Ready\n
  *             1  : Busy (peripheral busy with some configuration or internal operations)\n
  *          b1     Rx state\n
  *             0  : Ready (no Rx operation ongoing)\n
  *             1  : Busy (Rx operation ongoing)\n
  *          b0     Tx state\n
  *             0  : Ready (no Tx operation ongoing)\n
  *             1  : Busy (Tx operation ongoing)
  * @{
  */
typedef enum
{
  HAL_I2C_STATE_RESET             = 0x00U,   /*!< Peripheral is not yet Initialized         */
  HAL_I2C_STATE_READY             = 0x20U,   /*!< Peripheral Initialized and ready for use  */
  HAL_I2C_STATE_BUSY              = 0x24U,   /*!< An internal process is ongoing            */
  HAL_I2C_STATE_BUSY_TX           = 0x21U,   /*!< Data Transmission process is ongoing      */
  HAL_I2C_STATE_BUSY_RX           = 0x22U,   /*!< Data Reception process is ongoing         */
  HAL_I2C_STATE_LISTEN            = 0x28U,   /*!< Address Listen Mode is ongoing            */
  HAL_I2C_STATE_BUSY_TX_LISTEN    = 0x29U,   /*!< Address Listen Mode and Data Transmission
                                                 process is ongoing                         */
  HAL_I2C_STATE_BUSY_RX_LISTEN    = 0x2AU,   /*!< Address Listen Mode and Data Reception
                                                 process is ongoing                         */
  HAL_I2C_STATE_ABORT             = 0x60U,   /*!< Abort user request ongoing                */
  HAL_I2C_STATE_TIMEOUT           = 0xA0U,   /*!< Timeout state                             */
  HAL_I2C_STATE_ERROR             = 0xE0U    /*!< Error                                     */

} HAL_I2C_StateTypeDef;



/** @defgroup HAL_mode_structure_definition HAL mode structure definition
  * @brief I2C MODE enumeration
  * @{
  */

typedef enum
{
  HAL_I2C_MODE_NONE           = 0x00U,   /*!< No I2C communication on going             */
  HAL_I2C_MODE_MASTER         = 0x10U,   /*!< I2C communication is in Master Mode       */
  HAL_I2C_MODE_SLAVE          = 0x20U,   /*!< I2C communication is in Slave Mode        */

} HAL_I2C_ModeTypeDef;


/**
  * @}
  */

/** @defgroup I2C_Error_Code_definition I2C Error Code definition
  * @brief  I2C Error Code definition
  * @{
  */
#define HAL_I2C_ERROR_NONE      (0x00000000U)    /*!< No error              */
#define HAL_I2C_ERROR_BERR      (0x00000001U)    /*!< BERR error            */
#define HAL_I2C_ERROR_ARLO      (0x00000002U)    /*!< ARLO error            */
#define HAL_I2C_ERROR_AF        (0x00000004U)    /*!< ACKF error            */
#define HAL_I2C_ERROR_OVR       (0x00000008U)    /*!< OVR error             */
#define HAL_I2C_ERROR_DMA       (0x00000010U)    /*!< DMA transfer error    */
#define HAL_I2C_ERROR_TIMEOUT   (0x00000020U)    /*!< Timeout error         */
#define HAL_I2C_ERROR_SIZE      (0x00000040U)    /*!< Size Management error */
#define HAL_I2C_ERROR_DMA_PARAM (0x00000080U)    /*!< DMA Parameter Error   */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
#define HAL_I2C_ERROR_INVALID_CALLBACK  (0x00000100U)    /*!< Invalid Callback error */
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
#define HAL_I2C_ERROR_INVALID_PARAM     (0x00000200U)    /*!< Invalid Parameters error  */
/**
  * @}
  */

/** @defgroup HAL_mode_structure_definition HAL mode structure definition
  * @brief I2C MODE enumeration
  * @{
  */

typedef enum
{
  I2C_SLAVE_ADDR0           = 0x01U,   /*!< No I2C communication on going             */
  I2C_SLAVE_ADDR1           = 0x02U,   /*!< I2C communication is in Master Mode       */
  I2C_SLAVE_ADDR2           = 0x04U,   /*!< I2C communication is in Slave Mode        */
  I2C_SLAVE_ADDR3           = 0x08U, 
} I2C_SLAVE_ADDR;


/**
  * @}
  */


/** @defgroup HAL_Speed_structure_definition HAL mode structure definition
  * @brief I2C MODE enumeration
  * @{
  */
typedef enum
{
    I2C_SPEED_STAD_100K = 0,
    I2C_SPEED_FAST_400K,
    I2C_SPEED_HIGH_1M
}i2c_speed_typedef;
/**
  * @}
  */



/** @defgroup I2C_handle_Structure_definition I2C handle Structure definition
  * @brief  I2C handle Structure definition
  * @{
  */
typedef struct __I2C_HandleTypeDef
{
  I2C_TypeDef                *Instance;      /*!< I2C registers base address                */

  I2C_InitTypeDef            Init;           /*!< I2C communication parameters              */

  uint8_t                    *pBuffPtr;      /*!< Pointer to I2C transfer buffer            */

  uint16_t                   XferSize;       /*!< I2C transfer size                         */

  __IO uint16_t              XferCount;      /*!< I2C transfer counter                      */
    
  
  __IO uint32_t              XferOptions;    /*!< I2C sequantial transfer options          */

  __IO uint32_t              PreviousState;  /*!< I2C communication Previous state          */

  HAL_StatusTypeDef(*XferISR)(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags);  /*!< I2C transfer IRQ handler function pointer */

  HAL_StatusTypeDef(*TxISR) (struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags);  /*!< I2C transfer IRQ handler function pointer */
  
   HAL_LockTypeDef            Lock;           /*!< I2C locking object                        */

  __IO HAL_I2C_StateTypeDef  State;          /*!< I2C communication state                   */

  __IO HAL_I2C_ModeTypeDef   Mode;           /*!< I2C communication mode                    */

  __IO uint32_t              ErrorCode;      /*!< I2C Error code                            */

  __IO uint32_t              AddrEventCount; /*!< I2C Address Event counter                 */
  
  
  #if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
   void (* MspInitCallback)(struct __I2C_HandleTypeDef *hi2c);
 
   void (* MspDeInitCallback)(struct __I2C_HandleTypeDef *hi2c);
#endif  /* USE_HAL_I2C_REGISTER_CALLBACKS */


} I2C_HandleTypeDef;
/**
  * @}
  */




/** @defgroup I2C_FLAG_define I2C FLAG define
  * @{
  */

#define I2C_FLAG_ERROR                              (0x00UL & I2C_STAT_STA_Msk)     /*!< Due to the occurrence of illegal start 
                                                                                        or stop conditions, a bus error will occur
                                                                                        on the host or selected slave; When external 
                                                                                        interference causes I2C to enter undefined 
                                                                                        state, 0x00 state will also appear     */
#define I2C_FLAG_START                              (0x08UL & I2C_STAT_STA_Msk)      /*!< START sent   */
#define I2C_FLAG_RESTART                            (0x10UL & I2C_STAT_STA_Msk)      /*!< RESTART sent */
#define I2C_FLAG_SEND_ADDRW_ACK                     (0x18UL & I2C_STAT_STA_Msk)      /*!< SLAVE address+W sent, ACK received*/
#define I2C_FLAG_SEND_ADDRW_NAK                     (0x20UL & I2C_STAT_STA_Msk)      /*!< SLAVE address+W sent, ACK received*/
#define I2C_FLAG_MASTER_SEND_ACK                    (0x28UL & I2C_STAT_STA_Msk)      /*!< I2C has been sent in host mode Data in DATA, ACK received */
#define I2C_FLAG_MASTER_SEND_NAK                    (0x30UL & I2C_STAT_STA_Msk)      /*!< I2C has been sent in host mode_ Data in DATA, NAK received */
#define I2C_FLAG_LOST                               (0x38UL & I2C_STAT_STA_Msk)      /*!< Lost arbitration (address or data bytes) */
#define I2C_FLAG_SEND_ADDRR_ACK                     (0x40UL & I2C_STAT_STA_Msk)      /*!< SLAVE address+R sent, ACK received */
#define I2C_FLAG_SEND_ADDRR_NAK                     (0x48UL & I2C_STAT_STA_Msk)      /*!< SLAVE address+R sent, NAK received */
#define I2C_FLAG_MASTER_RECV_ACK                    (0x50UL & I2C_STAT_STA_Msk)      /*!< In host mode, data bytes have been received and ACK has been sent  */
#define I2C_FLAG_MASTER_RECV_NAK                    (0x58UL & I2C_STAT_STA_Msk)      /*!< In host mode, data bytes have been received and NAK has been sent  */
#define I2C_FLAG_RECV_ADDRW_ACK                     (0x60UL & I2C_STAT_STA_Msk)      /*!< Received its own SLAVE address+W, ACK sent  */
#define I2C_FLAG_RECV_ADDRW_ACK_LOST                (0x68UL & I2C_STAT_STA_Msk)      /*!< As the host, the arbitration is lost, the SLAVE address+W has been received, and the ACK has been sent  */
#define I2C_FLAG_RECV_UADDR_ACK                     (0x70UL & I2C_STAT_STA_Msk)      /*!< Universal call address received (0x00), ACK issued  */
#define I2C_FLAG_RECV_UADDR_ACK_LOST                (0x78UL & I2C_STAT_STA_Msk)      /*!< As the host, the arbitration is lost, the universal call address has been received, and the ACK has been issued */
#define I2C_FLAG_RECV_ADDRD_DATA_ACK                (0x80UL & I2C_STAT_STA_Msk)      /*!< After receiving its slave address, it received data bytes and returned ACK  */
#define I2C_FLAG_RECV_ADDRD_DATA_NAK                (0x88UL & I2C_STAT_STA_Msk)      /*!< After receiving its slave address, it received data bytes and returned NAK  */
#define I2C_FLAG_RECV_UADDR_DATA_ACK                (0x90UL & I2C_STAT_STA_Msk)      /*!< After receiving the general call address, data bytes are received and ACK is returned  */
#define I2C_FLAG_RECV_UADDR_DATA_NAK                (0x98UL & I2C_STAT_STA_Msk)      /*!< After receiving the general call address, data bytes are received and NAK is returned  */
#define I2C_FLAG_RECV_STOP_RS                       (0xA0UL & I2C_STAT_STA_Msk)      /*!< In slave mode, stop condition or repeat start condition is received  */
#define I2C_FLAG_RECV_ADDRR_ACK                     (0xA8UL & I2C_STAT_STA_Msk)      /*!< Received its own SLAVE address+R, ACK sent  */
#define I2C_FLAG_MASTER_RECV_UADDR_ACK_LOST         (0xB0UL & I2C_STAT_STA_Msk)      /*!< As the host, the arbitration is lost, the universal call address+R has been received, and the ACK has been issued  */
#define I2C_FLAG_SLAVE_AAK1_ACK                     (0xB8UL & I2C_STAT_STA_Msk)      /*!< In slave mode (AAK=1), data has been sent; ACK received  */
#define I2C_FLAG_SLAVE_AAK1_NAK                     (0xC0UL & I2C_STAT_STA_Msk)      /*!< In slave mode (AAK=1), data has been sent; NAK received  */
#define I2C_FLAG_SLAVE_AAK0_ACK                     (0xC8UL & I2C_STAT_STA_Msk)      /*!< In slave mode (AAK=0), data has been sent; ACK received  */
#define I2C_FLAG_SLAVE_AAK0_NAK                     (0xD0UL & I2C_STAT_STA_Msk)      /*!< In slave mode (AAK=0), data has been sent; NAK received  */
#define I2C_FLAG_SEND_10BITADDRW_ACK                (0xE0UL & I2C_STAT_STA_Msk)      /*!< 10-bit SLAVE second segment address+W sent, ACK received  */
#define I2C_FLAG_SEND_10BITADDRW_NACK               (0xE8UL & I2C_STAT_STA_Msk)      /*!< 10-bit SLAVE second segment address+W has been sent and NACK has been received  */
#define I2C_FLAG_NONE                               (0xF8UL & I2C_STAT_STA_Msk)      /*!< No relevant status information available, IFLG=0  */
                                                                                
#define I2C_FLAG_MASK                               I2C_STAT_STA_Msk
/**
  * @}
  */



/** @defgroup I2C_CLEAR_define I2C CLEAR define
  * @{
  */

/*===I2C OPCR===*/
#define I2C_CLEAR_AAK                               I2C_CLR_CLR_AAK_Msk         /*!< Answer flag clear  */
#define I2C_CLEAR_IFLG                              I2C_CLR_CLR_IFLG_Msk        /*!< Interrupt flag clear  */
#define I2C_CLEAR_STA                               I2C_CLR_CLR_STA_Msk         /*!< Start flag clear register  */
#define I2C_CLEAR_ENAB                              I2C_CLR_CLR_ENAB_Msk        /*!< I2C module enable clear*/
#define I2C_CLEAR_IEN                               I2C_CLR_CLR_IEN_Msk         /*!< I2C module interrupt enable clear*/
#define I2C_IRQ_IE                                  I2C_CR_IEN_Msk              /*!< I2C module interrupt enable  */
#define I2C_IRQ_NONE                                0x00000000U                 /*!< I2C module interrupt disable  */
/**
  * @}
  */



/** @defgroup I2C_ADDRESSING_MODE I2C Addressing Mode
  * @{
  */
#define I2C_ADDRESSINGMODE_7BIT                     (0x00000001U)
#define I2C_ADDRESSINGMODE_10BIT                    (0x00000002U)
            
            
#define I2C_SRST_VALUE                              (0x00000001U)
/**
  * @}
  */
  
/** @defgroup I2C_BAUDRATE SET
  * @{
  */

#define I2C_BAUDRATE_100K                           ((0x5UL & I2C_CCR_CCRN_Msk)|(0x40UL & I2C_CCR_CCRM_Msk))
#define I2C_BAUDRATE_400K                           ((0x5UL & I2C_CCR_CCRN_Msk)|(0x20UL & I2C_CCR_CCRM_Msk))
#define I2C_BAUDRATE_1M                             ((0x0UL & I2C_CCR_CCRN_Msk)|(0x20UL & I2C_CCR_CCRM_Msk))
#define I2C_CCR_FIELDS                              (uint32_t )(I2C_CCR_CCRN_Msk | I2C_CCR_CCRM_Msk)
/**
  * @}
  */
  
  
/** @defgroup I2C TIMEOUT SET
 * @{
 */ 
#define I2C_TIMEOUT                                  0x50 
/**
  * @}
  */
  
/**
  * @}
  */

/** @defgroup I2C_MEMORY_ADDRESS_SIZE I2C Memory Address Size
  * @{
  */
#define I2C_MEMADD_SIZE_8BIT            (0x00000001U)
#define I2C_MEMADD_SIZE_16BIT           (0x00000002U)
/**
  * @}
  */
  
/** @defgroup I2C READ/WRITE define
  * @{
  */

#define I2C_READ                                    (1UL)
#define I2C_WRITE                                   (0UL)
/**
  * @}
  */
	
	
/** @defgroup I2C_MEMORY_ADDRESS_SIZE
  * @{
  */
#define I2C_MEM_ADD_MSB(__ADDRESS__)              ((uint8_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)(0xFF00U))) >> 8U)))
#define I2C_MEM_ADD_LSB(__ADDRESS__)              ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)(0x00FFU))))
/**
  * @}
  */
	
  
/**
  * @}
  */  
  

/* Exported macro ------------------------------------------------------------*/
/** @defgroup I2c_Exported_Macros I2c Exported Macros
  * @{
  */

/** @brief  Enable the I2C Slave mode .
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_SLAVE_ENABLE(__HANDLE__)                  SET_BIT((__HANDLE__)->Instance->CR,I2C_CR_ENAB_Msk)

/** @brief  Disable the I2C Slave mode.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_SLAVE_DISABLE(__HANDLE__)                 SET_BIT((__HANDLE__)->Instance->CR,I2C_CLR_ENAB_Msk)


/** @brief  Enable the  I2C interrupt.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_ENABLE_IT(__HANDLE__)                 SET_BIT((__HANDLE__)->Instance->CR,I2C_CR_IEN_Msk)


/** @brief  Enable the I2C interrupt.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_DISABLE_IT(__HANDLE__)                SET_BIT((__HANDLE__)->Instance->CLR,I2C_CLR_CLR_IEN)  
  
/** @brief  Get the I2C interrupt Flag.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_GET_FLAG(__HANDLE__, __FLAG__)        (((__HANDLE__)->Instance->STAT) == (__FLAG__))  

/** @brief  Clear the I2C interrupt IFLG.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_CLEAR_IFLG(__HANDLE__)                SET_BIT((__HANDLE__)->Instance->CLR,I2C_CLR_CLR_IFLG)  
  
 
/** @brief  Enable AAK.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_AAK_ENABLE(__HANDLE__)                    SET_BIT((__HANDLE__)->Instance->CR,I2C_CR_AAK_Msk)

/** @brief  Disable AAK.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_AAK_DISABLE(__HANDLE__)                   SET_BIT((__HANDLE__)->Instance->CLR,I2C_CR_AAK_Msk)



/** @brief  Send I2c Start message.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_START_SEND(__HANDLE__)                    SET_BIT((__HANDLE__)->Instance->CR,I2C_CR_STA_Msk)

/** @brief  Send I2c Stop message.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_STOP_SEND(__HANDLE__)                     SET_BIT((__HANDLE__)->Instance->CR,I2C_CR_STP_Msk)

/** @brief  match 7 bit address.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_GET_SLAV7M(__HANDLE__)                    SET_BIT((__HANDLE__)->Instance->CR,I2C_CR_SLAV7M_Msk)

/** @brief  match 10 bit address.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_GET_SLAV7M(__HANDLE__)                    SET_BIT((__HANDLE__)->Instance->CR,I2C_CR_SLAV7M_Msk)

/** @brief  I2C Reset .
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_RSRT_ENABLE(__HANDLE__)                   SET_BIT((__HANDLE__)->Instance->SRST,I2C_SRST_VALUE)



/** @brief  I2C check interrupt Flag.
  * @param  __HANDLE__ specifies the I2C Handle.
  *
  * @retval None
  */
#define __HAL_I2C_CHECK_FLAG(__SR__, __FLAG__)                (((((__SR__) & I2C_FLAG_MASK)) == \
                                                        ((__FLAG__) & I2C_FLAG_MASK)) ? 1 : 0)

/**
  * @}
  */  


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2C_Exported_Functions
  * @{
  */

/** @addtogroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
  
/**
  * @brief  Initializes the I2C according to the specified parameters
  *         in the I2C_InitTypeDef and initialize the associated handle.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
  
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);
/**
  * @}
  */

/** @addtogroup I2C_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
  
/* i2c operation functions  ***************************************************/
/******* Blocking mode: Polling */
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                            uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                            uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,uint16_t MemAddress, uint16_t MemAddSize,
                                     uint8_t *pData, uint16_t Size,uint32_t Timeout);

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,uint16_t MemAddress, uint16_t MemAddSize,
                                      uint8_t *pData, uint16_t Size,uint32_t Timeout);
/******* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                               HAL_StatusTypeDef (*recv_callback)());
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                               HAL_StatusTypeDef (*recv_callback)());
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, HAL_StatusTypeDef (*recv_callback)());
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, HAL_StatusTypeDef (*recv_callback)());

/**
  * @}
  */

/** @addtogroup I2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */
 

void HAL_I2C_IRQHandler(I2C_HandleTypeDef *hi2c);  
 /**
  * @}
  */
/** @addtogroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
  * @{
  */
/* Peripheral State, Mode and Error functions  *********************************/
HAL_I2C_StateTypeDef HAL_I2C_GetState(const I2C_HandleTypeDef *hi2c);
HAL_I2C_ModeTypeDef  HAL_I2C_GetMode(const I2C_HandleTypeDef *hi2c);
uint32_t             HAL_I2C_GetError(const I2C_HandleTypeDef *hi2c);

/**
  * @}
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

#endif /* __Um324xx_HAL_I2C_H */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
