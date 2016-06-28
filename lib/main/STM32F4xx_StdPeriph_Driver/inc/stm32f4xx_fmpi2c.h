/**
  ******************************************************************************
  * @file    stm32f4xx_fmpi2c.h
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   This file contains all the functions prototypes for the I2C Fast Mode
  *          Plus firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_FMPI2C_H
#define __STM32F4xx_FMPI2C_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup FMPI2C
  * @{
  */
#if defined(STM32F410xx) || defined(STM32F412xG) || defined(STM32F446xx)
/* Exported types ------------------------------------------------------------*/

/**
  * @brief  FMPI2C Init structure definition
  */

typedef struct
{
  uint32_t FMPI2C_Timing;              /*!< Specifies the FMPI2C_TIMINGR_register value.
                                         This parameter calculated by referring to FMPI2C initialization 
                                         section in Reference manual*/

  uint32_t FMPI2C_AnalogFilter;        /*!< Enables or disables analog noise filter.
                                         This parameter can be a value of @ref FMPI2C_Analog_Filter */

  uint32_t FMPI2C_DigitalFilter;       /*!< Configures the digital noise filter.
                                         This parameter can be a number between 0x00 and 0x0F */

  uint32_t FMPI2C_Mode;                /*!< Specifies the FMPI2C mode.
                                         This parameter can be a value of @ref FMPI2C_mode */

  uint32_t FMPI2C_OwnAddress1;         /*!< Specifies the device own address 1.
                                         This parameter can be a 7-bit or 10-bit address */

  uint32_t FMPI2C_Ack;                 /*!< Enables or disables the acknowledgement.
                                         This parameter can be a value of @ref FMPI2C_acknowledgement */

  uint32_t FMPI2C_AcknowledgedAddress; /*!< Specifies if 7-bit or 10-bit address is acknowledged.
                                         This parameter can be a value of @ref FMPI2C_acknowledged_address */
}FMPI2C_InitTypeDef;

/* Exported constants --------------------------------------------------------*/


/** @defgroup FMPI2C_Exported_Constants
  * @{
  */

#define IS_FMPI2C_ALL_PERIPH(PERIPH)       ((PERIPH) == FMPI2C1) 

/** @defgroup FMPI2C_Analog_Filter 
  * @{
  */

#define FMPI2C_AnalogFilter_Enable         ((uint32_t)0x00000000)
#define FMPI2C_AnalogFilter_Disable        FMPI2C_CR1_ANFOFF

#define IS_FMPI2C_ANALOG_FILTER(FILTER)    (((FILTER) == FMPI2C_AnalogFilter_Enable) || \
                                         ((FILTER) == FMPI2C_AnalogFilter_Disable))
/**
  * @}
  */
     
/** @defgroup FMPI2C_Digital_Filter
  * @{
  */

#define IS_FMPI2C_DIGITAL_FILTER(FILTER)   ((FILTER) <= 0x0000000F)
/**
  * @}
  */

/** @defgroup FMPI2C_mode 
  * @{
  */

#define FMPI2C_Mode_FMPI2C                ((uint32_t)0x00000000)
#define FMPI2C_Mode_SMBusDevice            FMPI2C_CR1_SMBDEN
#define FMPI2C_Mode_SMBusHost              FMPI2C_CR1_SMBHEN

#define IS_FMPI2C_MODE(MODE)               (((MODE) == FMPI2C_Mode_FMPI2C) || \
                                         ((MODE) == FMPI2C_Mode_SMBusDevice) || \
                                         ((MODE) == FMPI2C_Mode_SMBusHost))
/**
  * @}
  */

/** @defgroup FMPI2C_acknowledgement
  * @{
  */

#define FMPI2C_Ack_Enable                  ((uint32_t)0x00000000)
#define FMPI2C_Ack_Disable                 FMPI2C_CR2_NACK

#define IS_FMPI2C_ACK(ACK)                 (((ACK) == FMPI2C_Ack_Enable) || \
                                         ((ACK) == FMPI2C_Ack_Disable))
/**
  * @}
  */

/** @defgroup FMPI2C_acknowledged_address
  * @{
  */

#define FMPI2C_AcknowledgedAddress_7bit    ((uint32_t)0x00000000)
#define FMPI2C_AcknowledgedAddress_10bit   FMPI2C_OAR1_OA1MODE

#define IS_FMPI2C_ACKNOWLEDGE_ADDRESS(ADDRESS) (((ADDRESS) == FMPI2C_AcknowledgedAddress_7bit) || \
                                             ((ADDRESS) == FMPI2C_AcknowledgedAddress_10bit))
/**
  * @}
  */ 

/** @defgroup FMPI2C_own_address1
  * @{
  */

#define IS_FMPI2C_OWN_ADDRESS1(ADDRESS1)   ((ADDRESS1) <= (uint32_t)0x000003FF)
/**
  * @}
  */

/** @defgroup FMPI2C_transfer_direction 
  * @{
  */

#define FMPI2C_Direction_Transmitter       ((uint16_t)0x0000)
#define FMPI2C_Direction_Receiver          ((uint16_t)0x0400)

#define IS_FMPI2C_DIRECTION(DIRECTION)     (((DIRECTION) == FMPI2C_Direction_Transmitter) || \
                                         ((DIRECTION) == FMPI2C_Direction_Receiver))
/**
  * @}
  */

/** @defgroup FMPI2C_DMA_transfer_requests 
  * @{
  */

#define FMPI2C_DMAReq_Tx                   FMPI2C_CR1_TXDMAEN
#define FMPI2C_DMAReq_Rx                   FMPI2C_CR1_RXDMAEN

#define IS_FMPI2C_DMA_REQ(REQ)             ((((REQ) & (uint32_t)0xFFFF3FFF) == 0x00) && ((REQ) != 0x00))
/**
  * @}
  */

/** @defgroup FMPI2C_slave_address
  * @{
  */

#define IS_FMPI2C_SLAVE_ADDRESS(ADDRESS)   ((ADDRESS) <= (uint16_t)0x03FF)
/**
  * @}
  */


/** @defgroup FMPI2C_own_address2
  * @{
  */

#define IS_FMPI2C_OWN_ADDRESS2(ADDRESS2)   ((ADDRESS2) <= (uint16_t)0x00FF)

/**
  * @}
  */

/** @defgroup FMPI2C_own_address2_mask
  * @{
  */

#define FMPI2C_OA2_NoMask                  ((uint8_t)0x00)
#define FMPI2C_OA2_Mask01                  ((uint8_t)0x01)
#define FMPI2C_OA2_Mask02                  ((uint8_t)0x02)
#define FMPI2C_OA2_Mask03                  ((uint8_t)0x03)
#define FMPI2C_OA2_Mask04                  ((uint8_t)0x04)
#define FMPI2C_OA2_Mask05                  ((uint8_t)0x05)
#define FMPI2C_OA2_Mask06                  ((uint8_t)0x06)
#define FMPI2C_OA2_Mask07                  ((uint8_t)0x07)

#define IS_FMPI2C_OWN_ADDRESS2_MASK(MASK)  (((MASK) == FMPI2C_OA2_NoMask) || \
                                         ((MASK) == FMPI2C_OA2_Mask01) || \
                                         ((MASK) == FMPI2C_OA2_Mask02) || \
                                         ((MASK) == FMPI2C_OA2_Mask03) || \
                                         ((MASK) == FMPI2C_OA2_Mask04) || \
                                         ((MASK) == FMPI2C_OA2_Mask05) || \
                                         ((MASK) == FMPI2C_OA2_Mask06) || \
                                         ((MASK) == FMPI2C_OA2_Mask07))  

/**
  * @}
  */

/** @defgroup FMPI2C_timeout
  * @{
  */

#define IS_FMPI2C_TIMEOUT(TIMEOUT)   ((TIMEOUT) <= (uint16_t)0x0FFF)

/**
  * @}
  */

/** @defgroup FMPI2C_registers 
  * @{
  */

#define FMPI2C_Register_CR1                ((uint8_t)0x00)
#define FMPI2C_Register_CR2                ((uint8_t)0x04)
#define FMPI2C_Register_OAR1               ((uint8_t)0x08)
#define FMPI2C_Register_OAR2               ((uint8_t)0x0C)
#define FMPI2C_Register_TIMINGR            ((uint8_t)0x10)
#define FMPI2C_Register_TIMEOUTR           ((uint8_t)0x14)
#define FMPI2C_Register_ISR                ((uint8_t)0x18)
#define FMPI2C_Register_ICR                ((uint8_t)0x1C)
#define FMPI2C_Register_PECR               ((uint8_t)0x20)
#define FMPI2C_Register_RXDR               ((uint8_t)0x24)
#define FMPI2C_Register_TXDR               ((uint8_t)0x28)

#define IS_FMPI2C_REGISTER(REGISTER)       (((REGISTER) == FMPI2C_Register_CR1) || \
                                         ((REGISTER) == FMPI2C_Register_CR2) || \
                                         ((REGISTER) == FMPI2C_Register_OAR1) || \
                                         ((REGISTER) == FMPI2C_Register_OAR2) || \
                                         ((REGISTER) == FMPI2C_Register_TIMINGR) || \
                                         ((REGISTER) == FMPI2C_Register_TIMEOUTR) || \
                                         ((REGISTER) == FMPI2C_Register_ISR) || \
                                         ((REGISTER) == FMPI2C_Register_ICR) || \
                                         ((REGISTER) == FMPI2C_Register_PECR) || \
                                         ((REGISTER) == FMPI2C_Register_RXDR) || \
                                         ((REGISTER) == FMPI2C_Register_TXDR))
/**
  * @}
  */

/** @defgroup FMPI2C_interrupts_definition 
  * @{
  */

#define FMPI2C_IT_ERRI                     FMPI2C_CR1_ERRIE
#define FMPI2C_IT_TCI                      FMPI2C_CR1_TCIE
#define FMPI2C_IT_STOPI                    FMPI2C_CR1_STOPIE
#define FMPI2C_IT_NACKI                    FMPI2C_CR1_NACKIE
#define FMPI2C_IT_ADDRI                    FMPI2C_CR1_ADDRIE
#define FMPI2C_IT_RXI                      FMPI2C_CR1_RXIE
#define FMPI2C_IT_TXI                      FMPI2C_CR1_TXIE

#define IS_FMPI2C_CONFIG_IT(IT)            ((((IT) & (uint32_t)0xFFFFFF01) == 0x00) && ((IT) != 0x00))

/**
  * @}
  */

/** @defgroup FMPI2C_flags_definition 
  * @{
  */

#define  FMPI2C_FLAG_TXE                   FMPI2C_ISR_TXE
#define  FMPI2C_FLAG_TXIS                  FMPI2C_ISR_TXIS
#define  FMPI2C_FLAG_RXNE                  FMPI2C_ISR_RXNE
#define  FMPI2C_FLAG_ADDR                  FMPI2C_ISR_ADDR
#define  FMPI2C_FLAG_NACKF                 FMPI2C_ISR_NACKF
#define  FMPI2C_FLAG_STOPF                 FMPI2C_ISR_STOPF
#define  FMPI2C_FLAG_TC                    FMPI2C_ISR_TC
#define  FMPI2C_FLAG_TCR                   FMPI2C_ISR_TCR
#define  FMPI2C_FLAG_BERR                  FMPI2C_ISR_BERR
#define  FMPI2C_FLAG_ARLO                  FMPI2C_ISR_ARLO
#define  FMPI2C_FLAG_OVR                   FMPI2C_ISR_OVR
#define  FMPI2C_FLAG_PECERR                FMPI2C_ISR_PECERR
#define  FMPI2C_FLAG_TIMEOUT               FMPI2C_ISR_TIMEOUT
#define  FMPI2C_FLAG_ALERT                 FMPI2C_ISR_ALERT
#define  FMPI2C_FLAG_BUSY                  FMPI2C_ISR_BUSY

#define IS_FMPI2C_CLEAR_FLAG(FLAG)         ((((FLAG) & (uint32_t)0xFFFF4000) == 0x00) && ((FLAG) != 0x00))

#define IS_FMPI2C_GET_FLAG(FLAG)           (((FLAG) == FMPI2C_FLAG_TXE) || ((FLAG) == FMPI2C_FLAG_TXIS) || \
                                         ((FLAG) == FMPI2C_FLAG_RXNE) || ((FLAG) == FMPI2C_FLAG_ADDR) || \
                                         ((FLAG) == FMPI2C_FLAG_NACKF) || ((FLAG) == FMPI2C_FLAG_STOPF) || \
                                         ((FLAG) == FMPI2C_FLAG_TC) || ((FLAG) == FMPI2C_FLAG_TCR) || \
                                         ((FLAG) == FMPI2C_FLAG_BERR) || ((FLAG) == FMPI2C_FLAG_ARLO) || \
                                         ((FLAG) == FMPI2C_FLAG_OVR) || ((FLAG) == FMPI2C_FLAG_PECERR) || \
                                         ((FLAG) == FMPI2C_FLAG_TIMEOUT) || ((FLAG) == FMPI2C_FLAG_ALERT) || \
                                         ((FLAG) == FMPI2C_FLAG_BUSY))

/**
  * @}
  */


/** @defgroup FMPI2C_interrupts_definition 
  * @{
  */

#define  FMPI2C_IT_TXIS                    FMPI2C_ISR_TXIS
#define  FMPI2C_IT_RXNE                    FMPI2C_ISR_RXNE
#define  FMPI2C_IT_ADDR                    FMPI2C_ISR_ADDR
#define  FMPI2C_IT_NACKF                   FMPI2C_ISR_NACKF
#define  FMPI2C_IT_STOPF                   FMPI2C_ISR_STOPF
#define  FMPI2C_IT_TC                      FMPI2C_ISR_TC
#define  FMPI2C_IT_TCR                     FMPI2C_ISR_TCR
#define  FMPI2C_IT_BERR                    FMPI2C_ISR_BERR
#define  FMPI2C_IT_ARLO                    FMPI2C_ISR_ARLO
#define  FMPI2C_IT_OVR                     FMPI2C_ISR_OVR
#define  FMPI2C_IT_PECERR                  FMPI2C_ISR_PECERR
#define  FMPI2C_IT_TIMEOUT                 FMPI2C_ISR_TIMEOUT
#define  FMPI2C_IT_ALERT                   FMPI2C_ISR_ALERT

#define IS_FMPI2C_CLEAR_IT(IT)             ((((IT) & (uint32_t)0xFFFFC001) == 0x00) && ((IT) != 0x00))
                               
#define IS_FMPI2C_GET_IT(IT)               (((IT) == FMPI2C_IT_TXIS) || ((IT) == FMPI2C_IT_RXNE) || \
                                         ((IT) == FMPI2C_IT_ADDR) || ((IT) == FMPI2C_IT_NACKF) || \
                                         ((IT) == FMPI2C_IT_STOPF) || ((IT) == FMPI2C_IT_TC) || \
                                         ((IT) == FMPI2C_IT_TCR) || ((IT) == FMPI2C_IT_BERR) || \
                                         ((IT) == FMPI2C_IT_ARLO) || ((IT) == FMPI2C_IT_OVR) || \
                                         ((IT) == FMPI2C_IT_PECERR) || ((IT) == FMPI2C_IT_TIMEOUT) || \
                                         ((IT) == FMPI2C_IT_ALERT))

/**
  * @}
  */

/** @defgroup FMPI2C_ReloadEndMode_definition 
  * @{
  */

#define  FMPI2C_Reload_Mode                FMPI2C_CR2_RELOAD
#define  FMPI2C_AutoEnd_Mode               FMPI2C_CR2_AUTOEND
#define  FMPI2C_SoftEnd_Mode               ((uint32_t)0x00000000)

                              
#define IS_RELOAD_END_MODE(MODE)        (((MODE) == FMPI2C_Reload_Mode) || \
                                         ((MODE) == FMPI2C_AutoEnd_Mode) || \
                                         ((MODE) == FMPI2C_SoftEnd_Mode))
                               

/**
  * @}
  */

/** @defgroup FMPI2C_StartStopMode_definition 
  * @{
  */

#define  FMPI2C_No_StartStop                 ((uint32_t)0x00000000)
#define  FMPI2C_Generate_Stop                FMPI2C_CR2_STOP
#define  FMPI2C_Generate_Start_Read          (uint32_t)(FMPI2C_CR2_START | FMPI2C_CR2_RD_WRN)
#define  FMPI2C_Generate_Start_Write         FMPI2C_CR2_START

                              
#define IS_START_STOP_MODE(MODE)        (((MODE) == FMPI2C_Generate_Stop) || \
                                         ((MODE) == FMPI2C_Generate_Start_Read) || \
                                         ((MODE) == FMPI2C_Generate_Start_Write) || \
                                         ((MODE) == FMPI2C_No_StartStop))
                               

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


/* Initialization and Configuration functions *********************************/
void FMPI2C_DeInit(FMPI2C_TypeDef* FMPI2Cx);
void FMPI2C_Init(FMPI2C_TypeDef* FMPI2Cx, FMPI2C_InitTypeDef* FMPI2C_InitStruct);
void FMPI2C_StructInit(FMPI2C_InitTypeDef* FMPI2C_InitStruct);
void FMPI2C_Cmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_SoftwareResetCmd(FMPI2C_TypeDef* FMPI2Cx);
void FMPI2C_ITConfig(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_IT, FunctionalState NewState);
void FMPI2C_StretchClockCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_DualAddressCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_OwnAddress2Config(FMPI2C_TypeDef* FMPI2Cx, uint16_t Address, uint8_t Mask);
void FMPI2C_GeneralCallCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_SlaveByteControlCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_SlaveAddressConfig(FMPI2C_TypeDef* FMPI2Cx, uint16_t Address);
void FMPI2C_10BitAddressingModeCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);

/* Communications handling functions ******************************************/
void FMPI2C_AutoEndCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_ReloadCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_NumberOfBytesConfig(FMPI2C_TypeDef* FMPI2Cx, uint8_t Number_Bytes);
void FMPI2C_MasterRequestConfig(FMPI2C_TypeDef* FMPI2Cx, uint16_t FMPI2C_Direction);
void FMPI2C_GenerateSTART(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_GenerateSTOP(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_10BitAddressHeaderCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_AcknowledgeConfig(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
uint8_t FMPI2C_GetAddressMatched(FMPI2C_TypeDef* FMPI2Cx);
uint16_t FMPI2C_GetTransferDirection(FMPI2C_TypeDef* FMPI2Cx);
void FMPI2C_TransferHandling(FMPI2C_TypeDef* FMPI2Cx, uint16_t Address, uint8_t Number_Bytes, uint32_t ReloadEndMode, uint32_t StartStopMode);

/*  SMBUS management functions ************************************************/
void FMPI2C_SMBusAlertCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_ClockTimeoutCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_ExtendedClockTimeoutCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_IdleClockTimeoutCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_TimeoutAConfig(FMPI2C_TypeDef* FMPI2Cx, uint16_t Timeout);
void FMPI2C_TimeoutBConfig(FMPI2C_TypeDef* FMPI2Cx, uint16_t Timeout);
void FMPI2C_CalculatePEC(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
void FMPI2C_PECRequestCmd(FMPI2C_TypeDef* FMPI2Cx, FunctionalState NewState);
uint8_t FMPI2C_GetPEC(FMPI2C_TypeDef* FMPI2Cx);

/* FMPI2C registers management functions *****************************************/
uint32_t FMPI2C_ReadRegister(FMPI2C_TypeDef* FMPI2Cx, uint8_t FMPI2C_Register);

/* Data transfers management functions ****************************************/
void FMPI2C_SendData(FMPI2C_TypeDef* FMPI2Cx, uint8_t Data);
uint8_t FMPI2C_ReceiveData(FMPI2C_TypeDef* FMPI2Cx);

/* DMA transfers management functions *****************************************/
void FMPI2C_DMACmd(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_DMAReq, FunctionalState NewState);

/* Interrupts and flags management functions **********************************/
FlagStatus FMPI2C_GetFlagStatus(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_FLAG);
void FMPI2C_ClearFlag(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_FLAG);
ITStatus FMPI2C_GetITStatus(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_IT);
void FMPI2C_ClearITPendingBit(FMPI2C_TypeDef* FMPI2Cx, uint32_t FMPI2C_IT);

#endif /* STM32F410xx || STM32F412xG || STM32F446xx */
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__STM32F4xx_FMPI2C_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
