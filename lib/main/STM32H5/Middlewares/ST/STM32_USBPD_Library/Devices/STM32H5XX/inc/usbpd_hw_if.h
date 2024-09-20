/**
  ******************************************************************************
  * @file    usbpd_hw_if.h
  * @author  MCD Application Team
  * @brief   This file contains the headers of usbpd_hw_if.h for USB-PD Hardware
             Interface layer. This file is specific for each device.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __USBPD_HW_IF_H_
#define __USBPD_HW_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
#include "usbpd_devices_conf.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_HW_IF
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_DEVICE_HW_IF_Exported_Types USBPD DEVICE HW_IF Exported Types
  * @{
  */

/**
  * @brief Enum used to get the status of decoding
  */
typedef enum
{
  USBPD_PHY_RX_STATUS_NONE,
  USBPD_PHY_RX_STATUS_OK,
  USBPD_PHY_RX_STATUS_SOP_DETECTING,
  USBPD_PHY_RX_STATUS_DATA,
  USBPD_PHY_RX_STATUS_MESSAGE_READY,
  USBPD_PHY_RX_STATUS_ERROR,
  USBPD_PHY_RX_STATUS_ERROR_UNSUPPORTED_SOP,
  USBPD_PHY_RX_STATUS_ERROR_INVALID_SOP,
  USBPD_PHY_RX_STATUS_ERROR_INVALID_SYMBOL,
  USBPD_PHY_RX_STATUS_ERROR_EOP_NOT_FOUND,
  USBPD_PHY_RX_STATUS_ERROR_CRC_FAILED,
}
USBPD_PHY_RX_Status_TypeDef;

/**
  * @brief CallBacks exposed by the HW_IF to the PHY
  */
typedef struct
{
  /**
    * @brief  The message transfer has been completed
    * @param  PortNum Port number
    * @param  Status (0 means OK)
    * @retval None
  */
  void (*USBPD_HW_IF_TxCompleted)(uint8_t PortNum, uint32_t Status);

  /**
    * @brief  Bist data sent callback from PHY_HW_IF
    * @param  PortNum Port number
    * @param  bistmode: Bist mode
    * @retval None
  */
  void (*USBPD_HW_IF_BistCompleted)(uint8_t PortNum, USBPD_BISTMsg_TypeDef bistmode);

  /**
    * @brief  The reception phase of an hard reset is completed notify it.
    * @param  PortNum Port number
    * @param  SOPType SOP Message Type based on @ref USBPD_SOPType_TypeDef
    * @retval None
  */
  void (*USBPD_HW_IF_RX_ResetIndication)(uint8_t PortNum, USBPD_SOPType_TypeDef Type);

  /**
    * @brief  The reception phase of the current message is completed and notify it.
    * @param  PortNum Port number
    * @param  MsgType Message Type
    * @retval None
  */
  void (*USBPD_HW_IF_RX_Completed)(uint8_t PortNum, uint32_t MsgType);

  /**
    * @brief  The emission of HRST has been completed.
    * @param  PortNum Port number
    * @retval None
  */
  void (*USBPD_HW_IF_TX_HardResetCompleted)(uint8_t PortNum, USBPD_SOPType_TypeDef Type);

  /**
    * @brief  FRS reception.
    * @param  PortNum Port number
    * @retval None
  */
  void (*USBPD_HW_IF_TX_FRSReception)(uint8_t PortNum);

} USBPD_HW_IF_Callbacks_TypeDef;

/** @defgroup USBPD_PORT_HandleTypeDef USB PD handle Structure definition for USBPD_PHY_HW_IF
  * @brief  USBPD PORT handle Structure definition
  * @{
  */
typedef struct
{
  UCPD_TypeDef                   *husbpd;         /*!< UCPD Handle parameters                              */
  DMA_Channel_TypeDef            *hdmatx;         /*!< Tx DMA Handle parameters                            */
  DMA_Channel_TypeDef            *hdmarx;         /*!< Rx DMA Handle parameters                            */

  USBPD_SettingsTypeDef          *settings;
  USBPD_ParamsTypeDef            *params;
  USBPD_HW_IF_Callbacks_TypeDef  cbs;             /*!< USBPD_PHY_HW_IF callbacks                           */

  void (*USBPD_CAD_WakeUp)(void);                 /*!< function used to wakeup cad task                    */

  uint8_t                        *ptr_RxBuff;     /*!< Pointer to Raw Rx transfer Buffer                   */

  CCxPin_TypeDef                 CCx;             /*!< CC pin used for communication                       */
  __IO uint8_t                   RXStatus;        /*!< Tracks the reception of a message to forbid any new
                                                       TX transaction until message completion             */
} USBPD_PORT_HandleTypeDef;

extern USBPD_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];

/**
  * @}
  */

/**
  * @}
  */

/* Exported define -----------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_DEVICE_HW_IF_Exported_Defines USBPD DEVICE HW_IF Exported Defines
  * @{
  */

#define SIZE_MAX_PD_TRANSACTION_CHUNK   30u
#define SIZE_MAX_PD_TRANSACTION_UNCHUNK (260u + 4u)

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_DEVICE_HW_IF_Exported_Functions USBPD DEVICE HW_IF Exported Functions
  * @{
  */

/**
  * @brief  Enable the interrupt for the reception.
  * @param  PortNum The handle of the port.
  * @retval None
  */
void USBPDM1_RX_EnableInterrupt(uint8_t PortNum);

/**
  * @brief  stop bist carrier mode 2.
  * @param  PortNum The port handle.
  * @retval None
  */
void USBPD_HW_IF_StopBISTMode2(uint8_t PortNum);

/**
  * @brief  Initialize specific peripheral for the APP.
  * @retval None
  */
void USBPD_HW_IF_GlobalHwInit(void);

/**
  * @brief  Send a Buffer .
  * @note   The data will be converted in bmc and send through the line
  * @param  PortNum     The port handle.
  * @param  Type        SOP Message Type based on @ref USBPD_SOPType_TypeDef
  * @param  pBuffer     Data buffer to be transmitted
  * @param  Bitsize     The number of bits to be transmitted
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t PortNum, USBPD_SOPType_TypeDef Type, uint8_t *pBuffer,
                                           uint32_t Bitsize);

#if defined(_SRC) || defined(_DRP)
/**
  * @brief  Enable the VBUS on a specified port.
  * @param  PortNum     The port handle.
  * @param  State       ENABLE or DISABLE.
  * @param  Cc          CC pin based on @ref CCxPin_TypeDef
  * @param  VconnState  VCONN State activation
  * @param  role        The role of the port.
  * @retval USBPD status
  */
USBPD_StatusTypeDef HW_IF_PWR_Enable(uint8_t PortNum, USBPD_FunctionalState State, CCxPin_TypeDef Cc,
                                     uint32_t VconnState, USBPD_PortPowerRole_TypeDef role);
#endif /* _SRC || _DRP */

/**
  * @brief  Retrieve the VBUS status for a specified port.
  * @param  PortNum The port handle.
  * @retval FunctionalState
  */
USBPD_FunctionalState HW_IF_PWR_VBUSIsEnabled(uint8_t PortNum);

/**
  * @brief  Set the VBUS voltage level on a specified port.
  * @param  PortNum The port handle.
  * @param  Voltage voltage value to be set.
  * @retval USBPD status
  */
USBPD_StatusTypeDef HW_IF_PWR_SetVoltage(uint8_t PortNum, uint16_t Voltage);

/**
  * @brief  Get the voltage level on a specified port.
  * @param  PortNum The port handle.
  * @retval The voltage value
  */
uint16_t HW_IF_PWR_GetVoltage(uint8_t PortNum);

/**
  * @brief  Get the current level on a specified port.
  * @param  PortNum The port handle.
  * @retval The current value
  */
int16_t HW_IF_PWR_GetCurrent(uint8_t PortNum);

/**
  * @brief  Connect the Rp resistors on the CC lines
  * @param  PortNum The port handle.
  * @retval none
  */
void USBPDM1_AssertRp(uint8_t PortNum);

/**
  * @brief  Disconnect the Rp resistors on the CC lines
  * @param  PortNum The port handle.
  * @retval none
  */
void USBPDM1_DeAssertRp(uint8_t PortNum);

/**
  * @brief  Connect the Rd resistors on the CC lines
  * @param  PortNum The port handle.
  * @retval none
  */
void USBPDM1_AssertRd(uint8_t PortNum);

/**
  * @brief  Disconnect the Rd resistors on the CC lines
  * @param  PortNum The port handle.
  * @retval none
  */
void USBPDM1_DeAssertRd(uint8_t PortNum);

/**
  * @brief  Set the CCx pin.
  * @param  PortNum The port handle.
  * @param  cc      Specify the ccx to be selected.
  * @retval None
  */
void USBPDM1_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc);

/**
  * @brief  Sends the BIST pattern
  * @param  PortNum The port handle.
  * @retval none
  */
void USBPD_HW_IF_Send_BIST_Pattern(uint8_t PortNum);

/**
  * @brief  Sends a detachemnt signal.
  * @param  PortNum The port handle.
  * @retval none
  */
void HW_SignalDetachment(uint8_t PortNum);

/**
  * @brief  Sends an Attachment signal.
  * @param  PortNum The port handle.
  * @param  cc the PD pin.
  * @retval none
  */
void HW_SignalAttachement(uint8_t PortNum, CCxPin_TypeDef cc);

/**
  * @brief  Set SinkTxNG resistor.
  * @param  PortNum The port handle.
  * @retval none
  */
void USBPD_HW_IF_SetResistor_SinkTxNG(uint8_t PortNum);

/**
  * @brief  Set SinkTxOk resistor.
  * @param  PortNum The port handle.
  * @retval none
  */
void USBPD_HW_IF_SetResistor_SinkTxOK(uint8_t PortNum);

/**
  * @brief  Is SinkTxOk resistor.
  * @param  PortNum The port handle.
  * @retval TRUE FALSE
  */
uint8_t USBPD_HW_IF_IsResistor_SinkTxOk(uint8_t PortNum);

/**
  * @brief  send a Fast Role swap signalling.
  * @param  PortNum The port handle.
  * @retval None
  */
void USBPD_HW_IF_FastRoleSwapSignalling(uint8_t PortNum);

/**
  * @brief  enter in error recovery state.
  * @param  PortNum The port handle.
  * @retval None
  */
void USBPDM1_EnterErrorRecovery(uint8_t PortNum);


void USBPD_PORT0_IRQHandler(void);
void USBPD_PORT1_IRQHandler(void);

/**
  * @brief  Enable RX
  * @param  PortNum The port handle.
  * @retval None
  */
void    USBPD_HW_IF_EnableRX(uint8_t PortNum);

/**
  * @brief  Disable RX
  * @param  PortNum The port handle.
  * @retval None
  */
void    USBPD_HW_IF_DisableRX(uint8_t PortNum);

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

#endif /* __USBPD_HW_IF_H_ */

