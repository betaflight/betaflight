/**
  ******************************************************************************
  * @file    usbpd_phy.c
  * @author  MCD Application Team
  * @brief   This file contains PHY layer functions.
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

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
#include "usbpd_phy.h"
#include "usbpd_hw_if.h"
#include "usbpd_pwr_if.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_PHY
  * @brief   This file contains PHY layer functions.
  * @details Receive from PRL a message and create a structured packet (according to the USBPD specifications):
  *          |SOP|DATA:[HEADER|DATAOBJECTS]|CRC|EOP|
  * @{
  */

/* Private defines -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_PHY_Private_typedef USBPD DEVICE PHY Private typedef
  * @brief Structures and enums internally used by the PHY layer
  * @{
  */


/**
  * @brief Handle to support the data of the layer
  */
typedef struct
{
  /**
    * @brief  Reports that a message has been received on a specified port.
    * @note   Received data are stored inside PortNum->pRxBuffPtr
    *         function called in the interrupt context
    * @param  PortNum The handle of the port
    * @param  Type    The type of the message received @ref USBPD_SOPType_TypeDef
    * @retval None
    */
  void (*USBPD_PHY_MessageReceived)(uint8_t PortNum, USBPD_SOPType_TypeDef Type);

  uint32_t  SupportedSOP;        /*!<bit field SOP"Debug SOP'Debug SOP" SOP' SOP */
} PHY_HandleTypeDef;

/**
  * @brief prototype definition shared in several callbacks
  */
typedef void (*PHY_CB_t)(uint8_t PortNum, USBPD_SOPType_TypeDef Type); /*!<  Common callback definition */

/**
  * @}
  */

/* Private define and macro --------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_PHY_Private_variables USBPD DEVICE PHY Private variables
  * @{
  */

/** Internal struct for RXTX */
static PHY_HandleTypeDef PHY_Ports[USBPD_PORT_COUNT];
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup USBPD_DEVICE_PHY_Private_functions USBPD DEVICE PHY Private functions
  * @{
  */
USBPD_StatusTypeDef         PHY_PortInit(uint8_t PortNum, const USBPD_PHY_Callbacks *cbs, uint8_t *pRxBuffer,
                                         uint32_t SupportedSOP);
void                        PHY_ResetCompleted(uint8_t PortNum, USBPD_SOPType_TypeDef Type);
void                        PHY_Rx_HardReset(uint8_t PortNum);
void                        PHY_Rx_Completed(uint8_t PortNum, uint32_t MsgType);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_PHY_Exported_Functions USBPD DEVICE PHY Exported functions
  * @{
  */
/**
  * @brief  Initialize the PHY of a specified port.
  * @param  PortNum       Number of the port.
  * @param  pCallbacks    PHY callbacks
  * @param  pRxBuffer     Buffer to storage received message.
  * @param  PowerRole     Power Role of the board.
  * @param  SupportedSOP  bit field of the supported SOP
  * @retval status        @ref USBPD_OK
  */
USBPD_StatusTypeDef USBPD_PHY_Init(uint8_t PortNum, const USBPD_PHY_Callbacks *pCallbacks, uint8_t *pRxBuffer,
                                   USBPD_PortPowerRole_TypeDef PowerRole, uint32_t SupportedSOP)
{
  (void)PowerRole;

  /* set all callbacks */
  Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted            = pCallbacks->USBPD_PHY_TxCompleted;
  Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted          = pCallbacks->USBPD_PHY_BistCompleted;
  Ports[PortNum].cbs.USBPD_HW_IF_RX_ResetIndication     = pCallbacks->USBPD_PHY_ResetIndication;
  Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed           = PHY_Rx_Completed;
  Ports[PortNum].cbs.USBPD_HW_IF_TX_HardResetCompleted  = pCallbacks->USBPD_PHY_ResetCompleted;
  Ports[PortNum].cbs.USBPD_HW_IF_TX_FRSReception        = pCallbacks->USBPD_PHY_FastRoleSwapReception;
  /* Initialize the hardware for the port */
  Ports[PortNum].ptr_RxBuff = pRxBuffer;

  /* Initialize port related functionalities inside this layer */
  PHY_Ports[PortNum].SupportedSOP = SupportedSOP;
  PHY_Ports[PortNum].USBPD_PHY_MessageReceived = pCallbacks->USBPD_PHY_MessageReceived;

  return USBPD_OK;
}

/**
  * @brief  this function return the retry counter value in us.
  * @note   time used to determine when the protocol layer must re-send a message not acknowledged by a goodCRC
  * @param  PortNum    Number of the port.
  * @retval retry counter value in us.
  */
uint16_t USBPD_PHY_GetRetryTimerValue(uint8_t PortNum)
{
  (void)PortNum;
  return 905u;
}

/**
  * @brief  this function return the min time to wait before sending a goodCRC to ack a message (in us).
  * @note   time used to guarantee the min time of 26us between two PD message.
  * @param  PortNum    Number of the port.
  * @retval value in us.
  */
uint16_t USBPD_PHY_GetMinGOODCRCTimerValue(uint8_t PortNum)
{
  return 30u;
}

/**
  * @brief  Reset the PHY of a specified port.
  * @param  PortNum    Number of the port.
  * @retval None
  */
void USBPD_PHY_Reset(uint8_t PortNum)
{
  (void)PortNum;
  /* reset PHY layer   */
  /* reset HW_IF layer */
}

/**
  * @brief  Request to send a reset on a port.
  * @param  PortNum Number of the port
  * @param  Type    Type of reset (hard or cable reset) @ref USBPD_SOPTYPE_HARD_RESET or @ref USBPD_SOPTYPE_CABLE_RESET
  * @retval status  @ref USBPD_OK
  */
USBPD_StatusTypeDef USBPD_PHY_ResetRequest(uint8_t PortNum, USBPD_SOPType_TypeDef Type)
{
  /* Send the requested reset */
  return USBPD_PHY_SendMessage(PortNum, Type, NULL, 0);
}

/**
  * @brief  Send a Message.
  * @param  PortNum   Number of the port
  * @param  Type      Type of the message
  * @param  pBuffer   Pointer to the buffer to be transmitted
  * @param  Size      Size of the buffer (bytes)
  * @retval status    @ref USBPD_OK
  */
USBPD_StatusTypeDef USBPD_PHY_SendMessage(uint8_t PortNum, USBPD_SOPType_TypeDef Type, uint8_t *pBuffer, uint16_t Size)
{
  /* Trace to track message */
  return USBPD_HW_IF_SendBuffer(PortNum, Type, pBuffer,  Size);
}

/**
  * @brief  Send BIST pattern.
  * @param  PortNum   Number of the port
  * @retval status    @ref USBPD_OK
  */
USBPD_StatusTypeDef USBPD_PHY_Send_BIST_Pattern(uint8_t PortNum)
{
  /* Call the low-level function (HW_IF) to accomplish the BIST Carrier Mode Transmission */
  USBPD_HW_IF_Send_BIST_Pattern(PortNum);
  return USBPD_OK;
}

/**
  * @brief  Request PHY to exit of BIST mode 2
  * @param  PortNum port number value
  * @param  mode    SOP BIST MODE 2
  * @retval USBPD   status
  */
USBPD_StatusTypeDef USBPD_PHY_ExitTransmit(uint8_t PortNum, USBPD_SOPType_TypeDef mode)
{
  if (USBPD_SOPTYPE_BIST_MODE_2 == mode)
  {
    USBPD_HW_IF_StopBISTMode2(PortNum);
  }
  return USBPD_OK;
}

/**
  * @brief  Set the SinkTxNg value of the resistor,
  * @note   used to manage the collision avoidance
  * @param  PortNum  Number of the port
  * @retval None
  */
void USBPD_PHY_SetResistor_SinkTxNG(uint8_t PortNum)
{
  USBPD_HW_IF_SetResistor_SinkTxNG(PortNum);
}

/**
  * @brief  function to set the SinkTxOK
  * @note   used to manage the collision avoidance
  * @param  PortNum  Number of the port.
  * @retval none.
  */
void USBPD_PHY_SetResistor_SinkTxOK(uint8_t PortNum)
{
  USBPD_HW_IF_SetResistor_SinkTxOK(PortNum);
}

/**
  * @brief  function to set the supported SOP
  * @param  PortNum  Number of the port.
  * @param  SOPSupported  List of the supported SOP
  * @retval None.
  */
void USBPD_PHY_SOPSupported(uint8_t PortNum, uint32_t SOPSupported)
{
  PHY_Ports[PortNum].SupportedSOP = SOPSupported;
}

/**
  * @brief  Check if SinkTxOK is set or not
  * @note   used to manage the collision avoidance
  * @param  PortNum  Number of the port.
  * @retval USBPD_TRUE or USBPD_FALSE
  */
uint8_t USBPD_PHY_IsResistor_SinkTxOk(uint8_t PortNum)
{
  return USBPD_HW_IF_IsResistor_SinkTxOk(PortNum);
}

/**
  * @brief  function to generate an FRS signalling
  * @param  PortNum  Number of the port.
  * @retval None.
  */
void USBPD_PHY_FastRoleSwapSignalling(uint8_t PortNum)
{
  USBPD_HW_IF_FastRoleSwapSignalling(PortNum);
}

/**
  * @brief  function used to enable RX
  * @param  PortNum    Number of the port.
  * @retval None
  */
void USBPD_PHY_EnableRX(uint8_t PortNum)
{
  USBPD_HW_IF_EnableRX(PortNum);
}

/**
  * @brief  function used to disable RX
  * @param  PortNum    Number of the port.
  * @retval None
  */
void USBPD_PHY_DisableRX(uint8_t PortNum)
{
  USBPD_HW_IF_DisableRX(PortNum);
}

/**
  * @}
  */

/** @addtogroup USBPD_DEVICE_PHY_Private_functions
  * @brief PHY internally used functions
  * @{
  */

/**
  * @brief  Callback to notify the end of the current reception
  * @param  PortNum   Number of the port.
  * @param  MsgType   SOP Message Type
  * @retval None.
  */
void PHY_Rx_Completed(uint8_t PortNum, uint32_t MsgType)
{
  const USBPD_SOPType_TypeDef tab_sop_value[] =
  {
    USBPD_SOPTYPE_SOP, USBPD_SOPTYPE_SOP1, USBPD_SOPTYPE_SOP2,
    USBPD_SOPTYPE_SOP1_DEBUG, USBPD_SOPTYPE_SOP2_DEBUG, USBPD_SOPTYPE_CABLE_RESET
  };
  USBPD_SOPType_TypeDef _msgtype;

  _msgtype = tab_sop_value[MsgType];

  /* check if the message must be forwarded to usbpd stack */
  switch (_msgtype)
  {
    case USBPD_SOPTYPE_CABLE_RESET :
      if (0x1Eu == (PHY_Ports[PortNum].SupportedSOP & 0x1Eu))
      {
        /* nothing to do the message will be discarded and the port partner retry the send */
        Ports[PortNum].cbs.USBPD_HW_IF_RX_ResetIndication(PortNum, USBPD_SOPTYPE_CABLE_RESET);
      }
      break;
    case USBPD_SOPTYPE_SOP :
    case USBPD_SOPTYPE_SOP1 :
    case USBPD_SOPTYPE_SOP2 :
    case USBPD_SOPTYPE_SOP1_DEBUG :
    case USBPD_SOPTYPE_SOP2_DEBUG :
      if (!((uint8_t)(0x1u << _msgtype) != (PHY_Ports[PortNum].SupportedSOP & (uint8_t)(0x1u << _msgtype))))
      {
        PHY_Ports[PortNum].USBPD_PHY_MessageReceived(PortNum, _msgtype);
      }
#if defined(DEBUG_NOTFWD)
      else
      {
        typedef union
        {
          uint16_t d16;
          struct
          {
            uint16_t MessageType            :5;   /*!< Message Header's message Type                      */
            uint16_t PortDataRole           :1;   /*!< Message Header's Port Data Role                    */
            uint16_t SpecificationRevision  :2;   /*!< Message Header's Spec Revision                     */
            uint16_t PortPowerRole_CablePlug:1;   /*!< Message Header's Port Power Role/Cable Plug field  */
            uint16_t MessageID              :3;   /*!< Message Header's message ID                        */
            uint16_t NumberOfDataObjects    :3;   /*!< Message Header's Number of data object             */
            uint16_t Extended               :1;   /*!< Reserved                                           */
          }
          b;
        } USBPD_MsgHeader_TypeDef;

        USBPD_MsgHeader_TypeDef header_rx;
        header_rx.d16 = USBPD_LE16(Ports[PortNum].ptr_RxBuff);
        USBPD_TRACE_Add(USBPD_TRACE_PHY_NOTFRWD, PortNum, _msgtype, Ports[PortNum].ptr_RxBuff,
                        2u + (header_rx.b.NumberOfDataObjects * 4u));
      }
#endif /* DEBUG_NOTFWD */
      break;
    default :
      break;
  }
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

/**
  * @}
  */

