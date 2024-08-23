/**
  *
  * @file    apm32f4xx_dal_smbus.c
  * @brief   SMBUS DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the System Management Bus (SMBus) peripheral,
  *          based on SMBUS principals of operation :
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State, Mode and Error functions
  *
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
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
    The SMBUS DAL driver can be used as follows:

    (#) Declare a SMBUS_HandleTypeDef handle structure, for example:
        SMBUS_HandleTypeDef  hsmbus;

    (#)Initialize the SMBUS low level resources by implementing the DAL_SMBUS_MspInit() API:
        (##) Enable the SMBUSx interface clock
        (##) SMBUS pins configuration
            (+++) Enable the clock for the SMBUS GPIOs
            (+++) Configure SMBUS pins as alternate function open-drain
        (##) NVIC configuration if you need to use interrupt process
            (+++) Configure the SMBUSx interrupt priority
            (+++) Enable the NVIC SMBUS IRQ Channel

    (#) Configure the Communication Speed, Duty cycle, Addressing mode, Own Address1,
        Dual Addressing mode, Own Address2, General call and Nostretch mode in the hsmbus Init structure.

    (#) Initialize the SMBUS registers by calling the DAL_SMBUS_Init(), configures also the low level Hardware
        (GPIO, CLOCK, NVIC...etc) by calling the customized DAL_SMBUS_MspInit(&hsmbus) API.

    (#) To check if target device is ready for communication, use the function DAL_SMBUS_IsDeviceReady()

    (#) For SMBUS IO operations, only one mode of operations is available within this driver :


    *** Interrupt mode IO operation ***
    ===================================

  [..]
      (+) Transmit in master/host SMBUS mode an amount of data in non blocking mode using DAL_SMBUS_Master_Transmit_IT()
      (++) At transmission end of transfer DAL_SMBUS_MasterTxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_SMBUS_MasterTxCpltCallback()
      (+) Receive in master/host SMBUS mode an amount of data in non blocking mode using DAL_SMBUS_Master_Receive_IT()
      (++) At reception end of transfer DAL_SMBUS_MasterRxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_SMBUS_MasterRxCpltCallback()
      (+) Abort a master/Host SMBUS process communication with Interrupt using DAL_SMBUS_Master_Abort_IT()
      (++) End of abort process, DAL_SMBUS_AbortCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_SMBUS_AbortCpltCallback()
      (+) Enable/disable the Address listen mode in slave/device or host/slave SMBUS mode
           using DAL_SMBUS_EnableListen_IT() DAL_SMBUS_DisableListen_IT()
      (++) When address slave/device SMBUS match, DAL_SMBUS_AddrCallback() is executed and user can
           add his own code to check the Address Match Code and the transmission direction request by master/host (Write/Read).
      (++) At Listen mode end DAL_SMBUS_ListenCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_SMBUS_ListenCpltCallback()
      (+) Transmit in slave/device SMBUS mode an amount of data in non blocking mode using DAL_SMBUS_Slave_Transmit_IT()
      (++) At transmission end of transfer DAL_SMBUS_SlaveTxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_SMBUS_SlaveTxCpltCallback()
      (+) Receive in slave/device SMBUS mode an amount of data in non blocking mode using DAL_SMBUS_Slave_Receive_IT()
      (++) At reception end of transfer DAL_SMBUS_SlaveRxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_SMBUS_SlaveRxCpltCallback()
      (+) Enable/Disable the SMBUS alert mode using DAL_SMBUS_EnableAlert_IT() and DAL_SMBUS_DisableAlert_IT()
      (++) When SMBUS Alert is generated DAL_SMBUS_ErrorCallback() is executed and user can
           add his own code by customization of function pointer DAL_SMBUS_ErrorCallback()
           to check the Alert Error Code using function DAL_SMBUS_GetError()
      (+) Get DAL state machine or error values using DAL_SMBUS_GetState() or DAL_SMBUS_GetError()
      (+) In case of transfer Error, DAL_SMBUS_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer DAL_SMBUS_ErrorCallback()
           to check the Error Code using function DAL_SMBUS_GetError()


     *** SMBUS DAL driver macros list ***
     ==================================
     [..]
       Below the list of most used macros in SMBUS DAL driver.

      (+) __DAL_SMBUS_ENABLE    : Enable the SMBUS peripheral
      (+) __DAL_SMBUS_DISABLE   : Disable the SMBUS peripheral
      (+) __DAL_SMBUS_GET_FLAG  : Checks whether the specified SMBUS flag is set or not
      (+) __DAL_SMBUS_CLEAR_FLAG: Clear the specified SMBUS pending flag
      (+) __DAL_SMBUS_ENABLE_IT : Enable the specified SMBUS interrupt
      (+) __DAL_SMBUS_DISABLE_IT: Disable the specified SMBUS interrupt

     [..]
       (@) You can refer to the SMBUS DAL driver header file for more useful macros

     *** Callback registration ***
     =============================================
    [..]
     The compilation flag USE_DAL_SMBUS_REGISTER_CALLBACKS when set to 1
     allows the user to configure dynamically the driver callbacks.
     Use Functions DAL_SMBUS_RegisterCallback() or DAL_SMBUS_RegisterXXXCallback()
     to register an interrupt callback.

     Function DAL_SMBUS_RegisterCallback() allows to register following callbacks:
       (+) MasterTxCpltCallback : callback for Master transmission end of transfer.
       (+) MasterRxCpltCallback : callback for Master reception end of transfer.
       (+) SlaveTxCpltCallback  : callback for Slave transmission end of transfer.
       (+) SlaveRxCpltCallback  : callback for Slave reception end of transfer.
       (+) ListenCpltCallback   : callback for end of listen mode.
       (+) ErrorCallback        : callback for error detection.
       (+) AbortCpltCallback    : callback for abort completion process.
       (+) MspInitCallback      : callback for Msp Init.
       (+) MspDeInitCallback    : callback for Msp DeInit.
     This function takes as parameters the DAL peripheral handle, the Callback ID
     and a pointer to the user callback function.
    [..]
     For specific callback AddrCallback use dedicated register callbacks : DAL_SMBUS_RegisterAddrCallback().
    [..]
     Use function DAL_SMBUS_UnRegisterCallback to reset a callback to the default
     weak function.
     DAL_SMBUS_UnRegisterCallback takes as parameters the DAL peripheral handle,
     and the Callback ID.
     This function allows to reset following callbacks:
       (+) MasterTxCpltCallback : callback for Master transmission end of transfer.
       (+) MasterRxCpltCallback : callback for Master reception end of transfer.
       (+) SlaveTxCpltCallback  : callback for Slave transmission end of transfer.
       (+) SlaveRxCpltCallback  : callback for Slave reception end of transfer.
       (+) ListenCpltCallback   : callback for end of listen mode.
       (+) ErrorCallback        : callback for error detection.
       (+) AbortCpltCallback    : callback for abort completion process.
       (+) MspInitCallback      : callback for Msp Init.
       (+) MspDeInitCallback    : callback for Msp DeInit.
    [..]
     For callback AddrCallback use dedicated register callbacks : DAL_SMBUS_UnRegisterAddrCallback().
    [..]
     By default, after the DAL_SMBUS_Init() and when the state is DAL_SMBUS_STATE_RESET
     all callbacks are set to the corresponding weak functions:
     examples DAL_SMBUS_MasterTxCpltCallback(), DAL_SMBUS_MasterRxCpltCallback().
     Exception done for MspInit and MspDeInit functions that are
     reset to the legacy weak functions in the DAL_SMBUS_Init()/ DAL_SMBUS_DeInit() only when
     these callbacks are null (not registered beforehand).
     If MspInit or MspDeInit are not null, the DAL_SMBUS_Init()/ DAL_SMBUS_DeInit()
     keep and use the user MspInit/MspDeInit callbacks (registered beforehand) whatever the state.
    [..]
     Callbacks can be registered/unregistered in DAL_SMBUS_STATE_READY state only.
     Exception done MspInit/MspDeInit functions that can be registered/unregistered
     in DAL_SMBUS_STATE_READY or DAL_SMBUS_STATE_RESET state,
     thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
     Then, the user first registers the MspInit/MspDeInit user callbacks
     using DAL_SMBUS_RegisterCallback() before calling DAL_SMBUS_DeInit()
     or DAL_SMBUS_Init() function.
    [..]
     When the compilation flag USE_DAL_SMBUS_REGISTER_CALLBACKS is set to 0 or
     not defined, the callback registration feature is not available and all callbacks
     are set to the corresponding weak functions.

  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup SMBUS SMBUS
  * @brief SMBUS DAL module driver
  * @{
  */

#ifdef DAL_SMBUS_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup SMBUS_Private_Define
  * @{
  */
#define SMBUS_TIMEOUT_FLAG          35U         /*!< Timeout 35 ms             */
#define SMBUS_TIMEOUT_BUSY_FLAG     25U         /*!< Timeout 25 ms             */
#define SMBUS_NO_OPTION_FRAME       0xFFFF0000U /*!< XferOptions default value */

#define SMBUS_SENDPEC_MODE          I2C_CTRL1_PEC
#define SMBUS_GET_PEC(__HANDLE__)             (((__HANDLE__)->Instance->STS2 & I2C_STS2_PECVALUE) >> 8)

/* Private define for @ref PreviousState usage */
#define SMBUS_STATE_MSK             ((uint32_t)((DAL_SMBUS_STATE_BUSY_TX | DAL_SMBUS_STATE_BUSY_RX) & (~(uint32_t)DAL_SMBUS_STATE_READY))) /*!< Mask State define, keep only RX and TX bits            */
#define SMBUS_STATE_NONE            ((uint32_t)(DAL_SMBUS_MODE_NONE))                                                                      /*!< Default Value                                          */
#define SMBUS_STATE_MASTER_BUSY_TX  ((uint32_t)((DAL_SMBUS_STATE_BUSY_TX & SMBUS_STATE_MSK) | DAL_SMBUS_MODE_MASTER))                      /*!< Master Busy TX, combinaison of State LSB and Mode enum */
#define SMBUS_STATE_MASTER_BUSY_RX  ((uint32_t)((DAL_SMBUS_STATE_BUSY_RX & SMBUS_STATE_MSK) | DAL_SMBUS_MODE_MASTER))                      /*!< Master Busy RX, combinaison of State LSB and Mode enum */
#define SMBUS_STATE_SLAVE_BUSY_TX   ((uint32_t)((DAL_SMBUS_STATE_BUSY_TX & SMBUS_STATE_MSK) | DAL_SMBUS_MODE_SLAVE))                       /*!< Slave Busy TX, combinaison of State LSB and Mode enum  */
#define SMBUS_STATE_SLAVE_BUSY_RX   ((uint32_t)((DAL_SMBUS_STATE_BUSY_RX & SMBUS_STATE_MSK) | DAL_SMBUS_MODE_SLAVE))                       /*!< Slave Busy RX, combinaison of State LSB and Mode enum  */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @addtogroup SMBUS_Private_Functions
  * @{
  */

static DAL_StatusTypeDef SMBUS_WaitOnFlagUntilTimeout(SMBUS_HandleTypeDef *hsmbus, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
static void SMBUS_ITError(SMBUS_HandleTypeDef *hsmbus);

/* Private functions for SMBUS transfer IRQ handler */
static DAL_StatusTypeDef SMBUS_MasterTransmit_TXE(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_MasterTransmit_BTF(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_MasterReceive_RXNE(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_MasterReceive_BTF(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_Master_SB(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_Master_ADD10(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_Master_ADDR(SMBUS_HandleTypeDef *hsmbus);

static DAL_StatusTypeDef SMBUS_SlaveTransmit_TXE(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_SlaveTransmit_BTF(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_SlaveReceive_RXNE(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_SlaveReceive_BTF(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_Slave_ADDR(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_Slave_STOPF(SMBUS_HandleTypeDef *hsmbus);
static DAL_StatusTypeDef SMBUS_Slave_AF(SMBUS_HandleTypeDef *hsmbus);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SMBUS_Exported_Functions SMBUS Exported Functions
  * @{
  */

/** @defgroup SMBUS_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the SMBUSx peripheral:

      (+) User must Implement DAL_SMBUS_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, IT and NVIC).

      (+) Call the function DAL_SMBUS_Init() to configure the selected device with
          the selected configuration:
        (++) Communication Speed
        (++) Addressing mode
        (++) Own Address 1
        (++) Dual Addressing mode
        (++) Own Address 2
        (++) General call mode
        (++) Nostretch mode
        (++) Packet Error Check mode
        (++) Peripheral mode

      (+) Call the function DAL_SMBUS_DeInit() to restore the default configuration
          of the selected SMBUSx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the SMBUS according to the specified parameters
  *         in the SMBUS_InitTypeDef and initialize the associated handle.
  * @param  hsmbus pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_Init(SMBUS_HandleTypeDef *hsmbus)
{
  uint32_t freqrange = 0U;
  uint32_t pclk1 = 0U;

  /* Check the SMBUS handle allocation */
  if (hsmbus == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_SMBUS_ALL_INSTANCE(hsmbus->Instance));
#if  defined(I2C_FILTER_ANFDIS)
  ASSERT_PARAM(IS_SMBUS_ANALOG_FILTER(hsmbus->Init.AnalogFilter));
#endif
  ASSERT_PARAM(IS_SMBUS_CLOCK_SPEED(hsmbus->Init.ClockSpeed));
  ASSERT_PARAM(IS_SMBUS_OWN_ADDRESS1(hsmbus->Init.OwnAddress1));
  ASSERT_PARAM(IS_SMBUS_ADDRESSING_MODE(hsmbus->Init.AddressingMode));
  ASSERT_PARAM(IS_SMBUS_DUAL_ADDRESS(hsmbus->Init.DualAddressMode));
  ASSERT_PARAM(IS_SMBUS_OWN_ADDRESS2(hsmbus->Init.OwnAddress2));
  ASSERT_PARAM(IS_SMBUS_GENERAL_CALL(hsmbus->Init.GeneralCallMode));
  ASSERT_PARAM(IS_SMBUS_NO_STRETCH(hsmbus->Init.NoStretchMode));
  ASSERT_PARAM(IS_SMBUS_PEC(hsmbus->Init.PacketErrorCheckMode));
  ASSERT_PARAM(IS_SMBUS_PERIPHERAL_MODE(hsmbus->Init.PeripheralMode));

  if (hsmbus->State == DAL_SMBUS_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hsmbus->Lock = DAL_UNLOCKED;
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
    /* Init the SMBUS Callback settings */
    hsmbus->MasterTxCpltCallback = DAL_SMBUS_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
    hsmbus->MasterRxCpltCallback = DAL_SMBUS_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
    hsmbus->SlaveTxCpltCallback  = DAL_SMBUS_SlaveTxCpltCallback;  /* Legacy weak SlaveTxCpltCallback  */
    hsmbus->SlaveRxCpltCallback  = DAL_SMBUS_SlaveRxCpltCallback;  /* Legacy weak SlaveRxCpltCallback  */
    hsmbus->ListenCpltCallback   = DAL_SMBUS_ListenCpltCallback;   /* Legacy weak ListenCpltCallback   */
    hsmbus->ErrorCallback        = DAL_SMBUS_ErrorCallback;        /* Legacy weak ErrorCallback        */
    hsmbus->AbortCpltCallback    = DAL_SMBUS_AbortCpltCallback;    /* Legacy weak AbortCpltCallback    */
    hsmbus->AddrCallback         = DAL_SMBUS_AddrCallback;         /* Legacy weak AddrCallback         */

    if (hsmbus->MspInitCallback == NULL)
    {
      hsmbus->MspInitCallback = DAL_SMBUS_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    hsmbus->MspInitCallback(hsmbus);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    DAL_SMBUS_MspInit(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
  }

  hsmbus->State = DAL_SMBUS_STATE_BUSY;

  /* Disable the selected SMBUS peripheral */
  __DAL_SMBUS_DISABLE(hsmbus);

  /* Get PCLK1 frequency */
  pclk1 = DAL_RCM_GetPCLK1Freq();

  /* Calculate frequency range */
  freqrange = SMBUS_FREQRANGE(pclk1);

  /*---------------------------- SMBUSx CTRL2 Configuration ----------------------*/
  /* Configure SMBUSx: Frequency range */
  MODIFY_REG(hsmbus->Instance->CTRL2, I2C_CTRL2_CLKFCFG, freqrange);

  /*---------------------------- SMBUSx RISETMAX Configuration --------------------*/
  /* Configure SMBUSx: Rise Time */
  MODIFY_REG(hsmbus->Instance->RISETMAX, I2C_RISETMAX_RISETMAX, SMBUS_RISE_TIME(freqrange));

  /*---------------------------- SMBUSx CLKCTRL Configuration ----------------------*/
  /* Configure SMBUSx: Speed */
  MODIFY_REG(hsmbus->Instance->CLKCTRL, (I2C_CLKCTRL_SPEEDCFG | I2C_CLKCTRL_FDUTYCFG | I2C_CLKCTRL_CLKS), SMBUS_SPEED_STANDARD(pclk1, hsmbus->Init.ClockSpeed));

  /*---------------------------- SMBUSx CTRL1 Configuration ----------------------*/
  /* Configure SMBUSx: Generalcall , PEC , Peripheral mode and  NoStretch mode */
  MODIFY_REG(hsmbus->Instance->CTRL1, (I2C_CTRL1_CLKSTRETCHD | I2C_CTRL1_SRBEN | I2C_CTRL1_PECEN | I2C_CTRL1_ARPEN | I2C_CTRL1_SMBTCFG | I2C_CTRL1_SMBEN), (hsmbus->Init.NoStretchMode | hsmbus->Init.GeneralCallMode |  hsmbus->Init.PacketErrorCheckMode | hsmbus->Init.PeripheralMode));

  /*---------------------------- SMBUSx SADDR1 Configuration ---------------------*/
  /* Configure SMBUSx: Own Address1 and addressing mode */
  MODIFY_REG(hsmbus->Instance->SADDR1, (I2C_SADDR1_ADDRLEN | I2C_SADDR1_ADDR8_9 | I2C_SADDR1_ADDR1_7 | I2C_SADDR1_ADDR0), (hsmbus->Init.AddressingMode | hsmbus->Init.OwnAddress1));

  /*---------------------------- SMBUSx SADDR2 Configuration ---------------------*/
  /* Configure SMBUSx: Dual mode and Own Address2 */
  MODIFY_REG(hsmbus->Instance->SADDR2, (I2C_SADDR2_ADDRNUM | I2C_SADDR2_ADDR2), (hsmbus->Init.DualAddressMode | hsmbus->Init.OwnAddress2));
#if  defined(I2C_FILTER_ANFDIS)
  /*---------------------------- SMBUSx FLTR Configuration ------------------------*/
  /* Configure SMBUSx: Analog noise filter */
  SET_BIT(hsmbus->Instance->FILTER, hsmbus->Init.AnalogFilter);
#endif

  /* Enable the selected SMBUS peripheral */
  __DAL_SMBUS_ENABLE(hsmbus);

  hsmbus->ErrorCode = DAL_SMBUS_ERROR_NONE;
  hsmbus->State = DAL_SMBUS_STATE_READY;
  hsmbus->PreviousState = SMBUS_STATE_NONE;
  hsmbus->Mode = DAL_SMBUS_MODE_NONE;
  hsmbus->XferPEC = 0x00;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the SMBUS peripheral.
  * @param  hsmbus pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_DeInit(SMBUS_HandleTypeDef *hsmbus)
{
  /* Check the SMBUS handle allocation */
  if (hsmbus == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_SMBUS_ALL_INSTANCE(hsmbus->Instance));

  hsmbus->State = DAL_SMBUS_STATE_BUSY;

  /* Disable the SMBUS Peripheral Clock */
  __DAL_SMBUS_DISABLE(hsmbus);

#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
  if (hsmbus->MspDeInitCallback == NULL)
  {
    hsmbus->MspDeInitCallback = DAL_SMBUS_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  hsmbus->MspDeInitCallback(hsmbus);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  DAL_SMBUS_MspDeInit(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */

  hsmbus->ErrorCode     = DAL_SMBUS_ERROR_NONE;
  hsmbus->State         = DAL_SMBUS_STATE_RESET;
  hsmbus->PreviousState = SMBUS_STATE_NONE;
  hsmbus->Mode          = DAL_SMBUS_MODE_NONE;

  /* Release Lock */
  __DAL_UNLOCK(hsmbus);

  return DAL_OK;
}

/**
  * @brief  Initialize the SMBUS MSP.
  * @param  hsmbus pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS
  * @retval None
  */
__weak void DAL_SMBUS_MspInit(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_SMBUS_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitialize the SMBUS MSP.
  * @param  hsmbus pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS
  * @retval None
  */
__weak void DAL_SMBUS_MspDeInit(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_SMBUS_MspDeInit could be implemented in the user file
   */
}

#if defined(I2C_FILTER_ANFDIS) && defined(I2C_FILTER_DNFCFG)
/**
  * @brief  Configures SMBUS Analog noise filter.
  * @param  hsmbus pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUSx peripheral.
  * @param  AnalogFilter new state of the Analog filter.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_ConfigAnalogFilter(SMBUS_HandleTypeDef *hsmbus, uint32_t AnalogFilter)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMBUS_ALL_INSTANCE(hsmbus->Instance));
  ASSERT_PARAM(IS_SMBUS_ANALOG_FILTER(AnalogFilter));

  if (hsmbus->State == DAL_SMBUS_STATE_READY)
  {
    hsmbus->State = DAL_SMBUS_STATE_BUSY;

    /* Disable the selected SMBUS peripheral */
    __DAL_SMBUS_DISABLE(hsmbus);

    /* Reset SMBUSx ANFDIS bit */
    hsmbus->Instance->FILTER &= ~(I2C_FILTER_ANFDIS);

    /* Disable the analog filter */
    hsmbus->Instance->FILTER |= AnalogFilter;

    __DAL_SMBUS_ENABLE(hsmbus);

    hsmbus->State = DAL_SMBUS_STATE_READY;

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Configures SMBUS Digital noise filter.
  * @param  hsmbus pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUSx peripheral.
  * @param  DigitalFilter Coefficient of digital noise filter between 0x00 and 0x0F.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_ConfigDigitalFilter(SMBUS_HandleTypeDef *hsmbus, uint32_t DigitalFilter)
{
  uint16_t tmpreg = 0;

  /* Check the parameters */
  ASSERT_PARAM(IS_SMBUS_ALL_INSTANCE(hsmbus->Instance));
  ASSERT_PARAM(IS_SMBUS_DIGITAL_FILTER(DigitalFilter));

  if (hsmbus->State == DAL_SMBUS_STATE_READY)
  {
    hsmbus->State = DAL_SMBUS_STATE_BUSY;

    /* Disable the selected SMBUS peripheral */
    __DAL_SMBUS_DISABLE(hsmbus);

    /* Get the old register value */
    tmpreg = hsmbus->Instance->FILTER;

    /* Reset SMBUSx DNFCFG bit [3:0] */
    tmpreg &= ~(I2C_FILTER_DNFCFG);

    /* Set SMBUSx DNFCFG coefficient */
    tmpreg |= DigitalFilter;

    /* Store the new register value */
    hsmbus->Instance->FILTER = tmpreg;

    __DAL_SMBUS_ENABLE(hsmbus);

    hsmbus->State = DAL_SMBUS_STATE_READY;

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}
#endif /* I2C_FILTER_ANFDIS && I2C_FILTER_DNFCFG */
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User SMBUS Callback
  *         To be used instead of the weak predefined callback
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref DAL_SMBUS_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref DAL_SMBUS_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref DAL_SMBUS_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref DAL_SMBUS_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref DAL_SMBUS_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref DAL_SMBUS_ERROR_CB_ID Error callback ID
  *          @arg @ref DAL_SMBUS_ABORT_CB_ID Abort callback ID
  *          @arg @ref DAL_SMBUS_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref DAL_SMBUS_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_RegisterCallback(SMBUS_HandleTypeDef *hsmbus, DAL_SMBUS_CallbackIDTypeDef CallbackID, pSMBUS_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hsmbus);

  if (DAL_SMBUS_STATE_READY == hsmbus->State)
  {
    switch (CallbackID)
    {
      case DAL_SMBUS_MASTER_TX_COMPLETE_CB_ID :
        hsmbus->MasterTxCpltCallback = pCallback;
        break;

      case DAL_SMBUS_MASTER_RX_COMPLETE_CB_ID :
        hsmbus->MasterRxCpltCallback = pCallback;
        break;

      case DAL_SMBUS_SLAVE_TX_COMPLETE_CB_ID :
        hsmbus->SlaveTxCpltCallback = pCallback;
        break;

      case DAL_SMBUS_SLAVE_RX_COMPLETE_CB_ID :
        hsmbus->SlaveRxCpltCallback = pCallback;
        break;

      case DAL_SMBUS_LISTEN_COMPLETE_CB_ID :
        hsmbus->ListenCpltCallback = pCallback;
        break;

      case DAL_SMBUS_ERROR_CB_ID :
        hsmbus->ErrorCallback = pCallback;
        break;

      case DAL_SMBUS_ABORT_CB_ID :
        hsmbus->AbortCpltCallback = pCallback;
        break;

      case DAL_SMBUS_MSPINIT_CB_ID :
        hsmbus->MspInitCallback = pCallback;
        break;

      case DAL_SMBUS_MSPDEINIT_CB_ID :
        hsmbus->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hsmbus->ErrorCode |= DAL_SMBUS_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_SMBUS_STATE_RESET == hsmbus->State)
  {
    switch (CallbackID)
    {
      case DAL_SMBUS_MSPINIT_CB_ID :
        hsmbus->MspInitCallback = pCallback;
        break;

      case DAL_SMBUS_MSPDEINIT_CB_ID :
        hsmbus->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hsmbus->ErrorCode |= DAL_SMBUS_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hsmbus);
  return status;
}

/**
  * @brief  Unregister an SMBUS Callback
  *         SMBUS callback is redirected to the weak predefined callback
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *         This parameter can be one of the following values:
  *          @arg @ref DAL_SMBUS_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref DAL_SMBUS_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref DAL_SMBUS_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref DAL_SMBUS_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref DAL_SMBUS_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref DAL_SMBUS_ERROR_CB_ID Error callback ID
  *          @arg @ref DAL_SMBUS_ABORT_CB_ID Abort callback ID
  *          @arg @ref DAL_SMBUS_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref DAL_SMBUS_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_UnRegisterCallback(SMBUS_HandleTypeDef *hsmbus, DAL_SMBUS_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hsmbus);

  if (DAL_SMBUS_STATE_READY == hsmbus->State)
  {
    switch (CallbackID)
    {
      case DAL_SMBUS_MASTER_TX_COMPLETE_CB_ID :
        hsmbus->MasterTxCpltCallback = DAL_SMBUS_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
        break;

      case DAL_SMBUS_MASTER_RX_COMPLETE_CB_ID :
        hsmbus->MasterRxCpltCallback = DAL_SMBUS_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
        break;

      case DAL_SMBUS_SLAVE_TX_COMPLETE_CB_ID :
        hsmbus->SlaveTxCpltCallback = DAL_SMBUS_SlaveTxCpltCallback;   /* Legacy weak SlaveTxCpltCallback  */
        break;

      case DAL_SMBUS_SLAVE_RX_COMPLETE_CB_ID :
        hsmbus->SlaveRxCpltCallback = DAL_SMBUS_SlaveRxCpltCallback;   /* Legacy weak SlaveRxCpltCallback  */
        break;

      case DAL_SMBUS_LISTEN_COMPLETE_CB_ID :
        hsmbus->ListenCpltCallback = DAL_SMBUS_ListenCpltCallback;     /* Legacy weak ListenCpltCallback   */
        break;

      case DAL_SMBUS_ERROR_CB_ID :
        hsmbus->ErrorCallback = DAL_SMBUS_ErrorCallback;               /* Legacy weak ErrorCallback        */
        break;

      case DAL_SMBUS_ABORT_CB_ID :
        hsmbus->AbortCpltCallback = DAL_SMBUS_AbortCpltCallback;       /* Legacy weak AbortCpltCallback    */
        break;

      case DAL_SMBUS_MSPINIT_CB_ID :
        hsmbus->MspInitCallback = DAL_SMBUS_MspInit;                   /* Legacy weak MspInit              */
        break;

      case DAL_SMBUS_MSPDEINIT_CB_ID :
        hsmbus->MspDeInitCallback = DAL_SMBUS_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        hsmbus->ErrorCode |= DAL_SMBUS_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_SMBUS_STATE_RESET == hsmbus->State)
  {
    switch (CallbackID)
    {
      case DAL_SMBUS_MSPINIT_CB_ID :
        hsmbus->MspInitCallback = DAL_SMBUS_MspInit;                   /* Legacy weak MspInit              */
        break;

      case DAL_SMBUS_MSPDEINIT_CB_ID :
        hsmbus->MspDeInitCallback = DAL_SMBUS_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        hsmbus->ErrorCode |= DAL_SMBUS_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hsmbus);
  return status;
}

/**
  * @brief  Register the Slave Address Match SMBUS Callback
  *         To be used instead of the weak DAL_SMBUS_AddrCallback() predefined callback
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  pCallback pointer to the Address Match Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_RegisterAddrCallback(SMBUS_HandleTypeDef *hsmbus, pSMBUS_AddrCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hsmbus);

  if (DAL_SMBUS_STATE_READY == hsmbus->State)
  {
    hsmbus->AddrCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hsmbus);
  return status;
}

/**
  * @brief  UnRegister the Slave Address Match SMBUS Callback
  *         Info Ready SMBUS Callback is redirected to the weak DAL_SMBUS_AddrCallback() predefined callback
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_UnRegisterAddrCallback(SMBUS_HandleTypeDef *hsmbus)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hsmbus);

  if (DAL_SMBUS_STATE_READY == hsmbus->State)
  {
    hsmbus->AddrCallback = DAL_SMBUS_AddrCallback; /* Legacy weak AddrCallback  */
  }
  else
  {
    /* Update the error code */
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hsmbus);
  return status;
}

#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup SMBUS_Exported_Functions_Group2 Input and Output operation functions
 *  @brief    Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the SMBUS data
    transfers.

    (#) Blocking mode function to check if device is ready for usage is :
        (++) DAL_SMBUS_IsDeviceReady()

    (#) There is only one mode of transfer:
       (++) Non Blocking mode : The communication is performed using Interrupts.
            These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated SMBUS IRQ when using Interrupt mode.

    (#) Non Blocking mode functions with Interrupt are :
        (++) DAL_SMBUS_Master_Transmit_IT()
        (++) DAL_SMBUS_Master_Receive_IT()
        (++) DAL_SMBUS_Master_Abort_IT()
        (++) DAL_SMBUS_Slave_Transmit_IT()
        (++) DAL_SMBUS_Slave_Receive_IT()
        (++) DAL_SMBUS_EnableAlert_IT()
        (++) DAL_SMBUS_DisableAlert_IT()

    (#) A set of Transfer Complete Callbacks are provided in No_Blocking mode:
        (++) DAL_SMBUS_MasterTxCpltCallback()
        (++) DAL_SMBUS_MasterRxCpltCallback()
        (++) DAL_SMBUS_SlaveTxCpltCallback()
        (++) DAL_SMBUS_SlaveRxCpltCallback()
        (++) DAL_SMBUS_AddrCallback()
        (++) DAL_SMBUS_ListenCpltCallback()
        (++) DAL_SMBUS_ErrorCallback()
        (++) DAL_SMBUS_AbortCpltCallback()

@endverbatim
  * @{
  */

/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_Master_Transmit_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  uint32_t count      = 0x00U;

  /* Check the parameters */
  ASSERT_PARAM(IS_SMBUS_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hsmbus->State == DAL_SMBUS_STATE_READY)
  {
    /* Check Busy Flag only if FIRST call of Master interface */
    if ((XferOptions == SMBUS_FIRST_AND_LAST_FRAME_NO_PEC) || (XferOptions == SMBUS_FIRST_AND_LAST_FRAME_WITH_PEC) || (XferOptions == SMBUS_FIRST_FRAME))
    {
      /* Wait until BUSY flag is reset */
      count = SMBUS_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
      do
      {
        if (count-- == 0U)
        {
          hsmbus->PreviousState = SMBUS_STATE_NONE;
          hsmbus->State = DAL_SMBUS_STATE_READY;

          /* Process Unlocked */
          __DAL_UNLOCK(hsmbus);

          return DAL_TIMEOUT;
        }
      }
      while (__DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_BUSY) != RESET);
    }

    /* Process Locked */
    __DAL_LOCK(hsmbus);

    /* Check if the SMBUS is already enabled */
    if ((hsmbus->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable SMBUS peripheral */
      __DAL_SMBUS_ENABLE(hsmbus);
    }

    /* Disable Pos */
    CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hsmbus->State     = DAL_SMBUS_STATE_BUSY_TX;
    hsmbus->ErrorCode = DAL_SMBUS_ERROR_NONE;
    hsmbus->Mode      = DAL_SMBUS_MODE_MASTER;

    /* Prepare transfer parameters */
    hsmbus->pBuffPtr    = pData;
    hsmbus->XferCount   = Size;
    hsmbus->XferOptions = XferOptions;
    hsmbus->XferSize    = hsmbus->XferCount;
    hsmbus->Devaddress  = DevAddress;

    /* Generate Start */
    SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_START);

    /* Process Unlocked */
    __DAL_UNLOCK(hsmbus);

    /* Note : The SMBUS interrupts must be enabled after unlocking current process
    to avoid the risk of hsmbus interrupt handle execution before current
    process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_SMBUS_ENABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}
/**
  * @brief  Receive in master/host SMBUS mode an amount of data in non blocking mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref SMBUS_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_Master_Receive_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  __IO uint32_t count = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_SMBUS_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hsmbus->State == DAL_SMBUS_STATE_READY)
  {
    /* Check Busy Flag only if FIRST call of Master interface */
    if ((XferOptions == SMBUS_FIRST_AND_LAST_FRAME_NO_PEC) || (XferOptions == SMBUS_FIRST_AND_LAST_FRAME_WITH_PEC) || (XferOptions == SMBUS_FIRST_FRAME))
    {
      /* Wait until BUSY flag is reset */
      count = SMBUS_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
      do
      {
        if (count-- == 0U)
        {
          hsmbus->PreviousState = SMBUS_STATE_NONE;
          hsmbus->State = DAL_SMBUS_STATE_READY;

          /* Process Unlocked */
          __DAL_UNLOCK(hsmbus);

          return DAL_TIMEOUT;
        }
      }
      while (__DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_BUSY) != RESET);
    }

    /* Process Locked */
    __DAL_LOCK(hsmbus);

    /* Check if the SMBUS is already enabled */
    if ((hsmbus->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable SMBUS peripheral */
      __DAL_SMBUS_ENABLE(hsmbus);
    }

    /* Disable Pos */
    CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hsmbus->State     = DAL_SMBUS_STATE_BUSY_RX;
    hsmbus->ErrorCode = DAL_SMBUS_ERROR_NONE;
    hsmbus->Mode      = DAL_SMBUS_MODE_MASTER;

    /* Prepare transfer parameters */
    hsmbus->pBuffPtr    = pData;
    hsmbus->XferCount   = Size;
    hsmbus->XferOptions = XferOptions;
    hsmbus->XferSize    = hsmbus->XferCount;
    hsmbus->Devaddress  = DevAddress;

    if ((hsmbus->PreviousState == SMBUS_STATE_MASTER_BUSY_TX) || (hsmbus->PreviousState == SMBUS_STATE_NONE))
    {
      /* Generate Start condition if first transfer */
      if ((XferOptions == SMBUS_NEXT_FRAME)  || (XferOptions == SMBUS_FIRST_AND_LAST_FRAME_NO_PEC) || (XferOptions == SMBUS_FIRST_AND_LAST_FRAME_WITH_PEC) || (XferOptions == SMBUS_FIRST_FRAME)  || (XferOptions == SMBUS_NO_OPTION_FRAME))
      {
        /* Enable Acknowledge */
        SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

        /* Generate Start */
        SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_START);
      }

      if ((XferOptions == SMBUS_LAST_FRAME_NO_PEC) || (XferOptions == SMBUS_LAST_FRAME_WITH_PEC))
      {
        if (hsmbus->PreviousState == SMBUS_STATE_NONE)
        {
          /* Enable Acknowledge */
          SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);
        }

        if (hsmbus->PreviousState == SMBUS_STATE_MASTER_BUSY_TX)
        {
          /* Enable Acknowledge */
          SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

          /* Generate Start */
          SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_START);
        }
      }
    }



    /* Process Unlocked */
    __DAL_UNLOCK(hsmbus);

    /* Note : The SMBUS interrupts must be enabled after unlocking current process
    to avoid the risk of SMBUS interrupt handle execution before current
    process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_SMBUS_ENABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Abort a master/host SMBUS process communication with Interrupt.
  * @note   This abort can be called only if state is ready
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_Master_Abort_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(DevAddress);
  if (hsmbus->Init.PeripheralMode == SMBUS_PERIPHERAL_MODE_SMBUS_HOST)
  {
    /* Process Locked */
    __DAL_LOCK(hsmbus);

    hsmbus->ErrorCode = DAL_SMBUS_ERROR_NONE;

    hsmbus->PreviousState = SMBUS_STATE_NONE;
    hsmbus->State = DAL_SMBUS_STATE_ABORT;


    /* Disable Acknowledge */
    CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Generate Stop */
    SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_STOP);

    hsmbus->XferCount = 0U;

    /* Disable EVT, BUF and ERR interrupt */
    __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

    /* Process Unlocked */
    __DAL_UNLOCK(hsmbus);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    SMBUS_ITError(hsmbus);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}


/**
  * @brief  Transmit in slave/device SMBUS mode an amount of data in non blocking mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref SMBUS_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_Slave_Transmit_IT(SMBUS_HandleTypeDef *hsmbus, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMBUS_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hsmbus->State == DAL_SMBUS_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hsmbus);

    /* Check if the SMBUS is already enabled */
    if ((hsmbus->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable SMBUS peripheral */
      __DAL_SMBUS_ENABLE(hsmbus);
    }

    /* Disable Pos */
    CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hsmbus->State     = DAL_SMBUS_STATE_BUSY_TX_LISTEN;
    hsmbus->ErrorCode = DAL_SMBUS_ERROR_NONE;
    hsmbus->Mode      = DAL_SMBUS_MODE_SLAVE;

    /* Prepare transfer parameters */
    hsmbus->pBuffPtr    = pData;
    hsmbus->XferCount   = Size;
    hsmbus->XferOptions = XferOptions;
    hsmbus->XferSize    = hsmbus->XferCount;

    /* Clear ADDR flag after prepare the transfer parameters */
    /* This action will generate an acknowledge to the HOST */
    __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);

    /* Process Unlocked */
    __DAL_UNLOCK(hsmbus);

    /* Note : The SMBUS interrupts must be enabled after unlocking current process
              to avoid the risk of SMBUS interrupt handle execution before current
              process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_SMBUS_ENABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Enable the Address listen mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref SMBUS_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_Slave_Receive_IT(SMBUS_HandleTypeDef *hsmbus, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_SMBUS_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hsmbus->State == DAL_SMBUS_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hsmbus);

    /* Check if the SMBUS is already enabled */
    if ((hsmbus->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable SMBUS peripheral */
      __DAL_SMBUS_ENABLE(hsmbus);
    }

    /* Disable Pos */
    CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hsmbus->State     = DAL_SMBUS_STATE_BUSY_RX_LISTEN;
    hsmbus->ErrorCode = DAL_SMBUS_ERROR_NONE;
    hsmbus->Mode      = DAL_SMBUS_MODE_SLAVE;



    /* Prepare transfer parameters */
    hsmbus->pBuffPtr = pData;
    hsmbus->XferCount = Size;
    hsmbus->XferOptions = XferOptions;
    hsmbus->XferSize    = hsmbus->XferCount;

    __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);

    /* Process Unlocked */
    __DAL_UNLOCK(hsmbus);

    /* Note : The SMBUS interrupts must be enabled after unlocking current process
              to avoid the risk of SMBUS interrupt handle execution before current
              process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_SMBUS_ENABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}


/**
  * @brief  Enable the Address listen mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_EnableListen_IT(SMBUS_HandleTypeDef *hsmbus)
{
  if (hsmbus->State == DAL_SMBUS_STATE_READY)
  {
    hsmbus->State = DAL_SMBUS_STATE_LISTEN;

    /* Check if the SMBUS is already enabled */
    if ((hsmbus->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable SMBUS peripheral */
      __DAL_SMBUS_ENABLE(hsmbus);
    }

    /* Enable Address Acknowledge */
    SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Enable EVT and ERR interrupt */
    __DAL_SMBUS_ENABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Disable the Address listen mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_DisableListen_IT(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of tmp to prevent undefined behavior of volatile usage */
  uint32_t tmp;

  /* Disable Address listen mode only if a transfer is not ongoing */
  if (hsmbus->State == DAL_SMBUS_STATE_LISTEN)
  {
    tmp = (uint32_t)(hsmbus->State) & SMBUS_STATE_MSK;
    hsmbus->PreviousState = tmp | (uint32_t)(hsmbus->Mode);
    hsmbus->State = DAL_SMBUS_STATE_READY;
    hsmbus->Mode = DAL_SMBUS_MODE_NONE;

    /* Disable Address Acknowledge */
    CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Disable EVT and ERR interrupt */
    __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Enable the SMBUS alert mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUSx peripheral.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_EnableAlert_IT(SMBUS_HandleTypeDef *hsmbus)
{
  /* Enable SMBus alert */
  SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ALERTEN);

  /* Clear ALERT flag */
  __DAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_SMBALERT);

  /* Enable Alert Interrupt */
  __DAL_SMBUS_ENABLE_IT(hsmbus, SMBUS_IT_ERR);

  return DAL_OK;
}
/**
  * @brief  Disable the SMBUS alert mode with Interrupt.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUSx peripheral.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_DisableAlert_IT(SMBUS_HandleTypeDef *hsmbus)
{
  /* Disable SMBus alert */
  CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ALERTEN);

  /* Disable Alert Interrupt */
  __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_ERR);

  return DAL_OK;
}


/**
  * @brief  Check if target device is ready for communication.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for the specified SMBUS.
  * @param  DevAddress Target device address The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  Trials Number of trials
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMBUS_IsDeviceReady(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
  uint32_t tickstart = 0U, tmp1 = 0U, tmp2 = 0U, tmp3 = 0U, SMBUS_Trials = 1U;

  /* Get tick */
  tickstart = DAL_GetTick();

  if (hsmbus->State == DAL_SMBUS_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if (SMBUS_WaitOnFlagUntilTimeout(hsmbus, SMBUS_FLAG_BUSY, SET, SMBUS_TIMEOUT_BUSY_FLAG, tickstart) != DAL_OK)
    {
      return DAL_BUSY;
    }

    /* Process Locked */
    __DAL_LOCK(hsmbus);

    /* Check if the SMBUS is already enabled */
    if ((hsmbus->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable SMBUS peripheral */
      __DAL_SMBUS_ENABLE(hsmbus);
    }

    /* Disable Pos */
    CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hsmbus->State = DAL_SMBUS_STATE_BUSY;
    hsmbus->ErrorCode = DAL_SMBUS_ERROR_NONE;
    hsmbus->XferOptions = SMBUS_NO_OPTION_FRAME;

    do
    {
      /* Generate Start */
      SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_START);

      /* Wait until SB flag is set */
      if (SMBUS_WaitOnFlagUntilTimeout(hsmbus, SMBUS_FLAG_SB, RESET, Timeout, tickstart) != DAL_OK)
      {
        return DAL_TIMEOUT;
      }

      /* Send slave address */
      hsmbus->Instance->DATA = SMBUS_7BIT_ADD_WRITE(DevAddress);

      /* Wait until ADDR or AF flag are set */
      /* Get tick */
      tickstart = DAL_GetTick();

      tmp1 = __DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_ADDR);
      tmp2 = __DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_AF);
      tmp3 = hsmbus->State;
      while ((tmp1 == RESET) && (tmp2 == RESET) && (tmp3 != DAL_SMBUS_STATE_TIMEOUT))
      {
        if ((Timeout == 0U) || ((DAL_GetTick() - tickstart) > Timeout))
        {
          hsmbus->State = DAL_SMBUS_STATE_TIMEOUT;
        }
        tmp1 = __DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_ADDR);
        tmp2 = __DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_AF);
        tmp3 = hsmbus->State;
      }

      hsmbus->State = DAL_SMBUS_STATE_READY;

      /* Check if the ADDR flag has been set */
      if (__DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_ADDR) == SET)
      {
        /* Generate Stop */
        SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_STOP);

        /* Clear ADDR Flag */
        __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);

        /* Wait until BUSY flag is reset */
        if (SMBUS_WaitOnFlagUntilTimeout(hsmbus, SMBUS_FLAG_BUSY, SET, SMBUS_TIMEOUT_BUSY_FLAG, tickstart) != DAL_OK)
        {
          return DAL_TIMEOUT;
        }

        hsmbus->State = DAL_SMBUS_STATE_READY;

        /* Process Unlocked */
        __DAL_UNLOCK(hsmbus);

        return DAL_OK;
      }
      else
      {
        /* Generate Stop */
        SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_STOP);

        /* Clear AF Flag */
        __DAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_AF);

        /* Wait until BUSY flag is reset */
        if (SMBUS_WaitOnFlagUntilTimeout(hsmbus, SMBUS_FLAG_BUSY, SET, SMBUS_TIMEOUT_BUSY_FLAG, tickstart) != DAL_OK)
        {
          return DAL_TIMEOUT;
        }
      }
    }
    while (SMBUS_Trials++ < Trials);

    hsmbus->State = DAL_SMBUS_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(hsmbus);

    return DAL_ERROR;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  This function handles SMBUS event interrupt request.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void DAL_SMBUS_EV_IRQHandler(SMBUS_HandleTypeDef *hsmbus)
{
  uint32_t sr2itflags   = READ_REG(hsmbus->Instance->STS2);
  uint32_t sr1itflags   = READ_REG(hsmbus->Instance->STS1);
  uint32_t itsources    = READ_REG(hsmbus->Instance->CTRL2);

  uint32_t CurrentMode  = hsmbus->Mode;

  /* Master mode selected */
  if (CurrentMode == DAL_SMBUS_MODE_MASTER)
  {
    /* SB Set ----------------------------------------------------------------*/
    if (((sr1itflags & SMBUS_FLAG_SB) != RESET) && ((itsources & SMBUS_IT_EVT) != RESET))
    {
      SMBUS_Master_SB(hsmbus);
    }
    /* ADD10 Set -------------------------------------------------------------*/
    else if (((sr1itflags & SMBUS_FLAG_ADD10) != RESET) && ((itsources & SMBUS_IT_EVT) != RESET))
    {
      SMBUS_Master_ADD10(hsmbus);
    }
    /* ADDR Set --------------------------------------------------------------*/
    else if (((sr1itflags & SMBUS_FLAG_ADDR) != RESET) && ((itsources & SMBUS_IT_EVT) != RESET))
    {
      SMBUS_Master_ADDR(hsmbus);
    }
    /* SMBUS in mode Transmitter -----------------------------------------------*/
    if ((sr2itflags & SMBUS_FLAG_TRA) != RESET)
    {
      /* TXE set and BTF reset -----------------------------------------------*/
      if (((sr1itflags & SMBUS_FLAG_TXE) != RESET) && ((itsources & SMBUS_IT_BUF) != RESET) && ((sr1itflags & SMBUS_FLAG_BTF) == RESET))
      {
        SMBUS_MasterTransmit_TXE(hsmbus);
      }
      /* BTF set -------------------------------------------------------------*/
      else if (((sr1itflags & SMBUS_FLAG_BTF) != RESET) && ((itsources & SMBUS_IT_EVT) != RESET))
      {
        SMBUS_MasterTransmit_BTF(hsmbus);
      }
    }
    /* SMBUS in mode Receiver --------------------------------------------------*/
    else
    {
      /* RXNE set and BTF reset -----------------------------------------------*/
      if (((sr1itflags & SMBUS_FLAG_RXNE) != RESET) && ((itsources & SMBUS_IT_BUF) != RESET) && ((sr1itflags & SMBUS_FLAG_BTF) == RESET))
      {
        SMBUS_MasterReceive_RXNE(hsmbus);
      }
      /* BTF set -------------------------------------------------------------*/
      else if (((sr1itflags & SMBUS_FLAG_BTF) != RESET) && ((itsources & SMBUS_IT_EVT) != RESET))
      {
        SMBUS_MasterReceive_BTF(hsmbus);
      }
    }
  }
  /* Slave mode selected */
  else
  {
    /* ADDR set --------------------------------------------------------------*/
    if (((sr1itflags & SMBUS_FLAG_ADDR) != RESET) && ((itsources & SMBUS_IT_EVT) != RESET))
    {
      SMBUS_Slave_ADDR(hsmbus);
    }
    /* STOPF set --------------------------------------------------------------*/
    else if (((sr1itflags & SMBUS_FLAG_STOPF) != RESET) && ((itsources & SMBUS_IT_EVT) != RESET))
    {
      SMBUS_Slave_STOPF(hsmbus);
    }
    /* SMBUS in mode Transmitter -----------------------------------------------*/
    else if ((sr2itflags & SMBUS_FLAG_TRA) != RESET)
    {
      /* TXE set and BTF reset -----------------------------------------------*/
      if (((sr1itflags & SMBUS_FLAG_TXE) != RESET) && ((itsources & SMBUS_IT_BUF) != RESET) && ((sr1itflags & SMBUS_FLAG_BTF) == RESET))
      {
        SMBUS_SlaveTransmit_TXE(hsmbus);
      }
      /* BTF set -------------------------------------------------------------*/
      else if (((sr1itflags & SMBUS_FLAG_BTF) != RESET) && ((itsources & SMBUS_IT_EVT) != RESET))
      {
        SMBUS_SlaveTransmit_BTF(hsmbus);
      }
    }
    /* SMBUS in mode Receiver --------------------------------------------------*/
    else
    {
      /* RXNE set and BTF reset ----------------------------------------------*/
      if (((sr1itflags & SMBUS_FLAG_RXNE) != RESET) && ((itsources & SMBUS_IT_BUF) != RESET) && ((sr1itflags & SMBUS_FLAG_BTF) == RESET))
      {
        SMBUS_SlaveReceive_RXNE(hsmbus);
      }
      /* BTF set -------------------------------------------------------------*/
      else if (((sr1itflags & SMBUS_FLAG_BTF) != RESET) && ((itsources & SMBUS_IT_EVT) != RESET))
      {
        SMBUS_SlaveReceive_BTF(hsmbus);
      }
    }
  }
}

/**
  * @brief  This function handles SMBUS error interrupt request.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
void DAL_SMBUS_ER_IRQHandler(SMBUS_HandleTypeDef *hsmbus)
{
  uint32_t tmp1 = 0U, tmp2 = 0U, tmp3 = 0U, tmp4 = 0U;
  uint32_t sr1itflags = READ_REG(hsmbus->Instance->STS1);
  uint32_t itsources  = READ_REG(hsmbus->Instance->CTRL2);

  /* SMBUS Bus error interrupt occurred ------------------------------------*/
  if (((sr1itflags & SMBUS_FLAG_BERR) != RESET) && ((itsources & SMBUS_IT_ERR) != RESET))
  {
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_BERR;

    /* Clear BERR flag */
    __DAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_BERR);

  }

  /* SMBUS Over-Run/Under-Run interrupt occurred ----------------------------------------*/
  if (((sr1itflags & SMBUS_FLAG_OVR) != RESET) && ((itsources & SMBUS_IT_ERR) != RESET))
  {
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_OVR;

    /* Clear OVR flag */
    __DAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_OVR);
  }

  /* SMBUS Arbitration Loss error interrupt occurred ------------------------------------*/
  if (((sr1itflags & SMBUS_FLAG_ARLO) != RESET) && ((itsources & SMBUS_IT_ERR) != RESET))
  {
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_ARLO;

    /* Clear ARLO flag */
    __DAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_ARLO);
  }

  /* SMBUS Acknowledge failure error interrupt occurred ------------------------------------*/
  if (((sr1itflags & SMBUS_FLAG_AF) != RESET) && ((itsources & SMBUS_IT_ERR) != RESET))
  {
    tmp1 = hsmbus->Mode;
    tmp2 = hsmbus->XferCount;
    tmp3 = hsmbus->State;
    tmp4 = hsmbus->PreviousState;

    if ((tmp1 == DAL_SMBUS_MODE_SLAVE) && (tmp2 == 0U) && \
        ((tmp3 == DAL_SMBUS_STATE_BUSY_TX) || (tmp3 == DAL_SMBUS_STATE_BUSY_TX_LISTEN) || \
         ((tmp3 == DAL_SMBUS_STATE_LISTEN) && (tmp4 == SMBUS_STATE_SLAVE_BUSY_TX))))
    {
      SMBUS_Slave_AF(hsmbus);
    }
    else
    {
      hsmbus->ErrorCode |= DAL_SMBUS_ERROR_AF;

      /* Do not generate a STOP in case of Slave receive non acknowledge during transfer (mean not at the end of transfer) */
      if (hsmbus->Mode == DAL_SMBUS_MODE_MASTER)
      {
        /* Generate Stop */
        SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_STOP);

      }

      /* Clear AF flag */
      __DAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_AF);
    }
  }

  /* SMBUS Timeout error interrupt occurred ---------------------------------------------*/
  if (((sr1itflags & SMBUS_FLAG_TIMEOUT) != RESET) && ((itsources & SMBUS_IT_ERR) != RESET))
  {
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_TIMEOUT;

    /* Clear TIMEOUT flag */
    __DAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_TIMEOUT);

  }

  /* SMBUS Alert error interrupt occurred -----------------------------------------------*/
  if (((sr1itflags & SMBUS_FLAG_SMBALERT) != RESET) && ((itsources & SMBUS_IT_ERR) != RESET))
  {
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_ALERT;

    /* Clear ALERT flag */
    __DAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_SMBALERT);
  }

  /* SMBUS Packet Error Check error interrupt occurred ----------------------------------*/
  if (((sr1itflags & SMBUS_FLAG_PECERR) != RESET) && ((itsources & SMBUS_IT_ERR) != RESET))
  {
    hsmbus->ErrorCode |= DAL_SMBUS_ERROR_PECERR;

    /* Clear PEC error flag */
    __DAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_PECERR);
  }

  /* Call the Error Callback in case of Error detected -----------------------*/
  if (hsmbus->ErrorCode != DAL_SMBUS_ERROR_NONE)
  {
    SMBUS_ITError(hsmbus);
  }
}

/**
  * @brief  Master Tx Transfer completed callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void DAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMBUS_MasterTxCpltCallback can be implemented in the user file
   */
}

/**
  * @brief  Master Rx Transfer completed callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void DAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMBUS_MasterRxCpltCallback can be implemented in the user file
   */
}

/** @brief  Slave Tx Transfer completed callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void DAL_SMBUS_SlaveTxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMBUS_SlaveTxCpltCallback can be implemented in the user file
   */
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void DAL_SMBUS_SlaveRxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMBUS_SlaveRxCpltCallback can be implemented in the user file
   */
}

/**
  * @brief  Slave Address Match callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  TransferDirection Master request Transfer Direction (Write/Read), value of @ref SMBUS_XferOptions_definition
  * @param  AddrMatchCode Address Match Code
  * @retval None
  */
__weak void DAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);
  UNUSED(TransferDirection);
  UNUSED(AddrMatchCode);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMBUS_AddrCallback can be implemented in the user file
   */
}

/**
  * @brief  Listen Complete callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void DAL_SMBUS_ListenCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
          the DAL_SMBUS_ListenCpltCallback can be implemented in the user file
  */
}

/**
  * @brief  SMBUS error callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void DAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMBUS_ErrorCallback can be implemented in the user file
   */
}

/**
  * @brief  SMBUS abort callback.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval None
  */
__weak void DAL_SMBUS_AbortCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsmbus);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMBUS_AbortCpltCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup   SMBUS_Exported_Functions_Group3 Peripheral State, Mode and Error functions
  *  @brief   Peripheral State and Errors functions
  *
@verbatim
 ===============================================================================
            ##### Peripheral State, Mode and Error functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the SMBUS handle state.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @retval DAL state
  */
DAL_SMBUS_StateTypeDef DAL_SMBUS_GetState(SMBUS_HandleTypeDef *hsmbus)
{
  /* Return SMBUS handle state */
  return hsmbus->State;
}

/**
  * @brief  Return the SMBUS Master, Slave or no mode.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *                the configuration information for SMBUS module
  * @retval DAL mode
  */
DAL_SMBUS_ModeTypeDef DAL_SMBUS_GetMode(SMBUS_HandleTypeDef *hsmbus)
{
  return hsmbus->Mode;
}

/**
  * @brief  Return the SMBUS error code
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *              the configuration information for the specified SMBUS.
  * @retval SMBUS Error Code
  */
uint32_t DAL_SMBUS_GetError(SMBUS_HandleTypeDef *hsmbus)
{
  return hsmbus->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup SMBUS_Private_Functions
  * @{
  */

/**
  * @brief  Handle TXE flag for Master
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_MasterTransmit_TXE(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentState       = hsmbus->State;
  uint32_t CurrentXferOptions = hsmbus->XferOptions;

  if ((hsmbus->XferSize == 0U) && (CurrentState == DAL_SMBUS_STATE_BUSY_TX))
  {
    /* Call TxCpltCallback() directly if no stop mode is set */
    if (((CurrentXferOptions == SMBUS_FIRST_FRAME) || (CurrentXferOptions == SMBUS_NEXT_FRAME)) && (CurrentXferOptions != SMBUS_NO_OPTION_FRAME))
    {
      __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

      hsmbus->PreviousState = SMBUS_STATE_MASTER_BUSY_TX;
      hsmbus->Mode = DAL_SMBUS_MODE_NONE;
      hsmbus->State = DAL_SMBUS_STATE_READY;

#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->MasterTxCpltCallback(hsmbus);
#else
      DAL_SMBUS_MasterTxCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
    }
    else /* Generate Stop condition then Call TxCpltCallback() */
    {
      /* Disable EVT, BUF and ERR interrupt */
      __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

      /* Generate Stop */
      SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_STOP);

      hsmbus->PreviousState = DAL_SMBUS_STATE_READY;
      hsmbus->State = DAL_SMBUS_STATE_READY;

      hsmbus->Mode = DAL_SMBUS_MODE_NONE;
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->MasterTxCpltCallback(hsmbus);
#else
      DAL_SMBUS_MasterTxCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
    }
  }
  else if (CurrentState == DAL_SMBUS_STATE_BUSY_TX)
  {

    if ((hsmbus->XferCount == 2U) && (SMBUS_GET_PEC_MODE(hsmbus) == SMBUS_PEC_ENABLE) && ((hsmbus->XferOptions == SMBUS_FIRST_AND_LAST_FRAME_WITH_PEC) || (hsmbus->XferOptions == SMBUS_LAST_FRAME_WITH_PEC)))
    {
      hsmbus->XferCount--;
    }

    if (hsmbus->XferCount == 0U)
    {

      /* Disable BUF interrupt */
      __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_BUF);

      if ((SMBUS_GET_PEC_MODE(hsmbus) == SMBUS_PEC_ENABLE) && ((hsmbus->XferOptions == SMBUS_FIRST_AND_LAST_FRAME_WITH_PEC) || (hsmbus->XferOptions == SMBUS_LAST_FRAME_WITH_PEC)))
      {
        SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_PEC);
      }

    }
    else
    {
      /* Write data to DATA */
      hsmbus->Instance->DATA = (*hsmbus->pBuffPtr++);
      hsmbus->XferCount--;
    }
  }
  return DAL_OK;
}

/**
  * @brief  Handle BTF flag for Master transmitter
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_MasterTransmit_BTF(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentXferOptions = hsmbus->XferOptions;

  if (hsmbus->State == DAL_SMBUS_STATE_BUSY_TX)
  {
    if (hsmbus->XferCount != 0U)
    {
      /* Write data to DATA */
      hsmbus->Instance->DATA = (*hsmbus->pBuffPtr++);
      hsmbus->XferCount--;
    }
    else
    {
      /* Call TxCpltCallback() directly if no stop mode is set */
      if (((CurrentXferOptions == SMBUS_FIRST_FRAME) || (CurrentXferOptions == SMBUS_NEXT_FRAME)) && (CurrentXferOptions != SMBUS_NO_OPTION_FRAME))
      {
        __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

        hsmbus->PreviousState = SMBUS_STATE_MASTER_BUSY_TX;
        hsmbus->Mode = DAL_SMBUS_MODE_NONE;
        hsmbus->State = DAL_SMBUS_STATE_READY;

#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
        hsmbus->MasterTxCpltCallback(hsmbus);
#else
        DAL_SMBUS_MasterTxCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
      }
      else /* Generate Stop condition then Call TxCpltCallback() */
      {
        /* Disable EVT, BUF and ERR interrupt */
        __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

        /* Generate Stop */
        SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_STOP);

        hsmbus->PreviousState = DAL_SMBUS_STATE_READY;
        hsmbus->State = DAL_SMBUS_STATE_READY;
        hsmbus->Mode = DAL_SMBUS_MODE_NONE;
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
        hsmbus->MasterTxCpltCallback(hsmbus);
#else
        DAL_SMBUS_MasterTxCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
      }
    }
  }
  return DAL_OK;
}

/**
  * @brief  Handle RXNE flag for Master
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_MasterReceive_RXNE(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentXferOptions = hsmbus->XferOptions;

  if (hsmbus->State == DAL_SMBUS_STATE_BUSY_RX)
  {
    uint32_t tmp = hsmbus->XferCount;

    if (tmp > 3U)
    {
      /* Read data from DATA */
      (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
      hsmbus->XferCount--;

      if (hsmbus->XferCount == 3)
      {
        /* Disable BUF interrupt, this help to treat correctly the last 4 bytes
        on BTF subroutine */
        /* Disable BUF interrupt */
        __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_BUF);
      }
    }

    else if (tmp == 2U)
    {

      /* Read data from DATA */
      (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
      hsmbus->XferCount--;

      if ((CurrentXferOptions == SMBUS_FIRST_AND_LAST_FRAME_WITH_PEC) || (CurrentXferOptions == SMBUS_LAST_FRAME_WITH_PEC))
      {
        /* PEC of slave */
        hsmbus->XferPEC = SMBUS_GET_PEC(hsmbus);

      }
      /* Generate Stop */
      SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_STOP);
    }

    else if ((tmp == 1U) || (tmp == 0U))
    {
      /* Disable Acknowledge */
      CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Disable EVT, BUF and ERR interrupt */
      __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

      /* Read data from DATA */
      (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
      hsmbus->XferCount--;

      hsmbus->State = DAL_SMBUS_STATE_READY;
      hsmbus->PreviousState = SMBUS_STATE_NONE;
      hsmbus->Mode = DAL_SMBUS_MODE_NONE;

#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->MasterRxCpltCallback(hsmbus);
#else
      DAL_SMBUS_MasterRxCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
    }
  }

  return DAL_OK;
}

/**
  * @brief  Handle BTF flag for Master receiver
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_MasterReceive_BTF(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentXferOptions = hsmbus->XferOptions;

  if (hsmbus->XferCount == 4U)
  {
    /* Disable BUF interrupt, this help to treat correctly the last 2 bytes
       on BTF subroutine if there is a reception delay between N-1 and N byte */
    __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_BUF);

    /* Read data from DATA */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
    hsmbus->XferCount--;
    hsmbus->XferPEC = SMBUS_GET_PEC(hsmbus);
  }
  else if (hsmbus->XferCount == 3U)
  {
    /* Disable BUF interrupt, this help to treat correctly the last 2 bytes
       on BTF subroutine if there is a reception delay between N-1 and N byte */
    __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_BUF);

    /* Disable Acknowledge */
    CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Read data from DATA */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
    hsmbus->XferCount--;
    hsmbus->XferPEC = SMBUS_GET_PEC(hsmbus);
  }
  else if (hsmbus->XferCount == 2U)
  {
    /* Prepare next transfer or stop current transfer */
    if ((CurrentXferOptions == SMBUS_NEXT_FRAME) || (CurrentXferOptions == SMBUS_FIRST_FRAME))
    {
      /* Disable Acknowledge */
      CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Generate ReStart */
      SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_START);
    }
    else
    {
      /* Generate Stop */
      SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_STOP);
    }

    /* Read data from DATA */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
    hsmbus->XferCount--;

    /* Read data from DATA */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
    hsmbus->XferCount--;

    /* Disable EVT and ERR interrupt */
    __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_ERR);

    hsmbus->State = DAL_SMBUS_STATE_READY;
    hsmbus->PreviousState = SMBUS_STATE_NONE;
    hsmbus->Mode = DAL_SMBUS_MODE_NONE;
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
    hsmbus->MasterRxCpltCallback(hsmbus);
#else
    DAL_SMBUS_MasterRxCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
  }
  else
  {
    /* Read data from DATA */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
    hsmbus->XferCount--;
  }
  return DAL_OK;
}

/**
  * @brief  Handle SB flag for Master
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_Master_SB(SMBUS_HandleTypeDef *hsmbus)
{
  if (hsmbus->Init.AddressingMode == SMBUS_ADDRESSINGMODE_7BIT)
  {
    /* Send slave 7 Bits address */
    if (hsmbus->State == DAL_SMBUS_STATE_BUSY_TX)
    {
      hsmbus->Instance->DATA = SMBUS_7BIT_ADD_WRITE(hsmbus->Devaddress);
    }
    else
    {
      hsmbus->Instance->DATA = SMBUS_7BIT_ADD_READ(hsmbus->Devaddress);
    }
  }
  else
  {
    if (hsmbus->EventCount == 0U)
    {
      /* Send header of slave address */
      hsmbus->Instance->DATA = SMBUS_10BIT_HEADER_WRITE(hsmbus->Devaddress);
    }
    else if (hsmbus->EventCount == 1U)
    {
      /* Send header of slave address */
      hsmbus->Instance->DATA = SMBUS_10BIT_HEADER_READ(hsmbus->Devaddress);
    }
  }
  return DAL_OK;
}

/**
  * @brief  Handle ADD10 flag for Master
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_Master_ADD10(SMBUS_HandleTypeDef *hsmbus)
{
  /* Send slave address */
  hsmbus->Instance->DATA = SMBUS_10BIT_ADDRESS(hsmbus->Devaddress);

  return DAL_OK;
}

/**
  * @brief  Handle ADDR flag for Master
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_Master_ADDR(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  uint32_t Prev_State         = hsmbus->PreviousState;

  if (hsmbus->State == DAL_SMBUS_STATE_BUSY_RX)
  {
    if ((hsmbus->EventCount == 0U) && (hsmbus->Init.AddressingMode == SMBUS_ADDRESSINGMODE_10BIT))
    {
      /* Clear ADDR flag */
      __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);

      /* Generate Restart */
      SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_START);

      hsmbus->EventCount++;
    }
    else
    {
      /*  In the case of the Quick Command, the ADDR flag is cleared and a stop is generated */
      if (hsmbus->XferCount == 0U)
      {
        /* Clear ADDR flag */
        __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);

        /* Generate Stop */
        SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_STOP);
      }
      else if (hsmbus->XferCount == 1U)
      {
        /* Prepare next transfer or stop current transfer */
        if ((hsmbus->XferOptions == SMBUS_FIRST_FRAME) && (Prev_State != SMBUS_STATE_MASTER_BUSY_RX))
        {
          /* Disable Acknowledge */
          CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

          /* Clear ADDR flag */
          __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);
        }
        else if ((hsmbus->XferOptions == SMBUS_NEXT_FRAME) && (Prev_State != SMBUS_STATE_MASTER_BUSY_RX))
        {
          /* Enable Acknowledge */
          SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

          /* Clear ADDR flag */
          __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);
        }
        else
        {
          /* Disable Acknowledge */
          CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

          /* Clear ADDR flag */
          __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);

          /* Generate Stop */
          SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_STOP);
        }
      }
      else if (hsmbus->XferCount == 2U)
      {
        if (hsmbus->XferOptions != SMBUS_NEXT_FRAME)
        {
          /* Disable Acknowledge */
          CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

          /* Enable Pos */
          SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKPOS);


        }
        else
        {
          /* Enable Acknowledge */
          SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);
        }

        /* Clear ADDR flag */
        __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);
      }
      else
      {
        /* Enable Acknowledge */
        SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

        /* Clear ADDR flag */
        __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);
      }

      /* Reset Event counter  */
      hsmbus->EventCount = 0U;
    }
  }
  else
  {
    /* Clear ADDR flag */
    __DAL_SMBUS_CLEAR_ADDRFLAG(hsmbus);
  }

  return DAL_OK;
}

/**
  * @brief  Handle TXE flag for Slave
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_SlaveTransmit_TXE(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentState = hsmbus->State;

  if (hsmbus->XferCount != 0U)
  {
    /* Write data to DATA */
    hsmbus->Instance->DATA = (*hsmbus->pBuffPtr++);
    hsmbus->XferCount--;

    if ((hsmbus->XferCount == 2U) && (SMBUS_GET_PEC_MODE(hsmbus) == SMBUS_PEC_ENABLE) && ((hsmbus->XferOptions == SMBUS_FIRST_AND_LAST_FRAME_WITH_PEC) || (hsmbus->XferOptions == SMBUS_LAST_FRAME_WITH_PEC)))
    {
      hsmbus->XferCount--;
    }

    if ((hsmbus->XferCount == 0U) && (CurrentState == (DAL_SMBUS_STATE_BUSY_TX_LISTEN)))
    {
      /* Last Byte is received, disable Interrupt */
      __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_BUF);

      /* Set state at DAL_SMBUS_STATE_LISTEN */
      hsmbus->PreviousState = SMBUS_STATE_SLAVE_BUSY_TX;
      hsmbus->State = DAL_SMBUS_STATE_LISTEN;

      /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->SlaveTxCpltCallback(hsmbus);
#else
      DAL_SMBUS_SlaveTxCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
    }
  }
  return DAL_OK;
}

/**
  * @brief  Handle BTF flag for Slave transmitter
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_SlaveTransmit_BTF(SMBUS_HandleTypeDef *hsmbus)
{
  if (hsmbus->XferCount != 0U)
  {
    /* Write data to DATA */
    hsmbus->Instance->DATA = (*hsmbus->pBuffPtr++);
    hsmbus->XferCount--;
  }



  else if ((hsmbus->XferCount == 0U) && (SMBUS_GET_PEC_MODE(hsmbus) == SMBUS_PEC_ENABLE) && ((hsmbus->XferOptions == SMBUS_FIRST_AND_LAST_FRAME_WITH_PEC) || (hsmbus->XferOptions == SMBUS_LAST_FRAME_WITH_PEC)))
  {
    SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_PEC);
  }
  return DAL_OK;
}

/**
  * @brief  Handle RXNE flag for Slave
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_SlaveReceive_RXNE(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentState = hsmbus->State;

  if (hsmbus->XferCount != 0U)
  {
    /* Read data from DATA */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
    hsmbus->XferCount--;

    if ((hsmbus->XferCount == 1U) && (SMBUS_GET_PEC_MODE(hsmbus) == SMBUS_PEC_ENABLE) && ((hsmbus->XferOptions == SMBUS_FIRST_AND_LAST_FRAME_WITH_PEC) || (hsmbus->XferOptions == SMBUS_LAST_FRAME_WITH_PEC)))
    {
      SET_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_PEC);
      hsmbus->XferPEC = SMBUS_GET_PEC(hsmbus);
    }
    if ((hsmbus->XferCount == 0U) && (CurrentState == DAL_SMBUS_STATE_BUSY_RX_LISTEN))
    {
      /* Last Byte is received, disable Interrupt */
      __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_BUF);

      /* Set state at DAL_SMBUS_STATE_LISTEN */
      hsmbus->PreviousState = SMBUS_STATE_SLAVE_BUSY_RX;
      hsmbus->State = DAL_SMBUS_STATE_LISTEN;

      /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->SlaveRxCpltCallback(hsmbus);
#else
      DAL_SMBUS_SlaveRxCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
    }
  }
  return DAL_OK;
}

/**
  * @brief  Handle BTF flag for Slave receiver
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_SlaveReceive_BTF(SMBUS_HandleTypeDef *hsmbus)
{
  if (hsmbus->XferCount != 0U)
  {
    /* Read data from DATA */
    (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
    hsmbus->XferCount--;
  }

  return DAL_OK;
}

/**
  * @brief  Handle ADD flag for Slave
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_Slave_ADDR(SMBUS_HandleTypeDef *hsmbus)
{
  uint8_t TransferDirection = SMBUS_DIRECTION_RECEIVE ;
  uint16_t SlaveAddrCode = 0U;

  /* Transfer Direction requested by Master */
  if (__DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_TRA) == RESET)
  {
    TransferDirection = SMBUS_DIRECTION_TRANSMIT;
  }

  if (__DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_DUALF) == RESET)
  {
    SlaveAddrCode = hsmbus->Init.OwnAddress1;
  }
  else
  {
    SlaveAddrCode = hsmbus->Init.OwnAddress2;
  }

  /* Call Slave Addr callback */
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
  hsmbus->AddrCallback(hsmbus, TransferDirection, SlaveAddrCode);
#else
  DAL_SMBUS_AddrCallback(hsmbus, TransferDirection, SlaveAddrCode);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */

  return DAL_OK;
}

/**
  * @brief  Handle STOPF flag for Slave
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_Slave_STOPF(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  uint32_t CurrentState = hsmbus->State;

  /* Disable EVT, BUF and ERR interrupt */
  __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

  /* Clear STOPF flag */
  __DAL_SMBUS_CLEAR_STOPFLAG(hsmbus);

  /* Disable Acknowledge */
  CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

  /* All data are not transferred, so set error code accordingly */
  if (hsmbus->XferCount != 0U)
  {
    /* Store Last receive data if any */
    if (__DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_BTF) == SET)
    {
      /* Read data from DATA */
      (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;

      if (hsmbus->XferCount > 0)
      {
        hsmbus->XferSize--;
        hsmbus->XferCount--;
      }
    }

    /* Store Last receive data if any */
    if (__DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_RXNE) == SET)
    {
      /* Read data from DATA */
      (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;

      if (hsmbus->XferCount > 0)
      {
        hsmbus->XferSize--;
        hsmbus->XferCount--;
      }
    }
  }

  if (hsmbus->ErrorCode != DAL_SMBUS_ERROR_NONE)
  {
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    SMBUS_ITError(hsmbus);
  }
  else
  {
    if ((CurrentState == DAL_SMBUS_STATE_LISTEN) || (CurrentState == DAL_SMBUS_STATE_BUSY_RX_LISTEN)  || \
        (CurrentState == DAL_SMBUS_STATE_BUSY_TX_LISTEN))
    {
      hsmbus->XferOptions = SMBUS_NO_OPTION_FRAME;
      hsmbus->PreviousState = DAL_SMBUS_STATE_READY;
      hsmbus->State = DAL_SMBUS_STATE_READY;
      hsmbus->Mode = DAL_SMBUS_MODE_NONE;

#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
      hsmbus->ListenCpltCallback(hsmbus);
#else
      DAL_SMBUS_ListenCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
    }
  }
  return DAL_OK;
}

/**
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_Slave_AF(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentState       = hsmbus->State;
  uint32_t CurrentXferOptions = hsmbus->XferOptions;

  if (((CurrentXferOptions ==  SMBUS_FIRST_AND_LAST_FRAME_NO_PEC) || (CurrentXferOptions ==  SMBUS_FIRST_AND_LAST_FRAME_WITH_PEC) || \
       (CurrentXferOptions == SMBUS_LAST_FRAME_NO_PEC) || (CurrentXferOptions ==  SMBUS_LAST_FRAME_WITH_PEC)) && \
      (CurrentState == DAL_SMBUS_STATE_LISTEN))
  {
    hsmbus->XferOptions = SMBUS_NO_OPTION_FRAME;

    /* Disable EVT, BUF and ERR interrupt */
    __DAL_SMBUS_DISABLE_IT(hsmbus, SMBUS_IT_EVT | SMBUS_IT_BUF | SMBUS_IT_ERR);

    /* Clear AF flag */
    __DAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_AF);

    /* Disable Acknowledge */
    CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKEN);

    hsmbus->PreviousState = DAL_SMBUS_STATE_READY;
    hsmbus->State = DAL_SMBUS_STATE_READY;
    hsmbus->Mode = DAL_SMBUS_MODE_NONE;

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
    hsmbus->ListenCpltCallback(hsmbus);
#else
    DAL_SMBUS_ListenCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
  }
  return DAL_OK;
}



/**
  * @brief SMBUS interrupts error process
  * @param  hsmbus SMBUS handle.
  * @retval None
  */
static void SMBUS_ITError(SMBUS_HandleTypeDef *hsmbus)
{
  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  uint32_t CurrentState = hsmbus->State;

  if ((CurrentState == DAL_SMBUS_STATE_BUSY_TX_LISTEN) || (CurrentState == DAL_SMBUS_STATE_BUSY_RX_LISTEN))
  {
    /* keep DAL_SMBUS_STATE_LISTEN */
    hsmbus->PreviousState = SMBUS_STATE_NONE;
    hsmbus->State = DAL_SMBUS_STATE_LISTEN;
  }
  else
  {
    /* If state is an abort treatment on going, don't change state */
    /* This change will be done later */
    if (hsmbus->State != DAL_SMBUS_STATE_ABORT)
    {
      hsmbus->State = DAL_SMBUS_STATE_READY;
    }
    hsmbus->PreviousState = SMBUS_STATE_NONE;
    hsmbus->Mode = DAL_SMBUS_MODE_NONE;
  }

  /* Disable Pos bit in SMBUS CR1 when error occurred in Master/Mem Receive IT Process */
  CLEAR_BIT(hsmbus->Instance->CTRL1, I2C_CTRL1_ACKPOS);

  if (hsmbus->State == DAL_SMBUS_STATE_ABORT)
  {
    hsmbus->State = DAL_SMBUS_STATE_READY;
    hsmbus->ErrorCode = DAL_SMBUS_ERROR_NONE;

    /* Store Last receive data if any */
    if (__DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_RXNE) == SET)
    {
      /* Read data from DATA */
      (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
    }

    /* Disable SMBUS peripheral to prevent dummy data in buffer */
    __DAL_SMBUS_DISABLE(hsmbus);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
    hsmbus->AbortCpltCallback(hsmbus);
#else
    DAL_SMBUS_AbortCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
  }
  else
  {
    /* Store Last receive data if any */
    if (__DAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_RXNE) == SET)
    {
      /* Read data from DATA */
      (*hsmbus->pBuffPtr++) = hsmbus->Instance->DATA;
    }

    /* Call user error callback */
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
    hsmbus->ErrorCallback(hsmbus);
#else
    DAL_SMBUS_ErrorCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
  }
  /* STOP Flag is not set after a NACK reception */
  /* So may inform upper layer that listen phase is stopped */
  /* during NACK error treatment */
  if ((hsmbus->State == DAL_SMBUS_STATE_LISTEN) && ((hsmbus->ErrorCode & DAL_SMBUS_ERROR_AF) == DAL_SMBUS_ERROR_AF))
  {
    hsmbus->XferOptions = SMBUS_NO_OPTION_FRAME;
    hsmbus->PreviousState = SMBUS_STATE_NONE;
    hsmbus->State = DAL_SMBUS_STATE_READY;
    hsmbus->Mode = DAL_SMBUS_MODE_NONE;

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
#if (USE_DAL_SMBUS_REGISTER_CALLBACKS == 1)
    hsmbus->ListenCpltCallback(hsmbus);
#else
    DAL_SMBUS_ListenCpltCallback(hsmbus);
#endif /* USE_DAL_SMBUS_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  This function handles SMBUS Communication Timeout.
  * @param  hsmbus Pointer to a SMBUS_HandleTypeDef structure that contains
  *         the configuration information for SMBUS module
  * @param  Flag specifies the SMBUS flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef SMBUS_WaitOnFlagUntilTimeout(SMBUS_HandleTypeDef *hsmbus, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
  /* Wait until flag is set */
  if (Status == RESET)
  {
    while (__DAL_SMBUS_GET_FLAG(hsmbus, Flag) == RESET)
    {
      /* Check for the Timeout */
      if (Timeout != DAL_MAX_DELAY)
      {
        if ((Timeout == 0U) || ((DAL_GetTick() - Tickstart) > Timeout))
        {
          hsmbus->PreviousState = SMBUS_STATE_NONE;
          hsmbus->State = DAL_SMBUS_STATE_READY;
          hsmbus->Mode = DAL_SMBUS_MODE_NONE;

          /* Process Unlocked */
          __DAL_UNLOCK(hsmbus);
          return DAL_TIMEOUT;
        }
      }
    }
  }
  return DAL_OK;
}

/**
  * @}
  */


#endif /* DAL_SMBUS_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

