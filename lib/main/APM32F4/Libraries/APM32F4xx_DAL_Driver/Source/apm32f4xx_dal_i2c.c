/**
  *
  * @file    apm32f4xx_dal_i2c.c
  * @brief   I2C DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Inter Integrated Circuit (I2C) peripheral:
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
    The I2C DAL driver can be used as follows:

    (#) Declare a I2C_HandleTypeDef handle structure, for example:
        I2C_HandleTypeDef  hi2c;

    (#)Initialize the I2C low level resources by implementing the DAL_I2C_MspInit() API:
        (##) Enable the I2Cx interface clock
        (##) I2C pins configuration
            (+++) Enable the clock for the I2C GPIOs
            (+++) Configure I2C pins as alternate function open-drain
        (##) NVIC configuration if you need to use interrupt process
            (+++) Configure the I2Cx interrupt priority
            (+++) Enable the NVIC I2C IRQ Channel
        (##) DMA Configuration if you need to use DMA process
            (+++) Declare a DMA_HandleTypeDef handle structure for the transmit or receive stream
            (+++) Enable the DMAx interface clock using
            (+++) Configure the DMA handle parameters
            (+++) Configure the DMA Tx or Rx stream
            (+++) Associate the initialized DMA handle to the hi2c DMA Tx or Rx handle
            (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on
                  the DMA Tx or Rx stream

    (#) Configure the Communication Speed, Duty cycle, Addressing mode, Own Address1,
        Dual Addressing mode, Own Address2, General call and Nostretch mode in the hi2c Init structure.

    (#) Initialize the I2C registers by calling the DAL_I2C_Init(), configures also the low level Hardware
        (GPIO, CLOCK, NVIC...etc) by calling the customized DAL_I2C_MspInit() API.

    (#) To check if target device is ready for communication, use the function DAL_I2C_IsDeviceReady()

    (#) For I2C IO and IO MEM operations, three operation modes are available within this driver :

    *** Polling mode IO operation ***
    =================================
    [..]
      (+) Transmit in master mode an amount of data in blocking mode using DAL_I2C_Master_Transmit()
      (+) Receive in master mode an amount of data in blocking mode using DAL_I2C_Master_Receive()
      (+) Transmit in slave mode an amount of data in blocking mode using DAL_I2C_Slave_Transmit()
      (+) Receive in slave mode an amount of data in blocking mode using DAL_I2C_Slave_Receive()

    *** Polling mode IO MEM operation ***
    =====================================
    [..]
      (+) Write an amount of data in blocking mode to a specific memory address using DAL_I2C_Mem_Write()
      (+) Read an amount of data in blocking mode from a specific memory address using DAL_I2C_Mem_Read()


    *** Interrupt mode IO operation ***
    ===================================
    [..]
      (+) Transmit in master mode an amount of data in non-blocking mode using DAL_I2C_Master_Transmit_IT()
      (+) At transmission end of transfer, DAL_I2C_MasterTxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_MasterTxCpltCallback()
      (+) Receive in master mode an amount of data in non-blocking mode using DAL_I2C_Master_Receive_IT()
      (+) At reception end of transfer, DAL_I2C_MasterRxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_MasterRxCpltCallback()
      (+) Transmit in slave mode an amount of data in non-blocking mode using DAL_I2C_Slave_Transmit_IT()
      (+) At transmission end of transfer, DAL_I2C_SlaveTxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_SlaveTxCpltCallback()
      (+) Receive in slave mode an amount of data in non-blocking mode using DAL_I2C_Slave_Receive_IT()
      (+) At reception end of transfer, DAL_I2C_SlaveRxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_SlaveRxCpltCallback()
      (+) In case of transfer Error, DAL_I2C_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer DAL_I2C_ErrorCallback()
      (+) Abort a master I2C process communication with Interrupt using DAL_I2C_Master_Abort_IT()
      (+) End of abort process, DAL_I2C_AbortCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_AbortCpltCallback()

    *** Interrupt mode or DMA mode IO sequential operation ***
    ==========================================================
    [..]
      (@) These interfaces allow to manage a sequential transfer with a repeated start condition
          when a direction change during transfer
    [..]
      (+) A specific option field manage the different steps of a sequential transfer
      (+) Option field values are defined through I2C_XferOptions_definition and are listed below:
      (++) I2C_FIRST_AND_LAST_FRAME: No sequential usage, functional is same as associated interfaces in no sequential mode
      (++) I2C_FIRST_FRAME: Sequential usage, this option allow to manage a sequence with start condition, address
                            and data to transfer without a final stop condition
      (++) I2C_FIRST_AND_NEXT_FRAME: Sequential usage (Master only), this option allow to manage a sequence with start condition, address
                            and data to transfer without a final stop condition, an then permit a call the same master sequential interface
                            several times (like DAL_I2C_Master_Seq_Transmit_IT() then DAL_I2C_Master_Seq_Transmit_IT()
                            or DAL_I2C_Master_Seq_Transmit_DMA() then DAL_I2C_Master_Seq_Transmit_DMA())
      (++) I2C_NEXT_FRAME: Sequential usage, this option allow to manage a sequence with a restart condition, address
                            and with new data to transfer if the direction change or manage only the new data to transfer
                            if no direction change and without a final stop condition in both cases
      (++) I2C_LAST_FRAME: Sequential usage, this option allow to manage a sequance with a restart condition, address
                            and with new data to transfer if the direction change or manage only the new data to transfer
                            if no direction change and with a final stop condition in both cases
      (++) I2C_LAST_FRAME_NO_STOP: Sequential usage (Master only), this option allow to manage a restart condition after several call of the same master sequential
                            interface several times (link with option I2C_FIRST_AND_NEXT_FRAME).
                            Usage can, transfer several bytes one by one using DAL_I2C_Master_Seq_Transmit_IT(option I2C_FIRST_AND_NEXT_FRAME then I2C_NEXT_FRAME)
                              or DAL_I2C_Master_Seq_Receive_IT(option I2C_FIRST_AND_NEXT_FRAME then I2C_NEXT_FRAME)
                              or DAL_I2C_Master_Seq_Transmit_DMA(option I2C_FIRST_AND_NEXT_FRAME then I2C_NEXT_FRAME)
                              or DAL_I2C_Master_Seq_Receive_DMA(option I2C_FIRST_AND_NEXT_FRAME then I2C_NEXT_FRAME).
                            Then usage of this option I2C_LAST_FRAME_NO_STOP at the last Transmit or Receive sequence permit to call the opposite interface Receive or Transmit
                              without stopping the communication and so generate a restart condition.
      (++) I2C_OTHER_FRAME: Sequential usage (Master only), this option allow to manage a restart condition after each call of the same master sequential
                            interface.
                            Usage can, transfer several bytes one by one with a restart with slave address between each bytes using DAL_I2C_Master_Seq_Transmit_IT(option I2C_FIRST_FRAME then I2C_OTHER_FRAME)
                              or DAL_I2C_Master_Seq_Receive_IT(option I2C_FIRST_FRAME then I2C_OTHER_FRAME)
                              or DAL_I2C_Master_Seq_Transmit_DMA(option I2C_FIRST_FRAME then I2C_OTHER_FRAME)
                              or DAL_I2C_Master_Seq_Receive_DMA(option I2C_FIRST_FRAME then I2C_OTHER_FRAME).
                            Then usage of this option I2C_OTHER_AND_LAST_FRAME at the last frame to help automatic generation of STOP condition.

      (+) Different sequential I2C interfaces are listed below:
      (++) Sequential transmit in master I2C mode an amount of data in non-blocking mode using DAL_I2C_Master_Seq_Transmit_IT()
            or using DAL_I2C_Master_Seq_Transmit_DMA()
      (+++) At transmission end of current frame transfer, DAL_I2C_MasterTxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_MasterTxCpltCallback()
      (++) Sequential receive in master I2C mode an amount of data in non-blocking mode using DAL_I2C_Master_Seq_Receive_IT()
            or using DAL_I2C_Master_Seq_Receive_DMA()
      (+++) At reception end of current frame transfer, DAL_I2C_MasterRxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_MasterRxCpltCallback()
      (++) Abort a master IT or DMA I2C process communication with Interrupt using DAL_I2C_Master_Abort_IT()
      (+++) End of abort process, DAL_I2C_AbortCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_AbortCpltCallback()
      (++) Enable/disable the Address listen mode in slave I2C mode using DAL_I2C_EnableListen_IT() DAL_I2C_DisableListen_IT()
      (+++) When address slave I2C match, DAL_I2C_AddrCallback() is executed and user can
           add his own code to check the Address Match Code and the transmission direction request by master (Write/Read).
      (+++) At Listen mode end DAL_I2C_ListenCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_ListenCpltCallback()
      (++) Sequential transmit in slave I2C mode an amount of data in non-blocking mode using DAL_I2C_Slave_Seq_Transmit_IT()
            or using DAL_I2C_Slave_Seq_Transmit_DMA()
      (+++) At transmission end of current frame transfer, DAL_I2C_SlaveTxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_SlaveTxCpltCallback()
      (++) Sequential receive in slave I2C mode an amount of data in non-blocking mode using DAL_I2C_Slave_Seq_Receive_IT()
            or using DAL_I2C_Slave_Seq_Receive_DMA()
      (+++) At reception end of current frame transfer, DAL_I2C_SlaveRxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_SlaveRxCpltCallback()
      (++) In case of transfer Error, DAL_I2C_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer DAL_I2C_ErrorCallback()

    *** Interrupt mode IO MEM operation ***
    =======================================
    [..]
      (+) Write an amount of data in non-blocking mode with Interrupt to a specific memory address using
          DAL_I2C_Mem_Write_IT()
      (+) At Memory end of write transfer, DAL_I2C_MemTxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_MemTxCpltCallback()
      (+) Read an amount of data in non-blocking mode with Interrupt from a specific memory address using
          DAL_I2C_Mem_Read_IT()
      (+) At Memory end of read transfer, DAL_I2C_MemRxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_MemRxCpltCallback()
      (+) In case of transfer Error, DAL_I2C_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer DAL_I2C_ErrorCallback()

    *** DMA mode IO operation ***
    ==============================
    [..]
      (+) Transmit in master mode an amount of data in non-blocking mode (DMA) using
          DAL_I2C_Master_Transmit_DMA()
      (+) At transmission end of transfer, DAL_I2C_MasterTxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_MasterTxCpltCallback()
      (+) Receive in master mode an amount of data in non-blocking mode (DMA) using
          DAL_I2C_Master_Receive_DMA()
      (+) At reception end of transfer, DAL_I2C_MasterRxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_MasterRxCpltCallback()
      (+) Transmit in slave mode an amount of data in non-blocking mode (DMA) using
          DAL_I2C_Slave_Transmit_DMA()
      (+) At transmission end of transfer, DAL_I2C_SlaveTxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_SlaveTxCpltCallback()
      (+) Receive in slave mode an amount of data in non-blocking mode (DMA) using
          DAL_I2C_Slave_Receive_DMA()
      (+) At reception end of transfer, DAL_I2C_SlaveRxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_SlaveRxCpltCallback()
      (+) In case of transfer Error, DAL_I2C_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer DAL_I2C_ErrorCallback()
      (+) Abort a master I2C process communication with Interrupt using DAL_I2C_Master_Abort_IT()
      (+) End of abort process, DAL_I2C_AbortCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_AbortCpltCallback()

    *** DMA mode IO MEM operation ***
    =================================
    [..]
      (+) Write an amount of data in non-blocking mode with DMA to a specific memory address using
          DAL_I2C_Mem_Write_DMA()
      (+) At Memory end of write transfer, DAL_I2C_MemTxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_MemTxCpltCallback()
      (+) Read an amount of data in non-blocking mode with DMA from a specific memory address using
          DAL_I2C_Mem_Read_DMA()
      (+) At Memory end of read transfer, DAL_I2C_MemRxCpltCallback() is executed and user can
           add his own code by customization of function pointer DAL_I2C_MemRxCpltCallback()
      (+) In case of transfer Error, DAL_I2C_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer DAL_I2C_ErrorCallback()


     *** I2C DAL driver macros list ***
     ==================================
     [..]
       Below the list of most used macros in I2C DAL driver.

      (+) __DAL_I2C_ENABLE:     Enable the I2C peripheral
      (+) __DAL_I2C_DISABLE:    Disable the I2C peripheral
      (+) __DAL_I2C_GET_FLAG:   Checks whether the specified I2C flag is set or not
      (+) __DAL_I2C_CLEAR_FLAG: Clear the specified I2C pending flag
      (+) __DAL_I2C_ENABLE_IT:  Enable the specified I2C interrupt
      (+) __DAL_I2C_DISABLE_IT: Disable the specified I2C interrupt

     *** Callback registration ***
     =============================================
    [..]
     The compilation flag USE_DAL_I2C_REGISTER_CALLBACKS when set to 1
     allows the user to configure dynamically the driver callbacks.
     Use Functions DAL_I2C_RegisterCallback() or DAL_I2C_RegisterAddrCallback()
     to register an interrupt callback.
    [..]
     Function DAL_I2C_RegisterCallback() allows to register following callbacks:
       (+) MasterTxCpltCallback : callback for Master transmission end of transfer.
       (+) MasterRxCpltCallback : callback for Master reception end of transfer.
       (+) SlaveTxCpltCallback  : callback for Slave transmission end of transfer.
       (+) SlaveRxCpltCallback  : callback for Slave reception end of transfer.
       (+) ListenCpltCallback   : callback for end of listen mode.
       (+) MemTxCpltCallback    : callback for Memory transmission end of transfer.
       (+) MemRxCpltCallback    : callback for Memory reception end of transfer.
       (+) ErrorCallback        : callback for error detection.
       (+) AbortCpltCallback    : callback for abort completion process.
       (+) MspInitCallback      : callback for Msp Init.
       (+) MspDeInitCallback    : callback for Msp DeInit.
     This function takes as parameters the DAL peripheral handle, the Callback ID
     and a pointer to the user callback function.
    [..]
     For specific callback AddrCallback use dedicated register callbacks : DAL_I2C_RegisterAddrCallback().
    [..]
     Use function DAL_I2C_UnRegisterCallback to reset a callback to the default
     weak function.
     DAL_I2C_UnRegisterCallback takes as parameters the DAL peripheral handle,
     and the Callback ID.
     This function allows to reset following callbacks:
       (+) MasterTxCpltCallback : callback for Master transmission end of transfer.
       (+) MasterRxCpltCallback : callback for Master reception end of transfer.
       (+) SlaveTxCpltCallback  : callback for Slave transmission end of transfer.
       (+) SlaveRxCpltCallback  : callback for Slave reception end of transfer.
       (+) ListenCpltCallback   : callback for end of listen mode.
       (+) MemTxCpltCallback    : callback for Memory transmission end of transfer.
       (+) MemRxCpltCallback    : callback for Memory reception end of transfer.
       (+) ErrorCallback        : callback for error detection.
       (+) AbortCpltCallback    : callback for abort completion process.
       (+) MspInitCallback      : callback for Msp Init.
       (+) MspDeInitCallback    : callback for Msp DeInit.
    [..]
     For callback AddrCallback use dedicated register callbacks : DAL_I2C_UnRegisterAddrCallback().
    [..]
     By default, after the DAL_I2C_Init() and when the state is DAL_I2C_STATE_RESET
     all callbacks are set to the corresponding weak functions:
     examples DAL_I2C_MasterTxCpltCallback(), DAL_I2C_MasterRxCpltCallback().
     Exception done for MspInit and MspDeInit functions that are
     reset to the legacy weak functions in the DAL_I2C_Init()/ DAL_I2C_DeInit() only when
     these callbacks are null (not registered beforehand).
     If MspInit or MspDeInit are not null, the DAL_I2C_Init()/ DAL_I2C_DeInit()
     keep and use the user MspInit/MspDeInit callbacks (registered beforehand) whatever the state.
    [..]
     Callbacks can be registered/unregistered in DAL_I2C_STATE_READY state only.
     Exception done MspInit/MspDeInit functions that can be registered/unregistered
     in DAL_I2C_STATE_READY or DAL_I2C_STATE_RESET state,
     thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
     Then, the user first registers the MspInit/MspDeInit user callbacks
     using DAL_I2C_RegisterCallback() before calling DAL_I2C_DeInit()
     or DAL_I2C_Init() function.
    [..]
     When the compilation flag USE_DAL_I2C_REGISTER_CALLBACKS is set to 0 or
     not defined, the callback registration feature is not available and all callbacks
     are set to the corresponding weak functions.



     [..]
       (@) You can refer to the I2C DAL driver header file for more useful macros

  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup I2C I2C
  * @brief I2C DAL module driver
  * @{
  */

#ifdef DAL_I2C_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup I2C_Private_Define
  * @{
  */
#define I2C_TIMEOUT_FLAG          35U         /*!< Timeout 35 ms             */
#define I2C_TIMEOUT_BUSY_FLAG     25U         /*!< Timeout 25 ms             */
#define I2C_TIMEOUT_STOP_FLAG     5U          /*!< Timeout 5 ms              */
#define I2C_NO_OPTION_FRAME       0xFFFF0000U /*!< XferOptions default value */

/* Private define for @ref PreviousState usage */
#define I2C_STATE_MSK             ((uint32_t)((uint32_t)((uint32_t)DAL_I2C_STATE_BUSY_TX | (uint32_t)DAL_I2C_STATE_BUSY_RX) & (uint32_t)(~((uint32_t)DAL_I2C_STATE_READY)))) /*!< Mask State define, keep only RX and TX bits            */
#define I2C_STATE_NONE            ((uint32_t)(DAL_I2C_MODE_NONE))                                                        /*!< Default Value                                          */
#define I2C_STATE_MASTER_BUSY_TX  ((uint32_t)(((uint32_t)DAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | (uint32_t)DAL_I2C_MODE_MASTER))            /*!< Master Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_MASTER_BUSY_RX  ((uint32_t)(((uint32_t)DAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | (uint32_t)DAL_I2C_MODE_MASTER))            /*!< Master Busy RX, combinaison of State LSB and Mode enum */
#define I2C_STATE_SLAVE_BUSY_TX   ((uint32_t)(((uint32_t)DAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | (uint32_t)DAL_I2C_MODE_SLAVE))             /*!< Slave Busy TX, combinaison of State LSB and Mode enum  */
#define I2C_STATE_SLAVE_BUSY_RX   ((uint32_t)(((uint32_t)DAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | (uint32_t)DAL_I2C_MODE_SLAVE))             /*!< Slave Busy RX, combinaison of State LSB and Mode enum  */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @defgroup I2C_Private_Functions I2C Private Functions
  * @{
  */
/* Private functions to handle DMA transfer */
static void I2C_DMAXferCplt(DMA_HandleTypeDef *hdma);
static void I2C_DMAError(DMA_HandleTypeDef *hdma);
static void I2C_DMAAbort(DMA_HandleTypeDef *hdma);

static void I2C_ITError(I2C_HandleTypeDef *hi2c);

static DAL_StatusTypeDef I2C_MasterRequestWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout, uint32_t Tickstart);
static DAL_StatusTypeDef I2C_MasterRequestRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout, uint32_t Tickstart);
static DAL_StatusTypeDef I2C_RequestMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);
static DAL_StatusTypeDef I2C_RequestMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);

/* Private functions to handle flags during polling transfer */
static DAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
static DAL_StatusTypeDef I2C_WaitOnMasterAddressFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, uint32_t Timeout, uint32_t Tickstart);
static DAL_StatusTypeDef I2C_WaitOnTXEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static DAL_StatusTypeDef I2C_WaitOnBTFFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static DAL_StatusTypeDef I2C_WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static DAL_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static DAL_StatusTypeDef I2C_WaitOnSTOPRequestThroughIT(I2C_HandleTypeDef *hi2c);
static DAL_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c);

/* Private functions for I2C transfer IRQ handler */
static void I2C_MasterTransmit_TXE(I2C_HandleTypeDef *hi2c);
static void I2C_MasterTransmit_BTF(I2C_HandleTypeDef *hi2c);
static void I2C_MasterReceive_RXNE(I2C_HandleTypeDef *hi2c);
static void I2C_MasterReceive_BTF(I2C_HandleTypeDef *hi2c);
static void I2C_Master_SB(I2C_HandleTypeDef *hi2c);
static void I2C_Master_ADD10(I2C_HandleTypeDef *hi2c);
static void I2C_Master_ADDR(I2C_HandleTypeDef *hi2c);

static void I2C_SlaveTransmit_TXE(I2C_HandleTypeDef *hi2c);
static void I2C_SlaveTransmit_BTF(I2C_HandleTypeDef *hi2c);
static void I2C_SlaveReceive_RXNE(I2C_HandleTypeDef *hi2c);
static void I2C_SlaveReceive_BTF(I2C_HandleTypeDef *hi2c);
static void I2C_Slave_ADDR(I2C_HandleTypeDef *hi2c, uint32_t IT2Flags);
static void I2C_Slave_STOPF(I2C_HandleTypeDef *hi2c);
static void I2C_Slave_AF(I2C_HandleTypeDef *hi2c);

static void I2C_MemoryTransmit_TXE_BTF(I2C_HandleTypeDef *hi2c);

/* Private function to Convert Specific options */
static void I2C_ConvertOtherXferOptions(I2C_HandleTypeDef *hi2c);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup I2C_Exported_Functions I2C Exported Functions
  * @{
  */

/** @defgroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the I2Cx peripheral:

      (+) User must Implement DAL_I2C_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC).

      (+) Call the function DAL_I2C_Init() to configure the selected device with
          the selected configuration:
        (++) Communication Speed
        (++) Duty cycle
        (++) Addressing mode
        (++) Own Address 1
        (++) Dual Addressing mode
        (++) Own Address 2
        (++) General call mode
        (++) Nostretch mode

      (+) Call the function DAL_I2C_DeInit() to restore the default configuration
          of the selected I2Cx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the I2C according to the specified parameters
  *         in the I2C_InitTypeDef and initialize the associated handle.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Init(I2C_HandleTypeDef *hi2c)
{
  uint32_t freqrange;
  uint32_t pclk1;

  /* Check the I2C handle allocation */
  if (hi2c == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  ASSERT_PARAM(IS_I2C_CLOCK_SPEED(hi2c->Init.ClockSpeed));
  ASSERT_PARAM(IS_I2C_DUTY_CYCLE(hi2c->Init.DutyCycle));
  ASSERT_PARAM(IS_I2C_OWN_ADDRESS1(hi2c->Init.OwnAddress1));
  ASSERT_PARAM(IS_I2C_ADDRESSING_MODE(hi2c->Init.AddressingMode));
  ASSERT_PARAM(IS_I2C_DUAL_ADDRESS(hi2c->Init.DualAddressMode));
  ASSERT_PARAM(IS_I2C_OWN_ADDRESS2(hi2c->Init.OwnAddress2));
  ASSERT_PARAM(IS_I2C_GENERAL_CALL(hi2c->Init.GeneralCallMode));
  ASSERT_PARAM(IS_I2C_NO_STRETCH(hi2c->Init.NoStretchMode));

  if (hi2c->State == DAL_I2C_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hi2c->Lock = DAL_UNLOCKED;

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
    /* Init the I2C Callback settings */
    hi2c->MasterTxCpltCallback = DAL_I2C_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
    hi2c->MasterRxCpltCallback = DAL_I2C_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
    hi2c->SlaveTxCpltCallback  = DAL_I2C_SlaveTxCpltCallback;  /* Legacy weak SlaveTxCpltCallback  */
    hi2c->SlaveRxCpltCallback  = DAL_I2C_SlaveRxCpltCallback;  /* Legacy weak SlaveRxCpltCallback  */
    hi2c->ListenCpltCallback   = DAL_I2C_ListenCpltCallback;   /* Legacy weak ListenCpltCallback   */
    hi2c->MemTxCpltCallback    = DAL_I2C_MemTxCpltCallback;    /* Legacy weak MemTxCpltCallback    */
    hi2c->MemRxCpltCallback    = DAL_I2C_MemRxCpltCallback;    /* Legacy weak MemRxCpltCallback    */
    hi2c->ErrorCallback        = DAL_I2C_ErrorCallback;        /* Legacy weak ErrorCallback        */
    hi2c->AbortCpltCallback    = DAL_I2C_AbortCpltCallback;    /* Legacy weak AbortCpltCallback    */
    hi2c->AddrCallback         = DAL_I2C_AddrCallback;         /* Legacy weak AddrCallback         */

    if (hi2c->MspInitCallback == NULL)
    {
      hi2c->MspInitCallback = DAL_I2C_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    hi2c->MspInitCallback(hi2c);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    DAL_I2C_MspInit(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
  }

  hi2c->State = DAL_I2C_STATE_BUSY;

  /* Disable the selected I2C peripheral */
  __DAL_I2C_DISABLE(hi2c);

  /*Reset I2C*/
  hi2c->Instance->CTRL1 |= I2C_CTRL1_SWRST;
  hi2c->Instance->CTRL1 &= ~I2C_CTRL1_SWRST;

  /* Get PCLK1 frequency */
  pclk1 = DAL_RCM_GetPCLK1Freq();

  /* Check the minimum allowed PCLK1 frequency */
  if (I2C_MIN_PCLK_FREQ(pclk1, hi2c->Init.ClockSpeed) == 1U)
  {
    return DAL_ERROR;
  }

  /* Calculate frequency range */
  freqrange = I2C_FREQRANGE(pclk1);

  /*---------------------------- I2Cx CTRL2 Configuration ----------------------*/
  /* Configure I2Cx: Frequency range */
  MODIFY_REG(hi2c->Instance->CTRL2, I2C_CTRL2_CLKFCFG, freqrange);

  /*---------------------------- I2Cx RISETMAX Configuration --------------------*/
  /* Configure I2Cx: Rise Time */
  MODIFY_REG(hi2c->Instance->RISETMAX, I2C_RISETMAX_RISETMAX, I2C_RISE_TIME(freqrange, hi2c->Init.ClockSpeed));

  /*---------------------------- I2Cx CLKCTRL Configuration ----------------------*/
  /* Configure I2Cx: Speed */
  MODIFY_REG(hi2c->Instance->CLKCTRL, (I2C_CLKCTRL_SPEEDCFG | I2C_CLKCTRL_FDUTYCFG | I2C_CLKCTRL_CLKS), I2C_SPEED(pclk1, hi2c->Init.ClockSpeed, hi2c->Init.DutyCycle));

  /*---------------------------- I2Cx CTRL1 Configuration ----------------------*/
  /* Configure I2Cx: Generalcall and NoStretch mode */
  MODIFY_REG(hi2c->Instance->CTRL1, (I2C_CTRL1_SRBEN | I2C_CTRL1_CLKSTRETCHD), (hi2c->Init.GeneralCallMode | hi2c->Init.NoStretchMode));

  /*---------------------------- I2Cx SADDR1 Configuration ---------------------*/
  /* Configure I2Cx: Own Address1 and addressing mode */
  MODIFY_REG(hi2c->Instance->SADDR1, (I2C_SADDR1_ADDRLEN | I2C_SADDR1_ADDR8_9 | I2C_SADDR1_ADDR1_7 | I2C_SADDR1_ADDR0), (hi2c->Init.AddressingMode | hi2c->Init.OwnAddress1));

  /*---------------------------- I2Cx SADDR2 Configuration ---------------------*/
  /* Configure I2Cx: Dual mode and Own Address2 */
  MODIFY_REG(hi2c->Instance->SADDR2, (I2C_SADDR2_ADDRNUM | I2C_SADDR2_ADDR2), (hi2c->Init.DualAddressMode | hi2c->Init.OwnAddress2));

  /* Enable the selected I2C peripheral */
  __DAL_I2C_ENABLE(hi2c);

  hi2c->ErrorCode = DAL_I2C_ERROR_NONE;
  hi2c->State = DAL_I2C_STATE_READY;
  hi2c->PreviousState = I2C_STATE_NONE;
  hi2c->Mode = DAL_I2C_MODE_NONE;

  return DAL_OK;
}

/**
  * @brief  DeInitialize the I2C peripheral.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_DeInit(I2C_HandleTypeDef *hi2c)
{
  /* Check the I2C handle allocation */
  if (hi2c == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_ALL_INSTANCE(hi2c->Instance));

  hi2c->State = DAL_I2C_STATE_BUSY;

  /* Disable the I2C Peripheral Clock */
  __DAL_I2C_DISABLE(hi2c);

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
  if (hi2c->MspDeInitCallback == NULL)
  {
    hi2c->MspDeInitCallback = DAL_I2C_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  hi2c->MspDeInitCallback(hi2c);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  DAL_I2C_MspDeInit(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */

  hi2c->ErrorCode     = DAL_I2C_ERROR_NONE;
  hi2c->State         = DAL_I2C_STATE_RESET;
  hi2c->PreviousState = I2C_STATE_NONE;
  hi2c->Mode          = DAL_I2C_MODE_NONE;

  /* Release Lock */
  __DAL_UNLOCK(hi2c);

  return DAL_OK;
}

/**
  * @brief  Initialize the I2C MSP.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitialize the I2C MSP.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_MspDeInit could be implemented in the user file
   */
}

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User I2C Callback
  *         To be used instead of the weak predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref DAL_I2C_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref DAL_I2C_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref DAL_I2C_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref DAL_I2C_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref DAL_I2C_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref DAL_I2C_MEM_TX_COMPLETE_CB_ID Memory Tx Transfer callback ID
  *          @arg @ref DAL_I2C_MEM_RX_COMPLETE_CB_ID Memory Rx Transfer completed callback ID
  *          @arg @ref DAL_I2C_ERROR_CB_ID Error callback ID
  *          @arg @ref DAL_I2C_ABORT_CB_ID Abort callback ID
  *          @arg @ref DAL_I2C_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref DAL_I2C_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_RegisterCallback(I2C_HandleTypeDef *hi2c, DAL_I2C_CallbackIDTypeDef CallbackID, pI2C_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hi2c->ErrorCode |= DAL_I2C_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hi2c);

  if (DAL_I2C_STATE_READY == hi2c->State)
  {
    switch (CallbackID)
    {
      case DAL_I2C_MASTER_TX_COMPLETE_CB_ID :
        hi2c->MasterTxCpltCallback = pCallback;
        break;

      case DAL_I2C_MASTER_RX_COMPLETE_CB_ID :
        hi2c->MasterRxCpltCallback = pCallback;
        break;

      case DAL_I2C_SLAVE_TX_COMPLETE_CB_ID :
        hi2c->SlaveTxCpltCallback = pCallback;
        break;

      case DAL_I2C_SLAVE_RX_COMPLETE_CB_ID :
        hi2c->SlaveRxCpltCallback = pCallback;
        break;

      case DAL_I2C_LISTEN_COMPLETE_CB_ID :
        hi2c->ListenCpltCallback = pCallback;
        break;

      case DAL_I2C_MEM_TX_COMPLETE_CB_ID :
        hi2c->MemTxCpltCallback = pCallback;
        break;

      case DAL_I2C_MEM_RX_COMPLETE_CB_ID :
        hi2c->MemRxCpltCallback = pCallback;
        break;

      case DAL_I2C_ERROR_CB_ID :
        hi2c->ErrorCallback = pCallback;
        break;

      case DAL_I2C_ABORT_CB_ID :
        hi2c->AbortCpltCallback = pCallback;
        break;

      case DAL_I2C_MSPINIT_CB_ID :
        hi2c->MspInitCallback = pCallback;
        break;

      case DAL_I2C_MSPDEINIT_CB_ID :
        hi2c->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_I2C_STATE_RESET == hi2c->State)
  {
    switch (CallbackID)
    {
      case DAL_I2C_MSPINIT_CB_ID :
        hi2c->MspInitCallback = pCallback;
        break;

      case DAL_I2C_MSPDEINIT_CB_ID :
        hi2c->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hi2c->ErrorCode |= DAL_I2C_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hi2c);
  return status;
}

/**
  * @brief  Unregister an I2C Callback
  *         I2C callback is redirected to the weak predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *         This parameter can be one of the following values:
  *          @arg @ref DAL_I2C_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref DAL_I2C_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref DAL_I2C_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref DAL_I2C_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref DAL_I2C_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref DAL_I2C_MEM_TX_COMPLETE_CB_ID Memory Tx Transfer callback ID
  *          @arg @ref DAL_I2C_MEM_RX_COMPLETE_CB_ID Memory Rx Transfer completed callback ID
  *          @arg @ref DAL_I2C_ERROR_CB_ID Error callback ID
  *          @arg @ref DAL_I2C_ABORT_CB_ID Abort callback ID
  *          @arg @ref DAL_I2C_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref DAL_I2C_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_UnRegisterCallback(I2C_HandleTypeDef *hi2c, DAL_I2C_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hi2c);

  if (DAL_I2C_STATE_READY == hi2c->State)
  {
    switch (CallbackID)
    {
      case DAL_I2C_MASTER_TX_COMPLETE_CB_ID :
        hi2c->MasterTxCpltCallback = DAL_I2C_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
        break;

      case DAL_I2C_MASTER_RX_COMPLETE_CB_ID :
        hi2c->MasterRxCpltCallback = DAL_I2C_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
        break;

      case DAL_I2C_SLAVE_TX_COMPLETE_CB_ID :
        hi2c->SlaveTxCpltCallback = DAL_I2C_SlaveTxCpltCallback;   /* Legacy weak SlaveTxCpltCallback  */
        break;

      case DAL_I2C_SLAVE_RX_COMPLETE_CB_ID :
        hi2c->SlaveRxCpltCallback = DAL_I2C_SlaveRxCpltCallback;   /* Legacy weak SlaveRxCpltCallback  */
        break;

      case DAL_I2C_LISTEN_COMPLETE_CB_ID :
        hi2c->ListenCpltCallback = DAL_I2C_ListenCpltCallback;     /* Legacy weak ListenCpltCallback   */
        break;

      case DAL_I2C_MEM_TX_COMPLETE_CB_ID :
        hi2c->MemTxCpltCallback = DAL_I2C_MemTxCpltCallback;       /* Legacy weak MemTxCpltCallback    */
        break;

      case DAL_I2C_MEM_RX_COMPLETE_CB_ID :
        hi2c->MemRxCpltCallback = DAL_I2C_MemRxCpltCallback;       /* Legacy weak MemRxCpltCallback    */
        break;

      case DAL_I2C_ERROR_CB_ID :
        hi2c->ErrorCallback = DAL_I2C_ErrorCallback;               /* Legacy weak ErrorCallback        */
        break;

      case DAL_I2C_ABORT_CB_ID :
        hi2c->AbortCpltCallback = DAL_I2C_AbortCpltCallback;       /* Legacy weak AbortCpltCallback    */
        break;

      case DAL_I2C_MSPINIT_CB_ID :
        hi2c->MspInitCallback = DAL_I2C_MspInit;                   /* Legacy weak MspInit              */
        break;

      case DAL_I2C_MSPDEINIT_CB_ID :
        hi2c->MspDeInitCallback = DAL_I2C_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_I2C_STATE_RESET == hi2c->State)
  {
    switch (CallbackID)
    {
      case DAL_I2C_MSPINIT_CB_ID :
        hi2c->MspInitCallback = DAL_I2C_MspInit;                   /* Legacy weak MspInit              */
        break;

      case DAL_I2C_MSPDEINIT_CB_ID :
        hi2c->MspDeInitCallback = DAL_I2C_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hi2c->ErrorCode |= DAL_I2C_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hi2c);
  return status;
}

/**
  * @brief  Register the Slave Address Match I2C Callback
  *         To be used instead of the weak DAL_I2C_AddrCallback() predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pCallback pointer to the Address Match Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_RegisterAddrCallback(I2C_HandleTypeDef *hi2c, pI2C_AddrCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hi2c->ErrorCode |= DAL_I2C_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hi2c);

  if (DAL_I2C_STATE_READY == hi2c->State)
  {
    hi2c->AddrCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hi2c->ErrorCode |= DAL_I2C_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hi2c);
  return status;
}

/**
  * @brief  UnRegister the Slave Address Match I2C Callback
  *         Info Ready I2C Callback is redirected to the weak DAL_I2C_AddrCallback() predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_UnRegisterAddrCallback(I2C_HandleTypeDef *hi2c)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hi2c);

  if (DAL_I2C_STATE_READY == hi2c->State)
  {
    hi2c->AddrCallback = DAL_I2C_AddrCallback; /* Legacy weak AddrCallback  */
  }
  else
  {
    /* Update the error code */
    hi2c->ErrorCode |= DAL_I2C_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hi2c);
  return status;
}

#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the I2C data
    transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode : The communication is performed in the polling mode.
            The status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode : The communication is performed using Interrupts
            or DMA. These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated I2C IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.

    (#) Blocking mode functions are :
        (++) DAL_I2C_Master_Transmit()
        (++) DAL_I2C_Master_Receive()
        (++) DAL_I2C_Slave_Transmit()
        (++) DAL_I2C_Slave_Receive()
        (++) DAL_I2C_Mem_Write()
        (++) DAL_I2C_Mem_Read()
        (++) DAL_I2C_IsDeviceReady()

    (#) No-Blocking mode functions with Interrupt are :
        (++) DAL_I2C_Master_Transmit_IT()
        (++) DAL_I2C_Master_Receive_IT()
        (++) DAL_I2C_Slave_Transmit_IT()
        (++) DAL_I2C_Slave_Receive_IT()
        (++) DAL_I2C_Mem_Write_IT()
        (++) DAL_I2C_Mem_Read_IT()
        (++) DAL_I2C_Master_Seq_Transmit_IT()
        (++) DAL_I2C_Master_Seq_Receive_IT()
        (++) DAL_I2C_Slave_Seq_Transmit_IT()
        (++) DAL_I2C_Slave_Seq_Receive_IT()
        (++) DAL_I2C_EnableListen_IT()
        (++) DAL_I2C_DisableListen_IT()
        (++) DAL_I2C_Master_Abort_IT()

    (#) No-Blocking mode functions with DMA are :
        (++) DAL_I2C_Master_Transmit_DMA()
        (++) DAL_I2C_Master_Receive_DMA()
        (++) DAL_I2C_Slave_Transmit_DMA()
        (++) DAL_I2C_Slave_Receive_DMA()
        (++) DAL_I2C_Mem_Write_DMA()
        (++) DAL_I2C_Mem_Read_DMA()
        (++) DAL_I2C_Master_Seq_Transmit_DMA()
        (++) DAL_I2C_Master_Seq_Receive_DMA()
        (++) DAL_I2C_Slave_Seq_Transmit_DMA()
        (++) DAL_I2C_Slave_Seq_Receive_DMA()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) DAL_I2C_MasterTxCpltCallback()
        (++) DAL_I2C_MasterRxCpltCallback()
        (++) DAL_I2C_SlaveTxCpltCallback()
        (++) DAL_I2C_SlaveRxCpltCallback()
        (++) DAL_I2C_MemTxCpltCallback()
        (++) DAL_I2C_MemRxCpltCallback()
        (++) DAL_I2C_AddrCallback()
        (++) DAL_I2C_ListenCpltCallback()
        (++) DAL_I2C_ErrorCallback()
        (++) DAL_I2C_AbortCpltCallback()

@endverbatim
  * @{
  */

/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = DAL_GetTick();

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != DAL_OK)
    {
      return DAL_BUSY;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State       = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode        = DAL_I2C_MODE_MASTER;
    hi2c->ErrorCode   = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    /* Send Slave Address */
    if (I2C_MasterRequestWrite(hi2c, DevAddress, Timeout, tickstart) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Clear ADDR flag */
    __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

    while (hi2c->XferSize > 0U)
    {
      /* Wait until TXE flag is set */
      if (I2C_WaitOnTXEFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
      {
        if (hi2c->ErrorCode == DAL_I2C_ERROR_AF)
        {
          /* Generate Stop */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
        }
        return DAL_ERROR;
      }

      /* Write data to DATA */
      hi2c->Instance->DATA = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferCount--;
      hi2c->XferSize--;

      if ((__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET) && (hi2c->XferSize != 0U))
      {
        /* Write data to DATA */
        hi2c->Instance->DATA = *hi2c->pBuffPtr;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        /* Update counter */
        hi2c->XferCount--;
        hi2c->XferSize--;
      }

      /* Wait until BTF flag is set */
      if (I2C_WaitOnBTFFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
      {
        if (hi2c->ErrorCode == DAL_I2C_ERROR_AF)
        {
          /* Generate Stop */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
        }
        return DAL_ERROR;
      }
    }

    /* Generate Stop */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

    hi2c->State = DAL_I2C_STATE_READY;
    hi2c->Mode = DAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Receives in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = DAL_GetTick();

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != DAL_OK)
    {
      return DAL_BUSY;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State       = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode        = DAL_I2C_MODE_MASTER;
    hi2c->ErrorCode   = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    /* Send Slave Address */
    if (I2C_MasterRequestRead(hi2c, DevAddress, Timeout, tickstart) != DAL_OK)
    {
      return DAL_ERROR;
    }

    if (hi2c->XferSize == 0U)
    {
      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
    }
    else if (hi2c->XferSize == 1U)
    {
      /* Disable Acknowledge */
      CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
    }
    else if (hi2c->XferSize == 2U)
    {
      /* Disable Acknowledge */
      CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Enable Pos */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }
    else
    {
      /* Enable Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }

    while (hi2c->XferSize > 0U)
    {
      if (hi2c->XferSize <= 3U)
      {
        /* One byte */
        if (hi2c->XferSize == 1U)
        {
          /* Wait until RXNE flag is set */
          if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
          {
            return DAL_ERROR;
          }

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
        /* Two bytes */
        else if (hi2c->XferSize == 2U)
        {
          /* Wait until BTF flag is set */
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != DAL_OK)
          {
            return DAL_ERROR;
          }

          /* Generate Stop */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
        /* 3 Last bytes */
        else
        {
          /* Wait until BTF flag is set */
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != DAL_OK)
          {
            return DAL_ERROR;
          }

          /* Disable Acknowledge */
          CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;

          /* Wait until BTF flag is set */
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != DAL_OK)
          {
            return DAL_ERROR;
          }

          /* Generate Stop */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
      }
      else
      {
        /* Wait until RXNE flag is set */
        if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
        {
          return DAL_ERROR;
        }

        /* Read data from DATA */
        *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        /* Update counter */
        hi2c->XferSize--;
        hi2c->XferCount--;

        if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET)
        {
          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
      }
    }

    hi2c->State = DAL_I2C_STATE_READY;
    hi2c->Mode = DAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Transmits in slave mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = DAL_GetTick();

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State       = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode        = DAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode   = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    /* Enable Address Acknowledge */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Wait until ADDR flag is set */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, RESET, Timeout, tickstart) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Clear ADDR flag */
    __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

    /* If 10bit addressing mode is selected */
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT)
    {
      /* Wait until ADDR flag is set */
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, RESET, Timeout, tickstart) != DAL_OK)
      {
        return DAL_ERROR;
      }

      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }

    while (hi2c->XferSize > 0U)
    {
      /* Wait until TXE flag is set */
      if (I2C_WaitOnTXEFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
      {
        /* Disable Address Acknowledge */
        CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

        return DAL_ERROR;
      }

      /* Write data to DATA */
      hi2c->Instance->DATA = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferCount--;
      hi2c->XferSize--;

      if ((__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET) && (hi2c->XferSize != 0U))
      {
        /* Write data to DATA */
        hi2c->Instance->DATA = *hi2c->pBuffPtr;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        /* Update counter */
        hi2c->XferCount--;
        hi2c->XferSize--;
      }
    }

    /* Wait until AF flag is set */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_AF, RESET, Timeout, tickstart) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Clear AF flag */
    __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

    /* Disable Address Acknowledge */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    hi2c->State = DAL_I2C_STATE_READY;
    hi2c->Mode = DAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in blocking mode
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = DAL_GetTick();

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == (uint16_t)0))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State       = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode        = DAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode   = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    /* Enable Address Acknowledge */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Wait until ADDR flag is set */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, RESET, Timeout, tickstart) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Clear ADDR flag */
    __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

    while (hi2c->XferSize > 0U)
    {
      /* Wait until RXNE flag is set */
      if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
      {
        /* Disable Address Acknowledge */
        CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

        return DAL_ERROR;
      }

      /* Read data from DATA */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferSize--;
      hi2c->XferCount--;

      if ((__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET) && (hi2c->XferSize != 0U))
      {
        /* Read data from DATA */
        *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        /* Update counter */
        hi2c->XferSize--;
        hi2c->XferCount--;
      }
    }

    /* Wait until STOP flag is set */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
    {
      /* Disable Address Acknowledge */
      CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      return DAL_ERROR;
    }

    /* Clear STOP flag */
    __DAL_I2C_CLEAR_STOPFLAG(hi2c);

    /* Disable Address Acknowledge */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    hi2c->State = DAL_I2C_STATE_READY;
    hi2c->Mode = DAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
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
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  __IO uint32_t count = 0U;

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
    do
    {
      count--;
      if (count == 0U)
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = DAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->Devaddress  = DevAddress;

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
    /* Enable EVT, BUF and ERR interrupt */
    __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    /* Generate Start */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
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
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  __IO uint32_t count = 0U;

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
    do
    {
      count--;
      if (count == 0U)
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = DAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->Devaddress  = DevAddress;


    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
    to avoid the risk of I2C interrupt handle execution before current
    process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    /* Enable Acknowledge */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Generate Start */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = DAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    /* Enable Address Acknowledge */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = DAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    /* Enable Address Acknowledge */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
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
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  __IO uint32_t count = 0U;
  DAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
    do
    {
      count--;
      if (count == 0U)
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = DAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->Devaddress  = DevAddress;

    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmatx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmatx->XferCpltCallback = I2C_DMAXferCplt;

        /* Set the DMA error callback */
        hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

        /* Set the unused DMA callbacks to NULL */
        hi2c->hdmatx->XferHalfCpltCallback = NULL;
        hi2c->hdmatx->XferM1CpltCallback = NULL;
        hi2c->hdmatx->XferM1HalfCpltCallback = NULL;
        hi2c->hdmatx->XferAbortCallback = NULL;

        /* Enable the DMA stream */
        dmaxferstatus = DAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)hi2c->pBuffPtr, (uint32_t)&hi2c->Instance->DATA, hi2c->XferSize);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }

      if (dmaxferstatus == DAL_OK)
      {
        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
        to avoid the risk of I2C interrupt handle execution before current
        process unlock */

        /* Enable EVT and ERR interrupt */
        __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

        /* Enable DMA Request */
        SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

        /* Enable Acknowledge */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

        /* Generate Start */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    else
    {
      /* Enable Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Generate Start */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
      to avoid the risk of I2C interrupt handle execution before current
      process unlock */

      /* Enable EVT, BUF and ERR interrupt */
      __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
    }

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
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
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  __IO uint32_t count = 0U;
  DAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
    do
    {
      count--;
      if (count == 0U)
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = DAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->Devaddress  = DevAddress;

    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmarx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmarx->XferCpltCallback = I2C_DMAXferCplt;

        /* Set the DMA error callback */
        hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

        /* Set the unused DMA callbacks to NULL */
        hi2c->hdmarx->XferHalfCpltCallback = NULL;
        hi2c->hdmarx->XferM1CpltCallback = NULL;
        hi2c->hdmarx->XferM1HalfCpltCallback = NULL;
        hi2c->hdmarx->XferAbortCallback = NULL;

        /* Enable the DMA stream */
        dmaxferstatus = DAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->DATA, (uint32_t)hi2c->pBuffPtr, hi2c->XferSize);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }

      if (dmaxferstatus == DAL_OK)
      {
        /* Enable Acknowledge */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

        /* Generate Start */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
        to avoid the risk of I2C interrupt handle execution before current
        process unlock */

        /* Enable EVT and ERR interrupt */
        __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

        /* Enable DMA Request */
        SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    else
    {
      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
      to avoid the risk of I2C interrupt handle execution before current
      process unlock */

      /* Enable EVT, BUF and ERR interrupt */
      __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

      /* Enable Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Generate Start */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
    }

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  DAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = DAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    if (hi2c->hdmatx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmatx->XferCpltCallback = I2C_DMAXferCplt;

      /* Set the DMA error callback */
      hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hi2c->hdmatx->XferHalfCpltCallback = NULL;
      hi2c->hdmatx->XferM1CpltCallback = NULL;
      hi2c->hdmatx->XferM1HalfCpltCallback = NULL;
      hi2c->hdmatx->XferAbortCallback = NULL;

      /* Enable the DMA stream */
      dmaxferstatus = DAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)hi2c->pBuffPtr, (uint32_t)&hi2c->Instance->DATA, hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = DAL_I2C_STATE_LISTEN;
      hi2c->Mode      = DAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= DAL_I2C_ERROR_DMA_PARAM;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }

    if (dmaxferstatus == DAL_OK)
    {
      /* Enable Address Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
      to avoid the risk of I2C interrupt handle execution before current
      process unlock */
      /* Enable EVT and ERR interrupt */
      __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

      /* Enable DMA Request */
      hi2c->Instance->CTRL2 |= I2C_CTRL2_DMAEN;

      return DAL_OK;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = DAL_I2C_STATE_READY;
      hi2c->Mode      = DAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  DAL_StatusTypeDef dmaxferstatus;

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = DAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    if (hi2c->hdmarx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmarx->XferCpltCallback = I2C_DMAXferCplt;

      /* Set the DMA error callback */
      hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hi2c->hdmarx->XferHalfCpltCallback = NULL;
      hi2c->hdmarx->XferM1CpltCallback = NULL;
      hi2c->hdmarx->XferM1HalfCpltCallback = NULL;
      hi2c->hdmarx->XferAbortCallback = NULL;

      /* Enable the DMA stream */
      dmaxferstatus = DAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->DATA, (uint32_t)hi2c->pBuffPtr, hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = DAL_I2C_STATE_LISTEN;
      hi2c->Mode      = DAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= DAL_I2C_ERROR_DMA_PARAM;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }

    if (dmaxferstatus == DAL_OK)
    {
      /* Enable Address Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
      to avoid the risk of I2C interrupt handle execution before current
      process unlock */
      /* Enable EVT and ERR interrupt */
      __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

      /* Enable DMA Request */
      SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

      return DAL_OK;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = DAL_I2C_STATE_READY;
      hi2c->Mode      = DAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = DAL_GetTick();

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != DAL_OK)
    {
      return DAL_BUSY;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = DAL_I2C_MODE_MEM;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    /* Send Slave Address and Memory Address */
    if (I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != DAL_OK)
    {
      return DAL_ERROR;
    }

    while (hi2c->XferSize > 0U)
    {
      /* Wait until TXE flag is set */
      if (I2C_WaitOnTXEFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
      {
        if (hi2c->ErrorCode == DAL_I2C_ERROR_AF)
        {
          /* Generate Stop */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
        }
        return DAL_ERROR;
      }

      /* Write data to DATA */
      hi2c->Instance->DATA = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferSize--;
      hi2c->XferCount--;

      if ((__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET) && (hi2c->XferSize != 0U))
      {
        /* Write data to DATA */
        hi2c->Instance->DATA = *hi2c->pBuffPtr;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        /* Update counter */
        hi2c->XferSize--;
        hi2c->XferCount--;
      }
    }

    /* Wait until BTF flag is set */
    if (I2C_WaitOnBTFFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
    {
      if (hi2c->ErrorCode == DAL_I2C_ERROR_AF)
      {
        /* Generate Stop */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
      }
      return DAL_ERROR;
    }

    /* Generate Stop */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

    hi2c->State = DAL_I2C_STATE_READY;
    hi2c->Mode = DAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = DAL_GetTick();

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != DAL_OK)
    {
      return DAL_BUSY;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = DAL_I2C_MODE_MEM;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    /* Send Slave Address and Memory Address */
    if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != DAL_OK)
    {
      return DAL_ERROR;
    }

    if (hi2c->XferSize == 0U)
    {
      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
    }
    else if (hi2c->XferSize == 1U)
    {
      /* Disable Acknowledge */
      CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
    }
    else if (hi2c->XferSize == 2U)
    {
      /* Disable Acknowledge */
      CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Enable Pos */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }
    else
    {
      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }

    while (hi2c->XferSize > 0U)
    {
      if (hi2c->XferSize <= 3U)
      {
        /* One byte */
        if (hi2c->XferSize == 1U)
        {
          /* Wait until RXNE flag is set */
          if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
          {
            return DAL_ERROR;
          }

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
        /* Two bytes */
        else if (hi2c->XferSize == 2U)
        {
          /* Wait until BTF flag is set */
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != DAL_OK)
          {
            return DAL_ERROR;
          }

          /* Generate Stop */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
        /* 3 Last bytes */
        else
        {
          /* Wait until BTF flag is set */
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != DAL_OK)
          {
            return DAL_ERROR;
          }

          /* Disable Acknowledge */
          CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;

          /* Wait until BTF flag is set */
          if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != DAL_OK)
          {
            return DAL_ERROR;
          }

          /* Generate Stop */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;

          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
      }
      else
      {
        /* Wait until RXNE flag is set */
        if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != DAL_OK)
        {
          return DAL_ERROR;
        }

        /* Read data from DATA */
        *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        /* Update counter */
        hi2c->XferSize--;
        hi2c->XferCount--;

        if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET)
        {
          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          /* Update counter */
          hi2c->XferSize--;
          hi2c->XferCount--;
        }
      }
    }

    hi2c->State = DAL_I2C_STATE_READY;
    hi2c->Mode = DAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Write an amount of data in non-blocking mode with Interrupt to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  __IO uint32_t count = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
    do
    {
      count--;
      if (count == 0U)
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = DAL_I2C_MODE_MEM;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->Devaddress  = DevAddress;
    hi2c->Memaddress  = MemAddress;
    hi2c->MemaddSize  = MemAddSize;
    hi2c->EventCount  = 0U;

    /* Generate Start */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
    to avoid the risk of I2C interrupt handle execution before current
    process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Read an amount of data in non-blocking mode with Interrupt from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  __IO uint32_t count = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
    do
    {
      count--;
      if (count == 0U)
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = DAL_I2C_MODE_MEM;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->Devaddress  = DevAddress;
    hi2c->Memaddress  = MemAddress;
    hi2c->MemaddSize  = MemAddSize;
    hi2c->EventCount  = 0U;

    /* Enable Acknowledge */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Generate Start */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    if (hi2c->XferSize > 0U)
    {
      /* Note : The I2C interrupts must be enabled after unlocking current process
      to avoid the risk of I2C interrupt handle execution before current
      process unlock */

      /* Enable EVT, BUF and ERR interrupt */
      __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
    }
    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Write an amount of data in non-blocking mode with DMA to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  __IO uint32_t count = 0U;
  DAL_StatusTypeDef dmaxferstatus;

  /* Init tickstart for timeout management*/
  uint32_t tickstart = DAL_GetTick();

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
    do
    {
      count--;
      if (count == 0U)
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = DAL_I2C_MODE_MEM;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->Devaddress  = DevAddress;
    hi2c->Memaddress  = MemAddress;
    hi2c->MemaddSize  = MemAddSize;
    hi2c->EventCount  = 0U;

    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmatx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmatx->XferCpltCallback = I2C_DMAXferCplt;

        /* Set the DMA error callback */
        hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

        /* Set the unused DMA callbacks to NULL */
        hi2c->hdmatx->XferHalfCpltCallback = NULL;
        hi2c->hdmatx->XferM1CpltCallback = NULL;
        hi2c->hdmatx->XferM1HalfCpltCallback = NULL;
        hi2c->hdmatx->XferAbortCallback = NULL;

        /* Enable the DMA stream */
        dmaxferstatus = DAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)hi2c->pBuffPtr, (uint32_t)&hi2c->Instance->DATA, hi2c->XferSize);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }

      if (dmaxferstatus == DAL_OK)
      {
        /* Send Slave Address and Memory Address */
        if (I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG, tickstart) != DAL_OK)
        {
          /* Abort the ongoing DMA */
          dmaxferstatus = DAL_DMA_Abort_IT(hi2c->hdmatx);

          /* Prevent unused argument(s) compilation and MISRA warning */
          UNUSED(dmaxferstatus);

          /* Set the unused I2C DMA transfer complete callback to NULL */
          hi2c->hdmatx->XferCpltCallback = NULL;

          /* Disable Acknowledge */
          CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

          hi2c->XferSize = 0U;
          hi2c->XferCount = 0U;

          /* Disable I2C peripheral to prevent dummy data in buffer */
          __DAL_I2C_DISABLE(hi2c);

          return DAL_ERROR;
        }

        /* Clear ADDR flag */
        __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
        to avoid the risk of I2C interrupt handle execution before current
        process unlock */
        /* Enable ERR interrupt */
        __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERR);

        /* Enable DMA Request */
        SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

        return DAL_OK;
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = DAL_I2C_STATE_READY;
      hi2c->Mode      = DAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= DAL_I2C_ERROR_SIZE;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Reads an amount of data in non-blocking mode with DMA from a specific memory address.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be read
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  /* Init tickstart for timeout management*/
  uint32_t tickstart = DAL_GetTick();
  __IO uint32_t count = 0U;
  DAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
    do
    {
      count--;
      if (count == 0U)
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = DAL_I2C_MODE_MEM;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->Devaddress  = DevAddress;
    hi2c->Memaddress  = MemAddress;
    hi2c->MemaddSize  = MemAddSize;
    hi2c->EventCount  = 0U;

    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmarx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmarx->XferCpltCallback = I2C_DMAXferCplt;

        /* Set the DMA error callback */
        hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

        /* Set the unused DMA callbacks to NULL */
        hi2c->hdmarx->XferHalfCpltCallback = NULL;
        hi2c->hdmarx->XferM1CpltCallback = NULL;
        hi2c->hdmarx->XferM1HalfCpltCallback = NULL;
        hi2c->hdmarx->XferAbortCallback = NULL;

        /* Enable the DMA stream */
        dmaxferstatus = DAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->DATA, (uint32_t)hi2c->pBuffPtr, hi2c->XferSize);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }

      if (dmaxferstatus == DAL_OK)
      {
        /* Send Slave Address and Memory Address */
        if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG, tickstart) != DAL_OK)
        {
          /* Abort the ongoing DMA */
          dmaxferstatus = DAL_DMA_Abort_IT(hi2c->hdmarx);

          /* Prevent unused argument(s) compilation and MISRA warning */
          UNUSED(dmaxferstatus);

          /* Set the unused I2C DMA transfer complete callback to NULL */
          hi2c->hdmarx->XferCpltCallback = NULL;

          /* Disable Acknowledge */
          CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

          hi2c->XferSize = 0U;
          hi2c->XferCount = 0U;

          /* Disable I2C peripheral to prevent dummy data in buffer */
          __DAL_I2C_DISABLE(hi2c);

          return DAL_ERROR;
        }

        if (hi2c->XferSize == 1U)
        {
          /* Disable Acknowledge */
          CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
        }
        else
        {
          /* Enable Last DMA bit */
          SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_LTCFG);
        }

        /* Clear ADDR flag */
        __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
        to avoid the risk of I2C interrupt handle execution before current
        process unlock */
        /* Enable ERR interrupt */
        __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERR);

        /* Enable DMA Request */
        hi2c->Instance->CTRL2 |= I2C_CTRL2_DMAEN;
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    else
    {
      /* Send Slave Address and Memory Address */
      if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_FLAG, tickstart) != DAL_OK)
      {
        return DAL_ERROR;
      }

      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

      hi2c->State = DAL_I2C_STATE_READY;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);
    }

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  Trials Number of trials
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
  /* Get tick */
  uint32_t tickstart = DAL_GetTick();
  uint32_t I2C_Trials = 0U;
  FlagStatus tmp1;
  FlagStatus tmp2;

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Wait until BUSY flag is reset */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != DAL_OK)
    {
      return DAL_BUSY;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State = DAL_I2C_STATE_BUSY;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    do
    {
      /* Generate Start */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

      /* Wait until SB flag is set */
      if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout, tickstart) != DAL_OK)
      {
        if (READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START) == I2C_CTRL1_START)
        {
          hi2c->ErrorCode = DAL_I2C_WRONG_START;
        }
        return DAL_TIMEOUT;
      }

      /* Send slave address */
      hi2c->Instance->DATA = I2C_7BIT_ADD_WRITE(DevAddress);

      /* Wait until ADDR or AF flag are set */
      /* Get tick */
      tickstart = DAL_GetTick();

      tmp1 = __DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR);
      tmp2 = __DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF);
      while ((hi2c->State != DAL_I2C_STATE_TIMEOUT) && (tmp1 == RESET) && (tmp2 == RESET))
      {
        if (((DAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          hi2c->State = DAL_I2C_STATE_TIMEOUT;
        }
        tmp1 = __DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR);
        tmp2 = __DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF);
      }

      hi2c->State = DAL_I2C_STATE_READY;

      /* Check if the ADDR flag has been set */
      if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR) == SET)
      {
        /* Generate Stop */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

        /* Clear ADDR Flag */
        __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

        /* Wait until BUSY flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != DAL_OK)
        {
          return DAL_ERROR;
        }

        hi2c->State = DAL_I2C_STATE_READY;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_OK;
      }
      else
      {
        /* Generate Stop */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

        /* Clear AF Flag */
        __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

        /* Wait until BUSY flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != DAL_OK)
        {
          return DAL_ERROR;
        }
      }

      /* Increment Trials */
      I2C_Trials++;
    }
    while (I2C_Trials < Trials);

    hi2c->State = DAL_I2C_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    return DAL_ERROR;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Sequential transmit in master I2C mode an amount of data in non-blocking mode with Interrupt.
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  __IO uint32_t Prev_State = 0x00U;
  __IO uint32_t count      = 0x00U;

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Check Busy Flag only if FIRST call of Master interface */
    if ((READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP) == I2C_CTRL1_STOP) || (XferOptions == I2C_FIRST_AND_LAST_FRAME) || (XferOptions == I2C_FIRST_FRAME))
    {
      /* Wait until BUSY flag is reset */
      count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
      do
      {
        count--;
        if (count == 0U)
        {
          hi2c->PreviousState       = I2C_STATE_NONE;
          hi2c->State               = DAL_I2C_STATE_READY;
          hi2c->Mode                = DAL_I2C_MODE_NONE;
          hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

          /* Process Unlocked */
          __DAL_UNLOCK(hi2c);

          return DAL_ERROR;
        }
      }
      while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = DAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;
    hi2c->Devaddress  = DevAddress;

    Prev_State = hi2c->PreviousState;

    /* If transfer direction not change and there is no request to start another frame, do not generate Restart Condition */
    /* Mean Previous state is same as current state */
    if ((Prev_State != I2C_STATE_MASTER_BUSY_TX) || (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 1))
    {
      /* Generate Start */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
    }

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
    to avoid the risk of I2C interrupt handle execution before current
    process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Sequential transmit in master I2C mode an amount of data in non-blocking mode with DMA.
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  __IO uint32_t Prev_State = 0x00U;
  __IO uint32_t count      = 0x00U;
  DAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Check Busy Flag only if FIRST call of Master interface */
    if ((READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP) == I2C_CTRL1_STOP) || (XferOptions == I2C_FIRST_AND_LAST_FRAME) || (XferOptions == I2C_FIRST_FRAME))
    {
      /* Wait until BUSY flag is reset */
      count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
      do
      {
        count--;
        if (count == 0U)
        {
          hi2c->PreviousState       = I2C_STATE_NONE;
          hi2c->State               = DAL_I2C_STATE_READY;
          hi2c->Mode                = DAL_I2C_MODE_NONE;
          hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

          /* Process Unlocked */
          __DAL_UNLOCK(hi2c);

          return DAL_ERROR;
        }
      }
      while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = DAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;
    hi2c->Devaddress  = DevAddress;

    Prev_State = hi2c->PreviousState;

    if (hi2c->XferSize > 0U)
    {
      if (hi2c->hdmatx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmatx->XferCpltCallback = I2C_DMAXferCplt;

        /* Set the DMA error callback */
        hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

        /* Set the unused DMA callbacks to NULL */
        hi2c->hdmatx->XferHalfCpltCallback = NULL;
        hi2c->hdmatx->XferAbortCallback = NULL;

        /* Enable the DMA stream */
        dmaxferstatus = DAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)hi2c->pBuffPtr, (uint32_t)&hi2c->Instance->DATA, hi2c->XferSize);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }

      if (dmaxferstatus == DAL_OK)
      {
        /* Enable Acknowledge */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

        /* If transfer direction not change and there is no request to start another frame, do not generate Restart Condition */
        /* Mean Previous state is same as current state */
        if ((Prev_State != I2C_STATE_MASTER_BUSY_TX) || (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 1))
        {
          /* Generate Start */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
        }

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
        to avoid the risk of I2C interrupt handle execution before current
        process unlock */

        /* If XferOptions is not associated to a new frame, mean no start bit is request, enable directly the DMA request */
        /* In other cases, DMA request is enabled after Slave address treatment in IRQHandler */
        if ((XferOptions == I2C_NEXT_FRAME) || (XferOptions == I2C_LAST_FRAME) || (XferOptions == I2C_LAST_FRAME_NO_STOP))
        {
          /* Enable DMA Request */
          SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);
        }

        /* Enable EVT and ERR interrupt */
        __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    else
    {
      /* Enable Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* If transfer direction not change and there is no request to start another frame, do not generate Restart Condition */
      /* Mean Previous state is same as current state */
      if ((Prev_State != I2C_STATE_MASTER_BUSY_TX) || (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 1))
      {
        /* Generate Start */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
      }

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
      to avoid the risk of I2C interrupt handle execution before current
      process unlock */

      /* Enable EVT, BUF and ERR interrupt */
      __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
    }

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Sequential receive in master I2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  __IO uint32_t Prev_State = 0x00U;
  __IO uint32_t count = 0U;
  uint32_t enableIT = (I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Check Busy Flag only if FIRST call of Master interface */
    if ((READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP) == I2C_CTRL1_STOP) || (XferOptions == I2C_FIRST_AND_LAST_FRAME) || (XferOptions == I2C_FIRST_FRAME))
    {
      /* Wait until BUSY flag is reset */
      count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
      do
      {
        count--;
        if (count == 0U)
        {
          hi2c->PreviousState       = I2C_STATE_NONE;
          hi2c->State               = DAL_I2C_STATE_READY;
          hi2c->Mode                = DAL_I2C_MODE_NONE;
          hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

          /* Process Unlocked */
          __DAL_UNLOCK(hi2c);

          return DAL_ERROR;
        }
      }
      while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = DAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;
    hi2c->Devaddress  = DevAddress;

    Prev_State = hi2c->PreviousState;

    if ((hi2c->XferCount == 2U) && ((XferOptions == I2C_LAST_FRAME) || (XferOptions == I2C_LAST_FRAME_NO_STOP)))
    {
      if (Prev_State == I2C_STATE_MASTER_BUSY_RX)
      {
        /* Disable Acknowledge */
        CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

        /* Enable Pos */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

        /* Remove Enabling of IT_BUF, mean RXNE treatment, treat the 2 bytes through BTF */
        enableIT &= ~I2C_IT_BUF;
      }
      else
      {
        /* Enable Acknowledge */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
      }
    }
    else
    {
      /* Enable Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
    }

    /* If transfer direction not change and there is no request to start another frame, do not generate Restart Condition */
    /* Mean Previous state is same as current state */
    if ((Prev_State != I2C_STATE_MASTER_BUSY_RX) || (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 1))
    {
      /* Generate Start */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
    }

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
    to avoid the risk of I2C interrupt handle execution before current
    process unlock */

    /* Enable interrupts */
    __DAL_I2C_ENABLE_IT(hi2c, enableIT);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Sequential receive in master mode an amount of data in non-blocking mode with DMA
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  __IO uint32_t Prev_State = 0x00U;
  __IO uint32_t count = 0U;
  uint32_t enableIT = (I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
  DAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    /* Check Busy Flag only if FIRST call of Master interface */
    if ((READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP) == I2C_CTRL1_STOP) || (XferOptions == I2C_FIRST_AND_LAST_FRAME) || (XferOptions == I2C_FIRST_FRAME))
    {
      /* Wait until BUSY flag is reset */
      count = I2C_TIMEOUT_BUSY_FLAG * (SystemCoreClock / 25U / 1000U);
      do
      {
        count--;
        if (count == 0U)
        {
          hi2c->PreviousState       = I2C_STATE_NONE;
          hi2c->State               = DAL_I2C_STATE_READY;
          hi2c->Mode                = DAL_I2C_MODE_NONE;
          hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

          /* Process Unlocked */
          __DAL_UNLOCK(hi2c);

          return DAL_ERROR;
        }
      }
      while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET);
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    /* Clear Last DMA bit */
    CLEAR_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_LTCFG);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = DAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;
    hi2c->Devaddress  = DevAddress;

    Prev_State = hi2c->PreviousState;

    if (hi2c->XferSize > 0U)
    {
      if ((hi2c->XferCount == 2U) && ((XferOptions == I2C_LAST_FRAME) || (XferOptions == I2C_LAST_FRAME_NO_STOP)))
      {
        if (Prev_State == I2C_STATE_MASTER_BUSY_RX)
        {
          /* Disable Acknowledge */
          CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

          /* Enable Pos */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

          /* Enable Last DMA bit */
          SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_LTCFG);
        }
        else
        {
          /* Enable Acknowledge */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
        }
      }
      else
      {
        /* Enable Acknowledge */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

        if ((XferOptions == I2C_LAST_FRAME) || (XferOptions == I2C_OTHER_AND_LAST_FRAME) || (XferOptions == I2C_LAST_FRAME_NO_STOP))
        {
          /* Enable Last DMA bit */
          SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_LTCFG);
        }
      }
      if (hi2c->hdmarx != NULL)
      {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmarx->XferCpltCallback = I2C_DMAXferCplt;

        /* Set the DMA error callback */
        hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

        /* Set the unused DMA callbacks to NULL */
        hi2c->hdmarx->XferHalfCpltCallback = NULL;
        hi2c->hdmarx->XferAbortCallback = NULL;

        /* Enable the DMA stream */
        dmaxferstatus = DAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->DATA, (uint32_t)hi2c->pBuffPtr, hi2c->XferSize);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA_PARAM;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
      if (dmaxferstatus == DAL_OK)
      {
        /* If transfer direction not change and there is no request to start another frame, do not generate Restart Condition */
        /* Mean Previous state is same as current state */
        if ((Prev_State != I2C_STATE_MASTER_BUSY_RX) || (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 1))
        {
          /* Generate Start */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

          /* Update interrupt for only EVT and ERR */
          enableIT = (I2C_IT_EVT | I2C_IT_ERR);
        }
        else
        {
          /* Update interrupt for only ERR */
          enableIT = I2C_IT_ERR;
        }

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
        to avoid the risk of I2C interrupt handle execution before current
        process unlock */

        /* If XferOptions is not associated to a new frame, mean no start bit is request, enable directly the DMA request */
        /* In other cases, DMA request is enabled after Slave address treatment in IRQHandler */
        if ((XferOptions == I2C_NEXT_FRAME) || (XferOptions == I2C_LAST_FRAME) || (XferOptions == I2C_LAST_FRAME_NO_STOP))
        {
          /* Enable DMA Request */
          SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);
        }

        /* Enable EVT and ERR interrupt */
        __DAL_I2C_ENABLE_IT(hi2c, enableIT);
      }
      else
      {
        /* Update I2C state */
        hi2c->State     = DAL_I2C_STATE_READY;
        hi2c->Mode      = DAL_I2C_MODE_NONE;

        /* Update I2C error code */
        hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
    else
    {
      /* Enable Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* If transfer direction not change and there is no request to start another frame, do not generate Restart Condition */
      /* Mean Previous state is same as current state */
      if ((Prev_State != I2C_STATE_MASTER_BUSY_RX) || (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 1))
      {
        /* Generate Start */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
      }

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
      to avoid the risk of I2C interrupt handle execution before current
      process unlock */

      /* Enable interrupts */
      __DAL_I2C_ENABLE_IT(hi2c, enableIT);
    }
    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Sequential transmit in slave mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (((uint32_t)hi2c->State & (uint32_t)DAL_I2C_STATE_LISTEN) == (uint32_t)DAL_I2C_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX_LISTEN;
    hi2c->Mode      = DAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;

    /* Clear ADDR flag */
    __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Sequential transmit in slave mode an amount of data in non-blocking mode with DMA
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Slave_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  DAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (((uint32_t)hi2c->State & (uint32_t)DAL_I2C_STATE_LISTEN) == (uint32_t)DAL_I2C_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

    /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
    /* and then toggle the DAL slave RX state to TX state */
    if (hi2c->State == DAL_I2C_STATE_BUSY_RX_LISTEN)
    {
      if ((hi2c->Instance->CTRL2 & I2C_CTRL2_DMAEN) == I2C_CTRL2_DMAEN)
      {
        /* Abort DMA Xfer if any */
        if (hi2c->hdmarx != NULL)
        {
          CLEAR_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

          /* Set the I2C DMA Abort callback :
           will lead to call DAL_I2C_ErrorCallback() at end of DMA abort procedure */
          hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

          /* Abort DMA RX */
          if (DAL_DMA_Abort_IT(hi2c->hdmarx) != DAL_OK)
          {
            /* Call Directly XferAbortCallback function in case of error */
            hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
          }
        }
      }
    }
    else if (hi2c->State == DAL_I2C_STATE_BUSY_TX_LISTEN)
    {
      if ((hi2c->Instance->CTRL2 & I2C_CTRL2_DMAEN) == I2C_CTRL2_DMAEN)
      {
        CLEAR_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

        /* Abort DMA Xfer if any */
        if (hi2c->hdmatx != NULL)
        {
          /* Set the I2C DMA Abort callback :
           will lead to call DAL_I2C_ErrorCallback() at end of DMA abort procedure */
          hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

          /* Abort DMA TX */
          if (DAL_DMA_Abort_IT(hi2c->hdmatx) != DAL_OK)
          {
            /* Call Directly XferAbortCallback function in case of error */
            hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
          }
        }
      }
    }
    else
    {
      /* Nothing to do */
    }

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_TX_LISTEN;
    hi2c->Mode      = DAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;

    if (hi2c->hdmatx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmatx->XferCpltCallback = I2C_DMAXferCplt;

      /* Set the DMA error callback */
      hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hi2c->hdmatx->XferHalfCpltCallback = NULL;
      hi2c->hdmatx->XferAbortCallback = NULL;

      /* Enable the DMA stream */
      dmaxferstatus = DAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)hi2c->pBuffPtr, (uint32_t)&hi2c->Instance->DATA, hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = DAL_I2C_STATE_LISTEN;
      hi2c->Mode      = DAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= DAL_I2C_ERROR_DMA_PARAM;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }

    if (dmaxferstatus == DAL_OK)
    {
      /* Enable Address Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      /* Note : The I2C interrupts must be enabled after unlocking current process
      to avoid the risk of I2C interrupt handle execution before current
      process unlock */
      /* Enable EVT and ERR interrupt */
      __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

      /* Enable DMA Request */
      hi2c->Instance->CTRL2 |= I2C_CTRL2_DMAEN;

      return DAL_OK;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = DAL_I2C_STATE_READY;
      hi2c->Mode      = DAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Sequential receive in slave mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (((uint32_t)hi2c->State & (uint32_t)DAL_I2C_STATE_LISTEN) == (uint32_t)DAL_I2C_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX_LISTEN;
    hi2c->Mode      = DAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;

    /* Clear ADDR flag */
    __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable EVT, BUF and ERR interrupt */
    __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Sequential receive in slave mode an amount of data in non-blocking mode with DMA
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XferOptions_definition
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Slave_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
  DAL_StatusTypeDef dmaxferstatus;

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

  if (((uint32_t)hi2c->State & (uint32_t)DAL_I2C_STATE_LISTEN) == (uint32_t)DAL_I2C_STATE_LISTEN)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hi2c);

    /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

    /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
    /* and then toggle the DAL slave RX state to TX state */
    if (hi2c->State == DAL_I2C_STATE_BUSY_RX_LISTEN)
    {
      if ((hi2c->Instance->CTRL2 & I2C_CTRL2_DMAEN) == I2C_CTRL2_DMAEN)
      {
        /* Abort DMA Xfer if any */
        if (hi2c->hdmarx != NULL)
        {
          CLEAR_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

          /* Set the I2C DMA Abort callback :
           will lead to call DAL_I2C_ErrorCallback() at end of DMA abort procedure */
          hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

          /* Abort DMA RX */
          if (DAL_DMA_Abort_IT(hi2c->hdmarx) != DAL_OK)
          {
            /* Call Directly XferAbortCallback function in case of error */
            hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
          }
        }
      }
    }
    else if (hi2c->State == DAL_I2C_STATE_BUSY_TX_LISTEN)
    {
      if ((hi2c->Instance->CTRL2 & I2C_CTRL2_DMAEN) == I2C_CTRL2_DMAEN)
      {
        CLEAR_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

        /* Abort DMA Xfer if any */
        if (hi2c->hdmatx != NULL)
        {
          /* Set the I2C DMA Abort callback :
           will lead to call DAL_I2C_ErrorCallback() at end of DMA abort procedure */
          hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

          /* Abort DMA TX */
          if (DAL_DMA_Abort_IT(hi2c->hdmatx) != DAL_OK)
          {
            /* Call Directly XferAbortCallback function in case of error */
            hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
          }
        }
      }
    }
    else
    {
      /* Nothing to do */
    }

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);

    hi2c->State     = DAL_I2C_STATE_BUSY_RX_LISTEN;
    hi2c->Mode      = DAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = XferOptions;

    if (hi2c->hdmarx != NULL)
    {
      /* Set the I2C DMA transfer complete callback */
      hi2c->hdmarx->XferCpltCallback = I2C_DMAXferCplt;

      /* Set the DMA error callback */
      hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

      /* Set the unused DMA callbacks to NULL */
      hi2c->hdmarx->XferHalfCpltCallback = NULL;
      hi2c->hdmarx->XferAbortCallback = NULL;

      /* Enable the DMA stream */
      dmaxferstatus = DAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->DATA, (uint32_t)hi2c->pBuffPtr, hi2c->XferSize);
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = DAL_I2C_STATE_LISTEN;
      hi2c->Mode      = DAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= DAL_I2C_ERROR_DMA_PARAM;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }

    if (dmaxferstatus == DAL_OK)
    {
      /* Enable Address Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      /* Enable DMA Request */
      SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

      /* Note : The I2C interrupts must be enabled after unlocking current process
      to avoid the risk of I2C interrupt handle execution before current
      process unlock */
      /* Enable EVT and ERR interrupt */
      __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

      return DAL_OK;
    }
    else
    {
      /* Update I2C state */
      hi2c->State     = DAL_I2C_STATE_READY;
      hi2c->Mode      = DAL_I2C_MODE_NONE;

      /* Update I2C error code */
      hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Enable the Address listen mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    hi2c->State = DAL_I2C_STATE_LISTEN;

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CTRL1 & I2C_CTRL1_I2CEN) != I2C_CTRL1_I2CEN)
    {
      /* Enable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);
    }

    /* Enable Address Acknowledge */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Enable EVT and ERR interrupt */
    __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Disable the Address listen mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of tmp to prevent undefined behavior of volatile usage */
  uint32_t tmp;

  /* Disable Address listen mode only if a transfer is not ongoing */
  if (hi2c->State == DAL_I2C_STATE_LISTEN)
  {
    tmp = (uint32_t)(hi2c->State) & I2C_STATE_MSK;
    hi2c->PreviousState = tmp | (uint32_t)(hi2c->Mode);
    hi2c->State = DAL_I2C_STATE_READY;
    hi2c->Mode = DAL_I2C_MODE_NONE;

    /* Disable Address Acknowledge */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Disable EVT and ERR interrupt */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Abort a master I2C IT or DMA process communication with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  DAL_I2C_ModeTypeDef CurrentMode   = hi2c->Mode;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(DevAddress);

  /* Abort Master transfer during Receive or Transmit process    */
  if ((__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET) && (CurrentMode == DAL_I2C_MODE_MASTER))
  {
    /* Process Locked */
    __DAL_LOCK(hi2c);

    hi2c->PreviousState = I2C_STATE_NONE;
    hi2c->State = DAL_I2C_STATE_ABORT;

    /* Disable Acknowledge */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    /* Generate Stop */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

    hi2c->XferCount = 0U;

    /* Disable EVT, BUF and ERR interrupt */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    I2C_ITError(hi2c);

    return DAL_OK;
  }
  else
  {
    /* Wrong usage of abort function */
    /* This function should be used only in case of abort monitored by master device */
    /* Or periphal is not in busy state, mean there is no active sequence to be abort */
    return DAL_ERROR;
  }
}

/**
  * @}
  */

/** @defgroup I2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */

/**
  * @brief  This function handles I2C event interrupt request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void DAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c)
{
  uint32_t sr1itflags;
  uint32_t sr2itflags               = 0U;
  uint32_t itsources                = READ_REG(hi2c->Instance->CTRL2);
  uint32_t CurrentXferOptions       = hi2c->XferOptions;
  DAL_I2C_ModeTypeDef CurrentMode   = hi2c->Mode;
  DAL_I2C_StateTypeDef CurrentState = hi2c->State;

  /* Master or Memory mode selected */
  if ((CurrentMode == DAL_I2C_MODE_MASTER) || (CurrentMode == DAL_I2C_MODE_MEM))
  {
    sr2itflags   = READ_REG(hi2c->Instance->STS2);
    sr1itflags   = READ_REG(hi2c->Instance->STS1);

    /* Exit IRQ event until Start Bit detected in case of Other frame requested */
    if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_SB) == RESET) && (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(CurrentXferOptions) == 1U))
    {
      return;
    }

    /* SB Set ----------------------------------------------------------------*/
    if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_SB) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_EVT) != RESET))
    {
      /* Convert OTHER_xxx XferOptions if any */
      I2C_ConvertOtherXferOptions(hi2c);

      I2C_Master_SB(hi2c);
    }
    /* ADD10 Set -------------------------------------------------------------*/
    else if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_ADD10) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_EVT) != RESET))
    {
      I2C_Master_ADD10(hi2c);
    }
    /* ADDR Set --------------------------------------------------------------*/
    else if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_ADDR) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_EVT) != RESET))
    {
      I2C_Master_ADDR(hi2c);
    }
    /* I2C in mode Transmitter -----------------------------------------------*/
    else if (I2C_CHECK_FLAG(sr2itflags, I2C_FLAG_TRA) != RESET)
    {
      /* Do not check buffer and BTF flag if a Xfer DMA is on going */
      if (READ_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN) != I2C_CTRL2_DMAEN)
      {
        /* TXE set and BTF reset -----------------------------------------------*/
        if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_TXE) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_BUF) != RESET) && (I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_BTF) == RESET))
        {
          I2C_MasterTransmit_TXE(hi2c);
        }
        /* BTF set -------------------------------------------------------------*/
        else if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_BTF) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_EVT) != RESET))
        {
          if (CurrentState == DAL_I2C_STATE_BUSY_TX)
          {
            I2C_MasterTransmit_BTF(hi2c);
          }
          else /* DAL_I2C_MODE_MEM */
          {
            if (CurrentMode == DAL_I2C_MODE_MEM)
            {
              I2C_MemoryTransmit_TXE_BTF(hi2c);
            }
          }
        }
        else
        {
          /* Do nothing */
        }
      }
    }
    /* I2C in mode Receiver --------------------------------------------------*/
    else
    {
      /* Do not check buffer and BTF flag if a Xfer DMA is on going */
      if (READ_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN) != I2C_CTRL2_DMAEN)
      {
        /* RXNE set and BTF reset -----------------------------------------------*/
        if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_RXNE) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_BUF) != RESET) && (I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_BTF) == RESET))
        {
          I2C_MasterReceive_RXNE(hi2c);
        }
        /* BTF set -------------------------------------------------------------*/
        else if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_BTF) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_EVT) != RESET))
        {
          I2C_MasterReceive_BTF(hi2c);
        }
        else
        {
          /* Do nothing */
        }
      }
    }
  }
  /* Slave mode selected */
  else
  {
    /* If an error is detected, read only STS1 register to prevent */
    /* a clear of ADDR flags by reading STS2 after reading STS1 in Error treatment */
    if (hi2c->ErrorCode != DAL_I2C_ERROR_NONE)
    {
      sr1itflags   = READ_REG(hi2c->Instance->STS1);
    }
    else
    {
      sr2itflags   = READ_REG(hi2c->Instance->STS2);
      sr1itflags   = READ_REG(hi2c->Instance->STS1);
    }

    /* ADDR set --------------------------------------------------------------*/
    if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_ADDR) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_EVT) != RESET))
    {
      /* Now time to read STS2, this will clear ADDR flag automatically */
      if (hi2c->ErrorCode != DAL_I2C_ERROR_NONE)
      {
        sr2itflags   = READ_REG(hi2c->Instance->STS2);
      }
      I2C_Slave_ADDR(hi2c, sr2itflags);
    }
    /* STOPF set --------------------------------------------------------------*/
    else if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_STOPF) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_EVT) != RESET))
    {
      I2C_Slave_STOPF(hi2c);
    }
    /* I2C in mode Transmitter -----------------------------------------------*/
    else if ((CurrentState == DAL_I2C_STATE_BUSY_TX) || (CurrentState == DAL_I2C_STATE_BUSY_TX_LISTEN))
    {
      /* TXE set and BTF reset -----------------------------------------------*/
      if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_TXE) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_BUF) != RESET) && (I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_BTF) == RESET))
      {
        I2C_SlaveTransmit_TXE(hi2c);
      }
      /* BTF set -------------------------------------------------------------*/
      else if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_BTF) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_EVT) != RESET))
      {
        I2C_SlaveTransmit_BTF(hi2c);
      }
      else
      {
        /* Do nothing */
      }
    }
    /* I2C in mode Receiver --------------------------------------------------*/
    else
    {
      /* RXNE set and BTF reset ----------------------------------------------*/
      if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_RXNE) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_BUF) != RESET) && (I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_BTF) == RESET))
      {
        I2C_SlaveReceive_RXNE(hi2c);
      }
      /* BTF set -------------------------------------------------------------*/
      else if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_BTF) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_EVT) != RESET))
      {
        I2C_SlaveReceive_BTF(hi2c);
      }
      else
      {
        /* Do nothing */
      }
    }
  }
}

/**
  * @brief  This function handles I2C error interrupt request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void DAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c)
{
  DAL_I2C_ModeTypeDef tmp1;
  uint32_t tmp2;
  DAL_I2C_StateTypeDef tmp3;
  uint32_t tmp4;
  uint32_t sr1itflags = READ_REG(hi2c->Instance->STS1);
  uint32_t itsources  = READ_REG(hi2c->Instance->CTRL2);
  uint32_t error      = DAL_I2C_ERROR_NONE;
  DAL_I2C_ModeTypeDef CurrentMode   = hi2c->Mode;

  /* I2C Bus error interrupt occurred ----------------------------------------*/
  if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_BERR) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_ERR) != RESET))
  {
    error |= DAL_I2C_ERROR_BERR;

    /* Clear BERR flag */
    __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR);
  }

  /* I2C Arbitration Lost error interrupt occurred ---------------------------*/
  if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_ARLO) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_ERR) != RESET))
  {
    error |= DAL_I2C_ERROR_ARLO;

    /* Clear ARLO flag */
    __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ARLO);
  }

  /* I2C Acknowledge failure error interrupt occurred ------------------------*/
  if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_AF) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_ERR) != RESET))
  {
    tmp1 = CurrentMode;
    tmp2 = hi2c->XferCount;
    tmp3 = hi2c->State;
    tmp4 = hi2c->PreviousState;
    if ((tmp1 == DAL_I2C_MODE_SLAVE) && (tmp2 == 0U) && \
        ((tmp3 == DAL_I2C_STATE_BUSY_TX) || (tmp3 == DAL_I2C_STATE_BUSY_TX_LISTEN) || \
         ((tmp3 == DAL_I2C_STATE_LISTEN) && (tmp4 == I2C_STATE_SLAVE_BUSY_TX))))
    {
      I2C_Slave_AF(hi2c);
    }
    else
    {
      /* Clear AF flag */
      __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

      error |= DAL_I2C_ERROR_AF;

      /* Do not generate a STOP in case of Slave receive non acknowledge during transfer (mean not at the end of transfer) */
      if ((CurrentMode == DAL_I2C_MODE_MASTER) || (CurrentMode == DAL_I2C_MODE_MEM))
      {
        /* Generate Stop */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
      }
    }
  }

  /* I2C Over-Run/Under-Run interrupt occurred -------------------------------*/
  if ((I2C_CHECK_FLAG(sr1itflags, I2C_FLAG_OVR) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_ERR) != RESET))
  {
    error |= DAL_I2C_ERROR_OVR;
    /* Clear OVR flag */
    __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_OVR);
  }

  /* Call the Error Callback in case of Error detected -----------------------*/
  if (error != DAL_I2C_ERROR_NONE)
  {
    hi2c->ErrorCode |= error;
    I2C_ITError(hi2c);
  }
}

/**
  * @brief  Master Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_MasterTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Master Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_MasterRxCpltCallback could be implemented in the user file
   */
}

/** @brief  Slave Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_SlaveTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_SlaveRxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection Master request Transfer Direction (Write/Read), value of @ref I2C_XferDirection_definition
  * @param  AddrMatchCode Address Match Code
  * @retval None
  */
__weak void DAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);
  UNUSED(TransferDirection);
  UNUSED(AddrMatchCode);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_AddrCallback() could be implemented in the user file
   */
}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_ListenCpltCallback() could be implemented in the user file
  */
}

/**
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_MemTxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_MemRxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  I2C error callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_ErrorCallback could be implemented in the user file
   */
}

/**
  * @brief  I2C abort callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void DAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2C_AbortCpltCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
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
  * @brief  Return the I2C handle state.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval DAL state
  */
DAL_I2C_StateTypeDef DAL_I2C_GetState(I2C_HandleTypeDef *hi2c)
{
  /* Return I2C handle state */
  return hi2c->State;
}

/**
  * @brief  Returns the I2C Master, Slave, Memory or no mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval DAL mode
  */
DAL_I2C_ModeTypeDef DAL_I2C_GetMode(I2C_HandleTypeDef *hi2c)
{
  return hi2c->Mode;
}

/**
  * @brief  Return the I2C error code.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
  * @retval I2C Error Code
  */
uint32_t DAL_I2C_GetError(I2C_HandleTypeDef *hi2c)
{
  return hi2c->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup I2C_Private_Functions
  * @{
  */

/**
  * @brief  Handle TXE flag for Master
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_MasterTransmit_TXE(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  DAL_I2C_StateTypeDef CurrentState = hi2c->State;
  DAL_I2C_ModeTypeDef CurrentMode   = hi2c->Mode;
  uint32_t CurrentXferOptions       = hi2c->XferOptions;

  if ((hi2c->XferSize == 0U) && (CurrentState == DAL_I2C_STATE_BUSY_TX))
  {
    /* Call TxCpltCallback() directly if no stop mode is set */
    if ((CurrentXferOptions != I2C_FIRST_AND_LAST_FRAME) && (CurrentXferOptions != I2C_LAST_FRAME) && (CurrentXferOptions != I2C_NO_OPTION_FRAME))
    {
      __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

      hi2c->PreviousState = I2C_STATE_MASTER_BUSY_TX;
      hi2c->Mode = DAL_I2C_MODE_NONE;
      hi2c->State = DAL_I2C_STATE_READY;

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->MasterTxCpltCallback(hi2c);
#else
      DAL_I2C_MasterTxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }
    else /* Generate Stop condition then Call TxCpltCallback() */
    {
      /* Disable EVT, BUF and ERR interrupt */
      __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

      hi2c->PreviousState = I2C_STATE_NONE;
      hi2c->State = DAL_I2C_STATE_READY;

      if (hi2c->Mode == DAL_I2C_MODE_MEM)
      {
        hi2c->Mode = DAL_I2C_MODE_NONE;
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->MemTxCpltCallback(hi2c);
#else
        DAL_I2C_MemTxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
      }
      else
      {
        hi2c->Mode = DAL_I2C_MODE_NONE;
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->MasterTxCpltCallback(hi2c);
#else
        DAL_I2C_MasterTxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
      }
    }
  }
  else if ((CurrentState == DAL_I2C_STATE_BUSY_TX) || \
           ((CurrentMode == DAL_I2C_MODE_MEM) && (CurrentState == DAL_I2C_STATE_BUSY_RX)))
  {
    if (hi2c->XferCount == 0U)
    {
      /* Disable BUF interrupt */
      __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_BUF);
    }
    else
    {
      if (hi2c->Mode == DAL_I2C_MODE_MEM)
      {
        I2C_MemoryTransmit_TXE_BTF(hi2c);
      }
      else
      {
        /* Write data to DATA */
        hi2c->Instance->DATA = *hi2c->pBuffPtr;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        /* Update counter */
        hi2c->XferCount--;
      }
    }
  }
  else
  {
    /* Do nothing */
  }
}

/**
  * @brief  Handle BTF flag for Master transmitter
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_MasterTransmit_BTF(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentXferOptions = hi2c->XferOptions;

  if (hi2c->State == DAL_I2C_STATE_BUSY_TX)
  {
    if (hi2c->XferCount != 0U)
    {
      /* Write data to DATA */
      hi2c->Instance->DATA = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferCount--;
    }
    else
    {
      /* Call TxCpltCallback() directly if no stop mode is set */
      if ((CurrentXferOptions != I2C_FIRST_AND_LAST_FRAME) && (CurrentXferOptions != I2C_LAST_FRAME) && (CurrentXferOptions != I2C_NO_OPTION_FRAME))
      {
        __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

        hi2c->PreviousState = I2C_STATE_MASTER_BUSY_TX;
        hi2c->Mode = DAL_I2C_MODE_NONE;
        hi2c->State = DAL_I2C_STATE_READY;

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->MasterTxCpltCallback(hi2c);
#else
        DAL_I2C_MasterTxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
      }
      else /* Generate Stop condition then Call TxCpltCallback() */
      {
        /* Disable EVT, BUF and ERR interrupt */
        __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

        /* Generate Stop */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

        hi2c->PreviousState = I2C_STATE_NONE;
        hi2c->State = DAL_I2C_STATE_READY;
        if (hi2c->Mode == DAL_I2C_MODE_MEM)
        {
          hi2c->Mode = DAL_I2C_MODE_NONE;
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
          hi2c->MemTxCpltCallback(hi2c);
#else
          DAL_I2C_MemTxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
        }
        else
        {
          hi2c->Mode = DAL_I2C_MODE_NONE;

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
          hi2c->MasterTxCpltCallback(hi2c);
#else
          DAL_I2C_MasterTxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
        }
      }
    }
  }
  else
  {
    /* Do nothing */
  }
}

/**
  * @brief  Handle TXE and BTF flag for Memory transmitter
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_MemoryTransmit_TXE_BTF(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  DAL_I2C_StateTypeDef CurrentState = hi2c->State;

  if (hi2c->EventCount == 0U)
  {
    /* If Memory address size is 8Bit */
    if (hi2c->MemaddSize == I2C_MEMADD_SIZE_8BIT)
    {
      /* Send Memory Address */
      hi2c->Instance->DATA = I2C_MEM_ADD_LSB(hi2c->Memaddress);

      hi2c->EventCount += 2U;
    }
    /* If Memory address size is 16Bit */
    else
    {
      /* Send MSB of Memory Address */
      hi2c->Instance->DATA = I2C_MEM_ADD_MSB(hi2c->Memaddress);

      hi2c->EventCount++;
    }
  }
  else if (hi2c->EventCount == 1U)
  {
    /* Send LSB of Memory Address */
    hi2c->Instance->DATA = I2C_MEM_ADD_LSB(hi2c->Memaddress);

    hi2c->EventCount++;
  }
  else if (hi2c->EventCount == 2U)
  {
    if (CurrentState == DAL_I2C_STATE_BUSY_RX)
    {
      /* Generate Restart */
      hi2c->Instance->CTRL1 |= I2C_CTRL1_START;

      hi2c->EventCount++;
    }
    else if ((hi2c->XferCount > 0U) && (CurrentState == DAL_I2C_STATE_BUSY_TX))
    {
      /* Write data to DATA */
      hi2c->Instance->DATA = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferCount--;
    }
    else if ((hi2c->XferCount == 0U) && (CurrentState == DAL_I2C_STATE_BUSY_TX))
    {
      /* Generate Stop condition then Call TxCpltCallback() */
      /* Disable EVT, BUF and ERR interrupt */
      __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

      hi2c->PreviousState = I2C_STATE_NONE;
      hi2c->State = DAL_I2C_STATE_READY;
      hi2c->Mode = DAL_I2C_MODE_NONE;
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->MemTxCpltCallback(hi2c);
#else
      DAL_I2C_MemTxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }
    else
    {
      /* Do nothing */
    }
  }
  else
  {
    /* Do nothing */
  }
}

/**
  * @brief  Handle RXNE flag for Master
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_MasterReceive_RXNE(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->State == DAL_I2C_STATE_BUSY_RX)
  {
    uint32_t tmp;

    tmp = hi2c->XferCount;
    if (tmp > 3U)
    {
      /* Read data from DATA */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferCount--;

      if (hi2c->XferCount == (uint16_t)3)
      {
        /* Disable BUF interrupt, this help to treat correctly the last 4 bytes
        on BTF subroutine */
        /* Disable BUF interrupt */
        __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_BUF);
      }
    }
    else if ((hi2c->XferOptions != I2C_FIRST_AND_NEXT_FRAME) && ((tmp == 1U) || (tmp == 0U)))
    {
      if (I2C_WaitOnSTOPRequestThroughIT(hi2c) == DAL_OK)
      {
        /* Disable Acknowledge */
        CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

        /* Disable EVT, BUF and ERR interrupt */
        __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

        /* Read data from DATA */
        *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        /* Update counter */
        hi2c->XferCount--;

        hi2c->State = DAL_I2C_STATE_READY;

        if (hi2c->Mode == DAL_I2C_MODE_MEM)
        {
          hi2c->Mode = DAL_I2C_MODE_NONE;
          hi2c->PreviousState = I2C_STATE_NONE;

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
          hi2c->MemRxCpltCallback(hi2c);
#else
          DAL_I2C_MemRxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
        }
        else
        {
          hi2c->Mode = DAL_I2C_MODE_NONE;
          hi2c->PreviousState = I2C_STATE_MASTER_BUSY_RX;

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
          hi2c->MasterRxCpltCallback(hi2c);
#else
          DAL_I2C_MasterRxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
        }
      }
      else
      {
        /* Disable EVT, BUF and ERR interrupt */
        __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

        /* Read data from DATA */
        *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        /* Update counter */
        hi2c->XferCount--;

        hi2c->State = DAL_I2C_STATE_READY;
        hi2c->Mode = DAL_I2C_MODE_NONE;

        /* Call user error callback */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->ErrorCallback(hi2c);
#else
        DAL_I2C_ErrorCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
      }
    }
    else
    {
      /* Disable BUF interrupt, this help to treat correctly the last 2 bytes
         on BTF subroutine if there is a reception delay between N-1 and N byte */
      __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_BUF);
    }
  }
}

/**
  * @brief  Handle BTF flag for Master receiver
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_MasterReceive_BTF(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  uint32_t CurrentXferOptions = hi2c->XferOptions;

  if (hi2c->XferCount == 4U)
  {
    /* Disable BUF interrupt, this help to treat correctly the last 2 bytes
       on BTF subroutine if there is a reception delay between N-1 and N byte */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_BUF);

    /* Read data from DATA */
    *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    /* Update counter */
    hi2c->XferCount--;
  }
  else if (hi2c->XferCount == 3U)
  {
    /* Disable BUF interrupt, this help to treat correctly the last 2 bytes
       on BTF subroutine if there is a reception delay between N-1 and N byte */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_BUF);

    if ((CurrentXferOptions != I2C_NEXT_FRAME) && (CurrentXferOptions != I2C_FIRST_AND_NEXT_FRAME))
    {
      /* Disable Acknowledge */
      CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
    }

    /* Read data from DATA */
    *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    /* Update counter */
    hi2c->XferCount--;
  }
  else if (hi2c->XferCount == 2U)
  {
    /* Prepare next transfer or stop current transfer */
    if ((CurrentXferOptions == I2C_FIRST_FRAME) || (CurrentXferOptions == I2C_LAST_FRAME_NO_STOP))
    {
      /* Disable Acknowledge */
      CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
    }
    else if ((CurrentXferOptions == I2C_NEXT_FRAME) || (CurrentXferOptions == I2C_FIRST_AND_NEXT_FRAME))
    {
      /* Enable Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
    }
    else if (CurrentXferOptions != I2C_LAST_FRAME_NO_STOP)
    {
      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
    }
    else
    {
      /* Do nothing */
    }

    /* Read data from DATA */
    *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    /* Update counter */
    hi2c->XferCount--;

    /* Read data from DATA */
    *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    /* Update counter */
    hi2c->XferCount--;

    /* Disable EVT and ERR interrupt */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

    hi2c->State = DAL_I2C_STATE_READY;
    if (hi2c->Mode == DAL_I2C_MODE_MEM)
    {
      hi2c->Mode = DAL_I2C_MODE_NONE;
      hi2c->PreviousState = I2C_STATE_NONE;
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->MemRxCpltCallback(hi2c);
#else
      DAL_I2C_MemRxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }
    else
    {
      hi2c->Mode = DAL_I2C_MODE_NONE;
      hi2c->PreviousState = I2C_STATE_MASTER_BUSY_RX;
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->MasterRxCpltCallback(hi2c);
#else
      DAL_I2C_MasterRxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }
  }
  else
  {
    /* Read data from DATA */
    *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    /* Update counter */
    hi2c->XferCount--;
  }
}

/**
  * @brief  Handle SB flag for Master
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_Master_SB(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Mode == DAL_I2C_MODE_MEM)
  {
    if (hi2c->EventCount == 0U)
    {
      /* Send slave address */
      hi2c->Instance->DATA = I2C_7BIT_ADD_WRITE(hi2c->Devaddress);
    }
    else
    {
      hi2c->Instance->DATA = I2C_7BIT_ADD_READ(hi2c->Devaddress);
    }
  }
  else
  {
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
      /* Send slave 7 Bits address */
      if (hi2c->State == DAL_I2C_STATE_BUSY_TX)
      {
        hi2c->Instance->DATA = I2C_7BIT_ADD_WRITE(hi2c->Devaddress);
      }
      else
      {
        hi2c->Instance->DATA = I2C_7BIT_ADD_READ(hi2c->Devaddress);
      }

      if (((hi2c->hdmatx != NULL) && (hi2c->hdmatx->XferCpltCallback != NULL))
          || ((hi2c->hdmarx != NULL) && (hi2c->hdmarx->XferCpltCallback != NULL)))
      {
        /* Enable DMA Request */
        SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);
      }
    }
    else
    {
      if (hi2c->EventCount == 0U)
      {
        /* Send header of slave address */
        hi2c->Instance->DATA = I2C_10BIT_HEADER_WRITE(hi2c->Devaddress);
      }
      else if (hi2c->EventCount == 1U)
      {
        /* Send header of slave address */
        hi2c->Instance->DATA = I2C_10BIT_HEADER_READ(hi2c->Devaddress);
      }
      else
      {
        /* Do nothing */
      }
    }
  }
}

/**
  * @brief  Handle ADD10 flag for Master
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_Master_ADD10(I2C_HandleTypeDef *hi2c)
{
  /* Send slave address */
  hi2c->Instance->DATA = I2C_10BIT_ADDRESS(hi2c->Devaddress);

  if (((hi2c->hdmatx != NULL) && (hi2c->hdmatx->XferCpltCallback != NULL))
      || ((hi2c->hdmarx != NULL) && (hi2c->hdmarx->XferCpltCallback != NULL)))
  {
    /* Enable DMA Request */
    SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);
  }
}

/**
  * @brief  Handle ADDR flag for Master
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_Master_ADDR(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  DAL_I2C_ModeTypeDef CurrentMode       = hi2c->Mode;
  uint32_t CurrentXferOptions           = hi2c->XferOptions;
  uint32_t Prev_State                   = hi2c->PreviousState;

  if (hi2c->State == DAL_I2C_STATE_BUSY_RX)
  {
    if ((hi2c->EventCount == 0U) && (CurrentMode == DAL_I2C_MODE_MEM))
    {
      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }
    else if ((hi2c->EventCount == 0U) && (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT))
    {
      /* Clear ADDR flag */
      __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

      /* Generate Restart */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

      hi2c->EventCount++;
    }
    else
    {
      if (hi2c->XferCount == 0U)
      {
        /* Clear ADDR flag */
        __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

        /* Generate Stop */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
      }
      else if (hi2c->XferCount == 1U)
      {
        if (CurrentXferOptions == I2C_NO_OPTION_FRAME)
        {
          /* Disable Acknowledge */
          CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

          if ((hi2c->Instance->CTRL2 & I2C_CTRL2_DMAEN) == I2C_CTRL2_DMAEN)
          {
            /* Disable Acknowledge */
            CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

            /* Clear ADDR flag */
            __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
          }
          else
          {
            /* Clear ADDR flag */
            __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

            /* Generate Stop */
            SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
          }
        }
        /* Prepare next transfer or stop current transfer */
        else if ((CurrentXferOptions != I2C_FIRST_AND_LAST_FRAME) && (CurrentXferOptions != I2C_LAST_FRAME) \
                 && ((Prev_State != I2C_STATE_MASTER_BUSY_RX) || (CurrentXferOptions == I2C_FIRST_FRAME)))
        {
          if ((CurrentXferOptions != I2C_NEXT_FRAME) && (CurrentXferOptions != I2C_FIRST_AND_NEXT_FRAME) && (CurrentXferOptions != I2C_LAST_FRAME_NO_STOP))
          {
            /* Disable Acknowledge */
            CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
          }
          else
          {
            /* Enable Acknowledge */
            SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
          }

          /* Clear ADDR flag */
          __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
        }
        else
        {
          /* Disable Acknowledge */
          CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

          /* Clear ADDR flag */
          __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

          /* Generate Stop */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
        }
      }
      else if (hi2c->XferCount == 2U)
      {
        if ((CurrentXferOptions != I2C_NEXT_FRAME) && (CurrentXferOptions != I2C_FIRST_AND_NEXT_FRAME) && (CurrentXferOptions != I2C_LAST_FRAME_NO_STOP))
        {
          /* Disable Acknowledge */
          CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

          /* Enable Pos */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKPOS);
        }
        else
        {
          /* Enable Acknowledge */
          SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
        }

        if (((hi2c->Instance->CTRL2 & I2C_CTRL2_DMAEN) == I2C_CTRL2_DMAEN) && ((CurrentXferOptions == I2C_NO_OPTION_FRAME) || (CurrentXferOptions == I2C_FIRST_FRAME) || (CurrentXferOptions == I2C_FIRST_AND_LAST_FRAME) || (CurrentXferOptions == I2C_LAST_FRAME_NO_STOP) || (CurrentXferOptions == I2C_LAST_FRAME)))
        {
          /* Enable Last DMA bit */
          SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_LTCFG);
        }

        /* Clear ADDR flag */
        __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
      }
      else
      {
        /* Enable Acknowledge */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

        if (((hi2c->Instance->CTRL2 & I2C_CTRL2_DMAEN) == I2C_CTRL2_DMAEN) && ((CurrentXferOptions == I2C_NO_OPTION_FRAME) || (CurrentXferOptions == I2C_FIRST_FRAME) || (CurrentXferOptions == I2C_FIRST_AND_LAST_FRAME) || (CurrentXferOptions == I2C_LAST_FRAME_NO_STOP) || (CurrentXferOptions == I2C_LAST_FRAME)))
        {
          /* Enable Last DMA bit */
          SET_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_LTCFG);
        }

        /* Clear ADDR flag */
        __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
      }

      /* Reset Event counter  */
      hi2c->EventCount = 0U;
    }
  }
  else
  {
    /* Clear ADDR flag */
    __DAL_I2C_CLEAR_ADDRFLAG(hi2c);
  }
}

/**
  * @brief  Handle TXE flag for Slave
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_SlaveTransmit_TXE(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  DAL_I2C_StateTypeDef CurrentState = hi2c->State;

  if (hi2c->XferCount != 0U)
  {
    /* Write data to DATA */
    hi2c->Instance->DATA = *hi2c->pBuffPtr;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    /* Update counter */
    hi2c->XferCount--;

    if ((hi2c->XferCount == 0U) && (CurrentState == DAL_I2C_STATE_BUSY_TX_LISTEN))
    {
      /* Last Byte is received, disable Interrupt */
      __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_BUF);

      /* Set state at DAL_I2C_STATE_LISTEN */
      hi2c->PreviousState = I2C_STATE_SLAVE_BUSY_TX;
      hi2c->State = DAL_I2C_STATE_LISTEN;

      /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->SlaveTxCpltCallback(hi2c);
#else
      DAL_I2C_SlaveTxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  Handle BTF flag for Slave transmitter
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_SlaveTransmit_BTF(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->XferCount != 0U)
  {
    /* Write data to DATA */
    hi2c->Instance->DATA = *hi2c->pBuffPtr;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    /* Update counter */
    hi2c->XferCount--;
  }
}

/**
  * @brief  Handle RXNE flag for Slave
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_SlaveReceive_RXNE(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  DAL_I2C_StateTypeDef CurrentState = hi2c->State;

  if (hi2c->XferCount != 0U)
  {
    /* Read data from DATA */
    *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    /* Update counter */
    hi2c->XferCount--;

    if ((hi2c->XferCount == 0U) && (CurrentState == DAL_I2C_STATE_BUSY_RX_LISTEN))
    {
      /* Last Byte is received, disable Interrupt */
      __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_BUF);

      /* Set state at DAL_I2C_STATE_LISTEN */
      hi2c->PreviousState = I2C_STATE_SLAVE_BUSY_RX;
      hi2c->State = DAL_I2C_STATE_LISTEN;

      /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->SlaveRxCpltCallback(hi2c);
#else
      DAL_I2C_SlaveRxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  Handle BTF flag for Slave receiver
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_SlaveReceive_BTF(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->XferCount != 0U)
  {
    /* Read data from DATA */
    *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    /* Update counter */
    hi2c->XferCount--;
  }
}

/**
  * @brief  Handle ADD flag for Slave
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  IT2Flags Interrupt2 flags to handle.
  * @retval None
  */
static void I2C_Slave_ADDR(I2C_HandleTypeDef *hi2c, uint32_t IT2Flags)
{
  uint8_t TransferDirection = I2C_DIRECTION_RECEIVE;
  uint16_t SlaveAddrCode;

  if (((uint32_t)hi2c->State & (uint32_t)DAL_I2C_STATE_LISTEN) == (uint32_t)DAL_I2C_STATE_LISTEN)
  {
    /* Disable BUF interrupt, BUF enabling is manage through slave specific interface */
    __DAL_I2C_DISABLE_IT(hi2c, (I2C_IT_BUF));

    /* Transfer Direction requested by Master */
    if (I2C_CHECK_FLAG(IT2Flags, I2C_FLAG_TRA) == RESET)
    {
      TransferDirection = I2C_DIRECTION_TRANSMIT;
    }

    if (I2C_CHECK_FLAG(IT2Flags, I2C_FLAG_DUALF) == RESET)
    {
      SlaveAddrCode = (uint16_t)hi2c->Init.OwnAddress1;
    }
    else
    {
      SlaveAddrCode = (uint16_t)hi2c->Init.OwnAddress2;
    }

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    /* Call Slave Addr callback */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->AddrCallback(hi2c, TransferDirection, SlaveAddrCode);
#else
    DAL_I2C_AddrCallback(hi2c, TransferDirection, SlaveAddrCode);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
  }
  else
  {
    /* Clear ADDR flag */
    __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);
  }
}

/**
  * @brief  Handle STOPF flag for Slave
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_Slave_STOPF(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  DAL_I2C_StateTypeDef CurrentState = hi2c->State;

  /* Disable EVT, BUF and ERR interrupt */
  __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

  /* Clear STOPF flag */
  __DAL_I2C_CLEAR_STOPFLAG(hi2c);

  /* Disable Acknowledge */
  CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

  /* If a DMA is ongoing, Update handle size context */
  if ((hi2c->Instance->CTRL2 & I2C_CTRL2_DMAEN) == I2C_CTRL2_DMAEN)
  {
    if ((CurrentState == DAL_I2C_STATE_BUSY_RX) || (CurrentState == DAL_I2C_STATE_BUSY_RX_LISTEN))
    {
      hi2c->XferCount = (uint16_t)(__DAL_DMA_GET_COUNTER(hi2c->hdmarx));

      if (hi2c->XferCount != 0U)
      {
        /* Set ErrorCode corresponding to a Non-Acknowledge */
        hi2c->ErrorCode |= DAL_I2C_ERROR_AF;
      }

      /* Disable, stop the current DMA */
      CLEAR_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

      /* Abort DMA Xfer if any */
      if (DAL_DMA_GetState(hi2c->hdmarx) != DAL_DMA_STATE_READY)
      {
        /* Set the I2C DMA Abort callback :
        will lead to call DAL_I2C_ErrorCallback() at end of DMA abort procedure */
        hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

        /* Abort DMA RX */
        if (DAL_DMA_Abort_IT(hi2c->hdmarx) != DAL_OK)
        {
          /* Call Directly XferAbortCallback function in case of error */
          hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
        }
      }
    }
    else
    {
      hi2c->XferCount = (uint16_t)(__DAL_DMA_GET_COUNTER(hi2c->hdmatx));

      if (hi2c->XferCount != 0U)
      {
        /* Set ErrorCode corresponding to a Non-Acknowledge */
        hi2c->ErrorCode |= DAL_I2C_ERROR_AF;
      }

      /* Disable, stop the current DMA */
      CLEAR_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

      /* Abort DMA Xfer if any */
      if (DAL_DMA_GetState(hi2c->hdmatx) != DAL_DMA_STATE_READY)
      {
        /* Set the I2C DMA Abort callback :
        will lead to call DAL_I2C_ErrorCallback() at end of DMA abort procedure */
        hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

        /* Abort DMA TX */
        if (DAL_DMA_Abort_IT(hi2c->hdmatx) != DAL_OK)
        {
          /* Call Directly XferAbortCallback function in case of error */
          hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
        }
      }
    }
  }

  /* All data are not transferred, so set error code accordingly */
  if (hi2c->XferCount != 0U)
  {
    /* Store Last receive data if any */
    if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET)
    {
      /* Read data from DATA */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferCount--;
    }

    /* Store Last receive data if any */
    if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == SET)
    {
      /* Read data from DATA */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      /* Update counter */
      hi2c->XferCount--;
    }

    if (hi2c->XferCount != 0U)
    {
      /* Set ErrorCode corresponding to a Non-Acknowledge */
      hi2c->ErrorCode |= DAL_I2C_ERROR_AF;
    }
  }

  if (hi2c->ErrorCode != DAL_I2C_ERROR_NONE)
  {
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    I2C_ITError(hi2c);
  }
  else
  {
    if (CurrentState == DAL_I2C_STATE_BUSY_RX_LISTEN)
    {
      /* Set state at DAL_I2C_STATE_LISTEN */
      hi2c->PreviousState = I2C_STATE_NONE;
      hi2c->State = DAL_I2C_STATE_LISTEN;

      /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->SlaveRxCpltCallback(hi2c);
#else
      DAL_I2C_SlaveRxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }

    if (hi2c->State == DAL_I2C_STATE_LISTEN)
    {
      hi2c->XferOptions = I2C_NO_OPTION_FRAME;
      hi2c->PreviousState = I2C_STATE_NONE;
      hi2c->State = DAL_I2C_STATE_READY;
      hi2c->Mode = DAL_I2C_MODE_NONE;

      /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->ListenCpltCallback(hi2c);
#else
      DAL_I2C_ListenCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }
    else
    {
      if ((hi2c->PreviousState  == I2C_STATE_SLAVE_BUSY_RX) || (CurrentState == DAL_I2C_STATE_BUSY_RX))
      {
        hi2c->PreviousState = I2C_STATE_NONE;
        hi2c->State = DAL_I2C_STATE_READY;
        hi2c->Mode = DAL_I2C_MODE_NONE;

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->SlaveRxCpltCallback(hi2c);
#else
        DAL_I2C_SlaveRxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
      }
    }
  }
}

/**
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
static void I2C_Slave_AF(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of temporary variables to prevent undefined behavior of volatile usage */
  DAL_I2C_StateTypeDef CurrentState = hi2c->State;
  uint32_t CurrentXferOptions       = hi2c->XferOptions;

  if (((CurrentXferOptions ==  I2C_FIRST_AND_LAST_FRAME) || (CurrentXferOptions == I2C_LAST_FRAME)) && \
      (CurrentState == DAL_I2C_STATE_LISTEN))
  {
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;

    /* Disable EVT, BUF and ERR interrupt */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    /* Clear AF flag */
    __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

    /* Disable Acknowledge */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

    hi2c->PreviousState = I2C_STATE_NONE;
    hi2c->State         = DAL_I2C_STATE_READY;
    hi2c->Mode          = DAL_I2C_MODE_NONE;

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->ListenCpltCallback(hi2c);
#else
    DAL_I2C_ListenCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
  }
  else if (CurrentState == DAL_I2C_STATE_BUSY_TX)
  {
    hi2c->XferOptions   = I2C_NO_OPTION_FRAME;
    hi2c->PreviousState = I2C_STATE_SLAVE_BUSY_TX;
    hi2c->State         = DAL_I2C_STATE_READY;
    hi2c->Mode          = DAL_I2C_MODE_NONE;

    /* Disable EVT, BUF and ERR interrupt */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    /* Clear AF flag */
    __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

    /* Disable Acknowledge */
    CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->SlaveTxCpltCallback(hi2c);
#else
    DAL_I2C_SlaveTxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
  }
  else
  {
    /* Clear AF flag only */
    /* State Listen, but XferOptions == FIRST or NEXT */
    __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
  }
}

/**
  * @brief  I2C interrupts error process
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_ITError(I2C_HandleTypeDef *hi2c)
{
  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  DAL_I2C_StateTypeDef CurrentState = hi2c->State;
  DAL_I2C_ModeTypeDef CurrentMode = hi2c->Mode;
  uint32_t CurrentError;

  if (((CurrentMode == DAL_I2C_MODE_MASTER) || (CurrentMode == DAL_I2C_MODE_MEM)) && (CurrentState == DAL_I2C_STATE_BUSY_RX))
  {
    /* Disable Pos bit in I2C CR1 when error occurred in Master/Mem Receive IT Process */
    hi2c->Instance->CTRL1 &= ~I2C_CTRL1_ACKPOS;
  }

  if (((uint32_t)CurrentState & (uint32_t)DAL_I2C_STATE_LISTEN) == (uint32_t)DAL_I2C_STATE_LISTEN)
  {
    /* keep DAL_I2C_STATE_LISTEN */
    hi2c->PreviousState = I2C_STATE_NONE;
    hi2c->State = DAL_I2C_STATE_LISTEN;
  }
  else
  {
    /* If state is an abort treatment on going, don't change state */
    /* This change will be do later */
    if ((READ_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN) != I2C_CTRL2_DMAEN) && (CurrentState != DAL_I2C_STATE_ABORT))
    {
      hi2c->State = DAL_I2C_STATE_READY;
      hi2c->Mode = DAL_I2C_MODE_NONE;
    }
    hi2c->PreviousState = I2C_STATE_NONE;
  }

  /* Abort DMA transfer */
  if (READ_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN) == I2C_CTRL2_DMAEN)
  {
    hi2c->Instance->CTRL2 &= ~I2C_CTRL2_DMAEN;

    if (hi2c->hdmatx->State != DAL_DMA_STATE_READY)
    {
      /* Set the DMA Abort callback :
      will lead to call DAL_I2C_ErrorCallback() at end of DMA abort procedure */
      hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

      if (DAL_DMA_Abort_IT(hi2c->hdmatx) != DAL_OK)
      {
        /* Disable I2C peripheral to prevent dummy data in buffer */
        __DAL_I2C_DISABLE(hi2c);

        hi2c->State = DAL_I2C_STATE_READY;

        /* Call Directly XferAbortCallback function in case of error */
        hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
      }
    }
    else
    {
      /* Set the DMA Abort callback :
      will lead to call DAL_I2C_ErrorCallback() at end of DMA abort procedure */
      hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

      if (DAL_DMA_Abort_IT(hi2c->hdmarx) != DAL_OK)
      {
        /* Store Last receive data if any */
        if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == SET)
        {
          /* Read data from DATA */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;
        }

        /* Disable I2C peripheral to prevent dummy data in buffer */
        __DAL_I2C_DISABLE(hi2c);

        hi2c->State = DAL_I2C_STATE_READY;

        /* Call Directly hi2c->hdmarx->XferAbortCallback function in case of error */
        hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
      }
    }
  }
  else if (hi2c->State == DAL_I2C_STATE_ABORT)
  {
    hi2c->State = DAL_I2C_STATE_READY;
    hi2c->ErrorCode = DAL_I2C_ERROR_NONE;

    /* Store Last receive data if any */
    if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == SET)
    {
      /* Read data from DATA */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;
    }

    /* Disable I2C peripheral to prevent dummy data in buffer */
    __DAL_I2C_DISABLE(hi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->AbortCpltCallback(hi2c);
#else
    DAL_I2C_AbortCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
  }
  else
  {
    /* Store Last receive data if any */
    if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == SET)
    {
      /* Read data from DATA */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DATA;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;
    }

    /* Call user error callback */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->ErrorCallback(hi2c);
#else
    DAL_I2C_ErrorCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
  }

  /* STOP Flag is not set after a NACK reception, BusError, ArbitrationLost, OverRun */
  CurrentError = hi2c->ErrorCode;

  if (((CurrentError & DAL_I2C_ERROR_BERR) == DAL_I2C_ERROR_BERR) || \
      ((CurrentError & DAL_I2C_ERROR_ARLO) == DAL_I2C_ERROR_ARLO) || \
      ((CurrentError & DAL_I2C_ERROR_AF) == DAL_I2C_ERROR_AF)     || \
      ((CurrentError & DAL_I2C_ERROR_OVR) == DAL_I2C_ERROR_OVR))
  {
    /* Disable EVT, BUF and ERR interrupt */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
  }

  /* So may inform upper layer that listen phase is stopped */
  /* during NACK error treatment */
  CurrentState = hi2c->State;
  if (((hi2c->ErrorCode & DAL_I2C_ERROR_AF) == DAL_I2C_ERROR_AF) && (CurrentState == DAL_I2C_STATE_LISTEN))
  {
    hi2c->XferOptions   = I2C_NO_OPTION_FRAME;
    hi2c->PreviousState = I2C_STATE_NONE;
    hi2c->State         = DAL_I2C_STATE_READY;
    hi2c->Mode          = DAL_I2C_MODE_NONE;

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->ListenCpltCallback(hi2c);
#else
    DAL_I2C_ListenCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
  }
}

/**
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_MasterRequestWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout, uint32_t Tickstart)
{
  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  uint32_t CurrentXferOptions = hi2c->XferOptions;

  /* Generate Start condition if first transfer */
  if ((CurrentXferOptions == I2C_FIRST_AND_LAST_FRAME) || (CurrentXferOptions == I2C_FIRST_FRAME) || (CurrentXferOptions == I2C_NO_OPTION_FRAME))
  {
    /* Generate Start */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
  }
  else if (hi2c->PreviousState == I2C_STATE_MASTER_BUSY_RX)
  {
    /* Generate ReStart */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
  }
  else
  {
    /* Do nothing */
  }

  /* Wait until SB flag is set */
  if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout, Tickstart) != DAL_OK)
  {
    if (READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START) == I2C_CTRL1_START)
    {
      hi2c->ErrorCode = DAL_I2C_WRONG_START;
    }
    return DAL_TIMEOUT;
  }

  if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
  {
    /* Send slave address */
    hi2c->Instance->DATA = I2C_7BIT_ADD_WRITE(DevAddress);
  }
  else
  {
    /* Send header of slave address */
    hi2c->Instance->DATA = I2C_10BIT_HEADER_WRITE(DevAddress);

    /* Wait until ADD10 flag is set */
    if (I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADD10, Timeout, Tickstart) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Send slave address */
    hi2c->Instance->DATA = I2C_10BIT_ADDRESS(DevAddress);
  }

  /* Wait until ADDR flag is set */
  if (I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout, Tickstart) != DAL_OK)
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Master sends target device address for read request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_MasterRequestRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout, uint32_t Tickstart)
{
  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  uint32_t CurrentXferOptions = hi2c->XferOptions;

  /* Enable Acknowledge */
  SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

  /* Generate Start condition if first transfer */
  if ((CurrentXferOptions == I2C_FIRST_AND_LAST_FRAME) || (CurrentXferOptions == I2C_FIRST_FRAME)  || (CurrentXferOptions == I2C_NO_OPTION_FRAME))
  {
    /* Generate Start */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
  }
  else if (hi2c->PreviousState == I2C_STATE_MASTER_BUSY_TX)
  {
    /* Generate ReStart */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);
  }
  else
  {
    /* Do nothing */
  }

  /* Wait until SB flag is set */
  if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout, Tickstart) != DAL_OK)
  {
    if (READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START) == I2C_CTRL1_START)
    {
      hi2c->ErrorCode = DAL_I2C_WRONG_START;
    }
    return DAL_TIMEOUT;
  }

  if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
  {
    /* Send slave address */
    hi2c->Instance->DATA = I2C_7BIT_ADD_READ(DevAddress);
  }
  else
  {
    /* Send header of slave address */
    hi2c->Instance->DATA = I2C_10BIT_HEADER_WRITE(DevAddress);

    /* Wait until ADD10 flag is set */
    if (I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADD10, Timeout, Tickstart) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Send slave address */
    hi2c->Instance->DATA = I2C_10BIT_ADDRESS(DevAddress);

    /* Wait until ADDR flag is set */
    if (I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout, Tickstart) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Clear ADDR flag */
    __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

    /* Generate Restart */
    SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

    /* Wait until SB flag is set */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout, Tickstart) != DAL_OK)
    {
      if (READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START) == I2C_CTRL1_START)
      {
        hi2c->ErrorCode = DAL_I2C_WRONG_START;
      }
      return DAL_TIMEOUT;
    }

    /* Send header of slave address */
    hi2c->Instance->DATA = I2C_10BIT_HEADER_READ(DevAddress);
  }

  /* Wait until ADDR flag is set */
  if (I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout, Tickstart) != DAL_OK)
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Master sends target device address followed by internal memory address for write request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_RequestMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
  /* Generate Start */
  SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

  /* Wait until SB flag is set */
  if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout, Tickstart) != DAL_OK)
  {
    if (READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START) == I2C_CTRL1_START)
    {
      hi2c->ErrorCode = DAL_I2C_WRONG_START;
    }
    return DAL_TIMEOUT;
  }

  /* Send slave address */
  hi2c->Instance->DATA = I2C_7BIT_ADD_WRITE(DevAddress);

  /* Wait until ADDR flag is set */
  if (I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout, Tickstart) != DAL_OK)
  {
    return DAL_ERROR;
  }

  /* Clear ADDR flag */
  __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

  /* Wait until TXE flag is set */
  if (I2C_WaitOnTXEFlagUntilTimeout(hi2c, Timeout, Tickstart) != DAL_OK)
  {
    if (hi2c->ErrorCode == DAL_I2C_ERROR_AF)
    {
      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
    }
    return DAL_ERROR;
  }

  /* If Memory address size is 8Bit */
  if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    /* Send Memory Address */
    hi2c->Instance->DATA = I2C_MEM_ADD_LSB(MemAddress);
  }
  /* If Memory address size is 16Bit */
  else
  {
    /* Send MSB of Memory Address */
    hi2c->Instance->DATA = I2C_MEM_ADD_MSB(MemAddress);

    /* Wait until TXE flag is set */
    if (I2C_WaitOnTXEFlagUntilTimeout(hi2c, Timeout, Tickstart) != DAL_OK)
    {
      if (hi2c->ErrorCode == DAL_I2C_ERROR_AF)
      {
        /* Generate Stop */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
      }
      return DAL_ERROR;
    }

    /* Send LSB of Memory Address */
    hi2c->Instance->DATA = I2C_MEM_ADD_LSB(MemAddress);
  }

  return DAL_OK;
}

/**
  * @brief  Master sends target device address followed by internal memory address for read request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_RequestMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
  /* Enable Acknowledge */
  SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

  /* Generate Start */
  SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

  /* Wait until SB flag is set */
  if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout, Tickstart) != DAL_OK)
  {
    if (READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START) == I2C_CTRL1_START)
    {
      hi2c->ErrorCode = DAL_I2C_WRONG_START;
    }
    return DAL_TIMEOUT;
  }

  /* Send slave address */
  hi2c->Instance->DATA = I2C_7BIT_ADD_WRITE(DevAddress);

  /* Wait until ADDR flag is set */
  if (I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout, Tickstart) != DAL_OK)
  {
    return DAL_ERROR;
  }

  /* Clear ADDR flag */
  __DAL_I2C_CLEAR_ADDRFLAG(hi2c);

  /* Wait until TXE flag is set */
  if (I2C_WaitOnTXEFlagUntilTimeout(hi2c, Timeout, Tickstart) != DAL_OK)
  {
    if (hi2c->ErrorCode == DAL_I2C_ERROR_AF)
    {
      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
    }
    return DAL_ERROR;
  }

  /* If Memory address size is 8Bit */
  if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    /* Send Memory Address */
    hi2c->Instance->DATA = I2C_MEM_ADD_LSB(MemAddress);
  }
  /* If Memory address size is 16Bit */
  else
  {
    /* Send MSB of Memory Address */
    hi2c->Instance->DATA = I2C_MEM_ADD_MSB(MemAddress);

    /* Wait until TXE flag is set */
    if (I2C_WaitOnTXEFlagUntilTimeout(hi2c, Timeout, Tickstart) != DAL_OK)
    {
      if (hi2c->ErrorCode == DAL_I2C_ERROR_AF)
      {
        /* Generate Stop */
        SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
      }
      return DAL_ERROR;
    }

    /* Send LSB of Memory Address */
    hi2c->Instance->DATA = I2C_MEM_ADD_LSB(MemAddress);
  }

  /* Wait until TXE flag is set */
  if (I2C_WaitOnTXEFlagUntilTimeout(hi2c, Timeout, Tickstart) != DAL_OK)
  {
    if (hi2c->ErrorCode == DAL_I2C_ERROR_AF)
    {
      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
    }
    return DAL_ERROR;
  }

  /* Generate Restart */
  SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START);

  /* Wait until SB flag is set */
  if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout, Tickstart) != DAL_OK)
  {
    if (READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_START) == I2C_CTRL1_START)
    {
      hi2c->ErrorCode = DAL_I2C_WRONG_START;
    }
    return DAL_TIMEOUT;
  }

  /* Send slave address */
  hi2c->Instance->DATA = I2C_7BIT_ADD_READ(DevAddress);

  /* Wait until ADDR flag is set */
  if (I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout, Tickstart) != DAL_OK)
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  DMA I2C process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_DMAXferCplt(DMA_HandleTypeDef *hdma)
{
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent; /* Derogation MISRAC2012-Rule-11.5 */

  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  DAL_I2C_StateTypeDef CurrentState = hi2c->State;
  DAL_I2C_ModeTypeDef CurrentMode   = hi2c->Mode;
  uint32_t CurrentXferOptions       = hi2c->XferOptions;

  /* Disable EVT and ERR interrupt */
  __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

  /* Clear Complete callback */
  if (hi2c->hdmatx != NULL)
  {
    hi2c->hdmatx->XferCpltCallback = NULL;
  }
  if (hi2c->hdmarx != NULL)
  {
    hi2c->hdmarx->XferCpltCallback = NULL;
  }

  if ((((uint32_t)CurrentState & (uint32_t)DAL_I2C_STATE_BUSY_TX) == (uint32_t)DAL_I2C_STATE_BUSY_TX) || ((((uint32_t)CurrentState & (uint32_t)DAL_I2C_STATE_BUSY_RX) == (uint32_t)DAL_I2C_STATE_BUSY_RX) && (CurrentMode == DAL_I2C_MODE_SLAVE)))
  {
    /* Disable DMA Request */
    CLEAR_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

    hi2c->XferCount = 0U;

    if (CurrentState == DAL_I2C_STATE_BUSY_TX_LISTEN)
    {
      /* Set state at DAL_I2C_STATE_LISTEN */
      hi2c->PreviousState = I2C_STATE_SLAVE_BUSY_TX;
      hi2c->State = DAL_I2C_STATE_LISTEN;

      /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->SlaveTxCpltCallback(hi2c);
#else
      DAL_I2C_SlaveTxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }
    else if (CurrentState == DAL_I2C_STATE_BUSY_RX_LISTEN)
    {
      /* Set state at DAL_I2C_STATE_LISTEN */
      hi2c->PreviousState = I2C_STATE_SLAVE_BUSY_RX;
      hi2c->State = DAL_I2C_STATE_LISTEN;

      /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->SlaveRxCpltCallback(hi2c);
#else
      DAL_I2C_SlaveRxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }
    else
    {
      /* Do nothing */
    }

    /* Enable EVT and ERR interrupt to treat end of transfer in IRQ handler */
    __DAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);
  }
  /* Check current Mode, in case of treatment DMA handler have been preempted by a prior interrupt */
  else if (hi2c->Mode != DAL_I2C_MODE_NONE)
  {
    if (hi2c->XferCount == (uint16_t)1)
    {
      /* Disable Acknowledge */
      CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);
    }

    /* Disable EVT and ERR interrupt */
    __DAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

    /* Prepare next transfer or stop current transfer */
    if ((CurrentXferOptions == I2C_NO_OPTION_FRAME) || (CurrentXferOptions == I2C_FIRST_AND_LAST_FRAME) || (CurrentXferOptions == I2C_OTHER_AND_LAST_FRAME) || (CurrentXferOptions == I2C_LAST_FRAME))
    {
      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);
    }

    /* Disable Last DMA */
    CLEAR_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_LTCFG);

    /* Disable DMA Request */
    CLEAR_BIT(hi2c->Instance->CTRL2, I2C_CTRL2_DMAEN);

    hi2c->XferCount = 0U;

    /* Check if Errors has been detected during transfer */
    if (hi2c->ErrorCode != DAL_I2C_ERROR_NONE)
    {
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->ErrorCallback(hi2c);
#else
      DAL_I2C_ErrorCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
    }
    else
    {
      hi2c->State = DAL_I2C_STATE_READY;

      if (hi2c->Mode == DAL_I2C_MODE_MEM)
      {
        hi2c->Mode = DAL_I2C_MODE_NONE;
        hi2c->PreviousState = I2C_STATE_NONE;

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->MemRxCpltCallback(hi2c);
#else
        DAL_I2C_MemRxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
      }
      else
      {
        hi2c->Mode = DAL_I2C_MODE_NONE;
        hi2c->PreviousState = I2C_STATE_MASTER_BUSY_RX;

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->MasterRxCpltCallback(hi2c);
#else
        DAL_I2C_MasterRxCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
      }
    }
  }
  else
  {
    /* Do nothing */
  }
}

/**
  * @brief  DMA I2C communication error callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_DMAError(DMA_HandleTypeDef *hdma)
{
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent; /* Derogation MISRAC2012-Rule-11.5 */

  /* Clear Complete callback */
  if (hi2c->hdmatx != NULL)
  {
    hi2c->hdmatx->XferCpltCallback = NULL;
  }
  if (hi2c->hdmarx != NULL)
  {
    hi2c->hdmarx->XferCpltCallback = NULL;
  }

  /* Ignore DMA FIFO error */
  if (DAL_DMA_GetError(hdma) != DAL_DMA_ERROR_FE)
  {
    /* Disable Acknowledge */
    hi2c->Instance->CTRL1 &= ~I2C_CTRL1_ACKEN;

    hi2c->XferCount = 0U;

    hi2c->State = DAL_I2C_STATE_READY;
    hi2c->Mode = DAL_I2C_MODE_NONE;

    hi2c->ErrorCode |= DAL_I2C_ERROR_DMA;

#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->ErrorCallback(hi2c);
#else
    DAL_I2C_ErrorCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
  }
}

/**
  * @brief DMA I2C communication abort callback
  *        (To be called at end of DMA Abort procedure).
  * @param hdma DMA handle.
  * @retval None
  */
static void I2C_DMAAbort(DMA_HandleTypeDef *hdma)
{
  __IO uint32_t count = 0U;
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent; /* Derogation MISRAC2012-Rule-11.5 */

  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  DAL_I2C_StateTypeDef CurrentState = hi2c->State;

  /* During abort treatment, check that there is no pending STOP request */
  /* Wait until STOP flag is reset */
  count = I2C_TIMEOUT_FLAG * (SystemCoreClock / 25U / 1000U);
  do
  {
    if (count == 0U)
    {
      hi2c->ErrorCode |= DAL_I2C_ERROR_TIMEOUT;
      break;
    }
    count--;
  }
  while (READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP) == I2C_CTRL1_STOP);

  /* Clear Complete callback */
  if (hi2c->hdmatx != NULL)
  {
    hi2c->hdmatx->XferCpltCallback = NULL;
  }
  if (hi2c->hdmarx != NULL)
  {
    hi2c->hdmarx->XferCpltCallback = NULL;
  }

  /* Disable Acknowledge */
  CLEAR_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

  hi2c->XferCount = 0U;

  /* Reset XferAbortCallback */
  if (hi2c->hdmatx != NULL)
  {
    hi2c->hdmatx->XferAbortCallback = NULL;
  }
  if (hi2c->hdmarx != NULL)
  {
    hi2c->hdmarx->XferAbortCallback = NULL;
  }

  /* Disable I2C peripheral to prevent dummy data in buffer */
  __DAL_I2C_DISABLE(hi2c);

  /* Check if come from abort from user */
  if (hi2c->State == DAL_I2C_STATE_ABORT)
  {
    hi2c->State         = DAL_I2C_STATE_READY;
    hi2c->Mode          = DAL_I2C_MODE_NONE;
    hi2c->ErrorCode     = DAL_I2C_ERROR_NONE;

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->AbortCpltCallback(hi2c);
#else
    DAL_I2C_AbortCpltCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
  }
  else
  {
    if (((uint32_t)CurrentState & (uint32_t)DAL_I2C_STATE_LISTEN) == (uint32_t)DAL_I2C_STATE_LISTEN)
    {
      /* Renable I2C peripheral */
      __DAL_I2C_ENABLE(hi2c);

      /* Enable Acknowledge */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_ACKEN);

      /* keep DAL_I2C_STATE_LISTEN */
      hi2c->PreviousState = I2C_STATE_NONE;
      hi2c->State = DAL_I2C_STATE_LISTEN;
    }
    else
    {
      hi2c->State = DAL_I2C_STATE_READY;
      hi2c->Mode = DAL_I2C_MODE_NONE;
    }

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_DAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->ErrorCallback(hi2c);
#else
    DAL_I2C_ErrorCallback(hi2c);
#endif /* USE_DAL_I2C_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  This function handles I2C Communication Timeout.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  Flag specifies the I2C flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
  /* Wait until flag is set */
  while (__DAL_I2C_GET_FLAG(hi2c, Flag) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != DAL_MAX_DELAY)
    {
      if (((DAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->PreviousState     = I2C_STATE_NONE;
        hi2c->State             = DAL_I2C_STATE_READY;
        hi2c->Mode              = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode         |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
  }
  return DAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for Master addressing phase.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  Flag specifies the I2C flag to check.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_WaitOnMasterAddressFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, uint32_t Timeout, uint32_t Tickstart)
{
  while (__DAL_I2C_GET_FLAG(hi2c, Flag) == RESET)
  {
    if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == SET)
    {
      /* Generate Stop */
      SET_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP);

      /* Clear AF Flag */
      __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

      hi2c->PreviousState       = I2C_STATE_NONE;
      hi2c->State               = DAL_I2C_STATE_READY;
      hi2c->Mode                = DAL_I2C_MODE_NONE;
      hi2c->ErrorCode           |= DAL_I2C_ERROR_AF;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }

    /* Check for the Timeout */
    if (Timeout != DAL_MAX_DELAY)
    {
      if (((DAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
  }
  return DAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of TXE flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_WaitOnTXEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXE) == RESET)
  {
    /* Check if a NACK is detected */
    if (I2C_IsAcknowledgeFailed(hi2c) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Check for the Timeout */
    if (Timeout != DAL_MAX_DELAY)
    {
      if (((DAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
  }
  return DAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of BTF flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_WaitOnBTFFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == RESET)
  {
    /* Check if a NACK is detected */
    if (I2C_IsAcknowledgeFailed(hi2c) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Check for the Timeout */
    if (Timeout != DAL_MAX_DELAY)
    {
      if (((DAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = DAL_I2C_STATE_READY;
        hi2c->Mode                = DAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2c);

        return DAL_ERROR;
      }
    }
  }
  return DAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of STOP flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == RESET)
  {
    /* Check if a NACK is detected */
    if (I2C_IsAcknowledgeFailed(hi2c) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Check for the Timeout */
    if (((DAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
    {
      hi2c->PreviousState       = I2C_STATE_NONE;
      hi2c->State               = DAL_I2C_STATE_READY;
      hi2c->Mode                = DAL_I2C_MODE_NONE;
      hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }
  }
  return DAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of STOP request through Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_WaitOnSTOPRequestThroughIT(I2C_HandleTypeDef *hi2c)
{
  __IO uint32_t count = 0U;

  /* Wait until STOP flag is reset */
  count = I2C_TIMEOUT_STOP_FLAG * (SystemCoreClock / 25U / 1000U);
  do
  {
    count--;
    if (count == 0U)
    {
      hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

      return DAL_ERROR;
    }
  }
  while (READ_BIT(hi2c->Instance->CTRL1, I2C_CTRL1_STOP) == I2C_CTRL1_STOP);

  return DAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of RXNE flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{

  while (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == RESET)
  {
    /* Check if a STOPF is detected */
    if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == SET)
    {
      /* Clear STOP Flag */
      __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

      hi2c->PreviousState       = I2C_STATE_NONE;
      hi2c->State               = DAL_I2C_STATE_READY;
      hi2c->Mode                = DAL_I2C_MODE_NONE;
      hi2c->ErrorCode           |= DAL_I2C_ERROR_NONE;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }

    /* Check for the Timeout */
    if (((DAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
    {
      hi2c->PreviousState       = I2C_STATE_NONE;
      hi2c->State               = DAL_I2C_STATE_READY;
      hi2c->Mode                = DAL_I2C_MODE_NONE;
      hi2c->ErrorCode           |= DAL_I2C_ERROR_TIMEOUT;

      /* Process Unlocked */
      __DAL_UNLOCK(hi2c);

      return DAL_ERROR;
    }
  }
  return DAL_OK;
}

/**
  * @brief  This function handles Acknowledge failed detection during an I2C Communication.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval DAL status
  */
static DAL_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c)
{
  if (__DAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == SET)
  {
    /* Clear NACKF Flag */
    __DAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

    hi2c->PreviousState       = I2C_STATE_NONE;
    hi2c->State               = DAL_I2C_STATE_READY;
    hi2c->Mode                = DAL_I2C_MODE_NONE;
    hi2c->ErrorCode           |= DAL_I2C_ERROR_AF;

    /* Process Unlocked */
    __DAL_UNLOCK(hi2c);

    return DAL_ERROR;
  }
  return DAL_OK;
}

/**
  * @brief  Convert I2Cx OTHER_xxx XferOptions to functional XferOptions.
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_ConvertOtherXferOptions(I2C_HandleTypeDef *hi2c)
{
  /* if user set XferOptions to I2C_OTHER_FRAME            */
  /* it request implicitly to generate a restart condition */
  /* set XferOptions to I2C_FIRST_FRAME                    */
  if (hi2c->XferOptions == I2C_OTHER_FRAME)
  {
    hi2c->XferOptions = I2C_FIRST_FRAME;
  }
  /* else if user set XferOptions to I2C_OTHER_AND_LAST_FRAME */
  /* it request implicitly to generate a restart condition    */
  /* then generate a stop condition at the end of transfer    */
  /* set XferOptions to I2C_FIRST_AND_LAST_FRAME              */
  else if (hi2c->XferOptions == I2C_OTHER_AND_LAST_FRAME)
  {
    hi2c->XferOptions = I2C_FIRST_AND_LAST_FRAME;
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @}
  */

#endif /* DAL_I2C_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

