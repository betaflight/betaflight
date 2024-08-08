/**
  *
  * @file    apm32f4xx_ddl_dma.h
  * @brief   Header file of DMA DDL module.
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
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DDL_DMA_H
#define APM32F4xx_DDL_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (DMA1) || defined (DMA2)

/** @defgroup DMA_DDL DMA
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup DMA_DDL_Private_Variables DMA Private Variables
  * @{
  */
/* Array used to get the DMA stream register offset versus stream index DDL_DMA_STREAM_x */
static const uint8_t STREAM_OFFSET_TAB[] =
{
  (uint8_t)(DMA1_Stream0_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream1_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream2_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream3_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream4_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream5_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream6_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Stream7_BASE - DMA1_BASE)
};

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup DMA_DDL_Private_Constants DMA Private Constants
  * @{
  */
/**
  * @}
  */


/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup DMA_DDL_ES_INIT DMA Exported Init structure
  * @{
  */
typedef struct
{
  uint32_t PeriphOrM2MSrcAddress;  /*!< Specifies the peripheral base address for DMA transfer
                                        or as Source base address in case of memory to memory transfer direction.

                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0xFFFFFFFF. */

  uint32_t MemoryOrM2MDstAddress;  /*!< Specifies the memory base address for DMA transfer
                                        or as Destination base address in case of memory to memory transfer direction.

                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0xFFFFFFFF. */

  uint32_t Direction;              /*!< Specifies if the data will be transferred from memory to peripheral,
                                        from memory to memory or from peripheral to memory.
                                        This parameter can be a value of @ref DMA_DDL_EC_DIRECTION

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetDataTransferDirection(). */

  uint32_t Mode;                   /*!< Specifies the normal or circular operation mode.
                                        This parameter can be a value of @ref DMA_DDL_EC_MODE
                                        @note The circular buffer mode cannot be used if the memory to memory
                                              data transfer direction is configured on the selected Stream

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetMode(). */

  uint32_t PeriphOrM2MSrcIncMode;  /*!< Specifies whether the Peripheral address or Source address in case of memory to memory transfer direction
                                        is incremented or not.
                                        This parameter can be a value of @ref DMA_DDL_EC_PERIPH

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetPeriphIncMode(). */

  uint32_t MemoryOrM2MDstIncMode;  /*!< Specifies whether the Memory address or Destination address in case of memory to memory transfer direction
                                        is incremented or not.
                                        This parameter can be a value of @ref DMA_DDL_EC_MEMORY

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetMemoryIncMode(). */

  uint32_t PeriphOrM2MSrcDataSize; /*!< Specifies the Peripheral data size alignment or Source data size alignment (byte, half word, word)
                                        in case of memory to memory transfer direction.
                                        This parameter can be a value of @ref DMA_DDL_EC_PDATAALIGN

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetPeriphSize(). */

  uint32_t MemoryOrM2MDstDataSize; /*!< Specifies the Memory data size alignment or Destination data size alignment (byte, half word, word)
                                        in case of memory to memory transfer direction.
                                        This parameter can be a value of @ref DMA_DDL_EC_MDATAALIGN

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetMemorySize(). */

  uint32_t NbData;                 /*!< Specifies the number of data to transfer, in data unit.
                                        The data unit is equal to the source buffer configuration set in PeripheralSize
                                        or MemorySize parameters depending in the transfer direction.
                                        This parameter must be a value between Min_Data = 0 and Max_Data = 0x0000FFFF

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetDataLength(). */

  uint32_t Channel;                /*!< Specifies the peripheral channel.
                                        This parameter can be a value of @ref DMA_DDL_EC_CHANNEL

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetChannelSelection(). */

  uint32_t Priority;               /*!< Specifies the channel priority level.
                                        This parameter can be a value of @ref DMA_DDL_EC_PRIORITY

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetStreamPriorityLevel(). */
                                        
  uint32_t FIFOMode;               /*!< Specifies if the FIFO mode or Direct mode will be used for the specified stream.
                                        This parameter can be a value of @ref DMA_DDL_FIFOMODE
                                        @note The Direct mode (FIFO mode disabled) cannot be used if the 
                                        memory-to-memory data transfer is configured on the selected stream

                                        This feature can be modified afterwards using unitary functions @ref DDL_DMA_EnableFifoMode() or @ref DDL_DMA_EnableFifoMode() . */

  uint32_t FIFOThreshold;          /*!< Specifies the FIFO threshold level.
                                        This parameter can be a value of @ref DMA_DDL_EC_FIFOTHRESHOLD

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetFIFOThreshold(). */

  uint32_t MemBurst;               /*!< Specifies the Burst transfer configuration for the memory transfers. 
                                        It specifies the amount of data to be transferred in a single non interruptible
                                        transaction.
                                        This parameter can be a value of @ref DMA_DDL_EC_MBURST 
                                        @note The burst mode is possible only if the address Increment mode is enabled. 

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetMemoryBurstxfer(). */

  uint32_t PeriphBurst;            /*!< Specifies the Burst transfer configuration for the peripheral transfers. 
                                        It specifies the amount of data to be transferred in a single non interruptible 
                                        transaction. 
                                        This parameter can be a value of @ref DMA_DDL_EC_PBURST
                                        @note The burst mode is possible only if the address Increment mode is enabled. 

                                        This feature can be modified afterwards using unitary function @ref DDL_DMA_SetPeriphBurstxfer(). */

} DDL_DMA_InitTypeDef;
/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup DMA_DDL_Exported_Constants DMA Exported Constants
  * @{
  */

/** @defgroup DMA_DDL_EC_STREAM STREAM
  * @{
  */
#define DDL_DMA_STREAM_0                   0x00000000U
#define DDL_DMA_STREAM_1                   0x00000001U
#define DDL_DMA_STREAM_2                   0x00000002U
#define DDL_DMA_STREAM_3                   0x00000003U
#define DDL_DMA_STREAM_4                   0x00000004U
#define DDL_DMA_STREAM_5                   0x00000005U
#define DDL_DMA_STREAM_6                   0x00000006U
#define DDL_DMA_STREAM_7                   0x00000007U
#define DDL_DMA_STREAM_ALL                 0xFFFF0000U
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_DIRECTION DIRECTION
  * @{
  */
#define DDL_DMA_DIRECTION_PERIPH_TO_MEMORY 0x00000000U               /*!< Peripheral to memory direction */
#define DDL_DMA_DIRECTION_MEMORY_TO_PERIPH DMA_SCFGx_DIRCFG_0            /*!< Memory to peripheral direction */
#define DDL_DMA_DIRECTION_MEMORY_TO_MEMORY DMA_SCFGx_DIRCFG_1            /*!< Memory to memory direction     */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_MODE MODE
  * @{
  */
#define DDL_DMA_MODE_NORMAL                0x00000000U               /*!< Normal Mode                  */
#define DDL_DMA_MODE_CIRCULAR              DMA_SCFGx_CIRCMEN             /*!< Circular Mode                */
#define DDL_DMA_MODE_PFCTRL                DMA_SCFGx_PERFC           /*!< Peripheral flow control mode */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_DOUBLEBUFFER_MODE DOUBLEBUFFER MODE
  * @{
  */
#define DDL_DMA_DOUBLEBUFFER_MODE_DISABLE  0x00000000U               /*!< Disable double buffering mode */
#define DDL_DMA_DOUBLEBUFFER_MODE_ENABLE   DMA_SCFGx_DBM              /*!< Enable double buffering mode  */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_PERIPH PERIPH
  * @{
  */
#define DDL_DMA_PERIPH_NOINCREMENT         0x00000000U               /*!< Peripheral increment mode Disable */
#define DDL_DMA_PERIPH_INCREMENT           DMA_SCFGx_PERIM             /*!< Peripheral increment mode Enable  */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_MEMORY MEMORY
  * @{
  */
#define DDL_DMA_MEMORY_NOINCREMENT         0x00000000U               /*!< Memory increment mode Disable */
#define DDL_DMA_MEMORY_INCREMENT           DMA_SCFGx_MEMIM             /*!< Memory increment mode Enable  */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_PDATAALIGN PDATAALIGN
  * @{
  */
#define DDL_DMA_PDATAALIGN_BYTE            0x00000000U               /*!< Peripheral data alignment : Byte     */
#define DDL_DMA_PDATAALIGN_HALFWORD        DMA_SCFGx_PERSIZECFG_0          /*!< Peripheral data alignment : HalfWord */
#define DDL_DMA_PDATAALIGN_WORD            DMA_SCFGx_PERSIZECFG_1          /*!< Peripheral data alignment : Word     */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_MDATAALIGN MDATAALIGN
  * @{
  */
#define DDL_DMA_MDATAALIGN_BYTE            0x00000000U               /*!< Memory data alignment : Byte     */
#define DDL_DMA_MDATAALIGN_HALFWORD        DMA_SCFGx_MEMSIZECFG_0          /*!< Memory data alignment : HalfWord */
#define DDL_DMA_MDATAALIGN_WORD            DMA_SCFGx_MEMSIZECFG_1          /*!< Memory data alignment : Word     */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_OFFSETSIZE OFFSETSIZE
  * @{
  */
#define DDL_DMA_OFFSETSIZE_PSIZE           0x00000000U               /*!< Peripheral increment offset size is linked to the PSIZE */
#define DDL_DMA_OFFSETSIZE_FIXEDTO4        DMA_SCFGx_PERIOSIZE           /*!< Peripheral increment offset size is fixed to 4 (32-bit alignment) */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_PRIORITY PRIORITY
  * @{
  */
#define DDL_DMA_PRIORITY_LOW               0x00000000U               /*!< Priority level : Low       */
#define DDL_DMA_PRIORITY_MEDIUM            DMA_SCFGx_PRILCFG_0             /*!< Priority level : Medium    */
#define DDL_DMA_PRIORITY_HIGH              DMA_SCFGx_PRILCFG_1             /*!< Priority level : High      */
#define DDL_DMA_PRIORITY_VERYHIGH          DMA_SCFGx_PRILCFG               /*!< Priority level : Very_High */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_CHANNEL CHANNEL
  * @{
  */
#define DDL_DMA_CHANNEL_0                  0x00000000U                                                                   /* Select Channel0 of DMA Instance */
#define DDL_DMA_CHANNEL_1                  DMA_SCFGx_CHSEL_0                                                              /* Select Channel1 of DMA Instance */
#define DDL_DMA_CHANNEL_2                  DMA_SCFGx_CHSEL_1                                                              /* Select Channel2 of DMA Instance */
#define DDL_DMA_CHANNEL_3                  (DMA_SCFGx_CHSEL_0 | DMA_SCFGx_CHSEL_1)                                         /* Select Channel3 of DMA Instance */
#define DDL_DMA_CHANNEL_4                  DMA_SCFGx_CHSEL_2                                                              /* Select Channel4 of DMA Instance */
#define DDL_DMA_CHANNEL_5                  (DMA_SCFGx_CHSEL_2 | DMA_SCFGx_CHSEL_0)                                         /* Select Channel5 of DMA Instance */
#define DDL_DMA_CHANNEL_6                  (DMA_SCFGx_CHSEL_2 | DMA_SCFGx_CHSEL_1)                                         /* Select Channel6 of DMA Instance */
#define DDL_DMA_CHANNEL_7                  (DMA_SCFGx_CHSEL_2 | DMA_SCFGx_CHSEL_1 | DMA_SCFGx_CHSEL_0)                      /* Select Channel7 of DMA Instance */
#if defined (DMA_SCFGx_CHSEL_3)
#define DDL_DMA_CHANNEL_8                  DMA_SCFGx_CHSEL_3                                                              /* Select Channel8 of DMA Instance */
#define DDL_DMA_CHANNEL_9                  (DMA_SCFGx_CHSEL_3 | DMA_SCFGx_CHSEL_0)                                         /* Select Channel9 of DMA Instance */
#define DDL_DMA_CHANNEL_10                 (DMA_SCFGx_CHSEL_3 | DMA_SCFGx_CHSEL_1)                                         /* Select Channel10 of DMA Instance */
#define DDL_DMA_CHANNEL_11                 (DMA_SCFGx_CHSEL_3 | DMA_SCFGx_CHSEL_1 | DMA_SCFGx_CHSEL_0)                      /* Select Channel11 of DMA Instance */
#define DDL_DMA_CHANNEL_12                 (DMA_SCFGx_CHSEL_3 | DMA_SCFGx_CHSEL_2)                                         /* Select Channel12 of DMA Instance */
#define DDL_DMA_CHANNEL_13                 (DMA_SCFGx_CHSEL_3 | DMA_SCFGx_CHSEL_2 | DMA_SCFGx_CHSEL_0)                      /* Select Channel13 of DMA Instance */
#define DDL_DMA_CHANNEL_14                 (DMA_SCFGx_CHSEL_3 | DMA_SCFGx_CHSEL_2 | DMA_SCFGx_CHSEL_1)                      /* Select Channel14 of DMA Instance */
#define DDL_DMA_CHANNEL_15                 (DMA_SCFGx_CHSEL_3 | DMA_SCFGx_CHSEL_2 | DMA_SCFGx_CHSEL_1 | DMA_SCFGx_CHSEL_0)   /* Select Channel15 of DMA Instance */
#endif /* DMA_SCFGx_CHSEL_3 */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_MBURST MBURST
  * @{
  */
#define DDL_DMA_MBURST_SINGLE              0x00000000U                             /*!< Memory burst single transfer configuration */
#define DDL_DMA_MBURST_INC4                DMA_SCFGx_MBCFG_0                       /*!< Memory burst of 4 beats transfer configuration */
#define DDL_DMA_MBURST_INC8                DMA_SCFGx_MBCFG_1                       /*!< Memory burst of 8 beats transfer configuration */
#define DDL_DMA_MBURST_INC16               (DMA_SCFGx_MBCFG_0 | DMA_SCFGx_MBCFG_1) /*!< Memory burst of 16 beats transfer configuration */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_PBURST PBURST
  * @{
  */
#define DDL_DMA_PBURST_SINGLE              0x00000000U                             /*!< Peripheral burst single transfer configuration */
#define DDL_DMA_PBURST_INC4                DMA_SCFGx_PBCFG_0                       /*!< Peripheral burst of 4 beats transfer configuration */
#define DDL_DMA_PBURST_INC8                DMA_SCFGx_PBCFG_1                       /*!< Peripheral burst of 8 beats transfer configuration */
#define DDL_DMA_PBURST_INC16               (DMA_SCFGx_PBCFG_0 | DMA_SCFGx_PBCFG_1) /*!< Peripheral burst of 16 beats transfer configuration */
/**
  * @}
  */
  
/** @defgroup DMA_DDL_FIFOMODE DMA_DDL_FIFOMODE
  * @{
  */
#define DDL_DMA_FIFOMODE_DISABLE           0x00000000U                             /*!< FIFO mode disable (direct mode is enabled) */
#define DDL_DMA_FIFOMODE_ENABLE            DMA_FCTRLx_DMDEN                         /*!< FIFO mode enable  */
/**
  * @}
  */  

/** @defgroup DMA_DDL_EC_FIFOSTATUS_0 FIFOSTATUS 0
  * @{
  */
#define DDL_DMA_FIFOSTATUS_0_25            0x00000000U                             /*!< 0 < fifo_level < 1/4    */
#define DDL_DMA_FIFOSTATUS_25_50           DMA_FCTRLx_FSTS_0                          /*!< 1/4 < fifo_level < 1/2  */
#define DDL_DMA_FIFOSTATUS_50_75           DMA_FCTRLx_FSTS_1                          /*!< 1/2 < fifo_level < 3/4  */
#define DDL_DMA_FIFOSTATUS_75_100          (DMA_FCTRLx_FSTS_1 | DMA_FCTRLx_FSTS_0)       /*!< 3/4 < fifo_level < full */
#define DDL_DMA_FIFOSTATUS_EMPTY           DMA_FCTRLx_FSTS_2                          /*!< FIFO is empty           */
#define DDL_DMA_FIFOSTATUS_FULL            (DMA_FCTRLx_FSTS_2 | DMA_FCTRLx_FSTS_0)       /*!< FIFO is full            */
/**
  * @}
  */

/** @defgroup DMA_DDL_EC_FIFOTHRESHOLD FIFOTHRESHOLD
  * @{
  */
#define DDL_DMA_FIFOTHRESHOLD_1_4          0x00000000U                             /*!< FIFO threshold 1 quart full configuration  */
#define DDL_DMA_FIFOTHRESHOLD_1_2          DMA_FCTRLx_FTHSEL_0                         /*!< FIFO threshold half full configuration     */
#define DDL_DMA_FIFOTHRESHOLD_3_4          DMA_FCTRLx_FTHSEL_1                         /*!< FIFO threshold 3 quarts full configuration */
#define DDL_DMA_FIFOTHRESHOLD_FULL         DMA_FCTRLx_FTHSEL                           /*!< FIFO threshold full configuration          */
/**
  * @}
  */
    
/** @defgroup DMA_DDL_EC_CURRENTTARGETMEM CURRENTTARGETMEM
  * @{
  */
#define DDL_DMA_CURRENTTARGETMEM0          0x00000000U                             /*!< Set CurrentTarget Memory to Memory 0  */
#define DDL_DMA_CURRENTTARGETMEM1          DMA_SCFGx_CTARG                             /*!< Set CurrentTarget Memory to Memory 1  */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DMA_DDL_Exported_Macros DMA Exported Macros
  * @{
  */

/** @defgroup DMA_DDL_EM_WRITE_READ Common Write and read registers macros
  * @{
  */
/**
  * @brief  Write a value in DMA register
  * @param  __INSTANCE__ DMA Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_DMA_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in DMA register
  * @param  __INSTANCE__ DMA Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_DMA_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/** @defgroup DMA_DDL_EM_CONVERT_DMAxCHANNELy Convert DMAxStreamy
  * @{
  */
/**
  * @brief  Convert DMAx_Streamy into DMAx
  * @param  __STREAM_INSTANCE__ DMAx_Streamy
  * @retval DMAx
  */
#define __DDL_DMA_GET_INSTANCE(__STREAM_INSTANCE__)   \
(((uint32_t)(__STREAM_INSTANCE__) > ((uint32_t)DMA1_Stream7)) ?  DMA2 : DMA1)

/**
  * @brief  Convert DMAx_Streamy into DDL_DMA_STREAM_y
  * @param  __STREAM_INSTANCE__ DMAx_Streamy
  * @retval DDL_DMA_CHANNEL_y
  */
#define __DDL_DMA_GET_STREAM(__STREAM_INSTANCE__)   \
(((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA1_Stream0)) ? DDL_DMA_STREAM_0 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA2_Stream0)) ? DDL_DMA_STREAM_0 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA1_Stream1)) ? DDL_DMA_STREAM_1 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA2_Stream1)) ? DDL_DMA_STREAM_1 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA1_Stream2)) ? DDL_DMA_STREAM_2 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA2_Stream2)) ? DDL_DMA_STREAM_2 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA1_Stream3)) ? DDL_DMA_STREAM_3 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA2_Stream3)) ? DDL_DMA_STREAM_3 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA1_Stream4)) ? DDL_DMA_STREAM_4 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA2_Stream4)) ? DDL_DMA_STREAM_4 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA1_Stream5)) ? DDL_DMA_STREAM_5 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA2_Stream5)) ? DDL_DMA_STREAM_5 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA1_Stream6)) ? DDL_DMA_STREAM_6 : \
 ((uint32_t)(__STREAM_INSTANCE__) == ((uint32_t)DMA2_Stream6)) ? DDL_DMA_STREAM_6 : \
 DDL_DMA_STREAM_7)

/**
  * @brief  Convert DMA Instance DMAx and DDL_DMA_STREAM_y into DMAx_Streamy
  * @param  __DMA_INSTANCE__ DMAx
  * @param  __STREAM__ DDL_DMA_STREAM_y
  * @retval DMAx_Streamy
  */
#define __DDL_DMA_GET_STREAM_INSTANCE(__DMA_INSTANCE__, __STREAM__)   \
((((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA1)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_0))) ? DMA1_Stream0 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA2)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_0))) ? DMA2_Stream0 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA1)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_1))) ? DMA1_Stream1 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA2)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_1))) ? DMA2_Stream1 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA1)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_2))) ? DMA1_Stream2 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA2)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_2))) ? DMA2_Stream2 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA1)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_3))) ? DMA1_Stream3 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA2)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_3))) ? DMA2_Stream3 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA1)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_4))) ? DMA1_Stream4 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA2)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_4))) ? DMA2_Stream4 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA1)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_5))) ? DMA1_Stream5 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA2)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_5))) ? DMA2_Stream5 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA1)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_6))) ? DMA1_Stream6 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA2)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_6))) ? DMA2_Stream6 : \
 (((uint32_t)(__DMA_INSTANCE__) == ((uint32_t)DMA1)) && ((uint32_t)(__STREAM__) == ((uint32_t)DDL_DMA_STREAM_7))) ? DMA1_Stream7 : \
 DMA2_Stream7)

/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
 /** @defgroup DMA_DDL_Exported_Functions DMA Exported Functions
  * @{
  */

/** @defgroup DMA_DDL_EF_Configuration Configuration
  * @{
  */
/**
  * @brief Enable DMA stream.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableStream(DMA_TypeDef *DMAx, uint32_t Stream)
{
  SET_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_EN);
}

/**
  * @brief Disable DMA stream.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableStream(DMA_TypeDef *DMAx, uint32_t Stream)
{
  CLEAR_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_EN);
}

/**
  * @brief Check if DMA stream is enabled or disabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledStream(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_EN) == (DMA_SCFGx_EN));
}

/**
  * @brief  Configure all parameters linked to DMA transfer.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Configuration This parameter must be a combination of all the following values:
  *         @arg @ref DDL_DMA_DIRECTION_PERIPH_TO_MEMORY or @ref DDL_DMA_DIRECTION_MEMORY_TO_PERIPH or @ref DDL_DMA_DIRECTION_MEMORY_TO_MEMORY
  *         @arg @ref DDL_DMA_MODE_NORMAL or @ref DDL_DMA_MODE_CIRCULAR  or @ref DDL_DMA_MODE_PFCTRL
  *         @arg @ref DDL_DMA_PERIPH_INCREMENT or @ref DDL_DMA_PERIPH_NOINCREMENT
  *         @arg @ref DDL_DMA_MEMORY_INCREMENT or @ref DDL_DMA_MEMORY_NOINCREMENT
  *         @arg @ref DDL_DMA_PDATAALIGN_BYTE or @ref DDL_DMA_PDATAALIGN_HALFWORD or @ref DDL_DMA_PDATAALIGN_WORD
  *         @arg @ref DDL_DMA_MDATAALIGN_BYTE or @ref DDL_DMA_MDATAALIGN_HALFWORD or @ref DDL_DMA_MDATAALIGN_WORD
  *         @arg @ref DDL_DMA_PRIORITY_LOW or @ref DDL_DMA_PRIORITY_MEDIUM or @ref DDL_DMA_PRIORITY_HIGH or @ref DDL_DMA_PRIORITY_VERYHIGH
  *@retval None
  */
__STATIC_INLINE void DDL_DMA_ConfigTransfer(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Configuration)
{
  MODIFY_REG(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG,
             DMA_SCFGx_DIRCFG | DMA_SCFGx_CIRCMEN | DMA_SCFGx_PERIM | DMA_SCFGx_MEMIM | DMA_SCFGx_PERSIZECFG | DMA_SCFGx_MEMSIZECFG | DMA_SCFGx_PRILCFG | DMA_SCFGx_PERFC,
             Configuration);
}

/**
  * @brief Set Data transfer direction (read from peripheral or from memory).
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Direction This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_MEMORY
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetDataTransferDirection(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t  Direction)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_DIRCFG, Direction);
}

/**
  * @brief Get Data transfer direction (read from peripheral or from memory).
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_MEMORY
  */
__STATIC_INLINE uint32_t DDL_DMA_GetDataTransferDirection(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_DIRCFG));
}

/**
  * @brief Set DMA mode normal, circular or peripheral flow control.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_MODE_NORMAL
  *         @arg @ref DDL_DMA_MODE_CIRCULAR
  *         @arg @ref DDL_DMA_MODE_PFCTRL
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMode(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Mode)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_CIRCMEN | DMA_SCFGx_PERFC, Mode);
}

/**
  * @brief Get DMA mode normal, circular or peripheral flow control.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_MODE_NORMAL
  *         @arg @ref DDL_DMA_MODE_CIRCULAR
  *         @arg @ref DDL_DMA_MODE_PFCTRL
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_CIRCMEN | DMA_SCFGx_PERFC));
}

/**
  * @brief Set Peripheral increment mode.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  IncrementMode This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_PERIPH_NOINCREMENT
  *         @arg @ref DDL_DMA_PERIPH_INCREMENT
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetPeriphIncMode(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t IncrementMode)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_PERIM, IncrementMode);
}

/**
  * @brief Get Peripheral increment mode.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_PERIPH_NOINCREMENT
  *         @arg @ref DDL_DMA_PERIPH_INCREMENT
  */
__STATIC_INLINE uint32_t DDL_DMA_GetPeriphIncMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_PERIM));
}

/**
  * @brief Set Memory increment mode.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  IncrementMode This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_MEMORY_NOINCREMENT
  *         @arg @ref DDL_DMA_MEMORY_INCREMENT
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMemoryIncMode(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t IncrementMode)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_MEMIM, IncrementMode);
}

/**
  * @brief Get Memory increment mode.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_MEMORY_NOINCREMENT
  *         @arg @ref DDL_DMA_MEMORY_INCREMENT
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMemoryIncMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_MEMIM));
}

/**
  * @brief Set Peripheral size.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Size This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_PDATAALIGN_BYTE
  *         @arg @ref DDL_DMA_PDATAALIGN_HALFWORD
  *         @arg @ref DDL_DMA_PDATAALIGN_WORD
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetPeriphSize(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t  Size)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_PERSIZECFG, Size);
}

/**
  * @brief Get Peripheral size.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_PDATAALIGN_BYTE
  *         @arg @ref DDL_DMA_PDATAALIGN_HALFWORD
  *         @arg @ref DDL_DMA_PDATAALIGN_WORD
  */
__STATIC_INLINE uint32_t DDL_DMA_GetPeriphSize(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_PERSIZECFG));
}

/**
  * @brief Set Memory size.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Size This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_MDATAALIGN_BYTE
  *         @arg @ref DDL_DMA_MDATAALIGN_HALFWORD
  *         @arg @ref DDL_DMA_MDATAALIGN_WORD
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMemorySize(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t  Size)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_MEMSIZECFG, Size);
}

/**
  * @brief Get Memory size.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_MDATAALIGN_BYTE
  *         @arg @ref DDL_DMA_MDATAALIGN_HALFWORD
  *         @arg @ref DDL_DMA_MDATAALIGN_WORD
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMemorySize(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_MEMSIZECFG));
}

/**
  * @brief Set Peripheral increment offset size.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  OffsetSize This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_OFFSETSIZE_PSIZE
  *         @arg @ref DDL_DMA_OFFSETSIZE_FIXEDTO4
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetIncOffsetSize(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t OffsetSize)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_PERIOSIZE, OffsetSize);
}

/**
  * @brief Get Peripheral increment offset size.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_OFFSETSIZE_PSIZE
  *         @arg @ref DDL_DMA_OFFSETSIZE_FIXEDTO4
  */
__STATIC_INLINE uint32_t DDL_DMA_GetIncOffsetSize(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_PERIOSIZE));
}

/**
  * @brief Set Stream priority level.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Priority This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_PRIORITY_LOW
  *         @arg @ref DDL_DMA_PRIORITY_MEDIUM
  *         @arg @ref DDL_DMA_PRIORITY_HIGH
  *         @arg @ref DDL_DMA_PRIORITY_VERYHIGH
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetStreamPriorityLevel(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t  Priority)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_PRILCFG, Priority);
}

/**
  * @brief Get Stream priority level.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_PRIORITY_LOW
  *         @arg @ref DDL_DMA_PRIORITY_MEDIUM
  *         @arg @ref DDL_DMA_PRIORITY_HIGH
  *         @arg @ref DDL_DMA_PRIORITY_VERYHIGH
  */
__STATIC_INLINE uint32_t DDL_DMA_GetStreamPriorityLevel(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_PRILCFG));
}

/**
  * @brief Set Number of data to transfer.
  * @note   This action has no effect if
  *         stream is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  NbData Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetDataLength(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t NbData)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->NDATA, DMA_NDATAx, NbData);
}

/**
  * @brief Get Number of data to transfer.
  * @note   Once the stream is enabled, the return value indicate the
  *         remaining bytes to be transmitted.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetDataLength(DMA_TypeDef* DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->NDATA, DMA_NDATAx));
}

/**
  * @brief Select Channel number associated to the Stream.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  *         @arg @ref DDL_DMA_CHANNEL_2
  *         @arg @ref DDL_DMA_CHANNEL_3
  *         @arg @ref DDL_DMA_CHANNEL_4
  *         @arg @ref DDL_DMA_CHANNEL_5
  *         @arg @ref DDL_DMA_CHANNEL_6
  *         @arg @ref DDL_DMA_CHANNEL_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetChannelSelection(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Channel)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_CHSEL, Channel);
}

/**
  * @brief Get the Channel number associated to the Stream.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_CHANNEL_0
  *         @arg @ref DDL_DMA_CHANNEL_1
  *         @arg @ref DDL_DMA_CHANNEL_2
  *         @arg @ref DDL_DMA_CHANNEL_3
  *         @arg @ref DDL_DMA_CHANNEL_4
  *         @arg @ref DDL_DMA_CHANNEL_5
  *         @arg @ref DDL_DMA_CHANNEL_6
  *         @arg @ref DDL_DMA_CHANNEL_7
  */
__STATIC_INLINE uint32_t DDL_DMA_GetChannelSelection(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_CHSEL));
}

/**
  * @brief Set Memory burst transfer configuration.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Mburst This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_MBURST_SINGLE
  *         @arg @ref DDL_DMA_MBURST_INC4
  *         @arg @ref DDL_DMA_MBURST_INC8
  *         @arg @ref DDL_DMA_MBURST_INC16
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMemoryBurstxfer(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Mburst)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_MBCFG, Mburst);
}

/**
  * @brief Get Memory burst transfer configuration.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_MBURST_SINGLE
  *         @arg @ref DDL_DMA_MBURST_INC4
  *         @arg @ref DDL_DMA_MBURST_INC8
  *         @arg @ref DDL_DMA_MBURST_INC16
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMemoryBurstxfer(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_MBCFG));
}

/**
  * @brief Set  Peripheral burst transfer configuration.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Pburst This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_PBURST_SINGLE
  *         @arg @ref DDL_DMA_PBURST_INC4
  *         @arg @ref DDL_DMA_PBURST_INC8
  *         @arg @ref DDL_DMA_PBURST_INC16
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetPeriphBurstxfer(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Pburst)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_PBCFG, Pburst);
}

/**
  * @brief Get Peripheral burst transfer configuration.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_PBURST_SINGLE
  *         @arg @ref DDL_DMA_PBURST_INC4
  *         @arg @ref DDL_DMA_PBURST_INC8
  *         @arg @ref DDL_DMA_PBURST_INC16
  */
__STATIC_INLINE uint32_t DDL_DMA_GetPeriphBurstxfer(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_PBCFG));
}

/**
  * @brief Set Current target (only in double buffer mode) to Memory 1 or Memory 0.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param CurrentMemory This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_CURRENTTARGETMEM0
  *         @arg @ref DDL_DMA_CURRENTTARGETMEM1
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetCurrentTargetMem(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t CurrentMemory)
{
   MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_CTARG, CurrentMemory);
}

/**
  * @brief Set Current target (only in double buffer mode) to Memory 1 or Memory 0.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_CURRENTTARGETMEM0
  *         @arg @ref DDL_DMA_CURRENTTARGETMEM1
  */
__STATIC_INLINE uint32_t DDL_DMA_GetCurrentTargetMem(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_CTARG));
}

/**
  * @brief Enable the double buffer mode.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableDoubleBufferMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  SET_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_DBM);
}

/**
  * @brief Disable the double buffer mode.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableDoubleBufferMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  CLEAR_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_DBM);
}

/**
  * @brief Get FIFO status.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_FIFOSTATUS_0_25
  *         @arg @ref DDL_DMA_FIFOSTATUS_25_50
  *         @arg @ref DDL_DMA_FIFOSTATUS_50_75
  *         @arg @ref DDL_DMA_FIFOSTATUS_75_100
  *         @arg @ref DDL_DMA_FIFOSTATUS_EMPTY
  *         @arg @ref DDL_DMA_FIFOSTATUS_FULL
  */
__STATIC_INLINE uint32_t DDL_DMA_GetFIFOStatus(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCTRL, DMA_FCTRLx_FSTS));
}

/**
  * @brief Disable Fifo mode.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableFifoMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  CLEAR_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCTRL, DMA_FCTRLx_DMDEN);
}

/**
  * @brief Enable Fifo mode.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableFifoMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  SET_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCTRL, DMA_FCTRLx_DMDEN);
}

/**
  * @brief Select FIFO threshold.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Threshold This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_2
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_3_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_FULL
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetFIFOThreshold(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Threshold)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCTRL, DMA_FCTRLx_FTHSEL, Threshold);
}

/**
  * @brief Get FIFO threshold.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_2
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_3_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_FULL
  */
__STATIC_INLINE uint32_t DDL_DMA_GetFIFOThreshold(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCTRL, DMA_FCTRLx_FTHSEL));
}

/**
  * @brief Configure the FIFO .
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  FifoMode This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_FIFOMODE_ENABLE
  *         @arg @ref DDL_DMA_FIFOMODE_DISABLE
  * @param  FifoThreshold This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_1_2
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_3_4
  *         @arg @ref DDL_DMA_FIFOTHRESHOLD_FULL
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ConfigFifo(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t FifoMode, uint32_t FifoThreshold)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCTRL, DMA_FCTRLx_FTHSEL|DMA_FCTRLx_DMDEN, FifoMode|FifoThreshold);
}

/**
  * @brief Configure the Source and Destination addresses.
  * @note   This API must not be called when the DMA stream is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  SrcAddress Between 0 to 0xFFFFFFFF
  * @param  DstAddress Between 0 to 0xFFFFFFFF
  * @param  Direction This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_DIRECTION_PERIPH_TO_MEMORY
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_PERIPH
  *         @arg @ref DDL_DMA_DIRECTION_MEMORY_TO_MEMORY
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ConfigAddresses(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Direction)
{
  /* Direction Memory to Periph */
  if (Direction == DDL_DMA_DIRECTION_MEMORY_TO_PERIPH)
  {
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0ADDR, SrcAddress);
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PADDR, DstAddress);
  }
  /* Direction Periph to Memory and Memory to Memory */
  else
  {
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PADDR, SrcAddress);
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0ADDR, DstAddress);
  }
}

/**
  * @brief  Set the Memory address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_PERIPH_TO_MEMORY or DDL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  MemoryAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMemoryAddress(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t MemoryAddress)
{
  WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0ADDR, MemoryAddress);
}

/**
  * @brief  Set the Peripheral address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_PERIPH_TO_MEMORY or DDL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  PeriphAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetPeriphAddress(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t PeriphAddress)
{
  WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PADDR, PeriphAddress);
}

/**
  * @brief  Get the Memory address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_PERIPH_TO_MEMORY or DDL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMemoryAddress(DMA_TypeDef* DMAx, uint32_t Stream)
{
  return (READ_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0ADDR));
}

/**
  * @brief  Get the Peripheral address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_PERIPH_TO_MEMORY or DDL_DMA_DIRECTION_MEMORY_TO_PERIPH only.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetPeriphAddress(DMA_TypeDef* DMAx, uint32_t Stream)
{
  return (READ_REG(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PADDR));
}

/**
  * @brief  Set the Memory to Memory Source address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  MemoryAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetM2MSrcAddress(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t MemoryAddress)
{
  WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PADDR, MemoryAddress);
}

/**
  * @brief  Set the Memory to Memory Destination address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @note   This API must not be called when the DMA channel is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  MemoryAddress Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetM2MDstAddress(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t MemoryAddress)
  {
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0ADDR, MemoryAddress);
  }

/**
  * @brief  Get the Memory to Memory Source address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetM2MSrcAddress(DMA_TypeDef* DMAx, uint32_t Stream)
  {
   return (READ_REG(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PADDR));
  }

/**
  * @brief  Get the Memory to Memory Destination address.
  * @note   Interface used for direction DDL_DMA_DIRECTION_MEMORY_TO_MEMORY only.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetM2MDstAddress(DMA_TypeDef* DMAx, uint32_t Stream)
{
 return (READ_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0ADDR));
}

/**
  * @brief Set Memory 1 address (used in case of Double buffer mode).
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @param  Address Between 0 to 0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_SetMemory1Address(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Address)
{
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M1ADDR, DMA_M1ADDRx_M1ADDR, Address);
}

/**
  * @brief Get Memory 1 address (used in case of Double buffer mode).
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval Between 0 to 0xFFFFFFFF
  */
__STATIC_INLINE uint32_t DDL_DMA_GetMemory1Address(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M1ADDR);
}

/**
  * @}
  */

/** @defgroup DMA_DDL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief Get Stream 0 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_HT0(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_HTXIFLG0)==(DMA_LINTSTS_HTXIFLG0));
}

/**
  * @brief Get Stream 1 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_HT1(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_HTXIFLG1)==(DMA_LINTSTS_HTXIFLG1));
}

/**
  * @brief Get Stream 2 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_HT2(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_HTXIFLG2)==(DMA_LINTSTS_HTXIFLG2));
}

/**
  * @brief Get Stream 3 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_HT3(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_HTXIFLG3)==(DMA_LINTSTS_HTXIFLG3));
}

/**
  * @brief Get Stream 4 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_HT4(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_HTXIFLG4)==(DMA_HINTSTS_HTXIFLG4));
}

/**
  * @brief Get Stream 5 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_HT5(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_HTXIFLG5)==(DMA_HINTSTS_HTXIFLG5));
}

/**
  * @brief Get Stream 6 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_HT6(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_HTXIFLG6)==(DMA_HINTSTS_HTXIFLG6));
}

/**
  * @brief Get Stream 7 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_HT7(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_HTXIFLG7)==(DMA_HINTSTS_HTXIFLG7));
} 

/**
  * @brief Get Stream 0 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TC0(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_TXCIFLG0)==(DMA_LINTSTS_TXCIFLG0));
}

/**
  * @brief Get Stream 1 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TC1(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_TXCIFLG1)==(DMA_LINTSTS_TXCIFLG1));
}

/**
  * @brief Get Stream 2 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TC2(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_TXCIFLG2)==(DMA_LINTSTS_TXCIFLG2));
}

/**
  * @brief Get Stream 3 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TC3(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_TXCIFLG3)==(DMA_LINTSTS_TXCIFLG3));
}

/**
  * @brief Get Stream 4 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TC4(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_TXCIFLG4)==(DMA_HINTSTS_TXCIFLG4));
}

/**
  * @brief Get Stream 5 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TC5(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_TXCIFLG5)==(DMA_HINTSTS_TXCIFLG5));
}

/**
  * @brief Get Stream 6 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TC6(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_TXCIFLG6)==(DMA_HINTSTS_TXCIFLG6));
}

/**
  * @brief Get Stream 7 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TC7(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_TXCIFLG7)==(DMA_HINTSTS_TXCIFLG7));
} 

/**
  * @brief Get Stream 0 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TE0(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_TXEIFLG0)==(DMA_LINTSTS_TXEIFLG0));
}

/**
  * @brief Get Stream 1 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TE1(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_TXEIFLG1)==(DMA_LINTSTS_TXEIFLG1));
}

/**
  * @brief Get Stream 2 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TE2(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_TXEIFLG2)==(DMA_LINTSTS_TXEIFLG2));
}

/**
  * @brief Get Stream 3 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TE3(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_TXEIFLG3)==(DMA_LINTSTS_TXEIFLG3));
}

/**
  * @brief Get Stream 4 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TE4(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_TXEIFLG4)==(DMA_HINTSTS_TXEIFLG4));
}

/**
  * @brief Get Stream 5 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TE5(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_TXEIFLG5)==(DMA_HINTSTS_TXEIFLG5));
}

/**
  * @brief Get Stream 6 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TE6(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_TXEIFLG6)==(DMA_HINTSTS_TXEIFLG6));
}

/**
  * @brief Get Stream 7 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_TE7(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_TXEIFLG7)==(DMA_HINTSTS_TXEIFLG7));
} 

/**
  * @brief Get Stream 0 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_DME0(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_DMEIFLG0)==(DMA_LINTSTS_DMEIFLG0));
}

/**
  * @brief Get Stream 1 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_DME1(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_DMEIFLG1)==(DMA_LINTSTS_DMEIFLG1));
}

/**
  * @brief Get Stream 2 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_DME2(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_DMEIFLG2)==(DMA_LINTSTS_DMEIFLG2));
}

/**
  * @brief Get Stream 3 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_DME3(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_DMEIFLG3)==(DMA_LINTSTS_DMEIFLG3));
}

/**
  * @brief Get Stream 4 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_DME4(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_DMEIFLG4)==(DMA_HINTSTS_DMEIFLG4));
}

/**
  * @brief Get Stream 5 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_DME5(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_DMEIFLG5)==(DMA_HINTSTS_DMEIFLG5));
}

/**
  * @brief Get Stream 6 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_DME6(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_DMEIFLG6)==(DMA_HINTSTS_DMEIFLG6));
}

/**
  * @brief Get Stream 7 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_DME7(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_DMEIFLG7)==(DMA_HINTSTS_DMEIFLG7));
}

/**
  * @brief Get Stream 0 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_FE0(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_FEIFLG0)==(DMA_LINTSTS_FEIFLG0));
}

/**
  * @brief Get Stream 1 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_FE1(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_FEIFLG1)==(DMA_LINTSTS_FEIFLG1));
}

/**
  * @brief Get Stream 2 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_FE2(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_FEIFLG2)==(DMA_LINTSTS_FEIFLG2));
}

/**
  * @brief Get Stream 3 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_FE3(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->LINTSTS ,DMA_LINTSTS_FEIFLG3)==(DMA_LINTSTS_FEIFLG3));
}

/**
  * @brief Get Stream 4 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_FE4(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_FEIFLG4)==(DMA_HINTSTS_FEIFLG4));
}

/**
  * @brief Get Stream 5 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_FE5(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_FEIFLG5)==(DMA_HINTSTS_FEIFLG5));
}

/**
  * @brief Get Stream 6 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_FE6(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_FEIFLG6)==(DMA_HINTSTS_FEIFLG6));
}

/**
  * @brief Get Stream 7 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsActiveFlag_FE7(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->HINTSTS ,DMA_HINTSTS_FEIFLG7)==(DMA_HINTSTS_FEIFLG7));
}

/**
  * @brief Clear Stream 0 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_HT0(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CHTXIFLG0);
}

/**
  * @brief Clear Stream 1 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_HT1(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CHTXIFLG1);
}

/**
  * @brief Clear Stream 2 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_HT2(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CHTXIFLG2);
}

/**
  * @brief Clear Stream 3 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_HT3(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CHTXIFLG3);
}

/**
  * @brief Clear Stream 4 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_HT4(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CHTXIFLG4);
}

/**
  * @brief Clear Stream 5 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_HT5(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CHTXIFLG5);
}

/**
  * @brief Clear Stream 6 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_HT6(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CHTXIFLG6);
}

/**
  * @brief Clear Stream 7 half transfer flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_HT7(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CHTXIFLG7);
}

/**
  * @brief Clear Stream 0 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TC0(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CTXCIFLG0);
}

/**
  * @brief Clear Stream 1 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TC1(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CTXCIFLG1);
}

/**
  * @brief Clear Stream 2 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TC2(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CTXCIFLG2);
}

/**
  * @brief Clear Stream 3 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TC3(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CTXCIFLG3);
}

/**
  * @brief Clear Stream 4 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TC4(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CTXCIFLG4);
}

/**
  * @brief Clear Stream 5 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TC5(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CTXCIFLG5);
}

/**
  * @brief Clear Stream 6 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TC6(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CTXCIFLG6);
}

/**
  * @brief Clear Stream 7 transfer complete flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TC7(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CTXCIFLG7);
}

/**
  * @brief Clear Stream 0 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TE0(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CTXEIFLG0);
}

/**
  * @brief Clear Stream 1 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TE1(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CTXEIFLG1);
}

/**
  * @brief Clear Stream 2 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TE2(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CTXEIFLG2);
}

/**
  * @brief Clear Stream 3 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TE3(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CTXEIFLG3);
}

/**
  * @brief Clear Stream 4 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TE4(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CTXEIFLG4);
}

/**
  * @brief Clear Stream 5 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TE5(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CTXEIFLG5);
}

/**
  * @brief Clear Stream 6 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TE6(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CTXEIFLG6);
}

/**
  * @brief Clear Stream 7 transfer error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_TE7(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CTXEIFLG7);
}

/**
  * @brief Clear Stream 0 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_DME0(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CDMEIFLG0);
}

/**
  * @brief Clear Stream 1 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_DME1(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CDMEIFLG1);
}

/**
  * @brief Clear Stream 2 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_DME2(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CDMEIFLG2);
}

/**
  * @brief Clear Stream 3 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_DME3(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CDMEIFLG3);
}

/**
  * @brief Clear Stream 4 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_DME4(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CDMEIFLG4);
}

/**
  * @brief Clear Stream 5 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_DME5(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CDMEIFLG5);
}

/**
  * @brief Clear Stream 6 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_DME6(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CDMEIFLG6);
}

/**
  * @brief Clear Stream 7 direct mode error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_DME7(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CDMEIFLG7);
}

/**
  * @brief Clear Stream 0 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_FE0(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CFEIFLG0);
}

/**
  * @brief Clear Stream 1 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_FE1(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CFEIFLG1);
}

/**
  * @brief Clear Stream 2 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_FE2(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CFEIFLG2);
}

/**
  * @brief Clear Stream 3 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_FE3(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->LIFCLR , DMA_LIFCLR_CFEIFLG3);
}

/**
  * @brief Clear Stream 4 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_FE4(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CFEIFLG4);
}

/**
  * @brief Clear Stream 5 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_FE5(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CFEIFLG5);
}

/**
  * @brief Clear Stream 6 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_FE6(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CFEIFLG6);
}

/**
  * @brief Clear Stream 7 FIFO error flag.
  * @param  DMAx DMAx Instance
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_ClearFlag_FE7(DMA_TypeDef *DMAx)
{
  WRITE_REG(DMAx->HIFCLR , DMA_HIFCLR_CFEIFLG7);
}

/**
  * @}
  */

/** @defgroup DMA_DDL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief Enable Half transfer interrupt.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableIT_HT(DMA_TypeDef *DMAx, uint32_t Stream)
{
  SET_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_HTXIEN);
}

/**
  * @brief Enable Transfer error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableIT_TE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  SET_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_TXEIEN);
}

/**
  * @brief Enable Transfer complete interrupt.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableIT_TC(DMA_TypeDef *DMAx, uint32_t Stream)
{
  SET_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_TXCIEN);
}

/**
  * @brief Enable Direct mode error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableIT_DME(DMA_TypeDef *DMAx, uint32_t Stream)
{
  SET_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_DMEIEN);
}

/**
  * @brief Enable FIFO error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_EnableIT_FE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  SET_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCTRL, DMA_FCTRLx_FEIEN);
}

/**
  * @brief Disable Half transfer interrupt.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableIT_HT(DMA_TypeDef *DMAx, uint32_t Stream)
{
  CLEAR_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_HTXIEN);
}

/**
  * @brief Disable Transfer error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableIT_TE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  CLEAR_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_TXEIEN);
}

/**
  * @brief Disable Transfer complete interrupt.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableIT_TC(DMA_TypeDef *DMAx, uint32_t Stream)
{
  CLEAR_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_TXCIEN);
}

/**
  * @brief Disable Direct mode error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableIT_DME(DMA_TypeDef *DMAx, uint32_t Stream)
{
  CLEAR_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_DMEIEN);
}

/**
  * @brief Disable FIFO error interrupt.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval None
  */
__STATIC_INLINE void DDL_DMA_DisableIT_FE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  CLEAR_BIT(((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCTRL, DMA_FCTRLx_FEIEN);
}

/**
  * @brief Check if Half transfer interrupt is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledIT_HT(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_HTXIEN) == DMA_SCFGx_HTXIEN);
}

/**
  * @brief Check if Transfer error nterrup is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledIT_TE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_TXEIEN) == DMA_SCFGx_TXEIEN);
}

/**
  * @brief Check if Transfer complete interrupt is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledIT_TC(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_TXCIEN) == DMA_SCFGx_TXCIEN);
}

/**
  * @brief Check if Direct mode error interrupt is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledIT_DME(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SCFG, DMA_SCFGx_DMEIEN) == DMA_SCFGx_DMEIEN);
}

/**
  * @brief Check if FIFO error interrupt is enabled.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref DDL_DMA_STREAM_0
  *         @arg @ref DDL_DMA_STREAM_1
  *         @arg @ref DDL_DMA_STREAM_2
  *         @arg @ref DDL_DMA_STREAM_3
  *         @arg @ref DDL_DMA_STREAM_4
  *         @arg @ref DDL_DMA_STREAM_5
  *         @arg @ref DDL_DMA_STREAM_6
  *         @arg @ref DDL_DMA_STREAM_7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_DMA_IsEnabledIT_FE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (READ_BIT(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCTRL, DMA_FCTRLx_FEIEN) == DMA_FCTRLx_FEIEN);
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup DMA_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

uint32_t DDL_DMA_Init(DMA_TypeDef *DMAx, uint32_t Stream, DDL_DMA_InitTypeDef *DMA_InitStruct);
uint32_t DDL_DMA_DeInit(DMA_TypeDef *DMAx, uint32_t Stream);
void DDL_DMA_StructInit(DDL_DMA_InitTypeDef *DMA_InitStruct);

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* DMA1 || DMA2 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_DMA_H */

