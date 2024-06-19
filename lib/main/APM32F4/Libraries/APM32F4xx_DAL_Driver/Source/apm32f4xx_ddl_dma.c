/**
  *
  * @file    apm32f4xx_ddl_dma.c
  * @brief   DMA DDL module driver.
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
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_dma.h"
#include "apm32f4xx_ddl_bus.h"
#ifdef  USE_FULL_ASSERT
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (DMA1) || defined (DMA2)

/** @defgroup DMA_DDL DMA
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup DMA_DDL_Private_Macros
  * @{
  */
#define IS_DDL_DMA_DIRECTION(__VALUE__)          (((__VALUE__) == DDL_DMA_DIRECTION_PERIPH_TO_MEMORY) || \
                                                 ((__VALUE__) == DDL_DMA_DIRECTION_MEMORY_TO_PERIPH) || \
                                                 ((__VALUE__) == DDL_DMA_DIRECTION_MEMORY_TO_MEMORY))

#define IS_DDL_DMA_MODE(__VALUE__)               (((__VALUE__) == DDL_DMA_MODE_NORMAL)    || \
                                                 ((__VALUE__) == DDL_DMA_MODE_CIRCULAR)  || \
                                                 ((__VALUE__) == DDL_DMA_MODE_PFCTRL))

#define IS_DDL_DMA_PERIPHINCMODE(__VALUE__)      (((__VALUE__) == DDL_DMA_PERIPH_INCREMENT) || \
                                                 ((__VALUE__) == DDL_DMA_PERIPH_NOINCREMENT))

#define IS_DDL_DMA_MEMORYINCMODE(__VALUE__)      (((__VALUE__) == DDL_DMA_MEMORY_INCREMENT) || \
                                                 ((__VALUE__) == DDL_DMA_MEMORY_NOINCREMENT))

#define IS_DDL_DMA_PERIPHDATASIZE(__VALUE__)     (((__VALUE__) == DDL_DMA_PDATAALIGN_BYTE)      || \
                                                 ((__VALUE__) == DDL_DMA_PDATAALIGN_HALFWORD)  || \
                                                 ((__VALUE__) == DDL_DMA_PDATAALIGN_WORD))

#define IS_DDL_DMA_MEMORYDATASIZE(__VALUE__)     (((__VALUE__) == DDL_DMA_MDATAALIGN_BYTE)      || \
                                                 ((__VALUE__) == DDL_DMA_MDATAALIGN_HALFWORD)  || \
                                                 ((__VALUE__) == DDL_DMA_MDATAALIGN_WORD))

#define IS_DDL_DMA_NBDATA(__VALUE__)             ((__VALUE__)  <= 0x0000FFFFU)

#define IS_DDL_DMA_CHANNEL(__VALUE__)            (((__VALUE__) == DDL_DMA_CHANNEL_0)  || \
                                                 ((__VALUE__) == DDL_DMA_CHANNEL_1)  || \
                                                 ((__VALUE__) == DDL_DMA_CHANNEL_2)  || \
                                                 ((__VALUE__) == DDL_DMA_CHANNEL_3)  || \
                                                 ((__VALUE__) == DDL_DMA_CHANNEL_4)  || \
                                                 ((__VALUE__) == DDL_DMA_CHANNEL_5)  || \
                                                 ((__VALUE__) == DDL_DMA_CHANNEL_6)  || \
                                                 ((__VALUE__) == DDL_DMA_CHANNEL_7))

#define IS_DDL_DMA_PRIORITY(__VALUE__)           (((__VALUE__) == DDL_DMA_PRIORITY_LOW)    || \
                                                 ((__VALUE__) == DDL_DMA_PRIORITY_MEDIUM) || \
                                                 ((__VALUE__) == DDL_DMA_PRIORITY_HIGH)   || \
                                                 ((__VALUE__) == DDL_DMA_PRIORITY_VERYHIGH))

#define IS_DDL_DMA_ALL_STREAM_INSTANCE(INSTANCE, STREAM)   ((((INSTANCE) == DMA1) && \
                                                           (((STREAM) == DDL_DMA_STREAM_0) || \
                                                            ((STREAM) == DDL_DMA_STREAM_1) || \
                                                            ((STREAM) == DDL_DMA_STREAM_2) || \
                                                            ((STREAM) == DDL_DMA_STREAM_3) || \
                                                            ((STREAM) == DDL_DMA_STREAM_4) || \
                                                            ((STREAM) == DDL_DMA_STREAM_5) || \
                                                            ((STREAM) == DDL_DMA_STREAM_6) || \
                                                            ((STREAM) == DDL_DMA_STREAM_7) || \
                                                            ((STREAM) == DDL_DMA_STREAM_ALL))) ||\
                                                            (((INSTANCE) == DMA2) && \
                                                          (((STREAM) == DDL_DMA_STREAM_0) || \
                                                           ((STREAM) == DDL_DMA_STREAM_1) || \
                                                           ((STREAM) == DDL_DMA_STREAM_2) || \
                                                           ((STREAM) == DDL_DMA_STREAM_3) || \
                                                           ((STREAM) == DDL_DMA_STREAM_4) || \
                                                           ((STREAM) == DDL_DMA_STREAM_5) || \
                                                           ((STREAM) == DDL_DMA_STREAM_6) || \
                                                           ((STREAM) == DDL_DMA_STREAM_7) || \
                                                           ((STREAM) == DDL_DMA_STREAM_ALL))))

#define IS_DDL_DMA_FIFO_MODE_STATE(STATE) (((STATE) == DDL_DMA_FIFOMODE_DISABLE ) || \
                                          ((STATE) == DDL_DMA_FIFOMODE_ENABLE))

#define IS_DDL_DMA_FIFO_THRESHOLD(THRESHOLD) (((THRESHOLD) == DDL_DMA_FIFOTHRESHOLD_1_4) || \
                                             ((THRESHOLD) == DDL_DMA_FIFOTHRESHOLD_1_2)  || \
                                             ((THRESHOLD) == DDL_DMA_FIFOTHRESHOLD_3_4)  || \
                                             ((THRESHOLD) == DDL_DMA_FIFOTHRESHOLD_FULL))

#define IS_DDL_DMA_MEMORY_BURST(BURST) (((BURST) == DDL_DMA_MBURST_SINGLE) || \
                                       ((BURST) == DDL_DMA_MBURST_INC4)   || \
                                       ((BURST) == DDL_DMA_MBURST_INC8)   || \
                                       ((BURST) == DDL_DMA_MBURST_INC16))

#define IS_DDL_DMA_PERIPHERAL_BURST(BURST) (((BURST) == DDL_DMA_PBURST_SINGLE) || \
                                           ((BURST) == DDL_DMA_PBURST_INC4)   || \
                                           ((BURST) == DDL_DMA_PBURST_INC8)   || \
                                           ((BURST) == DDL_DMA_PBURST_INC16))

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup DMA_DDL_Exported_Functions
  * @{
  */

/** @addtogroup DMA_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the DMA registers to their default reset values.
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
  *         @arg @ref DDL_DMA_STREAM_ALL
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DMA registers are de-initialized
  *          - ERROR: DMA registers are not de-initialized
  */
uint32_t DDL_DMA_DeInit(DMA_TypeDef *DMAx, uint32_t Stream)
{
  DMA_Stream_TypeDef *tmp = (DMA_Stream_TypeDef *)DMA1_Stream0;
  ErrorStatus status = SUCCESS;

  /* Check the DMA Instance DMAx and Stream parameters*/
  ASSERT_PARAM(IS_DDL_DMA_ALL_STREAM_INSTANCE(DMAx, Stream));

  if (Stream == DDL_DMA_STREAM_ALL)
  {
    if (DMAx == DMA1)
    {
      /* Force reset of DMA clock */
      DDL_AHB1_GRP1_ForceReset(DDL_AHB1_GRP1_PERIPH_DMA1);

      /* Release reset of DMA clock */
      DDL_AHB1_GRP1_ReleaseReset(DDL_AHB1_GRP1_PERIPH_DMA1);
    }
    else if (DMAx == DMA2)
    {
      /* Force reset of DMA clock */
      DDL_AHB1_GRP1_ForceReset(DDL_AHB1_GRP1_PERIPH_DMA2);

      /* Release reset of DMA clock */
      DDL_AHB1_GRP1_ReleaseReset(DDL_AHB1_GRP1_PERIPH_DMA2);
    }
    else
    {
      status = ERROR;
    }
  }
  else
  {
    /* Disable the selected Stream */
    DDL_DMA_DisableStream(DMAx,Stream);

    /* Get the DMA Stream Instance */
    tmp = (DMA_Stream_TypeDef *)(__DDL_DMA_GET_STREAM_INSTANCE(DMAx, Stream));

    /* Reset DMAx_Streamy configuration register */
    DDL_DMA_WriteReg(tmp, SCFG, 0U);

    /* Reset DMAx_Streamy remaining bytes register */
    DDL_DMA_WriteReg(tmp, NDATA, 0U);

    /* Reset DMAx_Streamy peripheral address register */
    DDL_DMA_WriteReg(tmp, PADDR, 0U);

    /* Reset DMAx_Streamy memory address register */
    DDL_DMA_WriteReg(tmp, M0ADDR, 0U);

    /* Reset DMAx_Streamy memory address register */
    DDL_DMA_WriteReg(tmp, M1ADDR, 0U);

    /* Reset DMAx_Streamy FIFO control register */
    DDL_DMA_WriteReg(tmp, FCTRL, 0x00000021U);

    /* Reset Channel register field for DMAx Stream*/
    DDL_DMA_SetChannelSelection(DMAx, Stream, DDL_DMA_CHANNEL_0);

    if(Stream == DDL_DMA_STREAM_0)
    {
       /* Reset the Stream0 pending flags */
       DMAx->LIFCLR = 0x0000003FU;
    }
    else if(Stream == DDL_DMA_STREAM_1)
    {
       /* Reset the Stream1 pending flags */
       DMAx->LIFCLR = 0x00000F40U;
    }
    else if(Stream == DDL_DMA_STREAM_2)
    {
       /* Reset the Stream2 pending flags */
       DMAx->LIFCLR = 0x003F0000U;
    }
    else if(Stream == DDL_DMA_STREAM_3)
    {
       /* Reset the Stream3 pending flags */
       DMAx->LIFCLR = 0x0F400000U;
    }
    else if(Stream == DDL_DMA_STREAM_4)
    {
       /* Reset the Stream4 pending flags */
       DMAx->HIFCLR = 0x0000003FU;
    }
    else if(Stream == DDL_DMA_STREAM_5)
    {
       /* Reset the Stream5 pending flags */
       DMAx->HIFCLR = 0x00000F40U;
    }
    else if(Stream == DDL_DMA_STREAM_6)
    {
       /* Reset the Stream6 pending flags */
       DMAx->HIFCLR = 0x003F0000U;
    }
    else if(Stream == DDL_DMA_STREAM_7)
    {
       /* Reset the Stream7 pending flags */
       DMAx->HIFCLR = 0x0F400000U;
    }
    else
    {
      status = ERROR;
    }
  }

  return status;
}

/**
  * @brief  Initialize the DMA registers according to the specified parameters in DMA_InitStruct.
  * @note   To convert DMAx_Streamy Instance to DMAx Instance and Streamy, use helper macros :
  *         @arg @ref __DDL_DMA_GET_INSTANCE
  *         @arg @ref __DDL_DMA_GET_STREAM
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
  * @param  DMA_InitStruct pointer to a @ref DDL_DMA_InitTypeDef structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DMA registers are initialized
  *          - ERROR: Not applicable
  */
uint32_t DDL_DMA_Init(DMA_TypeDef *DMAx, uint32_t Stream, DDL_DMA_InitTypeDef *DMA_InitStruct)
{
  /* Check the DMA Instance DMAx and Stream parameters*/
  ASSERT_PARAM(IS_DDL_DMA_ALL_STREAM_INSTANCE(DMAx, Stream));

  /* Check the DMA parameters from DMA_InitStruct */
  ASSERT_PARAM(IS_DDL_DMA_DIRECTION(DMA_InitStruct->Direction));
  ASSERT_PARAM(IS_DDL_DMA_MODE(DMA_InitStruct->Mode));
  ASSERT_PARAM(IS_DDL_DMA_PERIPHINCMODE(DMA_InitStruct->PeriphOrM2MSrcIncMode));
  ASSERT_PARAM(IS_DDL_DMA_MEMORYINCMODE(DMA_InitStruct->MemoryOrM2MDstIncMode));
  ASSERT_PARAM(IS_DDL_DMA_PERIPHDATASIZE(DMA_InitStruct->PeriphOrM2MSrcDataSize));
  ASSERT_PARAM(IS_DDL_DMA_MEMORYDATASIZE(DMA_InitStruct->MemoryOrM2MDstDataSize));
  ASSERT_PARAM(IS_DDL_DMA_NBDATA(DMA_InitStruct->NbData));
  ASSERT_PARAM(IS_DDL_DMA_CHANNEL(DMA_InitStruct->Channel));
  ASSERT_PARAM(IS_DDL_DMA_PRIORITY(DMA_InitStruct->Priority));
  ASSERT_PARAM(IS_DDL_DMA_FIFO_MODE_STATE(DMA_InitStruct->FIFOMode));
  /* Check the memory burst, peripheral burst and FIFO threshold parameters only
     when FIFO mode is enabled */
  if(DMA_InitStruct->FIFOMode != DDL_DMA_FIFOMODE_DISABLE)
  {
    ASSERT_PARAM(IS_DDL_DMA_FIFO_THRESHOLD(DMA_InitStruct->FIFOThreshold));
    ASSERT_PARAM(IS_DDL_DMA_MEMORY_BURST(DMA_InitStruct->MemBurst));
    ASSERT_PARAM(IS_DDL_DMA_PERIPHERAL_BURST(DMA_InitStruct->PeriphBurst));
  }

  /*---------------------------- DMAx SCFGx Configuration ------------------------
   * Configure DMAx_Streamy: data transfer direction, data transfer mode,
   *                          peripheral and memory increment mode,
   *                          data size alignment and  priority level with parameters :
   * - Direction:      DMA_SCFGx_DIRCFG[1:0] bits
   * - Mode:           DMA_SCFGx_CIRCMEN bit
   * - PeriphOrM2MSrcIncMode:  DMA_SCFGx_PERIM bit
   * - MemoryOrM2MDstIncMode:  DMA_SCFGx_MEMIM bit
   * - PeriphOrM2MSrcDataSize: DMA_SCFGx_PERSIZECFG[1:0] bits
   * - MemoryOrM2MDstDataSize: DMA_SCFGx_MEMSIZECFG[1:0] bits
   * - Priority:               DMA_SCFGx_PRILCFG[1:0] bits
   */
  DDL_DMA_ConfigTransfer(DMAx, Stream, DMA_InitStruct->Direction | \
                        DMA_InitStruct->Mode                    | \
                        DMA_InitStruct->PeriphOrM2MSrcIncMode   | \
                        DMA_InitStruct->MemoryOrM2MDstIncMode   | \
                        DMA_InitStruct->PeriphOrM2MSrcDataSize  | \
                        DMA_InitStruct->MemoryOrM2MDstDataSize  | \
                        DMA_InitStruct->Priority
                        );

  if(DMA_InitStruct->FIFOMode != DDL_DMA_FIFOMODE_DISABLE)
  {
    /*---------------------------- DMAx FCTRLx Configuration ------------------------
     * Configure DMAx_Streamy:  fifo mode and fifo threshold with parameters :
     * - FIFOMode:                DMA_FCTRLx_DMDEN bit
     * - FIFOThreshold:           DMA_FCTRLx_FTHSEL[1:0] bits
     */
    DDL_DMA_ConfigFifo(DMAx, Stream, DMA_InitStruct->FIFOMode, DMA_InitStruct->FIFOThreshold);   

    /*---------------------------- DMAx SCFGx Configuration --------------------------
     * Configure DMAx_Streamy:  memory burst transfer with parameters :
     * - MemBurst:                DMA_SCFGx_MBCFG[1:0] bits
     */
    DDL_DMA_SetMemoryBurstxfer(DMAx,Stream,DMA_InitStruct->MemBurst); 

    /*---------------------------- DMAx SCFGx Configuration --------------------------
     * Configure DMAx_Streamy:  peripheral burst transfer with parameters :
     * - PeriphBurst:             DMA_SCFGx_PBCFG[1:0] bits
     */
    DDL_DMA_SetPeriphBurstxfer(DMAx,Stream,DMA_InitStruct->PeriphBurst);
  }

  /*-------------------------- DMAx M0ADDRx Configuration --------------------------
   * Configure the memory or destination base address with parameter :
   * - MemoryOrM2MDstAddress:     DMA_M0ADDRx_M0ADDR[31:0] bits
   */
  DDL_DMA_SetMemoryAddress(DMAx, Stream, DMA_InitStruct->MemoryOrM2MDstAddress);

  /*-------------------------- DMAx PADDRx Configuration ---------------------------
   * Configure the peripheral or source base address with parameter :
   * - PeriphOrM2MSrcAddress:     DMA_PADDRx_PADDR[31:0] bits
   */
  DDL_DMA_SetPeriphAddress(DMAx, Stream, DMA_InitStruct->PeriphOrM2MSrcAddress);

  /*--------------------------- DMAx SxNDTR Configuration -------------------------
   * Configure the peripheral base address with parameter :
   * - NbData:                    DMA_NDATAx[15:0] bits
   */
  DDL_DMA_SetDataLength(DMAx, Stream, DMA_InitStruct->NbData);

  /*--------------------------- DMA SCFGx_CHSEL Configuration ----------------------
   * Configure the peripheral base address with parameter :
   * - PeriphRequest:             DMA_SCFGx_CHSEL[2:0] bits
   */
  DDL_DMA_SetChannelSelection(DMAx, Stream, DMA_InitStruct->Channel);

  return SUCCESS;
}

/**
  * @brief  Set each @ref DDL_DMA_InitTypeDef field to default value.
  * @param  DMA_InitStruct Pointer to a @ref DDL_DMA_InitTypeDef structure.
  * @retval None
  */
void DDL_DMA_StructInit(DDL_DMA_InitTypeDef *DMA_InitStruct)
{
  /* Set DMA_InitStruct fields to default values */
  DMA_InitStruct->PeriphOrM2MSrcAddress  = 0x00000000U;
  DMA_InitStruct->MemoryOrM2MDstAddress  = 0x00000000U;
  DMA_InitStruct->Direction              = DDL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct->Mode                   = DDL_DMA_MODE_NORMAL;
  DMA_InitStruct->PeriphOrM2MSrcIncMode  = DDL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct->MemoryOrM2MDstIncMode  = DDL_DMA_MEMORY_NOINCREMENT;
  DMA_InitStruct->PeriphOrM2MSrcDataSize = DDL_DMA_PDATAALIGN_BYTE;
  DMA_InitStruct->MemoryOrM2MDstDataSize = DDL_DMA_MDATAALIGN_BYTE;
  DMA_InitStruct->NbData                 = 0x00000000U;
  DMA_InitStruct->Channel                = DDL_DMA_CHANNEL_0;
  DMA_InitStruct->Priority               = DDL_DMA_PRIORITY_LOW;
  DMA_InitStruct->FIFOMode               = DDL_DMA_FIFOMODE_DISABLE;
  DMA_InitStruct->FIFOThreshold          = DDL_DMA_FIFOTHRESHOLD_1_4;
  DMA_InitStruct->MemBurst               = DDL_DMA_MBURST_SINGLE;
  DMA_InitStruct->PeriphBurst            = DDL_DMA_PBURST_SINGLE;
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

#endif /* DMA1 || DMA2 */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

