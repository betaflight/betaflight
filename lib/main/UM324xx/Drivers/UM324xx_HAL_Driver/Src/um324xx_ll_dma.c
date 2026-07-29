/**
  ******************************************************************************
  * @file    stm32f4xx_ll_dma.c
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    14-April-2017
  * @brief   DMA LL module driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "um324xx_ll_dma.h"
#include "um324xx_hal_rcm.h"
#include <string.h>

#ifdef  USE_FULL_ASSERT

#else
#define assert_param(expr) ((void)0U)
#endif


/** @addtogroup STM32F4xx_LL_Driver
  * @{
  */

#if defined (DMA1) || defined (DMA2)

/** @defgroup DMA_LL DMA
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup DMA_LL_Private_Macros
  * @{
  */

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);
/* Exported functions --------------------------------------------------------*/
/** @addtogroup DMA_LL_Exported_Functions
  * @{
  */

/** @addtogroup DMA_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the DMA registers to their default reset values.
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  *         @arg @ref LL_DMA_STREAM_ALL
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DMA registers are de-initialized
  *          - ERROR: DMA registers are not de-initialized
  */
uint32_t LL_DMA_DeInit(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ErrorStatus status = SUCCESS;

  if (Stream == LL_DMA_STREAM_ALL)
  {
    if (DMAx == DMA1)
    {
      /* Force reset of DMA clock */
      __HAL_RCM_DMA1_CLK_ENABLE();

      /* Release reset of DMA clock */
      __HAL_RCM_DMA1_RELEASE_RESET();
    }
    else if (DMAx == DMA1)
    {
      /* Force reset of DMA clock */
      __HAL_RCM_DMA2_CLK_ENABLE();

      /* Release reset of DMA clock */
      __HAL_RCM_DMA2_RELEASE_RESET();
    }
    else
    {
      status = ERROR;
    }
  }
  else
  {
    /* Disable the selected DMA Channelx */
    DMAx->CHENREG = ((1<<(uint32_t)Stream) << 8); __NOP();

    /* Reset interrupt pending bits for DMA Channelx */
    DMAx->MASKTFR = ((1<<(uint32_t)Stream) << 8);
    DMAx->MASKBLOCK = ((1<<(uint32_t)Stream) << 8); 
    DMAx->MASKSRCTRAN = ((1<<(uint32_t)Stream) << 8); 
    DMAx->MASKDSTTRAN = ((1<<(uint32_t)Stream) << 8);
    DMAx->MASKERR = ((1<<(uint32_t)Stream) << 8); 

    /* Clears interrupt pending bits for DMA Channelx */
    DMAx->CLEARTFR = (1<<(uint32_t)Stream);
    DMAx->CLEARBLOCK = (1<<(uint32_t)Stream); 
    DMAx->CLEARSRCTRAN = (1<<(uint32_t)Stream); 
    DMAx->CLEARDSTTRAN = (1<<(uint32_t)Stream); 
    DMAx->CLEARERR = (1<<(uint32_t)Stream);

    /* Reset DMA Channelx src address register */
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SAR, 0);

    /* Reset DMA Channelx dst address register */
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->DAR, 0);

    /* Reset DMA Channelx control register */
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CTLL, 0x00304801);
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CTLH, 0x00000002);

    /* Reset DMA Channelx config register */
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGL, (0x00000E00 + 0x20 * (uint32_t)Stream));
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGH, 0x00000004);
  }

  return status;
}

/**
  * @brief  Initialize the DMA registers according to the specified parameters in DMA_InitStruct.
  * @note   To convert DMAx_Streamy Instance to DMAx Instance and Streamy, use helper macros :
  *         @arg @ref __LL_DMA_GET_INSTANCE
  *         @arg @ref __LL_DMA_GET_STREAM
  * @param  DMAx DMAx Instance
  * @param  Stream This parameter can be one of the following values:
  *         @arg @ref LL_DMA_STREAM_0
  *         @arg @ref LL_DMA_STREAM_1
  *         @arg @ref LL_DMA_STREAM_2
  *         @arg @ref LL_DMA_STREAM_3
  *         @arg @ref LL_DMA_STREAM_4
  *         @arg @ref LL_DMA_STREAM_5
  *         @arg @ref LL_DMA_STREAM_6
  *         @arg @ref LL_DMA_STREAM_7
  * @param  DMA_InitStruct pointer to a @ref LL_DMA_InitTypeDef structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DMA registers are initialized
  *          - ERROR: Not applicable
  */
uint32_t LL_DMA_Init(DMA_TypeDef *DMAx, uint32_t Stream, LL_DMA_InitTypeDef *DMA_InitStruct)
{
  /* Reset DMA Channelx src address register */
  WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->SAR, DMA_InitStruct->SrcAddress);

  /* Reset DMA Channelx dst address register */
  WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->DAR, DMA_InitStruct->DstAddress);

  /* Reset DMA Channelx control register */
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CTLL, DMA_CTL0_TT_FC, DMA_InitStruct->Direction);
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CTLL, DMA_CTL0_SINC, DMA_InitStruct->SrcInc);
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CTLL, DMA_CTL0_DINC, DMA_InitStruct->DstInc);

  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CTLL, DMA_CTL0_SRC_TR_WIDTH, DMA_InitStruct->SrcDataAlignment);
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CTLL, DMA_CTL0_DST_TR_WIDTH, DMA_InitStruct->DstDataAlignment);  

  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CTLL, DMA_CTL0_SRC_MSIZE, DMA_InitStruct->SrcMSize);
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CTLL, DMA_CTL0_DST_MSIZE, DMA_InitStruct->DstMSize);

  WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CTLH, DMA_InitStruct->NbData);

  /* Reset DMA Channelx config register */
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGL, DMA_CFG0_HS_SEL_SRC, DMA_InitStruct->SrcHsSel);
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGL, DMA_CFG0_HS_SEL_DST, DMA_InitStruct->DstHsSel);

  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGL, DMA_CFG0_RELOAD_SRC, DMA_InitStruct->SrcReload);
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGL, DMA_CFG0_RELOAD_DST, DMA_InitStruct->DstReload);

  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGL, DMA_CFG0_CH_PRIOR, DMA_InitStruct->Priority);

  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGH, DMA_CFGH0_SRC_PER, DMA_InitStruct->SrcPer);
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGH, DMA_CFGH0_DEST_PER, DMA_InitStruct->DstPer);

  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGH, DMA_CFGH0_FIFO_MODE, DMA_InitStruct->FIFOMode);
  MODIFY_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGH, DMA_CFGH0_FCMODE, DMA_InitStruct->FCMode);

  ((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CFGL |= (1<<20);  //polo: SRAM2 used for DMA buffer.

  return SUCCESS;
}

/**
  * @brief  Set each @ref LL_DMA_InitTypeDef field to default value.
  * @param  DMA_InitStruct Pointer to a @ref LL_DMA_InitTypeDef structure.
  * @retval None
  */
void LL_DMA_StructInit(LL_DMA_InitTypeDef *DMA_InitStruct)
{
  /* Set DMA_InitStruct fields to default values */
  DMA_InitStruct->SrcAddress  = 0x00000000U;
  DMA_InitStruct->DstAddress  = 0x00000000U;

  DMA_InitStruct->Direction = LL_DMA_PERIPH_TO_MEMORY;
  DMA_InitStruct->SrcInc = LL_DMA_SRCINC_NOC;    
  DMA_InitStruct->DstInc = LL_DMA_DSTINC_NOC;    

  DMA_InitStruct->SrcDataAlignment = LL_DMA_SRCDATAALIGN_BYTE;  
  DMA_InitStruct->DstDataAlignment = LL_DMA_DSTDATAALIGN_BYTE;  

  DMA_InitStruct->SrcMSize = LL_DMA_BURST_SRC_NUM_1;   //src burst size
  DMA_InitStruct->DstMSize = LL_DMA_BURST_DST_NUM_1;   //dst burst size
  DMA_InitStruct->NbData   = 0x00000000U;

  DMA_InitStruct->SrcPer = LL_DMA_SRC_PER_HANDSHAKING_NULL;    
  DMA_InitStruct->DstPer = LL_DMA_DEST_PER_HANDSHAKING_NULL;    

  DMA_InitStruct->SrcReload = LL_DMA_SRC_RELOAD_DISABLE;  
  DMA_InitStruct->DstReload = LL_DMA_DST_RELOAD_DISABLE;  

  DMA_InitStruct->SrcHsSel = LL_DMA_SRC_HS_HW;        
  DMA_InitStruct->DstHsSel = LL_DMA_DST_HS_SW;       
    
  DMA_InitStruct->FIFOMode = LL_DMA_FIFOMODE_DISABLE;     //FIFO MODE SET
  DMA_InitStruct->FCMode   = LL_DMA_FCMODE_DISABLE;         //FCMODE SET

  DMA_InitStruct->Priority = LL_DMA_PRIORITY_0;
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

#endif /* DMA0 || DMA1 */




