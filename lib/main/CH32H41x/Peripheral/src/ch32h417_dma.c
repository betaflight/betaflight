/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_dma.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the DMA firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_dma.h"
#include "ch32h417_rcc.h"

/* DMA1 Channelx interrupt pending bit masks */
#define DMA1_Channel1_IT_Mask     ((uint32_t)(DMA_GIF1 | DMA_TCIF1 | DMA_HTIF1 | DMA_TEIF1))
#define DMA1_Channel2_IT_Mask     ((uint32_t)(DMA_GIF2 | DMA_TCIF2 | DMA_HTIF2 | DMA_TEIF2))
#define DMA1_Channel3_IT_Mask     ((uint32_t)(DMA_GIF3 | DMA_TCIF3 | DMA_HTIF3 | DMA_TEIF3))
#define DMA1_Channel4_IT_Mask     ((uint32_t)(DMA_GIF4 | DMA_TCIF4 | DMA_HTIF4 | DMA_TEIF4))
#define DMA1_Channel5_IT_Mask     ((uint32_t)(DMA_GIF5 | DMA_TCIF5 | DMA_HTIF5 | DMA_TEIF5))
#define DMA1_Channel6_IT_Mask     ((uint32_t)(DMA_GIF6 | DMA_TCIF6 | DMA_HTIF6 | DMA_TEIF6))
#define DMA1_Channel7_IT_Mask     ((uint32_t)(DMA_GIF7 | DMA_TCIF7 | DMA_HTIF7 | DMA_TEIF7))
#define DMA1_Channel8_IT_Mask     ((uint32_t)(DMA_GIF8 | DMA_TCIF8 | DMA_HTIF8 | DMA_TEIF8))

/* DMA2 Channelx interrupt pending bit masks */
#define DMA2_Channel1_IT_Mask     ((uint32_t)(DMA_GIF1 | DMA_TCIF1 | DMA_HTIF1 | DMA_TEIF1))
#define DMA2_Channel2_IT_Mask     ((uint32_t)(DMA_GIF2 | DMA_TCIF2 | DMA_HTIF2 | DMA_TEIF2))
#define DMA2_Channel3_IT_Mask     ((uint32_t)(DMA_GIF3 | DMA_TCIF3 | DMA_HTIF3 | DMA_TEIF3))
#define DMA2_Channel4_IT_Mask     ((uint32_t)(DMA_GIF4 | DMA_TCIF4 | DMA_HTIF4 | DMA_TEIF4))
#define DMA2_Channel5_IT_Mask     ((uint32_t)(DMA_GIF5 | DMA_TCIF5 | DMA_HTIF5 | DMA_TEIF5))
#define DMA2_Channel6_IT_Mask     ((uint32_t)(DMA_GIF6 | DMA_TCIF6 | DMA_HTIF6 | DMA_TEIF6))
#define DMA2_Channel7_IT_Mask     ((uint32_t)(DMA_GIF7 | DMA_TCIF7 | DMA_HTIF7 | DMA_TEIF7))
#define DMA2_Channel8_IT_Mask     ((uint32_t)(DMA_GIF8 | DMA_TCIF8 | DMA_HTIF8 | DMA_TEIF8))

/* DMA registers Masks */
#define CFGR_CLEAR_Mask           ((uint32_t)0xFFFE000F)

/*********************************************************************
 * @fn      DMA_DeInit
 *
 * @brief   Deinitializes the DMAy Channelx registers to their default
 *        reset values.
 *
 * @param   DMAy_Channelx - here y can be 1 or 2 to select the DMA and x can be
 *        1 to 8 for DMA to select the DMA Channel.
 *
 * @return  none
 */
void DMA_DeInit(DMA_Channel_TypeDef *DMAy_Channelx)
{
    DMAy_Channelx->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
    // DMAy_Channelx->CFGR = 0;
    // DMAy_Channelx->CNTR = 0;
    // DMAy_Channelx->PADDR = 0;
    // DMAy_Channelx->MADDR = 0;
    // DMAy_Channelx->M1ADDR = 0;

    // if(DMAy_Channelx == DMA1_Channel1)
    // {
    //     DMA1->INTFCR |= DMA1_Channel1_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA1_Channel2)
    // {
    //     DMA1->INTFCR |= DMA1_Channel2_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA1_Channel3)
    // {
    //     DMA1->INTFCR |= DMA1_Channel3_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA1_Channel4)
    // {
    //     DMA1->INTFCR |= DMA1_Channel4_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA1_Channel5)
    // {
    //     DMA1->INTFCR |= DMA1_Channel5_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA1_Channel6)
    // {
    //     DMA1->INTFCR |= DMA1_Channel6_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA1_Channel7)
    // {
    //     DMA1->INTFCR |= DMA1_Channel7_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA1_Channel8)
    // {
    //     DMA1->INTFCR |= DMA1_Channel8_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA2_Channel1)
    // {
    //     DMA2->INTFCR |= DMA2_Channel1_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA2_Channel2)
    // {
    //     DMA2->INTFCR |= DMA2_Channel2_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA2_Channel3)
    // {
    //     DMA2->INTFCR |= DMA2_Channel3_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA2_Channel4)
    // {
    //     DMA2->INTFCR |= DMA2_Channel4_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA2_Channel5)
    // {
    //     DMA2->INTFCR |= DMA2_Channel5_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA2_Channel6)
    // {
    //     DMA2->INTFCR |= DMA2_Channel6_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA2_Channel7)
    // {
    //     DMA2->INTFCR |= DMA2_Channel7_IT_Mask;
    // }
    // else if(DMAy_Channelx == DMA2_Channel8)
    // {
    //     DMA2->INTFCR |= DMA2_Channel8_IT_Mask;
    // }
}

/*********************************************************************
 * @fn      DMA_Init
 *
 * @brief   Initializes the DMAy Channelx according to the specified
 *        parameters in the DMA_InitStruct.
 *
 * @param   DMAy_Channelx - here y can be 1 or 2 to select the DMA and x can be
 *        1 to 8 for DMA to select the DMA Channel.
 *          DMA_InitStruct - pointer to a DMA_InitTypeDef structure that contains
 *        contains the configuration information for the specified DMA Channel.
 *
 * @return  none
 */
void DMA_Init(DMA_Channel_TypeDef *DMAy_Channelx, DMA_InitTypeDef *DMA_InitStruct)
{
    uint32_t tmpreg = 0;

    tmpreg = DMAy_Channelx->CFGR;
    tmpreg &= CFGR_CLEAR_Mask;
    tmpreg |= DMA_InitStruct->DMA_DIR | DMA_InitStruct->DMA_Mode |
              DMA_InitStruct->DMA_PeripheralInc | DMA_InitStruct->DMA_MemoryInc |
              DMA_InitStruct->DMA_PeripheralDataSize | DMA_InitStruct->DMA_MemoryDataSize |
              DMA_InitStruct->DMA_Priority | DMA_InitStruct->DMA_M2M | 
              DMA_InitStruct->DMA_BufferMode | DMA_InitStruct->DMA_DoubleBuffer_StartMemory;

    DMAy_Channelx->CFGR = tmpreg;
    DMAy_Channelx->CNTR = DMA_InitStruct->DMA_BufferSize;
    DMAy_Channelx->PADDR = DMA_InitStruct->DMA_PeripheralBaseAddr;
    DMAy_Channelx->MADDR = DMA_InitStruct->DMA_Memory0BaseAddr;
    DMAy_Channelx->M1ADDR = DMA_InitStruct->DMA_Memory1BaseAddr;
}

/*********************************************************************
 * @fn      DMA_StructInit
 *
 * @brief   Fills each DMA_InitStruct member with its default value.
 *
 * @param   DMAy_Channelx - here y can be 1 or 2 to select the DMA and x can be
 *        1 to 8 to select the DMA Channel.
 *          DMA_InitStruct - pointer to a DMA_InitTypeDef structure that contains
 *        contains the configuration information for the specified DMA Channel.
 *
 * @return  none
 */
void DMA_StructInit(DMA_InitTypeDef *DMA_InitStruct)
{
    DMA_InitStruct->DMA_PeripheralBaseAddr = 0;
    DMA_InitStruct->DMA_Memory0BaseAddr = 0;
    DMA_InitStruct->DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct->DMA_BufferSize = 0;
    DMA_InitStruct->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct->DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStruct->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct->DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct->DMA_Priority = DMA_Priority_Low;
    DMA_InitStruct->DMA_M2M = DMA_M2M_Disable;
    DMA_InitStruct->DMA_BufferMode = DMA_SingleBufferMode;
    DMA_InitStruct->DMA_Memory1BaseAddr = 0;
    DMA_InitStruct->DMA_DoubleBuffer_StartMemory = DMA_DoubleBufferMode_Memory_0;
}

/*********************************************************************
 * @fn      DMA_Cmd
 *
 * @brief   Enables or disables the specified DMAy Channelx.
 *
 * @param   DMAy_Channelx - here y can be 1 or 2 to select the DMA and x can be
 *        1 to 8 to select the DMA Channel.
 *          NewState - new state of the DMAy Channelx(ENABLE or DISABLE).
 *
 * @return  none
 */
void DMA_Cmd(DMA_Channel_TypeDef *DMAy_Channelx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DMAy_Channelx->CFGR |= DMA_CFGR1_EN;
    }
    else
    {
        DMAy_Channelx->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
    }
}

/*********************************************************************
 * @fn      DMA_ITConfig
 *
 * @brief   Enables or disables the specified DMAy Channelx interrupts.
 *
 * @param   DMAy_Channelx - here y can be 1 or 2 to select the DMA and x can be
 *        1 to 8 for DMA2 to select the DMA Channel.
 *          DMA_IT - specifies the DMA interrupts sources to be enabled
 *        or disabled.
 *            DMA_IT_TC - Transfer complete interrupt mask
 *            DMA_IT_HT - Half transfer interrupt mask
 *            DMA_IT_TE -  Transfer error interrupt mask
 *          NewState - new state of the DMAy Channelx(ENABLE or DISABLE).
 *
 * @return  none
 */
void DMA_ITConfig(DMA_Channel_TypeDef *DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DMAy_Channelx->CFGR |= DMA_IT;
    }
    else
    {
        DMAy_Channelx->CFGR &= ~DMA_IT;
    }
}

/*********************************************************************
 * @fn      DMA_SetCurrDataCounter
 *
 * @brief   Sets the number of data units in the current DMAy Channelx transfer.
 *
 * @param   DMAy_Channelx - here y can be 1 or 2 to select the DMA and x can be
 *        1 to 8 to select the DMA Channel.
 *          DataNumber - The number of data units in the current DMAy Channelx
 *        transfer.
 *
 * @return  none
 */
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef *DMAy_Channelx, uint16_t DataNumber)
{
    DMAy_Channelx->CNTR = DataNumber;
}

/*********************************************************************
 * @fn      DMA_GetCurrDataCounter
 *
 * @brief   Returns the number of remaining data units in the current
 *        DMAy Channelx transfer.
 *
 * @param   DMAy_Channelx - here y can be 1 or 2 to select the DMA and x can be
 *        1 to 8 to select the DMA Channel.
 *
 * @return  DataNumber - The number of remaining data units in the current
 *        DMAy Channelx transfer.
 */
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef *DMAy_Channelx)
{
    return ((uint16_t)(DMAy_Channelx->CNTR));
}

/*********************************************************************
 * @fn      DMA_GetFlagStatus
 *
 * @brief   Checks whether the specified DMAy Channelx flag is set or not.
 *
 * @param   DMAy_FLAG - specifies the flag to check.
 *            DMA1_FLAG_GL1 - DMA1 Channel1 global flag.
 *            DMA1_FLAG_TC1 - DMA1 Channel1 transfer complete flag.
 *            DMA1_FLAG_HT1 - DMA1 Channel1 half transfer flag.
 *            DMA1_FLAG_TE1 - DMA1 Channel1 transfer error flag.
 *            DMA1_FLAG_GL2 - DMA1 Channel2 global flag.
 *            DMA1_FLAG_TC2 - DMA1 Channel2 transfer complete flag.
 *            DMA1_FLAG_HT2 - DMA1 Channel2 half transfer flag.
 *            DMA1_FLAG_TE2 - DMA1 Channel2 transfer error flag.
 *            DMA1_FLAG_GL3 - DMA1 Channel3 global flag.
 *            DMA1_FLAG_TC3 - DMA1 Channel3 transfer complete flag.
 *            DMA1_FLAG_HT3 - DMA1 Channel3 half transfer flag.
 *            DMA1_FLAG_TE3 - DMA1 Channel3 transfer error flag.
 *            DMA1_FLAG_GL4 - DMA1 Channel4 global flag.
 *            DMA1_FLAG_TC4 - DMA1 Channel4 transfer complete flag.
 *            DMA1_FLAG_HT4 - DMA1 Channel4 half transfer flag.
 *            DMA1_FLAG_TE4 - DMA1 Channel4 transfer error flag.
 *            DMA1_FLAG_GL5 - DMA1 Channel5 global flag.
 *            DMA1_FLAG_TC5 - DMA1 Channel5 transfer complete flag.
 *            DMA1_FLAG_HT5 - DMA1 Channel5 half transfer flag.
 *            DMA1_FLAG_TE5 - DMA1 Channel5 transfer error flag.
 *            DMA1_FLAG_GL6 - DMA1 Channel6 global flag.
 *            DMA1_FLAG_TC6 - DMA1 Channel6 transfer complete flag.
 *            DMA1_FLAG_HT6 - DMA1 Channel6 half transfer flag.
 *            DMA1_FLAG_TE6 - DMA1 Channel6 transfer error flag.
 *            DMA1_FLAG_GL7 - DMA1 Channel7 global flag.
 *            DMA1_FLAG_TC7 - DMA1 Channel7 transfer complete flag.
 *            DMA1_FLAG_HT7 - DMA1 Channel7 half transfer flag.
 *            DMA1_FLAG_TE7 - DMA1 Channel7 transfer error flag.
 *            DMA1_FLAG_GL8 - DMA1 Channel8 global flag.
 *            DMA1_FLAG_TC8 - DMA1 Channel8 transfer complete flag.
 *            DMA1_FLAG_HT8 - DMA1 Channel8 half transfer flag.
 *            DMA1_FLAG_TE8 - DMA1 Channel8 transfer error flag.
 *            DMA2_FLAG_GL1 - DMA2 Channel1 global flag.
 *            DMA2_FLAG_TC1 - DMA2 Channel1 transfer complete flag.
 *            DMA2_FLAG_HT1 - DMA2 Channel1 half transfer flag.
 *            DMA2_FLAG_TE1 - DMA2 Channel1 transfer error flag.
 *            DMA2_FLAG_GL2 - DMA2 Channel2 global flag.
 *            DMA2_FLAG_TC2 - DMA2 Channel2 transfer complete flag.
 *            DMA2_FLAG_HT2 - DMA2 Channel2 half transfer flag.
 *            DMA2_FLAG_TE2 - DMA2 Channel2 transfer error flag.
 *            DMA2_FLAG_GL3 - DMA2 Channel3 global flag.
 *            DMA2_FLAG_TC3 - DMA2 Channel3 transfer complete flag.
 *            DMA2_FLAG_HT3 - DMA2 Channel3 half transfer flag.
 *            DMA2_FLAG_TE3 - DMA2 Channel3 transfer error flag.
 *            DMA2_FLAG_GL4 - DMA2 Channel4 global flag.
 *            DMA2_FLAG_TC4 - DMA2 Channel4 transfer complete flag.
 *            DMA2_FLAG_HT4 - DMA2 Channel4 half transfer flag.
 *            DMA2_FLAG_TE4 - DMA2 Channel4 transfer error flag.
 *            DMA2_FLAG_GL5 - DMA2 Channel5 global flag.
 *            DMA2_FLAG_TC5 - DMA2 Channel5 transfer complete flag.
 *            DMA2_FLAG_HT5 - DMA2 Channel5 half transfer flag.
 *            DMA2_FLAG_TE5 - DMA2 Channel5 transfer error flag.
 *            DMA2_FLAG_GL6 - DMA2 Channel6 global flag.
 *            DMA2_FLAG_TC6 - DMA2 Channel6 transfer complete flag.
 *            DMA2_FLAG_HT6 - DMA2 Channel6 half transfer flag.
 *            DMA2_FLAG_TE6 - DMA2 Channel6 transfer error flag.
 *            DMA2_FLAG_GL7 - DMA2 Channel7 global flag.
 *            DMA2_FLAG_TC7 - DMA2 Channel7 transfer complete flag.
 *            DMA2_FLAG_HT7 - DMA2 Channel7 half transfer flag.
 *            DMA2_FLAG_TE7 - DMA2 Channel7 transfer error flag.
 *            DMA2_FLAG_GL8 - DMA2 Channel8 global flag.
 *            DMA2_FLAG_TC8 - DMA2 Channel8 transfer complete flag.
 *            DMA2_FLAG_HT8 - DMA2 Channel8 half transfer flag.
 *            DMA2_FLAG_TE8 - DMA2 Channel8 transfer error flag.
 *
 * @return  The new state of DMAy_FLAG (SET or RESET).
 */
FlagStatus DMA_GetFlagStatus(DMA_TypeDef* DMAx, uint32_t DMAy_FLAG)
{
    FlagStatus bitstatus = RESET;
    uint32_t   tmpreg = 0;
    tmpreg = DMAx->INTFR;

    if((tmpreg & DMAy_FLAG) == DMAy_FLAG)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      DMA_ClearFlag
 *
 * @brief   Clears the DMAy Channelx's pending flags.
 *
 * @param   DMAy_FLAG - specifies the flag to check.
 *            DMA1_FLAG_GL1 - DMA1 Channel1 global flag.
 *            DMA1_FLAG_TC1 - DMA1 Channel1 transfer complete flag.
 *            DMA1_FLAG_HT1 - DMA1 Channel1 half transfer flag.
 *            DMA1_FLAG_TE1 - DMA1 Channel1 transfer error flag.
 *            DMA1_FLAG_GL2 - DMA1 Channel2 global flag.
 *            DMA1_FLAG_TC2 - DMA1 Channel2 transfer complete flag.
 *            DMA1_FLAG_HT2 - DMA1 Channel2 half transfer flag.
 *            DMA1_FLAG_TE2 - DMA1 Channel2 transfer error flag.
 *            DMA1_FLAG_GL3 - DMA1 Channel3 global flag.
 *            DMA1_FLAG_TC3 - DMA1 Channel3 transfer complete flag.
 *            DMA1_FLAG_HT3 - DMA1 Channel3 half transfer flag.
 *            DMA1_FLAG_TE3 - DMA1 Channel3 transfer error flag.
 *            DMA1_FLAG_GL4 - DMA1 Channel4 global flag.
 *            DMA1_FLAG_TC4 - DMA1 Channel4 transfer complete flag.
 *            DMA1_FLAG_HT4 - DMA1 Channel4 half transfer flag.
 *            DMA1_FLAG_TE4 - DMA1 Channel4 transfer error flag.
 *            DMA1_FLAG_GL5 - DMA1 Channel5 global flag.
 *            DMA1_FLAG_TC5 - DMA1 Channel5 transfer complete flag.
 *            DMA1_FLAG_HT5 - DMA1 Channel5 half transfer flag.
 *            DMA1_FLAG_TE5 - DMA1 Channel5 transfer error flag.
 *            DMA1_FLAG_GL6 - DMA1 Channel6 global flag.
 *            DMA1_FLAG_TC6 - DMA1 Channel6 transfer complete flag.
 *            DMA1_FLAG_HT6 - DMA1 Channel6 half transfer flag.
 *            DMA1_FLAG_TE6 - DMA1 Channel6 transfer error flag.
 *            DMA1_FLAG_GL7 - DMA1 Channel7 global flag.
 *            DMA1_FLAG_TC7 - DMA1 Channel7 transfer complete flag.
 *            DMA1_FLAG_HT7 - DMA1 Channel7 half transfer flag.
 *            DMA1_FLAG_TE7 - DMA1 Channel7 transfer error flag.
 *            DMA1_FLAG_GL8 - DMA1 Channel8 global flag.
 *            DMA1_FLAG_TC8 - DMA1 Channel8 transfer complete flag.
 *            DMA1_FLAG_HT8 - DMA1 Channel8 half transfer flag.
 *            DMA1_FLAG_TE8 - DMA1 Channel8 transfer error flag.
 *            DMA2_FLAG_GL1 - DMA2 Channel1 global flag.
 *            DMA2_FLAG_TC1 - DMA2 Channel1 transfer complete flag.
 *            DMA2_FLAG_HT1 - DMA2 Channel1 half transfer flag.
 *            DMA2_FLAG_TE1 - DMA2 Channel1 transfer error flag.
 *            DMA2_FLAG_GL2 - DMA2 Channel2 global flag.
 *            DMA2_FLAG_TC2 - DMA2 Channel2 transfer complete flag.
 *            DMA2_FLAG_HT2 - DMA2 Channel2 half transfer flag.
 *            DMA2_FLAG_TE2 - DMA2 Channel2 transfer error flag.
 *            DMA2_FLAG_GL3 - DMA2 Channel3 global flag.
 *            DMA2_FLAG_TC3 - DMA2 Channel3 transfer complete flag.
 *            DMA2_FLAG_HT3 - DMA2 Channel3 half transfer flag.
 *            DMA2_FLAG_TE3 - DMA2 Channel3 transfer error flag.
 *            DMA2_FLAG_GL4 - DMA2 Channel4 global flag.
 *            DMA2_FLAG_TC4 - DMA2 Channel4 transfer complete flag.
 *            DMA2_FLAG_HT4 - DMA2 Channel4 half transfer flag.
 *            DMA2_FLAG_TE4 - DMA2 Channel4 transfer error flag.
 *            DMA2_FLAG_GL5 - DMA2 Channel5 global flag.
 *            DMA2_FLAG_TC5 - DMA2 Channel5 transfer complete flag.
 *            DMA2_FLAG_HT5 - DMA2 Channel5 half transfer flag.
 *            DMA2_FLAG_TE5 - DMA2 Channel5 transfer error flag.
 *            DMA2_FLAG_GL6 - DMA2 Channel6 global flag.
 *            DMA2_FLAG_TC6 - DMA2 Channel6 transfer complete flag.
 *            DMA2_FLAG_HT6 - DMA2 Channel6 half transfer flag.
 *            DMA2_FLAG_TE6 - DMA2 Channel6 transfer error flag.
 *            DMA2_FLAG_GL7 - DMA2 Channel7 global flag.
 *            DMA2_FLAG_TC7 - DMA2 Channel7 transfer complete flag.
 *            DMA2_FLAG_HT7 - DMA2 Channel7 half transfer flag.
 *            DMA2_FLAG_TE7 - DMA2 Channel7 transfer error flag.
 *            DMA2_FLAG_GL8 - DMA2 Channel8 global flag.
 *            DMA2_FLAG_TC8 - DMA2 Channel8 transfer complete flag.
 *            DMA2_FLAG_HT8 - DMA2 Channel8 half transfer flag.
 *            DMA2_FLAG_TE8 - DMA2 Channel8 transfer error flag.
 *
 * @return  none
 */
void DMA_ClearFlag(DMA_TypeDef* DMAx, uint32_t DMAy_FLAG)
{
    DMAx->INTFCR = DMAy_FLAG;
}

/*********************************************************************
 * @fn      DMA_GetITStatus
 *
 * @brief   Checks whether the specified DMAy Channelx interrupt has
 *        occurred or not.
 *
 * @param   DMAy_IT - specifies the DMAy interrupt source to check.
 *            DMA1_IT_GL1 - DMA1 Channel1 global flag.
 *            DMA1_IT_TC1 - DMA1 Channel1 transfer complete flag.
 *            DMA1_IT_HT1 - DMA1 Channel1 half transfer flag.
 *            DMA1_IT_TE1 - DMA1 Channel1 transfer error flag.
 *            DMA1_IT_GL2 - DMA1 Channel2 global flag.
 *            DMA1_IT_TC2 - DMA1 Channel2 transfer complete flag.
 *            DMA1_IT_HT2 - DMA1 Channel2 half transfer flag.
 *            DMA1_IT_TE2 - DMA1 Channel2 transfer error flag.
 *            DMA1_IT_GL3 - DMA1 Channel3 global flag.
 *            DMA1_IT_TC3 - DMA1 Channel3 transfer complete flag.
 *            DMA1_IT_HT3 - DMA1 Channel3 half transfer flag.
 *            DMA1_IT_TE3 - DMA1 Channel3 transfer error flag.
 *            DMA1_IT_GL4 - DMA1 Channel4 global flag.
 *            DMA1_IT_TC4 - DMA1 Channel4 transfer complete flag.
 *            DMA1_IT_HT4 - DMA1 Channel4 half transfer flag.
 *            DMA1_IT_TE4 - DMA1 Channel4 transfer error flag.
 *            DMA1_IT_GL5 - DMA1 Channel5 global flag.
 *            DMA1_IT_TC5 - DMA1 Channel5 transfer complete flag.
 *            DMA1_IT_HT5 - DMA1 Channel5 half transfer flag.
 *            DMA1_IT_TE5 - DMA1 Channel5 transfer error flag.
 *            DMA1_IT_GL6 - DMA1 Channel6 global flag.
 *            DMA1_IT_TC6 - DMA1 Channel6 transfer complete flag.
 *            DMA1_IT_HT6 - DMA1 Channel6 half transfer flag.
 *            DMA1_IT_TE6 - DMA1 Channel6 transfer error flag.
 *            DMA1_IT_GL7 - DMA1 Channel7 global flag.
 *            DMA1_IT_TC7 - DMA1 Channel7 transfer complete flag.
 *            DMA1_IT_HT7 - DMA1 Channel7 half transfer flag.
 *            DMA1_IT_TE7 - DMA1 Channel7 transfer error flag.
 *            DMA1_IT_GL8 - DMA1 Channel8 global flag.
 *            DMA1_IT_TC8 - DMA1 Channel8 transfer complete flag.
 *            DMA1_IT_HT8 - DMA1 Channel8 half transfer flag.
 *            DMA1_IT_TE8 - DMA1 Channel8 transfer error flag.
 *            DMA2_IT_GL1 - DMA2 Channel1 global flag.
 *            DMA2_IT_TC1 - DMA2 Channel1 transfer complete flag.
 *            DMA2_IT_HT1 - DMA2 Channel1 half transfer flag.
 *            DMA2_IT_TE1 - DMA2 Channel1 transfer error flag.
 *            DMA2_IT_GL2 - DMA2 Channel2 global flag.
 *            DMA2_IT_TC2 - DMA2 Channel2 transfer complete flag.
 *            DMA2_IT_HT2 - DMA2 Channel2 half transfer flag.
 *            DMA2_IT_TE2 - DMA2 Channel2 transfer error flag.
 *            DMA2_IT_GL3 - DMA2 Channel3 global flag.
 *            DMA2_IT_TC3 - DMA2 Channel3 transfer complete flag.
 *            DMA2_IT_HT3 - DMA2 Channel3 half transfer flag.
 *            DMA2_IT_TE3 - DMA2 Channel3 transfer error flag.
 *            DMA2_IT_GL4 - DMA2 Channel4 global flag.
 *            DMA2_IT_TC4 - DMA2 Channel4 transfer complete flag.
 *            DMA2_IT_HT4 - DMA2 Channel4 half transfer flag.
 *            DMA2_IT_TE4 - DMA2 Channel4 transfer error flag.
 *            DMA2_IT_GL5 - DMA2 Channel5 global flag.
 *            DMA2_IT_TC5 - DMA2 Channel5 transfer complete flag.
 *            DMA2_IT_HT5 - DMA2 Channel5 half transfer flag.
 *            DMA2_IT_TE5 - DMA2 Channel5 transfer error flag.
 *            DMA2_IT_GL6 - DMA2 Channel6 global flag.
 *            DMA2_IT_TC6 - DMA2 Channel6 transfer complete flag.
 *            DMA2_IT_HT6 - DMA2 Channel6 half transfer flag.
 *            DMA2_IT_TE6 - DMA2 Channel6 transfer error flag.
 *            DMA2_IT_GL7 - DMA2 Channel7 global flag.
 *            DMA2_IT_TC7 - DMA2 Channel7 transfer complete flag.
 *            DMA2_IT_HT7 - DMA2 Channel7 half transfer flag.
 *            DMA2_IT_TE7 - DMA2 Channel7 transfer error flag.
 *            DMA2_IT_GL8 - DMA2 Channel8 global flag.
 *            DMA2_IT_TC8 - DMA2 Channel8 transfer complete flag.
 *            DMA2_IT_HT8 - DMA2 Channel8 half transfer flag.
 *            DMA2_IT_TE8 - DMA2 Channel8 transfer error flag.
 *
 * @return  The new state of DMAy_IT (SET or RESET).
 */
ITStatus DMA_GetITStatus(DMA_TypeDef* DMAx, uint32_t DMAy_IT)
{
    ITStatus bitstatus = RESET;
    uint32_t   tmpreg = 0;
    tmpreg = DMAx->INTFR;

    if((tmpreg & DMAy_IT) == DMAy_IT)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      DMA_ClearITPendingBit
 *
 * @brief   Clears the DMAy Channelx's interrupt pending bits.
 *
 * @param   DMAy_IT - specifies the DMAy interrupt source to check.
 *            DMA1_IT_GL1 - DMA1 Channel1 global flag.
 *            DMA1_IT_TC1 - DMA1 Channel1 transfer complete flag.
 *            DMA1_IT_HT1 - DMA1 Channel1 half transfer flag.
 *            DMA1_IT_TE1 - DMA1 Channel1 transfer error flag.
 *            DMA1_IT_GL2 - DMA1 Channel2 global flag.
 *            DMA1_IT_TC2 - DMA1 Channel2 transfer complete flag.
 *            DMA1_IT_HT2 - DMA1 Channel2 half transfer flag.
 *            DMA1_IT_TE2 - DMA1 Channel2 transfer error flag.
 *            DMA1_IT_GL3 - DMA1 Channel3 global flag.
 *            DMA1_IT_TC3 - DMA1 Channel3 transfer complete flag.
 *            DMA1_IT_HT3 - DMA1 Channel3 half transfer flag.
 *            DMA1_IT_TE3 - DMA1 Channel3 transfer error flag.
 *            DMA1_IT_GL4 - DMA1 Channel4 global flag.
 *            DMA1_IT_TC4 - DMA1 Channel4 transfer complete flag.
 *            DMA1_IT_HT4 - DMA1 Channel4 half transfer flag.
 *            DMA1_IT_TE4 - DMA1 Channel4 transfer error flag.
 *            DMA1_IT_GL5 - DMA1 Channel5 global flag.
 *            DMA1_IT_TC5 - DMA1 Channel5 transfer complete flag.
 *            DMA1_IT_HT5 - DMA1 Channel5 half transfer flag.
 *            DMA1_IT_TE5 - DMA1 Channel5 transfer error flag.
 *            DMA1_IT_GL6 - DMA1 Channel6 global flag.
 *            DMA1_IT_TC6 - DMA1 Channel6 transfer complete flag.
 *            DMA1_IT_HT6 - DMA1 Channel6 half transfer flag.
 *            DMA1_IT_TE6 - DMA1 Channel6 transfer error flag.
 *            DMA1_IT_GL7 - DMA1 Channel7 global flag.
 *            DMA1_IT_TC7 - DMA1 Channel7 transfer complete flag.
 *            DMA1_IT_HT7 - DMA1 Channel7 half transfer flag.
 *            DMA1_IT_TE7 - DMA1 Channel7 transfer error flag.
 *            DMA1_IT_GL8 - DMA1 Channel8 global flag.
 *            DMA1_IT_TC8 - DMA1 Channel8 transfer complete flag.
 *            DMA1_IT_HT8 - DMA1 Channel8 half transfer flag.
 *            DMA1_IT_TE8 - DMA1 Channel8 transfer error flag.
 *            DMA2_IT_GL1 - DMA2 Channel1 global flag.
 *            DMA2_IT_TC1 - DMA2 Channel1 transfer complete flag.
 *            DMA2_IT_HT1 - DMA2 Channel1 half transfer flag.
 *            DMA2_IT_TE1 - DMA2 Channel1 transfer error flag.
 *            DMA2_IT_GL2 - DMA2 Channel2 global flag.
 *            DMA2_IT_TC2 - DMA2 Channel2 transfer complete flag.
 *            DMA2_IT_HT2 - DMA2 Channel2 half transfer flag.
 *            DMA2_IT_TE2 - DMA2 Channel2 transfer error flag.
 *            DMA2_IT_GL3 - DMA2 Channel3 global flag.
 *            DMA2_IT_TC3 - DMA2 Channel3 transfer complete flag.
 *            DMA2_IT_HT3 - DMA2 Channel3 half transfer flag.
 *            DMA2_IT_TE3 - DMA2 Channel3 transfer error flag.
 *            DMA2_IT_GL4 - DMA2 Channel4 global flag.
 *            DMA2_IT_TC4 - DMA2 Channel4 transfer complete flag.
 *            DMA2_IT_HT4 - DMA2 Channel4 half transfer flag.
 *            DMA2_IT_TE4 - DMA2 Channel4 transfer error flag.
 *            DMA2_IT_GL5 - DMA2 Channel5 global flag.
 *            DMA2_IT_TC5 - DMA2 Channel5 transfer complete flag.
 *            DMA2_IT_HT5 - DMA2 Channel5 half transfer flag.
 *            DMA2_IT_TE5 - DMA2 Channel5 transfer error flag.
 *            DMA2_IT_GL6 - DMA2 Channel6 global flag.
 *            DMA2_IT_TC6 - DMA2 Channel6 transfer complete flag.
 *            DMA2_IT_HT6 - DMA2 Channel6 half transfer flag.
 *            DMA2_IT_TE6 - DMA2 Channel6 transfer error flag.
 *            DMA2_IT_GL7 - DMA2 Channel7 global flag.
 *            DMA2_IT_TC7 - DMA2 Channel7 transfer complete flag.
 *            DMA2_IT_HT7 - DMA2 Channel7 half transfer flag.
 *            DMA2_IT_TE7 - DMA2 Channel7 transfer error flag.
 *            DMA2_IT_GL8 - DMA2 Channel8 global flag.
 *            DMA2_IT_TC8 - DMA2 Channel8 transfer complete flag.
 *            DMA2_IT_HT8 - DMA2 Channel8 half transfer flag.
 *            DMA2_IT_TE8 - DMA2 Channel8 transfer error flag.
 *
 * @return  none
 */
void DMA_ClearITPendingBit(DMA_TypeDef* DMAx, uint32_t DMAy_IT)
{
    DMAx->INTFCR = DMAy_IT;
}

/*********************************************************************
 * @fn      DMA_MuxChannelConfig
 *
 * @brief   Configures the DMA input channel.
 *
 * @param   DMA_MuxChannelx - where x from 1 to 16 to select the Mux Channel.
 *          DMA_Requestx -  DMA request input.
 *                    This parameter must be a number between 1 and 0x7B.
 *
 * @return  none.
 */
void DMA_MuxChannelConfig(uint8_t DMA_MuxChannelx, uint32_t DMA_Requestx)
{
    if(DMA_MuxChannelx <= 3)
    {
        DMAMUX->CFGR0_3 &= ~(0x7F << (DMA_MuxChannelx * 8));
        DMAMUX->CFGR0_3 |= ((DMA_Requestx-1) << (DMA_MuxChannelx * 8));
    }
    else if((DMA_MuxChannelx >= 4) && (DMA_MuxChannelx <= 7))
    {
        DMAMUX->CFGR4_7 &= ~(0x7F << ((DMA_MuxChannelx - 4) * 8));
        DMAMUX->CFGR4_7 |= ((DMA_Requestx-1) << ((DMA_MuxChannelx - 4) * 8));
    }
    else if((DMA_MuxChannelx >= 8) && (DMA_MuxChannelx <= 11))
    {
        DMAMUX->CFGR8_11 &= ~(0x7F << ((DMA_MuxChannelx - 8) * 8));        
        DMAMUX->CFGR8_11 |= ((DMA_Requestx-1) << ((DMA_MuxChannelx - 8) * 8));
    }
    else if((DMA_MuxChannelx >= 12) && (DMA_MuxChannelx <= 15))
    {
        DMAMUX->CFGR12_15 &= ~(0x7F << ((DMA_MuxChannelx - 12) * 8));
        DMAMUX->CFGR12_15 |= ((DMA_Requestx-1) << ((DMA_MuxChannelx - 12) * 8));
    }
} 
