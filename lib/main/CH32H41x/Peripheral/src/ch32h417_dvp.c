/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_dvp.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the DVP firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_dvp.h"

/*********************************************************************
 * @fn      DVP_DeInit
 *
 * @brief   Deinitializes the DVP peripheral registers to their default
 *        reset values.
 *
 * @param   none
 *
 * @return  none
 */
void DVP_DeInit(void)
{
    RCC_HBPeriphResetCmd(RCC_HBPeriph_DVP, ENABLE);
    RCC_HBPeriphResetCmd(RCC_HBPeriph_DVP, DISABLE);
}

/*********************************************************************
 * @fn      DVP_Init
 *
 * @brief   Initializes the DVP peripheral according to the specified
 *        parameters in the DVP_InitStruct.
 *
 * @param   none
 *
 * @return  none
 */
void DVP_Init(DVP_InitTypeDef *DVP_InitStruct)
{
    DVP->CR0 = 0;
    DVP->CR1 = 0;

    DVP->CR0 = ((uint8_t)DVP_InitStruct->DVP_JPEGMode << 6) | (uint8_t)DVP_InitStruct->DVP_DataSize |
               (uint8_t)DVP_InitStruct->DVP_HCLK_P | (uint8_t)DVP_InitStruct->DVP_HSYNC_P |
               (uint8_t)DVP_InitStruct->DVP_VSYNC_P;
    DVP->CR1 = (uint8_t)DVP_InitStruct->DVP_FrameCapRate | ((uint8_t)DVP_InitStruct->DVP_Crop << 5) |
               (uint8_t)DVP_InitStruct->DVP_CaptureMode;
    DVP->ROW_NUM = (uint16_t )DVP_InitStruct->DVP_COL_NUM;
    DVP->COL_NUM = (uint16_t )DVP_InitStruct->DVP_COL_NUM;
    DVP->DMA_BUF0 = (uint32_t )DVP_InitStruct->DVP_DMA_BUF0_Addr;
    DVP->DMA_BUF1 = (uint32_t )DVP_InitStruct->DVP_DMA_BUF1_Addr;
    DVP->HOFFCNT = (uint16_t)DVP_InitStruct->DVP_Window_HOFFCNT;
    DVP->VST = (uint16_t)DVP_InitStruct->DVP_Window_VST;
    DVP->CAPCNT = (uint16_t)DVP_InitStruct->DVP_Window_CAPCNT;
    DVP->VLINE = (uint16_t)DVP_InitStruct->DVP_Window_VLine;
}

/*********************************************************************
 * @fn      DVP_StructInit
 *
 * @brief   Fills each DVP_InitStruct member with its default value.
 *
 * @param   DVP_InitStruct - pointer to an DVP_InitTypeDef structure that
 *        contains the configuration information for the specified DVP
 *        peripheral.
 *
 * @return  none
 */
void DVP_StructInit(DVP_InitTypeDef *DVP_InitStruct)
{
    DVP_InitStruct->DVP_ROW_NUM = 0;
    DVP_InitStruct->DVP_COL_NUM =0;
    DVP_InitStruct->DVP_DMA_BUF0_Addr = 0;
    DVP_InitStruct->DVP_DMA_BUF1_Addr =0;
    DVP_InitStruct->DVP_Window_HOFFCNT = 0;
    DVP_InitStruct->DVP_Window_VST = 0;
    DVP_InitStruct->DVP_Window_CAPCNT = 0;
    DVP_InitStruct->DVP_Window_VLine = 0;
    DVP_InitStruct->DVP_DataSize = DVP_DataSize_8b;
    DVP_InitStruct->DVP_HCLK_P = DVP_Hclk_P_Rising;
    DVP_InitStruct->DVP_HSYNC_P = DVP_Hsync_P_Low;
    DVP_InitStruct->DVP_VSYNC_P = DVP_Vsync_P_Low;
    DVP_InitStruct->DVP_FrameCapRate = DVP_FrameCapRate_100P;
    DVP_InitStruct->DVP_CaptureMode = DVP_CaptureMode_Snapshot;
    DVP_InitStruct->DVP_Crop = DISABLE;
    DVP_InitStruct->DVP_JPEGMode = DISABLE;
}

/*********************************************************************
 * @fn      DVP_Cmd
 *
 * @brief   Enables or disables the specified DVP peripheral.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void DVP_Cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DVP->CR0 |= RB_DVP_ENABLE;
    }
    else
    {
        DVP->CR0 &= ~RB_DVP_ENABLE;
    }
}

/*********************************************************************
 * @fn      DVP_DMACmd
 *
 * @brief   Enables or disables the specified DVP DMA request.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void DVP_DMACmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DVP->CR1 |= RB_DVP_DMA_EN;
    }
    else
    {
        DVP->CR1 &= ~RB_DVP_DMA_EN;
    }
}

/*********************************************************************
 * @fn      DVP_ReceiveCircuitResetCmd
 *
 * @brief   Enables or disables the DVP Receive Circuit Reset.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void DVP_ReceiveCircuitResetCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DVP->CR1 |= RB_DVP_RCV_CLR;
    }
    else
    {
        DVP->CR1 &= ~RB_DVP_RCV_CLR;
    }
}

/*********************************************************************
 * @fn      DVP_FIFO_ResetCmd
 *
 * @brief   Enables or disables the DVP FIFO reset.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void DVP_FIFO_ResetCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DVP->CR1 |= RB_DVP_ALL_CLR;
    }
    else
    {
        DVP->CR1 &= ~RB_DVP_ALL_CLR;
    }
}

/*********************************************************************
 * @fn      DVP_ITConfig
 *
 * @brief   Enables or disables the specified DVP interrupts.
 *
 * @param   DVP_IT - specifies the DVP interrupt sources to be enabled or disabled.
 *            DVP_IT_STR_FRM - DVP frame start interrupt Enable.
 *            DVP_IT_ROW_DONE - DVP row received done interrupt enable
 *            DVP_IT_FRM_DONE - DVP frame received done interrupt enable
 *            DVP_IT_FIFO_OV - DVP receive fifo overflow interrupt enable	
 *            DVP_IT_STP_FRM - DVP frame stop interrupt enable	
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void DVP_ITConfig(uint8_t DVP_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DVP->IER |= (uint32_t)DVP_IT;
    }
    else
    {
        DVP->IER &= (~(uint32_t)DVP_IT);
    }
}

/*********************************************************************
 * @fn      DVP_GetFlagStatus
 *
 * @brief   Checks whether the specified DVP flag is set or not.
 *
 * @param   DVP_FLAG - specifies the flag to check.
 *            DVP_FLAG_STR_FRM - flag for DVP frame start.
 *            DVP_FLAG_ROW_DONE - flag for DVP row receive done.
 *            DVP_FLAG_FRM_DONE - flag for DVP frame receive done.
 *            DVP_FLAG_FIFO_OV - flag for DVP receive fifo overflow.
 *            DVP_FLAG_STP_FRM - flag for DVP frame stop.
 *            DVP_FLAG_FIFO_RDY - flag for DVP receive fifo ready.
 *            DVP_FLAG_FIFO_FULL - flag for DVP receive fifo full.
 *            DVP_FLAG_FIFO_OV_1 - flag for DVP receive fifo overflow.
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus DVP_GetFlagStatus(uint8_t DVP_FLAG)
{
    FlagStatus bitstatus = RESET;

    if(DVP_FLAG & 0x80)
    {
        DVP_FLAG &= 0x7F; 
        if((DVP->STATUS & DVP_FLAG) != (uint8_t)RESET)
        {
            bitstatus = SET;
        }
        else
        {
            bitstatus = RESET;
        }
    }
    else
    {
        if((DVP->IFR & DVP_FLAG) != (uint8_t)RESET)
        {
            bitstatus = SET;
        }
        else
        {
            bitstatus = RESET;
        }
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      DVP_ClearFlag
 *
 * @brief   Clears the DVP's pending flags.
 *
 * @param   DVP_FLAG - specifies the flag to check.
 *            DVP_FLAG_STR_FRM - flag for DVP frame start.
 *            DVP_FLAG_ROW_DONE - flag for DVP row receive done.
 *            DVP_FLAG_FRM_DONE - flag for DVP frame receive done.
 *            DVP_FLAG_FIFO_OV - flag for DVP receive fifo overflow.
 *            DVP_FLAG_STP_FRM - flag for DVP frame stop.
 *
 * @return  none
 */
void DVP_ClearFlag(uint8_t DVP_FLAG)
{
    DVP->IFR = ~(uint8_t)DVP_FLAG;
}

/*********************************************************************
 * @fn      DVP_GetITStatus
 *
 * @brief   Checks whether the specified DVP interrupt has occurred or not.
 *
 * @param   DVP_IT - specifies the DVP interrupt source to check.
 *            DVP_IT_STR_FRM - DVP frame start interrupt.
 *            DVP_IT_ROW_DONE - DVP row received done interrupt
 *            DVP_IT_FRM_DONE - DVP frame received done interrupt
 *            DVP_IT_FIFO_OV - DVP receive fifo overflow interrupt	
 *            DVP_IT_STP_FRM - DVP frame stop interrupt enable	
 *
 * @return  FlagStatus - SET or RESET.
 */
ITStatus DVP_GetITStatus(uint8_t DVP_IT)
{
    ITStatus bitstatus = RESET;
    uint32_t enablestatus = 0;

    enablestatus = (DVP->IER & (uint8_t)DVP_IT);

    if(((DVP->IFR & DVP_IT) != (uint32_t)RESET) && enablestatus)
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
 * @fn      DVP_ClearITPendingBit
 *
 * @brief   Clears the DVP's interrupt pending bits.
 *
 * @param   DVP_IT - specifies the DVP interrupt source to check.
 *            DVP_IT_STR_FRM - DVP frame start interrupt.
 *            DVP_IT_ROW_DONE - DVP row received done interrupt
 *            DVP_IT_FRM_DONE - DVP frame received done interrupt
 *            DVP_IT_FIFO_OV - DVP receive fifo overflow interrupt	
 *            DVP_IT_STP_FRM - DVP frame stop interrupt	
 *
 * @return  none
 */
void DVP_ClearITPendingBit(uint8_t DVP_IT)
{
    DVP->IFR |= (uint8_t)DVP_IT;
}

/*********************************************************************
 * @fn      DVP_GetRxFIFO_count
 *
 * @brief   Returns the DVP receive FIFO count.
 *
 * @param   none
 *
 * @return  receive FIFO count.
 */
uint8_t DVP_GetRxFIFO_count(void)
{
    return (uint8_t)((DVP->STATUS & 0x70) >> 4);
}

/*********************************************************************
 * @fn      DVP_GetReceiveROW_count
 *
 * @brief   Returns the DVP receive ROW count.
 *
 * @param   none
 *
 * @return  receive FIFO count.
 */
uint16_t DVP_GetReceiveROW_count(void)
{
    return (uint16_t)(DVP->ROW_CNT);
}

/*********************************************************************
 * @fn      DVP_GetReceiveData
 *
 * @brief   Returns the DVP receive data.
 *
 * @param   none
 *
 * @return  DVP->DR - return receive data .
 */
uint32_t DVP_GetReceiveData(void)
{
    return (uint32_t)DVP->DR;
}