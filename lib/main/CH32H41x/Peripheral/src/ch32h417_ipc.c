/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_ipc.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the IPC firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_ipc.h"

/*********************************************************************
 * @fn      IPC_DeInit
 *
 * @brief   Deinitializes the IPC peripheral registers to their default
 *        reset values.
 *
 * @return  none
 */
void IPC_DeInit(void)
{
    IPC->CTLR = 0;
    IPC->ENA = 0;
    IPC->STS = 0;
    IPC->MSG[0] = 0;
    IPC->MSG[1] = 0;
    IPC->MSG[2] = 0;
    IPC->MSG[3] = 0;
}

/*********************************************************************
 * @fn      IPC_Init
 *
 * @brief   Initializes the IPC peripheral according to the specified
 *        parameters in the IPC_InitStruct.
 *
 * @param   IPC_InitStruct - pointer to a IPC_InitTypeDef structure
 *
 * @return  none
 */
void IPC_Init(IPC_InitTypeDef *IPC_InitStruct)
{
    uint32_t tmp = 0;

    tmp = IPC->CTLR;
    tmp &= ~((0xFF) << (IPC_InitStruct->IPC_CH * 8));

    tmp |= ((IPC_InitStruct->TxCID) << ((IPC_InitStruct->IPC_CH * 8) + 0)) | \
           ((IPC_InitStruct->RxCID) << ((IPC_InitStruct->IPC_CH * 8) + 2)) | \
           ((IPC_InitStruct->TxIER) << ((IPC_InitStruct->IPC_CH * 8) + 4)) | \
           ((IPC_InitStruct->RxIER) << ((IPC_InitStruct->IPC_CH * 8) + 5)) | \
           ((IPC_InitStruct->AutoEN) << ((IPC_InitStruct->IPC_CH * 8) + 6));

    IPC->CTLR = tmp;
}

/*********************************************************************
 * @fn      IPC_StructInit
 *
 * @brief   Fills each IPC_StructInit member with its reset value.
 *
 * @param   IPC_StructInit - pointer to a IPC_InitTypeDef structure
 *
 * @return  none
 */
void IPC_StructInit(IPC_InitTypeDef *IPC_InitStruct)
{
    IPC_InitStruct->IPC_CH = IPC_CH0;
    IPC_InitStruct->TxCID = IPC_TxCID0;
    IPC_InitStruct->RxCID = IPC_RxCID0;
    IPC_InitStruct->TxIER = DISABLE;
    IPC_InitStruct->RxIER = DISABLE;
    IPC_InitStruct->AutoEN = DISABLE;
}

/********************************************************************************
 * @fn      IPC_CH0_Lock
 *
 * @brief   Locks the IPC CH0 Controller.
 *
 * @return  none
 */
void IPC_CH0_Lock(void)
{
    IPC->CTLR |= (1 << 7);
}

/********************************************************************************
 * @fn      IPC_CH1_Lock
 *
 * @brief   Locks the IPC CH1 Controller.
 *
 * @return  none
 */
void IPC_CH1_Lock(void)
{
    IPC->CTLR |= (1 << 15);
}

/********************************************************************************
 * @fn      IPC_CH2_Lock
 *
 * @brief   Locks the IPC CH2 Controller.
 *
 * @return  none
 */
void IPC_CH2_Lock(void)
{
    IPC->CTLR |= (1 << 23);
}

/********************************************************************************
 * @fn      IPC_CH3_Lock
 *
 * @brief   Locks the IPC CH3 Controller.
 *
 * @return  none
 */
void IPC_CH3_Lock(void)
{
    IPC->CTLR |= (1 << 31);
}

/*********************************************************************
 * @fn      IPC_GetITStatus
 *
 * @brief   Checks whether the specified IPC interrupt has occurred or not.
 *
 * @param   IPC_CH - pointer to a IPC_Channel_TypeDef structure
 *          TPC_Sta_Bit - pointer to a IPC_ChannelStateBit_TypeDef structure
 *
 * @return  ITStatus - SET or RESET.
 */

ITStatus IPC_GetITStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit)
{
    ITStatus bitstatus = RESET;

    if((IPC->ISR & (1 << ((IPC_CH * 8) + TPC_Sta_Bit))) != (uint32_t)RESET)
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
 * @fn      IPC_GetITMask
 *
 * @brief   Checks whether the specified IPC interrupt request has occurred or not.
 *
 * @param   IPC_CH - pointer to a IPC_Channel_TypeDef structure
 *          TPC_Sta_Bit - pointer to a IPC_ChannelStateBit_TypeDef structure
 *
 * @return  ITStatus - SET or RESET.
 */

ITStatus IPC_GetITMask(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit)
{
    ITStatus bitstatus = RESET;

    if((IPC->ISM & (1 << ((IPC_CH * 8) + TPC_Sta_Bit))) != (uint32_t)RESET)
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
 * @fn      IPC_ITConfig
 *
 * @brief   Enables or disables the specified IPC interrupts.
 *
 * @param   IPC_CH - pointer to a IPC_Channel_TypeDef structure
 *          TPC_Sta_Bit - pointer to a IPC_ChannelStateBit_TypeDef structure
 *          NewState - new state of the specified RTC interrupts(ENABLE or DISABLE).
 *
 * @return  none
 */
void IPC_ITConfig(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        IPC->ENA |= (1 << ((IPC_CH * 8) + TPC_Sta_Bit));
    }
    else
    {
        IPC->ENA &= ~(1 << ((IPC_CH * 8) + TPC_Sta_Bit));
    }
}

/*********************************************************************
 * @fn      IPC_GetFlagStatus
 *
 * @brief   Checks whether the specified IPC flag is set or not
 *
 * @param   IPC_CH - pointer to a IPC_Channel_TypeDef structure
 *          TPC_Sta_Bit - pointer to a IPC_ChannelStateBit_TypeDef structure
 *
 * @return  The new state of PIC_FLAG (SET or RESET)
 */
FlagStatus IPC_GetFlagStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit)
{
    FlagStatus bitstatus = RESET;
    if((IPC->STS & (1 << ((IPC_CH * 8) + TPC_Sta_Bit))) != (uint32_t)RESET)
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
 * @fn      IPC_SetFlagStatus
 *
 * @brief   Set the specified IPC interrupts.
 *
 * @param   IPC_CH - pointer to a IPC_Channel_TypeDef structure
 *          TPC_Sta_Bit - pointer to a IPC_ChannelStateBit_TypeDef structure
 *
 * @return  none
 */
void IPC_SetFlagStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit)
{
    IPC->SET = (1 << ((IPC_CH * 8) + TPC_Sta_Bit));
}

/*********************************************************************
 * @fn      IPC_ClearFlagStatus
 *
 * @brief   Clear the specified IPC interrupts.
 *
 * @param   IPC_CH - pointer to a IPC_Channel_TypeDef structure
 *          TPC_Sta_Bit - pointer to a IPC_ChannelStateBit_TypeDef structure
 *
 * @return  none
 */
void IPC_ClearFlagStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit)
{
    IPC->CLR = (1 << ((IPC_CH * 8) + TPC_Sta_Bit));
}

/*********************************************************************
 * @fn      IPC_WriteMSG
 *
 * @brief   Writes user data to the specified MSG Register.
 *
 * @param   IPC_MSG - pointer to a IPC_MSG_TypeDef structure
 *          Data - data to write.
 *
 * @return  none
 */
void IPC_WriteMSG(IPC_MSG_TypeDef IPC_MSG, uint32_t Data)
{
    __IO uint32_t tmp = 0;

    tmp = (uint32_t)IPC->MSG;
    tmp += (IPC_MSG * 4);
    *(__IO uint32_t *)tmp = Data;
}

/*********************************************************************
 * @fn      IPC_ReadMSG
 *
 * @brief   Reads data from the specified MSG Register.
 *
 * @param   IPC_MSG - pointer to a IPC_MSG_TypeDef structure
 *
 * @return  MSG value.
 */
uint32_t IPC_ReadMSG(IPC_MSG_TypeDef IPC_MSG)
{
    __IO uint32_t tmp = 0;

    tmp = (uint32_t)IPC->MSG;
    tmp += (IPC_MSG * 4);

    return (*(__IO uint32_t *)tmp);
}
