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
#include "ch32h417_swpmi.h"
#include "ch32h417_rcc.h"


/*********************************************************************
 * @fn      SWPMI_DeInit
 *
 * @brief   Deinitializes the SWPMI peripheral registers to their default
 *        reset values.
 *
 * @param   none
 *
 * @return  none
 */
void SWPMI_DeInit(void)
{
    RCC_HB1PeriphResetCmd(RCC_HB1Periph_SWPMI, ENABLE);
    RCC_HB1PeriphResetCmd(RCC_HB1Periph_SWPMI, DISABLE);
}

/*********************************************************************
 * @fn      SWPMI_Init
 *
 * @brief   Initializes the SWPMI peripheral according to the specified
 *          parameters in the SWPMI_InitStruct.
 *
 * @return  none
 */
void SWPMI_Init(SWPMI_InitTypeDef* SWPMI_InitStruct)
{
    uint32_t apbclock = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;

    SWPMI->CR = 0;
    SWPMI->CR |= (uint32_t)(SWPMI_InitStruct->RxBufferingMode | SWPMI_InitStruct->TxBufferingMode | 
           ((uint32_t)SWPMI_InitStruct->LoopBackMode << 4));

    RCC_GetClocksFreq(&RCC_ClocksStatus);
    apbclock = RCC_ClocksStatus.HCLK_Frequency;
    SWPMI->BRR = (uint32_t)((apbclock/(SWPMI_InitStruct->BitRate)*4) - 1);
}

/*********************************************************************
 * @fn      SWPMI_StructInit
 *
 * @brief   Fills each SWPMI_InitStruct member with its default value.
 *
 * @param   SWPMI_InitStruct: pointer to a SWPMI_InitTypeDef structure
 *          which will be initialized.
 *
 * @return  none
 */
void SWPMI_StructInit(SWPMI_InitTypeDef* SWPMI_InitStruct)
{
    SWPMI_InitStruct->LoopBackMode = DISABLE;
    SWPMI_InitStruct->TxBufferingMode = SWPMI_TxMode_Buffering_None;
    SWPMI_InitStruct->RxBufferingMode = SWPMI_RxMode_Buffering_None;
    SWPMI_InitStruct->BitRate = 100000;
}

/*********************************************************************
 * @fn      SWPMI_Cmd
 *
 * @brief   Enables or disables the SWPMI transceiver.
 *
 * @param   NewState - new state of the SWPMI transceiver(ENABLE or DISABLE).
 *
 * @return  none
 */
void SWPMI_Cmd(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        SWPMI->CR |= SWPMI_SWPTEN; 
    }
    else
    {
        SWPMI->CR &= ~SWPMI_SWPTEN;
    }    
}

/*********************************************************************
 * @fn      SWPMI_ActivateCmd
 *
 * @brief   Enables or disables SWPMI activate.
 *
 * @param   NewState - new state of the SWPMI activate(ENABLE or DISABLE).
 *
 * @return  none
 */
void SWPMI_ActivateCmd(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        SWPMI->CR &= ~SWPMI_DEACT;
        SWPMI->CR |= SWPMI_SWPACT;
    }
    else
    {
        SWPMI->CR &= ~SWPMI_SWPACT;
    }    
}

/*********************************************************************
 * @fn      SWPMI_LoopbackCmd
 *
 * @brief   Enables or disables the SWPMI loopback mode.
 *
 * @param   NewState - new state of the SWPMI loopback mode(ENABLE or DISABLE).
 *
 * @return  none
 */
void SWPMI_LoopbackCmd(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        SWPMI->CR |= SWPMI_LPBK; 
    }
    else
    {
        SWPMI->CR &= ~SWPMI_LPBK;
    }
}

/*********************************************************************
 * @fn      SWPMI_TxBufferModeConfig
 *
 * @brief   configure the SWPMI transmit buffering mode.
 *
 * @param   TxBuffer_mode - the SWPMI transmit buffering mode only one parameter
 *        can be selected which is shown as below:
 *             SWPMI_TxMode_Buffering_None - transmit mode no buffering
 *             SWPMI_TxMode_Buffering_Single - transmit mode single buffering
 *             SWPMI_TxMode_Buffering_Multi - transmit mode multi buffering
 *
 * @return  none
 */
void SWPMI_TxBufferModeConfig(uint32_t TxBuffer_mode)
{
    SWPMI->CR &= ~SWPMI_SWPACT;
    SWPMI->CR &= ~(SWPMI_TXMODE | SWPMI_TXDMA);
    SWPMI->CR |= TxBuffer_mode;
}

/*********************************************************************
 * @fn      SWPMI_ReceiveBufferingMode
 *
 * @brief   configure the SWPMI receive buffering mode.
 *
 * @param   RxBuffer_mode - the SWPMI receive buffering mode only one parameter 
 *        can be selected which is shown as below:
 *            SWPMI_RxMode_Buffering_None - receive mode no buffering
 *            SWPMI_RxMode_Buffering_Single - receive mode single buffering
 *            SWPMI_RxMode_Buffering_Multi - receive mode multi buffering
 *
 * @return  none
 */
void SWPMI_RxBufferModeConfig(uint32_t RxBuffer_mode)
{
    SWPMI->CR &= ~SWPMI_SWPACT;
    SWPMI->CR &= ~(SWPMI_RXMODE | SWPMI_RXDMA);
    SWPMI->CR |= RxBuffer_mode;
}

/*********************************************************************
 * @fn      SWPMI_GetReceiveFrameLength
 *
 * @brief   Retrieve number of data bytes present in payload of received frame.
 *
 * @return  SWPMI->RFL - the value of the received length (0~0x1F)
 */
uint8_t SWPMI_GetReceiveFrameLength(void)
{
  return (uint8_t)(SWPMI->RFL & SWPMI_RFL);
}

/*********************************************************************
 * @fn      SWPMI_TransmitData32
 *
 * @brief   transmit data register.
 *
 * @param   TxData - transmit data
 *
 * @return  none
 */
void SWPMI_TransmitData32(uint32_t TxData)
{
    SWPMI->TDR = TxData;
}

/*********************************************************************
 * @fn      SWPMI_ReceiveData32
 *
 * @brief   receive data register.
 *
 * @return  SWPMI->RDR - the value of the received data.
 */
uint32_t SWPMI_ReceiveData32(void)
{
  return (uint32_t)(SWPMI->RDR);
}

/*********************************************************************
 * @fn      SWPMI_BypassCmd
 *
 * @brief   Enables or disables SWP Transceiver Bypass.
 *
 * @param   NewState- new state of SWP Transceiver Bypass(ENABLE or DISABLE).
 *
 * @return  none
 */
void SWPMI_BypassCmd(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        SWPMI->OR |= SWPMI_SWP_TBYP;
    }
    else
    {
        SWPMI->CR &= ~SWPMI_SWP_TBYP;
    }
}

/*********************************************************************
 * @fn      SWPMI_GetFlagStatus
 *
 * @brief   Checks whether the specified SWPMI flag is set or not.
 *
 * @param   SWPMI_FLAG - specifies the flag to check
 *        only one parameter can be selected which is shown as below:
 *            SWPMI_FLAG_RXBF - RXBF flag
 *            SWPMI_FLAG_TXBE - TXBE flag
 *            SWPMI_FLAG_RXBER - RXBER flag
 *            SWPMI_FLAG_RXOVR - RXOVR flag
 *            SWPMI_FLAG_TXUNR - TXUNR flag
 *            SWPMI_FLAG_RXNE - RXNE flag
 *            SWPMI_FLAG_TXE - TXE flag
 *            SWPMI_FLAG_TC - TC flag
 *            SWPMI_FLAG_SR - SR flag
 *            SWPMI_FLAG_SUSP - SUSP flag
 *            SWPMI_FLAG_DEACT - DEACT flag
 *            SWPMI_FLAG_RDY - RDY flag
 *
 * @return  FlagStatus - SET or RESET
 */
FlagStatus SWPMI_GetFlagStatus(uint32_t SWPMI_FLAG)
{
    FlagStatus bitstatus = RESET;

    if((SWPMI->ISR & SWPMI_FLAG) != (uint32_t)RESET)
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
 * @fn      SWPMI_ClearFlag
 *
 * @brief   Clears the SWPMI's pending bits.
 *
 * @param   flag - specifies the flag to clear
 *        only one parameter can be selected which is shown as below:
 *            SWPMI_FLAG_RXBF - RXBF flag
 *            SWPMI_FLAG_TXBE - TXBE flag
 *            SWPMI_FLAG_RXBER - RXBER flag
 *            SWPMI_FLAG_RXOVR - RXOVR flag
 *            SWPMI_FLAG_TXUNR - TXUNR flag
 *            SWPMI_FLAG_TC - TC flag
 *            SWPMI_FLAG_SR - SR flag
 *            SWPMI_FLAG_RDY - RDY flag
 *
 * @return  none
 */
void SWPMI_ClearFlag(uint32_t SWPMI_FLAG)
{
    SWPMI->ICR |= SWPMI_FLAG;
}

/*********************************************************************
 * @fn      SWPMI_ITConfig
 *
 * @brief   Enables or disables the SWPMI interrupts.
 *
 * @param   SWPMI_IT - SWPMI interrupt
 *            SWPMI_IT_RXBF - RXBF interrupt flag
 *            SWPMI_IT_TXBE - TXBE interrupt flag
 *            SWPMI_IT_RXBER - RXBER interrupt flag
 *            SWPMI_IT_RXOVR - RXOVR interrupt flag
 *            SWPMI_IT_TXUNR - TXUNR interrupt flag
 *            SWPMI_IT_RXNE - RXNE interrupt flag
 *            SWPMI_IT_TXE - TXE interrupt flag
 *            SWPMI_IT_TC - TC interrupt flag
 *            SWPMI_IT_SR - SR interrupt flag
 *            SWPMI_IT_RDY - RDY interrupt flag
 *          NewState - the SWPMI interrupt(ENABLE or DISABLE)
 *
 * @return  none
 */
void SWPMI_ITConfig(uint32_t SWPMI_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        SWPMI->IER |= SWPMI_IT;
    } 
    else
    {
        SWPMI->IER &= ~SWPMI_IT;
    }
}

/*********************************************************************
 * @fn      SWPMI_GetITStatus
 *
 * @brief   Checks whether the specified SWPMI interrupt has occurred or not.
 *
 * @param   SWPMI_IT - specifies the flag to check
 *            SWPMI_IT_RXBF - RXBF interrupt flag
 *            SWPMI_IT_TXBE - TXBE interrupt flag
 *            SWPMI_IT_RXBER - RXBER interrupt flag
 *            SWPMI_IT_RXOVR - RXOVR interrupt flag
 *            SWPMI_IT_TXUNR - TXUNR interrupt flag
 *            SWPMI_IT_RXNE - RXNE interrupt flag
 *            SWPMI_IT_TXE - TXE interrupt flag
 *            SWPMI_IT_TC - TC interrupt flag
 *            SWPMI_IT_SR - SR interrupt flag
 *            SWPMI_IT_RDY - RDY interrupt flag
 *
 * @return  FlagStatus - SET or RESET
 */
ITStatus SWPMI_GetITStatus(uint32_t SWPMI_IT)
{
    ITStatus bitstatus = RESET;
    uint16_t enablestatus = 0, enableit = 0;

    enableit = (SWPMI->IER & SWPMI_IT);
    enablestatus = (SWPMI->ISR & SWPMI_IT);

    if(enableit && enablestatus)
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
 * @fn      SWPMI_ClearITPendingBit
 *
 * @brief   Clears the SWPMI's interrupt pending bits.
 *
 * @param   interrupt - specifies the SWPMI interrupt pending bit to clear
 *            SWPMI_IT_RXBF - RXBF interrupt flag
 *            SWPMI_IT_TXBE - TXBE interrupt flag
 *            SWPMI_IT_RXBER - RXBER interrupt flag
 *            SWPMI_IT_RXOVR - RXOVR interrupt flag
 *            SWPMI_IT_TXUNR - TXUNR interrupt flag
 *            SWPMI_IT_TC - TC interrupt flag
 *            SWPMI_IT_SR - SR interrupt flag
 *            SWPMI_IT_RDY - RDY interrupt flag
 *
 * @return  none
 */
void SWPMI_ClearITPendingBit(uint32_t SWPMI_IT)
{
    SWPMI->ICR |= SWPMI_IT;
}
