/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_hsadc.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the HSADC firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_hsadc.h"
#include "ch32h417_rcc.h"

/*********************************************************************
 * @fn      HSADC_DeInit
 *
 * @brief   Deinitializes the HSADC peripheral registers to their default
 *        reset values.
 *
 * @param   none
 *
 * @return  none
 */
void HSADC_DeInit(void)
{
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_HSADC, ENABLE);
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_HSADC, DISABLE);
}

/*********************************************************************
 * @fn      HSADC_Init
 *
 * @brief   Initializes the HSADC peripheral according to the specified
 *        parameters in the HSADC_InitStruct.
 *
 * @param   none
 *
 * @return  none
 */
void HSADC_Init(HSADC_InitTypeDef *HSADC_InitStruct)
{
    uint32_t tmpreg1 = 0;

    HSADC->CFGR &= ~HSADC_EN;
    HSADC->ADDR0 = HSADC_InitStruct->HSADC_RxAddress0;
    HSADC->ADDR1 = HSADC_InitStruct->HSADC_RxAddress1;
    HSADC->CTLR2 = (uint32_t)(((uint32_t)HSADC_InitStruct->HSADC_BurstMode_DMA_LastTransferLen ) | HSADC_InitStruct->HSADC_BurstMode_TransferLen << 16 );

    tmpreg1 = ((uint32_t)HSADC_InitStruct->HSADC_DMA_TransferLen << 16) | ((uint32_t)HSADC_InitStruct->HSADC_BurstMode << 15) | 
              ((uint32_t)HSADC_InitStruct->HSADC_DualBuffer << 14) | ((uint32_t)HSADC_InitStruct->HSADC_ClockDivision << 8) |
              ((uint32_t)HSADC_InitStruct->HSADC_DataSize) | ((uint32_t)HSADC_InitStruct->HSADC_FirstConversionCycle) |
              ((uint32_t)HSADC_InitStruct->HSADC_DMA << 1);  

    HSADC->CFGR = tmpreg1;
}

/*********************************************************************
 * @fn      HSADC_StructInit
 *
 * @brief   Fills each HSADC_InitStruct member with its default value.
 *
 * @param   HSADC_InitStruct - pointer to an HSADC_InitTypeDef structure that
 *        contains the configuration information for the specified HSADC
 *        peripheral.
 *
 * @return  none
 */
void HSADC_StructInit(HSADC_InitTypeDef *HSADC_InitStruct)
{
    HSADC_InitStruct->HSADC_BurstMode_TransferLen = 0;
    HSADC_InitStruct->HSADC_BurstMode_DMA_LastTransferLen = 0;
    HSADC_InitStruct->HSADC_BurstMode = DISABLE;
    HSADC_InitStruct->HSADC_DMA_TransferLen = 0;
    HSADC_InitStruct->HSADC_ClockDivision = 9;
    HSADC_InitStruct->HSADC_DMA = DISABLE;
    HSADC_InitStruct->HSADC_DualBuffer =DISABLE;
    HSADC_InitStruct->HSADC_FirstConversionCycle = HSADC_First_Conversion_Cycle_8;
    HSADC_InitStruct->HSADC_RxAddress0 = 0;
    HSADC_InitStruct->HSADC_RxAddress1 = 0;
    HSADC_InitStruct->HSADC_DataSize = HSADC_DataSize_16b;
}

/*********************************************************************
 * @fn      HSADC_Cmd
 *
 * @brief   Enables or disables the specified HSADC peripheral.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void HSADC_Cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        HSADC->CFGR |= HSADC_EN;
    }
    else
    {
        HSADC->CFGR &= ~HSADC_EN;
    }
}

/*********************************************************************
 * @fn      HSADC_DMACmd
 *
 * @brief   Enables or disables the specified HSADC DMA request.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void HSADC_DMACmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        HSADC->CFGR |= HSADC_DMAEN;
    }
    else
    {
        HSADC->CFGR &= ~HSADC_DMAEN;
    }
}

/*********************************************************************
 * @fn      HSADC_DualBufferCmd
 *
 * @brief   Enables or disables the specified HSADC dual buffer.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void HSADC_DualBufferCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        HSADC->CFGR |= HSADC_PPMODE;
    }
    else
    {
        HSADC->CFGR &= ~HSADC_PPMODE;
    }
}

/*********************************************************************
 * @fn      HSADC_BurstModeCmd
 *
 * @brief   Enables or disables the specified HSADC burst mode.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void HSADC_BurstModeCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        HSADC->CFGR |= HSADC_BURST_EN;
    }
    else
    {
        HSADC->CFGR &= ~HSADC_BURST_EN;
    }
}

/*********************************************************************
 * @fn      HSADC_ChannelConfig
 *
 * @brief   Configures the HSADC channel.
 *
 * @param   HSADC_Channel - the HSADC channel to configure.
 *            HSADC_Channel_0 - HSADC Channel0 selected.
 *            HSADC_Channel_1 - HSADC Channel1 selected.
 *            HSADC_Channel_2 - HSADC Channel2 selected.
 *            HSADC_Channel_3 - HSADC Channel3 selected.
 *            HSADC_Channel_4 - HSADC Channel4 selected.
 *            HSADC_Channel_5 - HSADC Channel5 selected.
 *            HSADC_Channel_6 - HSADC Channel6 selected.
 *
 * @return  none
 */
void HSADC_ChannelConfig(uint8_t HSADC_Channel)
{
    HSADC->CFGR &= ~HSADC_CHSEL;
    HSADC->CFGR |= ((uint32_t)HSADC_Channel << 2);
}

/*********************************************************************
 * @fn      HSADC_BurstEndCmd
 *
 * @brief   Enables or disables the specified HSADC burst end.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void HSADC_BurstEndCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        HSADC->CTLR1 |= HSADC_BURSTEND;
    }
    else
    {
        HSADC->CTLR1 &= ~HSADC_BURSTEND;
    }
}

/*********************************************************************
 * @fn      HSADC_SoftwareStartConvCmd
 *
 * @brief   Enables or disables the selected HSADC software start conversion.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void HSADC_SoftwareStartConvCmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        HSADC->CTLR1 |= HSADC_START;
    }
    else
    {
        HSADC->CTLR1 &= ~HSADC_START;
    }
}

/*********************************************************************
 * @fn      HSADC_ITConfig
 *
 * @brief   Enables or disables the specified HSADC interrupts.
 *
 * @param   HSADC_IT - specifies the HSADC interrupt sources to be enabled or disabled.
 *            HSADC_IT_EOC - End of conversion interrupt mask.
 *            HSADC_IT_DMAEnd - End of DMA conversion interrupt mask.
 *            HSADC_IT_BurstEnd - End of burst conversion interrupt mask.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void HSADC_ITConfig(uint16_t HSADC_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        HSADC->CTLR1 |= (uint32_t)HSADC_IT;
    }
    else
    {
        HSADC->CTLR1 &= (~(uint32_t)HSADC_IT);
    }
}

/*********************************************************************
 * @fn      HSADC_GetFlagStatus
 *
 * @brief   Checks whether the specified HSADC flag is set or not.
 *
 * @param   HSADC_FLAG - specifies the flag to check.
 *            HSADC_FLAG_EOC - End of conversion flag.
 *            HSADC_FLAG_DMAEnd - End of DMA conversion flag.
 *            HSADC_FLAG_BurstEnd - End of burst conversion flag.
 *            HSADC_FLAG_RXNE - receive data register not empty flag.
 *            HSADC_FLAG_DualBufferAddr1 - Dual buffer mode receive address is address1 flag.
 *            HSADC_FLAG_FIFO_NE - Receive FIFO not empty flag.
 *            HSADC_FLAG_FIFO_Full - Receive FIFO full flag.
 *            HSADC_FLAG_FIFO_OV - Receive FIFO over flag.
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus HSADC_GetFlagStatus(uint16_t HSADC_FLAG)
{
    FlagStatus bitstatus = RESET;

    if((HSADC->STATR & HSADC_FLAG) != (uint8_t)RESET)
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
 * @fn      HSADC_ClearFlag
 *
 * @brief   Clears the HSADC's pending flags.
 *
 * @param   HSADC_FLAG - specifies the flag to check.
 *            HSADC_FLAG_EOC - End of conversion flag.
 *            HSADC_FLAG_DMAEnd - End of DMA conversion flag.
 *            HSADC_FLAG_BurstEnd - End of burst conversion flag.
 *
 * @return  none
 */
void HSADC_ClearFlag(uint16_t HSADC_FLAG)
{
    HSADC->STATR = ~(uint32_t)HSADC_FLAG;
}

/*********************************************************************
 * @fn      HSADC_GetITStatus
 *
 * @brief   Checks whether the specified HSADC interrupt has occurred or not.
 *
 * @param   HSADC_IT - specifies the ADC interrupt source to check.
 *            HSADC_IT_EOC - End of conversion interrupt mask.
 *            HSADC_IT_DMAEnd - End of DMA conversion interrupt mask.
 *            HSADC_IT_BurstEnd - End of burst conversion interrupt mask.
 *
 * @return  FlagStatus - SET or RESET.
 */
ITStatus HSADC_GetITStatus(uint16_t HSADC_IT)
{
    ITStatus bitstatus = RESET;
    uint32_t itmask = 0, enablestatus = 0;

    itmask = (HSADC_IT >> 8);
    enablestatus = (HSADC->CTLR1 & (uint16_t)HSADC_IT);

    if(((HSADC->STATR & itmask) != (uint32_t)RESET) && enablestatus)
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
 * @fn      HSADC_ClearITPendingBit
 *
 * @brief   Clears the HSADC's interrupt pending bits.
 *
 * @param   HSADC_IT - specifies the ADC interrupt source to check.
 *            HSADC_IT_EOC - End of conversion interrupt mask.
 *            HSADC_IT_DMAEnd - End of DMA conversion interrupt mask.
 *            HSADC_IT_BurstEnd - End of burst conversion interrupt mask.
 *
 * @return  none
 */
void HSADC_ClearITPendingBit(uint16_t HSADC_IT)
{
    uint8_t itmask = 0;

    itmask = (uint8_t)(HSADC_IT >> 8);
    HSADC->STATR = (uint32_t)itmask;
}

/*********************************************************************
 * @fn      HSADC_GetRxFIFO_count
 *
 * @brief   Returns the HSADC receive FIFO count.
 *
 * @param   none
 *
 * @return  receive FIFO count.
 */
uint8_t HSADC_GetRxFIFO_count(void)
{
    return (uint8_t)(HSADC->STATR >> 11);
}

/*********************************************************************
 * @fn      HSADC_GetConversionValue
 *
 * @brief   Returns the last HSADC conversion result data for regular channel.
 *
 * @param   none
 *
 * @return  HSADC->DATAR - The Data conversion value.
 */
uint16_t HSADC_GetConversionValue(void)
{
    return (uint16_t)HSADC->DATAR;
}
