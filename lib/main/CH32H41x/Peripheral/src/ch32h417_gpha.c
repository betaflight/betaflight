/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_gpha.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the GPHA firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_gpha.h"
#include "ch32h417_rcc.h"

#define CTLR_MASK    ((uint32_t)0xFFFCE0FC)
#define PFCCTLR_MASK ((uint32_t)0x00FC00C0)
#define DEAD_MASK    ((uint32_t)0xFFFF00FE)

/*********************************************************************
 * @fn      GPHA_DeInit
 *
 * @brief   Deinitializes the GPHA peripheral registers to their default reset
 *        values.
 * 
 * @param   none
 * 
 * @return  none
 */
void GPHA_DeInit(void)
{
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPHA, ENABLE);
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_GPHA, DISABLE);
}

/*********************************************************************
 * @fn      GPHA_Init
 *
 * @brief   Initializes the GPHA peripheral according to the specified parameters
 *        in the GPHA_InitStruct.
 * 
 * @param   GPHA_InitStruct - pointer to a GPHA_InitTypeDef structure that contains
 *        the configuration information for the specified GPHA peripheral.
 * 
 * @return  none
 */
void GPHA_Init(GPHA_InitTypeDef* GPHA_InitStruct)
{
    uint32_t outgreen = 0;
    uint32_t outred = 0;
    uint32_t outalpha = 0;
    uint32_t pixline = 0;

    GPHA->CTLR &= (uint32_t)CTLR_MASK;
    GPHA->CTLR |= (GPHA_InitStruct->GPHA_Mode);

    GPHA->OPFCCR &= ~(uint32_t)GPHA_OPFCCR_CM;
    GPHA->OPFCCR |= (GPHA_InitStruct->GPHA_CMode);

    outgreen = GPHA_InitStruct->GPHA_OutputGreen << 8;
    outred = GPHA_InitStruct->GPHA_OutputRed << 16;
    outalpha = GPHA_InitStruct->GPHA_OutputAlpha << 24;

    GPHA->OCOLR = ((outgreen) | (outred) | (GPHA_InitStruct->GPHA_OutputBlue & 0xff) | (outalpha));

    GPHA->OMAR = (GPHA_InitStruct->GPHA_OutputMemoryAdd);

    GPHA->OOR &= ~(uint32_t)GPHA_OOR_LO;
    GPHA->OOR |= (GPHA_InitStruct->GPHA_OutputOffset);

    pixline = GPHA_InitStruct->GPHA_PixelPerLine << 16;
    GPHA->NLR &= ~(GPHA_NLR_NL | GPHA_NLR_PL);
    GPHA->NLR |= ((GPHA_InitStruct->GPHA_NumberOfLine) | (pixline));
}

/*********************************************************************
 * @fn      GPHA_StructInit
 *
 * @brief   Fills each GPHA_InitStruct member with its default value.
 * 
 * @param   GPHA_InitStruct - pointer to a GPHA_InitTypeDef structure which will
 *        be initialized.
 * 
 * @return  none
 */
void GPHA_StructInit(GPHA_InitTypeDef* GPHA_InitStruct)
{
    GPHA_InitStruct->GPHA_Mode = GPHA_R2M;
    GPHA_InitStruct->GPHA_CMode = GPHA_ARGB8888;
    GPHA_InitStruct->GPHA_OutputGreen = 0x00;
    GPHA_InitStruct->GPHA_OutputBlue = 0x00;
    GPHA_InitStruct->GPHA_OutputRed = 0x00;
    GPHA_InitStruct->GPHA_OutputAlpha = 0x00;
    GPHA_InitStruct->GPHA_OutputMemoryAdd = 0x00;
    GPHA_InitStruct->GPHA_OutputOffset = 0x00;
    GPHA_InitStruct->GPHA_NumberOfLine = 0x00;
    GPHA_InitStruct->GPHA_PixelPerLine = 0x00;
}

/*********************************************************************
 * @fn      GPHA_StartTransfer
 *
 * @brief   Start the GPHA transfer.
 * 
 * @param   none
 * 
 * @return  none
 */
void GPHA_StartTransfer(void)
{
    GPHA->CTLR |= (uint32_t)GPHA_CTLR_START;
}

/*********************************************************************
 * @fn      GPHA_AbortTransfer
 *
 * @brief   Abort the GPHA transfer.
 *
 * @param   none
 *
 * @return  none
 */
void GPHA_AbortTransfer(void)
{
    GPHA->CTLR |= (uint32_t)GPHA_CTLR_ABORT;
}

/*********************************************************************
 * @fn      GPHA_Suspend
 *
 * @brief   Stop or continue the GPHA transfer.
 * 
 * @param   NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void GPHA_Suspend(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        GPHA->CTLR |= (uint32_t)GPHA_CTLR_SUSP;
    }
    else
    {
        GPHA->CTLR &= ~(uint32_t)GPHA_CTLR_SUSP;
    }
}

/*********************************************************************
 * @fn      GPHA_FGConfig
 *
 * @brief   Configures the Foreground according to the specified parameters
 *        in the GPHA_FGStruct.
 * 
 * @param   GPHA_FGStruct - pointer to a GPHA_FGTypeDef structure that contains
 *        the configuration information for the specified Background.
 * 
 * @return  none
 */
void GPHA_FGConfig(GPHA_FG_InitTypeDef* GPHA_FG_InitStruct)
{
    uint32_t fg_clutcolormode = 0;
    uint32_t fg_clutsize = 0;
    uint32_t fg_alpha_mode = 0;
    uint32_t fg_alphavalue = 0;
    uint32_t fg_colorgreen = 0;
    uint32_t fg_colorred = 0;

    GPHA->FGMAR = (GPHA_FG_InitStruct->GPHA_FGMA);

    GPHA->FGOR &= ~(uint32_t)GPHA_FGOR_LO;
    GPHA->FGOR |= (GPHA_FG_InitStruct->GPHA_FGO);

    GPHA->FGPFCCR &= (uint32_t)PFCCTLR_MASK;
    fg_clutcolormode = GPHA_FG_InitStruct->GPHA_FG_CLUT_CM << 4;
    fg_clutsize = GPHA_FG_InitStruct->GPHA_FG_CLUT_SIZE << 8;
    fg_alpha_mode = GPHA_FG_InitStruct->GPHA_FGPFC_ALPHA_MODE << 16;
    fg_alphavalue = GPHA_FG_InitStruct->GPHA_FGPFC_ALPHA_VALUE << 24;
    GPHA->FGPFCCR |= (GPHA_FG_InitStruct->GPHA_FGCM | fg_clutcolormode | fg_clutsize |
                      fg_alpha_mode | fg_alphavalue);

    GPHA->FGCOLR &= ~(GPHA_FGCOLR_BLUE | GPHA_FGCOLR_GREEN | GPHA_FGCOLR_RED);
    fg_colorgreen = GPHA_FG_InitStruct->GPHA_FGC_GREEN << 8;
    fg_colorred = GPHA_FG_InitStruct->GPHA_FGC_RED << 16;
    GPHA->FGCOLR |= (GPHA_FG_InitStruct->GPHA_FGC_BLUE | fg_colorgreen | fg_colorred);

    GPHA->FGCMAR = GPHA_FG_InitStruct->GPHA_FGCMAR;
}

/*********************************************************************
 * @fn      GPHA_FG_StructInit
 *
 * @brief   Fills each GPHA_FGStruct member with its default value.
 * 
 * @param   GPHA_FGStruct - pointer to a GPHA_FGTypeDef structure which will
 *        be initialized.
 * 
 * @return  none
 */
void GPHA_FG_StructInit(GPHA_FG_InitTypeDef* GPHA_FG_InitStruct)
{
    GPHA_FG_InitStruct->GPHA_FGMA = 0x00;
    GPHA_FG_InitStruct->GPHA_FGO = 0x00;
    GPHA_FG_InitStruct->GPHA_FGCM = CM_ARGB8888;
    GPHA_FG_InitStruct->GPHA_FG_CLUT_CM = CLUT_CM_ARGB8888;
    GPHA_FG_InitStruct->GPHA_FG_CLUT_SIZE = 0x00;
    GPHA_FG_InitStruct->GPHA_FGPFC_ALPHA_MODE = NO_MODIF_ALPHA_VALUE;
    GPHA_FG_InitStruct->GPHA_FGPFC_ALPHA_VALUE = 0x00;
    GPHA_FG_InitStruct->GPHA_FGC_BLUE = 0x00;
    GPHA_FG_InitStruct->GPHA_FGC_GREEN = 0x00;
    GPHA_FG_InitStruct->GPHA_FGC_RED = 0x00;
    GPHA_FG_InitStruct->GPHA_FGCMAR = 0x00;
}

/*********************************************************************
 * @fn      GPHA_BGConfig
 *
 * @brief   Configures the Background according to the specified parameters
 *        in the GPHA_BGStruct.
 * 
 * @param   GPHA_BGStruct - pointer to a GPHA_BGTypeDef structure that contains
 *        the configuration information for the specified Background.
 * 
 * @return  none
 */
void GPHA_BGConfig(GPHA_BG_InitTypeDef* GPHA_BG_InitStruct)
{
    uint32_t bg_clutcolormode = 0;
    uint32_t bg_clutsize = 0;
    uint32_t bg_alpha_mode = 0;
    uint32_t bg_alphavalue = 0;
    uint32_t bg_colorgreen = 0;
    uint32_t bg_colorred = 0;

    GPHA->BGMAR = (GPHA_BG_InitStruct->GPHA_BGMA);

    GPHA->BGOR &= ~(uint32_t)GPHA_BGOR_LO;
    GPHA->BGOR |= (GPHA_BG_InitStruct->GPHA_BGO);

    GPHA->BGPFCCR &= (uint32_t)PFCCTLR_MASK;
    bg_clutcolormode = GPHA_BG_InitStruct->GPHA_BG_CLUT_CM << 4;
    bg_clutsize = GPHA_BG_InitStruct->GPHA_BG_CLUT_SIZE << 8;
    bg_alpha_mode = GPHA_BG_InitStruct->GPHA_BGPFC_ALPHA_MODE << 16;
    bg_alphavalue = GPHA_BG_InitStruct->GPHA_BGPFC_ALPHA_VALUE << 24;
    GPHA->BGPFCCR |= (GPHA_BG_InitStruct->GPHA_BGCM | bg_clutcolormode | bg_clutsize |
                      bg_alpha_mode | bg_alphavalue);

    GPHA->BGCOLR &= ~(GPHA_BGCOLR_BLUE | GPHA_BGCOLR_GREEN | GPHA_BGCOLR_RED);
    bg_colorgreen = GPHA_BG_InitStruct->GPHA_BGC_GREEN << 8;
    bg_colorred = GPHA_BG_InitStruct->GPHA_BGC_RED << 16;
    GPHA->BGCOLR |= (GPHA_BG_InitStruct->GPHA_BGC_BLUE | bg_colorgreen | bg_colorred);

    GPHA->BGCMAR = GPHA_BG_InitStruct->GPHA_BGCMAR;
}

/*********************************************************************
 * @fn      GPHA_BG_StructInit
 *
 * @brief   Fills each GPHA_BGStruct member with its default value.
 * 
 * @param   GPHA_BGStruct - pointer to a GPHA_BGTypeDef structure which will
 *        be initialized.
 * 
 * @return  none
 */
void GPHA_BG_StructInit(GPHA_BG_InitTypeDef* GPHA_BG_InitStruct)
{
    GPHA_BG_InitStruct->GPHA_BGMA = 0x00;
    GPHA_BG_InitStruct->GPHA_BGO = 0x00;
    GPHA_BG_InitStruct->GPHA_BGCM = CM_ARGB8888;
    GPHA_BG_InitStruct->GPHA_BG_CLUT_CM = CLUT_CM_ARGB8888;
    GPHA_BG_InitStruct->GPHA_BG_CLUT_SIZE = 0x00;
    GPHA_BG_InitStruct->GPHA_BGPFC_ALPHA_MODE = NO_MODIF_ALPHA_VALUE;
    GPHA_BG_InitStruct->GPHA_BGPFC_ALPHA_VALUE = 0x00;
    GPHA_BG_InitStruct->GPHA_BGC_BLUE = 0x00;
    GPHA_BG_InitStruct->GPHA_BGC_GREEN = 0x00;
    GPHA_BG_InitStruct->GPHA_BGC_RED = 0x00;
    GPHA_BG_InitStruct->GPHA_BGCMAR = 0x00;
}

/*********************************************************************
 * @fn      GPHA_FGStart
 *
 * @brief   Start the automatic loading of the CLUT or abort the transfer.
 * 
 * @param   NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void GPHA_FGStart(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        GPHA->FGPFCCR |= GPHA_FGPFCCR_START;
    }
    else
    {
        GPHA->FGPFCCR &= (uint32_t)~GPHA_FGPFCCR_START;
    }
}

/*********************************************************************
 * @fn      GPHA_BGStart
 *
 * @brief   Start the automatic loading of the CLUT or abort the transfer.
 * 
 * @param   NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void GPHA_BGStart(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        GPHA->BGPFCCR |= GPHA_BGPFCCR_START;
    }
    else
    {
        GPHA->BGPFCCR &= (uint32_t)~GPHA_BGPFCCR_START;
    }
}

/*********************************************************************
 * @fn      GPHA_DeadTimeConfig
 *
 * @brief   Configures the GPHA dead time.
 * 
 * @param   GPHA_DeadTime - specifies the GPHA dead time.
 *        This parameter can be one of the following values:
 * 
 * @return  none
 */
void GPHA_DeadTimeConfig(uint32_t GPHA_DeadTime, FunctionalState NewState)
{
    uint32_t DeadTime;

    if (NewState != DISABLE)
    {
        GPHA->AMTCR &= (uint32_t)DEAD_MASK;
        DeadTime = GPHA_DeadTime << 8;
        GPHA->AMTCR |= (DeadTime | GPHA_AMTCR_EN);
    }
    else
    {
        GPHA->AMTCR &= ~(uint32_t)GPHA_AMTCR_EN;
    }
}

/*********************************************************************
 * @fn      GPHA_LineWatermarkConfig
 *
 * @brief   Define the configuration of the line watermark .
 * 
 * @param   GPHA_LWatermarkConfig - Line Watermark configuration.
 * 
 * @return  none
 */
void GPHA_LineWatermarkConfig(uint32_t GPHA_LWatermarkConfig)
{
    GPHA->LWR = (uint32_t)GPHA_LWatermarkConfig;
}

/*********************************************************************
 * @fn      GPHA_ITConfig
 *
 * @brief   Enables or disables the specified GPHA's interrupts.
 * 
 * @param   GPHA_IT - specifies the GPHA interrupts sources to be enabled or disabled.
 *        This parameter can be any combination of the following values:
 *            GPHA_IT_CE -   Configuration Error Interrupt Enable.
 *            GPHA_IT_CTC -  CLUT Transfer Complete Interrupt Enable.
 *            GPHA_IT_CAE -  CLUT Access Error Interrupt Enable.
 *            GPHA_IT_TW -   Transfer Watermark Interrupt Enable.
 *            GPHA_IT_TC -   Transfer Complete interrupt enable.
 *          NewState - new state of the specified GPHA interrupts.
 * 
 * @return  none
 */
void GPHA_ITConfig(uint32_t GPHA_IT, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        GPHA->CTLR |= GPHA_IT;
    }
    else
    {
        GPHA->CTLR &= (uint32_t)~GPHA_IT;
    }
}

/*********************************************************************
 * @fn      GPHA_GetFlagStatus
 *
 * @brief   Checks whether the specified GPHA's flag is set or not.
 * 
 * @param   GPHA_FLAG - specifies the flag to check.
 *        This parameter can be one of the following values:
 *            GPHA_FLAG_CE -   Configuration Error Interrupt flag.
 *            GPHA_FLAG_CTC -  CLUT Transfer Complete Interrupt flag.
 *            GPHA_FLAG_CAE -  CLUT Access Error Interrupt flag.
 *            GPHA_FLAG_TW -   Transfer Watermark Interrupt flag.
 *            GPHA_FLAG_TC -   Transfer Complete interrupt flag.
 * 
 * @return  The new state of GPHA_FLAG (SET or RESET).
 */
FlagStatus GPHA_GetFlagStatus(uint32_t GPHA_FLAG)
{
    FlagStatus bitstatus = RESET;

    if (((GPHA->ISR) & GPHA_FLAG) != (uint32_t)RESET)
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
 * @fn      GPHA_ClearFlag
 *
 * @brief   Clears the GPHA's pending flags.
 * 
 * @param   GPHA_FLAG - specifies the flag to clear.
 *        This parameter can be any combination of the following values:
 *            GPHA_FLAG_CE -   Configuration Error Interrupt flag.
 *            GPHA_FLAG_CTC -  CLUT Transfer Complete Interrupt flag.
 *            GPHA_FLAG_CAE -  CLUT Access Error Interrupt flag.
 *            GPHA_FLAG_TW -   Transfer Watermark Interrupt flag.
 *            GPHA_FLAG_TC -   Transfer Complete interrupt flag.
 * 
 * @return  none
 */
void GPHA_ClearFlag(uint32_t GPHA_FLAG)
{
    GPHA->IFCR = (uint32_t)GPHA_FLAG;
}

/*********************************************************************
 * @fn      GPHA_GetITStatus
 *
 * @brief   Checks whether the specified GPHA's interrupt has occurred or not.
 * 
 * @param   GPHA_IT - specifies the GPHA interrupts sources to check.
 *        This parameter can be one of the following values:
 *            GPHA_IT_CE -   Configuration Error Interrupt Enable.
 *            GPHA_IT_CTC -  CLUT Transfer Complete Interrupt Enable.
 *            GPHA_IT_CAE -  CLUT Access Error Interrupt Enable.
 *            GPHA_IT_TW -   Transfer Watermark Interrupt Enable.
 *            GPHA_IT_TC -   Transfer Complete interrupt enable.
 * 
 * @return  The new state of the GPHA_IT (SET or RESET).
 */
ITStatus GPHA_GetITStatus(uint32_t GPHA_IT)
{
    ITStatus bitstatus = RESET;
    uint32_t GPHA_IT_FLAG = GPHA_IT >> 8;

    if ((GPHA->ISR & GPHA_IT_FLAG) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    if (((GPHA->CTLR & GPHA_IT) != (uint32_t)RESET) && (bitstatus != (uint32_t)RESET))
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
 * @fn      GPHA_ClearITPendingBit
 *
 * @brief   Clears the GPHA's interrupt pending bits.
 * 
 * @param   GPHA_IT - specifies the interrupt pending bit to clear.
 *        This parameter can be any combination of the following values:
 *            GPHA_IT_CE -   Configuration Error Interrupt.
 *            GPHA_IT_CTC -  CLUT Transfer Complete Interrupt.
 *            GPHA_IT_CAE -  CLUT Access Error Interrupt.
 *            GPHA_IT_TW -   Transfer Watermark Interrupt.
 *            GPHA_IT_TC -   Transfer Complete interrupt.
 * 
 * @return  none
 */
void GPHA_ClearITPendingBit(uint32_t GPHA_IT)
{
    GPHA_IT = GPHA_IT >> 8;

    GPHA->IFCR = (uint32_t)GPHA_IT;
}
