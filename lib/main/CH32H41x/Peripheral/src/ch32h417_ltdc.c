/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_ltdc.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the LTDC firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_ltdc.h"
#include "ch32h417_rcc.h"

#define GCR_MASK ((uint32_t)0x0FFE888F)

/*********************************************************************
 * @fn      LTDC_DeInit
 *
 * @brief   Deinitializes the LTDC peripheral registers to their default reset
 *        values.
 * 
 * @param   none
 * 
 * @return  none
 */
void LTDC_DeInit(void)
{
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_LTDC, ENABLE);

    RCC_HB2PeriphResetCmd(RCC_HB2Periph_LTDC, DISABLE);
}

/*********************************************************************
 * @fn      LTDC_Init
 *
 * @brief   Initializes the LTDC peripheral according to the specified parameters
 *        in the LTDC_InitStruct.
 * 
 * @param   LTDC_InitStruct - pointer to a LTDC_InitTypeDef structure that contains
 *        the configuration information for the specified LTDC peripheral.
 * 
 * @return  none
 */
void LTDC_Init(LTDC_InitTypeDef* LTDC_InitStruct)
{
    uint32_t horizontalsync = 0;
    uint32_t accumulatedHBP = 0;
    uint32_t accumulatedactiveW = 0;
    uint32_t totalwidth = 0;
    uint32_t backgreen = 0;
    uint32_t backred = 0;

    LTDC->SSCR &= ~(LTDC_SSCR_VSH | LTDC_SSCR_HSW);
    horizontalsync = (LTDC_InitStruct->LTDC_HorizontalSync << 16);
    LTDC->SSCR |= (horizontalsync | LTDC_InitStruct->LTDC_VerticalSync);

    LTDC->BPCR &= ~(LTDC_BPCR_AVBP | LTDC_BPCR_AHBP);
    accumulatedHBP = (LTDC_InitStruct->LTDC_AccumulatedHBP << 16);
    LTDC->BPCR |= (accumulatedHBP | LTDC_InitStruct->LTDC_AccumulatedVBP);

    LTDC->AWCR &= ~(LTDC_AWCR_AAH | LTDC_AWCR_AAW);
    accumulatedactiveW = (LTDC_InitStruct->LTDC_AccumulatedActiveW << 16);
    LTDC->AWCR |= (accumulatedactiveW | LTDC_InitStruct->LTDC_AccumulatedActiveH);

    LTDC->TWCR &= ~(LTDC_TWCR_TOTALH | LTDC_TWCR_TOTALW);
    totalwidth = (LTDC_InitStruct->LTDC_TotalWidth << 16);
    LTDC->TWCR |= (totalwidth | LTDC_InitStruct->LTDC_TotalHeigh);

    LTDC->GCR &= (uint32_t)GCR_MASK;
    LTDC->GCR |= (uint32_t)(LTDC_InitStruct->LTDC_HSPolarity | LTDC_InitStruct->LTDC_VSPolarity |
                            LTDC_InitStruct->LTDC_DEPolarity | LTDC_InitStruct->LTDC_PCPolarity);

    backgreen = (LTDC_InitStruct->LTDC_BackgroundGreenValue << 8);
    backred = (LTDC_InitStruct->LTDC_BackgroundRedValue << 16);

    LTDC->BCCR &= ~(LTDC_BCCR_BCBLUE | LTDC_BCCR_BCGREEN | LTDC_BCCR_BCRED);
    LTDC->BCCR |= (backred | backgreen | LTDC_InitStruct->LTDC_BackgroundBlueValue);
}

/*********************************************************************
 * @fn      LTDC_StructInit
 *
 * @brief   Fills each LTDC_InitStruct member with its default value.
 * 
 * @param   LTDC_InitStruct - pointer to a LTDC_InitTypeDef structure which will
 *        be initialized.
 * 
 * @return  none
 */
void LTDC_StructInit(LTDC_InitTypeDef* LTDC_InitStruct)
{
    LTDC_InitStruct->LTDC_HSPolarity = LTDC_HSPolarity_AL;
    LTDC_InitStruct->LTDC_VSPolarity = LTDC_VSPolarity_AL;
    LTDC_InitStruct->LTDC_DEPolarity = LTDC_DEPolarity_AL;
    LTDC_InitStruct->LTDC_PCPolarity = LTDC_PCPolarity_IPC;
    LTDC_InitStruct->LTDC_HorizontalSync = 0x00;
    LTDC_InitStruct->LTDC_VerticalSync = 0x00;
    LTDC_InitStruct->LTDC_AccumulatedHBP = 0x00;
    LTDC_InitStruct->LTDC_AccumulatedVBP = 0x00;
    LTDC_InitStruct->LTDC_AccumulatedActiveW = 0x00;
    LTDC_InitStruct->LTDC_AccumulatedActiveH = 0x00;
    LTDC_InitStruct->LTDC_TotalWidth = 0x00;
    LTDC_InitStruct->LTDC_TotalHeigh = 0x00;
    LTDC_InitStruct->LTDC_BackgroundRedValue = 0x00;
    LTDC_InitStruct->LTDC_BackgroundGreenValue = 0x00;
    LTDC_InitStruct->LTDC_BackgroundBlueValue = 0x00;
}

/*********************************************************************
 * @fn      LTDC_Cmd
 *
 * @brief   Enables or disables the LTDC Controller.
 * 
 * @param   NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void LTDC_Cmd(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        LTDC->GCR |= (uint32_t)LTDC_GCR_LTDCEN;
    }
    else
    {
        LTDC->GCR &= ~(uint32_t)LTDC_GCR_LTDCEN;
    }
}

/*********************************************************************
 * @fn      LTDC_DitherCmd
 *
 * @brief   Enables or disables Dither.
 * 
 * @param   NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void LTDC_DitherCmd(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        LTDC->GCR |= (uint32_t)LTDC_GCR_DEN;
    }
    else
    {
        LTDC->GCR &= ~(uint32_t)LTDC_GCR_DEN;
    }
}

/*********************************************************************
 * @fn      LTDC_GetRGBWidth
 *
 * @brief   Get the dither RGB width.
 * 
 * @param   LTDC_RGB_InitStruct - pointer to a LTDC_RGBTypeDef structure that contains
 *        the Dither RGB width.
 * 
 * @return  none
 */
LTDC_RGBTypeDef LTDC_GetRGBWidth(void)
{
    LTDC_RGBTypeDef LTDC_RGB_InitStruct;

    LTDC->GCR &= (uint32_t)GCR_MASK;

    LTDC_RGB_InitStruct.LTDC_BlueWidth = (uint32_t)((LTDC->GCR >> 4) & 0x7);
    LTDC_RGB_InitStruct.LTDC_GreenWidth = (uint32_t)((LTDC->GCR >> 8) & 0x7);
    LTDC_RGB_InitStruct.LTDC_RedWidth = (uint32_t)((LTDC->GCR >> 12) & 0x7);

    return LTDC_RGB_InitStruct;
}

/*********************************************************************
 * @fn      LTDC_RGBStructInit
 *
 * @brief   Fills each LTDC_RGBStruct member with its default value.
 * 
 * @param   LTDC_RGB_InitStruct - pointer to a LTDC_RGBTypeDef structure which will
 *        be initialized.
 * 
 * @return  none
 */
void LTDC_RGBStructInit(LTDC_RGBTypeDef* LTDC_RGB_InitStruct)
{
    LTDC_RGB_InitStruct->LTDC_BlueWidth = 0x02;
    LTDC_RGB_InitStruct->LTDC_GreenWidth = 0x02;
    LTDC_RGB_InitStruct->LTDC_RedWidth = 0x02;
}

/*********************************************************************
 * @fn      LTDC_LIPConfig
 *
 * @brief   Define the position of the line interrupt .
 * 
 * @param   LTDC_LIPositionConfig - Line Interrupt Position.
 * 
 * @return  none
 */
void LTDC_LIPConfig(uint32_t LTDC_LIPositionConfig)
{
    LTDC->LIPCR = (uint32_t)LTDC_LIPositionConfig;
}

/*********************************************************************
 * @fn      LTDC_ReloadConfig
 *
 * @brief   reload layers registers with new parameters 
 * 
 * @param   LTDC_Reload - specifies the type of reload.
 *        This parameter can be one of the following values:
 *            LTDC_IMReload - Vertical blanking reload.
 *            LTDC_VBReload - Immediate reload.  
 * 
 * @return  none
 */
void LTDC_ReloadConfig(uint32_t LTDC_Reload)
{
    LTDC->SRCR = (uint32_t)LTDC_Reload;
}

/*********************************************************************
 * @fn      LTDC_LayerInit
 *
 * @brief   Initializes the LTDC Layer according to the specified parameters
 *        in the LTDC_LayerStruct.
 * 
 * @param   LTDC_layerx - Select the layer to be configured, this parameter can be 
 *        one of the following values: LTDC_Layer1, LTDC_Layer2    
 *          LTDC_LayerStruct - pointer to a LTDC_LayerTypeDef structure that contains
 *        the configuration information for the specified LTDC peripheral.
 * 
 * @return  none
 */
void LTDC_LayerInit(LTDC_Layer_TypeDef* LTDC_Layerx, LTDC_Layer_InitTypeDef* LTDC_Layer_InitStruct)
{
    uint32_t whsppos = 0;
    uint32_t wvsppos = 0;
    uint32_t dcgreen = 0;
    uint32_t dcred = 0;
    uint32_t dcalpha = 0;
    uint32_t cfbp = 0;

    whsppos = LTDC_Layer_InitStruct->LTDC_HorizontalStop << 16;
    LTDC_Layerx->WHPCR &= ~(LTDC_WHPCR_WHSTPOS | LTDC_WHPCR_WHSPPOS);
    LTDC_Layerx->WHPCR = (LTDC_Layer_InitStruct->LTDC_HorizontalStart | whsppos);

    wvsppos = LTDC_Layer_InitStruct->LTDC_VerticalStop << 16;
    LTDC_Layerx->WVPCR &= ~(LTDC_WVPCR_WVSTPOS | LTDC_WVPCR_WVSPPOS);
    LTDC_Layerx->WVPCR = (LTDC_Layer_InitStruct->LTDC_VerticalStart | wvsppos);

    LTDC_Layerx->PFCR &= ~(LTDC_PFCR_PF);
    LTDC_Layerx->PFCR = (LTDC_Layer_InitStruct->LTDC_PixelFormat);

    dcgreen = (LTDC_Layer_InitStruct->LTDC_DefaultColorGreen << 8);
    dcred = (LTDC_Layer_InitStruct->LTDC_DefaultColorRed << 16);
    dcalpha = (LTDC_Layer_InitStruct->LTDC_DefaultColorAlpha << 24);
    LTDC_Layerx->DCCR &= ~(LTDC_DCCR_DCBLUE | LTDC_DCCR_DCGREEN | LTDC_DCCR_DCRED | LTDC_DCCR_DCALPHA);
    LTDC_Layerx->DCCR = (LTDC_Layer_InitStruct->LTDC_DefaultColorBlue | dcgreen |
                         dcred | dcalpha);

    LTDC_Layerx->CACR &= ~(LTDC_CACR_CONSTA);
    LTDC_Layerx->CACR = (LTDC_Layer_InitStruct->LTDC_ConstantAlpha);

    LTDC_Layerx->BFCR &= ~(LTDC_BFCR_BF2 | LTDC_BFCR_BF1);
    LTDC_Layerx->BFCR = (LTDC_Layer_InitStruct->LTDC_BlendingFactor_1 | LTDC_Layer_InitStruct->LTDC_BlendingFactor_2);

    LTDC_Layerx->CFBAR &= ~(LTDC_CFBAR_CFBADD);
    LTDC_Layerx->CFBAR = (LTDC_Layer_InitStruct->LTDC_CFBStartAdress);

    cfbp = (LTDC_Layer_InitStruct->LTDC_CFBPitch << 16);
    LTDC_Layerx->CFBLR &= ~(LTDC_CFBLR_CFBLL | LTDC_CFBLR_CFBP);
    LTDC_Layerx->CFBLR = (LTDC_Layer_InitStruct->LTDC_CFBLineLength | cfbp);

    LTDC_Layerx->CFBLNR &= ~(LTDC_CFBLNR_CFBLNBR);
    LTDC_Layerx->CFBLNR = (LTDC_Layer_InitStruct->LTDC_CFBLineNumber);
}

/*********************************************************************
 * @fn      LTDC_LayerStructInit
 *
 * @brief   Fills each LTDC_Layer_InitStruct member with its default value.
 * 
 * @param   LTDC_Layer_InitStruct - pointer to a LTDC_LayerTypeDef structure which will
 *        be initialized.
 * 
 * @return  none
 */
void LTDC_LayerStructInit(LTDC_Layer_InitTypeDef* LTDC_Layer_InitStruct)
{
    LTDC_Layer_InitStruct->LTDC_HorizontalStart = 0x00;
    LTDC_Layer_InitStruct->LTDC_HorizontalStop = 0x00;
    LTDC_Layer_InitStruct->LTDC_VerticalStart = 0x00;
    LTDC_Layer_InitStruct->LTDC_VerticalStop = 0x00;
    LTDC_Layer_InitStruct->LTDC_PixelFormat = LTDC_Pixelformat_ARGB8888;
    LTDC_Layer_InitStruct->LTDC_ConstantAlpha = 0xFF;
    LTDC_Layer_InitStruct->LTDC_DefaultColorBlue = 0x00;
    LTDC_Layer_InitStruct->LTDC_DefaultColorGreen = 0x00;
    LTDC_Layer_InitStruct->LTDC_DefaultColorRed = 0x00;
    LTDC_Layer_InitStruct->LTDC_DefaultColorAlpha = 0x00;
    LTDC_Layer_InitStruct->LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_PAxCA;
    LTDC_Layer_InitStruct->LTDC_BlendingFactor_2 = LTDC_BlendingFactor2_PAxCA;
    LTDC_Layer_InitStruct->LTDC_CFBStartAdress = 0x00;
    LTDC_Layer_InitStruct->LTDC_CFBLineLength = 0x00;
    LTDC_Layer_InitStruct->LTDC_CFBPitch = 0x00;
    LTDC_Layer_InitStruct->LTDC_CFBLineNumber = 0x00;
}

/*********************************************************************
 * @fn      LTDC_LayerCmd
 *
 * @brief   Enables or disables the LTDC_Layer Controller.
 * 
 * @param   LTDC_layerx - Select the layer to be configured, this parameter can be 
 *        one of the following values: LTDC_Layer1, LTDC_Layer2
 *          NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void LTDC_LayerCmd(LTDC_Layer_TypeDef* LTDC_Layerx, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        LTDC_Layerx->CR |= (uint32_t)LTDC_CR_LEN;
    }
    else
    {
        LTDC_Layerx->CR &= ~(uint32_t)LTDC_CR_LEN;
    }
}

/*********************************************************************
 * @fn      LTDC_GetPosStatus
 *
 * @brief   Get the current position.
 * 
 * @param   LTDC_Pos_InitStruct - pointer to a LTDC_PosTypeDef structure that contains
 *        the current position.
 * 
 * @return  none
 */
LTDC_PosTypeDef LTDC_GetPosStatus(void)
{
    LTDC_PosTypeDef LTDC_Pos_InitStruct;

    LTDC->CPSR &= ~(LTDC_CPSR_CYPOS | LTDC_CPSR_CXPOS);

    LTDC_Pos_InitStruct.LTDC_POSX = (uint32_t)(LTDC->CPSR >> 16);
    LTDC_Pos_InitStruct.LTDC_POSY = (uint32_t)(LTDC->CPSR & 0xFFFF);

    return LTDC_Pos_InitStruct;
}

/*********************************************************************
 * @fn      LTDC_PosStructInit
 *
 * @brief   Fills each LTDC_Pos_InitStruct member with its default value.
 * 
 * @param   LTDC_Pos_InitStruct - pointer to a LTDC_PosTypeDef structure which will
 *        be initialized.
 * 
 * @return  none
 */
void LTDC_PosStructInit(LTDC_PosTypeDef* LTDC_Pos_InitStruct)
{
    LTDC_Pos_InitStruct->LTDC_POSX = 0x00;
    LTDC_Pos_InitStruct->LTDC_POSY = 0x00;
}

/*********************************************************************
 * @fn      LTDC_GetCDStatus
 *
 * @brief   Checks whether the specified LTDC's flag is set or not.
 * 
 * @param   LTDC_CD - specifies the flag to check.
 *        This parameter can be one of the following values:
 *            LTDC_CD_VDES - vertical data enable current status.
 *            LTDC_CD_HDES - horizontal data enable current status.
 *            LTDC_CD_VSYNC -  Vertical Synchronization current status.
 *            LTDC_CD_HSYNC -  Horizontal Synchronization current status.
 * 
 * @return  The new state of LTDC_CD (SET or RESET).
 */
FlagStatus LTDC_GetCDStatus(uint32_t LTDC_CD)
{
    FlagStatus bitstatus;

    if ((LTDC->CDSR & LTDC_CD) != (uint32_t)RESET)
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
 * @fn      LTDC_ColorKeyingConfig
 *
 * @brief   Set and configure the color keying.
 * 
 * @param   LTDC_colorkeying_InitStruct - pointer to a LTDC_ColorKeying_InitTypeDef 
 *        structure that contains the color keying configuration.
 *          LTDC_layerx - Select the layer to be configured, this parameter can be 
 *        one of the following values: LTDC_Layer1, LTDC_Layer2   
 * 
 * @return  none
 */
void LTDC_ColorKeyingConfig(LTDC_Layer_TypeDef* LTDC_Layerx, LTDC_ColorKeying_InitTypeDef* LTDC_colorkeying_InitStruct, FunctionalState NewState)
{
    uint32_t ckgreen = 0;
    uint32_t ckred = 0;

    if (NewState != DISABLE)
    {
        LTDC_Layerx->CR |= (uint32_t)LTDC_CR_COLKEN;

        ckgreen = (LTDC_colorkeying_InitStruct->LTDC_ColorKeyGreen << 8);
        ckred = (LTDC_colorkeying_InitStruct->LTDC_ColorKeyRed << 16);
        LTDC_Layerx->CKCR &= ~(LTDC_CKCR_CKBLUE | LTDC_CKCR_CKGREEN | LTDC_CKCR_CKRED);
        LTDC_Layerx->CKCR |= (LTDC_colorkeying_InitStruct->LTDC_ColorKeyBlue | ckgreen | ckred);
    }
    else
    {
        LTDC_Layerx->CR &= ~(uint32_t)LTDC_CR_COLKEN;
    }

    LTDC->SRCR = LTDC_IMReload;
}

/*********************************************************************
 * @fn      LTDC_ColorKeyingStructInit
 *
 * @brief   Fills each LTDC_colorkeying_InitStruct member with its default value.
 * 
 * @param   LTDC_colorkeying_InitStruct - pointer to a LTDC_ColorKeying_InitTypeDef structure which will
 *        be initialized.
 * 
 * @return  none
 */
void LTDC_ColorKeyingStructInit(LTDC_ColorKeying_InitTypeDef* LTDC_colorkeying_InitStruct)
{
    LTDC_colorkeying_InitStruct->LTDC_ColorKeyBlue = 0x00;
    LTDC_colorkeying_InitStruct->LTDC_ColorKeyGreen = 0x00;
    LTDC_colorkeying_InitStruct->LTDC_ColorKeyRed = 0x00;
}

/*********************************************************************
 * @fn      LTDC_CLUTCmd
 *
 * @brief   Enables or disables CLUT.
 * 
 * @param   NewState - ENABLE or DISABLE.
 *          LTDC_layerx - Select the layer to be configured, this parameter can be 
 *        one of the following values: LTDC_Layer1, LTDC_Layer2  
 * 
 * @return  none
 */
void LTDC_CLUTCmd(LTDC_Layer_TypeDef* LTDC_Layerx, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        LTDC_Layerx->CR |= (uint32_t)LTDC_CR_CLUTEN;
    }
    else
    {
        LTDC_Layerx->CR &= ~(uint32_t)LTDC_CR_CLUTEN;
    }

    LTDC->SRCR = LTDC_IMReload;
}

/*********************************************************************
 * @fn      LTDC_CLUTInit
 *
 * @brief   configure the CLUT.
 * 
 * @param   LTDC_CLUT_InitStruct - pointer to a LTDC_CLUT_InitTypeDef structure that contains
 *        the CLUT configuration.
 *          LTDC_layerx - Select the layer to be configured, this parameter can be 
 *        one of the following values: LTDC_Layer1, LTDC_Layer2   
 * 
 * @return  none
 */
void LTDC_CLUTInit(LTDC_Layer_TypeDef* LTDC_Layerx, LTDC_CLUT_InitTypeDef* LTDC_CLUT_InitStruct)
{
    uint32_t green = 0;
    uint32_t red = 0;
    uint32_t clutadd = 0;

    green = (LTDC_CLUT_InitStruct->LTDC_GreenValue << 8);
    red = (LTDC_CLUT_InitStruct->LTDC_RedValue << 16);
    clutadd = (LTDC_CLUT_InitStruct->LTDC_CLUTAdress << 24);
    LTDC_Layerx->CLUTWR = (clutadd | LTDC_CLUT_InitStruct->LTDC_BlueValue |
                           green | red);
}

/*********************************************************************
 * @fn      LTDC_CLUTStructInit
 *
 * @brief   Fills each LTDC_CLUT_InitStruct member with its default value.
 * 
 * @param   LTDC_CLUT_InitStruct - pointer to a LTDC_CLUT_InitTypeDef structure which will
 *        be initialized.
 * 
 * @return  none
 */
void LTDC_CLUTStructInit(LTDC_CLUT_InitTypeDef* LTDC_CLUT_InitStruct)
{
    LTDC_CLUT_InitStruct->LTDC_CLUTAdress = 0x00;
    LTDC_CLUT_InitStruct->LTDC_BlueValue = 0x00;
    LTDC_CLUT_InitStruct->LTDC_GreenValue = 0x00;
    LTDC_CLUT_InitStruct->LTDC_RedValue = 0x00;
}

/*********************************************************************
 * @fn      LTDC_LayerPosition
 *
 * @brief   reconfigure the layer position.
 * 
 * @param   OffsetX - horizontal offset from start active width .
 *          OffsetY - vertical offset from start active height.   
 *          LTDC_layerx - Select the layer to be configured, this parameter can be 
 *        one of the following values: LTDC_Layer1, LTDC_Layer2   
 * 
 * @return  Reload of the shadow registers values must be applied after layer 
 *        position reconfiguration.
 */
void LTDC_LayerPosition(LTDC_Layer_TypeDef* LTDC_Layerx, uint16_t OffsetX, uint16_t OffsetY)
{
    uint32_t tempreg, temp;
    uint32_t horizontal_start;
    uint32_t horizontal_stop;
    uint32_t vertical_start;
    uint32_t vertical_stop;

    LTDC_Layerx->WHPCR &= ~(LTDC_WHPCR_WHSTPOS | LTDC_WHPCR_WHSPPOS);
    LTDC_Layerx->WVPCR &= ~(LTDC_WVPCR_WVSTPOS | LTDC_WVPCR_WVSPPOS);

    tempreg = LTDC->BPCR;
    horizontal_start = (tempreg >> 16) + 1 + OffsetX;
    vertical_start = (tempreg & 0xFFFF) + 1 + OffsetY;

    tempreg = LTDC_Layerx->PFCR;

    if (tempreg == LTDC_Pixelformat_ARGB8888)
    {
        temp = 4;
    }
    else if (tempreg == LTDC_Pixelformat_RGB888)
    {
        temp = 3;
    }
    else if (
        (tempreg == LTDC_Pixelformat_RGB565) ||
        (tempreg == LTDC_Pixelformat_ARGB1555) ||
        (tempreg == LTDC_Pixelformat_AL88))
    {
        temp = 2;
    }
    else
    {
        temp = 1;
    }

    tempreg = LTDC_Layerx->CFBLR;
    horizontal_stop = (((tempreg & 0x1FFF) - 3) / temp) + horizontal_start - 1;

    tempreg = LTDC_Layerx->CFBLNR;
    vertical_stop = (tempreg & 0x7FF) + vertical_start - 1;

    LTDC_Layerx->WHPCR = horizontal_start | (horizontal_stop << 16);
    LTDC_Layerx->WVPCR = vertical_start | (vertical_stop << 16);
}

/*********************************************************************
 * @fn      LTDC_LayerAlpha
 *
 * @brief   reconfigure constant alpha.
 * 
 * @param   ConstantAlpha - constant alpha value.
 *          LTDC_layerx - Select the layer to be configured, this parameter can be 
 *        one of the following values: LTDC_Layer1, LTDC_Layer2    
 * 
 * @return  Reload of the shadow registers values must be applied after constant 
 *        alpha reconfiguration.         
 */
void LTDC_LayerAlpha(LTDC_Layer_TypeDef* LTDC_Layerx, uint8_t ConstantAlpha)
{
    LTDC_Layerx->CACR = ConstantAlpha;
}

/*********************************************************************
 * @fn      LTDC_LayerAddress
 *
 * @brief   reconfigure layer address.
 * 
 * @param   Address - The color frame buffer start address.
 *          LTDC_layerx - Select the layer to be configured, this parameter can be 
 *        one of the following values: LTDC_Layer1, LTDC_Layer2     
 * 
 * @return  Reload of the shadow registers values must be applied after layer 
 *        address reconfiguration.
 */
void LTDC_LayerAddress(LTDC_Layer_TypeDef* LTDC_Layerx, uint32_t Address)
{
    LTDC_Layerx->CFBAR = Address;
}

/*********************************************************************
 * @fn      LTDC_LayerSize
 *
 * @brief   reconfigure layer size.
 * 
 * @param   Width - layer window width.
 *          Height - layer window height.   
 *          LTDC_layerx - Select the layer to be configured, this parameter can be 
 *        one of the following values: LTDC_Layer1, LTDC_Layer2   
 * 
 * @return  Reload of the shadow registers values must be applied after layer 
 *        size reconfiguration.
 */
void LTDC_LayerSize(LTDC_Layer_TypeDef* LTDC_Layerx, uint32_t Width, uint32_t Height)
{
    uint8_t  temp;
    uint32_t tempreg;
    uint32_t horizontal_start;
    uint32_t horizontal_stop;
    uint32_t vertical_start;
    uint32_t vertical_stop;

    tempreg = LTDC_Layerx->PFCR;

    if (tempreg == LTDC_Pixelformat_ARGB8888)
    {
        temp = 4;
    }
    else if (tempreg == LTDC_Pixelformat_RGB888)
    {
        temp = 3;
    }
    else if ((tempreg == LTDC_Pixelformat_RGB565) ||
             (tempreg == LTDC_Pixelformat_ARGB1555) ||
             (tempreg == LTDC_Pixelformat_AL88))
    {
        temp = 2;
    }
    else
    {
        temp = 1;
    }

    tempreg = LTDC_Layerx->WHPCR;
    horizontal_start = (tempreg & 0x1FFF);
    horizontal_stop = Width + horizontal_start - 1;

    tempreg = LTDC_Layerx->WVPCR;
    vertical_start = (tempreg & 0x1FFF);
    vertical_stop = Height + vertical_start - 1;

    LTDC_Layerx->WHPCR = horizontal_start | (horizontal_stop << 16);
    LTDC_Layerx->WVPCR = vertical_start | (vertical_stop << 16);

    LTDC_Layerx->CFBLR = ((Width * temp) << 16) | ((Width * temp) + 3);

    LTDC_Layerx->CFBLNR = Height;
}

/*********************************************************************
 * @fn      LTDC_LayerPixelFormat
 *
 * @brief   reconfigure layer pixel format.
 * 
 * @param   PixelFormat - reconfigure the pixel format, this parameter can be 
 *        one of the following values:@ref LTDC_Pixelformat.   
 *          LTDC_layerx - Select the layer to be configured, this parameter can be 
 *        one of the following values: LTDC_Layer1, LTDC_Layer2   
 * 
 * @return  Reload of the shadow registers values must be applied after layer 
 *        pixel format reconfiguration.
 */
void LTDC_LayerPixelFormat(LTDC_Layer_TypeDef* LTDC_Layerx, uint32_t PixelFormat)
{
    uint8_t  temp;
    uint32_t tempreg;

    tempreg = LTDC_Layerx->PFCR;

    if (tempreg == LTDC_Pixelformat_ARGB8888)
    {
        temp = 4;
    }
    else if (tempreg == LTDC_Pixelformat_RGB888)
    {
        temp = 3;
    }
    else if ((tempreg == LTDC_Pixelformat_RGB565) ||
             (tempreg == LTDC_Pixelformat_ARGB1555) ||
             (tempreg == LTDC_Pixelformat_AL88))
    {
        temp = 2;
    }
    else
    {
        temp = 1;
    }

    tempreg = (LTDC_Layerx->CFBLR >> 16);
    tempreg = (tempreg / temp);

    if (PixelFormat == LTDC_Pixelformat_ARGB8888)
    {
        temp = 4;
    }
    else if (PixelFormat == LTDC_Pixelformat_RGB888)
    {
        temp = 3;
    }
    else if ((PixelFormat == LTDC_Pixelformat_RGB565) ||
             (PixelFormat == LTDC_Pixelformat_ARGB1555) ||
             (PixelFormat == LTDC_Pixelformat_AL88))
    {
        temp = 2;
    }
    else
    {
        temp = 1;
    }

    LTDC_Layerx->CFBLR = ((tempreg * temp) << 16) | ((tempreg * temp) + 3);

    LTDC_Layerx->PFCR = PixelFormat;
}

/*********************************************************************
 * @fn      LTDC_ITConfig
 *
 * @brief   Enables or disables the specified LTDC's interrupts.
 * 
 * @param   LTDC_IT - specifies the LTDC interrupts sources to be enabled or disabled.
 *        This parameter can be any combination of the following values:
 *            LTDC_IT_LI - Line Interrupt Enable.
 *            LTDC_IT_FU - FIFO Underrun Interrupt Enable.
 *            LTDC_IT_TERR - Transfer Error Interrupt Enable.
 *            LTDC_IT_RR - Register Reload interrupt enable.  
 *          NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void LTDC_ITConfig(uint32_t LTDC_IT, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        LTDC->IER |= LTDC_IT;
    }
    else
    {
        LTDC->IER &= (uint32_t)~LTDC_IT;
    }
}

/*********************************************************************
 * @fn      LTDC_GetFlagStatus
 *
 * @brief   Checks whether the specified LTDC's flag is set or not.
 * 
 * @param   LTDC_FLAG - specifies the flag to check.
 *        This parameter can be one of the following values:
 *            LTDC_FLAG_LI -    Line Interrupt flag.
 *            LTDC_FLAG_FU -   FIFO Underrun Interrupt flag.
 *            LTDC_FLAG_TERR - Transfer Error Interrupt flag.
 *            LTDC_FLAG_RR -   Register Reload interrupt flag.
 * 
 * @return  The new state of LTDC_FLAG (SET or RESET).
 */
FlagStatus LTDC_GetFlagStatus(uint32_t LTDC_FLAG)
{
    FlagStatus bitstatus = RESET;

    if ((LTDC->ISR & LTDC_FLAG) != (uint32_t)RESET)
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
 * @fn      LTDC_ClearFlag
 *
 * @brief   Clears the LTDC's pending flags.
 * 
 * @param   LTDC_FLAG - specifies the flag to clear.
 *        This parameter can be any combination of the following values:
 *            LTDC_FLAG_LI -    Line Interrupt flag.
 *            LTDC_FLAG_FU -   FIFO Underrun Interrupt flag.
 *            LTDC_FLAG_TERR - Transfer Error Interrupt flag.
 *            LTDC_FLAG_RR -   Register Reload interrupt flag.  
 * 
 * @return  none
 */
void LTDC_ClearFlag(uint32_t LTDC_FLAG)
{
    LTDC->ICR = (uint32_t)LTDC_FLAG;
}

/*********************************************************************
 * @fn      LTDC_GetITStatus
 *
 * @brief   Checks whether the specified LTDC's interrupt has occurred or not.
 * 
 * @param   LTDC_IT - specifies the LTDC interrupts sources to check.
 *        This parameter can be one of the following values:
 *            LTDC_IT_LI -    Line Interrupt Enable.
 *            LTDC_IT_FU -   FIFO Underrun Interrupt Enable.
 *            LTDC_IT_TERR - Transfer Error Interrupt Enable.
 *            LTDC_IT_RR -   Register Reload interrupt Enable.
 * 
 * @return  The new state of the LTDC_IT (SET or RESET).
 */
ITStatus LTDC_GetITStatus(uint32_t LTDC_IT)
{
    ITStatus bitstatus = RESET;

    if ((LTDC->ISR & LTDC_IT) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    if (((LTDC->IER & LTDC_IT) != (uint32_t)RESET) && (bitstatus != (uint32_t)RESET))
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
 * @fn      LTDC_ClearITPendingBit
 *
 * @brief   Clears the LTDC's interrupt pending bits.
 * 
 * @param   LTDC_IT - specifies the interrupt pending bit to clear.
 *        This parameter can be any combination of the following values:
 *            LTDC_IT_LIE -    Line Interrupt.
 *            LTDC_IT_FUIE -   FIFO Underrun Interrupt.
 *            LTDC_IT_TERRIE - Transfer Error Interrupt.
 *            LTDC_IT_RRIE -   Register Reload interrupt.
 * 
 * @return  none
 */
void LTDC_ClearITPendingBit(uint32_t LTDC_IT)
{
    LTDC->ICR = (uint32_t)LTDC_IT;
}