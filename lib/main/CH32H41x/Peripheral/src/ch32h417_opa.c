/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_opa.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the OPA firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_opa.h"
#include "ch32h417_rcc.h"


/*********************************************************************
 * @fn      OPA_DeInit
 *
 * @brief   Deinitializes the OPA and CMP peripheral registers to their default
 *        reset values.
 *
 * @return  none
 */
void OPA_CMP_DeInit(void)
{
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_OPCM, ENABLE);
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_OPCM, DISABLE);
}

/*********************************************************************
 * @fn      OPA_Init
 *
 * @brief   Initializes the OPA peripheral according to the specified
 *        parameters in the OPA_InitStruct.
 *
 * @param   OPA_InitStruct - pointer to a OPA_InitTypeDef structure
 *
 * @return  none
 */
void OPA_Init(OPA_Num_TypeDef OPAx,OPA_InitTypeDef *OPA_InitStruct)
{
    uint32_t tmp0 = 0, tmp = 0;

    tmp = (uint32_t)OPA_BASE;
    tmp += (OPAx << 2);

    tmp0 = (OPA_InitStruct->Mode)<< (1)|(OPA_InitStruct->PSEL)<<(3)\
         | (OPA_InitStruct->NSEL)<< (4)\
         | (OPA_InitStruct->FB)<< (8) | (OPA_InitStruct->PGADIF)<< (9)\
         | (OPA_InitStruct->HS)<< (10);

    *(__IO uint32_t *)tmp =  tmp0 ;
}

/*********************************************************************
 * @fn      OPA_StructInit
 *
 * @brief   Fills each OPA_StructInit member with its reset value.
 *
 * @param   OPA_StructInit - pointer to a OPA_InitTypeDef structure
 *
 * @return  none
 */
void OPA_StructInit(OPA_InitTypeDef *OPA_InitStruct)
{
    OPA_InitStruct->Mode = OUT_TO_CMP;
    OPA_InitStruct->PSEL = CHP0;
    OPA_InitStruct->NSEL = CHN_OFF;
    OPA_InitStruct->PGADIF = DIF_OFF;
    OPA_InitStruct->FB = FB_OFF;
    OPA_InitStruct->HS = HS_OFF;
}

/*********************************************************************
 * @fn      OPA_Cmd
 *
 * @brief   Enables or disables the specified OPA peripheral.
 *
 * @param   OPA_NUM - Select OPAx
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_Cmd(OPA_Num_TypeDef OPAx, FunctionalState NewState)
{
    uint32_t tmp = 0;
    tmp = (uint32_t)OPA_BASE;
    tmp += (OPAx << 2);

    if(NewState != DISABLE)
    {
        *(__IO uint32_t *)tmp |= (1 << 0);
    }
    else
    {
        *(__IO uint32_t *)tmp &= ~(1 << 0);
    }
}

/*********************************************************************
 * @fn      OPA_CMP_Init
 *
 * @brief   Initializes the CMP peripheral according to the specified
 *        parameters in the CMP_InitTypeDef.
 *
 * @param   CMP_InitStruct - pointer to a CMP_InitTypeDef structure
 *
 * @return  none
 */
void OPA_CMP_Init(CMP_InitTypeDef *CMP_InitStruct)
{
    uint32_t tmp = 0;
    tmp = OPA->CMP_CTLR ;
    tmp &= ~(0x1FFF);
    tmp |= (CMP_InitStruct->PSEL) | (CMP_InitStruct->NSEL << 2) |\
           (CMP_InitStruct->Mode << 4) | (CMP_InitStruct->HYPSEL << 9) |\
           (CMP_InitStruct->VREF << 11);

    OPA->CMP_CTLR = tmp;
}

/*********************************************************************
 * @fn     OPA_CMP_StructInit
 *
 * @brief   Fills each CMP_StructInit member with its reset value.
 *
 * @param   CMP_StructInit - pointer to a CMP_StructInit structure
 *
 * @return  none
 */
void OPA_CMP_StructInit(CMP_InitTypeDef *CMP_InitStruct)
{
    CMP_InitStruct->Mode = OUT_TO_IO;
    CMP_InitStruct->NSEL = CMP_CHN0;
    CMP_InitStruct->PSEL = CMP_CHP1;
    CMP_InitStruct->VREF = CMP_VREF_OFF;
    CMP_InitStruct->HYPSEL = CMP_HYPSEL_OFF;
}

/*********************************************************************
 * @fn      OPA_CMP_Cmd
 *
 * @brief   Enables or disables the specified CMP peripheral.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_CMP_Cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        OPA->CMP_CTLR |= OPA_CMP_CTLR_EN;
    }
    else
    {
        OPA->CMP_CTLR &= ~OPA_CMP_CTLR_EN;
    }
}

/*********************************************************************
 * @fn      OPA_CMP_FILT_Cmd
 *
 * @brief   Enables or disables digital filtering the specified CMP.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_CMP_FILT_Cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        OPA->CMP_CTLR |= OPA_CMP_CTLR_FILT_EN;
    }
    else
    {
        OPA->CMP_CTLR &= ~OPA_CMP_CTLR_FILT_EN;
    }
}

/*********************************************************************
 * @fn      OPA_CMP_FILTConfig
 *
 * @brief   Configures the digital filtering sampling time base and interval of the CMP.
 *
 * @param   FILT_TimeBase - the digital filtering sampling time base.
 *            CMP_FILT_TimeBase_Div1 - HSE or HSI clock/1.
 *            CMP_FILT_TimeBase_Div2 - HSE or HSI clock/2.
 *            CMP_FILT_TimeBase_Div3 - HSE or HSI clock/3.
 *            CMP_FILT_TimeBase_Div4 - HSE or HSI clock/4.
 *          FILT_TimeInterval - the digital filtering sampling interval.
 *            This parameter must be a 9bit value.
 *
 * @return  none
 */
void OPA_CMP_FILTConfig(uint8_t FILT_TimeBase, uint16_t FILT_TimeInterval)
{
    OPA->CMP_CTLR &= ~(OPA_CMP_CTLR_FILT_CFG | OPA_CMP_CTLR_FILT_BASE);
    OPA->CMP_CTLR |= ((uint32_t)FILT_TimeBase << 28) | ((uint32_t)FILT_TimeInterval << 16);
}

/*********************************************************************
 * @fn      OPA_CMP_GetOutStatus
 *
 * @brief   Obtain the output status of the comparator.
 *
 * @param   none
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus OPA_CMP_GetOutStatus(void)
{
    FlagStatus bitstatus = RESET;

    if((OPA->CMP_STATR & OPA_CMP_STATR_OUTFILT) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

