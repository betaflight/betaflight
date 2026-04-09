/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_dfsdm.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the  
*                      DFSDM firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_dfsdm.h"
#include "ch32h417_rcc.h"

/* DFSDM register bit offset */
#define CHyCFGR1_CKOUTDIV_OFFSET                ((uint8_t)0x10)
#define CHyCFGR1_CKOUTSRC_OFFSET                ((uint8_t)0x1E)
#define CHyCFGR2_DTRBS_OFFSET                   ((uint8_t)0x03)
#define CHyCFGR2_CALIBOFF_OFFSET                ((uint8_t)0x08)
#define CHyAWSCDR_AWFOSR_OFFSET                 ((uint8_t)0x10)
#define CHyAWSCDR_AWFORD_OFFSET                 ((uint8_t)0x16)
#define FLTxCR1_AWFSEL_OFFSET                   ((uint8_t)0x1E)
#define FLTxCR1_RCH_OFFSET                      ((uint8_t)0x18)
#define FLTxCR2_AWDCH_OFFSET                    ((uint8_t)0x10)
#define FLTxFCR3_FOSR_OFFSET                    ((uint8_t)0x10)
#define FLTxFCR3_FORD_OFFSET                    ((uint8_t)0x1D)
#define FLTxJDATAR_JDATA_OFFSET                 ((uint8_t)0x08)
#define FLTxRDATAR_RDATA_OFFSET                 ((uint8_t)0x08)
#define FLTxAWHTR_AWDHT_OFFSET                  ((uint8_t)0x08)
#define FLTxAWLTR_AWDLT_OFFSET                  ((uint8_t)0x08)
#define FLTxEXMAX_EXMAX_OFFSET                  ((uint8_t)0x08)
#define FLTYEMMIN_EXMIN_OFFSET                  ((uint8_t)0x08)

/* DFSDM Flag Mask */
#define DFSDM_Flag_Mask                         ((uint8_t)0x1F)
/* DFSDM IT Flag Mask */
#define DFSDM_IT_Flag_Mask                      ((uint8_t)0x7F)
/* DFSDM data Mask */
#define DFSDM_Data_Mask                         ((uint32_t)0xFF000000)
#define DFSDM_SignBit_Mask                      ((uint32_t)0x00800000)

/*********************************************************************
 * @fn      DFSDM_DeInit
 *
 * @brief   Deinitializes the DFSDM peripheral registers to their default
 *        reset values.
 *
 * @param   none
 *
 * @return  none
 */
void DFSDM_DeInit(void)
{
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_DFSDM, ENABLE);
    RCC_HB2PeriphResetCmd(RCC_HB2Periph_DFSDM, DISABLE);
}

/*********************************************************************
 * @fn      DFSDM_ChannelStructInit
 *
 * @brief   initialize the parameters of DFSDM channel struct with the default values.
 *
 * @param   DFSDM_ChannelInitStruct - pointer to an DFSDM_ChannelInitTypeDef structure that
 *        contains the configuration information for the specified DFSDM_Channel.
 *
 * @return  none
 */
void DFSDM_ChannelStructInit(DFSDM_ChannelInitTypeDef *DFSDM_ChannelInitStruct)
{
    DFSDM_ChannelInitStruct->DFSDM_ChDataPackMode = DFSDM_StandardMode;
    DFSDM_ChannelInitStruct->DFSDM_ChDataMultiplexer = DFSDM_SerialInput;
    DFSDM_ChannelInitStruct->DFSDM_ChInPinSelect = DFSDM_SelectCurrent;
    DFSDM_ChannelInitStruct->DFSDM_ChClockAbsenceDetMode = DISABLE;
    DFSDM_ChannelInitStruct->DFSDM_ChShortCircuitDetMode = DISABLE;
    DFSDM_ChannelInitStruct->DFSDM_ChSPIClockSource = DFSDM_ExternalClkIn;
    DFSDM_ChannelInitStruct->DFSDM_ChSerialInterface = DFSDM_SPIRising;
    DFSDM_ChannelInitStruct->DFSDM_ChCalibrationOffset = 0;
    DFSDM_ChannelInitStruct->DFSDM_ChDataRightBitShift = 0;
    DFSDM_ChannelInitStruct->DFSDM_ChAWDSincFilterOrder = DFSDM_AWD_FastSinc;
    DFSDM_ChannelInitStruct->DFSDM_ChAWDFilterOverSample = DFSDM_AWD_FLT_Bypass;
    DFSDM_ChannelInitStruct->DFSDM_ChSCDBreakSignal = DFSDM_SCD_BK_None;
    DFSDM_ChannelInitStruct->DFSDM_ChSCDCntthreshold = 0;
}

/*********************************************************************
 * @fn      DFSDM_FilterStructInit
 *
 * @brief   initialize the parameters of DFSDM filter struct with the default values.
 *
 * @param   DFSDM_FilterInitStruct - pointer to an DFSDM_FilterInitTypeDef structure that
 *          contains the configuration information for the specified DFSDM_Filter.
 *
 * @return  none
 */
void DFSDM_FilterStructInit(DFSDM_FilterInitTypeDef *DFSDM_FilterInitStruct)
{
    DFSDM_FilterInitStruct->DFSDM_FltAWDFastMode = DISABLE;
    DFSDM_FilterInitStruct->DFSDM_FltAWDChannel = DFSDM_AWD_Channel_Disable;
    DFSDM_FilterInitStruct->DFSDM_FltAWDHighThreshold = 0;
    DFSDM_FilterInitStruct->DFSDM_FltAWDLowThreshold = 0;
    DFSDM_FilterInitStruct->DFSDM_FltExtremeChannel = DFSDM_Extremes_Channel_Disable;
    DFSDM_FilterInitStruct->DFSDM_FltSincOrder = DFSDM_FLT_FastSinc;
    DFSDM_FilterInitStruct->DFSDM_FltOverSample = DFSDM_FLT_Bypass;
    DFSDM_FilterInitStruct->DFSDM_FltIntegratorOverSample = DFSDM_FLT_IOSR_Bypass;
    DFSDM_FilterInitStruct->DFSDM_FltAWDHighThrBreakSignal = DFSDM_AWDH_BK_None;
    DFSDM_FilterInitStruct->DFSDM_FltAWDLowThrBreakSignal = DFSDM_AWDL_BK_None;
}

/*********************************************************************
 * @fn      DFSDM_RcStructInit
 *
 * @brief   initialize the parameters of regular conversion struct with the default values.
 *
 * @param   DFSDM_RcInitStruct - pointer to an DFSDM_RcInitTypeDef structure that contains
 *        the configuration information for the specified regular conversion.
 *
 * @return  none
 */
void DFSDM_RcStructInit(DFSDM_RcInitTypeDef *DFSDM_RcInitStruct)
{
    DFSDM_RcInitStruct->DFSDM_RcContinuousMode = DISABLE;
    DFSDM_RcInitStruct->DFSDM_RcFastMode = DISABLE;
    DFSDM_RcInitStruct->DFSDM_RcDMAMode = DISABLE;
    DFSDM_RcInitStruct->DFSDM_RcSynchronousMode = DISABLE;
    DFSDM_RcInitStruct->DFSDM_RcChannel = DFSDM_RC_Channel0;
}

/*********************************************************************
 * @fn      DFSDM_JcStructInit
 *
 * @brief   initialize the parameters of injected conversion struct with the default values.
 *
 * @param   DFSDM_JcInitStruct - pointer to an DFSDM_JcInitTypeDef structure that contains
 *        the configuration information for the specified regular conversion.
 *
 * @return  none
 */
void DFSDM_JcStructInit(DFSDM_JcInitTypeDef *DFSDM_JcInitStruct)
{
    DFSDM_JcInitStruct->DFSDM_JcScanMode = DISABLE;
    DFSDM_JcInitStruct->DFSDM_JcDMAMode = DISABLE;
    DFSDM_JcInitStruct->DFSDM_JcChannelGroup = DFSDM_JC_Channel0;
    DFSDM_JcInitStruct->DFSDM_JcSynchronousMode = DISABLE;
    DFSDM_JcInitStruct->DFSDM_JcTriggerEdge = DFSDM_JC_Trigger_Disable;
    DFSDM_JcInitStruct->DFSDM_JcTriggerSignal = DFSDM_JC_Trigger_TIM1;
}

/*********************************************************************
 * @fn      DFSDM_ChannelInit
 *
 * @brief   Initializes the DFSDM channel according to the specified
 *          parameters in the DFSDM_ChannelInitStruct.
 *
 * @param   DFSDM_Channely - (y = 0,1)select the channel.
 *          DFSDM_ChannelInitStruct - pointer to a DFSDM_ChannelInitTypeDef structure
 *        that contains the configuration information for the specified channel.
 *
 * @return  none
 */
void DFSDM_ChannelInit(DFSDM_Channel_TypeDef *DFSDM_Channely, DFSDM_ChannelInitTypeDef *DFSDM_ChannelInitStruct)
{
    uint32_t reg;

    reg = DFSDM_Channely->CFGR1;
    reg &= ~(DFSDM_CFGR1_DFSDMEN | DFSDM_CFGR1_SPICKSEL | DFSDM_CFGR1_SITP | DFSDM_CFGR1_SCDEN | DFSDM_CFGR1_CKABEN | 
             DFSDM_CFGR1_CHINSEL | DFSDM_CFGR1_DATMPX | DFSDM_CFGR1_DATPACK);
    reg |= (uint32_t)(((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChSPIClockSource) | ((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChSerialInterface) |
                     ((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChDataMultiplexer) | ((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChInPinSelect) |
                     ((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChClockAbsenceDetMode << 0x6) |
                     ((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChShortCircuitDetMode << 0x5) |
                     ((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChDataPackMode));

    DFSDM_Channely->CFGR1 = reg;

    DFSDM_Channely->CFGR2 = 0;
    DFSDM_Channely->CFGR2 |= (((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChCalibrationOffset << CHyCFGR2_CALIBOFF_OFFSET) |
            ((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChDataRightBitShift << CHyCFGR2_DTRBS_OFFSET));

    DFSDM_Channely->AWSCDR = 0;
    DFSDM_Channely->AWSCDR |= (uint32_t)(((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChSCDCntthreshold) | ((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChSCDBreakSignal) |
                      ((uint32_t)DFSDM_ChannelInitStruct->DFSDM_ChAWDSincFilterOrder << CHyAWSCDR_AWFORD_OFFSET) |
                      ((uint32_t)(DFSDM_ChannelInitStruct->DFSDM_ChAWDFilterOverSample - 1) << CHyAWSCDR_AWFOSR_OFFSET));
}

/*********************************************************************
 * @fn      DFSDM_FilterInit
 *
 * @brief   Initializes the DFSDM filter according to the specified
 *          parameters in the DFSDM_FilterInitStruct.
 *
 * @param   DFSDM_FLTx - (x = 0,1)select the filter.
 *          DFSDM_ChannelInitStruct - pointer to a DFSDM_FilterInitTypeDef structure
 *        that contains the configuration information for the specified channel.
 *
 * @return  none
 */
void DFSDM_FilterInit(DFSDM_FLT_TypeDef *DFSDM_FLTx, DFSDM_FilterInitTypeDef *DFSDM_FilterInitStruct)
{
    uint32_t reg;

    reg = DFSDM_FLTx->CR1;
    reg &= ~(DFSDM_FLTCR1_AWFSEL);
    reg |= ((uint32_t)DFSDM_FilterInitStruct->DFSDM_FltAWDFastMode << FLTxCR1_AWFSEL_OFFSET);
    DFSDM_FLTx->CR1 = reg;

    reg = DFSDM_FLTx->CR2;
    reg &= ~(DFSDM_FLTCR2_AWDCH | DFSDM_FLTCR2_EXCH);
    reg |= (((uint32_t)DFSDM_FilterInitStruct->DFSDM_FltAWDChannel << FLTxCR2_AWDCH_OFFSET) |
            (uint32_t)DFSDM_FilterInitStruct->DFSDM_FltExtremeChannel);
    DFSDM_FLTx->CR2 = reg;

    DFSDM_FLTx->FCR3 &= ~(DFSDM_FLTFCR3_FORD | DFSDM_FLTFCR3_FOSR | DFSDM_FLTFCR3_IOSR);
    DFSDM_FLTx->FCR3 |= (((uint32_t)DFSDM_FilterInitStruct->DFSDM_FltSincOrder << FLTxFCR3_FORD_OFFSET) |
            ((uint32_t)(DFSDM_FilterInitStruct->DFSDM_FltOverSample - 1) << FLTxFCR3_FOSR_OFFSET) |
            ((uint32_t)(DFSDM_FilterInitStruct->DFSDM_FltIntegratorOverSample)));

    DFSDM_FLTx->AWHTR &= ~(DFSDM_FLTAWHTR_AWHT | DFSDM_FLTAWHTR_BKAWH);
    DFSDM_FLTx->AWHTR |= (((uint32_t)DFSDM_FilterInitStruct->DFSDM_FltAWDHighThreshold << FLTxAWHTR_AWDHT_OFFSET) |
            (uint32_t)DFSDM_FilterInitStruct->DFSDM_FltAWDHighThrBreakSignal);

    DFSDM_FLTx->AWLTR &= ~(DFSDM_FLTAWLTR_AWLT | DFSDM_FLTAWLTR_BKAWL);
    DFSDM_FLTx->AWLTR |= (((uint32_t)DFSDM_FilterInitStruct->DFSDM_FltAWDLowThreshold << FLTxAWLTR_AWDLT_OFFSET) |
            (uint32_t)DFSDM_FilterInitStruct->DFSDM_FltAWDLowThrBreakSignal);
}

/*********************************************************************
 * @fn      DFSDM_RcInit
 *
 * @brief   Initializes the regular conversion according to the specified
 *          parameters in the DFSDM_RcInitStruct.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          DFSDM_RcInitStruct - the initialization data needed to initialize regular conversion.
 *
 * @return  none
 */
void DFSDM_RcInit(DFSDM_FLT_TypeDef *DFSDM_FLTx, DFSDM_RcInitTypeDef *DFSDM_RcInitStruct)
{
    uint32_t reg;

    reg = DFSDM_FLTx->CR1;
    reg &= ~(DFSDM_FLTCR1_FAST | DFSDM_FLTCR1_RCH | DFSDM_FLTCR1_RDMAEN | DFSDM_FLTCR1_RSYNC | DFSDM_FLTCR1_RCONT);
    reg |= (((uint32_t)DFSDM_RcInitStruct->DFSDM_RcContinuousMode << 0x12) | ((uint32_t)DFSDM_RcInitStruct->DFSDM_RcFastMode << 0x1D) |
            ((uint32_t)DFSDM_RcInitStruct->DFSDM_RcDMAMode << 0x15) | ((uint32_t)DFSDM_RcInitStruct->DFSDM_RcSynchronousMode << 0x13) |
            ((uint32_t)DFSDM_RcInitStruct->DFSDM_RcChannel << FLTxCR1_RCH_OFFSET));
    DFSDM_FLTx->CR1 = reg;
}

/*********************************************************************
 * @fn      DFSDM_JcInit
 *
 * @brief   Initializes the injected conversion according to the specified
 *          parameters in the DFSDM_JcInitStruct.
 *
 * @param   DFSDM_FLTx- (x=0,1)select the filter.
 *          DFSDM_JcInitStruct - the initialization data needed to initialize injected conversion.
 *
 * @return  none
 */
void DFSDM_JcInit(DFSDM_FLT_TypeDef *DFSDM_FLTx, DFSDM_JcInitTypeDef *DFSDM_JcInitStruct)
{
    uint32_t reg;

    reg = DFSDM_FLTx->CR1;
    reg &= ~(DFSDM_FLTCR1_JEXTEN | DFSDM_FLTCR1_JEXTSEL | DFSDM_FLTCR1_JDMAEN | DFSDM_FLTCR1_JSCAN | DFSDM_FLTCR1_JSYNC);
    reg |= (uint32_t)(DFSDM_JcInitStruct->DFSDM_JcTriggerEdge | DFSDM_JcInitStruct->DFSDM_JcTriggerSignal |
         ((uint32_t)DFSDM_JcInitStruct->DFSDM_JcDMAMode << 0x5) | ((uint32_t)DFSDM_JcInitStruct->DFSDM_JcScanMode << 0x4) |
         ((uint32_t)DFSDM_JcInitStruct->DFSDM_JcSynchronousMode << 0x3));
    DFSDM_FLTx->CR1 = reg;

    // DFSDM_FLTx->JCHGR &= ~DFSDM_FLTJCHGR_JCHG;
    DFSDM_FLTx->JCHGR = (uint32_t)DFSDM_JcInitStruct->DFSDM_JcChannelGroup;
}

/*********************************************************************
 * @fn      DFSDM_OutSerialClkConfig
 *
 * @brief   Configure output serial clock source and divider.
 *
 * @param   Source - output serial clock source selection.
 *        only the following parameters can be selected:
 *            DFSDM_SysClk - source for output clock is from system clock.
 *            DFSDM_AudioClk - source for output clock is from audio clock.
 *          Div - output serial clock divider 0-255
 *            0 - No clock.
 *            others - Clock/(Div+1)
 *
 * @return  none
 */
void DFSDM_OutSerialClkConfig(uint16_t Source, uint8_t Div)
{
    DFSDM_Channel0->CFGR1 &= ~(DFSDM_CFGR1_CKOUTSRC | DFSDM_CFGR1_CKOUTDIV);
    DFSDM_Channel0->CFGR1 |= (uint32_t)((Source << CHyCFGR1_CKOUTSRC_OFFSET) | (Div << CHyCFGR1_CKOUTDIV_OFFSET));
}

/*********************************************************************
 * @fn      DFSDM_Cmd
 *
 * @brief   Enables or disables the DFSDM interface.
 *
 * @param   NewState - new state of the DFSDM interface(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_Cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_Channel0->CFGR1 |= DFSDM_CFGR1_DFSDMEN;
    }
    else
    {
        DFSDM_Channel0->CFGR1 &= ~DFSDM_CFGR1_DFSDMEN;
    }
}

/*********************************************************************
 * @fn      DFSDM_ChannelCmd
 *
 * @brief   Enables or disables the DFSDM channel.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          NewState - new state of the DFSDM channel(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_ChannelCmd(DFSDM_Channel_TypeDef *DFSDM_Channely, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_Channely->CFGR1 |= DFSDM_CFGR1_CHEN;
    }
    else
    {
        DFSDM_Channely->CFGR1 &= ~DFSDM_CFGR1_CHEN;
    }
}

/*********************************************************************
 * @fn      DFSDM_SPIClockSourceConfig
 *
 * @brief   Configure the SPI clock source.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Source - SPI clock source.
 *        only the following parameters can be selected:
 *            DFSDM_ExternalClkIn - external CKINy input.
 *            DFSDM_InternalClkOut - internal CKOUT output.
 *            DFSDM_InternalHalfFall - internal CKOUT(falling edge).
 *            DFSDM_InternalHalfRise - internal CKOUT(rising edge).
 *                  
 * @return  none
 */
void DFSDM_SPIClockSourceConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Source)
{
    DFSDM_Channely->CFGR1 &= ~DFSDM_CFGR1_SPICKSEL;
    DFSDM_Channely->CFGR1 |= Source;
}

/*********************************************************************
 * @fn      DFSDM_SerialInterfaceConfig
 *
 * @brief   Configure serial interface type.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          type - serial interface type.
 *        only the following parameters can be selected:
 *            DFSDM_SPIRising - SPI with rising edge to strobe data.
 *            DFSDM_SPIFalling - SPI with falling edge to strobe data.
 *                  
 * @return  none
 */
void DFSDM_SerialInterfaceConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Type)
{
    DFSDM_Channely->CFGR1 &= ~DFSDM_CFGR1_SITP;
    DFSDM_Channely->CFGR1 |= (uint32_t)Type;
}

/*********************************************************************
 * @fn      DFSDM_ShortCircuitDetCmd
 *
 * @brief   Enables or disables the short-circuit detector.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          NewState - new state of short circuit detector(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_ShortCircuitDetCmd(DFSDM_Channel_TypeDef *DFSDM_Channely, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_Channely->CFGR1 |= DFSDM_CFGR1_SCDEN;
    }
    else
    {
        DFSDM_Channely->CFGR1 &= ~DFSDM_CFGR1_SCDEN;
    }
}

/*********************************************************************
 * @fn      DFSDM_ClockAbsenceDetCmd
 *
 * @brief   Enables or disables the clock absence detector.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          NewState - new state of clock absence detector(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_ClockAbsenceDetCmd(DFSDM_Channel_TypeDef *DFSDM_Channely, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_Channely->CFGR1 |= DFSDM_CFGR1_CKABEN;
    }
    else
    {
        DFSDM_Channely->CFGR1 &= ~DFSDM_CFGR1_CKABEN;
    }
}

/*********************************************************************
 * @fn      DFSDM_ChannelInputSelect
 *
 * @brief   Channel inputs selection.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Pin_sel - channel inputs selection
 *        only the following parameters can be selected:
 *            DFSDM_SelectCurrent - current channel.
 *            DFSDM_SelectNext - next channel.
 *
 * @return  none
 */
void DFSDM_ChannelInputSelect(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Pin_sel)
{
    DFSDM_Channely->CFGR1 &= ~DFSDM_CFGR1_CHINSEL;
    DFSDM_Channely->CFGR1 |= (uint32_t)Pin_sel;
}

/*********************************************************************
 * @fn      DFSDM_DataPackModeConfig
 *
 * @brief   Configure data packing mode.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Mode - data packing mode.
 *        only the following parameters can be selected:
 *            DFSDM_StandardMode - standard mode.
 *            DFSDM_InterleaveMode - interleaved mode.
 *            DFSDM_DualMode - dual mode.
 *                  
 * @return  none
 */
void DFSDM_DataPackModeConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Mode)
{
    DFSDM_Channely->CFGR1 &= ~DFSDM_CFGR1_DATPACK;
    DFSDM_Channely->CFGR1 |= (uint32_t)Mode;
}

/*********************************************************************
 * @fn      DFSDM_ChannelInDataMpxConfig
 *
 * @brief   Configure Input data multiplexer for channel.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Source - input data source.
 *        only the following parameters can be selected:
 *            DFSDM_SerialInput - external serial inputs.
 *            DFSDM_ADCInput - ADC output register.
 *            DFSDM_InternalInput - internal register.
 *                  
 * @return  none
 */
void DFSDM_ChannelInDataMpxConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Source)
{
    DFSDM_Channely->CFGR1 &= ~DFSDM_CFGR1_DATMPX;
    DFSDM_Channely->CFGR1 |= (uint32_t)Source;
}

/*********************************************************************
 * @fn      DFSDM_CalibrationOffsetConfig
 *
 * @brief   Configure calibration offset for channel.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Offset - 24-bit calibration offset.
 *                  
 * @return  none
 */
void DFSDM_CalibrationOffsetConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, int32_t Offset)
{
    DFSDM_Channely->CFGR2 &= ~DFSDM_CFGR2_OFFSET;
    DFSDM_Channely->CFGR2 |= ((uint32_t)Offset << CHyCFGR2_CALIBOFF_OFFSET);
}

/*********************************************************************
 * @fn      DFSDM_DataRightBitShiftConfig
 *
 * @brief   Configure data right bit-shift for channel.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Shift - the right shift(0-8 is valid).
 *                  
 * @return  none
 */
void DFSDM_DataRightBitShiftConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint8_t Shift)
{
    DFSDM_Channely->CFGR2 &= ~DFSDM_CFGR2_DTRBS;
    DFSDM_Channely->CFGR2 |= ((uint32_t)Shift << CHyCFGR2_DTRBS_OFFSET);
}

/*********************************************************************
 * @fn      DFSDM_SCDBreakSignalConfig
 *
 * @brief   Configure break signal assignment for short-circuit detector.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Signal -  break signal assignment.
 *        only the following parameters can be selected:
 *            DFSDM_SCD_BK_None - break signal0 and signal1 is not assigned.
 *            DFSDM_SCD_BK_0 - break signal0 is assigned, break signal1 is not assigned.
 *            DFSDM_SCD_BK_1 - break signal1 is assigned, break signal0 is not assigned.
 *            DFSDM_SCD_BK_0_1 - break signal0 and signal1 is assigned.
 *                  
 * @return  none
 */
void DFSDM_SCDBreakSignalConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Signal)
{
    DFSDM_Channely->AWSCDR &= ~DFSDM_AWSCDR_BKSCD;
    DFSDM_Channely->AWSCDR |= (uint32_t)Signal;
}

/*********************************************************************
 * @fn      DFSDM_SCDCounterThrConfig
 *
 * @brief   Configure short-circuit detector threshold.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Threshold - the threshold counter for short-circuit detector(0-255).
 *                  
 * @return  none
 */
void DFSDM_SCDCounterThrConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint8_t Threshold)
{
    DFSDM_Channely->AWSCDR &= ~DFSDM_AWSCDR_SCDT;
    DFSDM_Channely->AWSCDR |= (uint32_t)Threshold;
}

/*********************************************************************
 * @fn      DFSDM_WriteParallelDataStanMode
 *
 * @brief   Write the parallel data on standard mode of data packing.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Data - the parallel data.
 *                  
 * @return  none
 */
void DFSDM_WriteParallelDataStanMode(DFSDM_Channel_TypeDef *DFSDM_Channely, int16_t Data)
{
    DFSDM_Channely->DATINR0 = (int16_t)Data;
}

/*********************************************************************
 * @fn      DFSDM_WriteParallelDataIntlMode
 *
 * @brief   Write the parallel data on interleaved mode of data packing.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Data0 - the parallel data0.
 *          Data1 - the parallel data1.
 *                  
 * @return  none
 */
void DFSDM_WriteParallelDataIntlMode(DFSDM_Channel_TypeDef *DFSDM_Channely, int16_t Data0, int16_t Data1)
{
    DFSDM_Channely->DATINR0 = (int16_t)Data0;
    DFSDM_Channely->DATINR1 = (int16_t)Data1;
}

/*********************************************************************
 * @fn      DFSDM_WriteParallelDataDualMode
 *
 * @brief   Write the parallel data on dual mode of data packing.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Data0 - the parallel data0.
 *          Data1 - the parallel data1.
 *                  
 * @return  none
 */
void DFSDM_WriteParallelDataDualMode(DFSDM_Channel_TypeDef *DFSDM_Channely, int16_t Data0, int16_t Data1)
{
    DFSDM_Channely->DATINR0 = (int16_t)Data0;
    DFSDM_Channely->DATINR1 = (int16_t)Data1;
}

/*********************************************************************
 * @fn      DFSDM_FilterCmd
 *
 * @brief   Enables or disables the DFSDM filter.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          NewState - new state of the DFSDM filter(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_FilterCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_DFEN;
    }
    else
    {
        DFSDM_FLTx->CR1 &= ~DFSDM_FLTCR1_DFEN;
    }
}

/*********************************************************************
 * @fn      DFSDM_FilterConfig
 *
 * @brief   Configure DFSDM sinc filter order,oversample and integrator oversample.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          order - sinc filter order
 *        only the following parameters can be selected:
 *            DFSDM_FLT_FastSinc - FastSinc filter type.
 *            DFSDM_FLT_Sinc1 - Sinc1 filter type.
 *            DFSDM_FLT_Sinc2 - Sinc2 filter type.
 *            DFSDM_FLT_Sinc3 - Sinc3 filter type.
 *            DFSDM_FLT_Sinc4 - Sinc4 filter type.
 *            DFSDM_FLT_Sinc5 - Sinc5 filter type.
 *         Fosr - sinc filter oversampling rate(1-1024).
 *         Iosr - integrator oversampling rate(0-2^Iosr).
 *                  
 * @return  none
 */
void DFSDM_FilterConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Order, uint16_t Fosr, uint16_t Iosr)
{
    DFSDM_FLTx->FCR3 &= ~(DFSDM_FLTFCR3_FORD | DFSDM_FLTFCR3_FOSR | DFSDM_FLTFCR3_IOSR);
    DFSDM_FLTx->FCR3 |= (((uint32_t)Order << FLTxFCR3_FORD_OFFSET) |
    ((uint32_t)(Fosr - 1) << FLTxFCR3_FOSR_OFFSET) | (uint32_t)(Iosr));
}
/*********************************************************************
 * @fn      DFSDM_AWDFilterConfig
 *
 * @brief   Configure analog watchdog Sinc filter order and oversample.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *          Order - analog watchdog Sinc filter order
 *        only the following parameters can be selected:
 *            DFSDM_AWD_FastSinc - FastSinc filter type.
 *            DFSDM_AWD_Sinc1 - Sinc1 filter type.
 *            DFSDM_AWD_Sinc2 - Sinc2 filter type.
 *            DFSDM_AWD_Sinc3 - Sinc3 filter type.
 *          Owfosr - Sinc filter oversampling rate(1-32).
 *                  
 * @return  none
 */
void DFSDM_AWDFilterConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Order, uint8_t Awfosr)
{
    DFSDM_Channely->AWSCDR &= ~(DFSDM_AWSCDR_AWFORD | DFSDM_AWSCDR_AWFOSR);
    DFSDM_Channely->AWSCDR |= (((uint32_t)Order << CHyAWSCDR_AWFORD_OFFSET) |
            ((uint32_t)(Awfosr - 1) << CHyAWSCDR_AWFOSR_OFFSET));
}

/*********************************************************************
 * @fn      DFSDM_ReadAWDFilterData
 *
 * @brief   Read the analog watchdog filter data.
 *
 * @param   DFSDM_Channely - (y=0,1)select the channel.
 *                  
 * @return  the input DFSDM_Channely watchdog data.
 */
int16_t DFSDM_ReadAWDFilterData(DFSDM_Channel_TypeDef *DFSDM_Channely)
{
    return (int16_t)DFSDM_Channely->WDATR;
}

/*********************************************************************
 * @fn      DFSDM_AWDFilterFastModeCmd
 *
 * @brief   Enables or disables the analog watchdog filter fast mode.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          NewState - new state of the analog watchdog filter fast mode(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_AWDFilterFastModeCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_AWFSEL;
    }
    else
    {
        DFSDM_FLTx->CR1 &= ~DFSDM_FLTCR1_AWFSEL;
    }
}

/*********************************************************************
 * @fn      DFSDM_AWDChannelConfig
 *
 * @brief   Configure the analog watchdog channel.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          Channel - select channel y(y=0,1).
 *        only the following parameters can be selected:
 *            DFSDM_AWD_Channel_Disable - disabled all channel.
 *            DFSDM_AWD_Channel0 - select channel0.
 *            DFSDM_AWD_Channel1 - select channel1.
 *            DFSDM_AWD_Channel0_1 - select channel0 and channel1.
 *                  
 * @return  none
 */
void DFSDM_AWDChannelConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Channel)
{
    DFSDM_FLTx->CR2 &= ~DFSDM_FLTCR2_AWDCH;
    DFSDM_FLTx->CR2 |= (uint32_t)Channel << FLTxCR2_AWDCH_OFFSET;
}

/*********************************************************************
 * @fn      DFSDM_AWDHighThresholdConfig
 *
 * @brief   Configure analog watchdog high threshold value.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          Value - high threshold value(-8388608~8388607).
 *                  
 * @return  none
 */
void DFSDM_AWDHighThresholdConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, int32_t Value)
{
    DFSDM_FLTx->AWHTR &= ~DFSDM_FLTAWHTR_AWHT;
    DFSDM_FLTx->AWHTR |= (uint32_t)Value << FLTxAWHTR_AWDHT_OFFSET;
}

/*********************************************************************
 * @fn      DFSDM_AWDLowThresholdConfig
 *
 * @brief   Configure analog watchdog low threshold value.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          Value - low threshold value(-8388608~8388607).
 *                  
 * @return  none
 */
void DFSDM_AWDLowThresholdConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, int32_t Value)
{
    DFSDM_FLTx->AWLTR &= ~DFSDM_FLTAWLTR_AWLT;
    DFSDM_FLTx->AWLTR |= (uint32_t)Value << FLTxAWLTR_AWDLT_OFFSET;
}

/*********************************************************************
 * @fn      DFSDM_AWDHighThrBKConfig
 *
 * @brief   Configure analog watchdog high threshold event break signal.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          Signal - high threshold event break signal.
 *        only the following parameters can be selected:
 *            DFSDM_AWDH_BK_None - break signal0 and signal1 is not assigned.
 *            DFSDM_AWDH_BK_0 - break signal0 is assigned.
 *            DFSDM_AWDH_BK_1 - break signal1 is assigned.
 *            DFSDM_AWDH_BK_0_1 - break signal0 and signal1 is assigned.
 *                  
 * @return  none
 */
void DFSDM_AWDHighThrBKConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Signal)
{
    DFSDM_FLTx->AWHTR &= ~DFSDM_FLTAWHTR_BKAWH;
    DFSDM_FLTx->AWHTR |= (uint32_t)Signal;
}

/*********************************************************************
 * @fn      DFSDM_AWDLowThrBKConfig
 *
 * @brief   Configure analog watchdog low threshold event break signal.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          Signal - high threshold event break signal.
 *            DFSDM_AWDL_BK_None - break signal0 and signal1 is not assigned.
 *            DFSDM_AWDL_BK_0 - break signal0 is assigned.
 *            DFSDM_AWDL_BK_1 - break signal1 is assigned.
 *            DFSDM_AWDL_BK_0_1 - break signal0 and signal1 is assigned.
 *                  
 * @return  none
 */
void DFSDM_AWDLowThrBKConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Signal)
{
    DFSDM_FLTx->AWLTR &= ~DFSDM_FLTAWLTR_BKAWL;
    DFSDM_FLTx->AWLTR |= (uint32_t)Signal;
}

/*********************************************************************
 * @fn      DFSDM_ExtrDetChannelConfig
 *
 * @brief   Configure extremes detector channel.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          Channel - extremes detector channel selection.
 *        only the following parameters can be selected:
 *            DFSDM_Extremes_Channel_Disable - extremes detector does not accept data.
 *            DFSDM_Extremes_Channel0 - accept data from channel0.
 *            DFSDM_Extremes_Channel1 - accept data from channel1.
 *            DFSDM_Extremes_Channel0_1 - accept data from channel0 and channel1.
 *                  
 * @return  none
 */
void DFSDM_ExtrDetChannelConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Channel)
{
    DFSDM_FLTx->CR2 &= ~DFSDM_FLTCR2_EXCH;
    DFSDM_FLTx->CR2 |= (uint32_t)Channel;
}

/*********************************************************************
 * @fn      DFSDM_ReadExtrMaxData
 *
 * @brief   Read extremes detector maximum value.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *                  
 * @return  the maximum value.
 */
int32_t DFSDM_ReadExtrMaxData(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    uint32_t val;
 
    val = DFSDM_FLTx->EXMAX >> FLTxEXMAX_EXMAX_OFFSET;
    if(val & DFSDM_SignBit_Mask)
    {
        val |= DFSDM_Data_Mask;
    }
    return (int32_t)val;
}

/*********************************************************************
 * @fn      DFSDM_GetExtrMaxDataChannel
 *
 * @brief   Get extremes detector maximum value channel.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *
 * @return  the channel.
 */
uint8_t DFSDM_GetExtrMaxDataChannel(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    return (uint8_t)(DFSDM_FLTx->EXMAX & DFSDM_FLTEXMAX_EXMAXCH);
}

/*********************************************************************
 * @fn      DFSDM_ReadExtrMinData
 *
 * @brief   Read extremes detector minimum value.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *                  
 * @return  the minimum value.
 */
int32_t DFSDM_ReadExtrMinData(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    uint32_t val;

    val = DFSDM_FLTx->EXMIN >> FLTYEMMIN_EXMIN_OFFSET;
    if(val & DFSDM_SignBit_Mask)
    {
        val |= DFSDM_Data_Mask;
    }
    return (int32_t)val;
}

/*********************************************************************
 * @fn      DFSDM_GetExtrMinDataChannel
 *
 * @brief   Get extremes detector minimum value channel.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *
 * @return  the channel.
 */
uint8_t DFSDM_GetExtrMinDataChannel(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    return (uint8_t)(DFSDM_FLTx->EXMIN & DFSDM_FLTEXMIN_EXMINCH);
}

/*********************************************************************
 * @fn      DFSDM_ReadCntConvTimeData
 *
 * @brief   Read the counting conversion time.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *                  
 * @return  counting conversion time value.
 */
uint32_t DFSDM_ReadCntConvTimeData(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    return (uint32_t)(DFSDM_FLTx->NVTIMR + 1);
}

/*********************************************************************
 * @fn      DFSDM_RcFastConvCmd
 *
 * @brief   Enables or disables the regular conversion fast conversion mode.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          NewState - new state of the regular conversion fast conversion mode(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_RcFastConvCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_FAST;
    }
    else
    {
        DFSDM_FLTx->CR1 &= ~DFSDM_FLTCR1_FAST;
    }  
}

/*********************************************************************
 * @fn      DFSDM_RcChannelConfig
 *
 * @brief   Configure regular conversion channel.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          Channel - the regular conversion channel selection.
 *        only the following parameters can be selected:
 *          DFSDM_RC_Channel0 - channel 0 is selected.
 *          DFSDM_RC_Channel1 - channel 1 is selected.
 *
 * @return  none
 */
void DFSDM_RcChannelConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Channel)
{
    uint32_t reg;

    reg = DFSDM_FLTx->CR1;
    reg &= ~DFSDM_FLTCR1_RCH;
    reg |= (uint32_t)Channel << FLTxCR1_RCH_OFFSET;
    DFSDM_FLTx->CR1 = reg;
}

/*********************************************************************
 * @fn      DFSDM_RcDMACmd
 *
 * @brief   Enables or disables the regular conversion DMA channel.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          NewState - new state of DMA channel(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_RcDMACmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_RDMAEN;
    }
    else
    {
        DFSDM_FLTx->CR1 &= ~DFSDM_FLTCR1_RDMAEN;
    }  
}

/*********************************************************************
 * @fn      DFSDM_RcConvSynsCmd
 *
 * @brief   Enables or disables Launch regular conversion synchronously.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          NewState - new state of conversion synchronously(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_RcConvSynsCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_RSYNC;
    }
    else
    {
        DFSDM_FLTx->CR1 &= ~DFSDM_FLTCR1_RSYNC;
    }    
}

/*********************************************************************
 * @fn      DFSDM_RcContinuousCmd
 *
 * @brief   Enables or disables continuous mode for regular conversions.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          NewState - new state of continuous mode(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_RcContinuousCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_RCONT;
    }
    else
    {
        DFSDM_FLTx->CR1 &= ~DFSDM_FLTCR1_RCONT;
    }
}

/*********************************************************************
 * @fn      DFSDM_RcSoftStartConversion
 *
 * @brief   Software start of a conversion on the regular channel.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *
 * @return  none
 */
void DFSDM_RcSoftStartConversion(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_RSWSTART;
}

/*********************************************************************
 * @fn      DFSDM_RcReadData
 *
 * @brief   Read the regular conversions data.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *
 * @return  the regular conversions data.
 */
int32_t DFSDM_RcReadConvData(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    uint32_t val;

    val = DFSDM_FLTx->RDATAR >> FLTxRDATAR_RDATA_OFFSET;
    if(val & DFSDM_SignBit_Mask)
    {
        val |= DFSDM_Data_Mask;
    }
    return (int32_t)val;
}

/*********************************************************************
 * @fn      DFSDM_RcGetLatestConvChannel
 *
 * @brief   Get the regular channel most recently converted.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *
 * @return  the regular channel most recently converted.
 */
uint32_t DFSDM_RcGetLatestConvChannel(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    return (uint32_t)(DFSDM_FLTx->RDATAR & DFSDM_FLTRDATAR_RDATACH);
}

/*********************************************************************
 * @fn      DFSDM_JcTrigSignConfig
 *
 * @brief   Configure injected trigger signal.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          Trigger - trigger enable and trigger edge selection for injected conversions.
 *        This parameter can be a value of @ref Inject_Trigger_Signal
 *          Trigger_edge - trigger signal selection.
 *        only the following parameters can be selected:
 *            DFSDM_JC_Trigger_Disable - trigger detection is disabled
 *            DFSDM_JC_Trigger_Rising - each rising edge on the selected trigger
 *            DFSDM_JC_Trigger_Falling - each falling edge on the selected trigger
 *            DFSDM_JC_Trigger_Edge - both rising edges and falling edges on the selected trigger
 *
 * @return  none
 */
void DFSDM_JcTrigSignConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Trigger, uint16_t Trigger_edge)
{
    DFSDM_FLTx->CR1 &= ~(DFSDM_FLTCR1_JEXTSEL | DFSDM_FLTCR1_JEXTEN);
    DFSDM_FLTx->CR1 |= (uint32_t)(Trigger | Trigger_edge);
}

/*********************************************************************
 * @fn      DFSDM_JcDMACmd
 *
 * @brief   Enables or disables the injected conversion DMA channel.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          NewState - new state of DMA channel(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_JcDMACmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_JDMAEN;
    }
    else
    {
        DFSDM_FLTx->CR1 &= ~DFSDM_FLTCR1_JDMAEN;
    }  
}

/*********************************************************************
 * @fn      DFSDM_JcScanConvCmd
 *
 * @brief   Enables or disables scan conversion mode for injected.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          NewState - new state of scan conversion mode(ENABLE or DISABLE)
 *
 * @return  none
 */
void DFSDM_JcScanConvCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_JSCAN;
    }
    else
    {
        DFSDM_FLTx->CR1 &= ~DFSDM_FLTCR1_JSCAN;
    }  
}

/*********************************************************************
 * @fn      DFSDM_JcConvSynsCmd
 *
 * @brief   Enables or disables the injected conversion synchronously.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          NewState - new state of conversion synchronously(ENABLE or DISABLE).
 *
 * @return  none
 */
void DFSDM_JcConvSynsCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_JSYNC;
    }
    else
    {
        DFSDM_FLTx->CR1 &= ~DFSDM_FLTCR1_JSYNC;
    }    
}

/*********************************************************************
 * @fn      DFSDM_JcSoftStartConversion
 *
 * @brief   Start a conversion of the injected group of channels.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *
 * @return  none
 */
void DFSDM_JcSoftStartConversion(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    DFSDM_FLTx->CR1 |= DFSDM_FLTCR1_JSWSTART;
}

/*********************************************************************
 * @fn      DFSDM_JcChannelConfig
 *
 * @brief   Configure injected group conversions channel.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          Channel - the injected conversion group channel selection.
 *        only the following parameters can be selected:
 *            DFSDM_JC_Channel_Disable - injected group conversions channel0 and channel1 is not assigned. 
 *            DFSDM_JC_Channel0 - injected group conversions channel0 is assigned. 
 *            DFSDM_JC_Channel1 - injected group conversions channel1 is assigned. 
 *            DFSDM_JC_Channel0_1 - injected group conversions channel0 and channel1 is assigned. 
 *
 * @return  none
 */
void DFSDM_JcChannelConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Channel)
{
    DFSDM_FLTx->JCHGR = 0;
    DFSDM_FLTx->JCHGR |= (uint32_t)Channel;
}

/*********************************************************************
 * @fn      DFSDM_JcReadData
 *
 * @brief   Reas the inject conversions data.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *
 * @return  the injected conversions data.
 */
int32_t DFSDM_JcReadConvData(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    uint32_t val;

    val = DFSDM_FLTx->JDATAR >> FLTxJDATAR_JDATA_OFFSET;
    if(val & DFSDM_SignBit_Mask)
    {
        val |= 0xFF000000;
    }
    return (int32_t)val;
}

/*********************************************************************
 * @fn      DFSDM_JcGetLatestConvChannel
 *
 * @brief   Get the injected channel most recently converted.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *
 * @return  the channel.
 */
uint8_t DFSDM_JcGetLatestConvChannel(DFSDM_FLT_TypeDef *DFSDM_FLTx)
{
    return (uint8_t)(DFSDM_FLTx->JDATAR & DFSDM_FLTJDATAR_JDATACH);
}

/*********************************************************************
 * @fn      DFSDM_GetFlagStatus
 *
 * @brief   get the DFSDM flags.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          DFSDM_FLAG - specifies the flag to check
 *        only one of the following parameters can be selected:
 *            DFSDM_FLAG_FLTx_JEOCF - JEOCF flag
 *            DFSDM_FLAG_FLTx_REOCF - REOCF flag
 *            DFSDM_FLAG_FLTx_JOVRF - JOVRF flag
 *            DFSDM_FLAG_FLTx_ROVRF - ROVRF flag
 *            DFSDM_FLAG_FLTx_AWDF - AWDF flag
 *            DFSDM_FLAG_FLTx_JCIP - JCIP flag
 *            DFSDM_FLAG_FLTx_RCIP - RCIP flag
 *            DFSDM_FLAG_FLT0_CKABF0 - CKABF0 flag
 *            DFSDM_FLAG_FLT0_CKABF1 - CKABF1 flag
 *            DFSDM_FLAG_FLT0_SCDF0 - SCDF0 flag
 *            DFSDM_FLAG_FLT0_SCDF1 - SCDF1 flag
 *            DFSDM_FLAG_FLTx_RPEND - RPEND flag
 *            DFSDM_FLAG_FLTx_AWLTF0 - AWLTF0 flag
 *            DFSDM_FLAG_FLTx_AWLTF1 - AWLTF1 flag
 *            DFSDM_FLAG_FLTx_AWHTF0 - AWHTF0 flag
 *            DFSDM_FLAG_FLTx_AWHTF1 - AWHTF1 flag
 *                  
 * @return  FlagStatus - SET or RESET
 */
FlagStatus DFSDM_GetFlagStatus(DFSDM_FLT_TypeDef *DFSDM_FLTx,uint8_t DFSDM_FLAG)
{
    FlagStatus bitstatus = RESET;
    uint8_t tmp = 0;
    uint32_t statusreg = 0;
    
    tmp = DFSDM_FLAG >> 5;
    if(tmp == 0x3) DFSDM_FLTx = DFSDM_FLT0;

    switch (tmp) {
        case 1:
            statusreg = DFSDM_FLTx->ISR;
            break;
        case 2:
            statusreg = DFSDM_FLTx->RDATAR;
            break;
        case 3:
            statusreg = DFSDM_FLTx->ISR;
            break;
        case 4:
            statusreg = DFSDM_FLTx->AWSR;
            break;
        default:
            break;
    }
    tmp = DFSDM_FLAG & DFSDM_Flag_Mask;
    if ((statusreg & ((uint32_t)1 << tmp)) != (uint32_t)RESET)
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
 * @fn      DFSDM_ClearFlag
 *
 * @brief   Clears the DFSDM's pending flags.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          DFSDM_FLAG - specifies the flag to check
 *        only one of the following parameters can be selected:
 *            DFSDM_FLAG_FLTx_JEOCF - JEOCF flag
 *            DFSDM_FLAG_FLTx_REOCF - REOCF flag
 *            DFSDM_FLAG_FLTx_JOVRF - JOVRF flag
 *            DFSDM_FLAG_FLTx_ROVRF - ROVRF flag
 *            DFSDM_FLAG_FLTx_AWDF - AWDF flag
 *            DFSDM_FLAG_FLT0_CKABF0 - CKABF0 flag
 *            DFSDM_FLAG_FLT0_CKABF1 - CKABF1 flag
 *            DFSDM_FLAG_FLT0_SCDF0 - SCDF0 flag
 *            DFSDM_FLAG_FLT0_SCDF1 - SCDF1 flag
 *            DFSDM_FLAG_FLTx_AWLTF0 - AWLTF0 flag
 *            DFSDM_FLAG_FLTx_AWLTF1 - AWLTF1 flag
 *            DFSDM_FLAG_FLTx_AWHTF0 - AWHTF0 flag
 *            DFSDM_FLAG_FLTx_AWHTF1 - AWHTF1 flag               
 *
 * @return  none
 */
void DFSDM_ClearFlag(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint8_t DFSDM_FLAG)
{
    uint8_t tmp = 0;
    uint8_t bit_pos = 0;

    tmp = DFSDM_FLAG >> 5;
    bit_pos = DFSDM_FLAG & DFSDM_Flag_Mask;
    
    if(tmp == 0x4)
    {
        DFSDM_FLTx->AWCFR |= ((uint32_t)1 << bit_pos);
    }
    else 
    {
        switch (DFSDM_FLAG) {
            case DFSDM_FLAG_FLTx_JEOCF:
                (void)DFSDM_FLTx->JDATAR;
                break;
            case DFSDM_FLAG_FLTx_REOCF:
                (void)DFSDM_FLTx->RDATAR;
                break;
            case DFSDM_FLAG_FLTx_AWDF:
                DFSDM_FLTx->AWCFR |= (DFSDM_FLTAWSR_AWLTF | DFSDM_FLTAWSR_AWHTF);
                break;
            default:
            {
                if(tmp == 0x3) DFSDM_FLTx = DFSDM_FLT0;
                DFSDM_FLTx->ICR |= ((uint32_t)1 << bit_pos);
                break;
            }
        }
    }
}

/*********************************************************************
 * @fn      DFSDM_ITConfig
 *
 * @brief   Enables or disables the DFSDM interrupts.
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          Interrupt - DFSDM interrupt
 *        only one of the following parameters can be selected:
 *            DFSDM_IT_FLTx_JEOCIE - JEOCIE interrupt enable 
 *            DFSDM_IT_FLTx_REOCIE - REOCIE interrupt enable
 *            DFSDM_IT_FLTx_JOVRIE - JOVRIE interrupt enable
 *            DFSDM_IT_FLTx_ROVRIE - ROVRIE interrupt enable
 *            DFSDM_IT_FLTx_AWDIE - AWDIE interrupt enable
 *            DFSDM_IT_FLT0_SCDIE - SCDIE interrupt enable
 *            DFSDM_IT_FLT0_CKABIE - CKABIE interrupt enable
 *          NewState - the DFSDM interrupt(ENABLE or DISABLE)
 *
 * @return  none
 */
void DFSDM_ITConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint8_t Interrupt, FunctionalState NewState)
{
    uint8_t is_flt0 = 0;
    uint8_t enablestatus = 0;

    is_flt0 = (Interrupt >> 0x7) & 0x1;
    enablestatus = Interrupt & DFSDM_IT_Flag_Mask;

    if(is_flt0) DFSDM_FLTx = DFSDM_FLT0;

    if(NewState != DISABLE)
    {
        DFSDM_FLTx->CR2 |= (uint32_t)enablestatus;
    }
    else
    {
        DFSDM_FLTx->CR2 &= ~(uint32_t)enablestatus;
    }
}

/*********************************************************************
 * @fn      DFSDM_GetITStatus
 *
 * @brief   Checks whether the DFSDM interrupt has occurred or not
 *
 * @param   DFSDM_FLTx - (x=0,1)select the filter.
 *          DFSDM_FLAG - specifies the flag to check
 *        only one of the following parameters can be selected:
 *            DFSDM_IT_FLAG_FLTx_JEOCF - JEOCIE interrupt flag 
 *            DFSDM_IT_FLAG_FLTx_REOCF - REOCF interrupt flag 
 *            DFSDM_IT_FLAG_FLTx_JOVRF - JOVRF interrupt flag 
 *            DFSDM_IT_FLAG_FLTx_ROVRF - ROVRF interrupt flag 
 *            DFSDM_IT_FLAG_FLTx_AWDF - AWDF interrupt flag 
 *            DFSDM_IT_FLAG_FLT0_CKABF0 - CKABF0 interrupt flag 
 *            DFSDM_IT_FLAG_FLT0_CKABF1 - CKABF1 interrupt flag 
 *            DFSDM_IT_FLAG_FLT0_SCDF0 - SCDF0 interrupt flag 
 *            DFSDM_IT_FLAG_FLT0_SCDF1 - SCDF1 interrupt flag 
 *                  
 * @return  FlagStatus - SET or RESET
 */
ITStatus DFSDM_GetITStatus(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t DFSDM_FLAG)
{
    ITStatus bitstatus = RESET;
    uint8_t bit_pos = 0;
    uint8_t enablestatus = 0;
    uint8_t is_flt0 = 0;

    bit_pos = (DFSDM_FLAG >> 8) & DFSDM_IT_Flag_Mask;
    enablestatus = DFSDM_FLAG & DFSDM_IT_Flag_Mask;
    is_flt0 = (DFSDM_FLAG >> 15) & 0x1;

    if(is_flt0) DFSDM_FLTx = DFSDM_FLT0;

    if(((DFSDM_FLTx->ISR & ((uint32_t)1 << bit_pos)) != (uint32_t)RESET) && 
         ((DFSDM_FLTx->CR2 & (uint32_t)enablestatus) != (uint32_t)RESET))
    {
        bitstatus = SET;
    }else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      DFSDM_ClearITPendingBit
 *
 * @brief   Clears the DFSDM's interrupt pending bits.
 *
 * @param   DFSDM_FLTx:(x=0,1)select the filter.
 *          DFSDM_FLAG - specifies the flag to clear.
 *        only one of the following parameters can be selected:
 *            DFSDM_IT_FLAG_FLTx_JEOCF - JEOCIE interrupt flag 
 *            DFSDM_IT_FLAG_FLTx_REOCF - REOCF interrupt flag 
 *            DFSDM_IT_FLAG_FLTx_JOVRF - JOVRF interrupt flag 
 *            DFSDM_IT_FLAG_FLTx_ROVRF - ROVRF interrupt flag 
 *            DFSDM_IT_FLAG_FLTx_AWDF - AWDF interrupt flag 
 *            DFSDM_IT_FLAG_FLT0_CKABF0 - CKABF0 interrupt flag 
 *            DFSDM_IT_FLAG_FLT0_CKABF1 - CKABF1 interrupt flag 
 *            DFSDM_IT_FLAG_FLT0_SCDF0 - SCDF0 interrupt flag 
 *            DFSDM_IT_FLAG_FLT0_SCDF1 - SCDF1 interrupt flag 
 *
 * @return  none
 */
void DFSDM_ClearITPendingBit(DFSDM_FLT_TypeDef *DFSDM_FLTx,uint16_t DFSDM_FLAG)
{
    uint16_t bit_pos = 0; 
    uint8_t is_flt0 = 0;

    bit_pos = (DFSDM_FLAG >> 8) & DFSDM_IT_Flag_Mask;
    is_flt0 = (DFSDM_FLAG >> 15) & 0x1; 

    switch (DFSDM_FLAG) {
        case DFSDM_IT_FLAG_FLTx_JEOCF:
            (void)DFSDM_FLTx->JDATAR;
            break;
        case DFSDM_IT_FLAG_FLTx_REOCF:
            (void)DFSDM_FLTx->RDATAR;
            break;
        case DFSDM_IT_FLAG_FLTx_AWDF:
            DFSDM_FLTx->AWCFR |= (DFSDM_FLTAWSR_AWLTF | DFSDM_FLTAWSR_AWHTF);
            break;
        default:
        {
            if(is_flt0) DFSDM_FLTx = DFSDM_FLT0;
            DFSDM_FLTx->ICR |= ((uint32_t)1 << bit_pos);
            break;
        }
    }
}
