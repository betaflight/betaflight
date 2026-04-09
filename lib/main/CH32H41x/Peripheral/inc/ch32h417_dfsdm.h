/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_dfsdm.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the  
*                      DFSDM firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_DFSDM_H
#define __CH32H417_DFSDM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

/* DFSDM channel Init structure definition */
typedef struct
{
    int32_t  DFSDM_ChCalibrationOffset; /* Set the channel y data right bit-shift.
                                        24-bit calibration offset */

    uint16_t DFSDM_ChDataPackMode; /* Specifies the data packing mode in DFSDM_CHyDATINR register.
                                    This parameter can be a value of @ref Data_Packing_Mode */

    uint16_t DFSDM_ChDataMultiplexer; /* Specifies channel y input data multiplexer data source.
                                    This parameter can be a value of @ref Input_Data_Multiplexer_Mode */

    uint16_t DFSDM_ChInPinSelect; /* Specifies whether the channel input is taken from
                                    the pin of the same channel y or from the pin of the next channel (y+1).
                                    This parameter can be a value of @ref Channel_Inputs_selection */

    uint16_t DFSDM_ChSPIClockSource;  /* Specifies the SPI clock source for channel y.
                                    This parameter can be a value of @ref SPI_Clock_Selection */

    uint16_t DFSDM_ChSerialInterface;  /* Specifies the serial interface type for channel y.
                                        This parameter can be a value of @ref Serial_Interface_Type */

    uint16_t DFSDM_ChAWDSincFilterOrder; /* Specifies the analog watchdog Sinc filter order on channel y.
                                        This parameter can be a value of @ref AWD_Sinc_Filter_Order */

    uint16_t DFSDM_ChSCDBreakSignal;  /* Specifies the Break signal assignment for short-circuit detector on
                                        channel y.
                                        This parameter can be a value of @ref SCD_Break_Signal */

    uint8_t DFSDM_ChDataRightBitShift; /* Set the shift bits(0-31) of the data result coming from the integrator -
                                        how many bit shifts to the right will be performed to have final results.
                                        Bit-shift is performed before offset correction. */

    uint8_t DFSDM_ChAWDFilterOverSample; /* Specifies the analog watchdog filter oversampling ratio (decimation rate)
                                    on channel y.It ranges from 1 to 32,If the value is 1, the filter is bypassed
                                    @ref AWD_Filter_Bypass */

    uint8_t DFSDM_ChSCDCntthreshold; /* Set zhe short-circuit detector threshold for channel y.
                                    It ranges from 0-255 */

    FunctionalState DFSDM_ChClockAbsenceDetMode; /* Specifies whether the channel y clock absence detector is enabled.
                                                This parameter can be set to ENABLE or DISABLE. */

    FunctionalState DFSDM_ChShortCircuitDetMode; /* Specifies whether the channel y Short-circuit detector is enabled.
                                                This parameter can be set to ENABLE or DISABLE. */
} DFSDM_ChannelInitTypeDef;

/* DFSDM filter Init structure definition */
typedef struct
{
    int32_t  DFSDM_FltAWDHighThreshold; /* Set the analog watchdog high threshold. It ranges from -8388608 to 8388607 */

    int32_t  DFSDM_FltAWDLowThreshold; /* Set the analog watchdog Ligh threshold. It ranges from -8388608 to 8388607 */
    
    uint16_t DFSDM_FltSincOrder; /* Specifies the Sinc filter order.
                                This parameter can be a value of @ref Sinc_Filter_Order */

    uint16_t DFSDM_FltOverSample; /* Specifies the filter oversampling ratio (decimation rate).It ranges from 1 to 1024
                                If the value is 1, the filter is bypassed @ref Filter_Bypass */

    uint16_t DFSDM_FltIntegratorOverSample; /* Specifies the filter integrator oversampling ratio (averaging length)
                                            2^DFSDM_FltIntegratorOverSample. If the value is 0, the integrator is bypassed
                                            @ref Filter_Integrator_Bypass*/

    uint16_t DFSDM_FltAWDChannel; /* Specifies which analog watchdog detector channel to select.
                                This parameter can be a value of @ref AWD_Channel_Selection */


    uint16_t DFSDM_FltExtremeChannel; /* Specifies which Extremes detector channel to select.
                                This parameter can be a value of @ref Extr_Channel_Selection */

    uint16_t DFSDM_FltAWDHighThrBreakSignal; /* Specifies the analog watchdog high threshold event Break signal.
                                        This parameter can be a value of @ref AWDH_Break_Signal */

    uint16_t DFSDM_FltAWDLowThrBreakSignal; /* Specifies the analog watchdog low threshold event Break signal.
                                        This parameter can be a value of @ref AWDL_Break_Signal */
    
    FunctionalState DFSDM_FltAWDFastMode; /* Specifies whether the analog watchdog fast mode is enabled.
                                        This parameter can be set to ENABLE or DISABLE. */
} DFSDM_FilterInitTypeDef;

/* DFSDM regular conversions Init structure definition */
typedef struct
{
    uint16_t DFSDM_RcChannel; /* Specifies which regular conversion channel to select.
                            This parameter can be a value of @ref Regular_Channel_Selection */

    FunctionalState DFSDM_RcFastMode; /* Specifies whether the regular conversion fast mode is enabled.
                                    This parameter can be set to ENABLE or DISABLE. */

    FunctionalState DFSDM_RcDMAMode; /* Specifies whether DMA channel to read regular conversion data is enabled.
                                    This parameter can be set to ENABLE or DISABLE. */

    FunctionalState DFSDM_RcSynchronousMode; /* Specifies whether Launch regular conversion synchronously with
                                            DFSDM_FLT0 is enabled.
                                            This parameter can be set to ENABLE or DISABLE. */
    FunctionalState DFSDM_RcContinuousMode; /* Specifies the regular conversion with continuous mode is enabled.
                                            This parameter can be set to ENABLE or DISABLE. */
} DFSDM_RcInitTypeDef;

/* DFSDM injected conversions Init structure definition */
typedef struct
{
    uint16_t DFSDM_JcChannelGroup;  /* Specifies which injected channel group to select.
                                    This parameter can be a value of @ref Inject_Channel_Group */

    uint16_t DFSDM_JcTriggerEdge; /* Specifies the trigger edge of the injected conversions.
                                This parameter can be a value of @ref Inject_Trigger_Edge */

    uint16_t DFSDM_JcTriggerSignal; /* Specifies the trigger signal for launching injected conversions.
                                    This parameter can be a value of @ref Inject_Trigger_Signal */

    FunctionalState DFSDM_JcDMAMode; /* Specifies whether DMA channel to read injected conversion data is enabled.
                                    This parameter can be set to ENABLE or DISABLE. */

    FunctionalState DFSDM_JcScanMode; /* Specifies whether scanning conversion mode for injected conversions is enabled.
                                    This parameter can be set to ENABLE or DISABLE. */

    FunctionalState DFSDM_JcSynchronousMode; /* Specifies whether Launch injected conversion synchronously with
                                            DFSDM_FLT0 is enabled.
                                            This parameter can be set to ENABLE or DISABLE. */
} DFSDM_JcInitTypeDef;

/* DFSDM_CHyCFGR1 */
#define DFSDM_SysClk                              ((uint16_t)0x0000)
#define DFSDM_AudioClk                            ((uint16_t)0x0001)

/* Data_Packing_Mode */
#define DFSDM_StandardMode                        ((uint16_t)0x0000)
#define DFSDM_InterleaveMode                      ((uint16_t)0x4000)
#define DFSDM_DualMode                            ((uint16_t)0x8000)

/* Input_Data_Multiplexer_Mode */
#define DFSDM_SerialInput                         ((uint16_t)0x0000)
#define DFSDM_ADCInput                            ((uint16_t)0x1000)
#define DFSDM_InternalInput                       ((uint16_t)0x2000)

/* Channel_Inputs_Selection */
#define DFSDM_SelectCurrent                       ((uint16_t)0x0000)
#define DFSDM_SelectNext                          ((uint16_t)0x0100)

/* SPI_Clock_Selection */
#define DFSDM_ExternalClkIn                       ((uint16_t)0x0000)
#define DFSDM_InternalClkOut                      ((uint16_t)0x0004)
#define DFSDM_InternalHalfFall                    ((uint16_t)0x0008)
#define DFSDM_InternalHalfRise                    ((uint16_t)0x000C)

/* Serial_Interface_Type */
#define DFSDM_SPIRising                           ((uint16_t)0x0000)
#define DFSDM_SPIFalling                          ((uint16_t)0x0001)

/* DFSDM_CHyAWSCDR */
/* AWD_Sinc_Filter_Order */
#define DFSDM_AWD_FastSinc                        ((uint16_t)0x0000)
#define DFSDM_AWD_Sinc1                           ((uint16_t)0x0001)
#define DFSDM_AWD_Sinc2                           ((uint16_t)0x0002)
#define DFSDM_AWD_Sinc3                           ((uint16_t)0x0003)

/* AWD_Filter_Bypass */
#define DFSDM_AWD_FLT_Bypass                      ((uint16_t)0x0001)

/* SCD_Break_Signal */
#define DFSDM_SCD_BK_None                         ((uint16_t)0x0000)
#define DFSDM_SCD_BK_0                            ((uint16_t)0x1000)
#define DFSDM_SCD_BK_1                            ((uint16_t)0x2000)
#define DFSDM_SCD_BK_0_1                          ((uint16_t)0x3000)

/* DFSDM_FLTxCR1 */
/* Regular_Channel_Selection */
#define DFSDM_RC_Channel0                         ((uint16_t)0x0000)
#define DFSDM_RC_Channel1                         ((uint16_t)0x0001)

/* Inject_Trigger_Edge */
#define DFSDM_JC_Trigger_Disable                  ((uint16_t)0x0000)
#define DFSDM_JC_Trigger_Rising                   ((uint16_t)0x2000)
#define DFSDM_JC_Trigger_Falling                  ((uint16_t)0x4000)
#define DFSDM_JC_Trigger_Edge                     ((uint16_t)0x6000)

/* Inject_Trigger_Signal */
#define DFSDM_JC_Trigger_TIM1                     ((uint16_t)0x0000)
#define DFSDM_JC_Trigger_TIM2                     ((uint16_t)0x0100)
#define DFSDM_JC_Trigger_TIM3                     ((uint16_t)0x0200)
#define DFSDM_JC_Trigger_TIM4                     ((uint16_t)0x0300)
#define DFSDM_JC_Trigger_TIM5                     ((uint16_t)0x0400)
#define DFSDM_JC_Trigger_TIM6                     ((uint16_t)0x0500)
#define DFSDM_JC_Trigger_TIM7                     ((uint16_t)0x0600)
#define DFSDM_JC_Trigger_TIM8                     ((uint16_t)0x0700)
#define DFSDM_JC_Trigger_TIM9                     ((uint16_t)0x0800)
#define DFSDM_JC_Trigger_TIM10                    ((uint16_t)0x0900)
#define DFSDM_JC_Trigger_TIM11                    ((uint16_t)0x0A00)
#define DFSDM_JC_Trigger_TIM12                    ((uint16_t)0x0B00)
#define DFSDM_JC_Trigger_EXTI11                   ((uint16_t)0x0C00)
#define DFSDM_JC_Trigger_EXTI15                   ((uint16_t)0x0D00)
#define DFSDM_JC_Trigger_LPTIM1                   ((uint16_t)0x0E00)
#define DFSDM_JC_Trigger_LPTIM2                   ((uint16_t)0x0F00)

/* DFSDM_FLTxCR2 */
/* AWD_Channel_Selection */
#define DFSDM_AWD_Channel_Disable                 ((uint16_t)0x0000)
#define DFSDM_AWD_Channel0                        ((uint16_t)0x0001)
#define DFSDM_AWD_Channel1                        ((uint16_t)0x0002)
#define DFSDM_AWD_Channel0_1                      ((uint16_t)0x0003)

/* Extr_Channel_Selection */
#define DFSDM_Extremes_Channel_Disable            ((uint16_t)0x0000)
#define DFSDM_Extremes_Channel0                   ((uint16_t)0x0100)
#define DFSDM_Extremes_Channel1                   ((uint16_t)0x0200)
#define DFSDM_Extremes_Channel0_1                 ((uint16_t)0x0300)

/* DFSDM_FLTxJCHGR */
/* Inject_Channel_Group */
#define DFSDM_JC_Channel_Disable                  ((uint32_t)0x0000)
#define DFSDM_JC_Channel0                         ((uint32_t)0x0001)
#define DFSDM_JC_Channel1                         ((uint32_t)0x0002)
#define DFSDM_JC_Channel0_1                       ((uint32_t)0x0003)

/* DFSDM_FLTxFCR3 */
/* Sinc_Filter_Order */
#define DFSDM_FLT_FastSinc                        ((uint16_t)0x0000)
#define DFSDM_FLT_Sinc1                           ((uint16_t)0x0001)
#define DFSDM_FLT_Sinc2                           ((uint16_t)0x0002)
#define DFSDM_FLT_Sinc3                           ((uint16_t)0x0003)
#define DFSDM_FLT_Sinc4                           ((uint16_t)0x0004)
#define DFSDM_FLT_Sinc5                           ((uint16_t)0x0005)

/* Filter_Bypass */
#define DFSDM_FLT_Bypass                          ((uint16_t)0x0001)

/* Filter_Integrator_Bypass */
#define DFSDM_FLT_IOSR_Bypass                     ((uint16_t)0x0000)

/* DFSDM_FLTxAWHTR */
/* AWDH_Break_Signal */
#define DFSDM_AWDH_BK_None                        ((uint16_t)0x0000)
#define DFSDM_AWDH_BK_0                           ((uint16_t)0x0001)
#define DFSDM_AWDH_BK_1                           ((uint16_t)0x0002)
#define DFSDM_AWDH_BK_0_1                         ((uint16_t)0x0003)

/* DFSDM_FLTxAWLTR */
/* AWDL_Break_Signal */
#define DFSDM_AWDL_BK_None                        ((uint16_t)0x0000)
#define DFSDM_AWDL_BK_0                           ((uint16_t)0x0001)
#define DFSDM_AWDL_BK_1                           ((uint16_t)0x0002)
#define DFSDM_AWDL_BK_0_1                         ((uint16_t)0x0003)

/* DFSDM_FLTxISR */
#define DFSDM_FLAG_FLTx_JEOCF                     ((uint8_t)0x20)
#define DFSDM_FLAG_FLTx_REOCF                     ((uint8_t)0x21)
#define DFSDM_FLAG_FLTx_JOVRF                     ((uint8_t)0x22)
#define DFSDM_FLAG_FLTx_ROVRF                     ((uint8_t)0x23)
#define DFSDM_FLAG_FLTx_AWDF                      ((uint8_t)0x24)
#define DFSDM_FLAG_FLTx_JCIP                      ((uint8_t)0x2D)
#define DFSDM_FLAG_FLTx_RCIP                      ((uint8_t)0x2E)
#define DFSDM_FLAG_FLT0_CKABF0                    ((uint8_t)0x70)
#define DFSDM_FLAG_FLT0_CKABF1                    ((uint8_t)0x71)
#define DFSDM_FLAG_FLT0_SCDF0                     ((uint8_t)0x78)
#define DFSDM_FLAG_FLT0_SCDF1                     ((uint8_t)0x79)

/* FLTxRDATAR */
#define DFSDM_FLAG_FLTx_RPEND                     ((uint8_t)0x44)

/* FLTxAWSR */
#define DFSDM_FLAG_FLTx_AWLTF0                    ((uint8_t)0x80)
#define DFSDM_FLAG_FLTx_AWLTF1                    ((uint8_t)0x81)
#define DFSDM_FLAG_FLTx_AWHTF0                    ((uint8_t)0x88)
#define DFSDM_FLAG_FLTx_AWHTF1                    ((uint8_t)0x89)


/* DFSDM interrupt definitions */
#define DFSDM_IT_FLTx_JEOCIE                      ((uint8_t)0x01)
#define DFSDM_IT_FLTx_REOCIE                      ((uint8_t)0x02)
#define DFSDM_IT_FLTx_JOVRIE                      ((uint8_t)0x04)
#define DFSDM_IT_FLTx_ROVRIE                      ((uint8_t)0x08)
#define DFSDM_IT_FLTx_AWDIE                       ((uint8_t)0x10)
#define DFSDM_IT_FLT0_SCDIE                       ((uint8_t)0xA0)
#define DFSDM_IT_FLT0_CKABIE                      ((uint8_t)0xC0)

/* DFSDM interrupt flag definitions */
#define DFSDM_IT_FLAG_FLTx_JEOCF                  ((uint16_t)0x0001)
#define DFSDM_IT_FLAG_FLTx_REOCF                  ((uint16_t)0x0102)
#define DFSDM_IT_FLAG_FLTx_JOVRF                  ((uint16_t)0x0204)
#define DFSDM_IT_FLAG_FLTx_ROVRF                  ((uint16_t)0x0308)
#define DFSDM_IT_FLAG_FLTx_AWDF                   ((uint16_t)0x0410)
#define DFSDM_IT_FLAG_FLT0_SCDF0                  ((uint16_t)0x9820)
#define DFSDM_IT_FLAG_FLT0_SCDF1                  ((uint16_t)0x9920)
#define DFSDM_IT_FLAG_FLT0_CKABF0                 ((uint16_t)0x9040)
#define DFSDM_IT_FLAG_FLT0_CKABF1                 ((uint16_t)0x9140)


void DFSDM_DeInit(void);
void DFSDM_ChannelStructInit(DFSDM_ChannelInitTypeDef *DFSDM_ChannelInitStruct);
void DFSDM_FilterStructInit(DFSDM_FilterInitTypeDef *DFSDM_FilterInitStruct);
void DFSDM_RcStructInit(DFSDM_RcInitTypeDef *DFSDM_RcInitStruct);
void DFSDM_JcStructInit(DFSDM_JcInitTypeDef *DFSDM_JcInitStruct);
void DFSDM_ChannelInit(DFSDM_Channel_TypeDef *DFSDM_Channely, DFSDM_ChannelInitTypeDef *DFSDM_ChannelInitStruct);
void DFSDM_FilterInit(DFSDM_FLT_TypeDef *DFSDM_FLTx, DFSDM_FilterInitTypeDef *DFSDM_FilterInitStruct);
void DFSDM_RcInit(DFSDM_FLT_TypeDef *DFSDM_FLTx, DFSDM_RcInitTypeDef *DFSDM_RcInitStruct);
void DFSDM_JcInit(DFSDM_FLT_TypeDef *DFSDM_FLTx, DFSDM_JcInitTypeDef *DFSDM_JcInitStruct);
void DFSDM_OutSerialClkConfig(uint16_t Source, uint8_t Div);
void DFSDM_Cmd(FunctionalState NewState);
void DFSDM_ChannelCmd(DFSDM_Channel_TypeDef *DFSDM_Channely, FunctionalState NewState);
void DFSDM_SPIClockSourceConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Source);
void DFSDM_SerialInterfaceConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Type);
void DFSDM_ShortCircuitDetCmd(DFSDM_Channel_TypeDef *DFSDM_Channely, FunctionalState NewState);
void DFSDM_ClockAbsenceDetCmd(DFSDM_Channel_TypeDef *DFSDM_Channely, FunctionalState NewState);
void DFSDM_ChannelInputSelect(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Pin_sel);
void DFSDM_DataPackModeConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Mode);
void DFSDM_ChannelInDataMpxConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Source);
void DFSDM_CalibrationOffsetConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, int32_t Offset);
void DFSDM_DataRightBitShiftConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint8_t Shift);
void DFSDM_SCDBreakSignalConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Signal);
void DFSDM_SCDCounterThrConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint8_t Threshold);
void DFSDM_WriteParallelDataStanMode(DFSDM_Channel_TypeDef *DFSDM_Channely, int16_t Data);
void DFSDM_WriteParallelDataIntlMode(DFSDM_Channel_TypeDef *DFSDM_Channely, int16_t Data0, int16_t Data1);
void DFSDM_WriteParallelDataDualMode(DFSDM_Channel_TypeDef *DFSDM_Channely, int16_t Data0, int16_t Data1);
void DFSDM_FilterCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState);
void DFSDM_FilterConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Order, uint16_t Fosr, uint16_t Iosr);
void DFSDM_AWDFilterConfig(DFSDM_Channel_TypeDef *DFSDM_Channely, uint16_t Order, uint8_t Awfosr);
int16_t DFSDM_ReadAWDFilterData(DFSDM_Channel_TypeDef *DFSDM_Channely);
void DFSDM_AWDFilterFastModeCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState);
void DFSDM_AWDChannelConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Channel);
void DFSDM_AWDHighThresholdConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, int32_t Value);
void DFSDM_AWDLowThresholdConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, int32_t Value);
void DFSDM_AWDHighThrBKConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Signal);
void DFSDM_AWDLowThrBKConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Signal);
void DFSDM_ExtrDetChannelConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Channel);
int32_t DFSDM_ReadExtrMaxData(DFSDM_FLT_TypeDef *DFSDM_FLTx);
uint8_t DFSDM_GetExtrMaxDataChannel(DFSDM_FLT_TypeDef *DFSDM_FLTx);
int32_t DFSDM_ReadExtrMinData(DFSDM_FLT_TypeDef *DFSDM_FLTx);
uint8_t DFSDM_GetExtrMinDataChannel(DFSDM_FLT_TypeDef *DFSDM_FLTx);
uint32_t DFSDM_ReadCntConvTimeData(DFSDM_FLT_TypeDef *DFSDM_FLTx);
void DFSDM_RcFastConvCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState);
void DFSDM_RcChannelConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Channel);
void DFSDM_RcDMACmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState);
void DFSDM_RcConvSynsCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState);
void DFSDM_RcContinuousCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState);
void DFSDM_RcSoftStartConversion(DFSDM_FLT_TypeDef *DFSDM_FLTx);
int32_t DFSDM_RcReadConvData(DFSDM_FLT_TypeDef *DFSDM_FLTx);
uint32_t DFSDM_RcGetLatestConvChannel(DFSDM_FLT_TypeDef *DFSDM_FLTx);
void DFSDM_JcTrigSignConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Trigger, uint16_t Trigger_edge);
void DFSDM_JcDMACmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState);
void DFSDM_JcScanConvCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState);
void DFSDM_JcConvSynsCmd(DFSDM_FLT_TypeDef *DFSDM_FLTx, FunctionalState NewState);
void DFSDM_JcSoftStartConversion(DFSDM_FLT_TypeDef *DFSDM_FLTx);
void DFSDM_JcChannelConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t Channel);
int32_t DFSDM_JcReadConvData(DFSDM_FLT_TypeDef *DFSDM_FLTx);
uint8_t DFSDM_JcGetLatestConvChannel(DFSDM_FLT_TypeDef *DFSDM_FLTx);
FlagStatus DFSDM_GetFlagStatus(DFSDM_FLT_TypeDef *DFSDM_FLTx,uint8_t DFSDM_FLAG);
void DFSDM_ClearFlag(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint8_t DFSDM_FLAG);
void DFSDM_ITConfig(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint8_t Interrupt, FunctionalState NewState);
ITStatus DFSDM_GetITStatus(DFSDM_FLT_TypeDef *DFSDM_FLTx, uint16_t DFSDM_FLAG);
void DFSDM_ClearITPendingBit(DFSDM_FLT_TypeDef *DFSDM_FLTx,uint16_t DFSDM_FLAG);

#ifdef __cplusplus
}
#endif

#endif 






