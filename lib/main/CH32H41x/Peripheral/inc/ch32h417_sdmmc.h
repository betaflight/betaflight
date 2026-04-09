/********************************** (C) COPYRIGHT  ******************************
* File Name          : ch32h417_sdmmc.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the
*                      SDMMC firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
********************************************************************************/

#ifndef __CH32H417_SDMMC_H
#define __CH32H417_SDMMC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"

/* SDMMC Init structure definition */
typedef struct
{
    uint16_t SDMMC_Mode;                /* Specifies the work mode of SDMMC controller.
                                            This parameter can be a value of @ref SDMMC_Mode */

    uint16_t SDMMC_PhaseInv;            /* Specifies the SDMMC output clock is inversed or not.
                                            This parameter can be a value of @ref SDMMC_PhaseInv */

    uint16_t SDMMC_ClockSpeed;          /* Specifies the clock mode of the SDMMC controller.
                                            This parameter can be a value  @ref SDMMC_ClockSpeed */

    uint8_t SDMMC_BusWidth;             /* Specifies the SDMMC bus width.
                                            This parameter can be a value of @ref SDMMC_BusWidth */

    uint8_t SDMMC_ClockEdge;            /* Specifies the clock transition on which the bit capture is made.
                                            This parameter can be a value of @ref SDMMC_ClockEdge */

    uint8_t SDMMC_ClockDiv;             /* Specifies the clock frequency of the SDMMC controller.
                                            This parameter can be a value between 0x00 and 0x1F. */

    uint8_t SDMMC_TimeOut;              /* Specifies the SDMMC time out value.
                                            This parameter can be a value between 0x00 and 0x0F. */

    FunctionalState SDMMC_SlaveForceCrc_ERR; /* Specifies whether the SDMMC CRC force error in slave mode. 
                                                This parameter can be set to ENABLE or DISABLE */

    FunctionalState SDMMC_DMA_EN;       /* Specifies whether the SDMMC DMA funcation enable. 
                                                This parameter can be set to ENABLE or DISABLE */                                                
    
    FunctionalState SDMMC_Clock_OE;       /* Specifies whether the SDMMC clock out enable . 
                                                This parameter can be set to ENABLE or DISABLE */
} SDMMC_InitTypeDef;

/* SDMMC CMD structure definition */
typedef struct
{
    uint32_t SDMMC_Argument;            /* Specifies the command argument which is sent to a card
                                          as part of a command message. Must be Loaded in this register
                                          before writing the R16_SDMMC_CMD_SET register.
                                          This parameter can be a 32-bit integer. */

    uint16_t SDMMC_RespExpect;          /* Specifies the response type.
                                          This parameter can be a value of @ref SDMMC_RespExpect */

    uint8_t SDMMC_CMDIdx;               /* Specifies the command index.
                                          This parameter can be a value between 0x00 and 0x3F. */

    FunctionalState SDMMC_CheckIdx;     /* Enable to check the command index regin of response.
                                          This parameter can be a value of ENABLE/DISABLE */

    FunctionalState SDMMC_CheckCRC;     /* Enable to check the command CRC regin of response.
                                          This parameter can be a value of ENABLE/DISABLE */

} SDMMC_CMDInitTypeDef;

/* SDMMC TranMode structure definition */
typedef struct
{
    uint8_t TranMode_DualDMA_TNCnt;     /* Config the block count to switch buffer when using dual buffer mode.
                                          This parameter can be a value between 0x00 and 0x7F. */

    uint32_t TranMode_Direction;        /* Config transmit direction.
                                          This parameter can be a value of @ref TranMode_Direction */

    uint32_t TranMode_DDR_ClockSW_Mode; /* Config SDMMC DDR mode clock swith.
                                          This parameter can be a value of @ref DDR_ClockSW_Mode */

    FunctionalState TranMode_DualDMA;   /* Enable SDMMC controller of dual buffer mode.
                                          This parameter can be a value of ENABLE/DISABLE */

    FunctionalState TranMode_AutoGapStop; /* Enable SDCK auto GAP stop.
                                          This parameter can be a value of ENABLE/DISABLE */

    FunctionalState TranMode_GapStop;    /* Enable SDCK GAP stop.
                                          This parameter can be a value of ENABLE/DISABLE */                                          

    FunctionalState TranMode_Boot;       /* Enable Boot mode for SDMMC storage type.
                                          This parameter can be a value of ENABLE/DISABLE */

    FunctionalState TranMode_DDR_EN;     /* Enable DDR mode for SDMMC storage type.
                                          This parameter can be a value of ENABLE/DISABLE */

    FunctionalState TranMode_DDR_ClockFall_Check; /* Enable DDR mode clock falling edge data check for SDMMC storage type.
                                                  This parameter can be a value of ENABLE/DISABLE */
                                        
} SDMMC_TranModeTypeDef;

/* SDMMC GPIO input delay structure definition */
typedef struct
{
    uint8_t SDMMC_CLK_IN_Delay ;     /* Config the SDMMC DDR mode clock line inptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */

    uint8_t SDMMC_CMD_IN_Delay ;     /* Config the SDMMC DDR mode CMD line inptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */

    uint8_t SDMMC_DATA0_IN_Delay ;     /* Config the SDMMC DDR mode data0 line inptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA1_IN_Delay ;     /* Config the SDMMC DDR mode data1 line inptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA2_IN_Delay ;     /* Config the SDMMC DDR mode data2 line inptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA3_IN_Delay ;     /* Config the SDMMC DDR mode data3 line inptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA4_IN_Delay ;     /* Config the SDMMC DDR mode data4 line inptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA5_IN_Delay ;     /* Config the SDMMC DDR mode data5 line inptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA6_IN_Delay ;     /* Config the SDMMC DDR mode data6 line inptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA7_IN_Delay ;     /* Config the SDMMC DDR mode data7 line inptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */                                                                                                                                                                                                                                                                                                                                               
} SDMMC_IOInputDelayDDRTypeDef;

/* SDMMC GPIO input delay structure definition */
typedef struct
{
    uint8_t SDMMC_CLK_OUT_Delay ;     /* Config the SDMMC DDR mode clock line outptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */

    uint8_t SDMMC_CMD_OUT_Delay ;     /* Config the SDMMC DDR mode CMD line outptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */

    uint8_t SDMMC_DATA0_OUT_Delay ;     /* Config the SDMMC DDR mode data0 line outptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA1_OUT_Delay ;     /* Config the SDMMC DDR mode data1 line outptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA2_OUT_Delay ;     /* Config the SDMMC DDR mode data2 line outptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA3_OUT_Delay ;     /* Config the SDMMC DDR mode data3 line outptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA4_OUT_Delay ;     /* Config the SDMMC DDR mode data4 line outptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA5_OUT_Delay ;     /* Config the SDMMC DDR mode data5 line outptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA6_OUT_Delay ;     /* Config the SDMMC DDR mode data6 line outptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */ 

    uint8_t SDMMC_DATA7_OUT_Delay ;     /* Config the SDMMC DDR mode data7 line outptu delay.
                                          This parameter can be a value between 0x0 and 0xF. */                                                                                                                                                                                                                                                                                                                                               
} SDMMC_IOOutputDelayDDRTypeDef;

/* SDMMC_Mode */
#define SDMMC_Mode_Host                             ((uint16_t)0x00000000)
#define SDMMC_Mode_Slave                            ((uint16_t)EMMC_SLV_MODE)

/* SDMMC_PhaseInv */
#define SDMMC_Phase_No_Inverse                      ((uint16_t)0x00000000)
#define SDMMC_Phase_Inverse                         ((uint16_t)EMMC_PHASEINV)

/* SDMMC_ClockSpeed */
#define SDMMC_ClockSpeed_Low                        ((uint16_t)0x00000000)
#define SDMMC_ClockSpeed_High                       ((uint16_t)(EMMC_CLKMode))

/* SDMMC_BusWidth */
#define SDMMC_BusWidth_1                            ((uint32_t)0x00000000)
#define SDMMC_BusWidth_4                            ((uint32_t)0x00000001)  
#define SDMMC_BusWidth_8                            ((uint32_t)0x00000002)  

/* SDMMC_ClockEdge */
#define SDMMC_SampleClock_Rising                    ((uint8_t)0x00000000)
#define SDMMC_SampleClock_Falling                   ((uint8_t)EMMC_NEGSMP)

/* SDMMC_RespExpect */
#define SDMMC_Resp_NONE                             ((uint16_t)0x0000)
#define SDMMC_Resp_136                              ((uint16_t)0x00100)
#define SDMMC_Resp_48                               ((uint16_t)0x00200)
#define SDMMC_Resp_R1b                              ((uint16_t)0x00300)

/* TranMode_Direction */
#define SDMMC_TranDir_Receive                       ((uint32_t)0x00000000)
#define SDMMC_TranDir_Send                          ((uint32_t)EMMC_DMA_DIR)

/* DDR_ClockSW_Mode */
#define SDMMC_DDR_ClockSW_Mode_In                   ((uint32_t)0x00000000)
#define SDMMC_DDR_ClockSW_Mode_Auto                 ((uint32_t)0x00080000)
#define SDMMC_DDR_ClockSW_Mode_Force                ((uint32_t)0x00100000)

/* SDMMC_Flags */
#define SDMMC_FLAG_SLV_BUF_RELEASE                  ((uint16_t)0x0400)
#define SDMMC_FLAG_SDIOINT                          ((uint16_t)0x0200)
#define SDMMC_FLAG_FIFO_OV                          ((uint16_t)0x0100)
#define SDMMC_FLAG_BKGAP                            ((uint16_t)0x0080)
#define SDMMC_FLAG_TRANDONE                         ((uint16_t)0x0040)
#define SDMMC_FLAG_TRANERR                          ((uint16_t)0x0020)
#define SDMMC_FLAG_DATTMO                           ((uint16_t)0x0010)
#define SDMMC_FLAG_CMDDONE                          ((uint16_t)0x0008)
#define SDMMC_FLAG_REIDX_ER                         ((uint16_t)0x0004)
#define SDMMC_FLAG_RECRC_WR                         ((uint16_t)0x0002)
#define SDMMC_FLAG_RE_TMOUT                         ((uint16_t)0x0001)

/* SDMMC_Interrupt_Sources */
#define SDMMC_IT_SDIOINT                            ((uint16_t)0x0200)
#define SDMMC_IT_FIFO_OV                            ((uint16_t)0x0100)
#define SDMMC_IT_BKGAP                              ((uint16_t)0x0080)
#define SDMMC_IT_TRANDONE                           ((uint16_t)0x0040)
#define SDMMC_IT_TRANERR                            ((uint16_t)0x0020)
#define SDMMC_IT_DATTMO                             ((uint16_t)0x0010)
#define SDMMC_IT_CMDDONE                            ((uint16_t)0x0008)
#define SDMMC_IT_REIDX_ER                           ((uint16_t)0x0004)
#define SDMMC_IT_RECRC_WR                           ((uint16_t)0x0002)
#define SDMMC_IT_RE_TMOUT                           ((uint16_t)0x0001)

/* SDMMC_Resps */
typedef enum
{
    Response0 = 0,
    Response1,
    Response2,
    Response3
} SDMMC_RespEnumTypeDef;

void SDMMC_DeInit(void);
void SDMMC_Init(SDMMC_InitTypeDef *SDMMC_InitStruct);
void SDMMC_InternalLogicReset(void);
void SDMMC_SetBusWidth(uint8_t BusWidth);
void SDMMC_SetClockSpeed(uint16_t ClockMode, uint16_t ClockDIV);
void SDMMC_ClockCmd(FunctionalState NewState);
void SDMMC_CommandConfig(SDMMC_CMDInitTypeDef *SDMMC_CMDInitStruct);
uint32_t SDMMC_GetResponse(SDMMC_RespEnumTypeDef SDMMC_Resp);
void SDMMC_DMAContinueWrite(void);
FlagStatus SDMMC_GetStatus_LineData0(void);
FlagStatus SDMMC_GetStatus_LineCMD(void);
uint16_t SDMMC_GetBlockNumSuccess(void);
FlagStatus SDMMC_GetFlagStatus(uint16_t SDMMC_Flag);
void SDMMC_ClearFlags(uint16_t SDMMC_Flag);
void SDMMC_ITConfig(uint16_t SDMMC_IT, FunctionalState NewState);
ITStatus SDMMC_GetITStatus(uint16_t SDMMC_IT);
void SDMMC_ClearITPendingBits(uint16_t SDMMC_IT);
void SDMMC_BlockConfig(uint32_t BlockSize, uint32_t BlockNum);
void SDMMC_TranMode_Init(SDMMC_TranModeTypeDef *SDMMC_TranModeStruct);
void SDMMC_SetAUTOGAPStop(FunctionalState NewState);
FlagStatus SDMMC_GetStatus_GAPStop(void);
void SDMMC_SetGAPStop(void);
void SDMMC_ClearGAPStop(void);
void SDMMC_SetDMAAddr1(uint32_t Address);
void SDMMC_SetDMAAddr2(uint32_t Address);
void SDMMC_IOInputDelayDDRInit(SDMMC_IOInputDelayDDRTypeDef *SSDMMC_IOInputDelayDDRStruct);
void SDMMC_IOOutputDelayDDRInit(SDMMC_IOOutputDelayDDRTypeDef *SSDMMC_IOOutputDelayDDRStruct);

#ifdef __cplusplus
}
#endif

#endif








