/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_i3c.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the
*                      RNG firmware library.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32H417_I3C_H
#define __CH32H417_I3C_H

#ifdef __cplusplus
 extern "C" {
#endif
#include "ch32h417.h"

/* I3C controller bus Init structure definition */
typedef struct
{
  uint32_t SDAHoldTime;         /* Specifies the I3C SDA hold time.
                                     This parameter must be a value of @ref SDAHoldTime */

  uint32_t WaitTime;            /* Specifies the time that the main and the new controllers should wait before
                                     issuing a start.
                                     This parameter must be a value of @ref WaitTime */

  uint8_t BusFreeDuration;      /* Specifies the I3C controller duration in number of kernel clock cycles, after
                                     a stop and before a start.
                                     This parameter must be a number  from 0 to 0xFF. */

  uint8_t BusIdleDuration;      /* Specifies the I3C controller duration in number of kernel clock cycles to be
                                     elapsed, after that both SDA and SCL are continuously high and stable
                                     before issuing a hot-join event.
                                     This parameter must be a number  from 0 to 0xFF. */

  uint8_t SCLPPLowDuration;     /* Specifies the I3C SCL low duration in number of kernel clock cycles
                                     in I3C push-pull phases.
                                     This parameter must be a number from 0 to 0xFF. */

  uint8_t SCLI3CHighDuration;   /* Specifies the I3C SCL high duration in number of kernel clock cycles,
                                     used for I3C messages for I3C open-drain and push pull phases.
                                     This parameter must be a number from 0 to 0xFF. */

  uint8_t SCLODLowDuration;     /* Specifies the I3C SCL low duration in number of kernel clock cycles in
                                     open-drain phases, used for legacy I2C commands and for I3C open-drain phases.
                                     This parameter must be a number  from 0 to 0xFF. */

  uint8_t SCLI2CHighDuration;   /* Specifies the I3C SCL high duration in number of kernel clock cycles, used
                                     for legacy I2C commands.
                                     This parameter must be a number  from 0 to 0xFF. */
} I3C_Ctrl_BusTypeDef;

/* I3C controller configration Init structure definition */
typedef struct
{
  uint8_t DynamicAddr;            /* Specifies the dynamic address of the controller when goes in target mode.
                                       This parameter must be a number  from 0 to 0x7F. */

  uint8_t StallTime;              /*  Specifies the controller clock stall time in number of kernel clock cycles.
                                       This parameter must be a number  from 0 to 0xFF. */

  FunctionalState HotJoinAllowed_EN; /*  Specifies the Enable/Disable state of the controller Hot Join acknowledgement
                                       when receiving a hot join request from target.
                                       This parameter can be set to ENABLE or DISABLE */

  FunctionalState ACKStallState;  /*  Specifies the Enable/Disable state of the controller clock stall
                                       on the ACK phase.
                                       This parameter can be set to ENABLE or DISABLE */

  FunctionalState CCCStallState;  /*  Specifies the Enable/Disable state of the controller clock stall on the
                                       T bit phase of a CCC communication to allow the target to decode command.
                                       This parameter can be set to ENABLE or DISABLE */

  FunctionalState TxStallState;   /*  Specifies the Enable/Disable state of the controller clock stall on
                                       parity phase of data to allow the target to read received data.
                                       This parameter can be set to ENABLE or DISABLE */

  FunctionalState RxStallState;   /*  Specifies the Enable/Disable state of the controller clock stall on the T bit
                                       phase of data enable to allow the target to prepare data to be sent.
                                       This parameter can be set to ENABLE or DISABLE */
} I3C_CtrlConfTypeDef;

/* I3C target configration Init structure definition */
typedef struct
{
  uint32_t MaxDataSpeed;               /*  Specifies the I3C target returned GETMXDS CCC format.
                                            This parameter must be a value of @ref MaxDataSpeed */

  uint32_t HandOffActivityState;       /*  Specifies the I3C target activity state when becoming controller.
                                            This parameter must be a value of @ref HandOffActivityState */

  uint32_t IBIPayloadSize;             /*  Specifies the I3C target payload data size.
                                            This parameter must be a value of @ref IBIPayloadSize  */

  uint32_t DataTurnAroundDuration;     /*  Specifies the I3C target clock-to-data turnaround time.
                                            This parameter must be a value of @ref DataTurnAroundDuration */
  
  uint16_t MaxReadDataSize;            /*  Specifies the numbers of data bytes that the target can read at maximum.
                                            This parameter must be a number from 0 to 0xFFFF. */

  uint16_t MaxWriteDataSize;           /*  Specifies the numbers of data bytes that the target can write at maximum.
                                            This parameter must be a number  from 0 to 0xFFFF. */

  uint8_t MaxReadTurnAround;           /*  Specifies the target maximum read turnaround byte.
                                            This parameter must be a number  from 0 to 0xFF. */

  uint8_t Identifier;                  /*  Specifies the target characteristic ID (MIPI named reference DCR).
                                            This parameter must be a number from 0 to 0xFF. */

  uint8_t MIPIIdentifier;              /*  Specifies the bits [12-15] of the 48-provisioned ID
                                            (MIPI named reference PID), other 48-provisioned ID are hardcoded.
                                            This parameter must be a number from 0 to 0xF. */

  FunctionalState CtrlRoleRequest;     /*  Specifies the Enable/Disable state of the target authorization request
                                            for a second master Chip.
                                            This parameter can be set to ENABLE or DISABLE */

  FunctionalState HotJoinRequest;      /*  Specifies the Enable/Disable state of the target hot join
                                            authorization request.
                                            This parameter can be set to ENABLE or DISABLE */

  FunctionalState IBIRequest;          /*  Specifies the Enable/Disable state of the target in Band Interrupt
                                            authorization request.
                                            This parameter can be set to ENABLE or DISABLE */

  FunctionalState IBIPayload_EN;       /*  Specifies the Enable/Disable state of sending data payload after
                                            an accepted IBI.
                                            This parameter can be set to ENABLE or DISABLE */


  FunctionalState CtrlCapability;      /*  Specifies the Enable/Disable state of the target controller capability.
                                            This parameter can be set to ENABLE or DISABLE */

  FunctionalState GroupAddrCapability; /*  Specifies the Enable/Disable state of the target support of group address
                                            after a controller role hand-off.
                                            This parameter can be set to ENABLE or DISABLE */

  FunctionalState MaxSpeedLimitation;  /*  Specifies the Enable/Disable state of the target max data speed limitation.
                                            This parameter can be set to ENABLE or DISABLE */

  FunctionalState HandOffDelay;        /*  Specifies the Enable/Disable state of the target need of delay to process
                                            the controller role hand-off.
                                            This parameter can be set to ENABLE or DISABLE */

  FunctionalState PendingReadMDB;      /*  Specifies the Enable/Disable state of the transmission of a mandatory
                                            data bytes indicating a pending read notification for GETCAPR CCC command.
                                            This parameter can be set to ENABLE or DISABLE */
} I3C_TgtConfTypeDef;

/* I3C device configration Init structure definition */
typedef struct
{
  uint8_t DeviceIndex;               /*  Specifies the index value of the device in the DEVRx register.
                                          This parameter must be a a value of @ref DeviceIndex */

  uint8_t TargetDynamicAddr;         /*  Specifies the dynamic address of the target x (1 to 4) connected on the bus.
                                          This parameter must be a number from 0 to 0x7F.  */

  FunctionalState IBIAck;            /*  Specifies the Enable/Disable state of the controller's ACK when receiving
                                          an IBI from a target x (1 to 4) connected on the bus.
                                          This parameter can be set to ENABLE or DISABLE */

  FunctionalState IBIPayload;        /*  Specifies the Enable/Disable state of the controller's receiving IBI payload
                                          after acknowledging an IBI requested from a target x (1 to 4) connected
                                          on the bus.
                                          This parameter can be set to ENABLE or DISABLE */

  FunctionalState CtrlRoleReqAck;    /*  Specifies the Enable/Disable state of the controller's ACK when receiving
                                          a control request from a target x (1 to 4) connected on the bus.
                                          This parameter can be set to ENABLE or DISABLE */

  FunctionalState CtrlStopTransfer;  /*  Specifies the Enable/Disable state of the controller's stop transfer after
                                          receiving an IBI request from a target x (1 to 4) connected on the bus.
                                          This parameter can be set to ENABLE or DISABLE */

} I3C_DeviceConfTypeDef;

/* I3C CCC information Init structure definition */
typedef struct
{
  uint32_t DynamicAddrValid;    /*  I3C target Dynamic Address Valid (updated during ENTDAA/RSTDAA/SETNEWDA CCC)
                                     This parameter can be Valid=1 or Not Valid=0                                    */
  uint32_t DynamicAddress;      /*  I3C target Dynamic Address (updated during ENTDAA/RSTDAA/SETNEWDA CCC)           */
  uint32_t MaxWriteLength;      /*  I3C target Maximum Write Length (updated during SETMWL CCC)                      */
  uint32_t MaxReadLength;       /*  I3C target Maximum Read Length (updated during SETMRL CCC)                       */
  uint32_t ResetAction;         /*  I3C target Reset Action level (updated during RSTACT CCC)                        */
  uint32_t ActivityState;       /*  I3C target Activity State (updated during ENTASx CCC)                            */
  uint32_t HotJoinAllowed;      /*  I3C target Hot Join (updated during ENEC/DISEC CCC)
                                     This parameter can be Allowed=1 or Not Allowed=0                                */
  uint32_t InBandAllowed;       /*  I3C target In Band Interrupt (updated during ENEC/DISEC CCC)
                                     This parameter can be Allowed=1 or Not Allowed=0                                */
  uint32_t CtrlRoleAllowed;     /*  I3C target Controller Role Request (updated during ENEC/DISEC CCC)
                                     This parameter can be Allowed=1 or Not Allowed=0                                */
  uint32_t IBICRTgtAddr;        /*  I3C controller receive Target Address during IBI or Controller Role Request event*/
  uint32_t IBITgtNbPayload;     /*  I3C controller get Number of Data Payload after an IBI event                     */
  uint32_t IBITgtPayload;       /*  I3C controller receive IBI Payload after an IBI event                            */
} I3C_CCCInfoTypeDef;

/* I3C_Notification_ID enumeration */
typedef enum
{
  EVENT_ID_GETACCCR = (0x00000001),    /* I3C target complete controller-role hand-off (direct GETACCR CCC) event         */
  EVENT_ID_IBIEND = (0x00000002),      /* I3C target IBI end process event                                                */
  EVENT_ID_DAU = (0x00000004),         /* I3C target receive a dynamic address update (ENTDAA/RSTDAA/SETNEWDA CCC) event  */
  EVENT_ID_GETx = (0x00000008),        /* I3C target receive any direct GETxxx CCC event                                  */
  EVENT_ID_GETSTATUS = (0x00000010),   /* I3C target receive get status command (direct GETSTATUS CCC) event              */
  EVENT_ID_SETMWL = (0x00000020),      /* I3C target receive maximum write length update (direct SETMWL CCC) event        */
  EVENT_ID_SETMRL = (0x00000040),      /* I3C target receive maximum read length update(direct SETMRL CCC) event          */
  EVENT_ID_RSTACT = (0x00000080),      /* I3C target detect reset pattern (broadcast or direct RSTACT CCC) event          */
  EVENT_ID_ENTASx = (0x00000100),      /* I3C target receive activity state update (direct or broadcast ENTASx) event     */
  EVENT_ID_ENEC_DISEC = (0x00000200),  /* I3C target receive a direct or broadcast ENEC/DISEC CCC event                   */
  EVENT_ID_DEFTGTS = (0x00000400),     /* I3C target receive a broadcast DEFTGTS CCC event                                */
  EVENT_ID_DEFGRPA = (0x00000800),     /* I3C target receive a group addressing (broadcast DEFGRPA CCC) event             */
  EVENT_ID_WKP = (0x00001000),         /* I3C target wakeup event                                                         */
  EVENT_ID_IBI = (0x00002000),         /* I3C controller receive IBI event                                                */
  EVENT_ID_CR = (0x00004000),          /* I3C controller controller-role request event                                    */
  EVENT_ID_HJ = (0x00008000),          /* I3C controller hot-join event                                                    */
}I3C_Notify_ID_TypeDef;

/* I3C_FIFO_Configration structure definition */
typedef struct
{
  uint32_t RxFifoThreshold; /*  Specifies the I3C Rx FIFO threshold level.
                                 This parameter must be a value of @ref I3C_RX_FIFO_THRESHOLD */

  uint32_t TxFifoThreshold; /*  Specifies the I3C Tx FIFO threshold level.
                                 This parameter must be a value of @ref I3C_TX_FIFO_THRESHOLD */

  uint32_t ControlFifo;     /*  Specifies the I3C control FIFO enable/disable state.
                                 This parameter is configured only with controller mode and it
                                 must be a value of @ref I3C_CONTROL_FIFO_STATE */

  uint32_t StatusFifo;      /*  Specifies the I3C status FIFO enable/disable state.
                                 This parameter is configured only with controller mode and it
                                 must be a value of @ref I3C_STATUS_FIFO_STATE */
} I3C_FifoConfTypeDef;


/* I3C_SDAHoldTime*/
#define I3C_SDAHoldTime_0_5                     ((uint32_t)0x00000000)
#define I3C_SDAHoldTime_1_5                     ((uint32_t)0x10000000)

/* I3C_WaitTime */
#define I3C_WaitTime_State_0                    ((uint32_t)0x00000000)
#define I3C_WaitTime_State_1                    ((uint32_t)0x00000100)
#define I3C_WaitTime_State_2                    ((uint32_t)0x00000200)
#define I3C_WaitTime_State_3                    ((uint32_t)0x00000300)

/* I3C_IBIPayloadSize */
#define I3C_IBIPayloadSize_None                 ((uint32_t)0x00000000)
#define I3C_IBIPayloadSize_1B                   ((uint32_t)0x00010000)
#define I3C_IBIPayloadSize_2B                   ((uint32_t)0x00020000)
#define I3C_IBIPayloadSize_3B                   ((uint32_t)0x00030000)
#define I3C_IBIPayloadSize_4B                   ((uint32_t)0x00040000)

/* I3C_DataTurnAroundDuration */
#define I3C_DataTurnAroundDuration_Mode0        ((uint32_t)0x00000000)
#define I3C_DataTurnAroundDuration_Mode1        ((uint32_t)0x01000000)

/* I3C_MaxDataSpeed */
#define I3C_MaxDataSpeed_Format_Mode0           ((uint32_t)0x00000000)
#define I3C_MaxDataSpeed_Format_Mode1           ((uint32_t)0x00000100)
#define I3C_MaxDataSpeed_Format_Mode2           ((uint32_t)0x00000200)
#define I3C_MaxDataSpeed_Format_Mode3           ((uint32_t)0x00000300)

/* I3C_HandOffActivityState */
#define I3C_HandOffActivityState_0              ((uint32_t)0x00000000)
#define I3C_HandOffActivityState_1              ((uint32_t)0x00000001)
#define I3C_HandOffActivityState_2              ((uint32_t)0x00000002)
#define I3C_HandOffActivityState_3              ((uint32_t)0x00000003)

/* I3C_DeviceIndex */
#define I3C_DeviceIndex_1                       ((uint32_t)0x00000001)
#define I3C_DeviceIndex_2                       ((uint32_t)0x00000002)
#define I3C_DeviceIndex_3                       ((uint32_t)0x00000003)
#define I3C_DeviceIndex_4                       ((uint32_t)0x00000004)

/* I3C_Direction */
#define I3C_Direction_RD                        ((uint32_t)0x00000000)
#define I3C_Direction_WR                        ((uint32_t)0x00010000)
/* I3C_EndMode */
#define I3C_EndMode_0                           ((uint32_t)0x00000000)
#define I3C_EndMode_1                           ((uint32_t)0x80000000)

/* I3C_CONTROLLER MessageType */
#define I3C_CONTROLLER_MessageType0             ((uint32_t)0x00000000)
#define I3C_CONTROLLER_MessageType1             ((uint32_t)0x08000000)
#define I3C_CONTROLLER_MessageType2             ((uint32_t)0x10000000)
#define I3C_CONTROLLER_MessageType3             ((uint32_t)0x18000000)
#define I3C_CONTROLLER_MessageType4             ((uint32_t)0x20000000)

/* I3C_TARGET_MessageType */
#define I3C_TARGET_MessageType0                 ((uint32_t)0x40000000)
#define I3C_TARGET_MessageType1                 ((uint32_t)0x48000000)
#define I3C_TARGET_MessageType2                 ((uint32_t)0x50000000)

/* I3C_PeripheralMode */
#define PeripheralMode_CONTROLLER               ((uint32_t)0x00000000)
#define PeripheralMode_TARGET                   ((uint32_t)0x00000002)

/* I3C_flags_definition */
#define I3C_FLAG_CFEF                           ((uint32_t)0x00000001)      
#define I3C_FLAG_TXFEF                          ((uint32_t)0x00000002)    
#define I3C_FLAG_CFNFF                          ((uint32_t)0x00000004)     
#define I3C_FLAG_SFNEF                          ((uint32_t)0x00000008)    
#define I3C_FLAG_TXFNFF                         ((uint32_t)0x00000010)    
#define I3C_FLAG_RXFNEF                         ((uint32_t)0x00000020)    
#define I3C_FLAG_TXLASTF                        ((uint32_t)0x00000040)  
#define I3C_FLAG_RXLASTF                        ((uint32_t)0x00000080)   
#define I3C_FLAG_FCF                            ((uint32_t)0x00000200)       
#define I3C_FLAG_RXTGTENDF                      ((uint32_t)0x00000400) 
#define I3C_FLAG_ERRF                           ((uint32_t)0x00000800)      
#define I3C_FLAG_IBIF                           ((uint32_t)0x00008000)      
#define I3C_FLAG_IBIENDF                        ((uint32_t)0x00010000)  
#define I3C_FLAG_CRF                            ((uint32_t)0x00020000)       
#define I3C_FLAG_CRUPDF                         ((uint32_t)0x00040000)   
#define I3C_FLAG_HJF                            ((uint32_t)0x00080000)       
#define I3C_FLAG_WKPF                           ((uint32_t)0x00200000)     
#define I3C_FLAG_GETF                           ((uint32_t)0x00400000)    
#define I3C_FLAG_STAF                           ((uint32_t)0x00800000)      
#define I3C_FLAG_DAUPDF                         ((uint32_t)0x01000000)    
#define I3C_FLAG_MWLUPDF                        ((uint32_t)0x02000000)   
#define I3C_FLAG_MRLUPDF                        ((uint32_t)0x04000000)  
#define I3C_FLAG_RSTF                           ((uint32_t)0x08000000)     
#define I3C_FLAG_ASUPDF                         ((uint32_t)0x10000000)    
#define I3C_FLAG_INTUPDF                        ((uint32_t)0x20000000)  
#define I3C_FLAG_DEFF                           ((uint32_t)0x40000000)      
#define I3C_FLAG_GRPF                           ((uint32_t)0x80000000)      

/* I3C_interrupts_definition */
#define I3C_IT_CFNFIE                           ((uint32_t)0x00000004)     
#define I3C_IT_SFNEIE                           ((uint32_t)0x00000008)     
#define I3C_IT_TXFNEIE                          ((uint32_t)0x00000010)
#define I3C_IT_RXFNEIE                          ((uint32_t)0x00000020)
#define I3C_IT_FCIE                             ((uint32_t)0x00000200)
#define I3C_IT_RXTGTENDIE                       ((uint32_t)0x00000400) 
#define I3C_IT_ERRIE                            ((uint32_t)0x00000800)
#define I3C_IT_IBIIE                            ((uint32_t)0x00008000)      
#define I3C_IT_IBIENDIE                         ((uint32_t)0x00010000) 
#define I3C_IT_CRIE                             ((uint32_t)0x00020000)       
#define I3C_IT_HJIE                             ((uint32_t)0x00080000)       
#define I3C_IT_CRUPDIE                          ((uint32_t)0x00040000)  
#define I3C_IT_WKPIE                            ((uint32_t)0x00200000)    
#define I3C_IT_GETIE                            ((uint32_t)0x00400000)   
#define I3C_IT_STAIE                            ((uint32_t)0x00800000)    
#define I3C_IT_DAUPDIE                          ((uint32_t)0x01000000)  
#define I3C_IT_MWLUPDIE                         ((uint32_t)0x02000000) 
#define I3C_IT_MRLUPDIE                         ((uint32_t)0x04000000) 
#define I3C_IT_RSTIE                            ((uint32_t)0x08000000)    
#define I3C_IT_ASUPDIE                          ((uint32_t)0x10000000)  
#define I3C_IT_INTUPDIE                         ((uint32_t)0x20000000) 
#define I3C_IT_DEFIE                            ((uint32_t)0x40000000)
#define I3C_IT_GRPIE                            ((uint32_t)0x80000000)

/* I3C_ERROR */
#define I3C_ERROR_CE0                           ((uint32_t)0x00000000)
#define I3C_ERROR_CE1                           ((uint32_t)0x00000001)          
#define I3C_ERROR_CE2                           ((uint32_t)0x00000002)  
#define I3C_ERROR_CE3                           ((uint32_t)0x00000003)  
#define I3C_ERROR_TE0                           ((uint32_t)0x00000008)   
#define I3C_ERROR_TE1                           ((uint32_t)0x00000009) 
#define I3C_ERROR_TE2                           ((uint32_t)0x0000000A)  
#define I3C_ERROR_TE3                           ((uint32_t)0x0000000B)  
#define I3C_ERROR_TE4                           ((uint32_t)0x0000000C)  
#define I3C_ERROR_TE5                           ((uint32_t)0x0000000D)  
#define I3C_ERROR_TE6                           ((uint32_t)0x0000000E)                                                                         
#define I3C_ERROR_PERR                          ((uint32_t)0x00000010) 
#define I3C_ERROR_STALL                         ((uint32_t)0x00000020)
#define I3C_ERROR_DOVR                          ((uint32_t)0x00000040)
#define I3C_ERROR_COVR                          ((uint32_t)0x00000080)
#define I3C_ERROR_ADDRESS_NACK                  ((uint32_t)0x00000100)
#define I3C_ERROR_DATA_NACK                     ((uint32_t)0x00000200)
#define I3C_ERROR_DATA_HAND_OFF                 ((uint32_t)0x00000400) 


void I3C_DeInit(void);
void I3C_Ctrl_Init(I3C_Ctrl_BusTypeDef *I3C_InitStruct);
void I3C_Tgt_Init(uint8_t BusAvailableDuration);
void I3C_Ctrl_Config(I3C_CtrlConfTypeDef *pConfig);
void I3C_Tgt_Config(I3C_TgtConfTypeDef *pConfig);
void I3C_Cmd(FunctionalState NewState);
void I3C_ArbitrationHeaderCmd(FunctionalState NewState);
void I3C_HJAckCmd(FunctionalState NewState);
void I3C_DMAReq_RXCmd(FunctionalState NewState);
void I3C_DMAReq_TXCmd(FunctionalState NewState);
void I3C_DMAReq_StatusCmd(FunctionalState NewState);
void I3C_DMAReq_ControlCmd(FunctionalState NewState);
void I3C_ExitPatternCmd(FunctionalState NewState);
void I3C_RequestTransfer(void);
void I3C_TxPreloadConfig(uint16_t TxDataCount);
void I3C_TARGET_ResetCmd(FunctionalState NewState);
void I3C_CONTROLLER_ResetCmd(FunctionalState NewState);
void I3C_SetModeConfig(uint32_t PeripheralMode);
void I3C_Ctrl_ConfigBusDevices(I3C_DeviceConfTypeDef *pDesc);
void I3C_Ctrl_SetConfigResetPattern(FunctionalState resetPattern);
FunctionalState I3C_Ctrl_GetConfigResetPattern(void);
void I3C_FlushAllFifo(void);
void I3C_FlushControlFifo(void);
void I3C_FlushStatusFifo(void);
void I3C_SetConfigFifo(I3C_FifoConfTypeDef *pConfig);
void I3C_ClearConfigFifo(void);
uint8_t I3C_ReadByte(void);
uint32_t I3C_ReadWord(void);
void I3C_WriteByte(uint8_t Byte);
void I3C_WriteWord(uint32_t Word);
void I3C_IBIDataConfig(uint32_t Data);
void  I3C_GetCCCInfo(I3C_Notify_ID_TypeDef notifyId, I3C_CCCInfoTypeDef *pCCCInfo);
void I3C_ControllerHandleMessage(uint32_t TargetAddr, uint32_t TransferSize, uint32_t Direction, uint32_t MessageType, uint32_t EndMode);
void I3C_ControllerHandleCCC(uint32_t CCCValue, uint32_t AddByteSize, uint32_t EndMode);
void I3C_TargetHandleMessage(uint32_t MessageType, uint32_t IBISize);
uint8_t I3C_GetMessageDirection(void);
uint8_t I3C_GetTargetAbortPrivateRead(void);
uint16_t I3C_GetGetXferDataCount(void);
uint8_t I3C_GetMessageIdentifier(void);
uint8_t I3C_GetGetIBITargetAddr(void);
uint8_t I3C_GetReceiveCommandCode(void);
uint8_t I3C_GetGetNbIBIAddData(void);
uint8_t I3C_GetGetResetAction(void);
FunctionalState I3C_GetAllowedPayloadUpdate(uint32_t DeviceIndex);
uint8_t I3C_GetMIPIInstanceID(void);
uint8_t I3C_GetIDTypeSelector(void);
uint16_t I3C_GetMIPIManufacturerID(void);
void I3C_ITConfig(uint32_t I3C_IT, FunctionalState NewState);
ITStatus I3C_GetITStatus(uint32_t I3C_IT);
void I3C_ClearITPendingBit(uint32_t I3C_IT);
FlagStatus I3C_GetFlagStatus(uint32_t I3C_FLAG);
FlagStatus I3C_GetErrorStatus(uint32_t I3C_Error);
void I3C_ClearFlag(uint32_t I3C_FLAG);


#ifdef __cplusplus
}
#endif

#endif
