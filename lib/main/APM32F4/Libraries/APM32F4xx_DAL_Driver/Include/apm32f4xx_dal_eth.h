/**
  *
  * @file    apm32f4xx_dal_eth.h
  * @brief   Header file of ETH DAL module.
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_ETH_H
#define APM32F4xx_DAL_ETH_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

#if defined(ETH)

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup ETH
  * @{
  */

/* Exported types ------------------------------------------------------------*/
#ifndef ETH_TX_DESC_CNT
#define ETH_TX_DESC_CNT         4U
#endif /* ETH_TX_DESC_CNT */

#ifndef ETH_RX_DESC_CNT
#define ETH_RX_DESC_CNT         4U
#endif /* ETH_RX_DESC_CNT */


/*********************** Descriptors struct def section ************************/
/** @defgroup ETH_Exported_Types ETH Exported Types
  * @{
  */

/**
  * @brief  ETH DMA Descriptor structure definition
  */
typedef struct
{
  __IO uint32_t DESC0;
  __IO uint32_t DESC1;
  __IO uint32_t DESC2;
  __IO uint32_t DESC3;
  __IO uint32_t DESC4;
  __IO uint32_t DESC5;
  __IO uint32_t DESC6;
  __IO uint32_t DESC7;
  uint32_t BackupAddr0; /* used to store rx buffer 1 address */
  uint32_t BackupAddr1; /* used to store rx buffer 2 address */
} ETH_DMADescTypeDef;
/**
  *
  */

/**
  * @brief  ETH Buffers List structure definition
  */
typedef struct __ETH_BufferTypeDef
{
  uint8_t *buffer;                /*<! buffer address */

  uint32_t len;                   /*<! buffer length */

  struct __ETH_BufferTypeDef *next; /*<! Pointer to the next buffer in the list */
} ETH_BufferTypeDef;
/**
  *
  */

/**
  * @brief  DMA Transmit Descriptors Wrapper structure definition
  */
typedef struct
{
  uint32_t  TxDesc[ETH_TX_DESC_CNT];        /*<! Tx DMA descriptors addresses */

  uint32_t  CurTxDesc;                      /*<! Current Tx descriptor index for packet transmission */

  uint32_t *PacketAddress[ETH_TX_DESC_CNT];  /*<! Ethernet packet addresses array */

  uint32_t *CurrentPacketAddress;           /*<! Current transmit NX_PACKET addresses */

  uint32_t BuffersInUse;                   /*<! Buffers in Use */

  uint32_t releaseIndex;                  /*<! Release index */
} ETH_TxDescListTypeDef;
/**
  *
  */

/**
  * @brief  Transmit Packet Configuration structure definition
  */
typedef struct
{
  uint32_t Attributes;              /*!< Tx packet HW features capabilities.
                                         This parameter can be a combination of @ref ETH_Tx_Packet_Attributes*/

  uint32_t Length;                  /*!< Total packet length   */

  ETH_BufferTypeDef *TxBuffer;      /*!< Tx buffers pointers */

  uint32_t SrcAddrCtrl;             /*!< Specifies the source address insertion control.
                                         This parameter can be a value of @ref ETH_Tx_Packet_Source_Addr_Control */

  uint32_t CRCPadCtrl;             /*!< Specifies the CRC and Pad insertion and replacement control.
                                        This parameter can be a value of @ref ETH_Tx_Packet_CRC_Pad_Control  */

  uint32_t ChecksumCtrl;           /*!< Specifies the checksum insertion control.
                                        This parameter can be a value of @ref ETH_Tx_Packet_Checksum_Control  */

  uint32_t MaxSegmentSize;         /*!< Sets TCP maximum segment size only when TCP segmentation is enabled.
                                        This parameter can be a value from 0x0 to 0x3FFF */

  uint32_t PayloadLen;             /*!< Sets Total payload length only when TCP segmentation is enabled.
                                        This parameter can be a value from 0x0 to 0x3FFFF */

  uint32_t TCPHeaderLen;           /*!< Sets TCP header length only when TCP segmentation is enabled.
                                        This parameter can be a value from 0x5 to 0xF */

  uint32_t VlanTag;                /*!< Sets VLAN Tag only when VLAN is enabled.
                                        This parameter can be a value from 0x0 to 0xFFFF*/

  uint32_t VlanCtrl;               /*!< Specifies VLAN Tag insertion control only when VLAN is enabled.
                                        This parameter can be a value of @ref ETH_Tx_Packet_VLAN_Control */

  uint32_t InnerVlanTag;           /*!< Sets Inner VLAN Tag only when Inner VLAN is enabled.
                                        This parameter can be a value from 0x0 to 0x3FFFF */

  uint32_t InnerVlanCtrl;          /*!< Specifies Inner VLAN Tag insertion control only when Inner VLAN is enabled.
                                        This parameter can be a value of @ref ETH_Tx_Packet_Inner_VLAN_Control   */

  void *pData;                     /*!< Specifies Application packet pointer to save   */

} ETH_TxPacketConfig;
/**
  *
  */

/**
  * @brief  ETH Timestamp structure definition
  */
typedef struct
{
  uint32_t TimeStampLow;
  uint32_t TimeStampHigh;

} ETH_TimeStampTypeDef;
/**
  *
  */

/**
  * @brief  ETH Timeupdate structure definition
  */
typedef struct
{
  uint32_t Seconds;
  uint32_t NanoSeconds;
} ETH_TimeTypeDef;
/**
  *
  */

/**
  * @brief  DMA Receive Descriptors Wrapper structure definition
  */
typedef struct
{
  uint32_t RxDesc[ETH_RX_DESC_CNT];     /*<! Rx DMA descriptors addresses. */

  uint32_t ItMode;                      /*<! If 1, DMA will generate the Rx complete interrupt.
                                             If 0, DMA will not generate the Rx complete interrupt. */

  uint32_t RxDescIdx;                 /*<! Current Rx descriptor. */

  uint32_t RxDescCnt;                 /*<! Number of descriptors . */

  uint32_t RxDataLength;              /*<! Received Data Length. */

  uint32_t RxBuildDescIdx;            /*<! Current Rx Descriptor for building descriptors. */

  uint32_t RxBuildDescCnt;            /*<! Number of Rx Descriptors awaiting building. */

  uint32_t pRxLastRxDesc;             /*<! Last received descriptor. */

  ETH_TimeStampTypeDef TimeStamp;     /*<! Time Stamp Low value for receive. */

  void *pRxStart;                     /*<! Pointer to the first buff. */

  void *pRxEnd;                       /*<! Pointer to the last buff. */

} ETH_RxDescListTypeDef;
/**
  *
  */

/**
  * @brief  ETH MAC Configuration Structure definition
  */
typedef struct
{
  uint32_t
  SourceAddrControl;           /*!< Selects the Source Address Insertion or Replacement Control.
                                                     This parameter can be a value of @ref ETH_Source_Addr_Control */

  FunctionalState
  ChecksumOffload;             /*!< Enables or Disable the checksum checking for received packet payloads TCP, UDP or ICMP headers */

  uint32_t         InterPacketGapVal;           /*!< Sets the minimum IPG between Packet during transmission.
                                                     This parameter can be a value of @ref ETH_Inter_Packet_Gap */

  FunctionalState  GiantPacketSizeLimitControl; /*!< Enables or disables the Giant Packet Size Limit Control. */

  FunctionalState  Support2KPacket;             /*!< Enables or disables the IEEE 802.3as Support for 2K length Packets */

  FunctionalState  CRCStripTypePacket;          /*!< Enables or disables the CRC stripping for Type packets.*/

  FunctionalState  AutomaticPadCRCStrip;        /*!< Enables or disables  the Automatic MAC Pad/CRC Stripping.*/

  FunctionalState  Watchdog;                    /*!< Enables or disables the Watchdog timer on Rx path.*/

  FunctionalState  Jabber;                      /*!< Enables or disables Jabber timer on Tx path.*/

  FunctionalState  JumboPacket;                 /*!< Enables or disables receiving Jumbo Packet
                                                           When enabled, the MAC allows jumbo packets of 9,018 bytes
                                                           without reporting a giant packet error */

  uint32_t         Speed;                       /*!< Sets the Ethernet speed: 10/100 Mbps.
                                                           This parameter can be a value of @ref ETH_Speed */

  uint32_t         DuplexMode;                  /*!< Selects the MAC duplex mode: Half-Duplex or Full-Duplex mode
                                                           This parameter can be a value of @ref ETH_Duplex_Mode */

  FunctionalState  LoopbackMode;                /*!< Enables or disables the loopback mode */

  FunctionalState
  CarrierSenseBeforeTransmit;  /*!< Enables or disables the Carrier Sense Before Transmission in Full Duplex Mode. */

  FunctionalState  ReceiveOwn;                  /*!< Enables or disables the Receive Own in Half Duplex mode. */

  FunctionalState
  CarrierSenseDuringTransmit;  /*!< Enables or disables the Carrier Sense During Transmission in the Half Duplex mode */

  FunctionalState
  RetryTransmission;           /*!< Enables or disables the MAC retry transmission, when a collision occurs in Half Duplex mode.*/

  uint32_t         BackOffLimit;                /*!< Selects the BackOff limit value.
                                                        This parameter can be a value of @ref ETH_Back_Off_Limit */

  FunctionalState
  DeferralCheck;               /*!< Enables or disables the deferral check function in Half Duplex mode. */

  uint32_t
  PreambleLength;              /*!< Selects or not the Preamble Length for Transmit packets (Full Duplex mode).
                                                           This parameter can be a value of @ref ETH_Preamble_Length */

  FunctionalState
  UnicastSlowProtocolPacketDetect;   /*!< Enable or disables the Detection of Slow Protocol Packets with unicast address. */

  FunctionalState  SlowProtocolDetect;          /*!< Enable or disables the Slow Protocol Detection. */

  FunctionalState  CRCCheckingRxPackets;        /*!< Enable or disables the CRC Checking for Received Packets. */

  uint32_t
  GiantPacketSizeLimit;        /*!< Specifies the packet size that the MAC will declare it as Giant, If it's size is
                                                    greater than the value programmed in this field in units of bytes
                                                    This parameter must be a number between
                                                    Min_Data = 0x618 (1518 byte) and Max_Data = 0x3FFF (32 Kbyte). */

  FunctionalState  ExtendedInterPacketGap;      /*!< Enable or disables the extended inter packet gap. */

  uint32_t         ExtendedInterPacketGapVal;   /*!< Sets the Extended IPG between Packet during transmission.
                                                           This parameter can be a value from 0x0 to 0xFF */

  FunctionalState  ProgrammableWatchdog;        /*!< Enable or disables the Programmable Watchdog.*/

  uint32_t         WatchdogTimeout;             /*!< This field is used as watchdog timeout for a received packet
                                                        This parameter can be a value of @ref ETH_Watchdog_Timeout */

  uint32_t
  PauseTime;                   /*!< This field holds the value to be used in the Pause Time field in the transmit control packet.
                                                   This parameter must be a number between
                                                   Min_Data = 0x0 and Max_Data = 0xFFFF.*/

  FunctionalState
  ZeroQuantaPause;             /*!< Enable or disables the automatic generation of Zero Quanta Pause Control packets.*/

  uint32_t
  PauseLowThreshold;           /*!< This field configures the threshold of the PAUSE to be checked for automatic retransmission of PAUSE Packet.
                                                   This parameter can be a value of @ref ETH_Pause_Low_Threshold */

  FunctionalState
  TransmitFlowControl;         /*!< Enables or disables the MAC to transmit Pause packets in Full Duplex mode
                                                   or the MAC back pressure operation in Half Duplex mode */

  FunctionalState
  UnicastPausePacketDetect;    /*!< Enables or disables the MAC to detect Pause packets with unicast address of the station */

  FunctionalState  ReceiveFlowControl;          /*!< Enables or disables the MAC to decodes the received Pause packet
                                                  and disables its transmitter for a specified (Pause) time */

  uint32_t         TransmitQueueMode;           /*!< Specifies the Transmit Queue operating mode.
                                                      This parameter can be a value of @ref ETH_Transmit_Mode */

  uint32_t         ReceiveQueueMode;            /*!< Specifies the Receive Queue operating mode.
                                                             This parameter can be a value of @ref ETH_Receive_Mode */

  FunctionalState  DropTCPIPChecksumErrorPacket; /*!< Enables or disables Dropping of TCPIP Checksum Error Packets. */

  FunctionalState  ForwardRxErrorPacket;        /*!< Enables or disables  forwarding Error Packets. */

  FunctionalState  ForwardRxUndersizedGoodPacket;  /*!< Enables or disables  forwarding Undersized Good Packets.*/
} ETH_MACConfigTypeDef;
/**
  *
  */

/**
  * @brief  ETH DMA Configuration Structure definition
  */
typedef struct
{
  uint32_t        DMAArbitration;          /*!< Sets the arbitration scheme between DMA Tx and Rx
                                                         This parameter can be a value of @ref ETH_DMA_Arbitration */

  FunctionalState AddressAlignedBeats;     /*!< Enables or disables the AHB Master interface address aligned
                                                            burst transfers on Read and Write channels  */

  uint32_t        BurstMode;               /*!< Sets the AHB Master interface burst transfers.
                                                     This parameter can be a value of @ref ETH_Burst_Mode */

  FunctionalState      DropTCPIPChecksumErrorFrame; /*!< Selects or not the Dropping of TCP/IP Checksum Error Frames */

  FunctionalState      ReceiveStoreForward;         /*!< Enables or disables the Receive store and forward mode */

  FunctionalState      TransmitStoreForward;        /*!< Enables or disables Transmit store and forward mode */


  uint32_t
  TxDMABurstLength;        /*!< Indicates the maximum number of beats to be transferred in one Tx DMA transaction.
                                                     This parameter can be a value of @ref ETH_Tx_DMA_Burst_Length */

  uint32_t             TransmitThresholdControl;    /*!< Selects or not the Transmit Threshold Control.
                                                             This parameter can be a value of
                                                             @ref ETH_Transmit_Threshold_Control */

  uint32_t
  RxDMABurstLength;        /*!< Indicates the maximum number of beats to be transferred in one Rx DMA transaction.
                                                    This parameter can be a value of @ref ETH_Rx_DMA_Burst_Length */

  FunctionalState      ForwardErrorFrames;          /*!< Selects or not the forward to the DMA of erroneous frames */
  FunctionalState FlushRxPacket;           /*!< Enables or disables the Rx Packet Flush */

  FunctionalState
  ForwardUndersizedGoodFrames; /*!< Enables or disables the Rx FIFO to forward Undersized frames (frames with no Error
                                                             and length less than 64 bytes)
                                                             including pad-bytes and CRC) */

  uint32_t             ReceiveThresholdControl;     /*!< Selects the threshold level of the Receive FIFO.
                                                             This parameter can be a value of
                                                             @ref ETH_Receive_Threshold_Control */

  FunctionalState
  SecondFrameOperate;          /*!< Selects or not the Operate on second frame mode, which allows the DMA to process a second
                                                             frame of Transmit data even before obtaining
                                                             the status for the first frame */

  FunctionalState      EnhancedDescriptorFormat;    /*!< Enables the enhanced descriptor format */

  uint32_t
  DescriptorSkipLength;        /*!< Specifies the number of word to skip between two unchained descriptors (Ring mode)
                                                             This parameter must be a number between
                                                             Min_Data = 0 and Max_Data = 32 */
} ETH_DMAConfigTypeDef;
/**
  *
  */

/**
  * @brief  DAL ETH Media Interfaces enum definition
  */
typedef enum
{
  DAL_ETH_MII_MODE             = 0x00U,   /*!<  Media Independent Interface               */
  DAL_ETH_RMII_MODE            = ((uint32_t)SYSCFG_PMCFG_ENETSEL)    /*!<   Reduced Media Independent Interface       */
} ETH_MediaInterfaceTypeDef;
/**
  *
  */

/**
  * @brief  DAL ETH PTP Update type enum definition
  */
typedef enum
{
  DAL_ETH_PTP_POSITIVE_UPDATE   = 0x00000000U,   /*!<  PTP positive time update       */
  DAL_ETH_PTP_NEGATIVE_UPDATE   = 0x00000001U   /*!<  PTP negative time update       */
} ETH_PtpUpdateTypeDef;
/**
  *
  */

/**
  * @brief  ETH Init Structure definition
  */
typedef struct
{

  uint8_t
  *MACAddr;                  /*!< MAC Address of used Hardware: must be pointer on an array of 6 bytes */

  ETH_MediaInterfaceTypeDef   MediaInterface;            /*!< Selects the MII interface or the RMII interface. */

  ETH_DMADescTypeDef
  *TxDesc;                   /*!< Provides the address of the first DMA Tx descriptor in the list */

  ETH_DMADescTypeDef
  *RxDesc;                   /*!< Provides the address of the first DMA Rx descriptor in the list */

  uint32_t                    RxBuffLen;                 /*!< Provides the length of Rx buffers size */

} ETH_InitTypeDef;
/**
  *
  */

/**
  * @brief  ETH PTP Init Structure definition
  */
typedef struct
{
  uint32_t                    Timestamp;                    /*!< Enable Timestamp */
  uint32_t                    TimestampUpdateMode;          /*!< Fine or Coarse Timestamp Update */
  uint32_t                    TimestampInitialize;          /*!< Initialize Timestamp */
  uint32_t                    TimestampUpdate;              /*!< Timestamp Update */
  uint32_t                    TimestampAddendUpdate;        /*!< Timestamp Addend Update */
  uint32_t                    TimestampAll;                 /*!< Enable Timestamp for All Packets */
  uint32_t                    TimestampRolloverMode;        /*!< Timestamp Digital or Binary Rollover Control */
  uint32_t                    TimestampV2;                  /*!< Enable PTP Packet Processing for Version 2 Format */
  uint32_t                    TimestampEthernet;            /*!< Enable Processing of PTP over Ethernet Packets */
  uint32_t                    TimestampIPv6;                /*!< Enable Processing of PTP Packets Sent over IPv6-UDP */
  uint32_t                    TimestampIPv4;                /*!< Enable Processing of PTP Packets Sent over IPv4-UDP */
  uint32_t                    TimestampEvent;               /*!< Enable Timestamp Snapshot for Event Messages */
  uint32_t                    TimestampMaster;              /*!< Enable Timestamp Snapshot for Event Messages */
  uint32_t                    TimestampFilter;              /*!< Enable MAC Address for PTP Packet Filtering */
  uint32_t                    TimestampClockType;           /*!< Time stamp clock node type */
  uint32_t                    TimestampAddend;              /*!< Timestamp addend value */
  uint32_t                    TimestampSubsecondInc;        /*!< Subsecond Increment */

} ETH_PTP_ConfigTypeDef;
/**
  *
  */

/**
  * @brief  DAL State structures definition
  */
typedef uint32_t DAL_ETH_StateTypeDef;
/**
  *
  */

/**
  * @brief  DAL ETH Rx Get Buffer Function definition
  */
typedef  void (*pETH_rxAllocateCallbackTypeDef)(uint8_t **buffer);  /*!< pointer to an ETH Rx Get Buffer Function */
/**
  *
  */

/**
  * @brief  DAL ETH Rx Set App Data Function definition
  */
typedef  void (*pETH_rxLinkCallbackTypeDef)(void **pStart, void **pEnd, uint8_t *buff,
                                            uint16_t Length); /*!< pointer to an ETH Rx Set App Data Function */
/**
  *
  */

/**
  * @brief  DAL ETH Tx Free Function definition
  */
typedef  void (*pETH_txFreeCallbackTypeDef)(uint32_t *buffer);  /*!< pointer to an ETH Tx Free function */
/**
  *
  */

/**
  * @brief  DAL ETH Tx Free Function definition
  */
typedef  void (*pETH_txPtpCallbackTypeDef)(uint32_t *buffer,
                                           ETH_TimeStampTypeDef *timestamp);  /*!< pointer to an ETH Tx Free function */
/**
  *
  */

/**
  * @brief  ETH Handle Structure definition
  */
#if (USE_DAL_ETH_REGISTER_CALLBACKS == 1)
typedef struct __ETH_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_ETH_REGISTER_CALLBACKS */
{
  ETH_TypeDef                *Instance;                 /*!< Register base address       */

  ETH_InitTypeDef            Init;                      /*!< Ethernet Init Configuration */

  ETH_TxDescListTypeDef      TxDescList;                /*!< Tx descriptor wrapper: holds all Tx descriptors list
                                                            addresses and current descriptor index  */

  ETH_RxDescListTypeDef      RxDescList;                /*!< Rx descriptor wrapper: holds all Rx descriptors list
                                                            addresses and current descriptor index  */

#ifdef DAL_ETH_USE_PTP
  ETH_TimeStampTypeDef       TxTimestamp;               /*!< Tx Timestamp */
#endif /* DAL_ETH_USE_PTP */

  __IO DAL_ETH_StateTypeDef  gState;                   /*!< ETH state information related to global Handle management
                                                              and also related to Tx operations. This parameter can
                                                              be a value of @ref DAL_ETH_StateTypeDef */

  __IO uint32_t              ErrorCode;                 /*!< Holds the global Error code of the ETH DAL status machine
                                                             This parameter can be a value of @ref ETH_Error_Code.*/

  __IO uint32_t
  DMAErrorCode;              /*!< Holds the DMA Rx Tx Error code when a DMA AIS interrupt occurs
                                                             This parameter can be a combination of
                                                             @ref ETH_DMA_Status_Flags */

  __IO uint32_t
  MACErrorCode;              /*!< Holds the MAC Rx Tx Error code when a MAC Rx or Tx status interrupt occurs
                                                             This parameter can be a combination of
                                                             @ref ETH_MAC_Rx_Tx_Status */

  __IO uint32_t              MACWakeUpEvent;            /*!< Holds the Wake Up event when the MAC exit the power down mode
                                                             This parameter can be a value of
                                                             @ref ETH_MAC_Wake_Up_Event */

  __IO uint32_t              MACLPIEvent;               /*!< Holds the LPI event when the an LPI status interrupt occurs.
                                                             This parameter can be a value of @ref ETHEx_LPI_Event */

  __IO uint32_t              IsPtpConfigured;           /*!< Holds the PTP configuration status.
                                                             This parameter can be a value of
                                                             @ref ETH_PTP_Config_Status */

#if (USE_DAL_ETH_REGISTER_CALLBACKS == 1)

  void (* TxCpltCallback)(struct __ETH_HandleTypeDef *heth);             /*!< ETH Tx Complete Callback */
  void (* RxCpltCallback)(struct __ETH_HandleTypeDef *heth);            /*!< ETH Rx  Complete Callback     */
  void (* ErrorCallback)(struct __ETH_HandleTypeDef *heth);             /*!< ETH Error Callback   */
  void (* PMTCallback)(struct __ETH_HandleTypeDef *heth);               /*!< ETH Power Management Callback            */
  void (* WakeUpCallback)(struct __ETH_HandleTypeDef *heth);            /*!< ETH Wake UP Callback   */

  void (* MspInitCallback)(struct __ETH_HandleTypeDef *heth);             /*!< ETH Msp Init callback              */
  void (* MspDeInitCallback)(struct __ETH_HandleTypeDef *heth);           /*!< ETH Msp DeInit callback            */

#endif  /* USE_DAL_ETH_REGISTER_CALLBACKS */

  pETH_rxAllocateCallbackTypeDef  rxAllocateCallback;  /*!< ETH Rx Get Buffer Function   */
  pETH_rxLinkCallbackTypeDef      rxLinkCallback; /*!< ETH Rx Set App Data Function */
  pETH_txFreeCallbackTypeDef      txFreeCallback;       /*!< ETH Tx Free Function         */
  pETH_txPtpCallbackTypeDef       txPtpCallback;  /*!< ETH Tx Handle Ptp Function */

} ETH_HandleTypeDef;
/**
  *
  */

#if (USE_DAL_ETH_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL ETH Callback ID enumeration definition
  */
typedef enum
{
  DAL_ETH_MSPINIT_CB_ID            = 0x00U,    /*!< ETH MspInit callback ID           */
  DAL_ETH_MSPDEINIT_CB_ID          = 0x01U,    /*!< ETH MspDeInit callback ID         */

  DAL_ETH_TX_COMPLETE_CB_ID        = 0x02U,    /*!< ETH Tx Complete Callback ID       */
  DAL_ETH_RX_COMPLETE_CB_ID        = 0x03U,    /*!< ETH Rx Complete Callback ID       */
  DAL_ETH_ERROR_CB_ID              = 0x04U,    /*!< ETH Error Callback ID             */
  DAL_ETH_PMT_CB_ID                = 0x06U,    /*!< ETH Power Management Callback ID  */
  DAL_ETH_WAKEUP_CB_ID             = 0x08U     /*!< ETH Wake UP Callback ID           */


} DAL_ETH_CallbackIDTypeDef;

/**
  * @brief  DAL ETH Callback pointer definition
  */
typedef  void (*pETH_CallbackTypeDef)(ETH_HandleTypeDef *heth);  /*!< pointer to an ETH callback function */

#endif /* USE_DAL_ETH_REGISTER_CALLBACKS */

/**
  * @brief  ETH MAC filter structure definition
  */
typedef struct
{
  FunctionalState PromiscuousMode;          /*!< Enable or Disable Promiscuous Mode */

  FunctionalState ReceiveAllMode;           /*!< Enable or Disable Receive All Mode */

  FunctionalState HachOrPerfectFilter;      /*!< Enable or Disable Perfect filtering in addition to Hash filtering */

  FunctionalState HashUnicast;              /*!< Enable or Disable Hash filtering on unicast packets */

  FunctionalState HashMulticast;            /*!< Enable or Disable Hash filtering on multicast packets */

  FunctionalState PassAllMulticast;         /*!< Enable or Disable passing all multicast packets */

  FunctionalState SrcAddrFiltering;         /*!< Enable or Disable source address filtering module */

  FunctionalState SrcAddrInverseFiltering;  /*!< Enable or Disable source address inverse filtering */

  FunctionalState DestAddrInverseFiltering; /*!< Enable or Disable destination address inverse filtering */

  FunctionalState BroadcastFilter;          /*!< Enable or Disable broadcast filter */

  uint32_t        ControlPacketsFilter;     /*!< Set the control packets filter
                                                 This parameter can be a value of @ref ETH_Control_Packets_Filter */
} ETH_MACFilterConfigTypeDef;
/**
  *
  */

/**
  * @brief  ETH Power Down structure definition
  */
typedef struct
{
  FunctionalState WakeUpPacket;    /*!< Enable or Disable Wake up packet detection in power down mode */

  FunctionalState MagicPacket;     /*!< Enable or Disable Magic packet detection in power down mode */

  FunctionalState GlobalUnicast;    /*!< Enable or Disable Global unicast packet detection in power down mode */

  FunctionalState WakeUpForward;    /*!< Enable or Disable Forwarding Wake up packets */

} ETH_PowerDownConfigTypeDef;
/**
  *
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup ETH_Exported_Constants ETH Exported Constants
  * @{
  */

/** @defgroup ETH_DMA_Tx_Descriptor_Bit_Definition ETH DMA Tx Descriptor Bit Definition
  * @{
  */

/*
   DMA Tx Normal Descriptor Read Format
  -----------------------------------------------------------------------------------------------
  TDES0 | OWN(31) | CTRL[30:26] | Reserved[25:24] | CTRL[23:20] | Reserved[19:17] | Status[16:0] |
  -----------------------------------------------------------------------------------------------
  TDES1 | Reserved[31:29] | Buffer2 ByteCount[28:16] | Reserved[15:13] | Buffer1 ByteCount[12:0] |
  -----------------------------------------------------------------------------------------------
  TDES2 |                         Buffer1 Address [31:0]                                         |
  -----------------------------------------------------------------------------------------------
  TDES3 |                   Buffer2 Address [31:0] / Next Descriptor Address [31:0]              |
  -----------------------------------------------------------------------------------------------
*/

/**
  * @brief  Bit definition of TDES0 register: DMA Tx descriptor status register
  */
#define ETH_DMATXDESC_OWN                     ((uint32_t)0x80000000U)  /*!< OWN bit: descriptor is owned by DMA engine */
#define ETH_DMATXDESC_IC                      ((uint32_t)0x40000000U)  /*!< Interrupt on Completion */
#define ETH_DMATXDESC_LS                      ((uint32_t)0x20000000U)  /*!< Last Segment */
#define ETH_DMATXDESC_FS                      ((uint32_t)0x10000000U)  /*!< First Segment */
#define ETH_DMATXDESC_DC                      ((uint32_t)0x08000000U)  /*!< Disable CRC */
#define ETH_DMATXDESC_DP                      ((uint32_t)0x04000000U)  /*!< Disable Padding */
#define ETH_DMATXDESC_TTSE                    ((uint32_t)0x02000000U)  /*!< Transmit Time Stamp Enable */
#define ETH_DMATXDESC_CIC                     ((uint32_t)0x00C00000U)  /*!< Checksum Insertion Control: 4 cases */
#define ETH_DMATXDESC_CIC_BYPASS              ((uint32_t)0x00000000U)  /*!< Do Nothing: Checksum Engine is bypassed */
#define ETH_DMATXDESC_CIC_IPV4HEADER          ((uint32_t)0x00400000U)  /*!< IPV4 header Checksum Insertion */
#define ETH_DMATXDESC_CIC_TCPUDPICMP_SEGMENT  ((uint32_t)0x00800000U)  /*!< TCP/UDP/ICMP Checksum Insertion calculated over segment only */
#define ETH_DMATXDESC_CIC_TCPUDPICMP_FULL     ((uint32_t)0x00C00000U)  /*!< TCP/UDP/ICMP Checksum Insertion fully calculated */
#define ETH_DMATXDESC_TER                     ((uint32_t)0x00200000U)  /*!< Transmit End of Ring */
#define ETH_DMATXDESC_TCH                     ((uint32_t)0x00100000U)  /*!< Second Address Chained */
#define ETH_DMATXDESC_TTSS                    ((uint32_t)0x00020000U)  /*!< Tx Time Stamp Status */
#define ETH_DMATXDESC_IHE                     ((uint32_t)0x00010000U)  /*!< IP Header Error */
#define ETH_DMATXDESC_ES                      ((uint32_t)0x00008000U)  /*!< Error summary: OR of the following bits: UE || ED || EC || LCO || NC || LCA || FF || JT */
#define ETH_DMATXDESC_JT                      ((uint32_t)0x00004000U)  /*!< Jabber Timeout */
#define ETH_DMATXDESC_FF                      ((uint32_t)0x00002000U)  /*!< Frame Flushed: DMA/MTL flushed the frame due to SW flush */
#define ETH_DMATXDESC_PCE                     ((uint32_t)0x00001000U)  /*!< Payload Checksum Error */
#define ETH_DMATXDESC_LCA                     ((uint32_t)0x00000800U)  /*!< Loss of Carrier: carrier lost during transmission */
#define ETH_DMATXDESC_NC                      ((uint32_t)0x00000400U)  /*!< No Carrier: no carrier signal from the transceiver */
#define ETH_DMATXDESC_LCO                     ((uint32_t)0x00000200U)  /*!< Late Collision: transmission aborted due to collision */
#define ETH_DMATXDESC_EC                      ((uint32_t)0x00000100U)  /*!< Excessive Collision: transmission aborted after 16 collisions */
#define ETH_DMATXDESC_VF                      ((uint32_t)0x00000080U)  /*!< VLAN Frame */
#define ETH_DMATXDESC_CC                      ((uint32_t)0x00000078U)  /*!< Collision Count */
#define ETH_DMATXDESC_ED                      ((uint32_t)0x00000004U)  /*!< Excessive Deferral */
#define ETH_DMATXDESC_UF                      ((uint32_t)0x00000002U)  /*!< Underflow Error: late data arrival from the memory */
#define ETH_DMATXDESC_DB                      ((uint32_t)0x00000001U)  /*!< Deferred Bit */

/**
  * @brief  Bit definition of TDES1 register
  */
#define ETH_DMATXDESC_TBS2  ((uint32_t)0x1FFF0000U)  /*!< Transmit Buffer2 Size */
#define ETH_DMATXDESC_TBS1  ((uint32_t)0x00001FFFU)  /*!< Transmit Buffer1 Size */

/**
  * @brief  Bit definition of TDES2 register
  */
#define ETH_DMATXDESC_B1AP  ((uint32_t)0xFFFFFFFFU)  /*!< Buffer1 Address Pointer */

/**
  * @brief  Bit definition of TDES3 register
  */
#define ETH_DMATXDESC_B2AP  ((uint32_t)0xFFFFFFFFU)  /*!< Buffer2 Address Pointer */

/*---------------------------------------------------------------------------------------------
TDES6 |                         Transmit Time Stamp Low [31:0]                                 |
-----------------------------------------------------------------------------------------------
TDES7 |                         Transmit Time Stamp High [31:0]                                |
----------------------------------------------------------------------------------------------*/

/* Bit definition of TDES6 register */
#define ETH_DMAPTPTXDESC_TTSL  ((uint32_t)0xFFFFFFFFU)  /* Transmit Time Stamp Low */

/* Bit definition of TDES7 register */
#define ETH_DMAPTPTXDESC_TTSH  ((uint32_t)0xFFFFFFFFU)  /* Transmit Time Stamp High */

/**
  * @}
  */


/** @defgroup ETH_DMA_Rx_Descriptor_Bit_Definition ETH DMA Rx Descriptor Bit Definition
  * @{
  */

/*
  DMA Rx Normal Descriptor read format
  --------------------------------------------------------------------------------------------------------------------
  RDES0 | OWN(31) |                                             Status [30:0]                                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES1 | CTRL(31) | Reserved[30:29] | Buffer2 ByteCount[28:16] | CTRL[15:14] | Reserved(13) | Buffer1 ByteCount[12:0] |
  ---------------------------------------------------------------------------------------------------------------------
  RDES2 |                                       Buffer1 Address [31:0]                                                 |
  ---------------------------------------------------------------------------------------------------------------------
  RDES3 |                          Buffer2 Address [31:0] / Next Descriptor Address [31:0]                             |
  ---------------------------------------------------------------------------------------------------------------------
*/

/**
  * @brief  Bit definition of RDES0 register: DMA Rx descriptor status register
  */
#define ETH_DMARXDESC_OWN         ((uint32_t)0x80000000U)  /*!< OWN bit: descriptor is owned by DMA engine  */
#define ETH_DMARXDESC_AFM         ((uint32_t)0x40000000U)  /*!< DA Filter Fail for the rx frame  */
#define ETH_DMARXDESC_FL          ((uint32_t)0x3FFF0000U)  /*!< Receive descriptor frame length  */
#define ETH_DMARXDESC_ES          ((uint32_t)0x00008000U)  /*!< Error summary: OR of the following bits: DE || OE || IPC || LC || RWT || RE || CE */
#define ETH_DMARXDESC_DE          ((uint32_t)0x00004000U)  /*!< Descriptor error: no more descriptors for receive frame  */
#define ETH_DMARXDESC_SAF         ((uint32_t)0x00002000U)  /*!< SA Filter Fail for the received frame */
#define ETH_DMARXDESC_LE          ((uint32_t)0x00001000U)  /*!< Frame size not matching with length field */
#define ETH_DMARXDESC_OE          ((uint32_t)0x00000800U)  /*!< Overflow Error: Frame was damaged due to buffer overflow */
#define ETH_DMARXDESC_VLAN        ((uint32_t)0x00000400U)  /*!< VLAN Tag: received frame is a VLAN frame */
#define ETH_DMARXDESC_FS          ((uint32_t)0x00000200U)  /*!< First descriptor of the frame  */
#define ETH_DMARXDESC_LS          ((uint32_t)0x00000100U)  /*!< Last descriptor of the frame  */
#define ETH_DMARXDESC_IPV4HCE     ((uint32_t)0x00000080U)  /*!< IPC Checksum Error: Rx Ipv4 header checksum error   */
#define ETH_DMARXDESC_LC          ((uint32_t)0x00000040U)  /*!< Late collision occurred during reception   */
#define ETH_DMARXDESC_FT          ((uint32_t)0x00000020U)  /*!< Frame type - Ethernet, otherwise 802.3    */
#define ETH_DMARXDESC_RWT         ((uint32_t)0x00000010U)  /*!< Receive Watchdog Timeout: watchdog timer expired during reception    */
#define ETH_DMARXDESC_RE          ((uint32_t)0x00000008U)  /*!< Receive error: error reported by MII interface  */
#define ETH_DMARXDESC_DBE         ((uint32_t)0x00000004U)  /*!< Dribble bit error: frame contains non int multiple of 8 bits  */
#define ETH_DMARXDESC_CE          ((uint32_t)0x00000002U)  /*!< CRC error */
#define ETH_DMARXDESC_MAMPCE      ((uint32_t)0x00000001U)  /*!< Rx MAC Address/Payload Checksum Error: Rx MAC address matched/ Rx Payload Checksum Error */

/**
  * @brief  Bit definition of RDES1 register
  */
#define ETH_DMARXDESC_DIC   ((uint32_t)0x80000000U)  /*!< Disable Interrupt on Completion */
#define ETH_DMARXDESC_RBS2  ((uint32_t)0x1FFF0000U)  /*!< Receive Buffer2 Size */
#define ETH_DMARXDESC_RER   ((uint32_t)0x00008000U)  /*!< Receive End of Ring */
#define ETH_DMARXDESC_RCH   ((uint32_t)0x00004000U)  /*!< Second Address Chained */
#define ETH_DMARXDESC_RBS1  ((uint32_t)0x00001FFFU)  /*!< Receive Buffer1 Size */

/**
  * @brief  Bit definition of RDES2 register
  */
#define ETH_DMARXDESC_B1AP  ((uint32_t)0xFFFFFFFFU)  /*!< Buffer1 Address Pointer */

/**
  * @brief  Bit definition of RDES3 register
  */
#define ETH_DMARXDESC_B2AP  ((uint32_t)0xFFFFFFFFU)  /*!< Buffer2 Address Pointer */

/*---------------------------------------------------------------------------------------------------------------------
  RDES4 |                   Reserved[31:15]              |             Extended Status [14:0]                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES5 |                                            Reserved[31:0]                                                    |
  ---------------------------------------------------------------------------------------------------------------------
  RDES6 |                                       Receive Time Stamp Low [31:0]                                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES7 |                                       Receive Time Stamp High [31:0]                                         |
  --------------------------------------------------------------------------------------------------------------------*/

/* Bit definition of RDES4 register */
#define ETH_DMAPTPRXDESC_PTPV                            ((uint32_t)0x00002000U)  /* PTP Version */
#define ETH_DMAPTPRXDESC_PTPFT                           ((uint32_t)0x00001000U)  /* PTP Frame Type */
#define ETH_DMAPTPRXDESC_PTPMT                           ((uint32_t)0x00000F00U)  /* PTP Message Type */
#define ETH_DMAPTPRXDESC_PTPMT_SYNC                      ((uint32_t)0x00000100U)  /* SYNC message
                                                                                     (all clock types) */
#define ETH_DMAPTPRXDESC_PTPMT_FOLLOWUP                  ((uint32_t)0x00000200U)  /* FollowUp message
                                                                                     (all clock types) */
#define ETH_DMAPTPRXDESC_PTPMT_DELAYREQ                  ((uint32_t)0x00000300U)  /* DelayReq message
                                                                                     (all clock types) */
#define ETH_DMAPTPRXDESC_PTPMT_DELAYRESP                 ((uint32_t)0x00000400U)  /* DelayResp message
                                                                                     (all clock types) */
#define ETH_DMAPTPRXDESC_PTPMT_PDELAYREQ_ANNOUNCE        ((uint32_t)0x00000500U)  /* PdelayReq message
                                                                                     (peer-to-peer transparent clock)
                                                                                      or Announce message (Ordinary
                                                                                      or Boundary clock) */
#define ETH_DMAPTPRXDESC_PTPMT_PDELAYRESP_MANAG          ((uint32_t)0x00000600U)  /* PdelayResp message
                                                                                     (peer-to-peer transparent clock)
                                                                                      or Management message (Ordinary
                                                                                      or Boundary clock)  */
#define ETH_DMAPTPRXDESC_PTPMT_PDELAYRESPFOLLOWUP_SIGNAL ((uint32_t)0x00000700U)  /* PdelayRespFollowUp message
                                                                                    (peer-to-peer transparent clock)
                                                                                     or Signaling message (Ordinary
                                                                                     or Boundary clock) */
#define ETH_DMAPTPRXDESC_IPV6PR                          ((uint32_t)0x00000080U)  /* IPv6 Packet Received */
#define ETH_DMAPTPRXDESC_IPV4PR                          ((uint32_t)0x00000040U)  /* IPv4 Packet Received */
#define ETH_DMAPTPRXDESC_IPCB                            ((uint32_t)0x00000020U)  /* IP Checksum Bypassed */
#define ETH_DMAPTPRXDESC_IPPE                            ((uint32_t)0x00000010U)  /* IP Payload Error */
#define ETH_DMAPTPRXDESC_IPHE                            ((uint32_t)0x00000008U)  /* IP Header Error */
#define ETH_DMAPTPRXDESC_IPPT                            ((uint32_t)0x00000007U)  /* IP Payload Type */
#define ETH_DMAPTPRXDESC_IPPT_UDP                        ((uint32_t)0x00000001U)  /* UDP payload encapsulated in
                                                                                     the IP datagram */
#define ETH_DMAPTPRXDESC_IPPT_TCP                        ((uint32_t)0x00000002U)  /* TCP payload encapsulated in
                                                                                     the IP datagram */
#define ETH_DMAPTPRXDESC_IPPT_ICMP                       ((uint32_t)0x00000003U)  /* ICMP payload encapsulated in
                                                                                     the IP datagram */

/* Bit definition of RDES6 register */
#define ETH_DMAPTPRXDESC_RTSL  ((uint32_t)0xFFFFFFFFU)  /* Receive Time Stamp Low */

/* Bit definition of RDES7 register */
#define ETH_DMAPTPRXDESC_RTSH  ((uint32_t)0xFFFFFFFFU)  /* Receive Time Stamp High */

/**
  * @}
  */

/** @defgroup ETH_Frame_settings ETH frame settings
  * @{
  */
#define ETH_MAX_PACKET_SIZE      ((uint32_t)1528U)    /*!< ETH_HEADER + 2*VLAN_TAG + MAX_ETH_PAYLOAD + ETH_CRC */
#define ETH_HEADER               ((uint32_t)14U)    /*!< 6 byte Dest addr, 6 byte Src addr, 2 byte length/type */
#define ETH_CRC                  ((uint32_t)4U)    /*!< Ethernet CRC */
#define ETH_VLAN_TAG             ((uint32_t)4U)    /*!< optional 802.1q VLAN Tag */
#define ETH_MIN_PAYLOAD          ((uint32_t)46U)    /*!< Minimum Ethernet payload size */
#define ETH_MAX_PAYLOAD          ((uint32_t)1500U)    /*!< Maximum Ethernet payload size */
#define ETH_JUMBO_FRAME_PAYLOAD  ((uint32_t)9000U)    /*!< Jumbo frame payload size */
/**
  * @}
  */

/** @defgroup ETH_Error_Code ETH Error Code
  * @{
  */
#define DAL_ETH_ERROR_NONE         ((uint32_t)0x00000000U)   /*!< No error            */
#define DAL_ETH_ERROR_PARAM        ((uint32_t)0x00000001U)   /*!< Busy error          */
#define DAL_ETH_ERROR_BUSY         ((uint32_t)0x00000002U)   /*!< Parameter error     */
#define DAL_ETH_ERROR_TIMEOUT      ((uint32_t)0x00000004U)   /*!< Timeout error       */
#define DAL_ETH_ERROR_DMA          ((uint32_t)0x00000008U)   /*!< DMA transfer error  */
#define DAL_ETH_ERROR_MAC          ((uint32_t)0x00000010U)   /*!< MAC transfer error  */
#if (USE_DAL_ETH_REGISTER_CALLBACKS == 1)
#define DAL_ETH_ERROR_INVALID_CALLBACK ((uint32_t)0x00000020U)    /*!< Invalid Callback error  */
#endif /* USE_DAL_ETH_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup ETH_Tx_Packet_Attributes ETH Tx Packet Attributes
  * @{
  */
#define ETH_TX_PACKETS_FEATURES_CSUM          ((uint32_t)0x00000001U)
#define ETH_TX_PACKETS_FEATURES_SAIC          ((uint32_t)0x00000002U)
#define ETH_TX_PACKETS_FEATURES_VLANTAG       ((uint32_t)0x00000004U)
#define ETH_TX_PACKETS_FEATURES_INNERVLANTAG  ((uint32_t)0x00000008U)
#define ETH_TX_PACKETS_FEATURES_TSO           ((uint32_t)0x00000010U)
#define ETH_TX_PACKETS_FEATURES_CRCPAD        ((uint32_t)0x00000020U)
/**
  * @}
  */

/** @defgroup ETH_Tx_Packet_Source_Addr_Control ETH Tx Packet Source Addr Control
  * @{
  */
#define ETH_SRC_ADDR_CONTROL_DISABLE          ETH_DMATXNDESCRF_SAIC_DISABLE
#define ETH_SRC_ADDR_INSERT                   ETH_DMATXNDESCRF_SAIC_INSERT
#define ETH_SRC_ADDR_REPLACE                  ETH_DMATXNDESCRF_SAIC_REPLACE
/**
  * @}
  */

/** @defgroup ETH_Tx_Packet_CRC_Pad_Control ETH Tx Packet CRC Pad Control
  * @{
  */
#define ETH_CRC_PAD_DISABLE      (uint32_t)(ETH_DMATXDESC_DP | ETH_DMATXDESC_DC)
#define ETH_CRC_PAD_INSERT       ((uint32_t)0x00000000U)
#define ETH_CRC_INSERT           ETH_DMATXDESC_DP
/**
  * @}
  */

/** @defgroup ETH_Tx_Packet_Checksum_Control ETH Tx Packet Checksum Control
  * @{
  */
#define ETH_CHECKSUM_DISABLE                         ETH_DMATXDESC_CIC_BYPASS
#define ETH_CHECKSUM_IPHDR_INSERT                    ETH_DMATXDESC_CIC_IPV4HEADER
#define ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT            ETH_DMATXDESC_CIC_TCPUDPICMP_SEGMENT
#define ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC  ETH_DMATXDESC_CIC_TCPUDPICMP_FULL
/**
  * @}
  */

/** @defgroup ETH_Tx_Packet_VLAN_Control ETH Tx Packet VLAN Control
  * @{
  */
#define ETH_VLAN_DISABLE  ETH_DMATXNDESCRF_VTIR_DISABLE
#define ETH_VLAN_REMOVE   ETH_DMATXNDESCRF_VTIR_REMOVE
#define ETH_VLAN_INSERT   ETH_DMATXNDESCRF_VTIR_INSERT
#define ETH_VLAN_REPLACE  ETH_DMATXNDESCRF_VTIR_REPLACE
/**
  * @}
  */

/** @defgroup ETH_Tx_Packet_Inner_VLAN_Control ETH Tx Packet Inner VLAN Control
  * @{
  */
#define ETH_INNER_VLAN_DISABLE  ETH_DMATXCDESC_IVTIR_DISABLE
#define ETH_INNER_VLAN_REMOVE   ETH_DMATXCDESC_IVTIR_REMOVE
#define ETH_INNER_VLAN_INSERT   ETH_DMATXCDESC_IVTIR_INSERT
#define ETH_INNER_VLAN_REPLACE  ETH_DMATXCDESC_IVTIR_REPLACE
/**
  * @}
  */


/** @defgroup ETH_Rx_MAC_Filter_Status ETH Rx MAC Filter Status
  * @{
  */
#define ETH_VLAN_FILTER_PASS        ETH_DMARXDESC_VLAN
#define ETH_DEST_ADDRESS_FAIL       ETH_DMARXDESC_AFM
#define ETH_SOURCE_ADDRESS_FAIL     ETH_DMARXDESC_SAF
/**
  * @}
  */

/** @defgroup ETH_Rx_Error_Code ETH Rx Error Code
  * @{
  */
#define ETH_DRIBBLE_BIT_ERROR   ETH_DMARXDESC_DBE
#define ETH_RECEIVE_ERROR       ETH_DMARXDESC_RE
#define ETH_RECEIVE_OVERFLOW    ETH_DMARXDESC_OE
#define ETH_WATCHDOG_TIMEOUT    ETH_DMARXDESC_RWT
#define ETH_GIANT_PACKET        ETH_DMARXDESC_IPV4HC
#define ETH_CRC_ERROR           ETH_DMARXDESC_CE
/**
  * @}
  */

/** @defgroup ETH_DMA_Arbitration ETH DMA Arbitration
  * @{
  */
#define ETH_DMAARBITRATION_RX        ETH_DMAMR_DA
#define ETH_DMAARBITRATION_RX1_TX1   ((uint32_t)0x00000000U)
#define ETH_DMAARBITRATION_RX2_TX1   ETH_DMAMR_PR_2_1
#define ETH_DMAARBITRATION_RX3_TX1   ETH_DMAMR_PR_3_1
#define ETH_DMAARBITRATION_RX4_TX1   ETH_DMAMR_PR_4_1
#define ETH_DMAARBITRATION_RX5_TX1   ETH_DMAMR_PR_5_1
#define ETH_DMAARBITRATION_RX6_TX1   ETH_DMAMR_PR_6_1
#define ETH_DMAARBITRATION_RX7_TX1   ETH_DMAMR_PR_7_1
#define ETH_DMAARBITRATION_RX8_TX1   ETH_DMAMR_PR_8_1
#define ETH_DMAARBITRATION_TX        (ETH_DMAMR_TXPR | ETH_DMAMR_DA)
#define ETH_DMAARBITRATION_TX1_RX1   ((uint32_t)0x00000000U)
#define ETH_DMAARBITRATION_TX2_RX1   (ETH_DMAMR_TXPR | ETH_DMAMR_PR_2_1)
#define ETH_DMAARBITRATION_TX3_RX1   (ETH_DMAMR_TXPR | ETH_DMAMR_PR_3_1)
#define ETH_DMAARBITRATION_TX4_RX1   (ETH_DMAMR_TXPR | ETH_DMAMR_PR_4_1)
#define ETH_DMAARBITRATION_TX5_RX1   (ETH_DMAMR_TXPR | ETH_DMAMR_PR_5_1)
#define ETH_DMAARBITRATION_TX6_RX1   (ETH_DMAMR_TXPR | ETH_DMAMR_PR_6_1)
#define ETH_DMAARBITRATION_TX7_RX1   (ETH_DMAMR_TXPR | ETH_DMAMR_PR_7_1)
#define ETH_DMAARBITRATION_TX8_RX1   (ETH_DMAMR_TXPR | ETH_DMAMR_PR_8_1)
/**
  * @}
  */

/** @defgroup ETH_Burst_Mode ETH Burst Mode
  * @{
  */
#define ETH_BURSTLENGTH_FIXED           ETH_DMABMOD_FB
#define ETH_BURSTLENGTH_MIXED           ETH_DMABMOD_MB
#define ETH_BURSTLENGTH_UNSPECIFIED     ((uint32_t)0x00000000U)
/**
  * @}
  */

/** @defgroup ETH_Tx_DMA_Burst_Length ETH Tx DMA Burst Length
  * @{
  */
#define ETH_TXDMABURSTLENGTH_1BEAT          0x00000100U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 1 */
#define ETH_TXDMABURSTLENGTH_2BEAT          0x00000200U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 2 */
#define ETH_TXDMABURSTLENGTH_4BEAT          0x00000400U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
#define ETH_TXDMABURSTLENGTH_8BEAT          0x00000800U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
#define ETH_TXDMABURSTLENGTH_16BEAT         0x00001000U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
#define ETH_TXDMABURSTLENGTH_32BEAT         0x00002000U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */
#define ETH_TXDMABURSTLENGTH_4XPBL_4BEAT    0x01000100U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
#define ETH_TXDMABURSTLENGTH_4XPBL_8BEAT    0x01000200U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
#define ETH_TXDMABURSTLENGTH_4XPBL_16BEAT   0x01000400U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
#define ETH_TXDMABURSTLENGTH_4XPBL_32BEAT   0x01000800U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */
#define ETH_TXDMABURSTLENGTH_4XPBL_64BEAT   0x01001000U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 64 */
#define ETH_TXDMABURSTLENGTH_4XPBL_128BEAT  0x01002000U  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 128 */
/**
  * @}
  */

/** @defgroup ETH_Rx_DMA_Burst_Length ETH Rx DMA Burst Length
  * @{
  */
#define ETH_RXDMABURSTLENGTH_1BEAT          0x00020000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 1 */
#define ETH_RXDMABURSTLENGTH_2BEAT          0x00040000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 2 */
#define ETH_RXDMABURSTLENGTH_4BEAT          0x00080000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 4 */
#define ETH_RXDMABURSTLENGTH_8BEAT          0x00100000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 8 */
#define ETH_RXDMABURSTLENGTH_16BEAT         0x00200000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 16 */
#define ETH_RXDMABURSTLENGTH_32BEAT         0x00400000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 32 */
#define ETH_RXDMABURSTLENGTH_4XPBL_4BEAT    0x01020000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 4 */
#define ETH_RXDMABURSTLENGTH_4XPBL_8BEAT    0x01040000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 8 */
#define ETH_RXDMABURSTLENGTH_4XPBL_16BEAT   0x01080000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 16 */
#define ETH_RXDMABURSTLENGTH_4XPBL_32BEAT   0x01100000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 32 */
#define ETH_RXDMABURSTLENGTH_4XPBL_64BEAT   0x01200000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 64 */
#define ETH_RXDMABURSTLENGTH_4XPBL_128BEAT  0x01400000U  /*!< maximum number of beats to be transferred in one RxDMA transaction is 128 */
/**
  * @}
  */

/** @defgroup ETH_DMA_Interrupts ETH DMA Interrupts
  * @{
  */
#define ETH_DMA_NORMAL_IT                 ETH_DMACIER_NIE
#define ETH_DMA_ABNORMAL_IT               ETH_DMACIER_AIE
#define ETH_DMA_CONTEXT_DESC_ERROR_IT     ETH_DMACIER_CDEE
#define ETH_DMA_FATAL_BUS_ERROR_IT        ETH_DMACIER_FBEE
#define ETH_DMA_EARLY_RX_IT               ETH_DMACIER_ERIE
#define ETH_DMA_EARLY_TX_IT               ETH_DMACIER_ETIE
#define ETH_DMA_RX_WATCHDOG_TIMEOUT_IT    ETH_DMACIER_RWTE
#define ETH_DMA_RX_PROCESS_STOPPED_IT     ETH_DMACIER_RSE
#define ETH_DMA_RX_BUFFER_UNAVAILABLE_IT  ETH_DMACIER_RBUE
#define ETH_DMA_RX_IT                     ETH_DMACIER_RIE
#define ETH_DMA_TX_BUFFER_UNAVAILABLE_IT  ETH_DMACIER_TBUE
#define ETH_DMA_TX_PROCESS_STOPPED_IT     ETH_DMACIER_TXSE
#define ETH_DMA_TX_IT                     ETH_DMACIER_TIE
/**
  * @}
  */

/** @defgroup ETH_DMA_Status_Flags ETH DMA Status Flags
  * @{
  */
#define ETH_DMA_RX_NO_ERROR_FLAG                 ((uint32_t)0x00000000U)
#define ETH_DMA_RX_DESC_READ_ERROR_FLAG          (ETH_DMACSR_REB_BIT_2 | ETH_DMACSR_REB_BIT_1 | ETH_DMACSR_REB_BIT_0)
#define ETH_DMA_RX_DESC_WRITE_ERROR_FLAG         (ETH_DMACSR_REB_BIT_2 | ETH_DMACSR_REB_BIT_1)
#define ETH_DMA_RX_BUFFER_READ_ERROR_FLAG        (ETH_DMACSR_REB_BIT_2 | ETH_DMACSR_REB_BIT_0)
#define ETH_DMA_RX_BUFFER_WRITE_ERROR_FLAG        ETH_DMACSR_REB_BIT_2
#define ETH_DMA_TX_NO_ERROR_FLAG                 ((uint32_t)0x00000000U)
#define ETH_DMA_TX_DESC_READ_ERROR_FLAG          (ETH_DMACSR_TEB_BIT_2 | ETH_DMACSR_TEB_BIT_1 | ETH_DMACSR_TEB_BIT_0)
#define ETH_DMA_TX_DESC_WRITE_ERROR_FLAG         (ETH_DMACSR_TEB_BIT_2 | ETH_DMACSR_TEB_BIT_1)
#define ETH_DMA_TX_BUFFER_READ_ERROR_FLAG        (ETH_DMACSR_TEB_BIT_2 | ETH_DMACSR_TEB_BIT_0)
#define ETH_DMA_TX_BUFFER_WRITE_ERROR_FLAG        ETH_DMACSR_TEB_BIT_2
#define ETH_DMA_CONTEXT_DESC_ERROR_FLAG           ETH_DMACSR_CDE
#define ETH_DMA_FATAL_BUS_ERROR_FLAG              ETH_DMACSR_FBE
#define ETH_DMA_EARLY_TX_IT_FLAG                  ETH_DMACSR_ERI
#define ETH_DMA_RX_WATCHDOG_TIMEOUT_FLAG          ETH_DMACSR_RWT
#define ETH_DMA_RX_PROCESS_STOPPED_FLAG           ETH_DMACSR_RPS
#define ETH_DMA_RX_BUFFER_UNAVAILABLE_FLAG        ETH_DMACSR_RBU
#define ETH_DMA_TX_PROCESS_STOPPED_FLAG           ETH_DMACSR_TPS
/**
  * @}
  */

/** @defgroup ETH_Transmit_Mode ETH Transmit Mode
  * @{
  */
#define ETH_TRANSMITSTOREFORWARD       ETH_MTLTQOMR_TSF
#define ETH_TRANSMITTHRESHOLD_32       ETH_MTLTQOMR_TTC_32BITS
#define ETH_TRANSMITTHRESHOLD_64       ETH_MTLTQOMR_TTC_64BITS
#define ETH_TRANSMITTHRESHOLD_96       ETH_MTLTQOMR_TTC_96BITS
#define ETH_TRANSMITTHRESHOLD_128      ETH_MTLTQOMR_TTC_128BITS
#define ETH_TRANSMITTHRESHOLD_192      ETH_MTLTQOMR_TTC_192BITS
#define ETH_TRANSMITTHRESHOLD_256      ETH_MTLTQOMR_TTC_256BITS
#define ETH_TRANSMITTHRESHOLD_384      ETH_MTLTQOMR_TTC_384BITS
#define ETH_TRANSMITTHRESHOLD_512      ETH_MTLTQOMR_TTC_512BITS
/**
  * @}
  */

/** @defgroup ETH_Receive_Mode ETH Receive Mode
  * @{
  */
#define ETH_RECEIVESTOREFORWARD        ETH_MTLRQOMR_RSF
#define ETH_RECEIVETHRESHOLD8_64       ETH_MTLRQOMR_RTC_64BITS
#define ETH_RECEIVETHRESHOLD8_32       ETH_MTLRQOMR_RTC_32BITS
#define ETH_RECEIVETHRESHOLD8_96       ETH_MTLRQOMR_RTC_96BITS
#define ETH_RECEIVETHRESHOLD8_128      ETH_MTLRQOMR_RTC_128BITS
/**
  * @}
  */

/** @defgroup ETH_Pause_Low_Threshold  ETH Pause Low Threshold
  * @{
  */
#define ETH_PAUSELOWTHRESHOLD_MINUS_4        ETH_MACTFCR_PLT_MINUS4
#define ETH_PAUSELOWTHRESHOLD_MINUS_28       ETH_MACTFCR_PLT_MINUS28
#define ETH_PAUSELOWTHRESHOLD_MINUS_36       ETH_MACTFCR_PLT_MINUS36
#define ETH_PAUSELOWTHRESHOLD_MINUS_144      ETH_MACTFCR_PLT_MINUS144
#define ETH_PAUSELOWTHRESHOLD_MINUS_256      ETH_MACTFCR_PLT_MINUS256
#define ETH_PAUSELOWTHRESHOLD_MINUS_512      ETH_MACTFCR_PLT_MINUS512
/**
  * @}
  */

/** @defgroup ETH_Watchdog_Timeout ETH Watchdog Timeout
  * @{
  */
#define ETH_WATCHDOGTIMEOUT_2KB      ETH_MACWTR_WTO_2KB
#define ETH_WATCHDOGTIMEOUT_3KB      ETH_MACWTR_WTO_3KB
#define ETH_WATCHDOGTIMEOUT_4KB      ETH_MACWTR_WTO_4KB
#define ETH_WATCHDOGTIMEOUT_5KB      ETH_MACWTR_WTO_5KB
#define ETH_WATCHDOGTIMEOUT_6KB      ETH_MACWTR_WTO_6KB
#define ETH_WATCHDOGTIMEOUT_7KB      ETH_MACWTR_WTO_7KB
#define ETH_WATCHDOGTIMEOUT_8KB      ETH_MACWTR_WTO_8KB
#define ETH_WATCHDOGTIMEOUT_9KB      ETH_MACWTR_WTO_9KB
#define ETH_WATCHDOGTIMEOUT_10KB     ETH_MACWTR_WTO_10KB
#define ETH_WATCHDOGTIMEOUT_11KB     ETH_MACWTR_WTO_12KB
#define ETH_WATCHDOGTIMEOUT_12KB     ETH_MACWTR_WTO_12KB
#define ETH_WATCHDOGTIMEOUT_13KB     ETH_MACWTR_WTO_13KB
#define ETH_WATCHDOGTIMEOUT_14KB     ETH_MACWTR_WTO_14KB
#define ETH_WATCHDOGTIMEOUT_15KB     ETH_MACWTR_WTO_15KB
#define ETH_WATCHDOGTIMEOUT_16KB     ETH_MACWTR_WTO_16KB
/**
  * @}
  */

/** @defgroup ETH_Inter_Packet_Gap ETH Inter Packet Gap
  * @{
  */
#define ETH_INTERPACKETGAP_96BIT   ETH_MACCFG_IPG_96BIT
#define ETH_INTERPACKETGAP_88BIT   ETH_MACCFG_IPG_88BIT
#define ETH_INTERPACKETGAP_80BIT   ETH_MACCFG_IPG_80BIT
#define ETH_INTERPACKETGAP_72BIT   ETH_MACCFG_IPG_72BIT
#define ETH_INTERPACKETGAP_64BIT   ETH_MACCFG_IPG_64BIT
#define ETH_INTERPACKETGAP_56BIT   ETH_MACCFG_IPG_56BIT
#define ETH_INTERPACKETGAP_48BIT   ETH_MACCFG_IPG_48BIT
#define ETH_INTERPACKETGAP_40BIT   ETH_MACCFG_IPG_40BIT
/**
  * @}
  */

/** @defgroup ETH_Speed  ETH Speed
  * @{
  */
#define ETH_SPEED_10M        ((uint32_t)0x00000000U)
#define ETH_SPEED_100M       0x00004000U
/**
  * @}
  */

/** @defgroup ETH_Duplex_Mode ETH Duplex Mode
  * @{
  */
#define ETH_FULLDUPLEX_MODE       ETH_MACCFG_DM
#define ETH_HALFDUPLEX_MODE       ((uint32_t)0x00000000U)
/**
  * @}
  */

/** @defgroup ETH_Back_Off_Limit ETH Back Off Limit
  * @{
  */
#define ETH_BACKOFFLIMIT_10  0x00000000U
#define ETH_BACKOFFLIMIT_8   0x00000020U
#define ETH_BACKOFFLIMIT_4   0x00000040U
#define ETH_BACKOFFLIMIT_1   0x00000060U
/**
  * @}
  */

/** @defgroup ETH_Preamble_Length ETH Preamble Length
  * @{
  */
#define ETH_PREAMBLELENGTH_7      ETH_MACCFG_PRELEN_7
#define ETH_PREAMBLELENGTH_5      ETH_MACCFG_PRELEN_5
#define ETH_PREAMBLELENGTH_3      ETH_MACCFG_PRELEN_3
/**
  * @}
  */

/** @defgroup ETH_Source_Addr_Control ETH Source Addr Control
  * @{
  */
#define ETH_SOURCEADDRESS_DISABLE           ((uint32_t)0x00000000U)
#define ETH_SOURCEADDRESS_INSERT_ADDR0      ETH_MACCFG_SARC_INSADDR0
#define ETH_SOURCEADDRESS_INSERT_ADDR1      ETH_MACCFG_SARC_INSADDR1
#define ETH_SOURCEADDRESS_REPLACE_ADDR0     ETH_MACCFG_SARC_REPADDR0
#define ETH_SOURCEADDRESS_REPLACE_ADDR1     ETH_MACCFG_SARC_REPADDR1
/**
  * @}
  */

/** @defgroup ETH_Control_Packets_Filter ETH Control Packets Filter
  * @{
  */
#define ETH_CTRLPACKETS_BLOCK_ALL                      ETH_MACPFR_PCF_BLOCKALL
#define ETH_CTRLPACKETS_FORWARD_ALL_EXCEPT_PA          ETH_MACPFR_PCF_FORWARDALLEXCEPTPA
#define ETH_CTRLPACKETS_FORWARD_ALL                    ETH_MACPFR_PCF_FORWARDALL
#define ETH_CTRLPACKETS_FORWARD_PASSED_ADDR_FILTER     ETH_MACPFR_PCF_FORWARDPASSEDADDRFILTER
/**
  * @}
  */

/** @defgroup ETH_VLAN_Tag_Comparison ETH VLAN Tag Comparison
  * @{
  */
#define ETH_VLANTAGCOMPARISON_12BIT    0x00010000U
#define ETH_VLANTAGCOMPARISON_16BIT    0x00000000U
/**
  * @}
  */

/** @defgroup ETH_MAC_addresses ETH MAC addresses
  * @{
  */
#define ETH_MAC_ADDRESS0     ((uint32_t)0x00000000U)
#define ETH_MAC_ADDRESS1     ((uint32_t)0x00000008U)
#define ETH_MAC_ADDRESS2     ((uint32_t)0x00000010U)
#define ETH_MAC_ADDRESS3     ((uint32_t)0x00000018U)
/**
  * @}
  */

/** @defgroup ETH_MAC_Interrupts ETH MAC Interrupts
  * @{
  */
#define ETH_MAC_PMT_IT           ETH_MACISTS_PMTIS
/**
  * @}
  */

/** @defgroup ETH_MAC_Wake_Up_Event ETH MAC Wake Up Event
  * @{
  */
#define ETH_WAKEUP_PACKET_RECIEVED    ETH_MACPCSR_RWKPRCVD
#define ETH_MAGIC_PACKET_RECIEVED     ETH_MACPCSR_MGKPRCVD
/**
  * @}
  */

/** @defgroup ETH_MAC_Rx_Tx_Status ETH MAC Rx Tx Status
  * @{
  */
#define ETH_RECEIVE_WATCHDOG_TIMEOUT        ETH_MACRXTXSR_RWT
#define ETH_EXECESSIVE_COLLISIONS           ETH_MACRXTXSR_EXCOL
#define ETH_LATE_COLLISIONS                 ETH_MACRXTXSR_LCOL
#define ETH_EXECESSIVE_DEFERRAL             ETH_MACRXTXSR_EXDEF
#define ETH_LOSS_OF_CARRIER                 ETH_MACRXTXSR_LCARR
#define ETH_NO_CARRIER                      ETH_MACRXTXSR_NCARR
#define ETH_TRANSMIT_JABBR_TIMEOUT          ETH_MACRXTXSR_TJT
/**
  * @}
  */

/** @defgroup DAL_ETH_StateTypeDef ETH States
  * @{
  */
#define DAL_ETH_STATE_RESET       ((uint32_t)0x00000000U)    /*!< Peripheral not yet Initialized or disabled */
#define DAL_ETH_STATE_READY       ((uint32_t)0x00000010U)    /*!< Peripheral Communication started           */
#define DAL_ETH_STATE_BUSY        ((uint32_t)0x00000023U)    /*!< an internal process is ongoing             */
#define DAL_ETH_STATE_STARTED     ((uint32_t)0x00000023U)    /*!< an internal process is started             */
#define DAL_ETH_STATE_ERROR       ((uint32_t)0x000000E0U)    /*!< Error State                                */
/**
  * @}
  */

/** @defgroup ETH_AutoNegotiation ETH AutoNegotiation
  * @{
  */
#define ETH_AUTONEGOTIATION_ENABLE     0x00000001U
#define ETH_AUTONEGOTIATION_DISABLE    0x00000000U

/**
  * @}
  */
/** @defgroup ETH_Rx_Mode ETH Rx Mode
  * @{
  */
#define ETH_RXPOLLING_MODE      0x00000000U
#define ETH_RXINTERRUPT_MODE    0x00000001U
/**
  * @}
  */

/** @defgroup ETH_Checksum_Mode ETH Checksum Mode
  * @{
  */
#define ETH_CHECKSUM_BY_HARDWARE      0x00000000U
#define ETH_CHECKSUM_BY_SOFTWARE      0x00000001U
/**
  * @}
  */

/** @defgroup ETH_Media_Interface ETH Media Interface
  * @{
  */
#define ETH_MEDIA_INTERFACE_MII       0x00000000U
#define ETH_MEDIA_INTERFACE_RMII      ((uint32_t)SYSCFG_PMCFG_ENETSEL)
/**
  * @}
  */

/** @defgroup ETH_Watchdog ETH Watchdog
  * @{
  */
#define ETH_WATCHDOG_ENABLE       0x00000000U
#define ETH_WATCHDOG_DISABLE      0x00800000U
/**
  * @}
  */

/** @defgroup ETH_Jabber ETH Jabber
  * @{
  */
#define ETH_JABBER_ENABLE    0x00000000U
#define ETH_JABBER_DISABLE   0x00400000U
/**
  * @}
  */

/** @defgroup ETH_Inter_Frame_Gap ETH Inter Frame Gap
  * @{
  */
#define ETH_INTERFRAMEGAP_96BIT   0x00000000U  /*!< minimum IFG between frames during transmission is 96Bit */
#define ETH_INTERFRAMEGAP_88BIT   0x00020000U  /*!< minimum IFG between frames during transmission is 88Bit */
#define ETH_INTERFRAMEGAP_80BIT   0x00040000U  /*!< minimum IFG between frames during transmission is 80Bit */
#define ETH_INTERFRAMEGAP_72BIT   0x00060000U  /*!< minimum IFG between frames during transmission is 72Bit */
#define ETH_INTERFRAMEGAP_64BIT   0x00080000U  /*!< minimum IFG between frames during transmission is 64Bit */
#define ETH_INTERFRAMEGAP_56BIT   0x000A0000U  /*!< minimum IFG between frames during transmission is 56Bit */
#define ETH_INTERFRAMEGAP_48BIT   0x000C0000U  /*!< minimum IFG between frames during transmission is 48Bit */
#define ETH_INTERFRAMEGAP_40BIT   0x000E0000U  /*!< minimum IFG between frames during transmission is 40Bit */
/**
  * @}
  */

/** @defgroup ETH_Carrier_Sense ETH Carrier Sense
  * @{
  */
#define ETH_CARRIERSENCE_ENABLE   0x00000000U
#define ETH_CARRIERSENCE_DISABLE  0x00010000U
/**
  * @}
  */

/** @defgroup ETH_Receive_Own ETH Receive Own
  * @{
  */
#define ETH_RECEIVEOWN_ENABLE     0x00000000U
#define ETH_RECEIVEOWN_DISABLE    0x00002000U
/**
  * @}
  */

/** @defgroup ETH_Loop_Back_Mode ETH Loop Back Mode
  * @{
  */
#define ETH_LOOPBACKMODE_ENABLE        0x00001000U
#define ETH_LOOPBACKMODE_DISABLE       0x00000000U
/**
  * @}
  */

/** @defgroup ETH_Checksum_Offload ETH Checksum Offload
  * @{
  */
#define ETH_CHECKSUMOFFLAOD_ENABLE     0x00000400U
#define ETH_CHECKSUMOFFLAOD_DISABLE    0x00000000U
/**
  * @}
  */

/** @defgroup ETH_Retry_Transmission ETH Retry Transmission
  * @{
  */
#define ETH_RETRYTRANSMISSION_ENABLE   0x00000000U
#define ETH_RETRYTRANSMISSION_DISABLE  0x00000200U
/**
  * @}
  */

/** @defgroup ETH_Automatic_Pad_CRC_Strip ETH Automatic Pad CRC Strip
  * @{
  */
#define ETH_AUTOMATICPADCRCSTRIP_ENABLE     0x00000080U
#define ETH_AUTOMATICPADCRCSTRIP_DISABLE    0x00000000U
/**
  * @}
  */

/** @defgroup ETH_Deferral_Check ETH Deferral Check
  * @{
  */
#define ETH_DEFFERRALCHECK_ENABLE       0x00000010U
#define ETH_DEFFERRALCHECK_DISABLE      0x00000000U
/**
  * @}
  */

/** @defgroup ETH_Receive_All ETH Receive All
  * @{
  */
#define ETH_RECEIVEALL_ENABLE     0x80000000U
#define ETH_RECEIVEALL_DISABLE    0x00000000U
/**
  * @}
  */

/** @defgroup ETH_Source_Addr_Filter ETH Source Addr Filter
  * @{
  */
#define ETH_SOURCEADDRFILTER_NORMAL_ENABLE       0x00000200U
#define ETH_SOURCEADDRFILTER_INVERSE_ENABLE      0x00000300U
#define ETH_SOURCEADDRFILTER_DISABLE             0x00000000U
/**
  * @}
  */

/** @defgroup ETH_Pass_Control_Frames ETH Pass Control Frames
  * @{
  */
#define ETH_PASSCONTROLFRAMES_BLOCKALL                0x00000040U  /*!< MAC filters all control frames from reaching the application */
#define ETH_PASSCONTROLFRAMES_FORWARDALL              0x00000080U  /*!< MAC forwards all control frames to application even if they fail the Address Filter */
#define ETH_PASSCONTROLFRAMES_FORWARDPASSEDADDRFILTER 0x000000C0U  /*!< MAC forwards control frames that pass the Address Filter. */
/**
  * @}
  */

/** @defgroup ETH_Broadcast_Frames_Reception ETH Broadcast Frames Reception
  * @{
  */
#define ETH_BROADCASTFRAMESRECEPTION_ENABLE     0x00000000U
#define ETH_BROADCASTFRAMESRECEPTION_DISABLE    0x00000020U
/**
  * @}
  */

/** @defgroup ETH_Destination_Addr_Filter ETH Destination Addr Filter
  * @{
  */
#define ETH_DESTINATIONADDRFILTER_NORMAL    0x00000000U
#define ETH_DESTINATIONADDRFILTER_INVERSE   0x00000008U
/**
  * @}
  */

/** @defgroup ETH_Promiscuous_Mode ETH Promiscuous Mode
  * @{
  */
#define ETH_PROMISCUOUS_MODE_ENABLE     0x00000001U
#define ETH_PROMISCUOUS_MODE_DISABLE    0x00000000U
/**
  * @}
  */

/** @defgroup ETH_Multicast_Frames_Filter ETH Multicast Frames Filter
  * @{
  */
#define ETH_MULTICASTFRAMESFILTER_PERFECTHASHTABLE    0x00000404U
#define ETH_MULTICASTFRAMESFILTER_HASHTABLE           0x00000004U
#define ETH_MULTICASTFRAMESFILTER_PERFECT             0x00000000U
#define ETH_MULTICASTFRAMESFILTER_NONE                0x00000010U
/**
  * @}
  */

/** @defgroup ETH_Unicast_Frames_Filter ETH Unicast Frames Filter
  * @{
  */
#define ETH_UNICASTFRAMESFILTER_PERFECTHASHTABLE 0x00000402U
#define ETH_UNICASTFRAMESFILTER_HASHTABLE        0x00000002U
#define ETH_UNICASTFRAMESFILTER_PERFECT          0x00000000U
/**
  * @}
  */

/** @defgroup ETH_Zero_Quanta_Pause ETH Zero Quanta Pause
  * @{
  */
#define ETH_ZEROQUANTAPAUSE_ENABLE     0x00000000U
#define ETH_ZEROQUANTAPAUSE_DISABLE    0x00000080U
/**
  * @}
  */

/** @defgroup ETH_Pause_Low_Threshold ETH Pause Low Threshold
  * @{
  */
#define ETH_PAUSELOWTHRESHOLD_MINUS4        0x00000000U  /*!< Pause time minus 4 slot times */
#define ETH_PAUSELOWTHRESHOLD_MINUS28       0x00000010U  /*!< Pause time minus 28 slot times */
#define ETH_PAUSELOWTHRESHOLD_MINUS144      0x00000020U  /*!< Pause time minus 144 slot times */
#define ETH_PAUSELOWTHRESHOLD_MINUS256      0x00000030U  /*!< Pause time minus 256 slot times */
/**
  * @}
  */

/** @defgroup ETH_Unicast_Pause_Frame_Detect ETH Unicast Pause Frame Detect
  * @{
  */
#define ETH_UNICASTPAUSEFRAMEDETECT_ENABLE  0x00000008U
#define ETH_UNICASTPAUSEFRAMEDETECT_DISABLE 0x00000000U
/**
  * @}
  */

/** @defgroup ETH_Receive_Flow_Control ETH Receive Flow Control
  * @{
  */
#define ETH_RECEIVEFLOWCONTROL_ENABLE       0x00000004U
#define ETH_RECEIVEFLOWCONTROL_DISABLE      0x00000000U
/**
  * @}
  */

/** @defgroup ETH_Transmit_Flow_Control ETH Transmit Flow Control
  * @{
  */
#define ETH_TRANSMITFLOWCONTROL_ENABLE      0x00000002U
#define ETH_TRANSMITFLOWCONTROL_DISABLE     0x00000000U
/**
  * @}
  */

/** @defgroup ETH_MAC_addresses_filter_SA_DA ETH MAC addresses filter SA DA
  * @{
  */
#define ETH_MAC_ADDRESSFILTER_SA       0x00000000U
#define ETH_MAC_ADDRESSFILTER_DA       0x00000008U
/**
  * @}
  */

/** @defgroup ETH_MAC_addresses_filter_Mask_bytes ETH MAC addresses filter Mask bytes
  * @{
  */
#define ETH_MAC_ADDRESSMASK_BYTE6      0x20000000U  /*!< Mask MAC Address high reg bits [15:8] */
#define ETH_MAC_ADDRESSMASK_BYTE5      0x10000000U  /*!< Mask MAC Address high reg bits [7:0] */
#define ETH_MAC_ADDRESSMASK_BYTE4      0x08000000U  /*!< Mask MAC Address low reg bits [31:24] */
#define ETH_MAC_ADDRESSMASK_BYTE3      0x04000000U  /*!< Mask MAC Address low reg bits [23:16] */
#define ETH_MAC_ADDRESSMASK_BYTE2      0x02000000U  /*!< Mask MAC Address low reg bits [15:8] */
#define ETH_MAC_ADDRESSMASK_BYTE1      0x01000000U  /*!< Mask MAC Address low reg bits [70] */
/**
  * @}
  */

/** @defgroup ETH_Transmit_Threshold_Control ETH Transmit Threshold Control
  * @{
  */
#define ETH_TRANSMITTHRESHOLDCONTROL_64BYTES     0x00000000U  /*!< threshold level of the MTL Transmit FIFO is 64 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_128BYTES    0x00004000U  /*!< threshold level of the MTL Transmit FIFO is 128 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_192BYTES    0x00008000U  /*!< threshold level of the MTL Transmit FIFO is 192 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_256BYTES    0x0000C000U  /*!< threshold level of the MTL Transmit FIFO is 256 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_40BYTES     0x00010000U  /*!< threshold level of the MTL Transmit FIFO is 40 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_32BYTES     0x00014000U  /*!< threshold level of the MTL Transmit FIFO is 32 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_24BYTES     0x00018000U  /*!< threshold level of the MTL Transmit FIFO is 24 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_16BYTES     0x0001C000U  /*!< threshold level of the MTL Transmit FIFO is 16 Bytes */
/**
  * @}
  */

/** @defgroup ETH_Receive_Threshold_Control ETH Receive Threshold Control
  * @{
  */
#define ETH_RECEIVEDTHRESHOLDCONTROL_64BYTES      0x00000000U  /*!< threshold level of the MTL Receive FIFO is 64 Bytes */
#define ETH_RECEIVEDTHRESHOLDCONTROL_32BYTES      0x00000008U  /*!< threshold level of the MTL Receive FIFO is 32 Bytes */
#define ETH_RECEIVEDTHRESHOLDCONTROL_96BYTES      0x00000010U  /*!< threshold level of the MTL Receive FIFO is 96 Bytes */
#define ETH_RECEIVEDTHRESHOLDCONTROL_128BYTES     0x00000018U  /*!< threshold level of the MTL Receive FIFO is 128 Bytes */
/**
  * @}
  */

/** @defgroup ETH_DMA_Arbitration ETH DMA Arbitration
  * @{
  */
#define ETH_DMAARBITRATION_ROUNDROBIN_RXTX_1_1   0x00000000U
#define ETH_DMAARBITRATION_ROUNDROBIN_RXTX_2_1   0x00004000U
#define ETH_DMAARBITRATION_ROUNDROBIN_RXTX_3_1   0x00008000U
#define ETH_DMAARBITRATION_ROUNDROBIN_RXTX_4_1   0x0000C000U
#define ETH_DMAARBITRATION_RXPRIORTX             0x00000002U
/**
  * @}
  */

/** @defgroup ETH_DMA_Tx_descriptor_segment ETH DMA Tx descriptor segment
  * @{
  */
#define ETH_DMATXDESC_LASTSEGMENTS      0x40000000U  /*!< Last Segment */
#define ETH_DMATXDESC_FIRSTSEGMENT      0x20000000U  /*!< First Segment */
/**
  * @}
  */

/** @defgroup ETH_DMA_Tx_descriptor_Checksum_Insertion_Control ETH DMA Tx descriptor Checksum Insertion Control
  * @{
  */
#define ETH_DMATXDESC_CHECKSUMBYPASS             0x00000000U   /*!< Checksum engine bypass */
#define ETH_DMATXDESC_CHECKSUMIPV4HEADER         0x00400000U   /*!< IPv4 header checksum insertion  */
#define ETH_DMATXDESC_CHECKSUMTCPUDPICMPSEGMENT  0x00800000U   /*!< TCP/UDP/ICMP checksum insertion. Pseudo header checksum is assumed to be present */
#define ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL     0x00C00000U   /*!< TCP/UDP/ICMP checksum fully in hardware including pseudo header */
/**
  * @}
  */

/** @defgroup ETH_DMA_Rx_descriptor_buffers ETH DMA Rx descriptor buffers
  * @{
  */
#define ETH_DMARXDESC_BUFFER1     0x00000000U  /*!< DMA Rx Desc Buffer1 */
#define ETH_DMARXDESC_BUFFER2     0x00000001U  /*!< DMA Rx Desc Buffer2 */
/**
  * @}
  */

/** @defgroup ETH_PMT_Flags ETH PMT Flags
  * @{
  */
#define ETH_PMT_FLAG_WUFFRPR      0x80000000U  /*!< Wake-Up Frame Filter Register Pointer Reset */
#define ETH_PMT_FLAG_WUFR         0x00000040U  /*!< Wake-Up Frame Received */
#define ETH_PMT_FLAG_MPR          0x00000020U  /*!< Magic Packet Received */
/**
  * @}
  */

/** @defgroup ETH_MMC_Tx_Interrupts ETH MMC Tx Interrupts
  * @{
  */
#define ETH_MMC_IT_TGF       0x00200000U  /*!< When Tx good frame counter reaches half the maximum value */
#define ETH_MMC_IT_TGFMSC    0x00008000U  /*!< When Tx good multi col counter reaches half the maximum value */
#define ETH_MMC_IT_TGFSC     0x00004000U  /*!< When Tx good single col counter reaches half the maximum value */
/**
  * @}
  */

/** @defgroup ETH_MMC_Rx_Interrupts ETH MMC Rx Interrupts
  * @{
  */
#define ETH_MMC_IT_RGUF      0x10020000U  /*!< When Rx good unicast frames counter reaches half the maximum value */
#define ETH_MMC_IT_RFAE      0x10000040U  /*!< When Rx alignment error counter reaches half the maximum value */
#define ETH_MMC_IT_RFCE      0x10000020U  /*!< When Rx crc error counter reaches half the maximum value */
/**
  * @}
  */

/** @defgroup ETH_MAC_Flags ETH MAC Flags
  * @{
  */
#define ETH_MAC_FLAG_TST     0x00000200U  /*!< Time stamp trigger flag (on MAC) */
#define ETH_MAC_FLAG_MMCT    0x00000040U  /*!< MMC transmit flag  */
#define ETH_MAC_FLAG_MMCR    0x00000020U  /*!< MMC receive flag */
#define ETH_MAC_FLAG_MMC     0x00000010U  /*!< MMC flag (on MAC) */
#define ETH_MAC_FLAG_PMT     0x00000008U  /*!< PMT flag (on MAC) */
/**
  * @}
  */

/** @defgroup ETH_DMA_Flags ETH DMA Flags
  * @{
  */
#define ETH_DMA_FLAG_TST               0x20000000U  /*!< Time-stamp trigger interrupt (on DMA) */
#define ETH_DMA_FLAG_PMT               0x10000000U  /*!< PMT interrupt (on DMA) */
#define ETH_DMA_FLAG_MMC               0x08000000U  /*!< MMC interrupt (on DMA) */
#define ETH_DMA_FLAG_DATATRANSFERERROR 0x00800000U  /*!< Error bits 0-Rx DMA, 1-Tx DMA */
#define ETH_DMA_FLAG_READWRITEERROR    0x01000000U  /*!< Error bits 0-write transfer, 1-read transfer */
#define ETH_DMA_FLAG_ACCESSERROR       0x02000000U  /*!< Error bits 0-data buffer, 1-desc. access */
#define ETH_DMA_FLAG_NIS               0x00010000U  /*!< Normal interrupt summary flag */
#define ETH_DMA_FLAG_AIS               0x00008000U  /*!< Abnormal interrupt summary flag */
#define ETH_DMA_FLAG_ER                0x00004000U  /*!< Early receive flag */
#define ETH_DMA_FLAG_FBE               0x00002000U  /*!< Fatal bus error flag */
#define ETH_DMA_FLAG_ET                0x00000400U  /*!< Early transmit flag */
#define ETH_DMA_FLAG_RWT               0x00000200U  /*!< Receive watchdog timeout flag */
#define ETH_DMA_FLAG_RPS               0x00000100U  /*!< Receive process stopped flag */
#define ETH_DMA_FLAG_RBU               0x00000080U  /*!< Receive buffer unavailable flag */
#define ETH_DMA_FLAG_R                 0x00000040U  /*!< Receive flag */
#define ETH_DMA_FLAG_TU                0x00000020U  /*!< Underflow flag */
#define ETH_DMA_FLAG_RO                0x00000010U  /*!< Overflow flag */
#define ETH_DMA_FLAG_TJT               0x00000008U  /*!< Transmit jabber timeout flag */
#define ETH_DMA_FLAG_TBU               0x00000004U  /*!< Transmit buffer unavailable flag */
#define ETH_DMA_FLAG_TPS               0x00000002U  /*!< Transmit process stopped flag */
#define ETH_DMA_FLAG_T                 0x00000001U  /*!< Transmit flag */
/**
  * @}
  */

/** @defgroup ETH_MAC_Interrupts ETH MAC Interrupts
  * @{
  */
#define ETH_MAC_IT_TST       0x00000200U  /*!< Time stamp trigger interrupt (on MAC) */
#define ETH_MAC_IT_MMCT      0x00000040U  /*!< MMC transmit interrupt */
#define ETH_MAC_IT_MMCR      0x00000020U  /*!< MMC receive interrupt */
#define ETH_MAC_IT_MMC       0x00000010U  /*!< MMC interrupt (on MAC) */
#define ETH_MAC_IT_PMT       0x00000008U  /*!< PMT interrupt (on MAC) */
/**
  * @}
  */

/** @defgroup ETH_DMA_Interrupts ETH DMA Interrupts
  * @{
  */
#define ETH_DMA_IT_TST       0x20000000U  /*!< Time-stamp trigger interrupt (on DMA) */
#define ETH_DMA_IT_PMT       0x10000000U  /*!< PMT interrupt (on DMA) */
#define ETH_DMA_IT_MMC       0x08000000U  /*!< MMC interrupt (on DMA) */
#define ETH_DMA_IT_NIS       0x00010000U  /*!< Normal interrupt summary */
#define ETH_DMA_IT_AIS       0x00008000U  /*!< Abnormal interrupt summary */
#define ETH_DMA_IT_ER        0x00004000U  /*!< Early receive interrupt */
#define ETH_DMA_IT_FBE       0x00002000U  /*!< Fatal bus error interrupt */
#define ETH_DMA_IT_ET        0x00000400U  /*!< Early transmit interrupt */
#define ETH_DMA_IT_RWT       0x00000200U  /*!< Receive watchdog timeout interrupt */
#define ETH_DMA_IT_RPS       0x00000100U  /*!< Receive process stopped interrupt */
#define ETH_DMA_IT_RBU       0x00000080U  /*!< Receive buffer unavailable interrupt */
#define ETH_DMA_IT_R         0x00000040U  /*!< Receive interrupt */
#define ETH_DMA_IT_TU        0x00000020U  /*!< Underflow interrupt */
#define ETH_DMA_IT_RO        0x00000010U  /*!< Overflow interrupt */
#define ETH_DMA_IT_TJT       0x00000008U  /*!< Transmit jabber timeout interrupt */
#define ETH_DMA_IT_TBU       0x00000004U  /*!< Transmit buffer unavailable interrupt */
#define ETH_DMA_IT_TPS       0x00000002U  /*!< Transmit process stopped interrupt */
#define ETH_DMA_IT_T         0x00000001U  /*!< Transmit interrupt */
/**
  * @}
  */

/** @defgroup ETH_DMA_transmit_process_state ETH DMA transmit process state
  * @{
  */
#define ETH_DMA_TRANSMITPROCESS_STOPPED     0x00000000U  /*!< Stopped - Reset or Stop Tx Command issued */
#define ETH_DMA_TRANSMITPROCESS_FETCHING    0x00100000U  /*!< Running - fetching the Tx descriptor */
#define ETH_DMA_TRANSMITPROCESS_WAITING     0x00200000U  /*!< Running - waiting for status */
#define ETH_DMA_TRANSMITPROCESS_READING     0x00300000U  /*!< Running - reading the data from host memory */
#define ETH_DMA_TRANSMITPROCESS_SUSPENDED   0x00600000U  /*!< Suspended - Tx Descriptor unavailable */
#define ETH_DMA_TRANSMITPROCESS_CLOSING     0x00700000U  /*!< Running - closing Rx descriptor */

/**
  * @}
  */


/** @defgroup ETH_DMA_receive_process_state ETH DMA receive process state
  * @{
  */
#define ETH_DMA_RECEIVEPROCESS_STOPPED      0x00000000U  /*!< Stopped - Reset or Stop Rx Command issued */
#define ETH_DMA_RECEIVEPROCESS_FETCHING     0x00020000U  /*!< Running - fetching the Rx descriptor */
#define ETH_DMA_RECEIVEPROCESS_WAITING      0x00060000U  /*!< Running - waiting for packet */
#define ETH_DMA_RECEIVEPROCESS_SUSPENDED    0x00080000U  /*!< Suspended - Rx Descriptor unavailable */
#define ETH_DMA_RECEIVEPROCESS_CLOSING      0x000A0000U  /*!< Running - closing descriptor */
#define ETH_DMA_RECEIVEPROCESS_QUEUING      0x000E0000U  /*!< Running - queuing the receive frame into host memory */

/**
  * @}
  */

/** @defgroup ETH_DMA_overflow ETH DMA overflow
  * @{
  */
#define ETH_DMA_OVERFLOW_RXFIFOCOUNTER      0x10000000U  /*!< Overflow bit for FIFO overflow counter */
#define ETH_DMA_OVERFLOW_MISSEDFRAMECOUNTER 0x00010000U  /*!< Overflow bit for missed frame counter */
/**
  * @}
  */
/** @defgroup ETH_PTP_Config_Status ETH PTP Config Status
  * @{
  */
#define DAL_ETH_PTP_NOT_CONFIGURATED       ((uint32_t)0x00000000U)    /*!< ETH PTP Configuration not done */
#define DAL_ETH_PTP_CONFIGURATED           ((uint32_t)0x00000001U)    /*!< ETH PTP Configuration done     */
/**
  * @}
  */
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup ETH_Exported_Macros ETH Exported Macros
  * @{
  */

/** @brief Reset ETH handle state
  * @param  __HANDLE__: specifies the ETH handle.
  * @retval None
  */
#if (USE_DAL_ETH_REGISTER_CALLBACKS == 1)
#define __DAL_ETH_RESET_HANDLE_STATE(__HANDLE__)  do{                                                   \
                                                      (__HANDLE__)->gState = DAL_ETH_STATE_RESET;      \
                                                      (__HANDLE__)->MspInitCallback = NULL;             \
                                                      (__HANDLE__)->MspDeInitCallback = NULL;           \
                                                    } while(0)
#else
#define __DAL_ETH_RESET_HANDLE_STATE(__HANDLE__)  do{                                                   \
                                                      (__HANDLE__)->gState = DAL_ETH_STATE_RESET;      \
                                                    } while(0)
#endif /*USE_DAL_ETH_REGISTER_CALLBACKS */

/**
  * @brief  Enables the specified ETHERNET DMA interrupts.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the ETHERNET DMA interrupt sources to be
  *   enabled @ref ETH_DMA_Interrupts
  * @retval None
  */
#define __DAL_ETH_DMA_ENABLE_IT(__HANDLE__, __INTERRUPT__)                 ((__HANDLE__)->Instance->DMAINTEN \
                                                                            |= (__INTERRUPT__))

/**
  * @brief  Disables the specified ETHERNET DMA interrupts.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the ETHERNET DMA interrupt sources to be
  *   disabled. @ref ETH_DMA_Interrupts
  * @retval None
  */
#define __DAL_ETH_DMA_DISABLE_IT(__HANDLE__, __INTERRUPT__)                ((__HANDLE__)->Instance->DMAINTEN \
                                                                            &= ~(__INTERRUPT__))

/**
  * @brief  Gets the ETHERNET DMA IT source enabled or disabled.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the interrupt source to get . @ref ETH_DMA_Interrupts
  * @retval The ETH DMA IT Source enabled or disabled
  */
#define __DAL_ETH_DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)      (((__HANDLE__)->Instance->DMAINTEN &\
                                                                      (__INTERRUPT__)) == (__INTERRUPT__))

/**
  * @brief  Gets the ETHERNET DMA IT pending bit.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the interrupt source to get . @ref ETH_DMA_Interrupts
  * @retval The state of ETH DMA IT (SET or RESET)
  */
#define __DAL_ETH_DMA_GET_IT(__HANDLE__, __INTERRUPT__)      (((__HANDLE__)->Instance->DMASTS &\
                                                               (__INTERRUPT__)) == (__INTERRUPT__))

/**
  * @brief  Clears the ETHERNET DMA IT pending bit.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the interrupt pending bit to clear. @ref ETH_DMA_Interrupts
  * @retval None
  */
#define __DAL_ETH_DMA_CLEAR_IT(__HANDLE__, __INTERRUPT__)      ((__HANDLE__)->Instance->DMASTS = (__INTERRUPT__))

/**
  * @brief  Checks whether the specified ETHERNET DMA flag is set or not.
  * @param  __HANDLE__: ETH Handle
  * @param  __FLAG__: specifies the flag to check. @ref ETH_DMA_Status_Flags
  * @retval The state of ETH DMA FLAG (SET or RESET).
  */
#define __DAL_ETH_DMA_GET_FLAG(__HANDLE__, __FLAG__)                   (((__HANDLE__)->Instance->DMACSR &\
                                                                         ( __FLAG__)) == ( __FLAG__))

/**
  * @brief  Clears the specified ETHERNET DMA flag.
  * @param  __HANDLE__: ETH Handle
  * @param  __FLAG__: specifies the flag to check. @ref ETH_DMA_Status_Flags
  * @retval The state of ETH DMA FLAG (SET or RESET).
  */
#define __DAL_ETH_DMA_CLEAR_FLAG(__HANDLE__, __FLAG__)                   ((__HANDLE__)->Instance->DMACSR = ( __FLAG__))

/**
  * @brief  Enables the specified ETHERNET MAC interrupts.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the ETHERNET MAC interrupt sources to be
  *   enabled @ref ETH_MAC_Interrupts
  *           @arg ETH_MAC_IT_TST
  *           @arg ETH_MAC_IT_PMT
  * @retval None
  */
#define __DAL_ETH_MAC_ENABLE_IT(__HANDLE__, __INTERRUPT__)                 ((__HANDLE__)->Instance->MACIMASK \
                                                                            |= (__INTERRUPT__))

/**
  * @brief  Disables the specified ETHERNET MAC interrupts.
  * @param  __HANDLE__   : ETH Handle
  * @param  __INTERRUPT__: specifies the ETHERNET MAC interrupt sources to be
  *   enabled @ref ETH_MAC_Interrupts
  *           @arg ETH_MAC_IT_TST
  *           @arg ETH_MAC_IT_PMT
  * @retval None
  */
#define __DAL_ETH_MAC_DISABLE_IT(__HANDLE__, __INTERRUPT__)                 ((__HANDLE__)->Instance->MACIMASK \
                                                                             &= ~(__INTERRUPT__))

/**
  * @brief  Checks whether the specified ETHERNET MAC flag is set or not.
  * @param  __HANDLE__: ETH Handle
  * @param  __INTERRUPT__: specifies the flag to check. @ref ETH_MAC_Interrupts
  * @retval The state of ETH MAC IT (SET or RESET).
  */
#define __DAL_ETH_MAC_GET_IT(__HANDLE__, __INTERRUPT__)                   (((__HANDLE__)->Instance->MACISTS &\
                                                                            ( __INTERRUPT__)) == ( __INTERRUPT__))

/*!< External interrupt line 19 Connected to the ETH wakeup EINT Line */
#define ETH_WAKEUP_EINT_LINE  ((uint32_t)0x00080000U)

/**
  * @brief Enable the ETH WAKEUP Exti Line.
  * @param  __EINT_LINE__: specifies the ETH WAKEUP Exti sources to be enabled.
  *   @arg ETH_WAKEUP_EINT_LINE
  * @retval None.
  */
#define __DAL_ETH_WAKEUP_EINT_ENABLE_IT(__EINT_LINE__)   (EINT->IMASK |= (__EINT_LINE__))

/**
  * @brief checks whether the specified ETH WAKEUP Exti interrupt flag is set or not.
  * @param  __EINT_LINE__: specifies the ETH WAKEUP Exti sources to be cleared.
  *   @arg ETH_WAKEUP_EINT_LINE
  * @retval EINT ETH WAKEUP Line Status.
  */
#define __DAL_ETH_WAKEUP_EINT_GET_FLAG(__EINT_LINE__)  (EINT->IPEND & (__EINT_LINE__))

/**
  * @brief Clear the ETH WAKEUP Exti flag.
  * @param  __EINT_LINE__: specifies the ETH WAKEUP Exti sources to be cleared.
  *   @arg ETH_WAKEUP_EINT_LINE
  * @retval None.
  */
#define __DAL_ETH_WAKEUP_EINT_CLEAR_FLAG(__EINT_LINE__) (EINT->IPEND = (__EINT_LINE__))


/**
  * @brief  enable rising edge interrupt on selected EINT line.
  * @param  __EINT_LINE__: specifies the ETH WAKEUP EINT sources to be disabled.
  *  @arg ETH_WAKEUP_EINT_LINE
  * @retval None
  */
#define __DAL_ETH_WAKEUP_EINT_ENABLE_RISING_EDGE(__EINT_LINE__) (EINT->FTEN &= ~(__EINT_LINE__)); \
  (EINT->RTEN |= (__EINT_LINE__))

/**
  * @brief  enable falling edge interrupt on selected EINT line.
  * @param  __EINT_LINE__: specifies the ETH WAKEUP EINT sources to be disabled.
  *  @arg ETH_WAKEUP_EINT_LINE
  * @retval None
  */
#define __DAL_ETH_WAKEUP_EINT_ENABLE_FALLING_EDGE(__EINT_LINE__) (EINT->RTEN &= ~(__EINT_LINE__));\
  (EINT->FTEN |= (__EINT_LINE__))

/**
  * @brief  enable falling edge interrupt on selected EINT line.
  * @param  __EINT_LINE__: specifies the ETH WAKEUP EINT sources to be disabled.
  *  @arg ETH_WAKEUP_EINT_LINE
  * @retval None
  */
#define __DAL_ETH_WAKEUP_EINT_ENABLE_RISING_FALLING_EDGE(__EINT_LINE__) (EINT->RTEN |= (__EINT_LINE__));\
  (EINT->FTEN |= (__EINT_LINE__))

/**
  * @brief  Generates a Software interrupt on selected EINT line.
  * @param  __EINT_LINE__: specifies the ETH WAKEUP EINT sources to be disabled.
  *  @arg ETH_WAKEUP_EINT_LINE
  * @retval None
  */
#define __DAL_ETH_WAKEUP_EINT_GENERATE_SWIT(__EINT_LINE__) (EINT->SWINTE |= (__EINT_LINE__))

#define __DAL_ETH_GET_PTP_CONTROL(__HANDLE__, __FLAG__) (((((__HANDLE__)->Instance->PTPTSCTRL) & \
                                                           (__FLAG__)) == (__FLAG__)) ? SET : RESET)

#define __DAL_ETH_SET_PTP_CONTROL(__HANDLE__, __FLAG__)   ((__HANDLE__)->Instance->PTPTSCTRL |= (__FLAG__))
/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/

/** @addtogroup ETH_Exported_Functions
  * @{
  */

/** @addtogroup ETH_Exported_Functions_Group1
  * @{
  */
/* Initialization and de initialization functions  **********************************/
DAL_StatusTypeDef DAL_ETH_Init(ETH_HandleTypeDef *heth);
DAL_StatusTypeDef DAL_ETH_DeInit(ETH_HandleTypeDef *heth);
void              DAL_ETH_MspInit(ETH_HandleTypeDef *heth);
void              DAL_ETH_MspDeInit(ETH_HandleTypeDef *heth);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_DAL_ETH_REGISTER_CALLBACKS == 1)
DAL_StatusTypeDef DAL_ETH_RegisterCallback(ETH_HandleTypeDef *heth, DAL_ETH_CallbackIDTypeDef CallbackID,
                                           pETH_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_ETH_UnRegisterCallback(ETH_HandleTypeDef *heth, DAL_ETH_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_ETH_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup ETH_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *******************************************************/
DAL_StatusTypeDef DAL_ETH_Start(ETH_HandleTypeDef *heth);
DAL_StatusTypeDef DAL_ETH_Start_IT(ETH_HandleTypeDef *heth);
DAL_StatusTypeDef DAL_ETH_Stop(ETH_HandleTypeDef *heth);
DAL_StatusTypeDef DAL_ETH_Stop_IT(ETH_HandleTypeDef *heth);

DAL_StatusTypeDef DAL_ETH_ReadData(ETH_HandleTypeDef *heth, void **pAppBuff);
DAL_StatusTypeDef DAL_ETH_RegisterRxAllocateCallback(ETH_HandleTypeDef *heth,
                                                     pETH_rxAllocateCallbackTypeDef rxAllocateCallback);
DAL_StatusTypeDef DAL_ETH_UnRegisterRxAllocateCallback(ETH_HandleTypeDef *heth);
DAL_StatusTypeDef DAL_ETH_RegisterRxLinkCallback(ETH_HandleTypeDef *heth, pETH_rxLinkCallbackTypeDef rxLinkCallback);
DAL_StatusTypeDef DAL_ETH_UnRegisterRxLinkCallback(ETH_HandleTypeDef *heth);
DAL_StatusTypeDef DAL_ETH_GetRxDataErrorCode(ETH_HandleTypeDef *heth, uint32_t *pErrorCode);
DAL_StatusTypeDef DAL_ETH_RegisterTxFreeCallback(ETH_HandleTypeDef *heth, pETH_txFreeCallbackTypeDef txFreeCallback);
DAL_StatusTypeDef DAL_ETH_UnRegisterTxFreeCallback(ETH_HandleTypeDef *heth);
DAL_StatusTypeDef DAL_ETH_ReleaseTxPacket(ETH_HandleTypeDef *heth);

#ifdef DAL_ETH_USE_PTP
DAL_StatusTypeDef DAL_ETH_PTP_SetConfig(ETH_HandleTypeDef *heth, ETH_PTP_ConfigTypeDef *ptpconfig);
DAL_StatusTypeDef DAL_ETH_PTP_GetConfig(ETH_HandleTypeDef *heth, ETH_PTP_ConfigTypeDef *ptpconfig);
DAL_StatusTypeDef DAL_ETH_PTP_SetTime(ETH_HandleTypeDef *heth, ETH_TimeTypeDef *time);
DAL_StatusTypeDef DAL_ETH_PTP_GetTime(ETH_HandleTypeDef *heth, ETH_TimeTypeDef *time);
DAL_StatusTypeDef DAL_ETH_PTP_AddTimeOffset(ETH_HandleTypeDef *heth, ETH_PtpUpdateTypeDef ptpoffsettype,
                                            ETH_TimeTypeDef *timeoffset);
DAL_StatusTypeDef DAL_ETH_PTP_InsertTxTimestamp(ETH_HandleTypeDef *heth);
DAL_StatusTypeDef DAL_ETH_PTP_GetTxTimestamp(ETH_HandleTypeDef *heth, ETH_TimeStampTypeDef *timestamp);
DAL_StatusTypeDef DAL_ETH_PTP_GetRxTimestamp(ETH_HandleTypeDef *heth, ETH_TimeStampTypeDef *timestamp);
DAL_StatusTypeDef DAL_ETH_RegisterTxPtpCallback(ETH_HandleTypeDef *heth, pETH_txPtpCallbackTypeDef txPtpCallback);
DAL_StatusTypeDef DAL_ETH_UnRegisterTxPtpCallback(ETH_HandleTypeDef *heth);
#endif /* DAL_ETH_USE_PTP */

DAL_StatusTypeDef DAL_ETH_Transmit(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig, uint32_t Timeout);
DAL_StatusTypeDef DAL_ETH_Transmit_IT(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig);

DAL_StatusTypeDef DAL_ETH_WritePHYRegister(ETH_HandleTypeDef *heth, uint32_t PHYAddr, uint32_t PHYReg,
                                           uint32_t RegValue);
DAL_StatusTypeDef DAL_ETH_ReadPHYRegister(ETH_HandleTypeDef *heth, uint32_t PHYAddr, uint32_t PHYReg,
                                          uint32_t *pRegValue);

void              DAL_ETH_IRQHandler(ETH_HandleTypeDef *heth);
void              DAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth);
void              DAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth);
void              DAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth);
void              DAL_ETH_PMTCallback(ETH_HandleTypeDef *heth);
void              DAL_ETH_WakeUpCallback(ETH_HandleTypeDef *heth);
void              DAL_ETH_RxAllocateCallback(uint8_t **buff);
void              DAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length);
void              DAL_ETH_TxFreeCallback(uint32_t *buff);
void              DAL_ETH_TxPtpCallback(uint32_t *buff, ETH_TimeStampTypeDef *timestamp);
/**
  * @}
  */

/** @addtogroup ETH_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions  **********************************************/
/* MAC & DMA Configuration APIs  **********************************************/
DAL_StatusTypeDef DAL_ETH_GetMACConfig(ETH_HandleTypeDef *heth, ETH_MACConfigTypeDef *macconf);
DAL_StatusTypeDef DAL_ETH_GetDMAConfig(ETH_HandleTypeDef *heth, ETH_DMAConfigTypeDef *dmaconf);
DAL_StatusTypeDef DAL_ETH_SetMACConfig(ETH_HandleTypeDef *heth, ETH_MACConfigTypeDef *macconf);
DAL_StatusTypeDef DAL_ETH_SetDMAConfig(ETH_HandleTypeDef *heth, ETH_DMAConfigTypeDef *dmaconf);
void              DAL_ETH_SetMDIOClockRange(ETH_HandleTypeDef *heth);

/* MAC VLAN Processing APIs    ************************************************/
void              DAL_ETH_SetRxVLANIdentifier(ETH_HandleTypeDef *heth, uint32_t ComparisonBits,
                                              uint32_t VLANIdentifier);

/* MAC L2 Packet Filtering APIs  **********************************************/
DAL_StatusTypeDef DAL_ETH_GetMACFilterConfig(ETH_HandleTypeDef *heth, ETH_MACFilterConfigTypeDef *pFilterConfig);
DAL_StatusTypeDef DAL_ETH_SetMACFilterConfig(ETH_HandleTypeDef *heth, ETH_MACFilterConfigTypeDef *pFilterConfig);
DAL_StatusTypeDef DAL_ETH_SetHashTable(ETH_HandleTypeDef *heth, uint32_t *pHashTable);
DAL_StatusTypeDef DAL_ETH_SetSourceMACAddrMatch(ETH_HandleTypeDef *heth, uint32_t AddrNbr, uint8_t *pMACAddr);

/* MAC Power Down APIs    *****************************************************/
void              DAL_ETH_EnterPowerDownMode(ETH_HandleTypeDef *heth, ETH_PowerDownConfigTypeDef *pPowerDownConfig);
void              DAL_ETH_ExitPowerDownMode(ETH_HandleTypeDef *heth);
DAL_StatusTypeDef DAL_ETH_SetWakeUpFilter(ETH_HandleTypeDef *heth, uint32_t *pFilter, uint32_t Count);

/**
  * @}
  */

/** @addtogroup ETH_Exported_Functions_Group4
  * @{
  */
/* Peripheral State functions  **************************************************/
DAL_ETH_StateTypeDef DAL_ETH_GetState(ETH_HandleTypeDef *heth);
uint32_t             DAL_ETH_GetError(ETH_HandleTypeDef *heth);
uint32_t             DAL_ETH_GetDMAError(ETH_HandleTypeDef *heth);
uint32_t             DAL_ETH_GetMACError(ETH_HandleTypeDef *heth);
uint32_t             DAL_ETH_GetMACWakeUpSource(ETH_HandleTypeDef *heth);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* ETH */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_ETH_H */


