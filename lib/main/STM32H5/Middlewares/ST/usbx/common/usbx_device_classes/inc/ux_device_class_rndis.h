/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

/**************************************************************************/
/**************************************************************************/
/**                                                                       */ 
/** USBX Component                                                        */ 
/**                                                                       */
/**   RNDIS Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_device_class_rndis.h                             PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file defines the equivalences for the USBX Device Class RNDIS  */ 
/*    component.                                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added wait and length DEFs, */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DEVICE_CLASS_RNDIS_H
#define UX_DEVICE_CLASS_RNDIS_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  

#if !defined(UX_DEVICE_STANDALONE)
#include "nx_api.h"
#include "ux_network_driver.h"
#else

/* Assume NX definitions for compiling.  */
#define NX_PACKET                                               VOID*
#define NX_IP                                                   VOID*
/*
UINT  _ux_network_driver_deactivate(VOID *ux_instance, VOID *ux_network_handle);
VOID  _ux_network_driver_link_up(VOID *ux_network_handle);
VOID  _ux_network_driver_link_down(VOID *ux_network_handle);
*/
#ifndef _ux_network_driver_deactivate
#define _ux_network_driver_deactivate(a,b)                      do {} while(0)
#endif
#ifndef _ux_network_driver_link_up
#define _ux_network_driver_link_up(a)                           do {} while(0)
#endif
#ifndef _ux_network_driver_link_down
#define _ux_network_driver_link_down(a)                         do {} while(0)
#endif
#endif

/* Define generic RNDIS equivalences.  */
#define UX_DEVICE_CLASS_RNDIS_CLASS_COMMUNICATION_CONTROL                       0x02
#define UX_DEVICE_CLASS_RNDIS_CLASS_COMMUNICATION_DATA                          0x0A
#define UX_DEVICE_CLASS_RNDIS_NEW_INTERRUPT_EVENT                               1
#define UX_DEVICE_CLASS_RNDIS_NEW_BULKOUT_EVENT                                 2
#define UX_DEVICE_CLASS_RNDIS_NEW_BULKIN_EVENT                                  4
#define UX_DEVICE_CLASS_RNDIS_NEW_DEVICE_STATE_CHANGE_EVENT                     8
#define UX_DEVICE_CLASS_RNDIS_INTERRUPT_RESPONSE_LENGTH                         8
#define UX_DEVICE_CLASS_RNDIS_INTERRUPT_RESPONSE_AVAILABLE_FLAG                 1
#define UX_DEVICE_CLASS_RNDIS_BASE_IP_ADDRESS                                   0xC0A80001
#define UX_DEVICE_CLASS_RNDIS_BASE_IP_MASK                                      0xFFFFFF00
#define UX_DEVICE_CLASS_RNDIS_MAX_MTU                                           1518
#define UX_DEVICE_CLASS_RNDIS_ETHERNET_IP                                       0x0800
#define UX_DEVICE_CLASS_RNDIS_ETHERNET_ARP                                      0x0806
#define UX_DEVICE_CLASS_RNDIS_ETHERNET_RARP                                     0x8035
#define UX_DEVICE_CLASS_RNDIS_ETHERNET_PACKET_SIZE                              1536    
#define UX_DEVICE_CLASS_RNDIS_NX_ALIGN_PADDING                                  2
#define UX_DEVICE_CLASS_RNDIS_NX_PKPOOL_ENTRIES                                 8  

#define UX_DEVICE_CLASS_RNDIS_NX_PACKET_SIZE                                    sizeof(NX_PACKET)

#define UX_DEVICE_CLASS_RNDIS_NX_PAYLOAD_SIZE_ASSERT                            UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(UX_DEVICE_CLASS_RNDIS_ETHERNET_PACKET_SIZE, UX_DEVICE_CLASS_RNDIS_NX_ALIGN_PADDING), UX_DEVICE_CLASS_RNDIS_NX_PAYLOAD_SIZE_calc_ovf)
#define UX_DEVICE_CLASS_RNDIS_NX_PAYLOAD_SIZE                                   (UX_DEVICE_CLASS_RNDIS_ETHERNET_PACKET_SIZE + UX_DEVICE_CLASS_RNDIS_NX_ALIGN_PADDING)

#define UX_DEVICE_CLASS_RNDIS_NX_BUFF_SIZE_ASSERT                           \
    UX_DEVICE_CLASS_RNDIS_NX_PAYLOAD_SIZE_ASSERT                            \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(                    \
        UX_DEVICE_CLASS_RNDIS_NX_PAYLOAD_SIZE,                              \
        UX_DEVICE_CLASS_RNDIS_NX_PACKET_SIZE),                              \
        UX_DEVICE_CLASS_RNDIS_NX_BUFF_SIZE_calc_ovf)
#define UX_DEVICE_CLASS_RNDIS_NX_BUFF_SIZE                                      (UX_DEVICE_CLASS_RNDIS_NX_PAYLOAD_SIZE + UX_DEVICE_CLASS_RNDIS_NX_PACKET_SIZE)

#define UX_DEVICE_CLASS_RNDIS_NX_ETHERNET_POOL_ALLOCSIZE_ASSERT             \
    UX_DEVICE_CLASS_RNDIS_NX_BUFF_SIZE_ASSERT                               \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_MULC_ULONG(                   \
        UX_DEVICE_CLASS_RNDIS_NX_PKPOOL_ENTRIES,                            \
        UX_DEVICE_CLASS_RNDIS_NX_BUFF_SIZE),                                \
        UX_DEVICE_CLASS_RNDIS_NX_ETHERNET_POOL_ALLOCSIZE_calc1_ovf)         \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(                    \
        UX_DEVICE_CLASS_RNDIS_NX_PKPOOL_ENTRIES *                           \
            UX_DEVICE_CLASS_RNDIS_NX_BUFF_SIZE,                             \
        32), UX_DEVICE_CLASS_RNDIS_NX_ETHERNET_POOL_ALLOCSIZE_calc2_ovf)
#define UX_DEVICE_CLASS_RNDIS_NX_ETHERNET_POOL_ALLOCSIZE                        (UX_DEVICE_CLASS_RNDIS_NX_PKPOOL_ENTRIES * UX_DEVICE_CLASS_RNDIS_NX_BUFF_SIZE + 32)

#define UX_DEVICE_CLASS_RNDIS_ETHERNET_SIZE                                     14
#define UX_DEVICE_CLASS_RNDIS_NODE_ID_LENGTH                                    6  
#define UX_DEVICE_CLASS_RNDIS_VENDOR_DESCRIPTION_MAX_LENGTH                     64  
#define UX_DEVICE_CLASS_RNDIS_MAC_OPTIONS                                       8
#define UX_DEVICE_CLASS_RNDIS_PACKET_HEADER_MSG                                 1 

#define UX_DEVICE_CLASS_RNDIS_OID_SUPPORTED_LIST_LENGTH                         30

/* Device RNDIS Requests */
#define UX_DEVICE_CLASS_RNDIS_SEND_ENCAPSULATED_COMMAND                         0x00
#define UX_DEVICE_CLASS_RNDIS_GET_ENCAPSULATED_RESPONSE                         0x01

/* Define RNDIS Versions.  Set to 1.0 here.  */
#define UX_DEVICE_CLASS_RNDIS_VERSION_MAJOR                                     0x00000001
#define UX_DEVICE_CLASS_RNDIS_VERSION_MINOR                                     0x00000000

/* Define RNDIS Connection type supported. Set to conectionless.  */
#define UX_DEVICE_CLASS_RNDIS_DF_CONNECTIONLESS                                 0x00000001
#define UX_DEVICE_CLASS_RNDIS_DF_CONNECTION_ORIENTED                            0x00000002
#define UX_DEVICE_CLASS_RNDIS_DF_CONNECTION_SUPPORTED                           UX_DEVICE_CLASS_RNDIS_DF_CONNECTIONLESS

/* Define RNDIS Medium supported by the device. */
#define UX_DEVICE_CLASS_RNDIS_MEDIUM_SUPPORTED                                  0x00000000

/* Define RNDIS Packet size and types supported.  */
#define UX_DEVICE_CLASS_RNDIS_MAX_PACKET_PER_TRANSFER                           0x00000001
#define UX_DEVICE_CLASS_RNDIS_MAX_PACKET_TRANSFER_SIZE                          0x00000640
#define UX_DEVICE_CLASS_RNDIS_PACKET_ALIGNEMENT_FACTOR                          0x00000003
#define UX_DEVICE_CLASS_RNDIS_MAX_FRAME_SIZE                                    0x000005DC
#define UX_DEVICE_CLASS_RNDIS_MAX_PACKET_LENGTH                                 0x000005EA

/* Define LINK speeds.  */
#define UX_DEVICE_CLASS_RNDIS_LINK_SPEED_FS                                     0x0001D4C0

/* Define LINK statess.  */
#define UX_DEVICE_CLASS_RNDIS_LINK_STATE_DOWN                                   0
#define UX_DEVICE_CLASS_RNDIS_LINK_STATE_UP                                     1
#define UX_DEVICE_CLASS_RNDIS_LINK_STATE_PENDING_UP                             2
#define UX_DEVICE_CLASS_RNDIS_LINK_STATE_PENDING_DOWN                           3

/* Define media connection values.  */
#define UX_DEVICE_CLASS_RNDIS_MEDIA_CONNECTED                                   0x00000000
#define UX_DEVICE_CLASS_RNDIS_MEDIA_DISCONNECTED                                0x00000001

/* Define media supported values.  */
#define UX_DEVICE_CLASS_RNDIS_MEDIA_802_3                                       0x00000000
#define UX_DEVICE_CLASS_RNDIS_MEDIA_802_5                                       0x00000001
#define UX_DEVICE_CLASS_RNDIS_MEDIA_FDDI                                        0x00000002
#define UX_DEVICE_CLASS_RNDIS_MEDIA_WAN                                         0x00000003
#define UX_DEVICE_CLASS_RNDIS_MEDIA_LOCAL_TALK                                  0x00000004
#define UX_DEVICE_CLASS_RNDIS_MEDIA_DIX                                         0x00000005
#define UX_DEVICE_CLASS_RNDIS_MEDIA_ARCNET_RAW                                  0x00000006
#define UX_DEVICE_CLASS_RNDIS_MEDIA_ARCNET_878_2                                0x00000007
#define UX_DEVICE_CLASS_RNDIS_MEDIA_ATM                                         0x00000008
#define UX_DEVICE_CLASS_RNDIS_MEDIA_WIRELESS_WAN                                0x00000009
#define UX_DEVICE_CLASS_RNDIS_MEDIA_IRDA                                        0x0000000A

/* Define RNDIS status values.  */
#define UX_DEVICE_CLASS_RNDIS_STATUS_SUCCESS                                    0x00000000
#define UX_DEVICE_CLASS_RNDIS_STATUS_FAILURE                                    0xC0000001
#define UX_DEVICE_CLASS_RNDIS_STATUS_INVALID_DATA                               0xC0010015
#define UX_DEVICE_CLASS_RNDIS_STATUS_NOT_SUPPORTED                              0xC00000BB
#define UX_DEVICE_CLASS_RNDIS_STATUS_MEDIA_CONNECTED                            0x4001000B
#define UX_DEVICE_CLASS_RNDIS_STATUS_MEDIA_DISCONNECT                           0x4001000C

/* Define RNDIS Control Messages values.  */
#define UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE                                    0x00000002
#define UX_DEVICE_CLASS_RNDIS_MSG_HALT                                          0x00000003
#define UX_DEVICE_CLASS_RNDIS_MSG_QUERY                                         0x00000004
#define UX_DEVICE_CLASS_RNDIS_MSG_SET                                           0x00000005
#define UX_DEVICE_CLASS_RNDIS_MSG_RESET                                         0x00000006
#define UX_DEVICE_CLASS_RNDIS_MSG_INDICATE_STATUS                               0x00000007
#define UX_DEVICE_CLASS_RNDIS_MSG_KEEP_ALIVE                                    0x00000008

/* Define RNDIS Control Completion values.  */
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE                                  0x80000002
#define UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY                                       0x80000004
#define UX_DEVICE_CLASS_RNDIS_CMPLT_SET                                         0x80000005
#define UX_DEVICE_CLASS_RNDIS_CMPLT_RESET                                       0x80000006
#define UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE                                  0x80000008

/* Define RNDIS Control Messages : MSG_INITIALIZE offsets.  */
#define UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE_MESSAGE_TYPE                       0x00000000
#define UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE_MESSAGE_LENGTH                     0x00000004
#define UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE_REQUEST_ID                         0x00000008
#define UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE_MAJOR_VERSION                      0x0000000C
#define UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE_MINOR_VERSION                      0x00000010
#define UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE_MAX_TRANSFER_SIZE                  0x00000014

/* Define RNDIS Control Messages : CMPLT_INITIALIZE offsets.  */
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MESSAGE_TYPE                     0x00000000
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MESSAGE_LENGTH                   0x00000004
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_REQUEST_ID                       0x00000008
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_STATUS                           0x0000000C
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MAJOR_VERSION                    0x00000010
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MINOR_VERSION                    0x00000014
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_DEVICE_FLAGS                     0x00000018
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MEDIUM                           0x0000001C
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MAX_PACKETS_PER_TRANSFER         0x00000020
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_MAX_TRANSFER_SIZE                0x00000024
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_PACKET_ALIGNMENT                 0x00000028
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_AFL_LIST_OFFSET                  0x0000002C
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_AFL_LIST_SIZE                    0x00000030
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_RESPONSE_LENGTH                  0x00000034

/* Define RNDIS Control Messages : MSG_HALT offsets.  */
#define UX_DEVICE_CLASS_RNDIS_MSG_HALT_MESSAGE_TYPE                             0x00000000
#define UX_DEVICE_CLASS_RNDIS_MSG_HALT_MESSAGE_LENGTH                           0x00000004
#define UX_DEVICE_CLASS_RNDIS_MSG_HALT_REQUEST_ID                               0x00000008

/* Define RNDIS Control Messages : MSG_QUERY offsets.  */
#define UX_DEVICE_CLASS_RNDIS_MSG_QUERY_MESSAGE_TYPE                            0x00000000
#define UX_DEVICE_CLASS_RNDIS_MSG_QUERY_MESSAGE_LENGTH                          0x00000004
#define UX_DEVICE_CLASS_RNDIS_MSG_QUERY_REQUEST_ID                              0x00000008
#define UX_DEVICE_CLASS_RNDIS_MSG_QUERY_OID                                     0x0000000C
#define UX_DEVICE_CLASS_RNDIS_MSG_QUERY_INFO_BUFFER_LENGTH                      0x00000010
#define UX_DEVICE_CLASS_RNDIS_MSG_QUERY_INFO_BUFFER_OFFSET                      0x00000014
#define UX_DEVICE_CLASS_RNDIS_MSG_QUERY_DEVICE_VC_HANDLE                        0x00000018

#define UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_MESSAGE_TYPE                          0x00000000
#define UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_MESSAGE_LENGTH                        0x00000004
#define UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_REQUEST_ID                            0x00000008
#define UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_STATUS                                0x0000000C
#define UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER_LENGTH                    0x00000010
#define UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER_OFFSET                    0x00000014
#define UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER                           0x00000018

/* Define RNDIS Control Messages : MSG_SET offsets.  */
#define UX_DEVICE_CLASS_RNDIS_MSG_SET_MESSAGE_TYPE                              0x00000000
#define UX_DEVICE_CLASS_RNDIS_MSG_SET_MESSAGE_LENGTH                            0x00000004
#define UX_DEVICE_CLASS_RNDIS_MSG_SET_REQUEST_ID                                0x00000008
#define UX_DEVICE_CLASS_RNDIS_MSG_SET_OID                                       0x0000000C
#define UX_DEVICE_CLASS_RNDIS_MSG_SET_INFO_BUFFER_LENGTH                        0x00000010
#define UX_DEVICE_CLASS_RNDIS_MSG_SET_INFO_BUFFER_OFFSET                        0x00000014
#define UX_DEVICE_CLASS_RNDIS_MSG_SET_DEVICE_VC_HANDLE                          0x00000018
#define UX_DEVICE_CLASS_RNDIS_CMPLT_SET_MESSAGE_TYPE                            0x00000000
#define UX_DEVICE_CLASS_RNDIS_CMPLT_SET_MESSAGE_LENGTH                          0x00000004
#define UX_DEVICE_CLASS_RNDIS_CMPLT_SET_REQUEST_ID                              0x00000008
#define UX_DEVICE_CLASS_RNDIS_CMPLT_SET_STATUS                                  0x0000000C
#define UX_DEVICE_CLASS_RNDIS_CMPLT_SET_RESPONSE_LENGTH                         0x00000010

/* Define RNDIS Control Messages : MSG_RESET offsets.  */
#define UX_DEVICE_CLASS_RNDIS_MSG_RESET_MESSAGE_TYPE                            0x00000000
#define UX_DEVICE_CLASS_RNDIS_MSG_RESET_MESSAGE_LENGTH                          0x00000004
#define UX_DEVICE_CLASS_RNDIS_MSG_RESET_RESERVED                                0x00000008
#define UX_DEVICE_CLASS_RNDIS_CMPLT_RESET_MESSAGE_TYPE                          0x00000000
#define UX_DEVICE_CLASS_RNDIS_CMPLT_RESET_MESSAGE_LENGTH                        0x00000004
#define UX_DEVICE_CLASS_RNDIS_CMPLT_RESET_STATUS                                0x00000008
#define UX_DEVICE_CLASS_RNDIS_CMPLT_RESET_ADDRESSING_RESET                      0x0000000C
#define UX_DEVICE_CLASS_RNDIS_CMPLT_RESET_RESPONSE_LENGTH                       0x00000010

/* Define RNDIS Control Messages : MSG_INDICATE_STATUS offsets.  */
#define UX_DEVICE_CLASS_RNDIS_MSG_INDICATE_STATUS_MESSAGE_TYPE                  0x00000000
#define UX_DEVICE_CLASS_RNDIS_MSG_INDICATE_STATUS_MESSAGE_LENGTH                0x00000004
#define UX_DEVICE_CLASS_RNDIS_MSG_INDICATE_STATUS_STATUS                        0x00000008
#define UX_DEVICE_CLASS_RNDIS_MSG_INDICATE_STATUS_STATUS_BUFFER_LENGTH          0x0000000C
#define UX_DEVICE_CLASS_RNDIS_MSG_INDICATE_STATUS_STATUS_BUFFER_OFFSET          0x00000010
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INDICATE_STATUS_DIAG                        0x00000000
#define UX_DEVICE_CLASS_RNDIS_CMPLT_INDICATE_STATUS_ERROR_OFFSET                0x00000004

/* Define RNDIS Control Messages : MSG_KEEP_ALIVE offsets.  */
#define UX_DEVICE_CLASS_RNDIS_MSG_KEEP_ALIVE_MESSAGE_TYPE                       0x00000000
#define UX_DEVICE_CLASS_RNDIS_MSG_KEEP_ALIVE_MESSAGE_LENGTH                     0x00000004
#define UX_DEVICE_CLASS_RNDIS_MSG_KEEP_ALIVE_REQUEST_ID                         0x00000008
#define UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_MESSAGE_TYPE                     0x00000000
#define UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_MESSAGE_LENGTH                   0x00000004
#define UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_REQUEST_ID                       0x00000008
#define UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_STATUS                           0x0000000C
#define UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_RESPONSE_LENGTH                  0x00000010

/* Define RNDIS State machine.  */
#define UX_DEVICE_CLASS_RNDIS_STATE_UNINITIALIZED                               0x00000000
#define UX_DEVICE_CLASS_RNDIS_STATE_INITIALIZED                                 0x00000001
#define UX_DEVICE_CLASS_RNDIS_STATE_DATA_INITIALIZED                            0x00000002

/* Define Required Object IDs (OIDs) */
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_SUPPORTED_LIST                            0x00010101
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_HARDWARE_STATUS                           0x00010102
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_SUPPORTED                           0x00010103
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_IN_USE                              0x00010104
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MAXIMUM_LOOKAHEAD                         0x00010105
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MAXIMUM_FRAME_SIZE                        0x00010106
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_LINK_SPEED                                0x00010107
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_TRANSMIT_BUFFER_SPACE                     0x00010108
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_RECEIVE_BUFFER_SPACE                      0x00010109
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_TRANSMIT_BLOCK_SIZE                       0x0001010A
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_RECEIVE_BLOCK_SIZE                        0x0001010B
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_VENDOR_ID                                 0x0001010C
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_VENDOR_DESCRIPTION                        0x0001010D
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_CURRENT_PACKET_FILTER                     0x0001010E
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_CURRENT_LOOKAHEAD                         0x0001010F
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_DRIVER_VERSION                            0x00010110
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE                        0x00010111
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_PROTOCOL_OPTIONS                          0x00010112
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MAC_OPTIONS                               0x00010113
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_CONNECT_STATUS                      0x00010114
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MAXIMUM_SEND_PACKETS                      0x00010115
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_VENDOR_DRIVER_VERSION                     0x00010116
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_SUPPORTED_GUIDS                           0x00010117
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_NETWORK_LAYER_ADDRESSES                   0x00010118
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_TRANSPORT_HEADER_OFFSET                   0x00010119
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MACHINE_NAME                              0x0001021A
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_RNDIS_CONFIG_PARAMETER                    0x0001021B
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_VLAN_ID                                   0x0001021C

/* Define Optional OIDs */
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_CAPABILITIES                        0x00010201
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_PHYSICAL_MEDIUM                           0x00010202

/* Define Required statistics OIDs */
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_XMIT_OK                                   0x00020101
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_RCV_OK                                    0x00020102
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_XMIT_ERROR                                0x00020103
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_RCV_ERROR                                 0x00020104
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_RCV_NO_BUFFER                             0x00020105

/* Define Optional statistics OIDs */
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_DIRECTED_BYTES_XMIT                       0x00020201
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_DIRECTED_FRAMES_XMIT                      0x00020202
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MULTICAST_BYTES_XMIT                      0x00020203
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MULTICAST_FRAMES_XMIT                     0x00020204
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_BROADCAST_BYTES_XMIT                      0x00020205
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_BROADCAST_FRAMES_XMIT                     0x00020206
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_DIRECTED_BYTES_RCV                        0x00020207
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_DIRECTED_FRAMES_RCV                       0x00020208
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MULTICAST_BYTES_RCV                       0x00020209
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MULTICAST_FRAMES_RCV                      0x0002020A
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_BROADCAST_BYTES_RCV                       0x0002020B
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_BROADCAST_FRAMES_RCV                      0x0002020C
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_RCV_CRC_ERROR                             0x0002020D
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_TRANSMIT_QUEUE_LENGTH                     0x0002020E
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_GET_TIME_CAPS                             0x0002020F
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_GET_NETCARD_TIME                          0x00020210
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_NETCARD_LOAD                              0x00020211
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_DEVICE_PROFILE                            0x00020212
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_INIT_TIME_MS                              0x00020213
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_RESET_COUNTS                              0x00020214
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MEDIA_SENSE_COUNTS                        0x00020215
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_FRIENDLY_NAME                             0x00020216
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_MINIPORT_INFO                             0x00020217
#define UX_DEVICE_CLASS_RNDIS_OID_GEN_RESET_VERIFY_PARAMETERS                   0x00020218

/* Define IEEE 802.3 (Ethernet) OIDs */

#define UX_DEVICE_CLASS_RNDIS_OID_802_3_PERMANENT_ADDRESS                       0x01010101
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_CURRENT_ADDRESS                         0x01010102
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_MULTICAST_LIST                          0x01010103
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_MAXIMUM_LIST_SIZE                       0x01010104
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_MAC_OPTIONS                             0x01010105
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_RCV_ERROR_ALIGNMENT                     0x01020101
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_XMIT_ONE_COLLISION                      0x01020102
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_XMIT_MORE_COLLISIONS                    0x01020103
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_XMIT_DEFERRED                           0x01020201
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_XMIT_MAX_COLLISIONS                     0x01020202
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_RCV_OVERRUN                             0x01020203
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_XMIT_UNDERRUN                           0x01020204
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_XMIT_HEARTBEAT_FAILURE                  0x01020205
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_XMIT_TIMES_CRS_LOST                     0x01020206
#define UX_DEVICE_CLASS_RNDIS_OID_802_3_XMIT_LATE_COLLISIONS                    0x01020207

/* Define Hardware status code.  */
#define UX_DEVICE_CLASS_RNDIS_OID_HW_STATUS_READY                               0x00000000
#define UX_DEVICE_CLASS_RNDIS_OID_HW_STATUS_INITIALIZING                        0x00000001
#define UX_DEVICE_CLASS_RNDIS_OID_HW_STATUS_RESET                               0x00000002
#define UX_DEVICE_CLASS_RNDIS_OID_HW_STATUS_CLOSING                             0x00000003
#define UX_DEVICE_CLASS_RNDIS_OID_HW_STATUS_NOT_READY                           0x00000004

/* Define RNDIS Packet Header format.  */
#define UX_DEVICE_CLASS_RNDIS_PACKET_MESSAGE_TYPE                               0x00000000
#define UX_DEVICE_CLASS_RNDIS_PACKET_MESSAGE_LENGTH                             0x00000004
#define UX_DEVICE_CLASS_RNDIS_PACKET_DATA_OFFSET                                0x00000008
#define UX_DEVICE_CLASS_RNDIS_PACKET_DATA_LENGTH                                0x0000000C
#define UX_DEVICE_CLASS_RNDIS_PACKET_OOB_DATA_OFFSET                            0x00000010
#define UX_DEVICE_CLASS_RNDIS_PACKET_OOB_DATA_LENGTH                            0x00000014
#define UX_DEVICE_CLASS_RNDIS_PACKET_NUM_OOB_DATA_ELEMENTS                      0x00000018
#define UX_DEVICE_CLASS_RNDIS_PACKET_INFO_OFFSET                                0x0000001C
#define UX_DEVICE_CLASS_RNDIS_PACKET_INFO_LENGTH                                0x00000020
#define UX_DEVICE_CLASS_RNDIS_PACKET_VC_HANDLE                                  0x00000024
#define UX_DEVICE_CLASS_RNDIS_PACKET_RESERVED                                   0x00000028
#define UX_DEVICE_CLASS_RNDIS_PACKET_BUFFER                                     0x0000002C
#define UX_DEVICE_CLASS_RNDIS_PACKET_HEADER_LENGTH                              0x0000002C

/* Define NetX errors inside the RNDIS class.  */
#define UX_DEVICE_CLASS_RNDIS_NX_SUCCESS                                        0x00
#define UX_DEVICE_CLASS_RNDIS_NX_NO_PACKET                                      0x01
#define UX_DEVICE_CLASS_RNDIS_NX_UNDERFLOW                                      0x02
#define UX_DEVICE_CLASS_RNDIS_NX_OVERFLOW                                       0x03
#define UX_DEVICE_CLASS_RNDIS_NX_NO_MAPPING                                     0x04
#define UX_DEVICE_CLASS_RNDIS_NX_DELETED                                        0x05
#define UX_DEVICE_CLASS_RNDIS_NX_POOL_ERROR                                     0x06
#define UX_DEVICE_CLASS_RNDIS_NX_PTR_ERROR                                      0x07
#define UX_DEVICE_CLASS_RNDIS_NX_WAIT_ERROR                                     0x08
#define UX_DEVICE_CLASS_RNDIS_NX_SIZE_ERROR                                     0x09
#define UX_DEVICE_CLASS_RNDIS_NX_OPTION_ERROR                                   0x0a
#define UX_DEVICE_CLASS_RNDIS_NX_DELETE_ERROR                                   0x10
#define UX_DEVICE_CLASS_RNDIS_NX_CALLER_ERROR                                   0x11
#define UX_DEVICE_CLASS_RNDIS_NX_INVALID_PACKET                                 0x12
#define UX_DEVICE_CLASS_RNDIS_NX_INVALID_SOCKET                                 0x13
#define UX_DEVICE_CLASS_RNDIS_NX_NOT_ENABLED                                    0x14
#define UX_DEVICE_CLASS_RNDIS_NX_ALREADY_ENABLED                                0x15
#define UX_DEVICE_CLASS_RNDIS_NX_ENTRY_NOT_FOUND                                0x16
#define UX_DEVICE_CLASS_RNDIS_NX_NO_MORE_ENTRIES                                0x17
#define UX_DEVICE_CLASS_RNDIS_NX_ARP_TIMER_ERROR                                0x18
#define UX_DEVICE_CLASS_RNDIS_NX_RESERVED_CODE0                                 0x19
#define UX_DEVICE_CLASS_RNDIS_NX_WAIT_ABORTED                                   0x1A
#define UX_DEVICE_CLASS_RNDIS_NX_IP_INTERNAL_ERROR                              0x20
#define UX_DEVICE_CLASS_RNDIS_NX_IP_ADDRESS_ERROR                               0x21
#define UX_DEVICE_CLASS_RNDIS_NX_ALREADY_BOUND                                  0x22
#define UX_DEVICE_CLASS_RNDIS_NX_PORT_UNAVAILABLE                               0x23
#define UX_DEVICE_CLASS_RNDIS_NX_NOT_BOUND                                      0x24
#define UX_DEVICE_CLASS_RNDIS_NX_RESERVED_CODE1                                 0x25
#define UX_DEVICE_CLASS_RNDIS_NX_SOCKET_UNBOUND                                 0x26
#define UX_DEVICE_CLASS_RNDIS_NX_NOT_CREATED                                    0x27
#define UX_DEVICE_CLASS_RNDIS_NX_SOCKETS_BOUND                                  0x28
#define UX_DEVICE_CLASS_RNDIS_NX_NO_RESPONSE                                    0x29
#define UX_DEVICE_CLASS_RNDIS_NX_POOL_DELETED                                   0x30
#define UX_DEVICE_CLASS_RNDIS_NX_ALREADY_RELEASED                               0x31
#define UX_DEVICE_CLASS_RNDIS_NX_RESERVED_CODE2                                 0x32
#define UX_DEVICE_CLASS_RNDIS_NX_MAX_LISTEN                                     0x33
#define UX_DEVICE_CLASS_RNDIS_NX_DUPLICATE_LISTEN                               0x34
#define UX_DEVICE_CLASS_RNDIS_NX_NOT_CLOSED                                     0x35
#define UX_DEVICE_CLASS_RNDIS_NX_NOT_LISTEN_STATE                               0x36
#define UX_DEVICE_CLASS_RNDIS_NX_IN_PROGRESS                                    0x37
#define UX_DEVICE_CLASS_RNDIS_NX_NOT_CONNECTED                                  0x38
#define UX_DEVICE_CLASS_RNDIS_NX_WINDOW_OVERFLOW                                0x39
#define UX_DEVICE_CLASS_RNDIS_NX_ALREADY_SUSPENDED                              0x40
#define UX_DEVICE_CLASS_RNDIS_NX_DISCONNECT_FAILED                              0x41
#define UX_DEVICE_CLASS_RNDIS_NX_STILL_BOUND                                    0x42
#define UX_DEVICE_CLASS_RNDIS_NX_NOT_SUCCESSFUL                                 0x43
#define UX_DEVICE_CLASS_RNDIS_NX_UNHANDLED_COMMAND                              0x44
#define UX_DEVICE_CLASS_RNDIS_NX_NO_FREE_PORTS                                  0x45
#define UX_DEVICE_CLASS_RNDIS_NX_INVALID_PORT                                   0x46
#define UX_DEVICE_CLASS_RNDIS_NX_INVALID_RELISTEN                               0x47
#define UX_DEVICE_CLASS_RNDIS_NX_CONNECTION_PENDING                             0x48
#define UX_DEVICE_CLASS_RNDIS_NX_TX_QUEUE_DEPTH                                 0x49
#define UX_DEVICE_CLASS_RNDIS_NX_NOT_IMPLEMENTED                                0x80

/* Define timeout packet allocation value.  */
#ifndef UX_DEVICE_CLASS_RNDIS_PACKET_POOL_WAIT
#define UX_DEVICE_CLASS_RNDIS_PACKET_POOL_WAIT                                  10  
#endif

#ifndef UX_DEVICE_CLASS_RNDIS_PACKET_POOL_INST_WAIT
#define UX_DEVICE_CLASS_RNDIS_PACKET_POOL_INST_WAIT                             100
#endif

/* Calculate message buffer length (not overflow).  */
#define UX_DEVICE_CLASS_RNDIS_MAX_MSG_LENGTH                                    (UX_DEVICE_CLASS_RNDIS_MAX_PACKET_LENGTH + UX_DEVICE_CLASS_RNDIS_PACKET_HEADER_LENGTH)
#if UX_DEVICE_CLASS_RNDIS_MAX_MSG_LENGTH > UX_SLAVE_REQUEST_DATA_MAX_LENGTH
#error "Error: the maximum-sized RNDIS response cannot fit inside the control endpoint's data buffer. Increase UX_SLAVE_REQUEST_DATA_MAX_LENGTH."
#endif

/* Calculate response buffer length.  */
#define UX_DEVICE_CLASS_RNDIS_OID_SUPPORTED_RESPONSE_LENGTH             (UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER + UX_DEVICE_CLASS_RNDIS_OID_SUPPORTED_LIST_LENGTH * 4)
#define UX_DEVICE_CLASS_RNDIS_VENDOR_DESCRIPTION_MAX_RESPONSE_LENGTH    (UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER + UX_DEVICE_CLASS_RNDIS_VENDOR_DESCRIPTION_MAX_LENGTH)
#define UX_DEVICE_CLASS_RNDIS_NODE_ID_RESPONSE_LENGTH                   (UX_DEVICE_CLASS_RNDIS_CMPLT_QUERY_INFO_BUFFER + UX_DEVICE_CLASS_RNDIS_NODE_ID_LENGTH)

/* Decide maximum size of RNDIS response buffer (with 1 ~ 4 bytes as padding and aligned to 4 bytes).  */
#define UX_DEVICE_CLASS_RNDIS_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define UX_DEVICE_CLASS_RNDIS_MAX_CONTROL_RESPONSE_LENGTH               (((                         \
        UX_DEVICE_CLASS_RNDIS_MAX(UX_DEVICE_CLASS_RNDIS_CMPLT_INITIALIZE_RESPONSE_LENGTH,           \
        UX_DEVICE_CLASS_RNDIS_MAX(UX_DEVICE_CLASS_RNDIS_CMPLT_KEEP_ALIVE_RESPONSE_LENGTH,           \
        UX_DEVICE_CLASS_RNDIS_MAX(UX_DEVICE_CLASS_RNDIS_CMPLT_RESET_RESPONSE_LENGTH,                \
        UX_DEVICE_CLASS_RNDIS_MAX(UX_DEVICE_CLASS_RNDIS_CMPLT_SET_RESPONSE_LENGTH,                  \
        UX_DEVICE_CLASS_RNDIS_MAX(UX_DEVICE_CLASS_RNDIS_OID_SUPPORTED_RESPONSE_LENGTH,              \
        UX_DEVICE_CLASS_RNDIS_MAX(UX_DEVICE_CLASS_RNDIS_VENDOR_DESCRIPTION_MAX_RESPONSE_LENGTH,     \
        UX_DEVICE_CLASS_RNDIS_NODE_ID_RESPONSE_LENGTH))))))) & (~0x3)) + 0x4)

/* Ensure maximum-sized RNDIS response can fit in the control endpoint's transfer buffer.  */
#if UX_DEVICE_CLASS_RNDIS_MAX_CONTROL_RESPONSE_LENGTH > UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH
#error "Error: the maximum-sized RNDIS response cannot fit inside the control endpoint's data buffer. Increase UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH."
#endif

/* Define Slave RNDIS Class Calling Parameter structure */

typedef struct UX_SLAVE_CLASS_RNDIS_PARAMETER_STRUCT
{
    VOID                    (*ux_slave_class_rndis_instance_activate)(VOID *);
    VOID                    (*ux_slave_class_rndis_instance_deactivate)(VOID *);
    ULONG                   ux_slave_class_rndis_parameter_media;
    ULONG                   ux_slave_class_rndis_parameter_vendor_id;
    ULONG                   ux_slave_class_rndis_parameter_driver_version;
    UCHAR                   ux_slave_class_rndis_parameter_vendor_description[UX_DEVICE_CLASS_RNDIS_VENDOR_DESCRIPTION_MAX_LENGTH];
    UCHAR                   ux_slave_class_rndis_parameter_local_node_id[UX_DEVICE_CLASS_RNDIS_NODE_ID_LENGTH];
    UCHAR                   ux_slave_class_rndis_parameter_remote_node_id[UX_DEVICE_CLASS_RNDIS_NODE_ID_LENGTH];
    NX_IP                   *ux_slave_class_rndis_parameter_nx_ip;
    ULONG                   ux_slave_class_rndis_parameter_nx_ip_address;
    ULONG                   ux_slave_class_rndis_parameter_nx_ip_network_mask;
    
} UX_SLAVE_CLASS_RNDIS_PARAMETER;

/* Define RNDIS Class structure.  */

typedef struct UX_SLAVE_CLASS_RNDIS_STRUCT
{
    UX_SLAVE_INTERFACE                      *ux_slave_class_rndis_interface;
    UX_SLAVE_CLASS_RNDIS_PARAMETER          ux_slave_class_rndis_parameter;
    UX_SEMAPHORE                            ux_slave_class_rndis_semaphore;
    UX_SLAVE_ENDPOINT                       *ux_slave_class_rndis_interrupt_endpoint;
    UX_SLAVE_ENDPOINT                       *ux_slave_class_rndis_bulkin_endpoint;
    UX_SLAVE_ENDPOINT                       *ux_slave_class_rndis_bulkout_endpoint;
    UCHAR                                   ux_slave_class_rndis_response[UX_DEVICE_CLASS_RNDIS_MAX_CONTROL_RESPONSE_LENGTH];
    ULONG                                   ux_slave_class_rndis_response_length;
    ULONG                                   ux_slave_class_rndis_state;
    ULONG                                   ux_slave_class_rndis_major_version;
    ULONG                                   ux_slave_class_rndis_minor_version;
    ULONG                                   ux_slave_class_rndis_max_transfer_size;
    ULONG                                   ux_slave_class_rndis_request_id;
    ULONG                                   ux_slave_class_rndis_statistics_xmit_ok;
    ULONG                                   ux_slave_class_rndis_statistics_rcv_ok;
    ULONG                                   ux_slave_class_rndis_statistics_xmit_error;
    ULONG                                   ux_slave_class_rndis_statistics_rcv_error;
    ULONG                                   ux_slave_class_rndis_statistics_rcv_no_buffer;
    ULONG                                   ux_slave_class_rndis_statistics_rcv_error_alignment;
    ULONG                                   ux_slave_class_rndis_statistics_xmit_one_collision;
    ULONG                                   ux_slave_class_rndis_statistics_xmit_more_collisions;
    UCHAR                                   ux_slave_class_rndis_local_node_id[UX_DEVICE_CLASS_RNDIS_NODE_ID_LENGTH];
    UCHAR                                   ux_slave_class_rndis_remote_node_id[UX_DEVICE_CLASS_RNDIS_NODE_ID_LENGTH];
    ULONG                                   ux_slave_class_rndis_nx_ip_address;
    ULONG                                   ux_slave_class_rndis_nx_ip_network_mask;

#if !defined(UX_DEVICE_STANDALONE)
    NX_IP                                   *ux_slave_class_rndis_nx_ip;
    NX_INTERFACE                            *ux_slave_class_rndis_nx_interface;
    NX_PACKET                               *ux_slave_class_rndis_xmit_queue;
    NX_PACKET                               *ux_slave_class_rndis_receive_queue;
    NX_PACKET_POOL                          *ux_slave_class_rndis_packet_pool;
#endif

#if !defined(UX_DEVICE_STANDALONE)
    UX_EVENT_FLAGS_GROUP                    ux_slave_class_rndis_event_flags_group;
    UX_THREAD                               ux_slave_class_rndis_interrupt_thread;
    UX_THREAD                               ux_slave_class_rndis_bulkin_thread;
    UX_THREAD                               ux_slave_class_rndis_bulkout_thread;
    UX_MUTEX                                ux_slave_class_rndis_mutex;
    UCHAR                                   *ux_slave_class_rndis_interrupt_thread_stack;
    UCHAR                                   *ux_slave_class_rndis_bulkin_thread_stack;
    UCHAR                                   *ux_slave_class_rndis_bulkout_thread_stack;
#endif

    ULONG                                   ux_slave_class_rndis_link_state;
    VOID                                    *ux_slave_class_rndis_network_handle;
    
} UX_SLAVE_CLASS_RNDIS;


/* Requests - Ethernet Networking Control Model */

#define UX_DEVICE_CLASS_RNDIS_SEND_ENCAPSULATED_COMMAND                        0x00        
                                        /* Issues a command in the format of the supported control
                                           protocol. The intent of this mechanism is to support
                                           networking devices (e.g., host-based cable modems)
                                           that require an additional vendor-defined interface for
                                           media specific hardware configuration and
                                           management.  */
#define UX_DEVICE_CLASS_RNDIS_GET_ENCAPSULATED_RESPONSE                        0x01        
                                        /* Requests a response in the format of the supported
                                           control protocol.  */


/* Define buffer length for IN/OUT pipes.  */

#define UX_DEVICE_CLASS_RNDIS_BUFFER_SIZE                                       4096


/* Define Device RNDIS Class prototypes.  */

UINT  _ux_device_class_rndis_activate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_rndis_control_request(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_rndis_deactivate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_rndis_entry(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_rndis_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_rndis_write(VOID *rndis_class, NX_PACKET *packet);
UINT  _ux_device_class_rndis_msg_query(UX_SLAVE_CLASS_RNDIS *rndis, UX_SLAVE_TRANSFER *transfer_request);
UINT  _ux_device_class_rndis_msg_reset(UX_SLAVE_CLASS_RNDIS *rndis, UX_SLAVE_TRANSFER *transfer_request);
UINT  _ux_device_class_rndis_msg_set(UX_SLAVE_CLASS_RNDIS *rndis, UX_SLAVE_TRANSFER *transfer_request);
UINT  _ux_device_class_rndis_msg_initialize(UX_SLAVE_CLASS_RNDIS *rndis, UX_SLAVE_TRANSFER *transfer_request);
UINT  _ux_device_class_rndis_msg_keep_alive(UX_SLAVE_CLASS_RNDIS *rndis, UX_SLAVE_TRANSFER *transfer_request);
VOID  _ux_device_class_rndis_interrupt_thread(ULONG rndis_class);
VOID  _ux_device_class_rndis_bulkin_thread(ULONG rndis_class);
VOID  _ux_device_class_rndis_bulkout_thread(ULONG rndis_class);


/* Define Device RNDIS Class API prototypes.  */

#define ux_device_class_rndis_entry        _ux_device_class_rndis_entry


/* Define OID supported List.  */
extern ULONG ux_device_class_rndis_oid_supported_list[];

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif /* UX_DEVICE_CLASS_RNDIS_H */
