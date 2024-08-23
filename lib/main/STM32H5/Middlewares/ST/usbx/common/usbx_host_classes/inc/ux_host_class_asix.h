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
/**   ASIX Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_asix.h                                PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX ASIX class.                                                    */ 
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
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            supported NX packet chain,  */
/*                                            refined VID/PID check flow, */
/*                                            removed internal NX pool,   */
/*                                            added some new definitions, */
/*                                            refined reception handling, */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/

#ifndef UX_HOST_CLASS_ASIX_H
#define UX_HOST_CLASS_ASIX_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  


#if !defined(UX_HOST_STANDALONE)
#include "nx_api.h"
#include "ux_network_driver.h"
#else

/* Assume NX things for compiling.  */
#define NX_PACKET                                   VOID*
#define NX_PACKET_POOL                              VOID*
#endif

/* Define to check if NetX preserved header size is compatible with ASIX.  */
/* #define UX_HOST_CLASS_ASIX_HEADER_CHECK_ENABLE */


/* Define ASIX Class constants.  Insert here the PID/VID of vendors and products using the Asix chipset. */
#ifndef UX_HOST_CLASS_ASIX_VENDOR_ID
#define UX_HOST_CLASS_ASIX_VENDOR_ID                        0X2001
#define UX_HOST_CLASS_ASIX_PRODUCT_ID                       0X3C05
#endif

#define UX_HOST_CLASS_ASIX_VENDOR_FUJIEI_ID                 0X0B95
#define UX_HOST_CLASS_ASIX_PRODUCT_FUJIEI_ID                0X772B

#define UX_HOST_CLASS_ASIX_VENDOR_LINKSYS_ID                0X13B1
#define UX_HOST_CLASS_ASIX_PRODUCT_LINKSYS_ID               0X0018

/* Define ASIX supported VID and PID array (of 16-bits).  */
#ifndef UX_HOST_CLASS_ASIX_VID_PID_ARRAY
#define UX_HOST_CLASS_ASIX_VID_PID_ARRAY                                        \
    UX_HOST_CLASS_ASIX_VENDOR_ID, UX_HOST_CLASS_ASIX_PRODUCT_ID,                \
    UX_HOST_CLASS_ASIX_VENDOR_FUJIEI_ID, UX_HOST_CLASS_ASIX_PRODUCT_FUJIEI_ID,  \
    UX_HOST_CLASS_ASIX_VENDOR_LINKSYS_ID, UX_HOST_CLASS_ASIX_PRODUCT_LINKSYS_ID
#endif

#define UX_HOST_CLASS_ASIX_SPEED_SELECTED_100MPBS           0x100 
#define UX_HOST_CLASS_ASIX_SPEED_SELECTED_10MPBS            0x10 
#define UX_HOST_CLASS_ASIX_LINK_STATE_DOWN                  0
#define UX_HOST_CLASS_ASIX_LINK_STATE_UP                    1
#define UX_HOST_CLASS_ASIX_LINK_STATE_PENDING_UP            2
#define UX_HOST_CLASS_ASIX_LINK_STATE_PENDING_DOWN          3
#define UX_HOST_CLASS_ASIX_BASE_IP_ADDRESS                  0xC0A80001
#define UX_HOST_CLASS_ASIX_BASE_IP_MASK                     0xFFFFFF00
#define UX_HOST_CLASS_ASIX_MAX_MTU                          1518
#define UX_HOST_CLASS_ASIX_ETHERNET_IP                      0x0800
#define UX_HOST_CLASS_ASIX_ETHERNET_ARP                     0x0806
#define UX_HOST_CLASS_ASIX_ETHERNET_RARP                    0x8035
#define UX_HOST_CLASS_ASIX_ETHERNET_PACKET_SIZE             1536    
#define UX_HOST_CLASS_ASIX_NX_ALIGN_PADDING                 2
#define UX_HOST_CLASS_ASIX_RX_HEADER_SIZE                   4
#define UX_HOST_CLASS_ASIX_OVERHEAD_SIZE                    (UX_HOST_CLASS_ASIX_NX_ALIGN_PADDING + UX_HOST_CLASS_ASIX_RX_HEADER_SIZE)

#define UX_HOST_CLASS_ASIX_TRANSMIT_BUFFER_SIZE             UX_HOST_CLASS_ASIX_NX_PAYLOAD_SIZE

#ifndef UX_HOST_CLASS_ASIX_RECEIVE_BUFFER_SIZE
#define UX_HOST_CLASS_ASIX_RECEIVE_BUFFER_SIZE              512 /* N*512.  */
#endif

#ifdef NX_DISABLE_PACKET_CHAIN
#undef UX_HOST_CLASS_ASIX_PACKET_CHAIN_SUPPORT
#else
#define UX_HOST_CLASS_ASIX_PACKET_CHAIN_SUPPORT
#endif

#define UX_HOST_CLASS_ASIX_NX_PACKET_SIZE                   sizeof(NX_PACKET)

#define UX_HOST_CLASS_ASIX_NX_PAYLOAD_SIZE_ASSERT                           \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(                    \
        UX_HOST_CLASS_ASIX_ETHERNET_PACKET_SIZE,                            \
        UX_HOST_CLASS_ASIX_OVERHEAD_SIZE),                                  \
        UX_HOST_CLASS_ASIX_NX_PAYLOAD_SIZE_calc_ovf)
#define UX_HOST_CLASS_ASIX_NX_PAYLOAD_SIZE                  (UX_HOST_CLASS_ASIX_ETHERNET_PACKET_SIZE + UX_HOST_CLASS_ASIX_OVERHEAD_SIZE)

#define UX_HOST_CLASS_ASIX_NX_BUFF_SIZE_ASSERT                              \
    UX_HOST_CLASS_ASIX_NX_PAYLOAD_SIZE_ASSERT                               \
    UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_ADD_ULONG(                    \
        UX_HOST_CLASS_ASIX_NX_PAYLOAD_SIZE,                                 \
        UX_HOST_CLASS_ASIX_NX_PACKET_SIZE),                                 \
        UX_HOST_CLASS_ASIX_NX_BUFF_SIZE_calc_ovf)
#define UX_HOST_CLASS_ASIX_NX_BUFF_SIZE                     (UX_HOST_CLASS_ASIX_NX_PAYLOAD_SIZE + UX_HOST_CLASS_ASIX_NX_PACKET_SIZE)

#define UX_HOST_CLASS_ASIX_ETHERNET_SIZE                    14
                                                                
#define UX_HOST_CLASS_ASIX_DEVICE_INIT_DELAY                (1 * UX_PERIODIC_RATE)
#define UX_HOST_CLASS_ASIX_CLASS_TRANSFER_TIMEOUT           300000
#define UX_HOST_CLASS_ASIX_SETUP_BUFFER_SIZE                16

#define UX_HOST_CLASS_ASIX_PACKET_POOL_WAIT                 100
#define UX_HOST_CLASS_ASIX_PACKET_ALLOCATE_WAIT             2000

/* Define NetX errors inside the Asix class.  */
#define UX_HOST_CLASS_ASIX_NX_SUCCESS                       0x00
#define UX_HOST_CLASS_ASIX_NX_NO_PACKET                     0x01
#define UX_HOST_CLASS_ASIX_NX_UNDERFLOW                     0x02
#define UX_HOST_CLASS_ASIX_NX_OVERFLOW                      0x03
#define UX_HOST_CLASS_ASIX_NX_NO_MAPPING                    0x04
#define UX_HOST_CLASS_ASIX_NX_DELETED                       0x05
#define UX_HOST_CLASS_ASIX_NX_POOL_ERROR                    0x06
#define UX_HOST_CLASS_ASIX_NX_PTR_ERROR                     0x07
#define UX_HOST_CLASS_ASIX_NX_WAIT_ERROR                    0x08
#define UX_HOST_CLASS_ASIX_NX_SIZE_ERROR                    0x09
#define UX_HOST_CLASS_ASIX_NX_OPTION_ERROR                  0x0a
#define UX_HOST_CLASS_ASIX_NX_DELETE_ERROR                  0x10
#define UX_HOST_CLASS_ASIX_NX_CALLER_ERROR                  0x11
#define UX_HOST_CLASS_ASIX_NX_INVALID_PACKET                0x12
#define UX_HOST_CLASS_ASIX_NX_INVALID_SOCKET                0x13
#define UX_HOST_CLASS_ASIX_NX_NOT_ENABLED                   0x14
#define UX_HOST_CLASS_ASIX_NX_ALREADY_ENABLED               0x15
#define UX_HOST_CLASS_ASIX_NX_ENTRY_NOT_FOUND               0x16
#define UX_HOST_CLASS_ASIX_NX_NO_MORE_ENTRIES               0x17
#define UX_HOST_CLASS_ASIX_NX_ARP_TIMER_ERROR               0x18
#define UX_HOST_CLASS_ASIX_NX_RESERVED_CODE0                0x19
#define UX_HOST_CLASS_ASIX_NX_WAIT_ABORTED                  0x1A
#define UX_HOST_CLASS_ASIX_NX_IP_INTERNAL_ERROR             0x20
#define UX_HOST_CLASS_ASIX_NX_IP_ADDRESS_ERROR              0x21
#define UX_HOST_CLASS_ASIX_NX_ALREADY_BOUND                 0x22
#define UX_HOST_CLASS_ASIX_NX_PORT_UNAVAILABLE              0x23
#define UX_HOST_CLASS_ASIX_NX_NOT_BOUND                     0x24
#define UX_HOST_CLASS_ASIX_NX_RESERVED_CODE1                0x25
#define UX_HOST_CLASS_ASIX_NX_SOCKET_UNBOUND                0x26
#define UX_HOST_CLASS_ASIX_NX_NOT_CREATED                   0x27
#define UX_HOST_CLASS_ASIX_NX_SOCKETS_BOUND                 0x28
#define UX_HOST_CLASS_ASIX_NX_NO_RESPONSE                   0x29
#define UX_HOST_CLASS_ASIX_NX_POOL_DELETED                  0x30
#define UX_HOST_CLASS_ASIX_NX_ALREADY_RELEASED              0x31
#define UX_HOST_CLASS_ASIX_NX_RESERVED_CODE2                0x32
#define UX_HOST_CLASS_ASIX_NX_MAX_LISTEN                    0x33
#define UX_HOST_CLASS_ASIX_NX_DUPLICATE_LISTEN              0x34
#define UX_HOST_CLASS_ASIX_NX_NOT_CLOSED                    0x35
#define UX_HOST_CLASS_ASIX_NX_NOT_LISTEN_STATE              0x36
#define UX_HOST_CLASS_ASIX_NX_IN_PROGRESS                   0x37
#define UX_HOST_CLASS_ASIX_NX_NOT_CONNECTED                 0x38
#define UX_HOST_CLASS_ASIX_NX_WINDOW_OVERFLOW               0x39
#define UX_HOST_CLASS_ASIX_NX_ALREADY_SUSPENDED             0x40
#define UX_HOST_CLASS_ASIX_NX_DISCONNECT_FAILED             0x41
#define UX_HOST_CLASS_ASIX_NX_STILL_BOUND                   0x42
#define UX_HOST_CLASS_ASIX_NX_NOT_SUCCESSFUL                0x43
#define UX_HOST_CLASS_ASIX_NX_UNHANDLED_COMMAND             0x44
#define UX_HOST_CLASS_ASIX_NX_NO_FREE_PORTS                 0x45
#define UX_HOST_CLASS_ASIX_NX_INVALID_PORT                  0x46
#define UX_HOST_CLASS_ASIX_NX_INVALID_RELISTEN              0x47
#define UX_HOST_CLASS_ASIX_NX_CONNECTION_PENDING            0x48
#define UX_HOST_CLASS_ASIX_NX_TX_QUEUE_DEPTH                0x49
#define UX_HOST_CLASS_ASIX_NX_NOT_IMPLEMENTED               0x80



/* Define ASIX command request values.  */

#define UX_HOST_CLASS_ASIX_REQ_RX_TX_SRAM_REG_READ          0x02
#define UX_HOST_CLASS_ASIX_REQ_RX_TX_SRAM_REG_WRITE         0x03
#define UX_HOST_CLASS_ASIX_REQ_OWN_SMI                      0x06
#define UX_HOST_CLASS_ASIX_REQ_READ_PHY_REG                 0x07
#define UX_HOST_CLASS_ASIX_REQ_WRITE_PHY_REG                0x08
#define UX_HOST_CLASS_ASIX_REQ_READ_STATION_STATUS          0x09
#define UX_HOST_CLASS_ASIX_REQ_WHO_OWNS_SMI                 0x09
#define UX_HOST_CLASS_ASIX_REQ_RELEASE_SMI                  0x0a
#define UX_HOST_CLASS_ASIX_REQ_READ_SROM                    0x0b
#define UX_HOST_CLASS_ASIX_REQ_WRITE_SROM                   0x0c
#define UX_HOST_CLASS_ASIX_REQ_WRITE_SROM_EN                0x0d
#define UX_HOST_CLASS_ASIX_REQ_WRITE_SROM_DIS               0x0e
#define UX_HOST_CLASS_ASIX_REQ_READ_RX_CTL                  0x0f
#define UX_HOST_CLASS_ASIX_REQ_WRITE_RX_CTL                 0x10
#define UX_HOST_CLASS_ASIX_REQ_READ_IPG012                  0x11
#define UX_HOST_CLASS_ASIX_REQ_WRITE_IPG012                 0x12
#define UX_HOST_CLASS_ASIX_REQ_READ_NODE_ID                 0x13
#define UX_HOST_CLASS_ASIX_REQ_WRITE_NODE_ID                0x14
#define UX_HOST_CLASS_ASIX_REQ_WRITE_MULTICAST_FILTER       0x16
#define UX_HOST_CLASS_ASIX_REQ_TEST_REGISTER                0x17
#define UX_HOST_CLASS_ASIX_REQ_READ_PHY_ID                  0x19
#define UX_HOST_CLASS_ASIX_REQ_READ_MEDIUM_STATUS           0x1a
#define UX_HOST_CLASS_ASIX_REQ_WRITE_MEDIUM_MODE            0x1b
#define UX_HOST_CLASS_ASIX_REQ_READ_MONITOR_MODE_STATUS     0x1c
#define UX_HOST_CLASS_ASIX_REQ_WRITE_MONITOR_MODE_STATUS    0x1d
#define UX_HOST_CLASS_ASIX_REQ_READ_GPIO_STATUS             0x1e
#define UX_HOST_CLASS_ASIX_REQ_WRITE_GPIO_STATUS            0x1f
#define UX_HOST_CLASS_ASIX_REQ_SW_RESET                     0x20
#define UX_HOST_CLASS_ASIX_REQ_READ_SW_PHY_SELECT_STATUS    0x21
#define UX_HOST_CLASS_ASIX_REQ_WRITE_SW_PHY_SELECT_STATUS   0x22

/* Define  ASIX Interrupt Packet format.  */

#define UX_HOST_CLASS_ASIX_INTERRUPT_SIGNATURE_VALUE        0xA1
#define UX_HOST_CLASS_ASIX_INTERRUPT_SIGNATURE_OFFSET       0x00
#define UX_HOST_CLASS_ASIX_INTERRUPT_STATE_OFFSET           0x02
#define UX_HOST_CLASS_ASIX_INTERRUPT_STATE_PPLS             0x01
#define UX_HOST_CLASS_ASIX_INTERRUPT_STATE_SPLS             0x02
#define UX_HOST_CLASS_ASIX_INTERRUPT_STATE_FLE              0x04
#define UX_HOST_CLASS_ASIX_INTERRUPT_STATE_MDINT            0x08
#define UX_HOST_CLASS_ASIX_INTERRUPT_PHY_REG_VALUE_OFFSET   0x05

/* Define  ASIX Class PHY ID Packet format.  */

#define UX_HOST_CLASS_ASIX_PHY_ID_SECONDARY                 0x00
#define UX_HOST_CLASS_ASIX_PHY_ID_PRIMARY                   0x01
#define UX_HOST_CLASS_ASIX_PHY_ID_MASK                      0x01f
#define UX_HOST_CLASS_ASIX_PHY_TYPE_SHIFT                   0x05
#define UX_HOST_CLASS_ASIX_PHY_TYPE_MASK                    0x07

/* Define  ASIX Class GPIO Register.  */

#define UX_HOST_CLASS_ASIX_GPIO_GPO0EN                      0x01
#define UX_HOST_CLASS_ASIX_GPIO_GPO_0                       0x02
#define UX_HOST_CLASS_ASIX_GPIO_GPO1EN                      0x04
#define UX_HOST_CLASS_ASIX_GPIO_GPO_1                       0x08
#define UX_HOST_CLASS_ASIX_GPIO_GPO2EN                      0x10
#define UX_HOST_CLASS_ASIX_GPIO_GPO_2                       0x20
#define UX_HOST_CLASS_ASIX_GPIO_RSE                         0x80

/* Define  ASIX Class Software reset Register.  */

#define UX_HOST_CLASS_ASIX_SW_RESET_RR                      0x01
#define UX_HOST_CLASS_ASIX_SW_RESET_RT                      0x02
#define UX_HOST_CLASS_ASIX_SW_RESET_PRTE                    0x04
#define UX_HOST_CLASS_ASIX_SW_RESET_PRL                     0x08
#define UX_HOST_CLASS_ASIX_SW_RESET_BZ                      0x10
#define UX_HOST_CLASS_ASIX_SW_RESET_IPRL                    0x20
#define UX_HOST_CLASS_ASIX_SW_RESET_IPPD                    0x40

/* Define  ASIX Class Receive Control Register.  */

#define UX_HOST_CLASS_ASIX_RXCR_PRO                         0x0001
#define UX_HOST_CLASS_ASIX_RXCR_AMALL                       0x0002
#define UX_HOST_CLASS_ASIX_RXCR_SEP                         0x0004
#define UX_HOST_CLASS_ASIX_RXCR_AB                          0x0008
#define UX_HOST_CLASS_ASIX_RXCR_AM                          0x0010
#define UX_HOST_CLASS_ASIX_RXCR_AP                          0x0020
#define UX_HOST_CLASS_ASIX_RXCR_SO                          0x0080

/* 88772.  */
#define UX_HOST_CLASS_ASIX_RXCR_MFB_2048                    0x0000
#define UX_HOST_CLASS_ASIX_RXCR_MFB_4096                    0x0100
#define UX_HOST_CLASS_ASIX_RXCR_MFB_8192                    0x0200
#define UX_HOST_CLASS_ASIX_RXCR_MFB_16384                   0x0300 /* Default.  */

/* 88772B.  */
#define UX_HOST_CLASS_ASIX_RXCR_RH1M                        0x0100 /* Default 1.  */
#define UX_HOST_CLASS_ASIX_RXCR_RH2M                        0x0200 /* Default 0.  */
#define UX_HOST_CLASS_ASIX_RXCR_RH3M                        0x0400 /* Default 0.  */


/* Define  ASIX Class packet equivalences.  */

#define UX_HOST_CLASS_ASIX_PACKET_SIZE                      128
#define UX_HOST_CLASS_ASIX_NODE_ID_LENGTH                   6  

#define UX_HOST_CLASS_ASIX_RX_PACKET_LENGTH_MASK            0xFFF

/* Define  ASIX PHY registers description. */

#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR                     0x00
#define UX_HOST_CLASS_ASIX_PHY_REG_BMSR                     0x01
#define UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR1                  0x02
#define UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR2                  0x03
#define UX_HOST_CLASS_ASIX_PHY_REG_ANAR                     0x04
#define UX_HOST_CLASS_ASIX_PHY_REG_ANLPAR                   0x05
#define UX_HOST_CLASS_ASIX_PHY_REG_ANER                     0x06

/* Define  ASIX PHY BMCR registers description. */

#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR_RESET               0x8000
#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR_LOOPBACK_ENABLED    0x4000
#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR_SPEED_100MBS        0x2000
#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR_SPEED_10MBS         0x0000
#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR_AUTO_NEGOTIATION    0x1000
#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR_POWER_DOWN          0x0800
#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR_ISOLATE             0x0400
#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR_RESTART_NEG         0x0200
#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR_DUPLEX_MODE         0x0100
#define UX_HOST_CLASS_ASIX_PHY_REG_BMCR_COLLISION_TEST      0x0080


/* Define  ASIX PHY PHYIDR1 register description. */

#define UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR1_MDL_REV_SHIFT    0x00
#define UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR1_MDL_REV_MASK     0x0f
#define UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR1_VNDR_REV_SHIFT   0x04
#define UX_HOST_CLASS_ASIX_PHY_REG_PHYIDR1_VNDR_REV_MASK    0x2f


/* Define  ASIX PHY ANAR register description. */

#define UX_HOST_CLASS_ASIX_PHY_REG_ANAR_DEFAULT_SELECTOR    0x0001
#define UX_HOST_CLASS_ASIX_PHY_REG_ANAR_10_HD               0x0020
#define UX_HOST_CLASS_ASIX_PHY_REG_ANAR_10_FD               0x0040
#define UX_HOST_CLASS_ASIX_PHY_REG_ANAR_TX_HD               0x0080
#define UX_HOST_CLASS_ASIX_PHY_REG_ANAR_TX_FD               0x0100
#define UX_HOST_CLASS_ASIX_PHY_REG_ANAR_T4                  0x0200
#define UX_HOST_CLASS_ASIX_PHY_REG_ANAR_PAUSE               0x0400
#define UX_HOST_CLASS_ASIX_PHY_REG_ANAR_RF                  0x1000
#define UX_HOST_CLASS_ASIX_PHY_REG_ANAR_ACK                 0x2000

/* Define  ASIX MEDIUM register description. */

#define UX_HOST_CLASS_ASIX_MEDIUM_FD                        0x0002
#define UX_HOST_CLASS_ASIX_MEDIUM_BIT2                      0x0004
#define UX_HOST_CLASS_ASIX_MEDIUM_BIT3                      0x0000
#define UX_HOST_CLASS_ASIX_MEDIUM_RFC_ENABLED               0x0010
#define UX_HOST_CLASS_ASIX_MEDIUM_TFC_ENABLED               0x0020

#define UX_HOST_CLASS_ASIX_MEDIUM_RE_ENABLED                0x0100
#define UX_HOST_CLASS_ASIX_MEDIUM_PS                        0x0200
#define UX_HOST_CLASS_ASIX_MEDIUM_SBP                       0x0800
#define UX_HOST_CLASS_ASIX_MEDIUM_SM                        0x1000

/* Define  ASIX IPG default values register description. */

#define UX_HOST_CLASS_ASIX_PPG0_IPG1                        0x0C15
#define UX_HOST_CLASS_ASIX_PPG2                             0x000E

/* Define  ASIX Reception States. */

#define UX_HOST_CLASS_ASIX_RECEPTION_STATE_STOPPED          0
#define UX_HOST_CLASS_ASIX_RECEPTION_STATE_STARTED          1
#define UX_HOST_CLASS_ASIX_RECEPTION_STATE_IN_TRANSFER      2


/* Define  ASIX Class instance structure.  */

typedef struct UX_HOST_CLASS_ASIX_STRUCT
{
    struct UX_HOST_CLASS_ASIX_STRUCT  
                    *ux_host_class_asix_next_instance;
    UX_HOST_CLASS   *ux_host_class_asix_class;
    UX_DEVICE       *ux_host_class_asix_device;
    UX_ENDPOINT     *ux_host_class_asix_bulk_in_endpoint;
    UX_ENDPOINT     *ux_host_class_asix_bulk_out_endpoint;
    UX_ENDPOINT     *ux_host_class_asix_interrupt_endpoint;
    UX_INTERFACE    *ux_host_class_asix_interface;
    UINT            ux_host_class_asix_instance_status;
    UINT            ux_host_class_asix_state;

    UX_SEMAPHORE    ux_host_class_asix_semaphore;
    UX_SEMAPHORE    ux_host_class_asix_interrupt_notification_semaphore;
    UX_THREAD       ux_host_class_asix_thread;

    UCHAR           *ux_host_class_asix_thread_stack;
    ULONG           ux_host_class_asix_notification_count;
    ULONG           ux_host_class_asix_primary_phy_id;
    ULONG           ux_host_class_asix_primary_phy_type;
    ULONG           ux_host_class_asix_secondary_phy_id;
    ULONG           ux_host_class_asix_secondary_phy_type;
    ULONG           ux_host_class_asix_model_revision_number;
    ULONG           ux_host_class_asix_vendor_model_number;
    ULONG           ux_host_class_asix_speed_selected;
    ULONG           ux_host_class_asix_device_state;
    ULONG           ux_host_class_asix_link_state;

    NX_PACKET       *ux_host_class_asix_xmit_queue;
#ifdef UX_HOST_CLASS_ASIX_PACKET_CHAIN_SUPPORT
    UCHAR           *ux_host_class_asix_xmit_buffer;
#endif
    NX_PACKET       *ux_host_class_asix_receive_queue;
    UCHAR           *ux_host_class_asix_receive_buffer;
    NX_PACKET_POOL  *ux_host_class_asix_packet_pool;
    ULONG           ux_host_class_asix_packet_available_min;

    UCHAR           ux_host_class_asix_node_id[UX_HOST_CLASS_ASIX_NODE_ID_LENGTH];
    VOID            (*ux_host_class_asix_device_status_change_callback)(struct UX_HOST_CLASS_ASIX_STRUCT *asix, 
                                                                ULONG  device_state);
    VOID            *ux_host_class_asix_network_handle;
    
} UX_HOST_CLASS_ASIX;


/* Define  ASIX reception structure. */

typedef struct UX_HOST_CLASS_ASIX_RECEPTION_STRUCT
{

    ULONG           ux_host_class_asix_reception_state;
    ULONG           ux_host_class_asix_reception_block_size;
    UCHAR           *ux_host_class_asix_reception_data_buffer;
    ULONG           ux_host_class_asix_reception_data_buffer_size;
    UCHAR           *ux_host_class_asix_reception_data_head;
    UCHAR           *ux_host_class_asix_reception_data_tail;
    VOID            (*ux_host_class_asix_reception_callback)(struct UX_HOST_CLASS_ASIX_STRUCT *asix, 
                                                                UINT  status,
                                                                UCHAR *reception_buffer, 
                                                                ULONG reception_size);

} UX_HOST_CLASS_ASIX_RECEPTION;

/* Define Asix Class function prototypes.  */

UINT  _ux_host_class_asix_activate(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_asix_configure(UX_HOST_CLASS_ASIX *asix);
UINT  _ux_host_class_asix_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_asix_endpoints_get(UX_HOST_CLASS_ASIX *asix);
UINT  _ux_host_class_asix_entry(UX_HOST_CLASS_COMMAND *command);
UINT  _ux_host_class_asix_read (UX_HOST_CLASS_ASIX *asix, UCHAR *data_pointer, 
                                  ULONG requested_length, ULONG *actual_length);
UINT  _ux_host_class_asix_write(VOID *asix_class, NX_PACKET *packet);
VOID  _ux_host_class_asix_interrupt_notification(UX_TRANSFER *transfer_request);
VOID  _ux_host_class_asix_reception_callback (UX_TRANSFER *transfer_request);
VOID  _ux_host_class_asix_thread(ULONG parameter);
VOID  _ux_host_class_asix_transmission_callback (UX_TRANSFER *transfer_request);
UINT  _ux_host_class_asix_setup(UX_HOST_CLASS_ASIX *asix);
                                    
/* Define Asix Class API prototypes.  */

#define ux_host_class_asix_entry        _ux_host_class_asix_entry
#define ux_host_class_asix_read         _ux_host_class_asix_read
#define ux_host_class_asix_write        _ux_host_class_asix_write

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif
