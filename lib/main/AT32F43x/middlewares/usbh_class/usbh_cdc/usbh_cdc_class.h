/**
  **************************************************************************
  * @file     usbh_cdc_class.h
  * @brief    usb host cdc class header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBH_MSC_CLASS_H
#define __USBH_MSC_CLASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbh_core.h"
#include "usb_conf.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @addtogroup USBH_cdc_class
  * @{
  */

/** @defgroup USBH_cdc_class_definition
  * @{
  */

/*Communication Class codes*/
#define USB_CDC_CLASS                                           0x02
#define COMMUNICATION_INTERFACE_CLASS_CODE                      0x02

/*Data Interface Class Codes*/
#define DATA_INTERFACE_CLASS_CODE                               0x0A

/*Communication sub class codes*/
#define RESERVED                                                0x00
#define DIRECT_LINE_CONTROL_MODEL                               0x01
#define ABSTRACT_CONTROL_MODEL                                  0x02
#define TELEPHONE_CONTROL_MODEL                                 0x03
#define MULTICHANNEL_CONTROL_MODEL                              0x04   
#define CAPI_CONTROL_MODEL                                      0x05
#define ETHERNET_NETWORKING_CONTROL_MODEL                       0x06
#define ATM_NETWORKING_CONTROL_MODEL                            0x07


/*Communication Interface Class Control Protocol Codes*/
#define NO_CLASS_SPECIFIC_PROTOCOL_CODE                         0x00
#define COMMON_AT_COMMAND                                       0x01
#define VENDOR_SPECIFIC                                         0xFF


#define CS_INTERFACE                                            0x24
#define CDC_PAGE_SIZE_64                                        0x40

/*Class-Specific Request Codes*/
#define CDC_SEND_ENCAPSULATED_COMMAND                           0x00
#define CDC_GET_ENCAPSULATED_RESPONSE                           0x01
#define CDC_SET_COMM_FEATURE                                    0x02
#define CDC_GET_COMM_FEATURE                                    0x03
#define CDC_CLEAR_COMM_FEATURE                                  0x04

#define CDC_SET_AUX_LINE_STATE                                  0x10
#define CDC_SET_HOOK_STATE                                      0x11
#define CDC_PULSE_SETUP                                         0x12
#define CDC_SEND_PULSE                                          0x13
#define CDC_SET_PULSE_TIME                                      0x14
#define CDC_RING_AUX_JACK                                       0x15

#define CDC_SET_LINE_CODING                                     0x20
#define CDC_GET_LINE_CODING                                     0x21
#define CDC_SET_CONTROL_LINE_STATE                              0x22
#define CDC_SEND_BREAK                                          0x23

#define CDC_SET_RINGER_PARMS                                    0x30
#define CDC_GET_RINGER_PARMS                                    0x31
#define CDC_SET_OPERATION_PARMS                                 0x32
#define CDC_GET_OPERATION_PARMS                                 0x33  
#define CDC_SET_LINE_PARMS                                      0x34
#define CDC_GET_LINE_PARMS                                      0x35
#define CDC_DIAL_DIGITS                                         0x36
#define CDC_SET_UNIT_PARAMETER                                  0x37  
#define CDC_GET_UNIT_PARAMETER                                  0x38
#define CDC_CLEAR_UNIT_PARAMETER                                0x39
#define CDC_GET_PROFILE                                         0x3A

#define CDC_SET_ETHERNET_MULTICAST_FILTERS                      0x40
#define CDC_SET_ETHERNET_POWER_MANAGEMENT_PATTERN FILTER        0x41
#define CDC_GET_ETHERNET_POWER_MANAGEMENT_PATTERN FILTER        0x42
#define CDC_SET_ETHERNET_PACKET_FILTER                          0x43
#define CDC_GET_ETHERNET_STATISTIC                              0x44

#define CDC_SET_ATM_DATA_FORMAT                                 0x50  
#define CDC_GET_ATM_DEVICE_STATISTICS                           0x51
#define CDC_SET_ATM_DEFAULT_VC                                  0x52
#define CDC_GET_ATM_VC_STATISTICS                               0x53


/* wValue for SetControlLineState*/
#define CDC_ACTIVATE_CARRIER_SIGNAL_RTS                         0x0002
#define CDC_DEACTIVATE_CARRIER_SIGNAL_RTS                       0x0000
#define CDC_ACTIVATE_SIGNAL_DTR                                 0x0001
#define CDC_DEACTIVATE_SIGNAL_DTR                               0x0000

#define LINE_CODING_STRUCTURE_SIZE                              0x07

/* states for cdc state machine */
typedef enum
{
  CDC_IDLE              = 0x0,
  CDC_SEND_DATA         = 0x1,
  CDC_SEND_DATA_WAIT    = 0x2,
  CDC_RECEIVE_DATA      = 0x3,
  CDC_RECEIVE_DATA_WAIT = 0x4,  
} cdc_data_state_type;

typedef enum
{
  CDC_IDLE_STATE                 = 0x0,
  CDC_SET_LINE_CODING_STATE      = 0x1,  
  CDC_GET_LAST_LINE_CODING_STATE = 0x2,    
  CDC_TRANSFER_DATA              = 0x3, 
  CDC_ERROR_STATE                = 0x4,  
} cdc_state_type;


/*line coding structure*/
typedef union _cdc_line_coding_structure
{
  uint8_t array[LINE_CODING_STRUCTURE_SIZE];
  
  struct
  {
    uint32_t             data_baudrate; /*data terminal rate, in bits per second*/
    uint8_t              char_format;   /* Stop bits
                                           0 - 1 Stop bit
                                           1 - 1.5 Stop bits
                                           2 - 2 Stop bits*/
    uint8_t              parity_type;   /* parity
                                           0 - none
                                           1 - odd
                                           2 - even
                                           3 - mark
                                           4 - space*/
    uint8_t              data_bits;     /* data bits (5, 6, 7, 8 or 16). */
  }line_coding_b;
} cdc_line_coding_type;



/* header functional descriptor */
typedef struct _functional_descriptor_header
{
  uint8_t     bfunctionlength;    /* size of this descriptor. */
  uint8_t     bdescriptortype;    /* cs_interface (0x24) */
  uint8_t     bdescriptorsubtype; /* header functional descriptor subtype as */
  uint16_t    bcdcdc;             /* usb class definitions for communication
                                     devices specification release number in
                                     binary-coded decimal. */
} cdc_headerfuncdesc_type;

/* call management functional descriptor */
typedef struct _callmgmt_functional_descriptor
{
  uint8_t    blength;            /* size of this functional descriptor, in bytes */
  uint8_t    bdescriptortype;    /* cs_interface (0x24) */
  uint8_t    bdescriptorsubtype; /* call management functional descriptor subtype */
  uint8_t    bmcapabilities;     /* bmcapabilities: d0+d1 */
  uint8_t    bdatainterface;     /* bdatainterface: 1 */
} cdc_callmgmtfuncdesc_type;

/* abstract control management functional descriptor */
typedef struct _abstractcntrlmgmt_functional_descriptor
{
  uint8_t    blength;            /* size of this functional descriptor, in bytes */
  uint8_t    bdescriptortype;    /* cs_interface (0x24) */
  uint8_t    bdescriptorsubtype; /* abstract control management functional
                                    descriptor subtype */
  uint8_t    bmcapabilities;     /* the capabilities that this configuration supports */
} cdc_abstcntrlmgmtfuncdesc_type;

/* union functional descriptor */
typedef struct _union_functional_descriptor
{
  uint8_t    blength;            /* size of this functional descriptor, in bytes */
  uint8_t    bdescriptortype;    /* cs_interface (0x24) */
  uint8_t    bdescriptorsubtype; /* union functional descriptor subtype */
  uint8_t    bmasterinterface;   /* the interface number of the communication or
                                    data class interface */
  uint8_t    bslaveinterface0;   /* interface number of first slave */
} cdc_unionfuncdesc_type;


typedef struct _usbh_cdcinterfacedesc
{
  cdc_headerfuncdesc_type           cdc_headerfuncdesc;
  cdc_callmgmtfuncdesc_type         cdc_callmgmtfuncdesc;
  cdc_abstcntrlmgmtfuncdesc_type    cdc_abstcntrlmgmtfuncdesc;
  cdc_unionfuncdesc_type            cdc_unionfuncdesc;  
} cdc_interfacedesc_type;


/* structure for cdc process */
typedef struct
{
  uint8_t              notif_channel; 
  uint8_t              notif_endpoint;
  uint8_t              buff[8];
  uint16_t             notif_endpoint_size;
} cdc_common_interface_type;

typedef struct
{
  uint8_t              in_channel; 
  uint8_t              out_channel;
  uint8_t              out_endpoint;
  uint8_t              in_endpoint;
  uint8_t              buff[8];
  uint16_t             out_endpoint_size;
  uint16_t             in_endpoint_size;  
} cdc_data_interface_type;

/**
  * @brief  usb cdc struct
  */
typedef struct
{
  cdc_common_interface_type       common_interface;
  cdc_data_interface_type         data_interface;  
  cdc_interfacedesc_type          cdc_desc;
  cdc_line_coding_type            linecoding;
  cdc_line_coding_type            *puserlinecoding;  
  cdc_state_type                  state;
  cdc_data_state_type             data_tx_state;
  cdc_data_state_type             data_rx_state;
  
  uint8_t                         *rx_data;
  uint8_t                         *tx_data;
  uint32_t                        rx_len;
  uint32_t                        tx_len;
}usbh_cdc_type;

extern usbh_class_handler_type uhost_cdc_class_handler;
extern usbh_cdc_type usbh_cdc;
void cdc_start_transmission(usbh_core_type *phost, uint8_t *data, uint32_t len);
void cdc_start_reception(usbh_core_type *uhost, uint8_t *data, uint32_t len);
__weak void cdc_transmit_complete(usbh_core_type *uhost);
__weak void cdc_receive_complete(usbh_core_type *uhost);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif
