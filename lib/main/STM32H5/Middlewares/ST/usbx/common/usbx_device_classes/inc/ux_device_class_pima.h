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
/**   PIMA Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_device_class_pima.h                              PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX PIMA class.                                                    */ 
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            updated definitions,        */
/*                                            improved internal function, */
/*                                            added cancel callback,      */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DEVICE_CLASS_PIMA_H
#define UX_DEVICE_CLASS_PIMA_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  

/* Define PIMA Class constants.  */

#define UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH                                 UX_SLAVE_REQUEST_DATA_MAX_LENGTH
#define UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD                                            (UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH - UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE)
#define UX_DEVICE_CLASS_PIMA_OBJECT_INFO_BUFFER_SIZE                                (UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD)
#define UX_DEVICE_CLASS_PIMA_DEVICE_INFO_BUFFER_SIZE                                (UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD)
#define UX_DEVICE_CLASS_PIMA_STORAGE_INFO_BUFFER_SIZE                               (UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD)
#define UX_DEVICE_CLASS_PIMA_ARRAY_BUFFER_SIZE                                      (UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD)
#define UX_DEVICE_CLASS_PIMA_DEVICE_PROP_VALUE_BUFFER_SIZE                          (UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD)
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_VALUE_BUFFER_SIZE                          (UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD)

#define UX_DEVICE_CLASS_PIMA_CLASS_TRANSFER_TIMEOUT                                 300000
#define UX_DEVICE_CLASS_PIMA_CLASS                                                  0x06
#define UX_DEVICE_CLASS_PIMA_SUBCLASS                                               0X01
#define UX_DEVICE_CLASS_PIMA_PROTOCOL                                               0X01
#define UX_DEVICE_CLASS_PIMA_MAGIC_NUMBER                                           0x50494D41
#define UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH                                     256
#define UX_DEVICE_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH                            64 
#define UX_DEVICE_CLASS_PIMA_MAX_EVENTS_QUEUE                                       16
#define UX_DEVICE_CLASS_PIMA_MAX_STORAGE_IDS                                        1
#define UX_DEVICE_CLASS_PIMA_ARRAY_MAX_LENGTH                                       256
#define UX_DEVICE_CLASS_PIMA_DEVICE_PROPERTIES_ARRAY_MAX_ITEMS                      32
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROPERTIES_ARRAY_MAX_ITEMS                      128
#define UX_DEVICE_CLASS_PIMA_PROP_VALUE_SIZE                                        256
#define UX_DEVICE_CLASS_PIMA_MICROSOFT_VENDOR_COMMAND_CODE                          0x54
                                                                                    
/* Define PIMA versions.  */                                                        
#define UX_DEVICE_CLASS_PIMA_STANDARD_VERSION                                       100
#define UX_DEVICE_CLASS_PIMA_VENDOR_EXTENSION_ID                                    6
#define UX_DEVICE_CLASS_PIMA_EXTENSION_VERSION                                      100
#define UX_DEVICE_CLASS_PIMA_STANDARD_MODE                                          0
                                                                                    

/* Define PIMA phases.  */

#define UX_DEVICE_CLASS_PIMA_PHASE_IDLE                                             0
#define UX_DEVICE_CLASS_PIMA_PHASE_COMMAND                                          1
#define UX_DEVICE_CLASS_PIMA_PHASE_RESPONSE                                         2
#define UX_DEVICE_CLASS_PIMA_PHASE_DATA_IN                                          3
#define UX_DEVICE_CLASS_PIMA_PHASE_DATA_OUT                                         4

/* Define PIMA data phases.  */                                                        
                                                                                    
#define UX_DEVICE_CLASS_PIMA_DATA_PHASE_NONE                                        0
#define UX_DEVICE_CLASS_PIMA_DATA_PHASE_IN                                          1
#define UX_DEVICE_CLASS_PIMA_DATA_PHASE_OUT                                         2
                                                                                    
/* Define PIMA session states.  */                                                    
                                                                                    
#define UX_DEVICE_CLASS_PIMA_SESSION_STATE_CLOSED                                   0
#define UX_DEVICE_CLASS_PIMA_SESSION_STATE_OPENED                                   1
                                                                                    
/* Define PIMA object and thumb states.  */                                            
                                                                                    
#define UX_DEVICE_CLASS_PIMA_OBJECT_STATE_CLOSED                                    0
#define UX_DEVICE_CLASS_PIMA_OBJECT_STATE_OPENED                                    1
                                                                                    
/* Define PIMA object and thumb transfer status.  */                                
                                                                                    
#define UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_STATUS_INACTIVE                        0
#define UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_STATUS_ACTIVE                          1
#define UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_STATUS_COMPLETED                       2
#define UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_STATUS_ABORTED                         3
                                                                                    
/* Define PIMA object and thumb transfer phase.  */                                    
                                                                                    
#define UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_PHASE_ACTIVE                           0
#define UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_PHASE_COMPLETED                        1
#define UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_PHASE_COMPLETED_ERROR                  2
                                                                                    
                                                                                    
/* Define PIMA Cancel Request equivalences.  */                                        
                                                                                    
#define UX_DEVICE_CLASS_PIMA_REQUEST_CANCEL_COMMAND                                 0x64
#define UX_DEVICE_CLASS_PIMA_REQUEST_CANCEL_DATA_LENGTH                             0x06
#define UX_DEVICE_CLASS_PIMA_REQUEST_CANCEL_CODE                                    0x04001
#define UX_DEVICE_CLASS_PIMA_REQUEST_CANCEL_OFFSET_CODE                             0x00
#define UX_DEVICE_CLASS_PIMA_REQUEST_CANCEL_OFFSET_TRANSACTION_ID                   0x02
                                                                                    
/* Define PIMA Reset Request equivalences.  */                                        
                                                                                    
#define UX_DEVICE_CLASS_PIMA_REQUEST_RESET_DEVICE                                   0x66
                                                                                    
/* Define PIMA Status Request equivalences.  */                                        
                                                                                    
#define UX_DEVICE_CLASS_PIMA_REQUEST_STATUS_COMMAND                                 0x67
#define UX_DEVICE_CLASS_PIMA_REQUEST_STATUS_DATA_LENGTH                             0x40
#define UX_DEVICE_CLASS_PIMA_REQUEST_STATUS_OFFSET_LENGTH                           0x00
#define UX_DEVICE_CLASS_PIMA_REQUEST_STATUS_OFFSET_CODE                             0x02
#define UX_DEVICE_CLASS_PIMA_REQUEST_STATUS_COMMAND_COUNTER                         16
#define UX_DEVICE_CLASS_PIMA_REQUEST_STATUS_COMMAND_DELAY                           (1 * UX_PERIODIC_RATE) 
                                                                                    
/* Define PIMA command container type.  */                                            
                                                                                    
#define UX_DEVICE_CLASS_PIMA_CT_UNDEFINED                                           0x00
#define UX_DEVICE_CLASS_PIMA_CT_COMMAND_BLOCK                                       0x01
#define UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK                                          0x02
#define UX_DEVICE_CLASS_PIMA_CT_RESPONSE_BLOCK                                      0x03
#define UX_DEVICE_CLASS_PIMA_CT_EVENT_BLOCK                                         0x04
                                                                                    
/* Define PIMA Extended Event Data Request payload Format.  */                        
                                                                                    
#define UX_DEVICE_CLASS_PIMA_EEDR_EVENT_CODE                                        0x00
#define UX_DEVICE_CLASS_PIMA_EEDR_TRANSACTION_ID                                    0x02
#define UX_DEVICE_CLASS_PIMA_EEDR_NUMBER_PARAMETERS                                 0x06
#define UX_DEVICE_CLASS_PIMA_EEDR_SIZE_PARAMETER                                    0x08
                                                                                    
/* Define PIMA Device Status Data Format.  */                                        
                                                                                    
#define UX_DEVICE_CLASS_PIMA_DSD_LENGTH                                             0x00
#define UX_DEVICE_CLASS_PIMA_DSD_CODE                                               0x02
#define UX_DEVICE_CLASS_PIMA_DSD_PARAMETER                                          0x04
                                                                                    
/* Define PIMA Command Header Format.  */                                            
                                                                                    
#define UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_LENGTH                                  0x00
#define UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_TYPE                                    0x04
#define UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_CODE                                    0x06
#define UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_TRANSACTION_ID                          0x08
#define UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_PARAMETER_1                             0x0C
#define UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_PARAMETER_2                             0x10
#define UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_PARAMETER_3                             0x14
#define UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_PARAMETER_4                             0x18
#define UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_PARAMETER_5                             0x1C
                                                                                    
#define UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_SIZE                                    0x0C
#define UX_DEVICE_CLASS_PIMA_CONTAINER_SIZE                                         0x40 
#define UX_DEVICE_CLASS_PIMA_ALL_HEADER_SIZE                                        0x20
                                                                                    
/* Define PIMA Data Header Format.  */                                                
                                                                                    
#define UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH                                     0x00
#define UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE                                       0x04
#define UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE                                       0x06
#define UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID                             0x08
#define UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE                                       0x0C
                                                                                    
                                                                                    
/* Define PIMA Response Header Format.  */                                            
                                                                                    
#define UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_LENGTH                                 0x00
#define UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_TYPE                                   0x04
#define UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_CODE                                   0x06
#define UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_TRANSACTION_ID                         0x08
#define UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_PARAMETERS                             0x0C
#define UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_SIZE                                   0x0C
                                                                                    
/* Define PIMA Asynchronous Event Interrupt Data Format.  */                        
                                                                                    
#define UX_DEVICE_CLASS_PIMA_AEI_DATA_LENGTH                                        0x00
#define UX_DEVICE_CLASS_PIMA_AEI_TYPE                                               0x04
#define UX_DEVICE_CLASS_PIMA_AEI_EVENT_CODE                                         0x06
#define UX_DEVICE_CLASS_PIMA_AEI_TRANSACTION_ID                                     0x08
#define UX_DEVICE_CLASS_PIMA_AEI_PARAMETER_1                                        0x0C
#define UX_DEVICE_CLASS_PIMA_AEI_PARAMETER_2                                        0x10
#define UX_DEVICE_CLASS_PIMA_AEI_PARAMETER_3                                        0x14
#define UX_DEVICE_CLASS_PIMA_AEI_MAX_LENGTH                                         0x18
                                                                                    
/* Define PIMA Operation Commands.  */                                                
                                                                                    
#define UX_DEVICE_CLASS_PIMA_OC_UNDEFINED                                           0x1000
#define UX_DEVICE_CLASS_PIMA_OC_GET_DEVICE_INFO                                     0x1001
#define UX_DEVICE_CLASS_PIMA_OC_OPEN_SESSION                                        0x1002
#define UX_DEVICE_CLASS_PIMA_OC_CLOSE_SESSION                                       0x1003
#define UX_DEVICE_CLASS_PIMA_OC_GET_STORAGE_IDS                                     0x1004
#define UX_DEVICE_CLASS_PIMA_OC_GET_STORAGE_INFO                                    0x1005
#define UX_DEVICE_CLASS_PIMA_OC_GET_NUM_OBJECTS                                     0x1006
#define UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_HANDLES                                  0x1007
#define UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_INFO                                     0x1008
#define UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT                                          0x1009
#define UX_DEVICE_CLASS_PIMA_OC_GET_THUMB                                           0x100A
#define UX_DEVICE_CLASS_PIMA_OC_DELETE_OBJECT                                       0x100B
#define UX_DEVICE_CLASS_PIMA_OC_SEND_OBJECT_INFO                                    0x100C
#define UX_DEVICE_CLASS_PIMA_OC_SEND_OBJECT                                         0x100D
#define UX_DEVICE_CLASS_PIMA_OC_INITIATE_CAPTURE                                    0x100E
#define UX_DEVICE_CLASS_PIMA_OC_FORMAT_STORE                                        0x100F
#define UX_DEVICE_CLASS_PIMA_OC_RESET_DEVICE                                        0x1010
#define UX_DEVICE_CLASS_PIMA_OC_SELF_TEST                                           0x1011
#define UX_DEVICE_CLASS_PIMA_OC_SET_OBJECT_PROTECTION                               0x1012  
#define UX_DEVICE_CLASS_PIMA_OC_POWER_DOWN                                          0x1013
#define UX_DEVICE_CLASS_PIMA_OC_GET_DEVICE_PROP_DESC                                0x1014
#define UX_DEVICE_CLASS_PIMA_OC_GET_DEVICE_PROP_VALUE                               0x1015
#define UX_DEVICE_CLASS_PIMA_OC_SET_DEVICE_PROP_VALUE                               0x1016
#define UX_DEVICE_CLASS_PIMA_OC_RESET_DEVICE_PROP_VALUE                             0x1017
#define UX_DEVICE_CLASS_PIMA_OC_TERMINATE_OPEN_CAPTURE                              0x1018
#define UX_DEVICE_CLASS_PIMA_OC_MOVE_OBJECT                                         0x1019
#define UX_DEVICE_CLASS_PIMA_OC_COPY_OBJECT                                         0x101A
#define UX_DEVICE_CLASS_PIMA_OC_GET_PARTIAL_OBJECT                                  0x101B
#define UX_DEVICE_CLASS_PIMA_OC_INITIATE_OPEN_CAPTURE                               0x101C
#define UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROPS_SUPPORTED                          0x9801
#define UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROP_DESC                                0x9802
#define UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROP_VALUE                               0x9803
#define UX_DEVICE_CLASS_PIMA_OC_SET_OBJECT_PROP_VALUE                               0x9804
#define UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_REFERENCES                               0x9810
#define UX_DEVICE_CLASS_PIMA_OC_SET_OBJECT_REFERENCES                               0x9811

/* Define PIMA Response Codes.  */                                                    
                                                                                    
#define UX_DEVICE_CLASS_PIMA_RC_UNDEFINED                                           0x2000
#define UX_DEVICE_CLASS_PIMA_RC_OK                                                  0x2001
#define UX_DEVICE_CLASS_PIMA_RC_GENERAL_ERROR                                       0x2002
#define UX_DEVICE_CLASS_PIMA_RC_SESSION_NOT_OPEN                                    0x2003
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_TRANSACTION_ID                              0x2004
#define UX_DEVICE_CLASS_PIMA_RC_OPERATION_NOT_SUPPORTED                             0x2005
#define UX_DEVICE_CLASS_PIMA_RC_PARAMETER_NOT_SUPPORTED                             0x2006
#define UX_DEVICE_CLASS_PIMA_RC_INCOMPLETE_TRANSFER                                 0x2007
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_STORAGE_ID                                  0x2008
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_OBJECT_HANDLE                               0x2009
#define UX_DEVICE_CLASS_PIMA_RC_DEVICE_PROP_NOT_SUPPORTED                           0x200A
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_OBJECT_FORMAT_CODE                          0x200B
#define UX_DEVICE_CLASS_PIMA_RC_STORE_FULL                                          0x200C
#define UX_DEVICE_CLASS_PIMA_RC_OBJECT_WRITE_PROTECTED                              0x200D
#define UX_DEVICE_CLASS_PIMA_RC_STORE_READ_ONLY                                     0x200E
#define UX_DEVICE_CLASS_PIMA_RC_ACCESS_DENIED                                       0x200F
#define UX_DEVICE_CLASS_PIMA_RC_NO_THUMBNAIL_PRESENT                                0x2010
#define UX_DEVICE_CLASS_PIMA_RC_SELF_TEST_FAILED                                    0x2011
#define UX_DEVICE_CLASS_PIMA_RC_PARTIAL_DELETION                                    0x2012
#define UX_DEVICE_CLASS_PIMA_RC_STORE_NOT_AVAILABLE                                 0x2013
#define UX_DEVICE_CLASS_PIMA_RC_FORMAT_UNSUPPORTED                                  0x2014
#define UX_DEVICE_CLASS_PIMA_RC_NO_VALID_OBJECT_INFO                                0x2015
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_CODE_FORMAT                                 0x2016
#define UX_DEVICE_CLASS_PIMA_RC_UNKNOWN_VENDOR_CODE                                 0x2017
#define UX_DEVICE_CLASS_PIMA_RC_CAPTURE_ALREADY_TERMINATED                          0x2018
#define UX_DEVICE_CLASS_PIMA_RC_DEVICE_BUSY                                         0x2019
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_PARENT_OBJECT                               0x201A
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_DEVICE_PROP_FORMAT                          0x201B
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_DEVICE_PROP_VALUE                           0x201C
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_PARAMETER                                   0x201D
#define UX_DEVICE_CLASS_PIMA_RC_SESSION_ALREADY_OPENED                              0x201E
#define UX_DEVICE_CLASS_PIMA_RC_TRANSACTION_CANCELED                                0x201F
#define UX_DEVICE_CLASS_PIMA_RC_DESTINATION_UNSUPPORTED                             0x2020
#define UX_DEVICE_CLASS_PIMA_RC_OBJECT_ALREADY_OPENED                               0x2021
#define UX_DEVICE_CLASS_PIMA_RC_OBJECT_ALREADY_CLOSED                               0x2022
#define UX_DEVICE_CLASS_PIMA_RC_OBJECT_NOT_OPENED                                   0x2023
                                                                                    
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_OBJECT_PROP_CODE                            0xA801
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_OBJECT_PROP_FORMAT                          0xA802
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_OBJECT_PROP_VALUE                           0xA803
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_OBJECT_REFERENCE                            0xA804
#define UX_DEVICE_CLASS_PIMA_RC_INVALID_DATASET                                     0xA806
#define UX_DEVICE_CLASS_PIMA_RC_SPECIFICATION_BY_GROUP_UNSUPPORTED                  0xA807
#define UX_DEVICE_CLASS_PIMA_RC_SPECIFICATION_BY_DEPTH_UNSUPPORTED                  0xA808
#define UX_DEVICE_CLASS_PIMA_RC_OBJECT_TOO_LARGE                                    0xA809
#define UX_DEVICE_CLASS_PIMA_RC_OBJECT_PROP_NOT_SUPPORTED                           0xA80A
                                                                                    
/* Define PIMA Event Codes.  */                                                        
                                                                                    
#define UX_DEVICE_CLASS_PIMA_EC_UNDEFINED                                           0x4000
#define UX_DEVICE_CLASS_PIMA_EC_CANCEL_TRANSACTION                                  0x4001
#define UX_DEVICE_CLASS_PIMA_EC_OBJECT_ADDED                                        0x4002
#define UX_DEVICE_CLASS_PIMA_EC_OBJECT_REMOVED                                      0x4003
#define UX_DEVICE_CLASS_PIMA_EC_STORE_ADDED                                         0x4004
#define UX_DEVICE_CLASS_PIMA_EC_STORE_REMOVED                                       0x4005
#define UX_DEVICE_CLASS_PIMA_EC_DEVICE_PROP_CHANGED                                 0x4006
#define UX_DEVICE_CLASS_PIMA_EC_OBJECT_INFO_CHANGED                                 0x4007
#define UX_DEVICE_CLASS_PIMA_EC_DEVICE_INFO_CHANGED                                 0x4008
#define UX_DEVICE_CLASS_PIMA_EC_REQUEST_OBJECT_TRANSFER                             0x4009
#define UX_DEVICE_CLASS_PIMA_EC_STORE_FULL                                          0x400A
#define UX_DEVICE_CLASS_PIMA_EC_DEVICE_RESET                                        0x400B
#define UX_DEVICE_CLASS_PIMA_EC_STORAGE_INFO_CHANGED                                0x400C
#define UX_DEVICE_CLASS_PIMA_EC_CAPTURE_COMPLETE                                    0x400D
#define UX_DEVICE_CLASS_PIMA_EC_UNREPORTED_STATUS                                   0x400E
                                                                                    
/* Define PIMA Object Format Codes.  */                                                
                                                                                    
#define UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED                                          0x3000
#define UX_DEVICE_CLASS_PIMA_OFC_ASSOCIATION                                        0x3001
#define UX_DEVICE_CLASS_PIMA_OFC_SCRIPT                                             0x3002
#define UX_DEVICE_CLASS_PIMA_OFC_EXECUTABLE                                         0x3003
#define UX_DEVICE_CLASS_PIMA_OFC_TEXT                                               0x3004
#define UX_DEVICE_CLASS_PIMA_OFC_HTML                                               0x3005
#define UX_DEVICE_CLASS_PIMA_OFC_DPOF                                               0x3006
#define UX_DEVICE_CLASS_PIMA_OFC_AIFF                                               0x3007
#define UX_DEVICE_CLASS_PIMA_OFC_WAV                                                0x3008
#define UX_DEVICE_CLASS_PIMA_OFC_MP3                                                0x3009
#define UX_DEVICE_CLASS_PIMA_OFC_AVI                                                0x300A
#define UX_DEVICE_CLASS_PIMA_OFC_MPEG                                               0x300B
#define UX_DEVICE_CLASS_PIMA_OFC_ASF                                                0x300C
#define UX_DEVICE_CLASS_PIMA_OFC_DEFINED                                            0x3800
#define UX_DEVICE_CLASS_PIMA_OFC_EXIF_JPEG                                          0x3801
#define UX_DEVICE_CLASS_PIMA_OFC_TIFF_EP                                            0x3802
#define UX_DEVICE_CLASS_PIMA_OFC_FLASHPIX                                           0x3803
#define UX_DEVICE_CLASS_PIMA_OFC_BMP                                                0x3804
#define UX_DEVICE_CLASS_PIMA_OFC_CIFF                                               0x3805
#define UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_2                                        0x3806
#define UX_DEVICE_CLASS_PIMA_OFC_GIF                                                0x3807
#define UX_DEVICE_CLASS_PIMA_OFC_JFIF                                               0x3808
#define UX_DEVICE_CLASS_PIMA_OFC_CD                                                 0x3809
#define UX_DEVICE_CLASS_PIMA_OFC_PICT                                               0x380A
#define UX_DEVICE_CLASS_PIMA_OFC_PNG                                                0x380B
#define UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_3                                        0x380C
#define UX_DEVICE_CLASS_PIMA_OFC_TIFF                                               0x380D
#define UX_DEVICE_CLASS_PIMA_OFC_TIFF_IT                                            0x380E
#define UX_DEVICE_CLASS_PIMA_OFC_JP2                                                0x380F
#define UX_DEVICE_CLASS_PIMA_OFC_JPX                                                0x3810
#define UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_FIRMWARE                                 0xB802
#define UX_DEVICE_CLASS_PIMA_OFC_WINDOWS_IMAGE_FORMAT                               0xB881
#define UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_AUDIO                                    0xB900
#define UX_DEVICE_CLASS_PIMA_OFC_WMA                                                0xB901
#define UX_DEVICE_CLASS_PIMA_OFC_OGG                                                0xB902
#define UX_DEVICE_CLASS_PIMA_OFC_AAC                                                0xB903
#define UX_DEVICE_CLASS_PIMA_OFC_AUDIBLE                                            0xB904
#define UX_DEVICE_CLASS_PIMA_OFC_FLAC                                               0xB906
#define UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_VIDEO                                    0xB980
#define UX_DEVICE_CLASS_PIMA_OFC_WMV                                                0xB981
#define UX_DEVICE_CLASS_PIMA_OFC_MP4_CONTAINER                                      0xB982
#define UX_DEVICE_CLASS_PIMA_OFC_MP2                                                0xB983
#define UX_DEVICE_CLASS_PIMA_OFC_3GP_CONTAINER                                      0xB984
#define UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_COLLECTION                               0xBA00
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_MULTIMEDIA_ALBUM                          0xBA01
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_IMAGE_ALBUM                               0xBA02
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_AUDIO_ALBUM                               0xBA03
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_VIDEO_ALBUM                               0xBA04
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_AUDIO_AND_VIDEO_PLAYLIST                  0xBA05
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_CONTACT_GROUP                             0xBA06
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_MESSAGE_FOLDER                            0xBA07
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_CHAPTERED_PRODUCTION                      0xBA08
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_AUDIO_PLAYLIST                            0xBA09
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_VIDEO_PLAYLIST                            0xBA0A
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_MEDIACAST                                 0xBA0B
#define UX_DEVICE_CLASS_PIMA_OFC_WPL_PLAYLIST                                       0xBA10
#define UX_DEVICE_CLASS_PIMA_OFC_M3U_PLAYLIST                                       0xBA11
#define UX_DEVICE_CLASS_PIMA_OFC_MPL_PLAYLIST                                       0xBA12
#define UX_DEVICE_CLASS_PIMA_OFC_ASX_PLAYLIST                                       0xBA13
#define UX_DEVICE_CLASS_PIMA_OFC_PLS_PLAYLIST                                       0xBA14
#define UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_DOCUMENT                                 0xBA80
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_DOCUMENT                                  0xBA81
#define UX_DEVICE_CLASS_PIMA_OFC_XML_DOCUMENT                                       0xBA82
#define UX_DEVICE_CLASS_PIMA_OFC_MICROSOFT_WORD_DOCUMENT                            0xBA83
#define UX_DEVICE_CLASS_PIMA_OFC_MHT_COMPILED_HTML_DOCUMENT                         0xBA84
#define UX_DEVICE_CLASS_PIMA_OFC_MICROSOFT_EXCEL_SPREADSHEET                        0xBA85
#define UX_DEVICE_CLASS_PIMA_OFC_MICROSOFT_POWERPOINT_PRESENTATION                  0xBA86
#define UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_MESSAGE                                  0xBB00
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_MESSAGE                                   0xBB01
#define UX_DEVICE_CLASS_PIMA_OFC_UNDEFINED_CONTACT                                  0xBB80
#define UX_DEVICE_CLASS_PIMA_OFC_ABSTRACT_CONTACT                                   0xBB81
#define UX_DEVICE_CLASS_PIMA_OFC_VCARD2                                             0xBB82
                                                                                    
/* Define PIMA Device Format Codes.  */                                                
                                                                                    
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_UNDEFINED                                     0x5000 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_BATTERY_LEVEL                                 0x5001 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_FUNCTIONAL_MODE                               0x5002 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_IMAGE_SIZE                                    0x5003 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_COMPRESSION_SETTING                           0x5004 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_WHITE_BALANCE                                 0x5005 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_RGB_GAIN                                      0x5006 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_F_NUMBER                                      0x5007 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_FOCAL_LENGTH                                  0x5008 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_FOCUS_DISTANCE                                0x5009 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_FOCUS_MODE                                    0x500A 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_EXPOSURE_METERING_MODE                        0x500B 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_FLASH_MODE                                    0x500C 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_EXPOSURE_TIME                                 0x500D 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_EXPOSURE_PROGRAM_MODE                         0x500E 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_EXPOSURE_INDEX                                0x500F 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_EXPOSURE_BIAS_COMPENSATION                    0x5010 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_DATE_TIME                                     0x5011 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_CAPTURE_DELAY                                 0x5012 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_STILL_CAPTURE_MODE                            0x5013 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_CONTRAST                                      0x5014 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_SHARPNESS                                     0x5015 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_DIGITAL_ZOOM                                  0x5016 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_EFFECT_MODE                                   0x5017 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_BURST_NUMBER                                  0x5018 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_BURST_INTERVAL                                0x5019 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_TIME_LAPSE_NUMBER                             0x501A 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_TIME_LAPSE_INTERVAL                           0x501B 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_FOCUS_METERING_MODE                           0x501C 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_UPLOAD_URL                                    0x501D 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_ARTIST                                        0x501E 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_COPYRIGHT_INFO                                0x501F 
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_SYNCHRONIZATION_PARTNER                       0xD401
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_DEVICE_FRIENDLY_NAME                          0xD402
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_VOLUME                                        0xD403
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_SUPPORTED_FORMATS_ORDERED                     0xD404
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_DEVICE_ICON                                   0xD405
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_PLAYBACK_RATE                                 0xD410
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_PLAYBACK_OBJECT                               0xD411
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_PLAYBACK_CONTAINER                            0xD412
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_SESSION_INITIATOR_VERSION_INFO                0xD406
#define UX_DEVICE_CLASS_PIMA_DEV_PROP_PERCEIVED_DEVICE_TYPE                         0xD407

/* Define PIMA Object Format Codes.  */

#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_STORAGEID                                  0xDC01
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_OBJECT_FORMAT                              0xDC02
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PROTECTION_STATUS                          0xDC03
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_OBJECT_SIZE                                0xDC04
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ASSOCIATION_TYPE                           0xDC05
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ASSOCIATION_DESC                           0xDC06
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_OBJECT_FILE_NAME                           0xDC07
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_DATE_CREATED                               0xDC08
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_DATE_MODIFIED                              0xDC09
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_KEYWORDS                                   0xDC0A
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PARENT_OBJECT                              0xDC0B
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ALLOWED_FOLDER_CONTENTS                    0xDC0C
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_HIDDEN                                     0xDC0D
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SYSTEM_OBJECT                              0xDC0E
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PERSISTENT_UNIQUE_OBJECT_IDENTIFIER        0xDC41
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SYNCID                                     0xDC42
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PROPERTY_BAG                               0xDC43
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_NAME                                       0xDC44
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_CREATED_BY                                 0xDC45
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ARTIST                                     0xDC46
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_DATE_AUTHORED                              0xDC47
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_DESCRIPTION                                0xDC48
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_URL_REFERENCE                              0xDC49
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_LANGUAGE_LOCALE                            0xDC4A
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_COPYRIGHT_INFORMATION                      0xDC4B
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SOURCE                                     0xDC4C
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ORIGIN_LOCATION                            0xDC4D
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_DATE_ADDED                                 0xDC4E
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_NON_CONSUMABLE                             0xDC4F
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_CORRUPT_UNPLAYABLE                         0xDC50
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PRODUCER_SERIA_LNUMBER                     0xDC51
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_REPRESENTATIVE_SAMPLE_FORMAT               0xDC81
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_REPRESENTATIVE_SAMPLE_SIZE                 0xDC82
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_REPRESENTATIVE_SAMPLE_HEIGHT               0xDC83
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_REPRESENTATIVE_SAMPLE_WIDTH                0xDC84
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_REPRESENTATIVE_SAMPLE_DURATION             0xDC85
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_REPRESENTATIVE_SAMPLE_DATA                 0xDC86
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_WIDTH                                      0xDC87
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_HEIGHT                                     0xDC88
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_DURATION                                   0xDC89
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_RATING                                     0xDC8A
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_TRACK                                      0xDC8B
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_GENRE                                      0xDC8C
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_CREDITS                                    0xDC8D
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_LYRICS                                     0xDC8E
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SUBSCRIPTION_CONTENT_ID                    0xDC8F
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PRODUCED_BY                                0xDC90
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_USE_COUNT                                  0xDC91
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SKIP_COUNT                                 0xDC92
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_LAST_ACCESSED                              0xDC93
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PARENTAL_RATING                            0xDC94
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_META_GENRE                                 0xDC95
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_COMPOSER                                   0xDC96
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_EFFECTIVE_RATING                           0xDC97
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SUBTITLE                                   0xDC98
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ORIGINAL_RELEASE_DATE                      0xDC99
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ALBUM_NAME                                 0xDC9A
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ALBUM_ARTIST                               0xDC9B
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_MOOD                                       0xDC9C
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_DRM_STATUS                                 0xDC9D
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SUB_DESCRIPTION                            0xDC9E
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_IS_CROPPED                                 0xDCD1
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_IS_COLOUR_CORRECTED                        0xDCD2
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_IMAGE_BIT_DEPTH                            0xDCD3
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_FNUMBER                                    0xDCD4
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_EXPOSURE_TIME                              0xDCD5
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_EXPOSURE_INDEX                             0xDCD6
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_TOTAL_BITRATE                              0xDE91
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_BITRATE_TYPE                               0xDE92
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SAMPLE_RATE                                0xDE93
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_NUMBER_OF_CHANNELS                         0xDE94
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_AUDIO_BITDEPTH                             0xDE95
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SCAN_TYPE                                  0xDE97
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_AUDIO_WAVE_CODEC                           0xDE99
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_AUDIO_BITRATE                              0xDE9A
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_VIDEO_FOURCC_CODEC                         0xDE9B
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_VIDEO_BITRATE                              0xDE9C
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_FRAMES_PER_THOUSAND_SECONDS                0xDE9D
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_KEYFRAME_DISTANCE                          0xDE9E
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_BUFFER_SIZE                                0xDE9F
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ENCODING_QUALITY                           0xDEA0
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ENCODING_PROFILE                           0xDEA1
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_DISPLAY_NAME                               0xDCE0
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_BODY_TEXT                                  0xDCE1
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SUBJECT                                    0xDCE2
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PRIORITY                                   0xDCE3
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_GIVEN_NAME                                 0xDD00
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_MIDDLE_NAMES                               0xDD01
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_FAMILY_NAME                                0xDD02
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PREFIX                                     0xDD03
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_SUFFIX                                     0xDD04
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONETIC_GIVEN_NAME                        0xDD05
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONETIC_FAMILY_NAME                       0xDD06
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_EMAIL_PRIMARY                              0xDD07
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_EMAIL_PERSONAL_1                           0xDD08
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_EMAIL_PERSONAL_2                           0xDD09
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_EMAIL_BUSINESS_1                           0xDD0A
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_EMAIL_BUSINESS_2                           0xDD0B
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_EMAIL_OTHERS                               0xDD0C
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONE_NUMBER_PRIMARY                       0xDD0D
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONE_NUMBER_PERSONAL                      0xDD0E
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONE_NUMBER_PERSONAL_2                    0xDD0F
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONE_NUMBER_BUSINESS                      0xDD10
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONE_NUMBER_BUSINESS_2                    0xDD11
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONE_NUMBER_MOBILE                        0xDD12
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONE_NUMBER_MOBILE_2                      0xDD13
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_FAX_NUMBER_PRIMARY                         0xDD14
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_FAX_NUMBER_PERSONAL                        0xDD15
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_FAX_NUMBER_BUSINESS                        0xDD16
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PAGER_NUMBER                               0xDD17
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONE_NUMBER_OTHERS                        0xDD18
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PRIMARY_WEB_ADDRESS                        0xDD19
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PERSONAL_WEB_ADDRESS                       0xDD1A
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_BUSINESS_WEB_ADDRESS                       0xDD1B
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_INSTANT_MESSENGER_ADDRESS                  0xDD1C
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_INSTANT_MESSENGER_ADDRESS_2                0xDD1D
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_INSTANT_MESSENGER_ADDRESS_3                0xDD1E
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_PERSONAL_FULL               0xDD1F
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_PERSONAL_LINE_1             0xDD20
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_PERSONAL_LINE_2             0xDD21
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_PERSONAL_CITY               0xDD22
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_PERSONAL_REGION             0xDD23
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_PERSONAL_POSTAL_CODE        0xDD24
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_PERSONAL_COUNTRY            0xDD25
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_BUSINESS_FULL               0xDD26
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_BUSINESS_LINE_1             0xDD27
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_BUSINESS_LINE_2             0xDD28
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_BUSINESS_CITY               0xDD29
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_BUSINESS_REGION             0xDD2A
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_BUSINESS_POSTAL_CODE        0xDD2B
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_BUSINESS_COUNTRY            0xDD2C
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_OTHER_FULL                  0xDD2D
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_OTHER_LINE_1                0xDD2E
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_OTHER_LINE_2                0xDD2F
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_OTHER_CITY                  0xDD30
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_OTHER_REGION                0xDD31
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_OTHER_POSTAL_CODE           0xDD32
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_POSTAL_ADDRESS_OTHER_COUNTRY               0xDD33
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ORGANIZATION_NAME                          0xDD34
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_PHONETIC_ORGANIZATION_NAME                 0xDD35
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ROLE                                       0xDD36
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_BIRTHDATE                                  0xDD37
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_MESSAGE_TO                                 0xDD40
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_MESSAGE_CC                                 0xDD41
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_MESSAGE_BCC                                0xDD42
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_MESSAGE_READ                               0xDD43
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_MESSAGE_RECEIVED_TIME                      0xDD44
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_MESSAGE_SENDER                             0xDD45
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ACTIVITY_BEGIN_TIME                        0xDD50
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ACTIVITY_END_TIME                          0xDD51
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ACTIVITY_LOCATION                          0xDD52
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ACTIVITY_REQUIRED_ATTENDEES                0xDD54
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ACTIVITY_OPTIONAL_ATTENDEES                0xDD55
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ACTIVITY_RESOURCES                         0xDD56
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_ACTIVITY_ACCEPTED                          0xDD57
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_OWNER                                      0xDD5D
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_EDITOR                                     0xDD5E
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_WEBMASTER                                  0xDD5F
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_URL_SOURCE                                 0xDD60
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_URL_DESTINATION                            0xDD61
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_TIME_BOOKMARK                              0xDD62
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_OBJECT_BOOKMARK                            0xDD63
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_BYTE_BOOKMARK                              0xDD64
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_LAST_BUILD_DATE                            0xDD70
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_TIME_TO_LIVE                               0xDD71
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROP_MEDIA_GUID                                 0xDD72

/* Define PIMA Object Protection Status Values.  */

#define UX_DEVICE_CLASS_PIMA_OPS_NO_PROTECTION                                      0x0000
#define UX_DEVICE_CLASS_PIMA_OPS_READ_ONLY                                          0x0001
                                                                    
/* Define PIMA Storage Types Codes.  */                                                
                                                                                    
#define UX_DEVICE_CLASS_PIMA_STC_UNDEFINED                                          0x0000
#define UX_DEVICE_CLASS_PIMA_STC_FIXED_ROM                                          0x0001
#define UX_DEVICE_CLASS_PIMA_STC_REMOVABLE_ROM                                      0x0002
#define UX_DEVICE_CLASS_PIMA_STC_FIXED_RAM                                          0x0003
#define UX_DEVICE_CLASS_PIMA_STC_REMOVABLE_RAM                                      0x0004
                                                                                    
/* Define PIMA File System Types Codes.  */                                            
                                                                                    
#define UX_DEVICE_CLASS_PIMA_FSTC_UNDEFINED                                         0x0000
#define UX_DEVICE_CLASS_PIMA_FSTC_GENERIC_FLAT                                      0x0001
#define UX_DEVICE_CLASS_PIMA_FSTC_GENERIC_HIERARCHICAL                              0x0002
#define UX_DEVICE_CLASS_PIMA_FSTC_DCF                                               0x0003
                                                                                    
/* Define PIMA File System Access Types Codes.  */                                    
                                                                                    
#define UX_DEVICE_CLASS_PIMA_AC_READ_WRITE                                          0x0000
#define UX_DEVICE_CLASS_PIMA_AC_RO_WITHOUT_OBJECT_DELETION                          0x0001
#define UX_DEVICE_CLASS_PIMA_AC_RO_WITH_OBJECT_DELETION                             0x0002

/* Define PIMA types.  */
#define UX_DEVICE_CLASS_PIMA_TYPES_INT8                                             0x0001
#define UX_DEVICE_CLASS_PIMA_TYPES_UINT8                                            0x0002
#define UX_DEVICE_CLASS_PIMA_TYPES_INT16                                            0x0003
#define UX_DEVICE_CLASS_PIMA_TYPES_UINT16                                           0x0004
#define UX_DEVICE_CLASS_PIMA_TYPES_INT32                                            0x0005
#define UX_DEVICE_CLASS_PIMA_TYPES_UINT32                                           0x0006
#define UX_DEVICE_CLASS_PIMA_TYPES_INT64                                            0x0007
#define UX_DEVICE_CLASS_PIMA_TYPES_UINT64                                           0x0008
#define UX_DEVICE_CLASS_PIMA_TYPES_INT128                                           0x0009
#define UX_DEVICE_CLASS_PIMA_TYPES_UINT128                                          0x000A
#define UX_DEVICE_CLASS_PIMA_TYPES_AINT8                                            0x4001
#define UX_DEVICE_CLASS_PIMA_TYPES_AUINT8                                           0x4002
#define UX_DEVICE_CLASS_PIMA_TYPES_AINT16                                           0x4003
#define UX_DEVICE_CLASS_PIMA_TYPES_AUINT16                                          0x4004
#define UX_DEVICE_CLASS_PIMA_TYPES_AINT32                                           0x4005
#define UX_DEVICE_CLASS_PIMA_TYPES_AUINT32                                          0x4006
#define UX_DEVICE_CLASS_PIMA_TYPES_AINT64                                           0x4007
#define UX_DEVICE_CLASS_PIMA_TYPES_AUINT64                                          0x4008
#define UX_DEVICE_CLASS_PIMA_TYPES_AINT128                                          0x4009
#define UX_DEVICE_CLASS_PIMA_TYPES_AUINT128                                         0x400A
#define UX_DEVICE_CLASS_PIMA_TYPES_STR                                              0xFFFF

/* Define PIMA Device Info fields.  */

#define UX_DEVICE_CLASS_PIMA_DEVICE_INFO_STANDARD_VERSION               (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 0x0000)
#define UX_DEVICE_CLASS_PIMA_DEVICE_INFO_VENDOR_EXTENSION_ID            (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 0x0002)
#define UX_DEVICE_CLASS_PIMA_DEVICE_INFO_VENDOR_EXTENSION_VERSION       (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 0x0006)
#define UX_DEVICE_CLASS_PIMA_DEVICE_INFO_VENDOR_EXTENSION_DESC          (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 0x0008)

/* Define PIMA MTP OBJECT PROPERTY DATASET.  */
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROPERTY_DATASET_CODE                           0x0000
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROPERTY_DATASET_DATATYPE                       0x0002
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROPERTY_DATASET_GETSET                         0x0004
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROPERTY_DATASET_VALUE                          0x0005

/* Define PIMA Dataset equivalences.  */
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROPERTY_DATASET_VALUE_GET                      0x00
#define UX_DEVICE_CLASS_PIMA_OBJECT_PROPERTY_DATASET_VALUE_GETSET                   0x01

/* Define PIMA event info structure.  */

typedef struct UX_SLAVE_CLASS_PIMA_EVENT_STRUCT
{
    ULONG                   ux_device_class_pima_event_code;
    ULONG                   ux_device_class_pima_event_session_id;
    ULONG                   ux_device_class_pima_event_transaction_id;
    ULONG                   ux_device_class_pima_event_parameter_1;
    ULONG                   ux_device_class_pima_event_parameter_2;
    ULONG                   ux_device_class_pima_event_parameter_3;
    
} UX_SLAVE_CLASS_PIMA_EVENT;

/* Define PIMA object info structure.  */

typedef struct UX_SLAVE_CLASS_PIMA_OBJECT_STRUCT
{

    ULONG                   ux_device_class_pima_object_storage_id;
    ULONG                   ux_device_class_pima_object_format;
    ULONG                   ux_device_class_pima_object_protection_status;
    ULONG                   ux_device_class_pima_object_compressed_size;
    ULONG                   ux_device_class_pima_object_thumb_format;
    ULONG                   ux_device_class_pima_object_thumb_compressed_size;
    ULONG                   ux_device_class_pima_object_thumb_pix_width;
    ULONG                   ux_device_class_pima_object_thumb_pix_height;
    ULONG                   ux_device_class_pima_object_image_pix_width;
    ULONG                   ux_device_class_pima_object_image_pix_height;
    ULONG                   ux_device_class_pima_object_image_bit_depth;
    ULONG                   ux_device_class_pima_object_parent_object;
    ULONG                   ux_device_class_pima_object_association_type;
    ULONG                   ux_device_class_pima_object_association_desc;
    ULONG                   ux_device_class_pima_object_sequence_number;
    UCHAR                   ux_device_class_pima_object_filename[UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_object_capture_date[UX_DEVICE_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_object_modification_date[UX_DEVICE_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_object_keywords[UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH];
    ULONG                   ux_device_class_pima_object_state;
    ULONG                   ux_device_class_pima_object_offset;
    ULONG                   ux_device_class_pima_object_transfer_status;
    ULONG                   ux_device_class_pima_object_handle_id;
    ULONG                   ux_device_class_pima_object_length;
    UCHAR                   *ux_device_class_pima_object_buffer;
    
} UX_SLAVE_CLASS_PIMA_OBJECT;

#define UX_SLAVE_CLASS_PIMA_OBJECT_DATA_LENGTH ((15 * sizeof(ULONG)) + \
                                                    UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH + \
                                                    UX_DEVICE_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH + \
                                                    UX_DEVICE_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH + \
                                                    UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH)

/* Define PIMA session info structure.  Not used in the device. Here for structure compatibility. */

typedef struct UX_SLAVE_CLASS_PIMA_SESSION_STRUCT
{

    ULONG                   ux_device_class_pima_session_id;
    
} UX_SLAVE_CLASS_PIMA_SESSION;

/* Define PIMA device info structure.  */

typedef struct UX_SLAVE_CLASS_PIMA_DEVICE_STRUCT
{

    ULONG                   ux_device_class_pima_device_standard_version;
    ULONG                   ux_device_class_pima_device_vendor_extension_id;
    ULONG                   ux_device_class_pima_device_vendor_extension_version;
    UCHAR                   ux_device_class_pima_device_vendor_extension_desc[UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH];
    ULONG                   ux_device_class_pima_device_functional_mode;
    UCHAR                   ux_device_class_pima_device_operations_supported[UX_DEVICE_CLASS_PIMA_ARRAY_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_device_events_supported[UX_DEVICE_CLASS_PIMA_ARRAY_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_device_properties_supported[UX_DEVICE_CLASS_PIMA_ARRAY_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_device_capture_formats[UX_DEVICE_CLASS_PIMA_ARRAY_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_device_image_formats[UX_DEVICE_CLASS_PIMA_ARRAY_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_device_manufacturer[UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_device_model[UX_DEVICE_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_device_version[UX_DEVICE_CLASS_PIMA_DATE_TIME_STRING_MAX_LENGTH];
    UCHAR                   ux_device_class_pima_device_serial_number[UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH];
    
} UX_SLAVE_CLASS_PIMA_DEVICE;

/* Define PIMA storage info structure.  */                                              
                                                                                        
typedef struct UX_SLAVE_CLASS_PIMA_STORAGE_STRUCT                                        
{                                                                                       
                                                                                        
    ULONG                   ux_device_class_pima_storage_type;                                               
    ULONG                   ux_device_class_pima_storage_file_system_type;                                   
    ULONG                   ux_device_class_pima_storage_access_capability;                                  
    ULONG                   ux_device_class_pima_storage_max_capacity_low;                                   
    ULONG                   ux_device_class_pima_storage_max_capacity_high;                                  
    ULONG                   ux_device_class_pima_storage_free_space_bytes_low;                               
    ULONG                   ux_device_class_pima_storage_free_space_bytes_high;                              
    ULONG                   ux_device_class_pima_storage_free_space_images;                                  
    UCHAR                   ux_device_class_pima_storage_description[UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH]; 
    UCHAR                   ux__class_pima_storage_volume_label[UX_DEVICE_CLASS_PIMA_UNICODE_MAX_LENGTH];
                                                                                        
} UX_SLAVE_CLASS_PIMA_STORAGE;                                                           
                                                                                        
/* Define PIMA structure.  */

typedef struct UX_SLAVE_CLASS_PIMA_STRUCT
{

    UX_SLAVE_INTERFACE      *ux_slave_class_pima_interface;
    UX_SLAVE_ENDPOINT       *ux_device_class_pima_bulk_in_endpoint;
    UX_SLAVE_ENDPOINT       *ux_device_class_pima_bulk_out_endpoint;
    UX_SLAVE_ENDPOINT       *ux_device_class_pima_interrupt_endpoint;
    UINT                    ux_device_class_pima_state;
    USHORT                  ux_device_class_pima_device_status;
    ULONG                   ux_device_class_pima_session_id;
    ULONG                   ux_device_class_pima_current_object_handle;
    ULONG                   ux_device_class_pima_transaction_id;
    UCHAR                   *ux_device_class_pima_manufacturer;
    UCHAR                   *ux_device_class_pima_model;
    UCHAR                   *ux_device_class_pima_device_version;
    UCHAR                   *ux_device_class_pima_serial_number;
    ULONG                   ux_device_class_pima_storage_id;
    ULONG                   ux_device_class_pima_storage_type;
    ULONG                   ux_device_class_pima_storage_file_system_type;
    ULONG                   ux_device_class_pima_storage_access_capability;
    ULONG                   ux_device_class_pima_storage_max_capacity_low;
    ULONG                   ux_device_class_pima_storage_max_capacity_high;
    ULONG                   ux_device_class_pima_storage_free_space_low;
    ULONG                   ux_device_class_pima_storage_free_space_high;
    ULONG                   ux_device_class_pima_storage_free_space_image;
    UCHAR                   *ux_device_class_pima_storage_description;
    UCHAR                   *ux_device_class_pima_storage_volume_label;
#if !defined(UX_DEVICE_STANDALONE)
    UX_SEMAPHORE            ux_device_class_pima_semaphore;
    UX_THREAD               ux_device_class_pima_interrupt_thread;
    UCHAR                   *ux_device_class_pima_interrupt_thread_stack;
    UX_SEMAPHORE            ux_device_class_pima_interrupt_thread_semaphore;
#endif
    UX_SLAVE_CLASS_PIMA_EVENT    
                            *ux_device_class_pima_event_array;
    UX_SLAVE_CLASS_PIMA_EVENT    
                            *ux_device_class_pima_event_array_head;
    UX_SLAVE_CLASS_PIMA_EVENT    
                            *ux_device_class_pima_event_array_tail;
    UX_SLAVE_CLASS_PIMA_EVENT    
                            *ux_device_class_pima_event_array_end;
    USHORT                  *ux_device_class_pima_device_properties_list;
    USHORT                  *ux_device_class_pima_supported_capture_formats_list;
    USHORT                  *ux_device_class_pima_supported_image_formats_list;
    USHORT                  *ux_device_class_pima_object_properties_list;
    UINT                    (*ux_device_class_pima_cancel)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima);
    UINT                    (*ux_device_class_pima_device_reset)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima);
    
    UINT                    (*ux_device_class_pima_device_prop_desc_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG device_property, UCHAR **device_prop_dataset, ULONG *device_prop_dataset_length);
    UINT                    (*ux_device_class_pima_device_prop_value_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG device_property, UCHAR **device_prop_value, ULONG *device_prop_value_length);
    UINT                    (*ux_device_class_pima_device_prop_value_set)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG device_property, UCHAR *device_prop_value, ULONG device_prop_value_length);
    
    UINT                    (*ux_device_class_pima_storage_format)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG storage_id);
    UINT                    (*ux_device_class_pima_storage_info_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG storage_id);
    UINT                    (*ux_device_class_pima_object_number_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_format_code, ULONG object_association, ULONG *object_number);
    UINT                    (*ux_device_class_pima_object_handles_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handles_format_code, 
                                                                        ULONG object_handles_association, 
                                                                        ULONG *object_handles_array,
                                                                        ULONG object_handles_max_number);
    UINT                    (*ux_device_class_pima_object_info_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, UX_SLAVE_CLASS_PIMA_OBJECT **object);
    UINT                    (*ux_device_class_pima_object_data_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, UCHAR *object_buffer, ULONG object_offset,
                                                                ULONG object_length_requested, ULONG *object_actual_length);
    UINT                    (*ux_device_class_pima_object_info_send)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, UX_SLAVE_CLASS_PIMA_OBJECT *object, ULONG storage_id, ULONG parent_object_handle, ULONG *object_handle);
    UINT                    (*ux_device_class_pima_object_data_send)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, ULONG phase, UCHAR *object_buffer, ULONG object_offset,
                                                                ULONG object_length);
    UINT                    (*ux_device_class_pima_object_delete)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle);
    UINT                    (*ux_device_class_pima_object_prop_desc_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_property, ULONG object_format_code, UCHAR **object_prop_value_dataset, ULONG *object_prop_value_dataset_length);
    UINT                    (*ux_device_class_pima_object_prop_value_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, ULONG object_property, UCHAR **object_prop_value, ULONG *object_prop_value_length);
    UINT                    (*ux_device_class_pima_object_prop_value_set)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, ULONG object_property, UCHAR *object_prop_value, ULONG object_prop_value_length);
    UINT                    (*ux_device_class_pima_object_references_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, UCHAR **object_handle_array, ULONG *object_handle_array_length);
    UINT                    (*ux_device_class_pima_object_references_set)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, UCHAR *object_handle_array, ULONG object_handle_array_length);
    VOID                    *ux_device_class_pima_application;
    VOID                    (*ux_device_class_pima_instance_activate)(VOID *);
    VOID                    (*ux_device_class_pima_instance_deactivate)(VOID *);
    
                                                                
} UX_SLAVE_CLASS_PIMA;

/* Define PIMA initialization command structure.  */

typedef struct UX_SLAVE_CLASS_PIMA_PARAMETER_STRUCT
{

    VOID                    (*ux_device_class_pima_instance_activate)(VOID *);
    VOID                    (*ux_device_class_pima_instance_deactivate)(VOID *);
    UCHAR                   *ux_device_class_pima_parameter_manufacturer;
    UCHAR                   *ux_device_class_pima_parameter_model;
    UCHAR                   *ux_device_class_pima_parameter_device_version;
    UCHAR                   *ux_device_class_pima_parameter_serial_number;
    ULONG                   ux_device_class_pima_parameter_storage_id;
    ULONG                   ux_device_class_pima_parameter_storage_type;
    ULONG                   ux_device_class_pima_parameter_storage_file_system_type;
    ULONG                   ux_device_class_pima_parameter_storage_access_capability;
    ULONG                   ux_device_class_pima_parameter_storage_max_capacity_low;
    ULONG                   ux_device_class_pima_parameter_storage_max_capacity_high;
    ULONG                   ux_device_class_pima_parameter_storage_free_space_low;
    ULONG                   ux_device_class_pima_parameter_storage_free_space_high;
    ULONG                   ux_device_class_pima_parameter_storage_free_space_image;
    UCHAR                   *ux_device_class_pima_parameter_storage_description;
    UCHAR                   *ux_device_class_pima_parameter_storage_volume_label;
    USHORT                  *ux_device_class_pima_parameter_device_properties_list;
    USHORT                  *ux_device_class_pima_parameter_supported_capture_formats_list;
    USHORT                  *ux_device_class_pima_parameter_supported_image_formats_list;
    USHORT                  *ux_device_class_pima_parameter_object_properties_list;

    /* PIMA callbacks, return UX success, or PIMA RC code.  */
    UINT                    (*ux_device_class_pima_parameter_cancel)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima);
    UINT                    (*ux_device_class_pima_parameter_device_reset)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima);
    UINT                    (*ux_device_class_pima_parameter_device_prop_desc_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG device_property, UCHAR **device_prop_dataset, ULONG *device_prop_dataset_length);
    UINT                    (*ux_device_class_pima_parameter_device_prop_value_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG device_property, UCHAR **device_prop_value, ULONG *device_prop_value_length);
    UINT                    (*ux_device_class_pima_parameter_device_prop_value_set)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG device_property, UCHAR *device_prop_value, ULONG device_prop_value_length);
    UINT                    (*ux_device_class_pima_parameter_storage_format)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG storage_id);
    UINT                    (*ux_device_class_pima_parameter_storage_info_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG storage_id);
    UINT                    (*ux_device_class_pima_parameter_object_number_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_format_code, ULONG object_association, ULONG *object_number);
    UINT                    (*ux_device_class_pima_parameter_object_handles_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handles_format_code, 
                                                                        ULONG object_handles_association, 
                                                                        ULONG *object_handles_array,
                                                                        ULONG object_handles_max_number);
    UINT                    (*ux_device_class_pima_parameter_object_info_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, UX_SLAVE_CLASS_PIMA_OBJECT **object);
    UINT                    (*ux_device_class_pima_parameter_object_data_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, UCHAR *object_buffer, ULONG object_offset,
                                                                ULONG object_length_requested, ULONG *object_actual_length);
                                                                        
    UINT                    (*ux_device_class_pima_parameter_object_info_send)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, UX_SLAVE_CLASS_PIMA_OBJECT *object, ULONG storage_id, ULONG parent_object_handle, ULONG *object_handle);
    UINT                    (*ux_device_class_pima_parameter_object_data_send)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, ULONG phase ,UCHAR *object_buffer, ULONG object_offset,
                                                                ULONG object_length);
    UINT                    (*ux_device_class_pima_parameter_object_delete)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle);
    UINT                    (*ux_device_class_pima_parameter_object_prop_desc_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, ULONG object_property, UCHAR **object_prop_dataset, ULONG *object_prop_dataset_length);
    UINT                    (*ux_device_class_pima_parameter_object_prop_value_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, ULONG object_property, UCHAR **object_prop_value, ULONG *object_prop_value_length);
    UINT                    (*ux_device_class_pima_parameter_object_prop_value_set)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, ULONG object_property, UCHAR *object_prop_value, ULONG object_prop_value_length);
    UINT                    (*ux_device_class_pima_parameter_object_references_get)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, UCHAR **object_handle_array, ULONG *object_handle_array_length);
    UINT                    (*ux_device_class_pima_parameter_object_references_set)(struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, UCHAR *object_handle_array, ULONG object_handle_array_length);
    VOID                    *ux_device_class_pima_parameter_application;


} UX_SLAVE_CLASS_PIMA_PARAMETER;


/* Define PIMA Object decompaction structure.  */

#define UX_DEVICE_CLASS_PIMA_OBJECT_MAX_LENGTH                              512
#define UX_DEVICE_CLASS_PIMA_OBJECT_VARIABLE_OFFSET                         52  
#define UX_DEVICE_CLASS_PIMA_OBJECT_ENTRIES                                 15


/* Define PIMA storage decompaction structure.  */

#define UX_DEVICE_CLASS_PIMA_STORAGE_TYPE                                   (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 0 )
#define UX_DEVICE_CLASS_PIMA_STORAGE_FILE_SYSTEM_TYPE                       (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 2 )
#define UX_DEVICE_CLASS_PIMA_STORAGE_ACCESS_CAPABILITY                      (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 4 )
#define UX_DEVICE_CLASS_PIMA_STORAGE_MAX_CAPACITY_LOW                       (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 6 )
#define UX_DEVICE_CLASS_PIMA_STORAGE_MAX_CAPACITY_HIGH                      (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 10) 
#define UX_DEVICE_CLASS_PIMA_STORAGE_FREE_SPACE_LOW                         (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 14)
#define UX_DEVICE_CLASS_PIMA_STORAGE_FREE_SPACE_HIGH                        (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 18) 
#define UX_DEVICE_CLASS_PIMA_STORAGE_FREE_SPACE_IMAGE                       (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 22)
#define UX_DEVICE_CLASS_PIMA_STORAGE_FREE_STORAGE_DESCRIPTION               (UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + 26)
 
/* Define Pima Class function prototypes.  */
UINT  _ux_device_class_pima_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_pima_activate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_pima_deactivate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_pima_control_request(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_pima_device_info_send(UX_SLAVE_CLASS_PIMA *pima);
UINT  _ux_device_class_pima_entry(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_pima_event_get(UX_SLAVE_CLASS_PIMA *pima, 
                                      UX_SLAVE_CLASS_PIMA_EVENT *pima_event);
UINT  _ux_device_class_pima_event_set(UX_SLAVE_CLASS_PIMA *pima, 
                                      UX_SLAVE_CLASS_PIMA_EVENT *pima_event);
VOID  _ux_device_class_pima_interrupt_thread(ULONG pima_class);
UINT  _ux_device_class_pima_response_send(UX_SLAVE_CLASS_PIMA *pima, ULONG response_code, 
                                            ULONG number_parameters, 
                                            ULONG parameter_1, ULONG parameter_2, ULONG paramater_3);
VOID  _ux_device_class_pima_thread(ULONG pima_class);
UINT  _ux_device_class_pima_object_handles_send(UX_SLAVE_CLASS_PIMA *pima, 
                                                    ULONG storage_id,
                                                    ULONG object_format_code,
                                                    ULONG object_association);
UINT  _ux_device_class_pima_objects_number_send(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG storage_id,
                                                    ULONG object_format_code,
                                                    ULONG object_association);

UINT  _ux_device_class_pima_device_prop_desc_get(UX_SLAVE_CLASS_PIMA *pima, ULONG device_property_code);
UINT  _ux_device_class_pima_device_prop_value_get(UX_SLAVE_CLASS_PIMA *pima, ULONG device_property_code);
UINT  _ux_device_class_pima_device_prop_value_set(UX_SLAVE_CLASS_PIMA *pima, ULONG device_property_code);
UINT  _ux_device_class_pima_object_info_get(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle);
UINT  _ux_device_class_pima_object_info_send(UX_SLAVE_CLASS_PIMA *pima, ULONG storage_id, ULONG parent_object_handle);
UINT  _ux_device_class_pima_object_data_get(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle);
UINT  _ux_device_class_pima_object_data_send(UX_SLAVE_CLASS_PIMA *pima);
UINT  _ux_device_class_pima_object_delete(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle, ULONG object_format);
UINT  _ux_device_class_pima_object_add(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle);
UINT  _ux_device_class_pima_partial_object_data_get(UX_SLAVE_CLASS_PIMA *pima, 
                                                    ULONG object_handle, 
                                                    ULONG offset_requested, 
                                                    ULONG length_requested);

UINT  _ux_device_class_pima_storage_id_send(UX_SLAVE_CLASS_PIMA *pima);
UINT  _ux_device_class_pima_storage_info_get(UX_SLAVE_CLASS_PIMA *pima, ULONG storage_id);
UINT  _ux_device_class_pima_object_props_supported_get(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_format_code);
UINT  _ux_device_class_pima_object_prop_value_get(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_handle,
                                                    ULONG object_property_code);
UINT  _ux_device_class_pima_object_prop_value_set(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_handle,
                                                    ULONG object_property_code);
UINT  _ux_device_class_pima_object_prop_desc_get(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_property,
                                                    ULONG object_format_code);
UINT  _ux_device_class_pima_object_references_get(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_handle);
UINT  _ux_device_class_pima_object_references_set(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_handle);
UINT  _ux_device_class_pima_object_prop_value_get(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_handle,
                                                    ULONG object_property_code);
UINT  _ux_device_class_pima_object_prop_value_set(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG object_handle,
                                                    ULONG object_property_code);
UINT  _ux_device_class_pima_storage_format(UX_SLAVE_CLASS_PIMA *pima, ULONG storage_id);
UINT  _ux_device_class_pima_device_reset(UX_SLAVE_CLASS_PIMA *pima);

/* Define Device PIMA Class API prototypes.  */

#define ux_device_class_pima_initialize         _ux_device_class_pima_initialize
#define ux_device_class_pima_activate           _ux_device_class_pima_activate
#define ux_device_class_pima_dectivate          _ux_device_class_pima_dectivate
#define ux_device_class_pima_entry              _ux_device_class_pima_entry   
#define ux_device_class_pima_control_request    _ux_device_class_pima_control_request
#define ux_device_class_pima_object_add         _ux_device_class_pima_object_add

/* Define Pima Class data prototypes.  */

extern UCHAR  _ux_device_class_pima_vendor_extension_descriptor[]; 
extern USHORT _ux_device_class_pima_supported_operations[];
extern USHORT _ux_device_class_pima_supported_events[];
extern USHORT _ux_device_class_pima_supported_capture_formats[];
extern USHORT _ux_device_class_pima_supported_image_formats[];
extern USHORT _ux_device_class_pima_device_prop_supported[];



/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif
