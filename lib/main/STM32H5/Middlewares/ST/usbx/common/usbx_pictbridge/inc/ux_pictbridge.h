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
/**   Pictbridge Application                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_pictbridge.h                                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX Pictbridge application                                         */ 
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added magic number defines, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_PICTBRIDGE_H
#define UX_PICTBRIDGE_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  

struct UX_PICTBRIDGE_STRUCT;

/* Define the behaviour of this Pictbridge device : DPSHOST or DPSCLIENT.  
   This should be done in the project management tool. */
#ifndef UX_PICTBRIDGE_DPSHOST_NO
#define UX_PICTBRIDGE_DPSHOST
#endif

#ifndef UX_PICTBRIDGE_DPSCLIENT_NO
#define UX_PICTBRIDGE_DPSCLIENT
#endif

#ifdef UX_PICTBRIDGE_DPSHOST
#include "ux_host_class_pima.h"
#endif

#ifdef UX_PICTBRIDGE_DPSCLIENT
#include "ux_device_class_pima.h"
#endif


/* Define PICTBRIDGE main equivalences.  */

#define UX_PICTBRIDGE_MAX_FILE_NAME_SIZE                    63
#define UX_PICTBRIDGE_MAX_TAG_SIZE                          64 
#define UX_PICTBRIDGE_MAX_ELEMENT_SIZE                      128
#define UX_PICTBRIDGE_MAX_TAG_DEPTH                         16
#define UX_PICTBRIDGE_MAX_VARIABLE_SIZE                     128
#define UX_PICTBRIDGE_MAX_STRING_SIZE                       128
#define UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER                1024
#define UX_PICTBRIDGE_MAX_EVENT_NUMBER                      8
#define UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE                16
#define UX_PICTBRIDGE_MAX_NUMBER_STORAGE_IDS                64
#define UX_PICTBRIDGE_MAX_NUMBER_OBJECT_HANDLES             64
#define UX_PICTBRIDGE_OBJECT_SCRIPT                         0x3002
#define UX_PICTBRIDGE_THREAD_STACK_SIZE                     2048
#define UX_PICTBRIDGE_THREAD_PRIORITY_CLASS                 20
#define UX_PICTBRIDGE_LEADING_ZERO_ON                       1
#define UX_PICTBRIDGE_LEADING_ZERO_OFF                      2
#define UX_PICTBRIDGE_REQUEST                               1
#define UX_PICTBRIDGE_RESPONSE                              2
#define UX_PICTBRIDGE_ALL_CONTAINERS                        0xFFFFFFFF

#define UX_PICTBRIDGE_XML_LEAF                              ((void*)(ALIGN_TYPE)0xFFFFFFFF)
#define UX_PICTBRIDGE_EVENT_TIMEOUT                         (30 * UX_PERIODIC_RATE)

/* Define PICTBRIDGE OBJECT HANDLES.  */

#define UX_PICTBRIDGE_OBJECT_HANDLE_HOST_REQUEST            1
#define UX_PICTBRIDGE_OBJECT_HANDLE_HOST_RESPONSE           1
#define UX_PICTBRIDGE_OBJECT_HANDLE_CLIENT_REQUEST          2
#define UX_PICTBRIDGE_OBJECT_HANDLE_PRINT                   3

/* Define PICTBRIDGE HOST\CLIENT state machine flags.  */

#define UX_PICTBRIDGE_STATE_MACHINE_IDLE                    0x00000001
#define UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST_PENDING  0x00000002
#define UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST          0x00000004
#define UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST_PENDING    0x00000008
#define UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST            0x00000010

/* Define PICTBRIDGE event flags.  */

#define UX_PICTBRIDGE_EVENT_FLAG_CONFIGURE_PRINT_SERVICE    0x00000001
#define UX_PICTBRIDGE_EVENT_FLAG_CAPABILITY                 0x00000002
#define UX_PICTBRIDGE_EVENT_FLAG_JOB_STATUS                 0x00000004
#define UX_PICTBRIDGE_EVENT_FLAG_DEVICE_STATUS              0x00000008
#define UX_PICTBRIDGE_EVENT_FLAG_START_JOB                  0x00000010
#define UX_PICTBRIDGE_EVENT_FLAG_ABORT_JOB                  0x00000020
#define UX_PICTBRIDGE_EVENT_FLAG_CONTINUE_JOB               0x00000040
#define UX_PICTBRIDGE_EVENT_FLAG_NOTIFY_JOB_STATUS          0x00000080
#define UX_PICTBRIDGE_EVENT_FLAG_NOTIFY_DEVICE_STATUS       0x00000100

#define UX_PICTBRIDGE_EVENT_FLAG_APPLICATION_1              0x00100000
#define UX_PICTBRIDGE_EVENT_FLAG_APPLICATION_2              0x00200000
#define UX_PICTBRIDGE_EVENT_FLAG_APPLICATION_3              0x00400000
#define UX_PICTBRIDGE_EVENT_FLAG_APPLICATION_4              0x00800000

#define UX_PICTBRIDGE_EVENT_FLAG_DISCOVERY                  0x00010000
#define UX_PICTBRIDGE_EVENT_FLAG_ERROR                      0x80000000
#define UX_PICTBRIDGE_EVENT_FLAG_STATE_MACHINE_READY        0x40000000

/* Define PICTBRIDGE QUALITY API equivalences.  */

#define UX_PICTBRIDGE_API_QUALITIES                         0x00000001
#define UX_PICTBRIDGE_API_PAPER_SIZES                       0x00000002
#define UX_PICTBRIDGE_API_PAPER_TYPES                       0x00000004
#define UX_PICTBRIDGE_API_PAPER_TYPES_SIZE                  0x00000008
#define UX_PICTBRIDGE_API_FILE_TYPES                        0x00000010
#define UX_PICTBRIDGE_API_DATE_PRINTS                       0x00000020
#define UX_PICTBRIDGE_API_FILE_NAME_PRINTS                  0x00000040
#define UX_PICTBRIDGE_API_IMAGE_OPTIMIZES                   0x00000080
#define UX_PICTBRIDGE_API_LAYOUTS                           0x00000100
#define UX_PICTBRIDGE_API_LAYOUTS_SIZE                      0x00000200
#define UX_PICTBRIDGE_API_FIXED_SIZES                       0x00000400
#define UX_PICTBRIDGE_API_CROPPINGS                         0x00000800
#define UX_PICTBRIDGE_API_CHAR_REPERTOIRES                  0x00001000

/* Define PICTBRIDGE tag characters.  */

#define UX_PICTBRIDGE_TAG_CHAR_SPACE                        0x20
#define UX_PICTBRIDGE_TAG_CHAR_START_BRACKET                0x3C
#define UX_PICTBRIDGE_TAG_CHAR_END_BRACKET                  0x3E
#define UX_PICTBRIDGE_TAG_CHAR_EQUAL                        0x3D
#define UX_PICTBRIDGE_TAG_CHAR_QUOTE                        0x22
#define UX_PICTBRIDGE_TAG_CHAR_CR                           0x0d
#define UX_PICTBRIDGE_TAG_CHAR_LF                           0x0a
#define UX_PICTBRIDGE_TAG_CHAR_SLASH                        0x2f
#define UX_PICTBRIDGE_TAG_CHAR_QUESTION_MARK                0x3f

/* Define PICTBRIDGE tag flags.  */

#define UX_PICTBRIDGE_TAG_FLAG_BEGIN                        0x00000001
#define UX_PICTBRIDGE_TAG_FLAG_END                          0x00000002
#define UX_PICTBRIDGE_TAG_FLAG_TRAILING_SPACE               0x00000004
#define UX_PICTBRIDGE_TAG_FLAG_CLOSING                      0x00000008
#define UX_PICTBRIDGE_TAG_FLAG_SELF_CLOSING                 0x00000010
#define UX_PICTBRIDGE_TAG_FLAG_CHAR_BEGIN                   0x00000020
#define UX_PICTBRIDGE_TAG_FLAG_QUOTE                        0x00000040
#define UX_PICTBRIDGE_TAG_FLAG_COMMENT                      0x00000080
#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA                0x00000100
#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE_DECIMAL             0x00000200
#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE_MAJOR_MINOR         0x00000400
#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE_STRING              0x00000800
#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA          0x00001000
#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_DECIMAL       0x00002000
#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_MAJOR_MINOR   0x00004000
#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_STRING        0x00008000
#define UX_PICTBRIDGE_TAG_FLAG_FORCE_LF                     0x00010000
#define UX_PICTBRIDGE_TAG_FLAG_IN_TAG                       0x00020000
#define UX_PICTBRIDGE_TAG_FLAG_IN_VARIABLE                  0x00040000
#define UX_PICTBRIDGE_TAG_FLAG_EXPECTING_STRING             0x00080000
#define UX_PICTBRIDGE_TAG_FLAG_IN_STRING                    0x00100000
#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE_DECIMAL_3DIGITS     0x00200000
#define UX_PICTBRIDGE_TAG_FLAG_FORCE_SLASH_AT_END           0x00400000

#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE                     (UX_PICTBRIDGE_TAG_FLAG_VARIABLE_HEXA             | \
                                                            UX_PICTBRIDGE_TAG_FLAG_VARIABLE_DECIMAL           | \
                                                            UX_PICTBRIDGE_TAG_FLAG_VARIABLE_MAJOR_MINOR       | \
                                                            UX_PICTBRIDGE_TAG_FLAG_VARIABLE_STRING            | \
                                                            UX_PICTBRIDGE_TAG_FLAG_VARIABLE_DECIMAL_3DIGITS)

#define UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY               (UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_HEXA       | \
                                                            UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_DECIMAL     | \
                                                            UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_MAJOR_MINOR | \
                                                            UX_PICTBRIDGE_TAG_FLAG_VARIABLE_ARRAY_STRING)

/* Define PICTBRIDGE error codes.  */

#define UX_PICTBRIDGE_ERROR_UNDEFINED                       0x00002000
#define UX_PICTBRIDGE_ERROR_GENERAL_ERROR                   0x00002002
#define UX_PICTBRIDGE_ERROR_SESSION_NOT_OPEN                0x00002003
#define UX_PICTBRIDGE_ERROR_INVALID_TRANSACTION_ID          0x00002004
#define UX_PICTBRIDGE_ERROR_OPERATION_NOT_SUPPORTED         0x00002005
#define UX_PICTBRIDGE_ERROR_PARAMETER_NOT_SUPPORTED         0x00002006
#define UX_PICTBRIDGE_ERROR_INCOMPLETE_TRANSFER             0x00002007
#define UX_PICTBRIDGE_ERROR_INVALID_STORAGE_ID              0x00002008
#define UX_PICTBRIDGE_ERROR_INVALID_OBJECT_HANDLE           0x00002009
#define UX_PICTBRIDGE_ERROR_DEVICE_PROP_NOT_SUPPORTED       0x0000200A
#define UX_PICTBRIDGE_ERROR_INVALID_OBJECT_FORMAT_CODE      0x0000200B
#define UX_PICTBRIDGE_ERROR_STORE_FULL                      0x0000200C
#define UX_PICTBRIDGE_ERROR_OBJECT_WRITE_PROTECTED          0x0000200D
#define UX_PICTBRIDGE_ERROR_STORE_READ_ONLY                 0x0000200E
#define UX_PICTBRIDGE_ERROR_ACCESS_DENIED                   0x0000200F
#define UX_PICTBRIDGE_ERROR_NO_THUMBNAIL_PRESENT            0x00002010
#define UX_PICTBRIDGE_ERROR_SELF_TEST_FAILED                0x00002011
#define UX_PICTBRIDGE_ERROR_PARTIAL_DELETION                0x00002012
#define UX_PICTBRIDGE_ERROR_STORE_NOT_AVAILABLE             0x00002013
#define UX_PICTBRIDGE_ERROR_FORMAT_UNSUPPORTED              0x00002014
#define UX_PICTBRIDGE_ERROR_NO_VALID_OBJECT_INFO            0x00002015
#define UX_PICTBRIDGE_ERROR_INVALID_CODE_FORMAT             0x00002016
#define UX_PICTBRIDGE_ERROR_UNKNOWN_VENDOR_CODE             0x00002017
#define UX_PICTBRIDGE_ERROR_CAPTURE_ALREADY_TERMINATED      0x00002018
#define UX_PICTBRIDGE_ERROR_DEVICE_BUSY                     0x00002019
#define UX_PICTBRIDGE_ERROR_INVALID_PARENT_OBJECT           0x0000201A
#define UX_PICTBRIDGE_ERROR_INVALID_DEVICE_PROP_FORMAT      0x0000201B
#define UX_PICTBRIDGE_ERROR_INVALID_DEVICE_PROP_VALUE       0x0000201C
#define UX_PICTBRIDGE_ERROR_INVALID_PARAMETER               0x0000201D
#define UX_PICTBRIDGE_ERROR_SESSION_ALREADY_OPENED          0x0000201E
#define UX_PICTBRIDGE_ERROR_TRANSACTION_CANCELED            0x0000201F
#define UX_PICTBRIDGE_ERROR_DESTINATION_UNSUPPORTED         0x00002020
#define UX_PICTBRIDGE_ERROR_OBJECT_ALREADY_OPENED           0x00002021
#define UX_PICTBRIDGE_ERROR_OBJECT_ALREADY_CLOSED           0x00002022
#define UX_PICTBRIDGE_ERROR_OBJECT_NOT_OPENED               0x00002023
#define UX_PICTBRIDGE_ERROR_NO_DISCOVERY_SCRIPT             0x00002024
#define UX_PICTBRIDGE_ERROR_SCRIPT_BUFFER_OVERFLOW          0x00002025
#define UX_PICTBRIDGE_ERROR_SCRIPT_SYNTAX_ERROR             0x00002026
#define UX_PICTBRIDGE_ERROR_PARAMETER_UNKNOWN               0x00002027
#define UX_PICTBRIDGE_ERROR_PARAMETER_MISSING               0x00002028
#define UX_PICTBRIDGE_ERROR_EMPTY_LINE                      0x00002029

/* Define Pictbridge Event Codes.  */

#define UX_PICTBRIDGE_EC_UNDEFINED                          0x00004000
#define UX_PICTBRIDGE_EC_CANCEL_TRANSACTION                 0x00004001
#define UX_PICTBRIDGE_EC_OBJECT_ADDED                       0x00004002
#define UX_PICTBRIDGE_EC_OBJECT_REMOVED                     0x00004003
#define UX_PICTBRIDGE_EC_STORE_ADDED                        0x00004004
#define UX_PICTBRIDGE_EC_STORE_REMOVED                      0x00004005
#define UX_PICTBRIDGE_EC_DEVICE_PROP_CHANGED                0x00004006
#define UX_PICTBRIDGE_EC_OBJECT_INFO_CHANGED                0x00004007
#define UX_PICTBRIDGE_EC_DEVICE_INFO_CHANGED                0x00004008
#define UX_PICTBRIDGE_EC_REQUEST_OBJECT_TRANSFER            0x00004009
#define UX_PICTBRIDGE_EC_STORE_FULL                         0x0000400A
#define UX_PICTBRIDGE_EC_DEVICE_RESET                       0x0000400B
#define UX_PICTBRIDGE_EC_STORAGE_INFO_CHANGED               0x0000400C
#define UX_PICTBRIDGE_EC_CAPTURE_COMPLETE                   0x0000400D
#define UX_PICTBRIDGE_EC_UNREPORTED_STATUS                  0x0000400E

/* Define Pictbridge Internal Event Codes.  */

#define UX_PICTBRIDGE_EC_START_JOB                          0x00004100

/* Define Pictbridge state machine.  */
#define UX_PICTBRIDGE_DPSCLIENT_DISCOVERY_PENDING           0x00000000
#define UX_PICTBRIDGE_DPSCLIENT_DISCOVERY_COMPLETE          0x00000001


/* Define Pictbridge XML Object input\output direction flags.  */
#define UX_PICTBRIDGE_OBJECT_DIRECTION_INPUT                0x00000001
#define UX_PICTBRIDGE_OBJECT_DIRECTION_OUTPUT               0x00000002

/* Define Pictbridge output request codes.  */

#define UX_PICTBRIDGE_OR_CONFIGURE_PRINT_SERVICE            0x00000001
#define UX_PICTBRIDGE_OR_GET_CAPABILITY                     0x00000002
#define UX_PICTBRIDGE_OR_GET_JOB_STATUS                     0x00000003
#define UX_PICTBRIDGE_OR_GET_DEVICE_STATUS                  0x00000004
#define UX_PICTBRIDGE_OR_START_JOB                          0x00000005
#define UX_PICTBRIDGE_OR_ABORT_JOB                          0x00000006
#define UX_PICTBRIDGE_OR_CONTINUE_JOB                       0x00000007
#define UX_PICTBRIDGE_OR_NOTIFY_JOB_STATUS                  0x00000008
#define UX_PICTBRIDGE_OR_NOTIFY_DEVICE_STATUS               0x00000009

/* Define Pictbridge input request codes.  */

#define UX_PICTBRIDGE_IR_CONFIGURE_PRINT_SERVICE            0x00000001
#define UX_PICTBRIDGE_IR_GET_CAPABILITY                     0x00000002
#define UX_PICTBRIDGE_IR_GET_JOB_STATUS                     0x00000003
#define UX_PICTBRIDGE_IR_GET_DEVICE_STATUS                  0x00000004
#define UX_PICTBRIDGE_IR_START_JOB                          0x00000005
#define UX_PICTBRIDGE_IR_ABORT_JOB                          0x00000006
#define UX_PICTBRIDGE_IR_CONTINUE_JOB                       0x00000007
#define UX_PICTBRIDGE_IR_NOTIFY_JOB_STATUS                  0x00000008
#define UX_PICTBRIDGE_IR_NOTIFY_DEVICE_STATUS               0x00000009

/* Define Pictbridge input request subfunction flags.  */

#define UX_PICTBRIDGE_IR_CPS_DPS_VERSIONS                   0x00000001
#define UX_PICTBRIDGE_IR_CPS_VENDOR_NAME                    0x00000002
#define UX_PICTBRIDGE_IR_CPS_VENDOR_SPECIFIC_VERSION        0x00000004
#define UX_PICTBRIDGE_IR_CPS_PRODUCT_NAME                   0x00000008
#define UX_PICTBRIDGE_IR_CPS_SERIAL_NO                      0x00000010

#define UX_PICTBRIDGE_IR_GC_QUALITIES                       0x00000001
#define UX_PICTBRIDGE_IR_GC_PAPER_SIZES                     0x00000002
#define UX_PICTBRIDGE_IR_GC_PAPER_TYPES                     0x00000004
#define UX_PICTBRIDGE_IR_GC_PAPER_TYPES_SIZE                0x00000008
#define UX_PICTBRIDGE_IR_GC_FILE_TYPES                      0x00000010
#define UX_PICTBRIDGE_IR_GC_DATE_PRINTS                     0x00000020
#define UX_PICTBRIDGE_IR_GC_FILE_NAME_PRINTS                0x00000040
#define UX_PICTBRIDGE_IR_GC_IMAGE_OPTIMIZES                 0x00000080
#define UX_PICTBRIDGE_IR_GC_LAYOUTS                         0x00000100
#define UX_PICTBRIDGE_IR_GC_LAYOUTS_SIZE                    0x00000200
#define UX_PICTBRIDGE_IR_GC_FIXED_SIZES                     0x00000400
#define UX_PICTBRIDGE_IR_GC_CROPPINGS                       0x00000800
#define UX_PICTBRIDGE_IR_GC_CHAR_REPERTOIRES                0x00001000

#define UX_PICTBRIDGE_IR_SJJC_QUALITY                       0x00000001
#define UX_PICTBRIDGE_IR_SJJC_PAPER_TYPE                    0x00000002
#define UX_PICTBRIDGE_IR_SJJC_PAPER_SIZE                    0x00000004
#define UX_PICTBRIDGE_IR_SJJC_FILE_TYPE                     0x00000008
#define UX_PICTBRIDGE_IR_SJJC_DATE_PRINT                    0x00000010
#define UX_PICTBRIDGE_IR_SJJC_FILE_NAME_PRINT               0x00000020
#define UX_PICTBRIDGE_IR_SJJC_IMAGE_OPTIMIZE                0x00000040
#define UX_PICTBRIDGE_IR_SJJC_LAYOUT                        0x00000080
#define UX_PICTBRIDGE_IR_SJJC_FIXED_SIZE                    0x00000100
#define UX_PICTBRIDGE_IR_SJJC_CROPPINGS                     0x00000200

#define UX_PICTBRIDGE_IR_SJPI_CROPPING_AREA                 0x00000400
#define UX_PICTBRIDGE_IR_SJPI_FILE_ID                       0x00000800
#define UX_PICTBRIDGE_IR_SJPI_FILE_NAME                     0x00001000
#define UX_PICTBRIDGE_IR_SJPI_DATE                          0x00002000
#define UX_PICTBRIDGE_IR_SJPI_COPIES                        0x00004000
#define UX_PICTBRIDGE_IR_SJPI_PRT_PID                       0x00008000
#define UX_PICTBRIDGE_IR_SJPI_FILE_PATH                     0x00010000
#define UX_PICTBRIDGE_IR_SJPI_COPY_ID                       0x00020000

#define UX_PICTBRIDGE_IR_NJS_PRT_PID                        0x00000001
#define UX_PICTBRIDGE_IR_NJS_FILE_PATH                      0x00000002
#define UX_PICTBRIDGE_IR_NJS_COPY_ID                        0x00000004
#define UX_PICTBRIDGE_IR_NJS_PROGRESS                       0x00000008
#define UX_PICTBRIDGE_IR_NJS_IMAGES_PRINTED                 0x00000010


/* Define parameter format masks.  */
#define UX_PICTBRIDGE_PARAMETER_MAJOR_CODE_MASK             0xFFFF0000
#define UX_PICTBRIDGE_PARAMETER_MINOR_CODE_MASK             0x0000FFFF
#define UX_PICTBRIDGE_ERROR_REASON_MINOR_CODE_MASK          0x0000FF00
#define UX_PICTBRIDGE_ERROR_REASON_DETAIL_CODE_MASK         0x000000FF

/* Define Pictbridge Qualities Codes.  */

#define UX_PICTBRIDGE_QUALITIES_DEFAULT                     0x50000000
#define UX_PICTBRIDGE_QUALITIES_NORMAL                      0x50010000
#define UX_PICTBRIDGE_QUALITIES_DRAFT                       0x50020000
#define UX_PICTBRIDGE_QUALITIES_FINE                        0x50030000

/* Define Pictbridge PaperSizes Codes.  */

#define UX_PICTBRIDGE_PAPER_SIZES_DEFAULT                   0x51000000
#define UX_PICTBRIDGE_PAPER_SIZES_L                         0x51010000
#define UX_PICTBRIDGE_PAPER_SIZES_2L                        0x51020000
#define UX_PICTBRIDGE_PAPER_SIZES_HAGAKI_POSTCARD           0x51030000
#define UX_PICTBRIDGE_PAPER_SIZES_CARD_SIZE                 0x51040000
#define UX_PICTBRIDGE_PAPER_SIZES_100X150                   0x51050000
#define UX_PICTBRIDGE_PAPER_SIZES_4IX6I                     0x51060000
#define UX_PICTBRIDGE_PAPER_SIZES_8IX10I                    0x51070000
#define UX_PICTBRIDGE_PAPER_SIZES_LETTER                    0x51080000
#define UX_PICTBRIDGE_PAPER_SIZES_11IX17I                   0x510A0000

/* Define Pictbridge PaperTypes Codes.  */

#define UX_PICTBRIDGE_PAPER_TYPES_DEFAULT                   0x52000000
#define UX_PICTBRIDGE_PAPER_TYPES_PLAIN                     0x52010000
#define UX_PICTBRIDGE_PAPER_TYPES_PHOTO                     0x52020000
#define UX_PICTBRIDGE_PAPER_TYPES_FAST_PHOTO                0x52030000

/* Define Pictbridge FileTypes Codes.  */

#define UX_PICTBRIDGE_FILE_TYPES_DEFAULT                    0x53000000
#define UX_PICTBRIDGE_FILE_TYPES_EXIF_JPEG                  0x53010000
#define UX_PICTBRIDGE_FILE_TYPES_OTHER_EXIF                 0x53020000
#define UX_PICTBRIDGE_FILE_TYPES_JPEG                       0x53030000
#define UX_PICTBRIDGE_FILE_TYPES_TIFF_EP                    0x53040000    
#define UX_PICTBRIDGE_FILE_TYPES_FLASHPIX                   0x53050000    
#define UX_PICTBRIDGE_FILE_TYPES_BMP                        0x53060000    
#define UX_PICTBRIDGE_FILE_TYPES_CIFF                       0x53070000    
#define UX_PICTBRIDGE_FILE_TYPES_GIF                        0x53080000    
#define UX_PICTBRIDGE_FILE_TYPES_JFIF                       0x53090000    
#define UX_PICTBRIDGE_FILE_TYPES_PCD                        0x530A0000    
#define UX_PICTBRIDGE_FILE_TYPES_PICT                       0x530B0000    
#define UX_PICTBRIDGE_FILE_TYPES_PNG                        0x530C0000    
#define UX_PICTBRIDGE_FILE_TYPES_TIFF                       0x530D0000    
#define UX_PICTBRIDGE_FILE_TYPES_TIFF_IT                    0x530E0000    
#define UX_PICTBRIDGE_FILE_TYPES_JP2                        0x530F0000    
#define UX_PICTBRIDGE_FILE_TYPES_JPX                        0x53110000    
#define UX_PICTBRIDGE_FILE_TYPES_UNDEFINED                  0x53120000    
#define UX_PICTBRIDGE_FILE_TYPES_ASSOCIATION                0x53130000    
#define UX_PICTBRIDGE_FILE_TYPES_SCRIPT                     0x53140000    
#define UX_PICTBRIDGE_FILE_TYPES_EXECUTABLE                 0x53150000    
#define UX_PICTBRIDGE_FILE_TYPES_TEXT                       0x53160000    
#define UX_PICTBRIDGE_FILE_TYPES_HTML                       0x53170000    
#define UX_PICTBRIDGE_FILE_TYPES_DPOF                       0x53180000    
#define UX_PICTBRIDGE_FILE_TYPES_AIFF                       0x53190000    
#define UX_PICTBRIDGE_FILE_TYPES_WAV                        0x531A0000    
#define UX_PICTBRIDGE_FILE_TYPES_MP3                        0x531B0000    
#define UX_PICTBRIDGE_FILE_TYPES_AVI                        0x531C0000    
#define UX_PICTBRIDGE_FILE_TYPES_MPEG                       0x531D0000    
#define UX_PICTBRIDGE_FILE_TYPES_ASF                        0x531E0000    

/* Define Pictbridge action result codes.  */

#define UX_PICTBRIDGE_ACTION_RESULT_OK                      0x10000000
#define UX_PICTBRIDGE_ACTION_RESULT_NOT_EXECUTED            0x10010000
#define UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_DEFAULT   0x10020000
#define UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_UP        0x10020001
#define UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_IP        0x10020002
#define UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_MP        0x10020003
#define UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_BO        0x10020004
#define UX_PICTBRIDGE_ACTION_RESULT_NOT_RECOGNIZED          0x10030000

/* Define Pictbridge DatePrints Codes.  */

#define UX_PICTBRIDGE_DATE_PRINTS_DEFAULT                   0x54000000
#define UX_PICTBRIDGE_DATE_PRINTS_OFF                       0x54010000
#define UX_PICTBRIDGE_DATE_PRINTS_ON                        0x54020000

/* Define Pictbridge FileNamePrints Codes.  */

#define UX_PICTBRIDGE_FILE_NAME_PRINTS_DEFAULT              0x55000000
#define UX_PICTBRIDGE_FILE_NAME_PRINTS_OFF                  0x55010000
#define UX_PICTBRIDGE_FILE_NAME_PRINTS_ON                   0x55020000

/* Define Pictbridge ImageOptimizes Codes.  */

#define UX_PICTBRIDGE_IMAGE_OPTIMIZES_DEFAULT               0x56000000
#define UX_PICTBRIDGE_IMAGE_OPTIMIZES_OFF                   0x56010000
#define UX_PICTBRIDGE_IMAGE_OPTIMIZES_ON                    0x56020000

/* Define Pictbridge Layouts Codes.  */

#define UX_PICTBRIDGE_LAYOUTS_DEFAULT                       0x57000000
#define UX_PICTBRIDGE_LAYOUTS_1_UP_BORDER                   0x57010000
#define UX_PICTBRIDGE_LAYOUTS_1_UP_LAYOUT                   0x57020000
#define UX_PICTBRIDGE_LAYOUTS_2_UP_LAYOUT                   0x57030000
#define UX_PICTBRIDGE_LAYOUTS_3_UP_LAYOUT                   0x57040000
#define UX_PICTBRIDGE_LAYOUTS_4_UP_LAYOUT                   0x57050000
#define UX_PICTBRIDGE_LAYOUTS_5_UP_LAYOUT                   0x57060000
#define UX_PICTBRIDGE_LAYOUTS_6_UP_LAYOUT                   0x57070000
#define UX_PICTBRIDGE_LAYOUTS_7_UP_LAYOUT                   0x57080000
#define UX_PICTBRIDGE_LAYOUTS_8_UP_LAYOUT                   0x57090000
#define UX_PICTBRIDGE_LAYOUTS_INDEX_PRINT                   0x57FE0000
#define UX_PICTBRIDGE_LAYOUTS_1_UP_BORDERLESS               0x57FF0000

/* Define Pictbridge FixedSizes Codes.  */

#define UX_PICTBRIDGE_FIXED_SIZE_DEFAULT                    0x58000000
#define UX_PICTBRIDGE_FIXED_SIZE_2IX3I                      0x58010000
#define UX_PICTBRIDGE_FIXED_SIZE_35IX5I                     0x58020000
#define UX_PICTBRIDGE_FIXED_SIZE_4IX6I                      0x58030000
#define UX_PICTBRIDGE_FIXED_SIZE_5IX7I                      0x58040000
#define UX_PICTBRIDGE_FIXED_SIZE_8IX10I                     0x58050000
#define UX_PICTBRIDGE_FIXED_SIZE_254MMX178MM                0x58060000
#define UX_PICTBRIDGE_FIXED_SIZE_110MMX74MM                 0x58070000
#define UX_PICTBRIDGE_FIXED_SIZE_6CMX8CM                    0x58080000
#define UX_PICTBRIDGE_FIXED_SIZE_7CMX10CM                   0x58090000
#define UX_PICTBRIDGE_FIXED_SIZE_9CMX13CM                   0x580A0000
#define UX_PICTBRIDGE_FIXED_SIZE_10CMX13CM                  0x580B0000
#define UX_PICTBRIDGE_FIXED_SIZE_15CMX21CM                  0x580C0000
#define UX_PICTBRIDGE_FIXED_SIZE_18CMX24CM                  0x580D0000
#define UX_PICTBRIDGE_FIXED_SIZE_A4                         0x580E0000
#define UX_PICTBRIDGE_FIXED_SIZE_LETTER                     0x580F0000

/* Define Pictbridge Croppings Codes.  */

#define UX_PICTBRIDGE_CROPPINGS_DEFAULT                     0x59000000
#define UX_PICTBRIDGE_CROPPINGS_OFF                         0x59010000
#define UX_PICTBRIDGE_CROPPINGS_ON                          0x59020000

/* Define Pictbridge Abort Style Codes.  */

#define UX_PICTBRIDGE_ABORT_STYLE_IMMEDIATELY               0x90000000
#define UX_PICTBRIDGE_ABORT_STYLE_AFTER_CURRENT_PAGE        0x90010000

/* Define Pictbridge Print Service Status Codes.  */

#define UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_ACTIVE        0x70000000
#define UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_IDLE          0x70010000
#define UX_PICTBRIDGE_DPS_PRINTSERVICE_STATUS_PAUSED        0x70020000

/* Define Pictbridge Print Service Status Codes.  */

#define UX_PICTBRIDGE_JOB_END_REASON_NOT_ENDED              0x71000000
#define UX_PICTBRIDGE_JOB_END_REASON_NORMAL                 0x71010000
#define UX_PICTBRIDGE_JOB_END_REASON_ABORTED_9000000        0x71020000
#define UX_PICTBRIDGE_JOB_END_REASON_ABORTED_9001000        0x71030000
#define UX_PICTBRIDGE_JOB_END_REASON_ABORTED_OTHER          0x71040000

/* Define Pictbridge Error Status Codes.  */

#define UX_PICTBRIDGE_ERROR_STATUS_NO_ERROR                 0x72000000
#define UX_PICTBRIDGE_ERROR_STATUS_WARNING                  0x72010000
#define UX_PICTBRIDGE_ERROR_STATUS_FATAL                    0x72020000

/* Define Pictbridge Error REASON Status Codes.  */

#define UX_PICTBRIDGE_ERROR_REASON_NO_REASON                0x73000000
#define UX_PICTBRIDGE_ERROR_REASON_PAPER_RELATED            0x73010000
#define UX_PICTBRIDGE_ERROR_REASON_INK_RELATED              0x73020000
#define UX_PICTBRIDGE_ERROR_REASON_HARDWARE_RELATED         0x73030000
#define UX_PICTBRIDGE_ERROR_REASON_FILE_RELATED             0x73040000

/* Define Pictbridge Disconnect Enable Status Codes.  */

#define UX_PICTBRIDGE_DISCONNECT_ENABLE_FALSE               0x74000000
#define UX_PICTBRIDGE_DISCONNECT_ENABLE_TRUE                0x74010000

/* Define Pictbridge Capability Changed Status Codes.  */

#define UX_PICTBRIDGE_CAPABILITY_CHANGED_FALSE              0x75000000
#define UX_PICTBRIDGE_CAPABILITY_CHANGED_TRUE               0x75010000

/* Define Pictbridge New Job Status Codes.  */

#define UX_PICTBRIDGE_NEW_JOB_FALSE                         0x76000000
#define UX_PICTBRIDGE_NEW_JOB_TRUE                          0x76010000

/* Define Pictbridge Print Info structure.  */
typedef struct UX_PICTBRIDGE_PRINTINFO_STRUCT
{

    struct UX_PICTBRIDGE_PRINTINFO_STRUCT                   *ux_pictbridge_printinfo_next;
    ULONG                                                   ux_pictbridge_printinfo_croppingarea_xcoordinate;
    ULONG                                                   ux_pictbridge_printinfo_croppingarea_ycoordinate;
    ULONG                                                   ux_pictbridge_printinfo_croppingarea_width;
    ULONG                                                   ux_pictbridge_printinfo_croppingarea_height;
    ULONG                                                   ux_pictbridge_printinfo_fileid;
    UCHAR                                                   ux_pictbridge_printinfo_filename[UX_PICTBRIDGE_MAX_STRING_SIZE];
    UCHAR                                                   ux_pictbridge_printinfo_date[UX_PICTBRIDGE_MAX_STRING_SIZE];
    ULONG                                                   ux_pictbridge_printinfo_copies;
    ULONG                                                   ux_pictbridge_printinfo_prtpid;
    UCHAR                                                   ux_pictbridge_printinfo_filepath[UX_PICTBRIDGE_MAX_STRING_SIZE];
    ULONG                                                   ux_pictbridge_printinfo_copyid;
    ULONG                                                   ux_pictbridge_printinfo_current_page;
    ULONG                                                   ux_pictbridge_printinfo_total_page;
    ULONG                                                   ux_pictbridge_printinfo_images_printed;
} UX_PICTBRIDGE_PRINTINFO;

/* Define Pictbridge Job Info structure.  */
typedef struct UX_PICTBRIDGE_JOBINFO_STRUCT
{

    ULONG                                                   ux_pictbridge_jobinfo_status;
    ULONG                                                   ux_pictbridge_jobinfo_quality;
    ULONG                                                   ux_pictbridge_jobinfo_papertype;
    ULONG                                                   ux_pictbridge_jobinfo_papersize;
    ULONG                                                   ux_pictbridge_jobinfo_filetype;
    ULONG                                                   ux_pictbridge_jobinfo_dateprint;
    ULONG                                                   ux_pictbridge_jobinfo_filenameprint;
    ULONG                                                   ux_pictbridge_jobinfo_imageoptimize;
    ULONG                                                   ux_pictbridge_jobinfo_layout;
    ULONG                                                   ux_pictbridge_jobinfo_fixedsize;
    ULONG                                                   ux_pictbridge_jobinfo_cropping;
    ULONG                                                   ux_pictbridge_jobinfo_abort_style;
    VOID                                                    *ux_pictbridge_jobinfo_object;
    UINT                                                    (*ux_pictbridge_jobinfo_object_data_read)(struct UX_PICTBRIDGE_STRUCT *pictbridge, ULONG object_handle, UCHAR *buffer,  ULONG offset, ULONG length, ULONG *actual_length);
    UINT                                                    (*ux_pictbridge_jobinfo_object_info_get) (struct UX_SLAVE_CLASS_PIMA_STRUCT *pima, ULONG object_handle, UX_SLAVE_CLASS_PIMA_OBJECT **object);
    struct UX_PICTBRIDGE_PRINTINFO_STRUCT                   *ux_pictbridge_jobinfo_printinfo_start;
    struct UX_PICTBRIDGE_PRINTINFO_STRUCT                   *ux_pictbridge_jobinfo_printinfo_current;
} UX_PICTBRIDGE_JOBINFO;

/* Define Pictbridge Device Info structure.  */
typedef struct UX_PICTBRIDGE_DEVINFO_STRUCT
{

    UCHAR                                                   ux_pictbridge_devinfo_vendor_name[UX_PICTBRIDGE_MAX_STRING_SIZE];
    UCHAR                                                   ux_pictbridge_devinfo_product_name[UX_PICTBRIDGE_MAX_STRING_SIZE];
    UCHAR                                                   ux_pictbridge_devinfo_serial_no[UX_PICTBRIDGE_MAX_STRING_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_dpsversions[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_vendor_specific_version;
    ULONG                                                   ux_pictbridge_devinfo_print_service_available;
    ULONG                                                   ux_pictbridge_devinfo_qualities[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_papersizes[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_papertypes[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_papertypes_papersize;
    ULONG                                                   ux_pictbridge_devinfo_filetypes[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_dateprints[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_filenameprints[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_imageoptimizes[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_layouts[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_layouts_papersize;
    ULONG                                                   ux_pictbridge_devinfo_fixedsizes[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_croppings[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_charrepertoires[UX_PICTBRIDGE_MAX_DEVINFO_ARRAY_SIZE];
    ULONG                                                   ux_pictbridge_devinfo_dpsprintservicestatus;
    ULONG                                                   ux_pictbridge_devinfo_jobendreason;
    ULONG                                                   ux_pictbridge_devinfo_errorstatus;
    ULONG                                                   ux_pictbridge_devinfo_errorreason;
    ULONG                                                   ux_pictbridge_devinfo_disconnectenable;
    ULONG                                                   ux_pictbridge_devinfo_capabilitychanged;
    ULONG                                                   ux_pictbridge_devinfo_newjobok;
    ULONG                                                   ux_pictbridge_devinfo_storage_size;

} UX_PICTBRIDGE_DEVINFO;


/* Define Pictbridge event structure.  */
typedef struct UX_PICTBRIDGE_EVENT_STRUCT
{
    ULONG                                                   ux_pictbridge_event_code;
    ULONG                                                   ux_pictbridge_event_parameter_1;
    ULONG                                                   ux_pictbridge_event_parameter_2;
    ULONG                                                   ux_pictbridge_event_parameter_3;
    
} UX_PICTBRIDGE_EVENT;

/* Define Pictbridge structure.  */
typedef struct UX_PICTBRIDGE_STRUCT
{

    VOID                                                    *ux_pictbridge_pima;
    ULONG                                                   ux_pictbridge_storage_ids[UX_PICTBRIDGE_MAX_NUMBER_STORAGE_IDS];
    ULONG                                                   ux_pictbridge_object_handles_array[UX_PICTBRIDGE_MAX_NUMBER_OBJECT_HANDLES];
    UX_PICTBRIDGE_EVENT                                     ux_pictbridge_event_array[UX_PICTBRIDGE_MAX_EVENT_NUMBER];
    UX_PICTBRIDGE_EVENT                                     *ux_pictbridge_event_array_head;
    UX_PICTBRIDGE_EVENT                                     *ux_pictbridge_event_array_tail;
    UX_PICTBRIDGE_EVENT                                     *ux_pictbridge_event_array_end;
    UX_SEMAPHORE                                            ux_pictbridge_notification_semaphore;
    UCHAR                                                   *ux_pictbridge_thread_stack;
    UX_THREAD                                               ux_pictbridge_thread;
    UX_EVENT_FLAGS_GROUP                                    ux_pictbridge_event_flags_group;
    ULONG                                                   ux_pictbridge_operation_result;
    ULONG                                                   ux_pictbridge_output_result;
    ULONG                                                   ux_pictbridge_xml_report_direction;
    ULONG                                                   ux_pictbridge_request_response;
    ULONG                                                   ux_pictbridge_input_tags;
    ULONG                                                   ux_pictbridge_input_request;
    ULONG                                                   ux_pictbridge_discovery_state;
    ULONG                                                   ux_pictbridge_host_client_state_machine;
    UX_PICTBRIDGE_DEVINFO                                   ux_pictbridge_dpslocal;
    UX_PICTBRIDGE_DEVINFO                                   ux_pictbridge_dpsclient;
    UX_PICTBRIDGE_JOBINFO                                   ux_pictbridge_jobinfo;
    VOID                                                    *ux_pictbridge_storage;
    VOID                                                    *ux_pictbridge_object_host;
    VOID                                                    *ux_pictbridge_object_client;
    VOID                                                    *ux_pictbridge_session;
    VOID                                                    *ux_pictbridge_device;
    UINT                                                    (*ux_pictbridge_application_object_data_write)(struct UX_PICTBRIDGE_STRUCT *pictbridge, UCHAR *buffer,  ULONG offset, ULONG total_length, ULONG length);
#ifdef UX_PICTBRIDGE_DPSCLIENT
    UX_SLAVE_CLASS_PIMA_PARAMETER                           ux_pictbridge_pima_parameter;
    UINT                                                    (*ux_pictbridge_dps_event_callback_function)(struct UX_PICTBRIDGE_STRUCT *pictbridge, UINT event_flag);
#endif

} UX_PICTBRIDGE;

/* Define PICTBRIDGE XML Item structure.  */

typedef struct UX_PICTBRIDGE_XML_ITEM_STRUCT
{
    UCHAR                                                   ux_pictbridge_xml_item_tag_name[UX_PICTBRIDGE_MAX_TAG_SIZE];
    ULONG                                                   ux_pictbridge_xml_item_tag_code;
    struct UX_PICTBRIDGE_XML_ITEM_STRUCT                    *ux_pictbridge_xml_item_child;
    struct UX_PICTBRIDGE_XML_ITEM_STRUCT                    *ux_pictbridge_xml_item_parent;
    UINT                                                    (*ux_pictbridge_xml_item_function)(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
} UX_PICTBRIDGE_XML_ITEM;





UINT  _ux_pictbridge_xml_function_root_xml(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
UINT  _ux_pictbridge_xml_function_root_dps(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
UINT  _ux_pictbridge_xml_function_root_input(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
UINT  _ux_pictbridge_xml_function_root_output(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
UINT  _ux_pictbridge_xml_function_null(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
UINT  _ux_pictbridge_xml_function_input_getcapability_capability_layouts(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
UINT  _ux_pictbridge_xml_function_input_getcapability_capability_papertypes(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
UINT  _ux_pictbridge_xml_function_input_startjob(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
UINT  _ux_pictbridge_xml_function_input_startjob_jobconfig_quality(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);         
UINT  _ux_pictbridge_xml_function_input_startjob_jobconfig_papersize(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);   
UINT  _ux_pictbridge_xml_function_input_startjob_jobconfig_papertype(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);   
UINT  _ux_pictbridge_xml_function_input_startjob_jobconfig_filetype(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);        
UINT  _ux_pictbridge_xml_function_input_startjob_jobconfig_dateprint(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);   
UINT  _ux_pictbridge_xml_function_input_startjob_jobconfig_filenameprint(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter); 
UINT  _ux_pictbridge_xml_function_input_startjob_jobconfig_imageoptimize(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter); 
UINT  _ux_pictbridge_xml_function_input_startjob_jobconfig_layout(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_startjob_jobconfig_fixedsize(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);   
UINT  _ux_pictbridge_xml_function_input_startjob_jobconfig_cropping(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);        
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo_croppingarea(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);    
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo_fileid(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo_filename(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);        
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo_date(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);            
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo_copies(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo_prtpid(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo_filepath(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);        
UINT  _ux_pictbridge_xml_function_input_startjob_printinfo_copyid(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_notifydevicestatus_dpsprintservicestatus(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_notifydevicestatus_jobendreason(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_notifydevicestatus_errorstatus(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_notifydevicestatus_errorreason(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_notifydevicestatus_capabilitychanged(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_notifydevicestatus_disconnectenable(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_input_notifydevicestatus_newjobok(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_output_result(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);
UINT  _ux_pictbridge_xml_function_output_getcapability_capability_qualities(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);        
UINT  _ux_pictbridge_xml_function_output_getcapability_capability_papersizes(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);       
UINT  _ux_pictbridge_xml_function_output_getcapability_capability_filetypes(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);        
UINT  _ux_pictbridge_xml_function_output_getcapability_capability_dateprints(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);       
UINT  _ux_pictbridge_xml_function_output_getcapability_capability_filenameprints(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);       
UINT  _ux_pictbridge_xml_function_output_getcapability_capability_imageoptimizes(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);       
UINT  _ux_pictbridge_xml_function_output_getcapability_capability_fixedsizes(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);       
UINT  _ux_pictbridge_xml_function_output_getcapability_capability_croppings(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);        
UINT  _ux_pictbridge_xml_function_output_getcapability_capability_papertypes(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);       
UINT  _ux_pictbridge_xml_function_output_getcapability_capability_layouts(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);      
UINT  _ux_pictbridge_xml_function_output_getdevicestatus_dpsprintservicestatus(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);                        
UINT  _ux_pictbridge_xml_function_output_getdevicestatus_jobendreason(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);                             
UINT  _ux_pictbridge_xml_function_output_getdevicestatus_errorstatus(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);                              
UINT  _ux_pictbridge_xml_function_output_getdevicestatus_errorreason(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);                              
UINT  _ux_pictbridge_xml_function_output_getdevicestatus_disconnectenable(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);                         
UINT  _ux_pictbridge_xml_function_output_getdevicestatus_capabilitychanged(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);                            
UINT  _ux_pictbridge_xml_function_output_getdevicestatus_newjobok(UX_PICTBRIDGE *pictbridge, UCHAR *input_variable, UCHAR *input_string, UCHAR *xml_parameter);                                 

/* Remapping of pictbridge functions that point to a NULL function..  */

#define        _ux_pictbridge_xml_function_input_getfilelist_filetype                               _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getfilelist_parentfileid                           _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getfilelist_maxnumids                              _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getfilelist_fileids                               _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getfilelist_numids                                _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getpartialfile_fileid                              _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getpartialfile_offset                              _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getpartialfile_maxsize                             _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getpartialfile_bytesread                          _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getfile_fileid                                     _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getfile_bytesread                                 _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getthumb_fileid                                    _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getthumb_bytesread                                _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getfileid_fileid                                   _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getfileinfo_filetype                              _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getfileinfo_filesize                              _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getfileinfo_thumbformat                           _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getfileinfo_thumbsize                             _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getfileid_basepathid                               _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getfileid_filepath                                 _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getfileid_basepathid                              _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getfileid_filepath                                _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_notifydevicestatus                                 _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_notifyjobstatus                                    _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_notifyjobstatus_prtpid                             _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_notifyjobstatus_filepath                           _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_notifyjobstatus_copyid                             _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_notifyjobstatus_progress                           _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_notifyjobstatus_imagesprinted                     _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getdevicestatus                                    _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_abortjob                                           _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getjobstatus_prtpid                               _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getjobstatus_filepath                             _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getjobstatus_copyid                               _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getjobstatus_progress                             _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getjobstatus_imagesprinted                        _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_printserviceavailable                             _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_dpsversion                                        _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_vendorname                                        _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_vendorspecificversion                             _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_productname                                       _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_serialno                                          _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_startjob                                          _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_abortjob                                          _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_continuejob                                       _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_notifyjobstatus                                   _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_notifydevicestatus                                _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getcapability_capability                          _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getcapability_capability_qualities                 _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getcapability_capability_papersizes                _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getcapability_capability_filetypes                 _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getcapability_capability_dateprints                _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getcapability_capability_filenameprints            _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getcapability_capability_imageoptimizes            _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getcapability_capability_fixedsizes                _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getcapability_capability_croppings                 _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getcapability_capability_charrepertoires           _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_getjobstatus                                       _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_continuejob                                        _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_dpsversion                                         _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_vendorname                                         _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_vendorspecificversion                              _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_productname                                        _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_input_serialno                                           _ux_pictbridge_xml_function_null
#define        _ux_pictbridge_xml_function_output_getcapability_capability_charrepertoires          _ux_pictbridge_xml_function_null

#ifdef UX_PICTBRIDGE_DPSCLIENT

VOID  _ux_pictbridge_dpsclient_thread(ULONG parameter);
UINT  _ux_pictbridge_dpsclient_object_data_get(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle, UCHAR *object_buffer, ULONG object_offset,
                                                                ULONG object_length_requested, ULONG *object_actual_length);
UINT  _ux_pictbridge_dpsclient_object_data_send(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle, ULONG phase,
                                                UCHAR *object_buffer, ULONG object_offset,
                                                ULONG object_length);
UINT  _ux_pictbridge_dpsclient_object_number_get(UX_SLAVE_CLASS_PIMA *pima, ULONG object_format_code, ULONG object_association, ULONG *number_objects);
UINT  _ux_pictbridge_dpsclient_object_info_get(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle, 
                                                UX_SLAVE_CLASS_PIMA_OBJECT **object);
UINT  _ux_pictbridge_dpsclient_object_info_send(UX_SLAVE_CLASS_PIMA *pima, UX_SLAVE_CLASS_PIMA_OBJECT *object, 
                                                ULONG storage_id, ULONG parent_object_handle, ULONG *object_handle);
UINT  _ux_pictbridge_dpsclient_object_delete(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle);
UINT  _ux_pictbridge_dpsclient_object_handles_get(UX_SLAVE_CLASS_PIMA *pima, 
                                                    ULONG object_handles_format_code, 
                                                    ULONG object_handles_association, 
                                                    ULONG *object_handles_array,
                                                    ULONG object_handles_max_number);
UINT  _ux_pictbridge_dpsclient_input_object_prepare(UX_PICTBRIDGE *pictbridge, 
                                                    ULONG input_function, 
                                                    ULONG input_subfunction,
                                                    ULONG input_parameter);
UINT _ux_pictbridge_dpsclient_input_object_configure_print_service(UX_PICTBRIDGE *pictbridge, 
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated);
UINT _ux_pictbridge_dpsclient_input_object_get_capability(UX_PICTBRIDGE *pictbridge, 
                                                            ULONG input_subcode,
                                                            ULONG input_parameter,
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated);
UINT _ux_pictbridge_dpsclient_input_object_startjob(UX_PICTBRIDGE *pictbridge, 
                                                            ULONG input_subcode,
                                                            ULONG input_parameter,
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated);
UINT _ux_pictbridge_dpsclient_input_object_abortjob(UX_PICTBRIDGE *pictbridge, 
                                                            ULONG input_subcode,
                                                            ULONG input_parameter,
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated);
UINT _ux_pictbridge_dpsclient_input_object_continuejob(UX_PICTBRIDGE *pictbridge, 
                                                            ULONG input_subcode,
                                                            ULONG input_parameter,
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated);
UINT _ux_pictbridge_dpsclient_start(UX_PICTBRIDGE *pictbridge);
UINT _ux_pictbridge_dpsclient_api_configure_print_service(UX_PICTBRIDGE *pictbridge);
UINT _ux_pictbridge_dpsclient_api_capability(UX_PICTBRIDGE *pictbridge, ULONG capability_code, 
                                             ULONG capability_parameter);
UINT _ux_pictbridge_dpsclient_api_device_status(UX_PICTBRIDGE *pictbridge);
UINT _ux_pictbridge_dpsclient_api_start_job(UX_PICTBRIDGE *pictbridge);
UINT _ux_pictbridge_dpsclient_api_abort_job(UX_PICTBRIDGE *pictbridge);
UINT _ux_pictbridge_dpsclient_api_continue_job(UX_PICTBRIDGE *pictbridge);
UINT _ux_pictbridge_dpsclient_register_event_callback_function(UX_PICTBRIDGE *pictbridge, 
                                                               UINT (*event_callback_function)(struct UX_PICTBRIDGE_STRUCT *pictbridge, UINT event_flag));


/* Define Pictbridge Client API prototypes.  */

#define ux_pictbridge_dpsclient_object_data_get                                    _ux_pictbridge_dpsclient_object_data_get
#define ux_pictbridge_dpsclient_object_data_send                                   _ux_pictbridge_dpsclient_object_data_send
#define ux_pictbridge_dpsclient_object_number_get                                  _ux_pictbridge_dpsclient_object_number_get
#define ux_pictbridge_dpsclient_object_info_get                                    _ux_pictbridge_dpsclient_object_info_get
#define ux_pictbridge_dpsclient_object_info_send                                   _ux_pictbridge_dpsclient_object_info_send
#define ux_pictbridge_dpsclient_object_delete                                      _ux_pictbridge_dpsclient_object_delete
#define ux_pictbridge_dpsclient_object_handles_get                                 _ux_pictbridge_dpsclient_object_handles_get
#define ux_pictbridge_dpsclient_input_object_prepare                               _ux_pictbridge_dpsclient_input_object_prepare
#define ux_pictbridge_dpsclient_input_object_configure_print_service               _ux_pictbridge_dpsclient_input_object_configure_print_service
#define ux_pictbridge_dpsclient_input_object_get_capability                        _ux_pictbridge_dpsclient_input_object_get_capability
#define ux_pictbridge_dpsclient_input_object_startjob                              _ux_pictbridge_dpsclient_input_object_startjob
#define ux_pictbridge_dpsclient_input_object_abortjob                              _ux_pictbridge_dpsclient_input_object_abortjob
#define ux_pictbridge_dpsclient_input_object_continuejob                           _ux_pictbridge_dpsclient_input_object_continuejob
#define ux_pictbridge_dpsclient_start                                              _ux_pictbridge_dpsclient_start
#define ux_pictbridge_dpsclient_api_configure_print_service                        _ux_pictbridge_dpsclient_api_configure_print_service
#define ux_pictbridge_dpsclient_api_capability                                     _ux_pictbridge_dpsclient_api_capability
#define ux_pictbridge_dpsclient_api_device_status                                  _ux_pictbridge_dpsclient_api_device_status
#define ux_pictbridge_dpsclient_api_start_job                                      _ux_pictbridge_dpsclient_api_start_job
#define ux_pictbridge_dpsclient_api_abort_job                                      _ux_pictbridge_dpsclient_api_abort_job
#define ux_pictbridge_dpsclient_api_continue_job                                   _ux_pictbridge_dpsclient_api_continue_job
#define ux_pictbridge_dpsclient_register_event_callback_function                   _ux_pictbridge_dpsclient_register_event_callback_function


#endif
#ifdef UX_PICTBRIDGE_DPSHOST
UINT  _ux_pictbridge_dpshost_start(UX_PICTBRIDGE *pictbridge, UX_HOST_CLASS_PIMA *pima);
UINT  _ux_pictbridge_dpshost_startjob(UX_PICTBRIDGE *pictbridge);
VOID  _ux_pictbridge_dpshost_thread(ULONG parameter);
VOID  _ux_pictbridge_dpshost_notification_callback(UX_HOST_CLASS_PIMA_EVENT *pima_event);
UINT  _ux_pictbridge_dpshost_object_get(UX_PICTBRIDGE *pictbridge, ULONG object_handle);
UINT  _ux_pictbridge_dpshost_response_get(UX_PICTBRIDGE *pictbridge);
UINT  _ux_pictbridge_dpshost_input_object_send(UX_PICTBRIDGE *pictbridge, ULONG input_function);
UINT  _ux_pictbridge_dpshost_output_object_create(UX_PICTBRIDGE *pictbridge);
UINT  _ux_pictbridge_dpshost_output_object_configure_print_service(UX_PICTBRIDGE *pictbridge, 
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated);

UINT  _ux_pictbridge_dpshost_input_object_notify_job_status(UX_PICTBRIDGE *pictbridge, 
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated);

UINT  _ux_pictbridge_dpshost_input_object_notify_device_status(UX_PICTBRIDGE *pictbridge, 
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated);

UINT  _ux_pictbridge_dpshost_output_object_get_capability(UX_PICTBRIDGE *pictbridge, 
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated);

UINT  _ux_pictbridge_dpshost_output_object_get_device_status(UX_PICTBRIDGE *pictbridge, 
                                                            UCHAR *pima_object_buffer, 
                                                            ULONG object_length, 
                                                            UCHAR **pima_object_buffer_updated, 
                                                            ULONG *object_length_updated);

/* Define Pictbridge Client API prototypes.  */

#endif

/* Define the XML schema root prototype.  */
extern UX_PICTBRIDGE_XML_ITEM  _ux_pictbridge_xml_item_root[];

/* Define pictbridge permanent function.  */
UINT  _ux_pictbridge_object_parse(UX_PICTBRIDGE *pictbridge, UCHAR *xml_object_buffer,
                                    ULONG xml_object_length);
UINT  _ux_pictbridge_array_element_to_array_hexa(UCHAR *element, ULONG *hexa_array);
UINT  _ux_pictbridge_element_to_decimal(UCHAR *element, ULONG *decimal_value);
UINT  _ux_pictbridge_element_to_hexa(UCHAR *element, ULONG *hexa_value);
UINT  _ux_pictbridge_hexa_to_element(ULONG hexa_value, UCHAR *element);
UINT  _ux_pictbridge_tag_name_get(UCHAR *input_buffer, ULONG input_length, 
                                  UCHAR *tag_name,
                                  UCHAR *variable_name,
                                  UCHAR *variable_string,
                                  UCHAR *xml_parameter,
                                  UCHAR **output_buffer, ULONG *output_length,
                                  ULONG *tag_flag);
UINT  _ux_pictbridge_tag_name_scan(UX_PICTBRIDGE_XML_ITEM *tag_item,
                                  UCHAR *tag_name,
                                  UX_PICTBRIDGE_XML_ITEM **tag_entry);
UINT  _ux_pictbridge_hexa_to_major_minor(ULONG hexa_value, UCHAR *output_buffer);
UINT  _ux_pictbridge_hexa_to_decimal_string(ULONG hexa_value, UCHAR *decimal_string, 
                                            ULONG leading_zero_flag, ULONG max_digit_string_size);
UINT  _ux_pictbridge_object_tag_line_add(UCHAR *pima_object_buffer, 
                                                 ULONG object_length, 
                                                 UCHAR *tag_element_string,
                                                 ULONG tag_flag,
                                                 UCHAR *tag_variable,
                                                 ULONG  tag_variable_value,
                                                 VOID  *tag_element,
                                                 UCHAR **pima_object_buffer_updated, 
                                                 ULONG *object_length_updated);

UINT  _ux_pictbridge_input_object_create(UX_PICTBRIDGE *pictbridge, ULONG input_function);
                                                 


/* Define external strings and xml tag lines.  */
extern UCHAR _ux_pictbridge_volume_description[];
extern UCHAR _ux_pictbridge_volume_label[];
extern UCHAR _ux_pictbridge_ddiscovery_name[];
extern UCHAR _ux_pictbridge_hdiscovery_name[];
extern UCHAR _ux_pictbridge_hrequest_name[];
extern UCHAR _ux_pictbridge_hrsponse_name[];
extern UCHAR _ux_pictbridge_drequest_name[];
extern UCHAR _ux_pictbridge_drsponse_name[];
extern UCHAR _ux_pictbridge_xml_tag_line_xmlversion[];                
extern UCHAR _ux_pictbridge_xml_tag_line_dpsxmlns[];                  
extern UCHAR _ux_pictbridge_xml_tag_line_dps[];                   
extern UCHAR _ux_pictbridge_xml_tag_line_output[];                    
extern UCHAR _ux_pictbridge_xml_tag_line_input[];                     
extern UCHAR _ux_pictbridge_xml_tag_line_result[];                    
extern UCHAR _ux_pictbridge_xml_tag_line_configureprintservice[];     
extern UCHAR _ux_pictbridge_xml_tag_line_printserviceavailable[];     
extern UCHAR _ux_pictbridge_xml_tag_line_dpsversions[];               
extern UCHAR _ux_pictbridge_xml_tag_line_vendorname[];                
extern UCHAR _ux_pictbridge_xml_tag_line_vendorspecificversion[];     
extern UCHAR _ux_pictbridge_xml_tag_line_productname[];               
extern UCHAR _ux_pictbridge_xml_tag_line_serialno[];                  
extern UCHAR _ux_pictbridge_xml_tag_line_capability[];          
extern UCHAR _ux_pictbridge_xml_tag_line_getcapability[];       
extern UCHAR _ux_pictbridge_xml_tag_line_qualities[];           
extern UCHAR _ux_pictbridge_xml_tag_line_papersizes[];          
extern UCHAR _ux_pictbridge_xml_tag_line_papertypes[];          
extern UCHAR _ux_pictbridge_xml_tag_line_filetypes[];           
extern UCHAR _ux_pictbridge_xml_tag_line_dateprints[];          
extern UCHAR _ux_pictbridge_xml_tag_line_filenameprints[];      
extern UCHAR _ux_pictbridge_xml_tag_line_imageoptimizes[];      
extern UCHAR _ux_pictbridge_xml_tag_line_layouts[];             
extern UCHAR _ux_pictbridge_xml_tag_line_fixedsizes[];          
extern UCHAR _ux_pictbridge_xml_tag_line_croppings[];           
extern UCHAR _ux_pictbridge_xml_tag_line_charrepertoires[];     
extern UCHAR _ux_pictbridge_xml_variable_papersize[];
extern UCHAR _ux_pictbridge_xml_variable_version[];
extern UCHAR _ux_pictbridge_xml_variable_xmlns[];
extern UCHAR _ux_pictbridge_xml_string_xmlns[];
extern UCHAR _ux_pictbridge_xml_tag_line_getdevicestatus[]; 
extern UCHAR _ux_pictbridge_xml_tag_line_dpsprintservicestatus[]; 
extern UCHAR _ux_pictbridge_xml_tag_line_jobendreason[]; 
extern UCHAR _ux_pictbridge_xml_tag_line_errorstatus[]; 
extern UCHAR _ux_pictbridge_xml_tag_line_errorreason[]; 
extern UCHAR _ux_pictbridge_xml_tag_line_disconnectenable[]; 
extern UCHAR _ux_pictbridge_xml_tag_line_capabilitychanged[]; 
extern UCHAR _ux_pictbridge_xml_tag_line_newjobok[]; 
extern UCHAR _ux_pictbridge_xml_tag_line_notifydevicestatus[];
extern UCHAR _ux_pictbridge_xml_tag_line_notifyjobstatus[];
extern UCHAR _ux_pictbridge_xml_tag_line_progress[];
extern UCHAR _ux_pictbridge_xml_tag_line_imagesprinted[];
extern UCHAR _ux_pictbridge_xml_tag_line_startjob[];
extern UCHAR _ux_pictbridge_xml_tag_line_abortjob[];
extern UCHAR _ux_pictbridge_xml_tag_line_abortstyle[];
extern UCHAR _ux_pictbridge_xml_tag_line_continuejob[];

extern UCHAR _ux_pictbridge_xml_tag_line_quality[];           
extern UCHAR _ux_pictbridge_xml_tag_line_papersize[];          
extern UCHAR _ux_pictbridge_xml_tag_line_papertype[];          
extern UCHAR _ux_pictbridge_xml_tag_line_filetype[];           
extern UCHAR _ux_pictbridge_xml_tag_line_dateprint[];          
extern UCHAR _ux_pictbridge_xml_tag_line_filenameprint[];      
extern UCHAR _ux_pictbridge_xml_tag_line_imageoptimize[];      
extern UCHAR _ux_pictbridge_xml_tag_line_layout[];             
extern UCHAR _ux_pictbridge_xml_tag_line_fixedsize[];          
extern UCHAR _ux_pictbridge_xml_tag_line_cropping[];           
extern UCHAR _ux_pictbridge_xml_tag_line_jobconfig[];
extern UCHAR _ux_pictbridge_xml_tag_line_printinfo[];
extern UCHAR _ux_pictbridge_xml_tag_line_fileid[];
extern UCHAR _ux_pictbridge_xml_tag_line_filename[];
extern UCHAR _ux_pictbridge_xml_tag_line_date[];

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif




