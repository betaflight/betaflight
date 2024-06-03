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
/**   Host Sierra Wireless AR module class                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_swar.h                                PORTABLE C      */ 
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX Sierra Wireless AR Class.                                      */ 
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
/*                                                                        */
/**************************************************************************/

#ifndef UX_HOST_CLASS_SWAR_H
#define UX_HOST_CLASS_SWAR_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  


/* Define Sierra Wireless AR Class constants.  */

#define UX_HOST_CLASS_SWAR_CLASS_TRANSFER_TIMEOUT               300000
#ifndef UX_HOST_CLASS_SWAR_VENDOR_ID
#define UX_HOST_CLASS_SWAR_VENDOR_ID                            0X1199
#define UX_HOST_CLASS_SWAR_PRODUCT_ID                           0X68A3 
#endif

/* Define Sierra Wireless AR Class packet equivalences.  */
#define UX_HOST_CLASS_SWAR_PACKET_SIZE                          128

/* Define Sierra Wireless AR Class data interface.  */
#define UX_HOST_CLASS_SWAR_DATA_INTERFACE                       3

/* Define Sierra Wireless AR IOCTL functions.  */
#define UX_HOST_CLASS_SWAR_IOCTL_ABORT_IN_PIPE                  1
#define UX_HOST_CLASS_SWAR_IOCTL_ABORT_OUT_PIPE                 2

/* Define CDC ACM Reception States. */

#define UX_HOST_CLASS_SWAR_RECEPTION_STATE_STOPPED              0
#define UX_HOST_CLASS_SWAR_RECEPTION_STATE_STARTED              1
#define UX_HOST_CLASS_SWAR_RECEPTION_STATE_IN_TRANSFER          2


/* Define Sierra Wireless Airprime Class instance structure.  */

typedef struct UX_HOST_CLASS_SWAR_STRUCT
{

    struct UX_HOST_CLASS_SWAR_STRUCT  
                    *ux_host_class_swar_next_instance;
    UX_HOST_CLASS   *ux_host_class_swar_class;
    UX_DEVICE       *ux_host_class_swar_device;
    UX_INTERFACE    *ux_host_class_swar_interface;
    UX_ENDPOINT     *ux_host_class_swar_bulk_out_endpoint;
    UX_ENDPOINT     *ux_host_class_swar_bulk_in_endpoint;
    UINT            ux_host_class_swar_state;
    UX_SEMAPHORE    ux_host_class_swar_semaphore;

    struct UX_HOST_CLASS_SWAR_RECEPTION_STRUCT  
                    *ux_host_class_swar_reception;
    ULONG           ux_host_class_swar_notification_count;
} UX_HOST_CLASS_SWAR;

/* Define Sierra Wireless reception structure. */

typedef struct UX_HOST_CLASS_SWAR_RECEPTION_STRUCT
{

    ULONG           ux_host_class_swar_reception_state;
    ULONG           ux_host_class_swar_reception_block_size;
    UCHAR           *ux_host_class_swar_reception_data_buffer;
    ULONG           ux_host_class_swar_reception_data_buffer_size;
    UCHAR           *ux_host_class_swar_reception_data_head;
    UCHAR           *ux_host_class_swar_reception_data_tail;
    VOID            (*ux_host_class_swar_reception_callback)(struct UX_HOST_CLASS_SWAR_STRUCT *swar, 
                                                                UINT  status,
                                                                UCHAR *reception_buffer, 
                                                                ULONG reception_size);

} UX_HOST_CLASS_SWAR_RECEPTION;

/* Define  Sierra Wireless Airprime Classfunction prototypes.  */

UINT    _ux_host_class_swar_activate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_swar_configure(UX_HOST_CLASS_SWAR *swar);
UINT    _ux_host_class_swar_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_swar_endpoints_get(UX_HOST_CLASS_SWAR *swar);
UINT    _ux_host_class_swar_entry(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_swar_read (UX_HOST_CLASS_SWAR *swar, UCHAR *data_pointer, 
                                    ULONG requested_length, ULONG *actual_length);
UINT    _ux_host_class_swar_write(UX_HOST_CLASS_SWAR *swar, UCHAR *data_pointer, 
                                    ULONG requested_length, ULONG *actual_length);
UINT    _ux_host_class_swar_ioctl(UX_HOST_CLASS_SWAR *swar, ULONG ioctl_function,
                                    VOID *parameter);
VOID    _ux_host_class_swar_reception_callback (UX_TRANSFER *transfer_request);
UINT    _ux_host_class_swar_reception_stop (UX_HOST_CLASS_SWAR *swar, 
                                    UX_HOST_CLASS_SWAR_RECEPTION *swar_reception);
UINT    _ux_host_class_swar_reception_start (UX_HOST_CLASS_SWAR *swar, 
                                    UX_HOST_CLASS_SWAR_RECEPTION *swar_reception);
                                    
/* Define SWAR Class API prototypes.  */

#define  ux_host_class_swar_entry                           _ux_host_class_swar_entry
#define  ux_host_class_swar_read                            _ux_host_class_swar_read
#define  ux_host_class_swar_write                           _ux_host_class_swar_write
#define  ux_host_class_swar_ioctl                           _ux_host_class_swar_ioctl
#define  ux_host_class_swar_command                         _ux_host_class_swar_command
#define  ux_host_class_swar_reception_stop                  _ux_host_class_swar_reception_stop
#define  ux_host_class_swar_reception_start                 _ux_host_class_swar_reception_start
#define  ux_host_class_swar_setup                           _ux_host_class_swar_setup

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif
