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
/**   Device Stack                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_device_stack.h                                   PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file defines the equivalences for the USBX Device Stack        */ 
/*    component.                                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DEVICE_STACK_H
#define UX_DEVICE_STACK_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  


/* Define USB Device Stack prototypes.  */

UINT    _ux_device_stack_alternate_setting_get(ULONG interface_value);
UINT    _ux_device_stack_alternate_setting_set(ULONG interface_value, ULONG alternate_setting_value);
UINT    _ux_device_stack_class_register(UCHAR *class_name,
                    UINT (*class_entry_function)(struct UX_SLAVE_CLASS_COMMAND_STRUCT *),
                    ULONG configuration_number,
                    ULONG interface_number,
                    VOID *parameter);
UINT    _ux_device_stack_clear_feature(ULONG request_type, ULONG request_value, ULONG request_index);
UINT    _ux_device_stack_configuration_get(VOID);
UINT    _ux_device_stack_configuration_set(ULONG configuration_value);
UINT    _ux_device_stack_control_request_process(UX_SLAVE_TRANSFER *transfer_request);
UINT    _ux_device_stack_descriptor_send(ULONG descriptor_type, ULONG request_index, ULONG host_length);
UINT    _ux_device_stack_disconnect(VOID);
UINT    _ux_device_stack_endpoint_stall(UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_device_stack_get_status(ULONG request_type, ULONG request_index, ULONG request_length);
UINT    _ux_device_stack_host_wakeup(VOID);
UINT    _ux_device_stack_initialize(UCHAR * device_framework_high_speed, ULONG device_framework_length_high_speed,
                    UCHAR * device_framework_full_speed, ULONG device_framework_length_full_speed,
                    UCHAR * string_framework, ULONG string_framework_length,
                    UCHAR * language_id_framework, ULONG language_id_framework_length,
                    UINT (*ux_system_slave_change_function)(ULONG));
UINT    _ux_device_stack_interface_delete(UX_SLAVE_INTERFACE *ux_interface);
UINT    _ux_device_stack_interface_get(UINT interface_value);
UINT    _ux_device_stack_interface_set(UCHAR * device_framework, ULONG device_framework_length,
                    ULONG alternate_setting_value);
UINT    _ux_device_stack_interface_start(UX_SLAVE_INTERFACE *ux_interface);
UINT    _ux_device_stack_set_feature(ULONG request_type, ULONG request_value, ULONG request_index);
UINT    _ux_device_stack_transfer_all_request_abort(UX_SLAVE_ENDPOINT *endpoint, ULONG completion_code);
UINT    _ux_device_stack_transfer_request(UX_SLAVE_TRANSFER *transfer_request, ULONG slave_length, ULONG host_length);
UINT    _ux_device_stack_transfer_abort(UX_SLAVE_TRANSFER *transfer_request, ULONG completion_code);
UINT    _ux_device_stack_class_unregister(UCHAR *class_name, UINT (*class_entry_function)(struct UX_SLAVE_CLASS_COMMAND_STRUCT *));
UINT    _ux_device_stack_microsoft_extension_register(ULONG vendor_request, UINT (*vendor_request_function)(ULONG, ULONG, ULONG, ULONG, UCHAR *, ULONG *));
UINT    _ux_device_stack_uninitialize(VOID);

UINT    _ux_device_stack_tasks_run(VOID);
UINT    _ux_device_stack_transfer_run(UX_SLAVE_TRANSFER *transfer_request, ULONG slave_length, ULONG host_length);

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif

#endif

