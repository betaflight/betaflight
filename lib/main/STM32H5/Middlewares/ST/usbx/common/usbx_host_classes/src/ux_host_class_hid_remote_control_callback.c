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
/**   HID Remote Control Class                                            */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_remote_control.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_remote_control_callback          PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the callback mechanism for a report registration.  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    callback                              Pointer to callback           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Remote Control Class                                            */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_hid_remote_control_callback(UX_HOST_CLASS_HID_REPORT_CALLBACK *callback)
{

UX_HOST_CLASS_HID_CLIENT            *hid_client;
UX_HOST_CLASS_HID_REMOTE_CONTROL    *remote_control_instance;
ULONG                               *array_head;
ULONG                               *array_tail;
ULONG                               *array_end;
ULONG                               *array_start;
ULONG                               *array_head_next;


    /* Get the HID client instance that issued the callback.  */
    hid_client = callback -> ux_host_class_hid_report_callback_client;

    /* Get the remote control local instance.  */
    remote_control_instance =  (UX_HOST_CLASS_HID_REMOTE_CONTROL *) hid_client -> ux_host_class_hid_client_local_instance;

    /* Load the remote control usage/value array info.  */
    array_start =  remote_control_instance -> ux_host_class_hid_remote_control_usage_array;
    array_end =    array_start + UX_HOST_CLASS_HID_REMOTE_CONTROL_USAGE_ARRAY_LENGTH;
    array_head =   remote_control_instance -> ux_host_class_hid_remote_control_usage_array_head;
    array_tail =   remote_control_instance -> ux_host_class_hid_remote_control_usage_array_tail;

    /* We have a single usage/value. We have to store it into the array. If the array overflows, 
       there is no mechanism for flow control here so we ignore the usage/value until the 
       applications makes more room in the array.  */

    /* Get position where next head will be.  */
    array_head_next =  array_head + 2;

    /* The head always points to where we will insert the next element. This
       is the reason for the wrap.  */
    if (array_head_next == array_end)
        array_head_next =  array_start;

    /* Do we have enough space to store the new usage?  */
    if (array_head_next != array_tail)
    {

        /* Yes, we have some space.  */
        *array_head =        callback -> ux_host_class_hid_report_callback_usage;
        *(array_head + 1) =  callback -> ux_host_class_hid_report_callback_value;

        /* Now update the array head.  */
        remote_control_instance -> ux_host_class_hid_remote_control_usage_array_head =  array_head_next;
    }
    else
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_REMOTE_CONTROL_CALLBACK, hid_client, remote_control_instance, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* Notify application.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_BUFFER_OVERFLOW);
    }

    /* Return to caller.  */
    return;    
}

