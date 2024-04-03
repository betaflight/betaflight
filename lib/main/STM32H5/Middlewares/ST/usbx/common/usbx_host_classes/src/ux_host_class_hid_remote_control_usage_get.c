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
/*    _ux_host_class_hid_remote_control_usage_get         PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function reads the usage and value from the remote control     */ 
/*    round-robin buffer.                                                 */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    remote_control_instance               Pointer to remote control     */ 
/*    usage                                 Pointer to usage              */ 
/*    value                                 Pointer to value              */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
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
UINT  _ux_host_class_hid_remote_control_usage_get(UX_HOST_CLASS_HID_REMOTE_CONTROL *remote_control_instance, ULONG *usage, ULONG *value)
{

ULONG               *array_head;
ULONG               *array_tail;
ULONG               *array_end;
ULONG               *array_start;
UX_HOST_CLASS_HID   *hid;

    /* Get the HID class associated with the HID client. */
    hid = remote_control_instance -> ux_host_class_hid_remote_control_hid;
    
    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_hid_name, (VOID *) hid) != UX_SUCCESS)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Load the remote control usage/value array info.  */
    array_start =  remote_control_instance -> ux_host_class_hid_remote_control_usage_array;
    array_end =    array_start + UX_HOST_CLASS_HID_REMOTE_CONTROL_USAGE_ARRAY_LENGTH;
    array_head =   remote_control_instance -> ux_host_class_hid_remote_control_usage_array_head;
    array_tail =   remote_control_instance -> ux_host_class_hid_remote_control_usage_array_tail;

    /* We want to extract a usage/value from the circular queue of the remote control instance.  */
    if (array_tail == array_head)
        return(UX_ERROR);

    /* Get the usage/value from the current tail.  */
    *usage =  *array_tail;
    *value =  *(array_tail + 1);

    /* Now we need to update the tail value. Are we at the end of the array?  */
    if ((array_tail+2) >= array_end)
        array_tail =  array_start;
    else
        array_tail += 2;

    /* Set the tail pointer.  */
    remote_control_instance -> ux_host_class_hid_remote_control_usage_array_tail =  array_tail;

    /* The status will tell the application there is something valid in the usage/value.  */
    return(UX_SUCCESS);    
}

