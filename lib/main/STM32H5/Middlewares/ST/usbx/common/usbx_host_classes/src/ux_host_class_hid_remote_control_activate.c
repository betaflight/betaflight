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
/*    _ux_host_class_hid_remote_control_activate          PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs the enumeration of a HID Remote Control      */ 
/*    class.                                                              */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to command            */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hid_periodic_report_start    Start periodic report   */ 
/*    _ux_host_class_hid_report_callback_register Register callback       */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed clients management,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_remote_control_activate(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{

UX_HOST_CLASS_HID_REPORT_CALLBACK        call_back;
UX_HOST_CLASS_HID                        *hid;
UX_HOST_CLASS_HID_CLIENT                 *hid_client;
UX_HOST_CLASS_HID_CLIENT_REMOTE_CONTROL  *client_remote_control;
UX_HOST_CLASS_HID_REMOTE_CONTROL         *remote_control_instance;
UINT                                     status = UX_SUCCESS;


    /* Get the instance to the HID class.  */
    hid =  command -> ux_host_class_hid_client_command_instance;

    /* Get some memory for both the HID class instance and copy of this client
       and for the callback.  */
    client_remote_control =  (UX_HOST_CLASS_HID_CLIENT_REMOTE_CONTROL *)
                    _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, 
                                sizeof(UX_HOST_CLASS_HID_CLIENT_REMOTE_CONTROL));
    if (client_remote_control == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Get client instance and client copy.  */
    remote_control_instance = &client_remote_control -> ux_host_class_hid_client_remote_control_remote_control;
    hid_client = &client_remote_control -> ux_host_class_hid_client_remote_control_client;
    _ux_utility_memory_copy(hid_client, hid -> ux_host_class_hid_client, sizeof(UX_HOST_CLASS_HID_CLIENT)); /* Use case of memcpy is verified. */

    /* Attach the remote control instance to the client instance.  */
    hid_client -> ux_host_class_hid_client_local_instance =  (VOID *) remote_control_instance;

    /* Save the HID instance in the client instance.  */
    remote_control_instance -> ux_host_class_hid_remote_control_hid =  hid;

    /* The instance is live now.  */
    remote_control_instance -> ux_host_class_hid_remote_control_state =  UX_HOST_CLASS_INSTANCE_LIVE;

    /* Allocate the round-robin buffer that the remote control instance will use
     * to store the usages as they come in.
     * Size calculation overflow is checked near where _USAGE_ARRAY_LENGTH is defined.
     */
    remote_control_instance -> ux_host_class_hid_remote_control_usage_array =  (ULONG *)
                            _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, 
                                                        UX_HOST_CLASS_HID_REMOTE_CONTROL_USAGE_ARRAY_LENGTH*4);

    /* Check memory pointer. */
    if (remote_control_instance -> ux_host_class_hid_remote_control_usage_array == UX_NULL)
        status = (UX_MEMORY_INSUFFICIENT);

    /* If there is no error, go on.  */
    if (status == UX_SUCCESS)
    {

        /* Initialize the head and tail of this array.  */
        remote_control_instance -> ux_host_class_hid_remote_control_usage_array_head =  remote_control_instance -> ux_host_class_hid_remote_control_usage_array;
        remote_control_instance -> ux_host_class_hid_remote_control_usage_array_tail =  remote_control_instance -> ux_host_class_hid_remote_control_usage_array;

        /* Initialize the report callback.  */
        call_back.ux_host_class_hid_report_callback_id =         0;
        call_back.ux_host_class_hid_report_callback_function =   _ux_host_class_hid_remote_control_callback;
        call_back.ux_host_class_hid_report_callback_buffer =     UX_NULL;
        call_back.ux_host_class_hid_report_callback_flags =      UX_HOST_CLASS_HID_REPORT_INDIVIDUAL_USAGE;
        call_back.ux_host_class_hid_report_callback_length =     0;

        /* Register the report call back when data comes in on this report.  */
        status =  _ux_host_class_hid_report_callback_register(hid, &call_back);
    }

    /* If there is no error, go on.  */
    if (status == UX_SUCCESS)
    {

        /* Start the periodic report.  */
        status =  _ux_host_class_hid_periodic_report_start(hid);

        /* If OK, we are done.  */
        if (status == UX_SUCCESS)
        {

            /* Use out copy of client.  */
            hid -> ux_host_class_hid_client = hid_client;

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_REMOTE_CONTROL_ACTIVATE, hid, remote_control_instance, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

            /* If all is fine and the device is mounted, we may need to inform the application
            if a function has been programmed in the system structure.  */
            if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            {

                /* Call system change function.  */
                _ux_system_host ->  ux_system_host_change_function(UX_HID_CLIENT_INSERTION, hid -> ux_host_class_hid_class, (VOID *) hid_client);
            }

            /* We are done success.  */
            return (UX_SUCCESS);
        }
    }

    /* We are here when there is error.  */

    /* Free usage array.  */
    if (remote_control_instance -> ux_host_class_hid_remote_control_usage_array)
        _ux_utility_memory_free(remote_control_instance -> ux_host_class_hid_remote_control_usage_array);

    /* Free instance memory.  */
    _ux_utility_memory_free(remote_control_instance);

    /* Return completion status.  */
    return(status);    
}

