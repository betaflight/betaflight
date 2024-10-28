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
/**   HID Mouse Client Class                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_mouse_activate                   PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs the enumeration of a HID mouse.              */ 
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
/*    _ux_host_class_hid_idle_set           Set idle rate                 */
/*    _ux_host_class_hid_report_callback_register                         */
/*                                          Register report callback      */
/*    _ux_host_class_hid_report_id_get      Get report ID                 */
/*    _ux_host_class_hid_periodic_report_start                            */
/*                                          Start periodic report         */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_free               Free memory block             */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Mouse Class                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed clients management,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_mouse_activate(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{

#if !defined(UX_HOST_STANDALONE)
UX_HOST_CLASS_HID_REPORT_CALLBACK       call_back;
#endif
UX_HOST_CLASS_HID_REPORT_GET_ID         report_id;
UX_HOST_CLASS_HID                       *hid;
UX_HOST_CLASS_HID_CLIENT                *hid_client;
UX_HOST_CLASS_HID_CLIENT_MOUSE          *client_mouse;
UX_HOST_CLASS_HID_MOUSE                 *mouse_instance;
UINT                                    status;


    /* Get the instance to the HID class.  */
    hid =  command -> ux_host_class_hid_client_command_instance;

    /* Get some memory for both the HID class instance and copy of this client
       and for the callback.  */
    client_mouse =  (UX_HOST_CLASS_HID_CLIENT_MOUSE *)
                    _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, 
                                        sizeof(UX_HOST_CLASS_HID_CLIENT_MOUSE));
    if(client_mouse == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Get client and mouse instance.  */
    mouse_instance = &client_mouse -> ux_host_class_hid_client_mouse_mouse;
    hid_client = &client_mouse -> ux_host_class_hid_client_mouse_client;
    _ux_utility_memory_copy(hid_client, hid -> ux_host_class_hid_client, sizeof(UX_HOST_CLASS_HID_CLIENT)); /* Use case of memcpy is verified. */

    /* Attach the mouse instance to the client instance.  */
    hid_client -> ux_host_class_hid_client_local_instance =  (VOID *) mouse_instance;

    /* Save the HID instance in the client instance.  */
    mouse_instance -> ux_host_class_hid_mouse_hid =  hid;

#if defined(UX_HOST_STANDALONE)

    /* The instance is mounting now.  */
    mouse_instance -> ux_host_class_hid_mouse_state =  UX_HOST_CLASS_INSTANCE_MOUNTING;

    /* Get the report ID for the mouse. The mouse is a INPUT report.
       This should be 0 but in case. */
    report_id.ux_host_class_hid_report_get_report = UX_NULL;
    report_id.ux_host_class_hid_report_get_type = UX_HOST_CLASS_HID_REPORT_TYPE_INPUT;
    status = _ux_host_class_hid_report_id_get(hid, &report_id);

    /* The report ID should exist.  */
    if (status == UX_SUCCESS)
    {

        /* Save the mouse report ID. */
        mouse_instance -> ux_host_class_hid_mouse_id = (USHORT)report_id.ux_host_class_hid_report_get_id;

        /* Set state for activate wait steps.  */
        mouse_instance -> ux_host_class_hid_mouse_enum_state = UX_STATE_WAIT;

    }

    /* Use our copy of client.  */
    hid -> ux_host_class_hid_client = hid_client;
    return(status);
#else

    /* The instance is live now.  */
    mouse_instance -> ux_host_class_hid_mouse_state =  UX_HOST_CLASS_INSTANCE_LIVE;

    /* Get the report ID for the mouse. The mouse is a INPUT report.
       This should be 0 but in case. */
    report_id.ux_host_class_hid_report_get_report = UX_NULL;
    report_id.ux_host_class_hid_report_get_type = UX_HOST_CLASS_HID_REPORT_TYPE_INPUT;
    status = _ux_host_class_hid_report_id_get(hid, &report_id);

    /* The report ID should exist.  */
    if (status == UX_SUCCESS)
    {

        /* Save the mouse report ID. */
        mouse_instance -> ux_host_class_hid_mouse_id = (USHORT)report_id.ux_host_class_hid_report_get_id;

        /* Set the idle rate of the mouse to 0. This way a report is generated only when there is an activity.  */
        status = _ux_host_class_hid_idle_set(hid, 0, mouse_instance -> ux_host_class_hid_mouse_id);

        /* Check for error, accept protocol error since it's optional for mouse.  */
        if (status == UX_TRANSFER_STALLED)
            status = UX_SUCCESS;
    }

    /* If we are OK, go on.  */
    if (status == UX_SUCCESS)
    {

        /* Initialize the report callback.  */
        call_back.ux_host_class_hid_report_callback_id =         mouse_instance -> ux_host_class_hid_mouse_id;
        call_back.ux_host_class_hid_report_callback_function =   _ux_host_class_hid_mouse_callback;
        call_back.ux_host_class_hid_report_callback_buffer =     UX_NULL;
        call_back.ux_host_class_hid_report_callback_flags =      UX_HOST_CLASS_HID_REPORT_INDIVIDUAL_USAGE;
        call_back.ux_host_class_hid_report_callback_length =     0;

        /* Register the report call back when data comes it on this report.  */
        status =  _ux_host_class_hid_report_callback_register(hid, &call_back);
    }

    /* If we are OK, go on.  */
    if (status == UX_SUCCESS)
    {

        /* Start the periodic report.  */
        status =  _ux_host_class_hid_periodic_report_start(hid);

        if (status == UX_SUCCESS)
        {

            /* Use our copy of client.  */
            hid -> ux_host_class_hid_client = hid_client;

            /* If all is fine and the device is mounted, we may need to inform the application
               if a function has been programmed in the system structure.  */
            if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
            {

                /* Call system change function.  */
                _ux_system_host ->  ux_system_host_change_function(UX_HID_CLIENT_INSERTION, hid -> ux_host_class_hid_class, (VOID *) hid_client);
            }

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_MOUSE_ACTIVATE, hid, mouse_instance, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

            /* Return completion status.  */
            return(status);
        }
    }

    /* We are here if there is error.  */

    /* Free mouse client instance.  */
    _ux_utility_memory_free(mouse_instance);

    /* Return completion status.  */
    return(status);    
#endif
}

