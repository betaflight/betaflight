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


#if defined(UX_HOST_STANDALONE)

#define UX_HOST_CLASS_HID_MOUSE_ENUM_START           (UX_STATE_WAIT)
#define UX_HOST_CLASS_HID_MOUSE_ENUM_SET_IDLE        (UX_STATE_STACK_STEP + 1)
#define UX_HOST_CLASS_HID_MOUSE_ENUM_PERIODIC_START  (UX_STATE_STACK_STEP + 3)
#define UX_HOST_CLASS_HID_MOUSE_ENUM_DONE            (UX_STATE_STACK_STEP + 4)

static inline UINT _ux_host_class_hid_mouse_activate_wait(UX_HOST_CLASS_HID_CLIENT_COMMAND *command);
#endif


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_mouse_entry                      PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the entry point of the HID Mouse client. This      */ 
/*    function is called by the HID class after it has parsed a new HID   */
/*    report descriptor and is searching for a HID client.                */ 
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
/*    _ux_host_class_hid_mouse_activate     Activate HID mouse class      */ 
/*    _ux_host_class_hid_mouse_deactivate   Deactivate HID mouse class    */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Host Stack                                                          */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_mouse_entry(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{

UINT        status;


    /* The command request will tell us we need to do here, either a enumeration
       query, an activation or a deactivation.  */
    switch (command -> ux_host_class_hid_client_command_request)
    {


    case UX_HOST_CLASS_COMMAND_QUERY:

        /* The query command is used to let the HID class know if we want to own
           this device or not.  */
        if ((command -> ux_host_class_hid_client_command_page == UX_HOST_CLASS_HID_PAGE_GENERIC_DESKTOP_CONTROLS) &&
            (command -> ux_host_class_hid_client_command_usage == UX_HOST_CLASS_HID_GENERIC_DESKTOP_MOUSE))
            return(UX_SUCCESS);                        
        else            
            return(UX_NO_CLASS_MATCH);                        
                
    
    case UX_HOST_CLASS_COMMAND_ACTIVATE:

        /* The activate command is used by the HID class to start the HID client.  */
        status =  _ux_host_class_hid_mouse_activate(command);

        /* Return completion status.  */
        return(status);

#if defined(UX_HOST_STANDALONE)
    case UX_HOST_CLASS_COMMAND_ACTIVATE_WAIT:
        status = _ux_host_class_hid_mouse_activate_wait(command);
        return(status);
#endif


    case UX_HOST_CLASS_COMMAND_DEACTIVATE:

        /* The deactivate command is used by the HID class when it received a deactivate
           command from the USBX stack and there was a HID client attached to the HID instance.  */
        status =  _ux_host_class_hid_mouse_deactivate(command);

        /* Return completion status.  */
        return(status);
    }   

    /* Return error status.  */
    return(UX_ERROR);
}

#if defined(UX_HOST_STANDALONE)
static inline UINT _ux_host_class_hid_mouse_activate_wait(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{

UX_HOST_CLASS_HID                       *hid;
UX_HOST_CLASS_HID_CLIENT                *hid_client;
UX_HOST_CLASS_HID_MOUSE                 *mouse;
UINT                                    status;
UX_HOST_CLASS_HID_REPORT_CALLBACK       call_back;

    /* Get the instance to the HID class.  */
    hid = command -> ux_host_class_hid_client_command_instance;

    /* And of the HID client.  */
    hid_client = hid -> ux_host_class_hid_client;

    /* And of the mouse instance.  */
    mouse = (UX_HOST_CLASS_HID_MOUSE *) hid_client -> ux_host_class_hid_client_local_instance;

    /* Run states.  */
    switch(mouse -> ux_host_class_hid_mouse_enum_state)
    {
    case UX_HOST_CLASS_HID_MOUSE_ENUM_START:
        mouse -> ux_host_class_hid_mouse_enum_state = UX_HOST_CLASS_HID_MOUSE_ENUM_SET_IDLE;
        /* Fall through.  */
    case UX_HOST_CLASS_HID_MOUSE_ENUM_SET_IDLE:

        /* Run SET_IDLE states.  */
        status = _ux_host_class_hid_idle_set_run(hid, 0,
                            mouse -> ux_host_class_hid_mouse_id);
        if (status < UX_STATE_WAIT)
        {
            mouse -> ux_host_class_hid_mouse_status =
                                hid -> ux_host_class_hid_status;

            /* Set_Idle is optional, stall is accepted.  */
            if (mouse -> ux_host_class_hid_mouse_status == UX_TRANSFER_STALLED)
                mouse -> ux_host_class_hid_mouse_status = UX_SUCCESS;

            if (mouse -> ux_host_class_hid_mouse_status == UX_SUCCESS)
                mouse -> ux_host_class_hid_mouse_enum_state =
                                UX_HOST_CLASS_HID_MOUSE_ENUM_PERIODIC_START;
            else
                mouse -> ux_host_class_hid_mouse_enum_state =
                                UX_HOST_CLASS_HID_MOUSE_ENUM_DONE;
        }
        return(UX_STATE_WAIT);

    case UX_HOST_CLASS_HID_MOUSE_ENUM_PERIODIC_START:

        /* Set callback.  */
        call_back.ux_host_class_hid_report_callback_id =         mouse -> ux_host_class_hid_mouse_id;
        call_back.ux_host_class_hid_report_callback_function =   _ux_host_class_hid_mouse_callback;
        call_back.ux_host_class_hid_report_callback_buffer =     UX_NULL;
        call_back.ux_host_class_hid_report_callback_flags =      UX_HOST_CLASS_HID_REPORT_INDIVIDUAL_USAGE;
        call_back.ux_host_class_hid_report_callback_length =     0;
        status =  _ux_host_class_hid_report_callback_register(hid, &call_back);
        if (status != UX_SUCCESS)
        {
            mouse -> ux_host_class_hid_mouse_status = status;
            mouse -> ux_host_class_hid_mouse_enum_state =
                            UX_HOST_CLASS_HID_MOUSE_ENUM_DONE;
            break;
        }

        /* Start the periodic report.  */
        status =  _ux_host_class_hid_periodic_report_start(hid);
        mouse -> ux_host_class_hid_mouse_status = status;

        /* Fall through.  */
    case UX_HOST_CLASS_HID_MOUSE_ENUM_DONE:

        /* Anything failed, Free resources.  */
        if (mouse -> ux_host_class_hid_mouse_status != UX_SUCCESS)
        {

            /* Detach instance.  */
            hid_client -> ux_host_class_hid_client_local_instance = UX_NULL;

            /* Free instance.  */
            _ux_utility_memory_free(mouse);

            return(UX_STATE_ERROR);
        }

        /* The instance is live now.  */
        mouse -> ux_host_class_hid_mouse_state =  UX_HOST_CLASS_INSTANCE_LIVE;

        /* If all is fine and the device is mounted, we may need to inform the application
            if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {

            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_HID_CLIENT_INSERTION, hid -> ux_host_class_hid_class, (VOID *) hid_client);
        }

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_MOUSE_ACTIVATE, hid, mouse, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* Return completion status.  */
        return(UX_STATE_NEXT);

    default: /* IDLE, Other states.  */
        return(UX_STATE_NEXT);
    }

    /* By default, wait.  */
    return(UX_STATE_WAIT);
}
#endif
