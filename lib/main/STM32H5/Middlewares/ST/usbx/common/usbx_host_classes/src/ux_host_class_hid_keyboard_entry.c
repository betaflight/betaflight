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
/**   HID Keyboard Client                                                 */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_keyboard.h"
#include "ux_host_stack.h"


#if defined(UX_HOST_STANDALONE)

#define UX_HOST_CLASS_HID_KEYBOARD_ENUM_START           (UX_STATE_WAIT)
#define UX_HOST_CLASS_HID_KEYBOARD_ENUM_SET_REPORT      (UX_STATE_STACK_STEP + 0)
#define UX_HOST_CLASS_HID_KEYBOARD_ENUM_SET_IDLE        (UX_STATE_STACK_STEP + 1)
#define UX_HOST_CLASS_HID_KEYBOARD_ENUM_CMD_WAIT        (UX_STATE_STACK_STEP + 2)
#define UX_HOST_CLASS_HID_KEYBOARD_ENUM_PERIODIC_START  (UX_STATE_STACK_STEP + 3)
#define UX_HOST_CLASS_HID_KEYBOARD_ENUM_DONE            (UX_STATE_STACK_STEP + 4)


static inline UINT  _ux_host_class_hid_keyboard_activate_wait(UX_HOST_CLASS_HID_CLIENT_COMMAND *command);
#endif


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_keyboard_entry                   PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the entry point of the HID keyboard client.        */
/*    This function is called by the HID class after it has parsed a new  */ 
/*    HID report descriptor and is searching for a HID client.            */ 
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
/*    _ux_host_class_hid_keyboard_activate                                */ 
/*                                              Activate HID keyboard     */ 
/*    _ux_host_class_hid_keyboard_deactivate                              */ 
/*                                              Deactivate HID keyboard   */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class                                                           */ 
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
UINT  _ux_host_class_hid_keyboard_entry(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{

UINT        status;


    /* The command request will tell us we need to do here, either a enumeration
       query, an activation or a deactivation.  */
    switch (command -> ux_host_class_hid_client_command_request)
    {


    case UX_HOST_CLASS_COMMAND_QUERY:

        /* The query command is used to let the HID class know if we want to own
           this device or not */
        if ((command -> ux_host_class_hid_client_command_page == UX_HOST_CLASS_HID_PAGE_GENERIC_DESKTOP_CONTROLS) &&
            (command -> ux_host_class_hid_client_command_usage == UX_HOST_CLASS_HID_GENERIC_DESKTOP_KEYBOARD))
            return(UX_SUCCESS);                        
        else            
            return(UX_NO_CLASS_MATCH);                        
           

    case UX_HOST_CLASS_COMMAND_ACTIVATE:

        /* The activate command is used by the HID class to start the HID client.  */
        status =  _ux_host_class_hid_keyboard_activate(command);

        /* Return completion status.  */
        return(status);


#if defined(UX_HOST_STANDALONE)
    case UX_HOST_CLASS_COMMAND_ACTIVATE_WAIT:

        return(_ux_host_class_hid_keyboard_activate_wait(command));
#endif


    case UX_HOST_CLASS_COMMAND_DEACTIVATE:

        /* The deactivate command is used by the HID class when it received a deactivate
           command from the USBX stack and there was a HID client attached to the HID instance */
        status =  _ux_host_class_hid_keyboard_deactivate(command);

        /* Return completion status.  */
        return(status);
    }   

    /* Return error status.  */
    return(UX_ERROR);
}

#if defined(UX_HOST_STANDALONE)
static inline UINT _ux_host_class_hid_keyboard_activate_wait(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{

UX_HOST_CLASS_HID                       *hid;
UX_HOST_CLASS_HID_CLIENT                *hid_client;
UX_HOST_CLASS_HID_KEYBOARD              *keyboard;
UINT                                    status;

    /* Get the instance to the HID class.  */
    hid = command -> ux_host_class_hid_client_command_instance;

    /* And of the HID client.  */
    hid_client = hid -> ux_host_class_hid_client;

    /* And of the keyboard instance.  */
    keyboard = (UX_HOST_CLASS_HID_KEYBOARD *) hid_client -> ux_host_class_hid_client_local_instance;

    /* Run states.  */
    switch(keyboard -> ux_host_class_hid_keyboard_enum_state)
    {
    case UX_HOST_CLASS_HID_KEYBOARD_ENUM_START:
    /* Fall through.  */
    case UX_HOST_CLASS_HID_KEYBOARD_ENUM_SET_REPORT:

        /* Update LED in task.  */
        keyboard -> ux_host_class_hid_keyboard_out_state = UX_STATE_WAIT;
        keyboard -> ux_host_class_hid_keyboard_enum_state =
                                    UX_HOST_CLASS_HID_KEYBOARD_ENUM_CMD_WAIT;
        keyboard -> ux_host_class_hid_keyboard_next_state =
                                    UX_HOST_CLASS_HID_KEYBOARD_ENUM_SET_IDLE;
        return(UX_STATE_WAIT);

    case UX_HOST_CLASS_HID_KEYBOARD_ENUM_CMD_WAIT:

        /* LEDs processed in background task.  */
        if (keyboard -> ux_host_class_hid_keyboard_out_state != UX_STATE_WAIT)
        {
            if (keyboard -> ux_host_class_hid_keyboard_status != UX_SUCCESS)
                keyboard -> ux_host_class_hid_keyboard_enum_state =
                                        UX_HOST_CLASS_HID_KEYBOARD_ENUM_DONE;
            else
                keyboard -> ux_host_class_hid_keyboard_enum_state =
                            keyboard -> ux_host_class_hid_keyboard_next_state;
        }
        return(UX_STATE_WAIT);

    case UX_HOST_CLASS_HID_KEYBOARD_ENUM_SET_IDLE:

        /* Run SET_IDLE states.  */
        status = _ux_host_class_hid_idle_set_run(hid, 0,
                            keyboard -> ux_host_class_hid_keyboard_id);
        if (status < UX_STATE_WAIT)
        {
            keyboard -> ux_host_class_hid_keyboard_status =
                                hid -> ux_host_class_hid_status;

            /* Set_Idle is mandatory, if there is error enum fail.  */
            if (keyboard -> ux_host_class_hid_keyboard_status == UX_SUCCESS)
                keyboard -> ux_host_class_hid_keyboard_enum_state =
                                UX_HOST_CLASS_HID_KEYBOARD_ENUM_PERIODIC_START;
            else
                keyboard -> ux_host_class_hid_keyboard_enum_state =
                                UX_HOST_CLASS_HID_KEYBOARD_ENUM_DONE;

        }
        return(UX_STATE_WAIT);

    case UX_HOST_CLASS_HID_KEYBOARD_ENUM_PERIODIC_START:

        /* Start the periodic report.  */
        status =  _ux_host_class_hid_periodic_report_start(hid);
        keyboard -> ux_host_class_hid_keyboard_status = status;

        /* Fall through.  */
    case UX_HOST_CLASS_HID_KEYBOARD_ENUM_DONE:

        /* Anything failed, Free resources.  */
        if (keyboard -> ux_host_class_hid_keyboard_status != UX_SUCCESS)
        {

            /* Detach instance.  */
            hid_client -> ux_host_class_hid_client_local_instance = UX_NULL;

            /* Free usage state.  */
            if (keyboard -> ux_host_class_hid_keyboard_key_state)
                _ux_utility_memory_free(keyboard -> ux_host_class_hid_keyboard_key_state);

            /* Free usage array.  */
            if (keyboard -> ux_host_class_hid_keyboard_usage_array)
                _ux_utility_memory_free(keyboard -> ux_host_class_hid_keyboard_usage_array);

            /* Free instance.  */
            _ux_utility_memory_free(keyboard);

            return(UX_STATE_ERROR);
        }

        /* Now keyboard instance is live.  */
        keyboard -> ux_host_class_hid_keyboard_state =  UX_HOST_CLASS_INSTANCE_LIVE;

        /* If all is fine and the device is mounted, we may need to inform the application
            if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {
            
            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_HID_CLIENT_INSERTION, hid -> ux_host_class_hid_class, (VOID *) hid_client);
        }

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_KEYBOARD_ACTIVATE, hid, keyboard, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* Enumeration idle.  */
        keyboard -> ux_host_class_hid_keyboard_enum_state = UX_STATE_IDLE;
        return(UX_STATE_NEXT);

    default: /* IDLE, Other states.  */
        return(UX_STATE_NEXT);
    }
}
#endif
