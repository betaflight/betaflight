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
/*    _ux_host_class_hid_remote_control_entry             PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the entry point of the HID Remote Control client.  */
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
/*    _ux_host_class_hid_remote_control_activate                          */ 
/*                                              Activate HID RC class     */ 
/*    _ux_host_class_hid_remote_control_deactivate                        */ 
/*                                              Deactivate HID RC class   */ 
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
UINT  _ux_host_class_hid_remote_control_entry(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{

UINT        status;


    /* The command request will tell us we need to do here, either a enumeration
       query, an activation or a deactivation.  */
    switch (command -> ux_host_class_hid_client_command_request)
    {


    case UX_HOST_CLASS_COMMAND_QUERY:

        /* The query command is used to let the HID class know if we want to own
           this device or not */
        if ((command -> ux_host_class_hid_client_command_page == UX_HOST_CLASS_HID_PAGE_CONSUMER) &&
            (command -> ux_host_class_hid_client_command_usage == UX_HOST_CLASS_HID_CONSUMER_REMOTE_CONTROL))
            return(UX_SUCCESS);                        
        else            
            return(UX_NO_CLASS_MATCH);                        
           

    case UX_HOST_CLASS_COMMAND_ACTIVATE:

        /* The activate command is used by the HID class to start the HID client.  */
        status =  _ux_host_class_hid_remote_control_activate(command);

        /* Return completion status.  */
        return(status);


    case UX_HOST_CLASS_COMMAND_DEACTIVATE:

        /* The deactivate command is used by the HID class when it received a deactivate
           command from the USBX stack and there was a HID client attached to the HID instance */
        status =  _ux_host_class_hid_remote_control_deactivate(command);

        /* Return completion status.  */
        return(status);

#if defined(UX_HOST_STANDALONE)
    case UX_HOST_CLASS_COMMAND_ACTIVATE_WAIT:

        /* Nothing to do, just next state.  */
        return(UX_STATE_NEXT);
#endif


    default:
        break;
    }   

    /* Return error status.  */
    return(UX_ERROR);
}

