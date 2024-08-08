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
/**   HID Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_client_search                    PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function searches for a HID client that wants to own this HID  */
/*    device.                                                             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_host_class_hid_client_handler)    HID client handler            */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_client_search(UX_HOST_CLASS_HID *hid)
{

UX_HOST_CLASS_HID_CLIENT            *hid_client;
ULONG                               hid_client_index;
UINT                                status;
UX_HOST_CLASS_HID_CLIENT_COMMAND    hid_client_command;


    /* In order to search a HID client, we need both the main page and the main usage.  */
    hid_client_command.ux_host_class_hid_client_command_page =       hid -> ux_host_class_hid_parser.ux_host_class_hid_parser_main_page;
    hid_client_command.ux_host_class_hid_client_command_usage =      hid -> ux_host_class_hid_parser.ux_host_class_hid_parser_main_usage & 0xffff;
    hid_client_command.ux_host_class_hid_client_command_instance =   hid;
    hid_client_command.ux_host_class_hid_client_command_container =  (VOID *) hid -> ux_host_class_hid_class;
    hid_client_command.ux_host_class_hid_client_command_request =    UX_HOST_CLASS_COMMAND_QUERY;
        
    /* Dereference the client pointer into a HID client pointer.  */
    hid_client =  (UX_HOST_CLASS_HID_CLIENT *) hid -> ux_host_class_hid_class -> ux_host_class_client;
    
    /* If the hid_client pointer is NULL, the array of clients was not initialized.  */
    if (hid_client == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_UNKNOWN, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_HID_UNKNOWN);
    }
            
    /* We need to parse the hid client driver table to find a registered client.  */
    for (hid_client_index = 0; hid_client_index < UX_HOST_CLASS_HID_MAX_CLIENTS; hid_client_index++)
    {

        /* Check if this HID client is registered. */
        if (hid_client -> ux_host_class_hid_client_status == UX_USED)
        {

            /* Call the HID client with a query command.  */
            status =  hid_client -> ux_host_class_hid_client_handler(&hid_client_command);
    
            /* Have we found a HID client?  */
            if (status == UX_SUCCESS)
            {

                /* Update the command to activate the client.  */
                hid_client_command.ux_host_class_hid_client_command_request =  UX_HOST_CLASS_COMMAND_ACTIVATE;

                /* Memorize the client for this HID device.  */
                hid -> ux_host_class_hid_client =  hid_client;

                /* Call the HID client with an activate command.  */
                status =  hid_client -> ux_host_class_hid_client_handler(&hid_client_command);

                /* Unmount the client if activation fail. */
                if (status != UX_SUCCESS)
                    hid -> ux_host_class_hid_client = UX_NULL;
        
                /* Return completion status.  */
                return(status);
            }
        }

        /* Try the next HID client.  */
        hid_client++;
    }    

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_HID_UNKNOWN);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_HID_UNKNOWN, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Return an error.  */
    return(UX_HOST_CLASS_HID_UNKNOWN);
}

