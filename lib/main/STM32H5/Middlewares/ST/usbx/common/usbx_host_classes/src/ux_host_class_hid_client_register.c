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

UX_COMPILE_TIME_ASSERT(!UX_OVERFLOW_CHECK_MULC_ULONG(UX_HOST_CLASS_HID_MAX_CLIENTS, sizeof(UX_HOST_CLASS_HID_CLIENT)), UX_HOST_CLASS_HID_MAX_CLIENTS_mem_alloc_ovf)

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_client_register                  PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function registers a USB HID client to the HID class. The      */ 
/*    mechanism is similar to the USB stack class registration. The Class */ 
/*    must specify an entry point for the USB stack to send commands      */ 
/*    such as:                                                            */
/*                                                                        */
/*          UX_HOST_CLASS_COMMAND_QUERY                                   */
/*          UX_HOST_CLASS_COMMAND_ACTIVATE                                */
/*          UX_HOST_CLASS_COMMAND_DESTROY                                 */ 
/*                                                                        */ 
/*    Note: The C string of hid_client_name must be NULL-terminated and   */
/*    the length of it (without the NULL-terminator itself) must be no    */
/*    larger than UX_HOST_CLASS_HID_MAX_CLIENT_NAME_LENGTH.               */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid_client_name                       Name of HID client            */
/*    hid_client_handler                    Handler for HID client        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_get              Get class                     */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_copy               Copy memory block             */ 
/*    _ux_utility_string_length_check       Check C string and return     */
/*                                          length if null-terminated     */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    HID Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_client_register(UCHAR *hid_client_name,
                                UINT (*hid_client_handler)(struct UX_HOST_CLASS_HID_CLIENT_COMMAND_STRUCT *))
{

UX_HOST_CLASS               *class_ptr;
ULONG                       hid_client_index;
UINT                        status;
UX_HOST_CLASS_HID_CLIENT    *hid_client;
UINT                        client_name_length =  0;
                            
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_CLIENT_REGISTER, hid_client_name, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Get the length of the client name (exclude null-terminator).  */
    status =  _ux_utility_string_length_check(hid_client_name, &client_name_length, UX_HOST_CLASS_HID_MAX_CLIENT_NAME_LENGTH);
    if (status)
        return(status);

    /* We need to locate our class container.  */
    status =  _ux_host_stack_class_get(_ux_system_host_class_hid_name, &class_ptr);

    /* If we cannot get the class container, it means the HID class was not registered.  */
    if (status != UX_SUCCESS)
        return(status);

    /* From the class container, we get the client pointer which has the list of 
       HID clients. If the pointer is NULL, the client list was not assigned.  */
    if (class_ptr -> ux_host_class_client == UX_NULL)
    {

        /* Allocate memory for the class client.
         * Allocate size overflow static checked outside the function.
         */
        class_ptr -> ux_host_class_client =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, 
                                            sizeof(UX_HOST_CLASS_HID_CLIENT)*UX_HOST_CLASS_HID_MAX_CLIENTS);
        
        /* Check for successful allocation.  */
        if (class_ptr -> ux_host_class_client == UX_NULL)
            return(UX_MEMORY_INSUFFICIENT);
    }

    /* De-reference the client pointer into a HID client array pointer.  */
    hid_client =  (UX_HOST_CLASS_HID_CLIENT *) class_ptr -> ux_host_class_client;

    /* We need to parse the HID client handler table to find an empty spot.  */
    for (hid_client_index = 0; hid_client_index < UX_HOST_CLASS_HID_MAX_CLIENTS; hid_client_index++)
    {

        /* Check if this HID client is already used. */
        if (hid_client -> ux_host_class_hid_client_status == UX_UNUSED)
        {

            /* We have found a free container for the HID client. Copy the name (with null-terminator). */
            _ux_utility_memory_copy(hid_client -> ux_host_class_hid_client_name, hid_client_name, client_name_length + 1); /* Use case of memcpy is verified. */
            
            /* Memorize the handler address of this client. */
            hid_client -> ux_host_class_hid_client_handler =  hid_client_handler;

            /* Mark it as being in use.  */
            hid_client -> ux_host_class_hid_client_status =  UX_USED;

            /* Return successful completion.  */
            return(UX_SUCCESS);
        }
        else
        {

            /* Do a sanity check to make sure the handler is not already installed by
               mistake. To verify this, we simple check for the handler entry point.  */
               if (hid_client -> ux_host_class_hid_client_handler == hid_client_handler)
            {

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_ALREADY_INSTALLED);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_ALREADY_INSTALLED, hid_client_name, 0, 0, UX_TRACE_ERRORS, 0, 0)

                return(UX_HOST_CLASS_ALREADY_INSTALLED);
            }
        }

        /* Try the next class.  */
        hid_client++;
    }    

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_ARRAY_FULL);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_ARRAY_FULL, hid_client_name, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* No more entries in the class table.  */
    return(UX_MEMORY_ARRAY_FULL);
}

