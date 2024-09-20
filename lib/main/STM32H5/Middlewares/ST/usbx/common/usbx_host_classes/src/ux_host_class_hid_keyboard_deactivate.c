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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_keyboard_deactivate              PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function performs the deactivation of a HID Keyboard Client.   */ 
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
/*    _ux_host_class_hid_periodic_report_stop                             */
/*                                          Stop periodic report          */ 
/*    _ux_utility_memory_free               Release memory block          */
/*    _ux_host_semaphore_delete             Delete semaphore              */
/*    _ux_utility_thread_delete             Delete thread                 */
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_keyboard_deactivate(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{

UX_HOST_CLASS_HID              *hid;
UX_HOST_CLASS_HID_CLIENT       *hid_client;
UX_HOST_CLASS_HID_KEYBOARD     *keyboard_instance;
UINT                            status = UX_SUCCESS;


    /* Get the instance to the HID class.  */
    hid =  command -> ux_host_class_hid_client_command_instance;

    /* Stop the periodic report.  */
    _ux_host_class_hid_periodic_report_stop(hid);

    /* Get the HID client pointer.  */
    hid_client =  hid -> ux_host_class_hid_client;

    /* Get the remote control local instance.  */
    keyboard_instance =  (UX_HOST_CLASS_HID_KEYBOARD *) hid_client -> ux_host_class_hid_client_local_instance;

#if !defined(UX_HOST_STANDALONE)

    /* Stop the semaphore.  */
    status =  _ux_host_semaphore_delete(&keyboard_instance -> ux_host_class_hid_keyboard_semaphore);

    /* Terminate the thread.  */
    _ux_utility_thread_delete(&keyboard_instance -> ux_host_class_hid_keyboard_thread);

    /* Return to the pool the thread stack.  */
    _ux_utility_memory_free(keyboard_instance -> ux_host_class_hid_keyboard_thread_stack);
#endif

    /* Free memory for key states.  */
    _ux_utility_memory_free(keyboard_instance -> ux_host_class_hid_keyboard_key_state);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_KEYBOARD_DEACTIVATE, hid, keyboard_instance, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Unload all the memory used by the keyboard client.  */
    _ux_utility_memory_free(keyboard_instance -> ux_host_class_hid_keyboard_usage_array);

    /* Now free the instance memory.  */
    _ux_utility_memory_free(hid_client -> ux_host_class_hid_client_local_instance);
    
    /* We may need to inform the application
       if a function has been programmed in the system structure.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {
        
        /* Call system change function.  */
        _ux_system_host ->  ux_system_host_change_function(UX_HID_CLIENT_REMOVAL, hid -> ux_host_class_hid_class, (VOID *) hid_client);
    }
    
    /* Return completion status.  */
    return(status);    
}

