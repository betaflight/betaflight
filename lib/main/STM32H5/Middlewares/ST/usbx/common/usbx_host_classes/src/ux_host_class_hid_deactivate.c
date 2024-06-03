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
/*    _ux_host_class_hid_deactivate                       PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when this instance of the HID has been      */
/*    removed from the bus either directly or indirectly. The interrupt   */ 
/*    pipe will be destroyed and the instanced removed.                   */ 
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
/*    (ux_host_class_hid_client_handler)    HID client handler            */ 
/*    _ux_host_class_hid_instance_clean     HID instance clean            */ 
/*    _ux_host_stack_class_instance_destroy Destroy the class instance    */ 
/*    _ux_host_stack_endpoint_transfer_abort                              */
/*                                          Abort transfer                */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*    _ux_host_semaphore_delete             Delete semaphore              */ 
/*    _ux_host_semaphore_get                Get semaphore                 */ 
/*    _ux_utility_thread_schedule_other     Schedule other threads        */
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
UINT  _ux_host_class_hid_deactivate(UX_HOST_CLASS_COMMAND *command)
{

UX_HOST_CLASS_HID                   *hid;
UX_HOST_CLASS_HID_CLIENT_COMMAND    hid_client_command;
UX_TRANSFER                         *transfer_request;
#if !defined(UX_HOST_STANDALONE)
UINT                                status;
#endif


    /* Get the instance for this class.  */
    hid =  (UX_HOST_CLASS_HID *) command -> ux_host_class_command_instance;

    /* The HID is being shut down.  */
    hid -> ux_host_class_hid_state =  UX_HOST_CLASS_INSTANCE_SHUTDOWN;

#if !defined(UX_HOST_STANDALONE)

    /* Protect thread reentry to this instance.  */
    status =  _ux_host_semaphore_get(&hid -> ux_host_class_hid_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)

        /* Return error.  */
        return(status);
#endif

#if defined(UX_HOST_STANDALONE)
    if (hid -> ux_host_class_hid_interrupt_endpoint)
#endif
    {

        /* We need to abort transactions on the interrupt pipe.  */
        _ux_host_stack_endpoint_transfer_abort(hid -> ux_host_class_hid_interrupt_endpoint);

        /* If the Hid class instance has a interrupt pipe with a data payload associated with it
        it must be freed.  */
        transfer_request =  &hid -> ux_host_class_hid_interrupt_endpoint -> ux_endpoint_transfer_request;

        /* Then de allocate the memory.  */
        _ux_utility_memory_free(transfer_request -> ux_transfer_request_data_pointer);
    }

#if defined(UX_HOST_STANDALONE)
    if (hid -> ux_host_class_hid_allocated)
        _ux_utility_memory_free(hid -> ux_host_class_hid_allocated);
#endif

    /* We need to inform the HID client, if any, of the deactivation.  */
    hid_client_command.ux_host_class_hid_client_command_instance =   (VOID *) hid;
    hid_client_command.ux_host_class_hid_client_command_container =  (VOID *) hid -> ux_host_class_hid_class;
    hid_client_command.ux_host_class_hid_client_command_request =    UX_HOST_CLASS_COMMAND_DEACTIVATE;
    
    /* Call the HID client with a deactivate command if there was a client registered.  */
    if (hid -> ux_host_class_hid_client != UX_NULL)
        hid -> ux_host_class_hid_client -> ux_host_class_hid_client_handler(&hid_client_command);

    /* Clean all the HID memory fields.  */
    _ux_host_class_hid_instance_clean(hid);

    /* The enumeration thread needs to sleep a while to allow the application or the class that may be using
       endpoints to exit properly.  */
    _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 

    /* Destroy the instance.  */
    _ux_host_stack_class_instance_destroy(hid -> ux_host_class_hid_class, (VOID *) hid);

    /* Destroy the semaphore.  */
    _ux_host_semaphore_delete(&hid -> ux_host_class_hid_semaphore);

    /* Before we free the device resources, we need to inform the application
        that the device is removed.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {
        
        /* Inform the application the device is removed.  */
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_REMOVAL, hid -> ux_host_class_hid_class, (VOID *) hid);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_DEACTIVATE, hid, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(hid);

    /* The HID is now free again.  */
    _ux_utility_memory_free(hid);

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

