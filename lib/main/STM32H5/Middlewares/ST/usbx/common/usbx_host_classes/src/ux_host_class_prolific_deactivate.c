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
/**   Prolific Class                                                      */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_prolific.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_prolific_deactivate                  PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when this instance of the prolific has been */
/*    removed from the bus either directly or indirectly. The bulk in\out */ 
/*    and optional interrupt pipes will be destroyed and the instance     */ 
/*    removed.                                                            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                              Prolific class command pointer */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_instance_destroy Destroy the class instance    */ 
/*    _ux_host_stack_endpoint_transfer_abort Abort endpoint transfer      */ 
/*    _ux_utility_memory_free               Free memory block             */ 
/*    _ux_host_semaphore_get                Get protection semaphore      */ 
/*    _ux_host_semaphore_delete             Delete protection semaphore   */ 
/*    _ux_utility_thread_schedule_other     Schedule other threads        */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_prolific_entry         Entry of prolific class       */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_prolific_deactivate(UX_HOST_CLASS_COMMAND *command)
{

UX_HOST_CLASS_PROLIFIC      *prolific;
UX_TRANSFER                 *transfer_request;
UINT                        status;


    /* Get the instance for this class.  */
    prolific =  (UX_HOST_CLASS_PROLIFIC *) command -> ux_host_class_command_instance;

    /* The prolific is being shut down.  */
    prolific -> ux_host_class_prolific_state =  UX_HOST_CLASS_INSTANCE_SHUTDOWN;

    /* Protect thread reentry to this instance.  */
    status =  _ux_host_semaphore_get(&prolific -> ux_host_class_prolific_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)

        /* Return error.  */
        return(status);

    /* If the interrupt endpoint is defined, clean any pending transfer.  */
    if (prolific -> ux_host_class_prolific_interrupt_endpoint != UX_NULL)
    {    
        
        /* Wait for any current transfer to be out of pending.  */
        transfer_request =  &prolific -> ux_host_class_prolific_interrupt_endpoint -> ux_endpoint_transfer_request;
        if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)

            /* And abort any transfer.  */
            _ux_host_stack_endpoint_transfer_abort(prolific -> ux_host_class_prolific_interrupt_endpoint);
        
        /* And free the memory used by the interrupt endpoint.  */
        _ux_utility_memory_free(transfer_request -> ux_transfer_request_data_pointer);
        
    }
    
           
    /* First we take care of cleaning endpoint IN.  */
    transfer_request =  &prolific -> ux_host_class_prolific_bulk_in_endpoint -> ux_endpoint_transfer_request;
    if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)

        /* We need to abort transactions on the bulk In pipe.  */
        _ux_host_stack_endpoint_transfer_abort(prolific -> ux_host_class_prolific_bulk_in_endpoint);
    
    
    /* Then endpoint OUT.  */       
    transfer_request =  &prolific -> ux_host_class_prolific_bulk_out_endpoint -> ux_endpoint_transfer_request;
    if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)

        /* We need to abort transactions on the bulk Out pipe. */
        _ux_host_stack_endpoint_transfer_abort(prolific -> ux_host_class_prolific_bulk_out_endpoint);
    

    /* The enumeration thread needs to sleep a while to allow the application or the class that may be using
       endpoints to exit properly.  */
    _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 

    /* Destroy the instance.  */
    _ux_host_stack_class_instance_destroy(prolific -> ux_host_class_prolific_class, (VOID *) prolific);

    /* Destroy the semaphore.  */
    _ux_host_semaphore_delete(&prolific -> ux_host_class_prolific_semaphore);

    /* Before we free the device resources, we need to inform the application
        that the device is removed.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {
        
        /* Inform the application the device is removed.  */
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_REMOVAL, prolific -> ux_host_class_prolific_class, (VOID *) prolific);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_DEACTIVATE, prolific, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(prolific);

    /* Free the prolific instance memory.  */
    _ux_utility_memory_free(prolific);

    /* Return successful status.  */
    return(UX_SUCCESS);         
}

