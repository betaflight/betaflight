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
/**   PIMA Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_pima.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_pima_deactivate                      PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called when this instance of the pima has been     */
/*    removed from the bus either directly or indirectly. The bulk in\out */ 
/*    and interrupt pipes will be destroyed and the instance removed.     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                                PIMA   class command pointer */ 
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
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_pima_entry                Entry of pima class        */ 
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
UINT  _ux_host_class_pima_deactivate(UX_HOST_CLASS_COMMAND *command)
{

UX_HOST_CLASS_PIMA              *pima;
UX_HOST_CLASS_PIMA_SESSION      *pima_session;
UX_TRANSFER                     *transfer_request;
UINT                            status;
                            

    /* Get the instance for this class.  */
    pima =  (UX_HOST_CLASS_PIMA *) command -> ux_host_class_command_instance;

    /* The pima is being shut down.  */
    pima -> ux_host_class_pima_state =  UX_HOST_CLASS_INSTANCE_SHUTDOWN;

    /* Protect thread reentry to this instance.  */
    status =  _ux_host_semaphore_get(&pima -> ux_host_class_pima_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)

        /* Return error. */
        return(status);
    
    /* We come to this point when the device has been extracted. So there may have been a transaction
       being scheduled. We make sure the transaction has been completed by the controller driver.
       When the device is extracted, the controller tries multiple times the transaction and retires it
       with a DEVICE_NOT_RESPONDING error code.  
       
       First we take care of endpoint IN.  */
    transfer_request =  &pima -> ux_host_class_pima_bulk_in_endpoint -> ux_endpoint_transfer_request;
    if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)

        /* We need to abort transactions on the bulk In pipe.  */
        _ux_host_stack_endpoint_transfer_abort(pima -> ux_host_class_pima_bulk_in_endpoint);


    /* Then endpoint OUT.  */       
    transfer_request =  &pima -> ux_host_class_pima_bulk_out_endpoint -> ux_endpoint_transfer_request;
    if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)

        /* We need to abort transactions on the bulk Out pipe.  We normally don't need that anymore. */
        _ux_host_stack_endpoint_transfer_abort(pima -> ux_host_class_pima_bulk_out_endpoint);

       
    /* Then interrupt endpoint.  */
    transfer_request =  &pima -> ux_host_class_pima_interrupt_endpoint -> ux_endpoint_transfer_request;
    if (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_PENDING)
       
        /* We need to abort transactions on the Interrupt pipe.  */
        _ux_host_stack_endpoint_transfer_abort(pima -> ux_host_class_pima_interrupt_endpoint);
    
    /* The enumeration thread needs to sleep a while to allow the application or the class that may be using
       endpoints to exit properly.  */
    _ux_host_thread_schedule_other(UX_THREAD_PRIORITY_ENUM); 

    /* Free the header container buffer.  */
    if (pima -> ux_host_class_pima_container != UX_NULL)
        _ux_utility_memory_free(pima -> ux_host_class_pima_container);

    /* Free the event data buffer.  */
    if (pima -> ux_host_class_pima_event_buffer != UX_NULL)
        _ux_utility_memory_free(pima -> ux_host_class_pima_event_buffer);

    /* Get the pointer to the PIMA session.  */
    pima_session = pima -> ux_host_class_pima_session;

    /* Was there a PIMA session ?  */
    if(pima_session != UX_NULL)
    {
        /* Clean the PIMA session and free the storage ID buffer in the session if it is opened.  */
        if ((pima_session -> ux_host_class_pima_session_magic == UX_HOST_CLASS_PIMA_MAGIC_NUMBER) &&
            (pima_session -> ux_host_class_pima_session_state == UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED))

        {

            /* Reset the magic field.  */
            pima_session -> ux_host_class_pima_session_magic =  0;
            
            /* Declare the session closed.  */
            pima_session -> ux_host_class_pima_session_state = UX_HOST_CLASS_PIMA_SESSION_STATE_CLOSED;

        }                
    }

    /* Destroy the instance.  */
    _ux_host_stack_class_instance_destroy(pima -> ux_host_class_pima_class, (VOID *) pima);

    /* Destroy the semaphore.  */
    _ux_host_semaphore_delete(&pima -> ux_host_class_pima_semaphore);

    /* Before we free the device resources, we need to inform the application
        that the device is removed.  */
    if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
    {
        
        /* Inform the application the device is removed.  */
        _ux_system_host -> ux_system_host_change_function(UX_DEVICE_REMOVAL, pima -> ux_host_class_pima_class, (VOID *) pima);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_DEACTIVATE, pima, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(pima);

    /* Free the pima instance memory.  */
    _ux_utility_memory_free(pima);

    /* Return successful status.  */
    return(UX_SUCCESS);         
}

