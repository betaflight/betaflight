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
/**  HUB Class                                                            */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hub.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hub_interrupt_endpoint_start         PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function search for the handle of the only interrupt endpoint  */
/*    in the default alternate setting of the HUB interface. The          */ 
/*    interrupt endpoint should always be there. When it is located, the  */
/*    first transfer for this endpoint is made. The HUB will report status*/
/*    changes on this pipe.                                               */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hub                                   Pointer to HUB class          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_interface_endpoint_get Get endpoint of interface     */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HUB Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            use pre-calculated value    */
/*                                            instead of wMaxPacketSize,  */
/*                                            resulting in version 6.1.9  */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added timeout value init,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hub_interrupt_endpoint_start(UX_HOST_CLASS_HUB *hub)
{

UINT            status;
UX_TRANSFER     *transfer_request;


    /* Search the interrupt endpoint. It is attached to the interface container.  */
    status =  _ux_host_stack_interface_endpoint_get(hub -> ux_host_class_hub_interface, 0, &hub -> ux_host_class_hub_interrupt_endpoint);

    /* Check completion status.  */
    if (status != UX_SUCCESS)
        return(status);
    
    /* Do a sanity check on the nature of the endpoint. Must be interrupt and its direction must be IN.  */        
    if (((hub -> ux_host_class_hub_interrupt_endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN) &&
        ((hub -> ux_host_class_hub_interrupt_endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT))
    {

        /* The endpoint is correct, fill in the transfer_request with the length requested for this endpoint.  */
        transfer_request =  &hub -> ux_host_class_hub_interrupt_endpoint -> ux_endpoint_transfer_request;
        transfer_request -> ux_transfer_request_requested_length = transfer_request -> ux_transfer_request_packet_length;
        transfer_request -> ux_transfer_request_actual_length =  0;

        /* Set timeout - wait forever.  */
        transfer_request -> ux_transfer_request_timeout_value = UX_WAIT_FOREVER;

        /* Since this transfer_request has a callback, we need the HUB instance to be stored in the transfer request.  */
        transfer_request -> ux_transfer_request_class_instance =  (VOID *) hub;

        /* This transfer request always has the IN direction.  */
        hub -> ux_host_class_hub_interrupt_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_IN;

        /* Interrupt transactions have a completion routine.  */
        transfer_request -> ux_transfer_request_completion_function =  _ux_host_class_hub_transfer_request_completed;

        /* Obtain a buffer for this transaction. The buffer will always be reused.  */
        transfer_request -> ux_transfer_request_data_pointer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 
                                                                transfer_request -> ux_transfer_request_requested_length);

        /* Check the memory pointer.  */
        if (transfer_request -> ux_transfer_request_data_pointer == UX_NULL)
        {

            /* Clear the interrupt endpoint.  */
            hub -> ux_host_class_hub_interrupt_endpoint = UX_NULL;

            /* Return error.  */
            return(UX_MEMORY_INSUFFICIENT);
        }

        /* Send the request to the stack.   */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Return completion status.  */
        return(status);
    }            
    
    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HUB, UX_ENDPOINT_HANDLE_UNKNOWN);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, hub -> ux_host_class_hub_interrupt_endpoint, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Return error.  */
    return(UX_ENDPOINT_HANDLE_UNKNOWN);
}
