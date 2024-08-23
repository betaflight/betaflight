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
/**   Storage Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_storage.h"
#include "ux_host_stack.h"


#if !defined(UX_HOST_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_storage_transport_cbi                PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the transport layer for the Control/Bulk/Interrupt */
/*    transport. The command is sent on the control endpoint, the data    */ 
/*    payload on the bulk endpoint. The status from the command is        */ 
/*    returned by the interrupt endpoint. This transport is mainly used   */ 
/*    by storage devices that have very slow read/write commands.         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    storage                               Pointer to storage class      */ 
/*    data_pointer                          Pointer to data               */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process host stack transfer   */ 
/*    _ux_host_stack_transfer_request_abort Abort transfer request        */ 
/*    _ux_utility_long_get                  Get 32-bit word               */ 
/*    _ux_host_semaphore_get                Get semaphore                 */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Storage Class                                                       */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            prefixed UX to MS_TO_TICK,  */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed CBI request index,    */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_transport_cbi(UX_HOST_CLASS_STORAGE *storage, UCHAR *data_pointer)
{

UX_TRANSFER     *transfer_request;
UINT            status;
UCHAR           *ufi;
UCHAR           *cbw;
ULONG           data_phase_requested_length;
UX_ENDPOINT     *control_endpoint;


    /* Reset the data phase memory size.  */
    storage -> ux_host_class_storage_data_phase_length =  0;

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &storage -> ux_host_class_storage_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Initialize the transfer request with the control SETUP values.  */
    transfer_request -> ux_transfer_request_data_pointer =      UX_NULL;
    transfer_request -> ux_transfer_request_requested_length =  0;
    transfer_request -> ux_transfer_request_function =          0;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =             0;
    transfer_request -> ux_transfer_request_index =             storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceNumber;
    
    /* Use a pointer for the ufi portion of the command.  */
    cbw =  (UCHAR *) storage -> ux_host_class_storage_cbw;
    ufi =  cbw + UX_HOST_CLASS_STORAGE_CBW_CB;

    /* Fill in the transfer_request parameters.  */
    transfer_request -> ux_transfer_request_data_pointer =      ufi;
    transfer_request -> ux_transfer_request_requested_length =  (ULONG)*(cbw+UX_HOST_CLASS_STORAGE_CBW_CB_LENGTH);

    /* Send the ufi block on the control endpoint.  */
    _ux_host_stack_transfer_request(transfer_request);

    /* Check the transfer status. If there is a transport error, the host must perform
       a reset recovery.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
        return(transfer_request -> ux_transfer_request_completion_code);
    
    /* Get the length of the data payload.  */    
    data_phase_requested_length =  _ux_utility_long_get(cbw+UX_HOST_CLASS_STORAGE_CBW_DATA_LENGTH);
    
    /* Perform the data stage - if there is any.  */    
    if (data_phase_requested_length != 0)
    {

        /* Check the direction and determine which endpoint to use.  */
        if (*(cbw+UX_HOST_CLASS_STORAGE_CBW_FLAGS) == UX_HOST_CLASS_STORAGE_DATA_IN)
            transfer_request =  &storage -> ux_host_class_storage_bulk_in_endpoint -> ux_endpoint_transfer_request;
        else            
            transfer_request =  &storage -> ux_host_class_storage_bulk_out_endpoint -> ux_endpoint_transfer_request;

        /* Fill in the transfer_request data payload buffer.  */
        transfer_request -> ux_transfer_request_data_pointer =  data_pointer;
        
        /* Store the requested length in the transfer request.  */
        transfer_request -> ux_transfer_request_requested_length =  data_phase_requested_length;

        /* Perform data payload transfer (in or out).  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check the status of the data payload.  */
        if (status != UX_SUCCESS)
            return(status);

        /* Wait for the completion of the transfer request.  */
        status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_HOST_CLASS_STORAGE_TRANSFER_TIMEOUT));

        /* Get the actual transfer length and update the cumulated stored value for upper layers.  
           This could be a non complete packet. But we don't test here because it only matters for
           sector read.  */
        storage -> ux_host_class_storage_data_phase_length += transfer_request -> ux_transfer_request_actual_length;

        /* If the semaphore did not succeed we probably have a time out.  */
        if (status != UX_SUCCESS)
        {

            /* All transfers pending need to abort. There may have been a partial transfer.  */
            _ux_host_stack_transfer_request_abort(transfer_request);
        
            /* Set the completion code.  */
            transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_TIMEOUT;
        
            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_TIMEOUT);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_TIMEOUT, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)
        
            /* There was an error, return to the caller.  */
            return(UX_TRANSFER_TIMEOUT);
        }            

        /* We need to check the completion code as well.  */
        if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
            return(transfer_request -> ux_transfer_request_completion_code);

    }

    /* Arm the interrupt endpoint with a transfer request.  */
    transfer_request =  &storage -> ux_host_class_storage_interrupt_endpoint -> ux_endpoint_transfer_request;
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check the status of the data payload.  */
    if (status != UX_SUCCESS)
        return(status);
    
    /* We must wait for the interrupt endpoint to return the status stage now. This can
       take a fairly long time.  */
    status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_HOST_CLASS_STORAGE_CBI_STATUS_TIMEOUT));

    /* If the status is not successful, we may have a timeout error.  */
    if (status != UX_SUCCESS)
        return(status);

    /* Return the status code.  */
    return(transfer_request -> ux_transfer_request_completion_code);
}
#endif
