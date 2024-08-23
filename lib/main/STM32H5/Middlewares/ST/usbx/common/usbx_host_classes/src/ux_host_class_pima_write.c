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
/**   Pima Class                                                          */
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
/*    _ux_host_class_pima_write                           PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function writes a data payload to the Pima device. This        */ 
/*    function first write a header followed by some data.                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*    data_pointer                          Pointer to data to write      */ 
/*    data_length                           Length of data to write       */ 
/*    operation_code                        Code to send in header (this  */ 
/*                                          is from the command block     */ 
/*    max_payload_data                      Maximum data sent in one load */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_host_stack_transfer_request_abort Abort transfer request        */ 
/*    _ux_host_stack_endpoint_reset         Reset endpoint                */
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_host_semaphore_get                Get protection semaphore      */ 
/*    _ux_utility_long_put                  Put a long 32 bit value       */
/*    _ux_utility_short_put                 Put a short 16 bit value      */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            prefixed UX to MS_TO_TICK,  */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_pima_write(UX_HOST_CLASS_PIMA *pima, UCHAR *data_pointer, 
                                    ULONG data_length,
                                    ULONG operation_code, 
                                    ULONG max_payload_length)

{

UX_TRANSFER     *transfer_request;
UINT            status;
UCHAR            *ptp_payload;
ULONG            requested_length;
ULONG            payload_length;

    UX_PARAMETER_NOT_USED(operation_code);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_WRITE, pima, data_pointer, data_length, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* We use the Bulk Out pipe for receiving data ..  */
    transfer_request =  &pima -> ux_host_class_pima_bulk_out_endpoint -> ux_endpoint_transfer_request;
    
    /* Get the pointer to the ptp payload.  */
    ptp_payload =  pima -> ux_host_class_pima_container ;
    
    /* Initialize the transfer_request.  */
    transfer_request -> ux_transfer_request_data_pointer =  ptp_payload;

    /* Fill in the header values.  Start with the length of the container. */
    _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_DATA_HEADER_LENGTH, data_length + UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE);

    /* Check for remainder in last packet.  */
    if (((data_length + UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE) % pima -> ux_host_class_pima_bulk_out_endpoint -> ux_endpoint_descriptor.wMaxPacketSize) == 0)
    
        /* We have a ZLP condition on a OUT.  */
        pima -> ux_host_class_pima_zlp_flag = UX_HOST_CLASS_PIMA_ZLP_OUT;
    else
        
        /* Do not expect a ZLP.  */
        pima -> ux_host_class_pima_zlp_flag = UX_HOST_CLASS_PIMA_ZLP_NONE;

    /* Container type is a data type.  */
    *(ptp_payload + UX_HOST_CLASS_PIMA_DATA_HEADER_TYPE) =  UX_HOST_CLASS_PIMA_CT_DATA_BLOCK;
    
    /* Put the operation code.  */
    *(ptp_payload + UX_HOST_CLASS_PIMA_DATA_HEADER_CODE) =  (UCHAR)pima -> ux_host_class_pima_operation_code;
    
    /* Store the transaction ID.  */
    _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID, 
                                    pima -> ux_host_class_pima_transaction_id);

    /* Calculate the requested length in the first container.  */
    requested_length = data_length + UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE;
    
    /* Can we fill the container ?  */
    if(requested_length > UX_HOST_CLASS_PIMA_CONTAINER_SIZE)

        /* We have more data in the payload than the first container so adjust the length.  */
        requested_length =  UX_HOST_CLASS_PIMA_CONTAINER_SIZE;
    
    /* Add the data payload to fill that container.  */
    _ux_utility_memory_copy(ptp_payload + UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE, data_pointer, 
                            requested_length - UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE); /* Use case of memcpy is verified. */
    
    /* Initialize the transfer of that container.  */
    transfer_request -> ux_transfer_request_requested_length =  requested_length;
        
    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* If the transfer is successful, we need to wait for the transfer request to be completed.  */
    if (status == UX_SUCCESS)
    {
        
        /* Wait for the completion of the transfer request.  */
        status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_HOST_CLASS_PIMA_CLASS_TRANSFER_TIMEOUT));

        /* If the semaphore did not succeed we probably have a time out.  */
        if (status != UX_SUCCESS)
        {

            /* All transfers pending need to abort. There may have been a partial transfer.  */
            _ux_host_stack_transfer_request_abort(transfer_request);

            /* The endpoint was halted by a transfer error and needs to be reset.  */
            _ux_host_stack_endpoint_reset(pima -> ux_host_class_pima_bulk_in_endpoint);

            /* The endpoint was halted by a transfer error  and needs to be reset.  */
            _ux_host_stack_endpoint_reset(pima -> ux_host_class_pima_bulk_out_endpoint);
            
            
            /* There was an error, return to the caller.  */
            return(status);
        }            
    }
    else
    {

        /* There was a non transfer error, no partial transfer to be checked */
        return(status);
    }

    /* Isolate the length of the data payload that still needs to be sent.  */
    data_length -= (requested_length - UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE);
    
    /* Adjust the data payload pointer.  */
    data_pointer += (requested_length - UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE);
    
    /* Now we can send the data to the device.  */
    while(data_length)
    {
    
        /* Check if need to split the data payload into smaller packets.  */
        if (data_length > max_payload_length)

            /* We cannot send everything in this payload. */
            payload_length = max_payload_length;
        else

            /* Either this is the last packet or we we have a small packet to send.  */
            payload_length = data_length;  
            
        /* Initialize the transfer_request.  */
        transfer_request -> ux_transfer_request_data_pointer =  data_pointer;
        transfer_request -> ux_transfer_request_requested_length =  payload_length;

        /* Send request to HCD layer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* If the transfer is successful, we need to wait for the transfer request to be completed.  */
        if (status == UX_SUCCESS)
        {
        
            /* Wait for the completion of the transfer request.  */
            status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_HOST_CLASS_PIMA_CLASS_TRANSFER_TIMEOUT));

            /* If the semaphore did not succeed we probably have a time out.  */
            if (status != UX_SUCCESS)
            {

                /* All transfers pending need to abort. There may have been a partial transfer.  */
                _ux_host_stack_transfer_request_abort(transfer_request);
        
                /* The endpoint was halted by a transfer error and needs to be reset.  */
                _ux_host_stack_endpoint_reset(pima -> ux_host_class_pima_bulk_in_endpoint);

                /* The endpoint was halted by a transfer error  and needs to be reset.  */
                _ux_host_stack_endpoint_reset(pima -> ux_host_class_pima_bulk_out_endpoint);
            
                /* There was an error, return to the caller.  */
                return(status);
            }            
        }
        else
        {

            /* There was a non transfer error, no partial transfer to be checked */
            return(status);
        }                

        /* Check for completion of transfer. If the transfer is partial, return to caller.
           Partial transfer is not OK. */
        if (payload_length != transfer_request -> ux_transfer_request_actual_length)
            return(UX_TRANSFER_ERROR);    

        /* Adjust the total length to transfer.  */
        data_length -= payload_length;
        
        /* Adjust the data pointer.  */
        data_pointer += payload_length;

    }

    /* If we have a ZLP condition, write the device one more time with a zero packet.  */
    if (pima -> ux_host_class_pima_zlp_flag == UX_HOST_CLASS_PIMA_ZLP_OUT)
    {
        
        /* Initialize the transfer_request.  */
        transfer_request -> ux_transfer_request_data_pointer =  UX_NULL;
        transfer_request -> ux_transfer_request_requested_length =  0;

        /* Send request to HCD layer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* If the transfer is successful, we need to wait for the transfer request to be completed.  */
        if (status == UX_SUCCESS)
        {
        
            /* Wait for the completion of the transfer request.  */
            status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_HOST_CLASS_PIMA_CLASS_TRANSFER_TIMEOUT));

            /* If the semaphore did not succeed we probably have a time out.  */
            if (status != UX_SUCCESS)
            {

                /* All transfers pending need to abort. There may have been a partial transfer.  */
                _ux_host_stack_transfer_request_abort(transfer_request);
        
                /* The endpoint was halted by a transfer error and needs to be reset.  */
                _ux_host_stack_endpoint_reset(pima -> ux_host_class_pima_bulk_in_endpoint);

                /* The endpoint was halted by a transfer error  and needs to be reset.  */
                _ux_host_stack_endpoint_reset(pima -> ux_host_class_pima_bulk_out_endpoint);
            
                /* There was an error, return to the caller.  */
                return(status);
            }            
    
        /* Reset the ZLP.  */
        pima -> ux_host_class_pima_zlp_flag = UX_HOST_CLASS_PIMA_ZLP_NONE;
        }
    }            

    /* We have finished receiving the data.  */
    return(UX_SUCCESS);
}

