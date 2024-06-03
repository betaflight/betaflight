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


#if UX_SLAVE_REQUEST_DATA_MAX_LENGTH < UX_HOST_CLASS_PIMA_CONTAINER_SIZE
#error UX_SLAVE_REQUEST_DATA_MAX_LENGTH too small, please check
#endif

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_pima_read                            PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function reads a data payload from the Pima device. This       */ 
/*    function first read a header followed by some data.                 */ 
/*                                                                        */
/*    Note header only transfer and partial transfer is not accepted.     */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*    data_pointer                          Pointer to data to read       */ 
/*    data_length                           Length of data to read        */ 
/*    callback_function                     Application function to call  */ 
/*                                          to get the rest of the data   */ 
/*    max_payload_data                      Maximum data received in one  */ 
/*                                          data payload.                 */ 
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
/*    _ux_host_semaphore_get                Get protection semaphore      */ 
/*    _ux_utility_long_get                  Get a long 32 bit value       */
/*    _ux_utility_short_get                 Get a short 16 bit value      */
/*    _ux_utility_memory_copy               Copy memory                   */
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved length checks,     */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_pima_read(UX_HOST_CLASS_PIMA *pima, UCHAR *data_pointer, 
                                    ULONG data_length,
                                    ULONG max_payload_length)

{

UX_TRANSFER     *transfer_request;
UINT            status;
ULONG            header_length;
UCHAR            *ptp_payload;
ULONG            requested_length;
ULONG            payload_length;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_READ, pima, data_pointer, data_length, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* We use the Bulk In pipe for receiving data ..  */
    transfer_request =  &pima -> ux_host_class_pima_bulk_in_endpoint -> ux_endpoint_transfer_request;
    
    /* Get the pointer to the ptp payload.  */
    ptp_payload =  pima -> ux_host_class_pima_container ;
    
    /* Calculate the requested length for this payload.  */
    requested_length =  UX_HOST_CLASS_PIMA_CONTAINER_SIZE;

    /* Initialize the transfer_request.  */
    transfer_request -> ux_transfer_request_data_pointer =  ptp_payload;
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

    /* Ensure the transfer is greater than the size of a PIMA header.  */
    if (transfer_request -> ux_transfer_request_actual_length <= UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE)

        /* We have a malformed packet. Return error.  */
        return(UX_CLASS_MALFORMED_PACKET_RECEIVED_ERROR);

    /* Get the expected length from the header. */
    header_length =  _ux_utility_long_get(ptp_payload + UX_HOST_CLASS_PIMA_DATA_HEADER_LENGTH);

    /* Device reported data length should be equal or more than this packet
     * (and UX_HOST_CLASS_PIMA_DATA_HEADER_LENGTH).  */
    if (header_length < transfer_request -> ux_transfer_request_actual_length)
        return(UX_CLASS_MALFORMED_PACKET_RECEIVED_ERROR);

    /* Check for remainder in last packet.  */
    if ((header_length % pima -> ux_host_class_pima_bulk_in_endpoint -> ux_endpoint_descriptor.wMaxPacketSize) == 0)
    
        /* We have a ZLP condition on a IN.  */
        pima -> ux_host_class_pima_zlp_flag = UX_HOST_CLASS_PIMA_ZLP_IN;
    else
        
        /* Do not expect a ZLP.  */
        pima -> ux_host_class_pima_zlp_flag = UX_HOST_CLASS_PIMA_ZLP_NONE;

    /* The length returned should be smaller than the length requested.  */
    if ((header_length - UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE) > data_length)
        return(UX_ERROR);

    /* We may have had data in the first packet, if so adjust the data_length.  */
    data_length = header_length - transfer_request -> ux_transfer_request_actual_length;
    
    /* Copying the necessary partial memory.  */
    _ux_utility_memory_copy(data_pointer, ptp_payload + UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE, 
                            transfer_request -> ux_transfer_request_actual_length - UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE); /* Use case of memcpy is verified. */

    /* Adjust the data payload pointer.  */
    data_pointer += (transfer_request -> ux_transfer_request_actual_length - UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE);
    
    /* Now we can read the data from the device.  */
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

    /* If we have a ZLP condition, read from the device one more time with a zero packet.  */
    if (pima -> ux_host_class_pima_zlp_flag == UX_HOST_CLASS_PIMA_ZLP_IN)
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

