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
/*    _ux_host_class_pima_object_send                     PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function sends an object to the media. This commands should be */ 
/*    be proceeded by a object_info_send command.                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                       Pointer to pima class    */ 
/*    pima_session                               Pointer to pima session  */ 
/*    object                                     Pointer to object info   */ 
/*    object_buffer                              Buffer to be used        */ 
/*    object_buffer_length                       Buffer length            */ 
/*    callback_function                          Application function to  */ 
/*                                               callback when buffer     */ 
/*                                               needs to be written      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request            Transfer request         */
/*    _ux_host_stack_transfer_request_abort      Abort transfer           */
/*    _ux_host_stack_endpoint_reset              Reset endpoint           */
/*    _ux_host_semaphore_get                     Get semaphore            */
/*    _ux_utility_long_put                       Put 32-bit value         */
/*    _ux_utility_short_put                      Put 16-bit value         */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USB application                                                     */ 
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
UINT  _ux_host_class_pima_object_send(UX_HOST_CLASS_PIMA *pima, 
                                      UX_HOST_CLASS_PIMA_SESSION *pima_session,
                                      UX_HOST_CLASS_PIMA_OBJECT *object,
                                      UCHAR *object_buffer, ULONG object_buffer_length)
{


UX_HOST_CLASS_PIMA_COMMAND             command;
UX_TRANSFER                         *transfer_request;
UCHAR                                 *ptp_payload;
ULONG                                requested_length;
UINT                                status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_OBJECT_SEND, pima, object, object_buffer, object_buffer_length, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Check if this session is valid or not.  */
    if (pima_session -> ux_host_class_pima_session_magic != UX_HOST_CLASS_PIMA_MAGIC_NUMBER)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check if this session is opened or not.  */
    if (pima_session -> ux_host_class_pima_session_state != UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check if the object is already opened.  */
    if (object -> ux_host_class_pima_object_state != UX_HOST_CLASS_PIMA_OBJECT_STATE_OPENED)
        return (UX_HOST_CLASS_PIMA_RC_OBJECT_NOT_OPENED);    

    /* Check the transfer status. If there was an error or transfer is completed, refuse transfer.  */
    if ((object -> ux_host_class_pima_object_transfer_status == UX_HOST_CLASS_PIMA_OBJECT_TRANSFER_STATUS_COMPLETED) ||
        (object -> ux_host_class_pima_object_transfer_status == UX_HOST_CLASS_PIMA_OBJECT_TRANSFER_STATUS_ABORTED))
        return (UX_HOST_CLASS_PIMA_RC_ACCESS_DENIED);    

    /* This variable will remain untouched if the offset is not 0 and the requested length is 0.  */
    status =  UX_SUCCESS;

    /* Check if the offset to be read is at 0, if so, prepare the first write command. */
    if (object -> ux_host_class_pima_object_offset == 0)
    {

        /* Set the object transfer status to active.  */
        object -> ux_host_class_pima_object_transfer_status = UX_HOST_CLASS_PIMA_OBJECT_TRANSFER_STATUS_ACTIVE;

        /* Issue command to get the object info. no parameter.  */
        command.ux_host_class_pima_command_nb_parameters =  0;
    
        /* Then set the command to SEND_OBJECT.  */
        command.ux_host_class_pima_command_operation_code =  UX_HOST_CLASS_PIMA_OC_SEND_OBJECT;
    
        /* We use the Bulk Out pipe for sending command out..  */
        transfer_request =  &pima -> ux_host_class_pima_bulk_out_endpoint -> ux_endpoint_transfer_request;
    
        /* Get the pointer to the ptp payload.  */
        ptp_payload =  pima -> ux_host_class_pima_container ;
    
        /* Calculate the requested length for this payload.  */
        requested_length =  UX_HOST_CLASS_PIMA_COMMAND_HEADER_SIZE + ((ULONG)sizeof(ULONG) * command.ux_host_class_pima_command_nb_parameters); 

        /* Fill the command container. First the length of the total header and payload.  */
        _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_LENGTH, requested_length);
                        
        /* Then the type of container : a command block here.  */
        _ux_utility_short_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_TYPE, UX_HOST_CLASS_PIMA_CT_COMMAND_BLOCK);
    
        /* Now the command code to send.  */
        _ux_utility_short_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_CODE, (USHORT)command.ux_host_class_pima_command_operation_code);

        /* Put the transaction ID.  */
        _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_TRANSACTION_ID, 
                        pima -> ux_host_class_pima_transaction_id++);

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
            
                /* Set the object transfer status to aborted.  */
                object -> ux_host_class_pima_object_transfer_status = UX_HOST_CLASS_PIMA_OBJECT_TRANSFER_STATUS_ABORTED;

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
        if (requested_length == transfer_request -> ux_transfer_request_actual_length)
        {

            /* Send the first packet.  This packet contains the header and some data. 
               We use the Bulk Out pipe for sending data ..  */
            transfer_request =  &pima -> ux_host_class_pima_bulk_out_endpoint -> ux_endpoint_transfer_request;
            
            /* Get the pointer to the ptp payload.  */
            ptp_payload =  pima -> ux_host_class_pima_container ;
            
            /* Container type is a data type.  */
            *(ptp_payload + UX_HOST_CLASS_PIMA_DATA_HEADER_TYPE) =  UX_HOST_CLASS_PIMA_CT_DATA_BLOCK;
            
            /* Fill in the header values.  Start with the length of the container. */
            _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_DATA_HEADER_LENGTH, 
                                    object -> ux_host_class_pima_object_compressed_size + UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE);

            /* If the total payload is on a boundary, we need a ZLP to mark the end.  */
            if (((object -> ux_host_class_pima_object_compressed_size + UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE) % 
                    pima -> ux_host_class_pima_bulk_out_endpoint-> ux_endpoint_descriptor.wMaxPacketSize) == 0)
            
                /* We have a ZLP condition.  */
                pima -> ux_host_class_pima_zlp_flag = UX_HOST_CLASS_PIMA_ZLP_OUT;
            else
                
                /* Do not expect a ZLP.  */
                pima -> ux_host_class_pima_zlp_flag = UX_HOST_CLASS_PIMA_ZLP_NONE;
            
            /* Calculate the requested length for this payload.  */
            requested_length =  UX_HOST_CLASS_PIMA_CONTAINER_SIZE;

            /* Check to see if we have enough to fill the first packet.  */
            if (requested_length > (UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE + object_buffer_length))
                
                /* This is a small object, enough to fit into the fist packet.  */
                requested_length = UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE + object_buffer_length;

            /* We need to skip the header.  Copying the necessary partial memory only.  */
            _ux_utility_memory_copy(ptp_payload + UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE, object_buffer,
                                    requested_length - UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE); /* Use case of memcpy is verified. */

            /* Update the offset.  */
            object -> ux_host_class_pima_object_offset += requested_length - UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE;
            
            /* And the current object buffer pointer.  */
            object_buffer += requested_length - UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE;           
            
            /* Adjust the remaining buffer length.  */
            object_buffer_length -= requested_length - UX_HOST_CLASS_PIMA_DATA_HEADER_SIZE;           

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
                    
                    /* Set the object transfer status to aborted.  */
                    object -> ux_host_class_pima_object_transfer_status = UX_HOST_CLASS_PIMA_OBJECT_TRANSFER_STATUS_ABORTED;

                    /* There was an error, return to the caller.  */
                    return(status);
                }            
            }
            else
            {
        
                /* There was a non transfer error, no partial transfer to be checked */
                return(status);
            }
        
            
        }
        else

            /* We got a premature error.  */
            return(UX_HOST_CLASS_PIMA_RC_INCOMPLETE_TRANSFER);
    }


    /* We use the Bulk Out pipe for sending data ..  */
    transfer_request =  &pima -> ux_host_class_pima_bulk_out_endpoint -> ux_endpoint_transfer_request;

    /* We send a complete block.  */
    while(object_buffer_length != 0)
    {

        /* It may take several transactions.  */
        if (object_buffer_length > UX_HOST_CLASS_PIMA_MAX_PAYLOAD)
            
            /* Set the requested length to the payload maximum.  */
            requested_length =  UX_HOST_CLASS_PIMA_MAX_PAYLOAD;
    
        else
            
            /* We can use the user supplied length to complete this request.  */
            requested_length =  object_buffer_length;
           
        /* Initialize the transfer_request.  */
        transfer_request -> ux_transfer_request_data_pointer =  object_buffer;
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

                /* Set the object transfer status to aborted.  */
                object -> ux_host_class_pima_object_transfer_status = UX_HOST_CLASS_PIMA_OBJECT_TRANSFER_STATUS_ABORTED;

                /* There was an error, return to the caller.  */
                return(status);
            }            
        }
        else
        {
        
            /* There was a non transfer error, no partial transfer to be checked */
            return(status);
        }
        
        /* Update the object length expected by the user with what we actually sent.  */
        object_buffer_length -= transfer_request -> ux_transfer_request_actual_length;
                  
        /* And the offset.  */
        object -> ux_host_class_pima_object_offset += transfer_request -> ux_transfer_request_actual_length;
        
        /* And the object buffer pointer .  */
        object_buffer += transfer_request -> ux_transfer_request_actual_length;

        /* Check to see if we are at the end of the object.  */
        if (object -> ux_host_class_pima_object_offset == object -> ux_host_class_pima_object_compressed_size)
        
            /* The transfer for this transaction is completed.  */
            object -> ux_host_class_pima_object_transfer_status = UX_HOST_CLASS_PIMA_OBJECT_TRANSFER_STATUS_COMPLETED;
        
    }

    /* Return completion status.  */
    return(status);
}

