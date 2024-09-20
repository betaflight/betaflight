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
/*    _ux_host_class_pima_command                         PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will send a command to the PIMA device.               */ 
/*    It will perform a data phase if necessary and the status phase.     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*    command                               pointer to command container  */ 
/*    direction                             either IN or OUT              */ 
/*    data_buffer                           buffer to be sent or received */ 
/*    data_length                           length of the buffer to send  */ 
/*                                          or receive                    */ 
/*    max_payload_length                    maximum payload length        */ 
/*                                                                        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
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
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_pima_command(UX_HOST_CLASS_PIMA *pima, UX_HOST_CLASS_PIMA_COMMAND *command,
                                    ULONG direction, UCHAR *data_buffer, ULONG data_length,        
                                    ULONG max_payload_length)
{

UX_TRANSFER     *transfer_request;
UCHAR             *ptp_payload;
ULONG            requested_length;
UINT            status;

    /* We use the Bulk Out pipe for sending data out..  */
    transfer_request =  &pima -> ux_host_class_pima_bulk_out_endpoint -> ux_endpoint_transfer_request;
    
    /* Get the pointer to the ptp payload.  */
    ptp_payload =  pima -> ux_host_class_pima_container ;
    
    /* Calculate the requested length for this payload.  */
    requested_length =  UX_HOST_CLASS_PIMA_COMMAND_HEADER_SIZE + ((ULONG)sizeof(ULONG) * command -> ux_host_class_pima_command_nb_parameters);

    /* Fill the command container. First the length of the total header and payload.  */
    _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_LENGTH, requested_length);
                        
    /* Then the type of container : a command block here.  */
    _ux_utility_short_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_TYPE, UX_HOST_CLASS_PIMA_CT_COMMAND_BLOCK);
    
    /* Now the command code to send.  */
    _ux_utility_short_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_CODE, (USHORT)command -> ux_host_class_pima_command_operation_code);

    /* Save the operation code.  */
    pima -> ux_host_class_pima_operation_code =  command -> ux_host_class_pima_command_operation_code;    

    /* Put the transaction ID.  */
    _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_TRANSACTION_ID, 
                        pima -> ux_host_class_pima_transaction_id++);
    
    /* Then fill in all the parameters. To make it quick we fill the 5 parameters, regardless
       of the number contained in the command. But when the payload is transmitted, only the 
       relevant data is sent over the Bulk Out pipe.  */
    _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_PARAMETER_1, 
                        command -> ux_host_class_pima_command_parameter_1);
    _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_PARAMETER_2, 
                        command -> ux_host_class_pima_command_parameter_2);
    _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_PARAMETER_3, 
                        command -> ux_host_class_pima_command_parameter_3);
    _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_PARAMETER_4, 
                        command -> ux_host_class_pima_command_parameter_4);
    _ux_utility_long_put(ptp_payload + UX_HOST_CLASS_PIMA_COMMAND_HEADER_PARAMETER_5, 
                        command -> ux_host_class_pima_command_parameter_5);

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
            
            /* Set the completion code.  */
            transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_TIMEOUT;
        
            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_TIMEOUT);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_TIMEOUT, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)
        
            /* There was an error, return to the caller.  */
            return(UX_TRANSFER_TIMEOUT);
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
        
        /* The command was sent successfully. Now examine the direction and check
           if we need a data phase. If so which direction.  */

        switch (direction)
        {
        
            /* No data phase, proceed to response directly. */
            case     UX_HOST_CLASS_PIMA_DATA_PHASE_NONE    :

                /* No error here.  */
                status = UX_SUCCESS;
                break;
    
            /* We need to transfer data IN.  */
            case    UX_HOST_CLASS_PIMA_DATA_PHASE_IN    :
                status = _ux_host_class_pima_read(pima, data_buffer, data_length, max_payload_length);
                break;

            /* We need to transfer data OUT.  */
            case    UX_HOST_CLASS_PIMA_DATA_PHASE_OUT    :
                status = _ux_host_class_pima_write(pima, data_buffer, data_length, command -> ux_host_class_pima_command_operation_code,
                                                    max_payload_length);
                break;

            default                                     :
                return(UX_ERROR);
        
        }

        /* Analyze the status. If no errors during the data phase proceed to response code.  */
        if (status == UX_SUCCESS)
        {

            /* We use the Bulk In pipe for receiving the response payload ..  */
            transfer_request =  &pima -> ux_host_class_pima_bulk_in_endpoint -> ux_endpoint_transfer_request;
            
            /* Get the pointer to the ptp payload.  */
            ptp_payload =  pima -> ux_host_class_pima_container ;
            
            /* Calculate the requested length for this payload.  */
            requested_length =  UX_HOST_CLASS_PIMA_RESPONSE_HEADER_SIZE;
        
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
                    
                    /* Set the completion code.  */
                    transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_TIMEOUT;
        
                    /* Error trap. */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_TIMEOUT);

                    /* If trace is enabled, insert this event into the trace buffer.  */
                    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_TIMEOUT, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)
        
                    /* There was an error, return to the caller.  */
                    return(UX_TRANSFER_TIMEOUT);
                }            
            }
            else
            {
        
                /* There was a non transfer error, no partial transfer to be checked */
                return(status);
            }
        
            /* Check to ensure this is a Response packet.  */
            if (_ux_utility_short_get(ptp_payload + UX_HOST_CLASS_PIMA_RESPONSE_HEADER_TYPE) != 
                                        UX_HOST_CLASS_PIMA_CT_RESPONSE_BLOCK)

                /* We have a wrong packet.  */
                return(UX_ERROR);                                        
            
            /* Then get all the response parameters.  */
            command -> ux_host_class_pima_command_parameter_1 =  _ux_utility_long_get(ptp_payload + 
                                                                 UX_HOST_CLASS_PIMA_RESPONSE_HEADER_PARAMETER_1);
            command -> ux_host_class_pima_command_parameter_2 =  _ux_utility_long_get(ptp_payload + 
                                                                 UX_HOST_CLASS_PIMA_RESPONSE_HEADER_PARAMETER_2);
            command -> ux_host_class_pima_command_parameter_3 =  _ux_utility_long_get(ptp_payload + 
                                                                 UX_HOST_CLASS_PIMA_RESPONSE_HEADER_PARAMETER_3);
            command -> ux_host_class_pima_command_parameter_4 =  _ux_utility_long_get(ptp_payload + 
                                                                 UX_HOST_CLASS_PIMA_RESPONSE_HEADER_PARAMETER_4);
            command -> ux_host_class_pima_command_parameter_5 =  _ux_utility_long_get(ptp_payload + 
                                                                 UX_HOST_CLASS_PIMA_RESPONSE_HEADER_PARAMETER_5);

            /* We are done with the command.  */
            return(UX_SUCCESS);
        }
        /* Return the error.  */
        return(status);
    }
    else
        /* Truncated command. */
        return(UX_TRANSFER_ERROR);

}

