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
/*    _ux_host_class_storage_transport_bo                 PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the transport layer for the Bulk Only protocol.    */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage                               Pointer to storage class      */
/*    data_pointer                          Pointer to data               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    The transfer completion status. It's possible for the transfer to   */
/*    succeed but for the command to fail. The CSW must be checked to     */
/*    determine command status.                                           */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_storage_device_reset   Reset mass storage            */
/*    _ux_host_stack_endpoint_reset         Reset endpoint                */
/*    _ux_host_stack_transfer_request       Process host stack transfer   */
/*    _ux_host_stack_transfer_request_abort Abort transfer request        */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
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
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_transport_bo(UX_HOST_CLASS_STORAGE *storage, UCHAR *data_pointer)
{

UX_TRANSFER     *transfer_request;
UINT            status;
UCHAR           *cbw;
UINT            retry;
ULONG           data_phase_requested_length;
ULONG           data_phase_transfer_size;
UCHAR           *get_status_response;


    /* Get the pointer to the transfer request.  */
    transfer_request =  &storage -> ux_host_class_storage_bulk_out_endpoint -> ux_endpoint_transfer_request;

    /* Use a pointer for the cbw, easier to manipulate.  */
    cbw =  (UCHAR *) storage -> ux_host_class_storage_cbw;

    /* Fill in the transfer request parameters.  */
    transfer_request -> ux_transfer_request_data_pointer =      cbw;
    transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_STORAGE_CBW_LENGTH;

    /* It's possible for the bulk out endpoint to stall; in that case, we clear
       it and try again.  */
    for (retry =  0; ; retry++)
    {

        /* Send the CBW on the bulk out endpoint.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check the status of the command (NB, the command is not sent yet since
           the bulk transport is non blocking).  */
        if (status != UX_SUCCESS)
            return(status);

        /* Wait for the completion of the transfer request.  */
        status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_HOST_CLASS_STORAGE_TRANSFER_TIMEOUT));

        /* If the semaphore did not succeed we probably have a time out.  */
        if (status != UX_SUCCESS)
        {

            /* All transfers pending need to abort. There may have been a partial transfer.  */
            _ux_host_stack_transfer_request_abort(transfer_request);

            /* Set the completion code.  */
            transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_TIMEOUT;

            /* Error trap.  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_TIMEOUT);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_TIMEOUT, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* There was an error, return to the caller.  */
            return(UX_TRANSFER_TIMEOUT);
        }

        /* Did we successfully send the CBW?  */
        if (transfer_request -> ux_transfer_request_completion_code == UX_SUCCESS)

            /* Jump to the data stage.  */
            break;

        /* The transfer stalled. Is this the first time?  */
        if (retry == 0)
        {

            /* We must do a reset recovery.  */
            _ux_host_class_storage_device_reset(storage);

            /* In virtually all cases, the reset should've fixed it, so just
               resend the command.  */
        }
        else
        {

            /* Well, we tried!  */
            return(transfer_request -> ux_transfer_request_completion_code);
        }
    }

    /* Get the length of the data payload.  */
    data_phase_requested_length =  _ux_utility_long_get(cbw + UX_HOST_CLASS_STORAGE_CBW_DATA_LENGTH);

    /* Reset the data phase memory size.  */
    storage -> ux_host_class_storage_data_phase_length =  0;

    /* Perform the data stage - if there is any.  */
    while (data_phase_requested_length != 0)
    {

        /* Check if we can finish the transaction with one data phase.  */
        if (data_phase_requested_length > UX_HOST_CLASS_STORAGE_MAX_TRANSFER_SIZE)

            /* We have too much data to send in one phase. Split into smaller chunks.  */
            data_phase_transfer_size =  UX_HOST_CLASS_STORAGE_MAX_TRANSFER_SIZE;

        else

            /* The transfer size can be the requested length.  */
            data_phase_transfer_size =  data_phase_requested_length;

        /* Check the direction and determine which endpoint to use.  */
        if (*(cbw + UX_HOST_CLASS_STORAGE_CBW_FLAGS) == UX_HOST_CLASS_STORAGE_DATA_IN)
            transfer_request =  &storage -> ux_host_class_storage_bulk_in_endpoint -> ux_endpoint_transfer_request;

        /* Fill in the transfer request data payload buffer.  */
        transfer_request -> ux_transfer_request_data_pointer =  data_pointer;

        /* Store the requested length in the transfer request.  */
        transfer_request -> ux_transfer_request_requested_length =  data_phase_transfer_size;

        /* Perform data payload transfer (in or out).  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check the status of the data payload.  */
        if (status == UX_SUCCESS)

            /* Wait for the completion of the transfer request.  */
            status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_HOST_CLASS_STORAGE_TRANSFER_TIMEOUT));

        /* If the semaphore did not succeed we may have a time out or if we had a problem during the preparation of the transaction
           we should abort the SCSI transaction.  */
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

            /* There was an error that cannot be recovered, return to the caller.  */
            return(UX_TRANSFER_TIMEOUT);
        }

        /* Check the transfer status. If there is a transport error, we still need to read the CSW
           and let the upper layer retry.  */
        if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
        {

            /* This is most likely a STALL. We must clear it and go straight
               to reading the CSW. Note this doesn't necessarily mean the transfer
               failed completely failed. For example, if this was a read, it
               could mean the device had less data to send than we requested,
               but we still received data.  */

            /* Check the direction and determine which endpoint to reset.  */
            if (*(cbw + UX_HOST_CLASS_STORAGE_CBW_FLAGS) == UX_HOST_CLASS_STORAGE_DATA_IN)
                _ux_host_stack_endpoint_reset(storage -> ux_host_class_storage_bulk_in_endpoint);
            else
                _ux_host_stack_endpoint_reset(storage -> ux_host_class_storage_bulk_out_endpoint);

            /* We need to read the CSW now.   */
            break;
        }

        /* Adjust the total size that was requested.  */
        data_phase_requested_length -=  data_phase_transfer_size;

        /* And the data pointer.  */
        data_pointer +=  data_phase_transfer_size;
    }

    /* Now perform the status phase, i.e. try to get the CSW from the device.
       According to the spec, if there is a failure the first time, we need to retry once
       before reporting an error.  */

    /* Get the pointer to the transfer request, on the bulk in endpoint.  */
    transfer_request =  &storage -> ux_host_class_storage_bulk_in_endpoint -> ux_endpoint_transfer_request;

    /* Fill in the transfer_request parameters.  */
    transfer_request -> ux_transfer_request_data_pointer =      (UCHAR *) &storage -> ux_host_class_storage_csw;
    transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_STORAGE_CSW_LENGTH;

    /* Retry loop, we have 2 tries here.  */
    for (retry =  0; retry < 2; retry++)
    {

        /* Get the CSW on the bulk in endpoint.  */
        status =  _ux_host_stack_transfer_request(transfer_request);
        if (status != UX_SUCCESS)
            return(status);

        /* Wait for the completion of the transfer request.  */
        status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_HOST_CLASS_STORAGE_TRANSFER_TIMEOUT));

        /* Check semaphore status.  */
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

        /* Did the transfer succeed?  */
        if (transfer_request -> ux_transfer_request_completion_code == UX_SUCCESS)
        {

            /* The PASSED and COMMAND_FAILED error codes are nearly interchangeable
               according to the spec, so we treat them similarly.  */
            if (storage -> ux_host_class_storage_csw[UX_HOST_CLASS_STORAGE_CSW_STATUS] == UX_HOST_CLASS_STORAGE_CSW_PASSED ||
                storage -> ux_host_class_storage_csw[UX_HOST_CLASS_STORAGE_CSW_STATUS] == UX_HOST_CLASS_STORAGE_CSW_FAILED)
            {

                /* Was this an OUT transport?  */
                if (*(cbw + UX_HOST_CLASS_STORAGE_CBW_FLAGS) == UX_HOST_CLASS_STORAGE_DATA_OUT)
                {

                    /* Did the command fail, or succeed with non-zero data residue?  */
                    if (storage -> ux_host_class_storage_csw[UX_HOST_CLASS_STORAGE_CSW_STATUS] == UX_HOST_CLASS_STORAGE_CSW_FAILED ||
                        _ux_utility_long_get(storage -> ux_host_class_storage_csw + UX_HOST_CLASS_STORAGE_CSW_DATA_RESIDUE) != 0)
                    {

                        /* It's possible the bulk out endpoint is stalled.
                           This happens when 1) the device expects less data
                           than the host wants to send and 2) the endpoint
                           stalls after the last packet was sent (otherwise,
                           we'd have seen the stall during the data stage).
                           Query its status before clearing the stall.  */

                        /* Allocate memory for Get Status response.  */
                        get_status_response =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 2);
                        if (get_status_response == UX_NULL)
                            return(UX_MEMORY_INSUFFICIENT);

                        /* Get the control endpoint's transfer request.  */
                        transfer_request =  &storage -> ux_host_class_storage_device -> ux_device_control_endpoint.ux_endpoint_transfer_request;

                        /* Setup transfer request for Get Status command.  */
                        transfer_request -> ux_transfer_request_data_pointer =      get_status_response;
                        transfer_request -> ux_transfer_request_requested_length =  0x02;
                        transfer_request -> ux_transfer_request_function =          UX_GET_STATUS;
                        transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_ENDPOINT;
                        transfer_request -> ux_transfer_request_value =             0;
                        transfer_request -> ux_transfer_request_index =             storage -> ux_host_class_storage_bulk_out_endpoint-> ux_endpoint_descriptor.bEndpointAddress;

                        /* Send transfer request.  */
                        status =  _ux_host_stack_transfer_request(transfer_request);

                        /* Check status of transfer.  */
                        if (status == UX_SUCCESS)
                        {

                            /* Check the Halt bit.  */
                            if (get_status_response[0] & 0x01)
                            {

                                /* Clear the halt.  */
                                status =  _ux_host_stack_endpoint_reset(storage -> ux_host_class_storage_bulk_out_endpoint);
                            }
                        }

                        /* Free the get status memory.  */
                        _ux_utility_memory_free(get_status_response);

                        /* Check result of Get Status and Clear Feature commands. */
                        if (status != UX_SUCCESS)
                            return(status);
                    }
                }

                /* Save the amount of relevant data.  */
                storage -> ux_host_class_storage_data_phase_length =  _ux_utility_long_get(cbw + UX_HOST_CLASS_STORAGE_CBW_DATA_LENGTH) -
                    _ux_utility_long_get(storage -> ux_host_class_storage_csw + UX_HOST_CLASS_STORAGE_CSW_DATA_RESIDUE);
            }
            else
            {

                /* Must be phase error. Need to reset the device.  */
                _ux_host_class_storage_device_reset(storage);
            }

            /* Return success since the transfer succeeded. The caller should
               look at the CSW to determine if the command's status.  */
            return(UX_SUCCESS);
        }

        /* The transfer stalled. We must clear the stall and attempt to read
           the CSW again.  */
        _ux_host_stack_endpoint_reset(storage -> ux_host_class_storage_bulk_in_endpoint);
    }

    /* If we get here, the CSW transfer stalled twice. We must reset the device.  */
    _ux_host_class_storage_device_reset(storage);

    /* Return the error.  */
    return(transfer_request -> ux_transfer_request_completion_code);
}
#endif
