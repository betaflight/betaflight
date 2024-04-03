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
/*    _ux_host_class_pima_request_cancel                  PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function cancels a request.                                    */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                       Pointer to pima class    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Transfer request              */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_utility_short_put                 Put 16-bit value              */
/*    _ux_utility_short_get                 Get 16-bit value              */
/*    _ux_utility_long_put                  Put 32-bit value              */
/*    _ux_utility_delay_ms                  Delay ms                      */ 
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
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_pima_request_cancel(UX_HOST_CLASS_PIMA *pima)
{

UX_TRANSFER                         *transfer_request;
UX_ENDPOINT                         *control_endpoint;
UCHAR                                 *request_payload;
UINT                                status;
UINT                                device_status;
ULONG                               payload_length;
ULONG                               command_retry_counter;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_REQUEST_CANCEL, pima, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Allocate some DMA safe memory for the command data payload.  We use mode than needed because
       we use this buffer for both the command and the status phase.  */
    request_payload =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PIMA_REQUEST_STATUS_DATA_LENGTH);
    if (request_payload == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Fill in the payload buffer with cancellation code.  */
    _ux_utility_short_put(request_payload + UX_HOST_CLASS_PIMA_REQUEST_CANCEL_OFFSET_CODE, 
                        UX_HOST_CLASS_PIMA_REQUEST_CANCEL_CODE );
    
    /* Fill in the payload buffer with transactionID.  */
    _ux_utility_long_put(request_payload + UX_HOST_CLASS_PIMA_REQUEST_CANCEL_OFFSET_TRANSACTION_ID, 
                        pima -> ux_host_class_pima_transaction_id++);
    
    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &pima -> ux_host_class_pima_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Create a transfer_request for the CANCEL request.  */
    transfer_request -> ux_transfer_request_data_pointer =      request_payload;
    transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_PIMA_REQUEST_CANCEL_DATA_LENGTH;
    transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_PIMA_REQUEST_CANCEL_COMMAND;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =             0;
    transfer_request -> ux_transfer_request_index  =            0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check if status is OK. If the command did not work, we have a major problem.  */
    if(status == UX_SUCCESS)
    {

        /* Initialize the command retry counter.  */
        command_retry_counter = UX_HOST_CLASS_PIMA_REQUEST_STATUS_COMMAND_COUNTER;

        /* This command may be retried a few times.  */
        while (command_retry_counter-- != 0)
        {

            /* We need to wait for the device to be OK. For that we issue a GET_DEVICE_STATUS command.  */
            transfer_request -> ux_transfer_request_data_pointer =      request_payload;
            transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_PIMA_REQUEST_CANCEL_DATA_LENGTH;
            transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_PIMA_REQUEST_STATUS_COMMAND;
            transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
            transfer_request -> ux_transfer_request_value =             0;
            transfer_request -> ux_transfer_request_index  =            0;
        
            /* Send request to HCD layer.  */
            status =  _ux_host_stack_transfer_request(transfer_request);

            /* Check status, abort if there is an error.  */
            if (status != UX_SUCCESS)
                break;

            /* Extract the payload length from the status buffer.  */
            payload_length =  _ux_utility_short_get(request_payload + UX_HOST_CLASS_PIMA_REQUEST_STATUS_OFFSET_LENGTH);

            /* Check the data payload length.  */
            if (payload_length < UX_HOST_CLASS_PIMA_REQUEST_STATUS_OFFSET_CODE + sizeof(UINT))
            {

                /* We have a data format error.  */
                status =  UX_ERROR;
                break;
            }
            
            /* Extract the device status.  */
            device_status =  _ux_utility_short_get(request_payload + UX_HOST_CLASS_PIMA_REQUEST_STATUS_OFFSET_CODE);
            
            /* If the device status is OK, we have a successful cancellation.  */
            if (device_status == UX_HOST_CLASS_PIMA_RC_OK)
            {

                /* Status is OK. exit the command loop.  */
                status = UX_SUCCESS;
                break;
            }         
            else
            {
                /* Force the status to error.  */
                status = UX_ERROR;       


                /* We should wait a little bit before re-issuing the command.  */
                _ux_utility_delay_ms(UX_HOST_CLASS_PIMA_REQUEST_STATUS_COMMAND_DELAY); 
                
            }            
        } 

    }

    /* Free allocated resources.  */
    _ux_utility_memory_free(request_payload);

    /* Return completion status.  */
    return(status);

}

