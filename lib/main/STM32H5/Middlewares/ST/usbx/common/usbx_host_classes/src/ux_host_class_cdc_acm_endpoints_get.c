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
/**   CDC ACM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_acm_endpoints_get                PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function distinguishes for either the Data or Control Class.   */
/*    For the data class, we mount the bulk in and bulk out endpoints.    */ 
/*    For the control class, we mount the optional interrupt endpoint.    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_acm                               Pointer to cdc_acm class      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_interface_endpoint_get Get interface endpoint        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_cdc_acm_activate       Activate cdc_acm class        */ 
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            initialized timeout value,  */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_cdc_acm_endpoints_get(UX_HOST_CLASS_CDC_ACM *cdc_acm)
{

UINT            status;
UINT            endpoint_index;
UX_ENDPOINT     *endpoint;
UX_TRANSFER     *transfer_request;


    /* Check what interface we are mounting.  */
    if (cdc_acm -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bInterfaceClass == UX_HOST_CLASS_CDC_DATA_CLASS)
    {

        /* Search the bulk OUT endpoint. It is attached to the interface container.  */
        for (endpoint_index = 0; endpoint_index < cdc_acm -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bNumEndpoints;
                            endpoint_index++)
        {                        
    
            /* Get interface endpoint.  */
            _ux_host_stack_interface_endpoint_get(cdc_acm -> ux_host_class_cdc_acm_interface, endpoint_index, &endpoint);
    
            /* Check if endpoint is bulk and OUT.  */
            if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT) &&
                ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT))
            {

                /* This transfer_request always have the OUT direction.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_OUT;

                /* Set default timeout for transfer.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_timeout_value = UX_HOST_CLASS_CDC_ACM_CLASS_TRANSFER_TIMEOUT;

                /* We have found the bulk endpoint, save it.  */
                cdc_acm -> ux_host_class_cdc_acm_bulk_out_endpoint =  endpoint;

                /* If found all, we break.  */
                if (cdc_acm -> ux_host_class_cdc_acm_bulk_in_endpoint)
                    break;
            }

            /* Check if endpoint is bulk and IN.  */
            if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN) &&
                ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT))
            {

                /* This transfer_request always have the IN direction.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_IN;

                /* Set default timeout for transfer.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_timeout_value = UX_HOST_CLASS_CDC_ACM_CLASS_TRANSFER_TIMEOUT;

                /* We have found the bulk endpoint, save it.  */
                cdc_acm -> ux_host_class_cdc_acm_bulk_in_endpoint =  endpoint;

                /* If found all, we break.  */
                if (cdc_acm -> ux_host_class_cdc_acm_bulk_out_endpoint)
                    break;
            }
        }

        /* The both bulk endpoints are mandatory.  */
        if (cdc_acm -> ux_host_class_cdc_acm_bulk_out_endpoint == UX_NULL ||
            cdc_acm -> ux_host_class_cdc_acm_bulk_in_endpoint == UX_NULL)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, cdc_acm, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_ENDPOINT_HANDLE_UNKNOWN);
        }
    }
    else
    {
        /* Search the Interrupt endpoint. It is attached to the interface container of the control interface. It is not mandatory.  */
        for (endpoint_index = 0; endpoint_index < cdc_acm -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bNumEndpoints;
                            endpoint_index++)
        {                        
    
            /* Get the endpoint handle.  */
            _ux_host_stack_interface_endpoint_get(cdc_acm -> ux_host_class_cdc_acm_interface, endpoint_index, &endpoint);

            /* Check if endpoint is Interrupt and IN.  */
            if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN) &&
                ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT))
            {

                /* This transfer_request always have the IN direction.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_IN;

                /* We have found the interrupt endpoint, save it.  */
                cdc_acm -> ux_host_class_cdc_acm_interrupt_endpoint =  endpoint;

                /* The endpoint is correct, Fill in the transfer request with the length requested for this endpoint.  */
                transfer_request =  &cdc_acm -> ux_host_class_cdc_acm_interrupt_endpoint -> ux_endpoint_transfer_request;
                transfer_request -> ux_transfer_request_requested_length =  transfer_request -> ux_transfer_request_packet_length;
                transfer_request -> ux_transfer_request_actual_length =     0;

                /* The direction is always IN for the CDC interrupt endpoint.  */
                transfer_request -> ux_transfer_request_type =  UX_REQUEST_IN;

                /* There is a callback function associated with the transfer request, so we need the class instance.  */
                transfer_request -> ux_transfer_request_class_instance =  (VOID *) cdc_acm;

                /* Interrupt transactions have a completion routine. */
                transfer_request -> ux_transfer_request_completion_function =  _ux_host_class_cdc_acm_transfer_request_completed;

                /* Obtain a buffer for this transaction. The buffer will always be reused.  */
                transfer_request -> ux_transfer_request_data_pointer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 
                                                                transfer_request -> ux_transfer_request_requested_length);

                /* If the endpoint is available and we have memory, we start the interrupt endpoint.  */
                if (transfer_request -> ux_transfer_request_data_pointer != UX_NULL)
                {
                    
                    /* The transfer on the interrupt endpoint can be started.  */
                    status =  _ux_host_stack_transfer_request(transfer_request);
                    
                    /* Check error, if endpoint interrupt IN transfer not successful, do not proceed. */
                    if (status != UX_SUCCESS)
                    
                        /* Error, do not proceed.  */
                        return(status);
                }
                else
                {

                    /* Error trap. */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

                    /* If trace is enabled, insert this event into the trace buffer.  */
                    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, cdc_acm, 0, 0, UX_TRACE_ERRORS, 0, 0)

                    /* We must return an error.  */
                    return(UX_ENDPOINT_HANDLE_UNKNOWN);
                }
                break;
            }
        }
    }

    /* All endpoints have been mounted.  */
    return(UX_SUCCESS);
}

