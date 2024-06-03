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


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_storage_endpoints_get                PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function searches for the handle of the bulk out endpoint and  */
/*    optionally the bulk in endpoint.                                    */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage                               Pointer to storage class      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_interface_endpoint_get Get an endpoint pointer       */
/*    _ux_utility_memory_allocate           Allocate memory               */
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
/*                                            resulting in version 6.1    */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            use pre-calculated value    */
/*                                            instead of wMaxPacketSize,  */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            initial the timeout value,  */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_endpoints_get(UX_HOST_CLASS_STORAGE *storage)
{

UINT            endpoint_index;
UX_ENDPOINT     *endpoint;


    /* Search for the bulk OUT endpoint. It is attached to the interface container.  */
    for (endpoint_index = 0; endpoint_index < storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bNumEndpoints;
                        endpoint_index++)
    {

        /* Get an endpoint.  */
        _ux_host_stack_interface_endpoint_get(storage -> ux_host_class_storage_interface, endpoint_index, &endpoint);

        /* We have an endpoint. Check if endpoint is bulk and OUT.  */
        if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT) &&
            ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT))
        {

            /* We have found the bulk OUT endpoint, save it!  */
            storage -> ux_host_class_storage_bulk_out_endpoint =  endpoint;

            /* This transfer request always has the OUT direction.  */
            endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_OUT;

            break;
        }
    }

    /* The bulk OUT endpoint is mandatory.  */
    if (storage -> ux_host_class_storage_bulk_out_endpoint == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, storage, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_ENDPOINT_HANDLE_UNKNOWN);
    }

    /* Search for the bulk IN endpoint. It is attached to the interface container.  */
    for (endpoint_index = 0; endpoint_index < storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bNumEndpoints;
                        endpoint_index++)
    {

        /* Get an endpoint.  */
        _ux_host_stack_interface_endpoint_get(storage -> ux_host_class_storage_interface, endpoint_index, &endpoint);

        /* We have an endpoint. Check if endpoint is bulk and IN.  */
        if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN) &&
            ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT))
        {

            /* We have found the bulk IN endpoint, save it!  */
            storage -> ux_host_class_storage_bulk_in_endpoint =  endpoint;

            /* This transfer request always has the IN direction.  */
            endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_IN;

            break;
        }
    }

    /* The bulk IN endpoint is mandatory */
    if (storage -> ux_host_class_storage_bulk_in_endpoint == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, storage, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_ENDPOINT_HANDLE_UNKNOWN);
    }

    /* Set default transfer timeout value.  */
    endpoint = storage -> ux_host_class_storage_bulk_in_endpoint;
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_timeout_value =
                UX_MS_TO_TICK_NON_ZERO(UX_HOST_CLASS_STORAGE_TRANSFER_TIMEOUT);
    endpoint = storage -> ux_host_class_storage_bulk_out_endpoint;
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_timeout_value =
                UX_MS_TO_TICK_NON_ZERO(UX_HOST_CLASS_STORAGE_TRANSFER_TIMEOUT);

#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    /* Search the Interrupt IN endpoint. This endpoint is optional and only valid for
       storage devices that use the CBI protocol.  */
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceProtocol == UX_HOST_CLASS_STORAGE_PROTOCOL_CBI)
    {

        /* Yes, CBI protocol is present.  Search for Interrupt IN endpoint.  */
        for (endpoint_index = 0; endpoint_index < storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bNumEndpoints;
                            endpoint_index++)
        {

            /* Get an endpoint.  */
            _ux_host_stack_interface_endpoint_get(storage -> ux_host_class_storage_interface, endpoint_index, &endpoint);

            /* Check if endpoint is Interrupt and IN.  */
            if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN) &&
                ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT))
            {

                /* We have found the Interrupt IN endpoint, save it!  */
                storage -> ux_host_class_storage_interrupt_endpoint =  endpoint;

                /* This transfer request always has the IN direction.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type =  UX_REQUEST_IN;

                /* The size of the transfer is fixed to the endpoint size.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_requested_length =
                                            endpoint -> ux_endpoint_transfer_request.ux_transfer_request_packet_length;

                /* Get some memory for this data transfer.  */
                endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer =
                                                        _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY,
                                                                                endpoint -> ux_endpoint_descriptor.wMaxPacketSize);

                /* Check the memory pointer.  */
                if (endpoint -> ux_endpoint_transfer_request.ux_transfer_request_data_pointer == UX_NULL)
                    return(UX_MEMORY_INSUFFICIENT);

                break;
            }
        }

        if (storage -> ux_host_class_storage_interrupt_endpoint == UX_NULL)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, endpoint, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_ENDPOINT_HANDLE_UNKNOWN);
        }
    }
#endif

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

