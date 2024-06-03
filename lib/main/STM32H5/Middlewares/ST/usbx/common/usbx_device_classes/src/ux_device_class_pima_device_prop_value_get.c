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
/**   Device Pima Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_pima.h"
#include "ux_device_stack.h"


#if UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH < UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE
#error UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH too small, please check
#endif

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_pima_device_prop_value_get         PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*   Return an Device Property Value.                                     */
/*                                                                        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    pima                                  Pointer to pima class         */
/*    device_property_code                  Device Property code for      */
/*                                          which the value is obtained.  */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_utility_short_put                 Put 16-bit value              */
/*    _ux_utility_long_put                  Put 32-bit value              */
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_device_class_pima_response_send   Send PIMA response            */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Pima Class                                                   */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            updated status handling,    */
/*                                            improved sanity checks,     */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_device_prop_value_get(UX_SLAVE_CLASS_PIMA *pima,
                                                    ULONG device_property_code)
{

UINT                                status;
UX_SLAVE_TRANSFER                   *transfer_request;
UCHAR                               *pima_data_buffer;
ULONG                               device_property_value_length;
UCHAR                               *device_property_value;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_GET_DEVICE_PROP_VALUE, pima, device_property_code, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;

    /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
    pima_data_buffer =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Fill in the data container type.  */
    _ux_utility_short_put(pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TYPE,
                            UX_DEVICE_CLASS_PIMA_CT_DATA_BLOCK);

    /* Fill in the data code.  */
    _ux_utility_short_put(pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_CODE,
                            UX_DEVICE_CLASS_PIMA_OC_GET_DEVICE_PROP_VALUE);

    /* Fill in the Transaction ID.  */
    _ux_utility_long_put(pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_TRANSACTION_ID,
                            pima -> ux_device_class_pima_transaction_id);

    /* Ask the application to retrieve for us the device prop value.  */
    device_property_value_length = UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD;
    status = pima -> ux_device_class_pima_device_prop_value_get(pima, device_property_code,
                            &device_property_value, &device_property_value_length);

    /* Result should always be OK, but to be sure .... */
    if (status != UX_SUCCESS)
    {

        /* We return an error.  */
        _ux_device_class_pima_response_send(pima, status, 0, 0, 0, 0);
    }
    else
    {

        /* Ensure the application's data can fit in the endpoint's data buffer.  */
        if (device_property_value_length > UX_DEVICE_CLASS_PIMA_MAX_PAYLOAD)
        {

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* We return an error.  */
            _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_GENERAL_ERROR, 0, 0, 0, 0);

            /* Return overflow error.  */
            return(UX_MEMORY_INSUFFICIENT);
        }

        /* Copy the property dataset into the local buffer.  */
        _ux_utility_memory_copy(pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE,
                        device_property_value, device_property_value_length); /* Use case of memcpy is verified. */

        /* Add the header size to the payload.  */
        device_property_value_length += UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;

        /* Fill in the size of the response header.  */
        _ux_utility_long_put(pima_data_buffer + UX_DEVICE_CLASS_PIMA_DATA_HEADER_LENGTH,
                        device_property_value_length);

        /* Send a data payload with the device prop value.  */
        status =  _ux_device_stack_transfer_request(transfer_request, device_property_value_length, 0);

        /* Now we return a response with success.  */
        _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);
    }

    /* Return completion status.  */
    return(status);
}
