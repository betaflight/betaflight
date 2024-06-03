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


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_pima_device_prop_value_set         PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function receives an device object value from the host.        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    pima                                  Pointer to pima class         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_endpoint_stall       Stall endpoint                */
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_device_class_pima_response_send   Send PIMA response            */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Storage Class                                                */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            updated status handling,    */
/*                                            improved sanity checks,     */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_device_prop_value_set(UX_SLAVE_CLASS_PIMA *pima, ULONG device_property_code)
{

UINT                        status;
UX_SLAVE_TRANSFER           *transfer_request;
UCHAR                       *device_property_value;
ULONG                       device_property_value_length;
UCHAR                       *device_property_value_pointer;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_GET_DEVICE_PROP_VALUE_SET, pima, device_property_code, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_out_endpoint -> ux_slave_endpoint_transfer_request;

    /* Obtain memory for this object info. Use the transfer request pre-allocated memory.  */
    device_property_value =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Get the data payload.  */
    status =  _ux_device_stack_transfer_request(transfer_request,
                                    UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH,
                                    UX_DEVICE_CLASS_PIMA_TRANSFER_BUFFER_LENGTH);

    /* Check if there was an error. If so, stall the endpoint.  */
    if (status != UX_SUCCESS)
    {

        /* Stall the endpoint.  */
        _ux_device_stack_endpoint_stall(pima -> ux_device_class_pima_bulk_out_endpoint);

        /* Return the status.  */
        return(status);
    }

    /* Allocate the device info pointer to the beginning of the dynamic object info field.  */
    device_property_value_pointer = device_property_value + UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;

    /* Obtain the length of the data payload.  */
    device_property_value_length = transfer_request -> ux_slave_transfer_request_actual_length;

    /* Ensure there is some data payload.  */
    if (device_property_value_length > UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE)
    {

        /* Take out the header.  */
        device_property_value_length -= UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE;

        /* Send the object to the application.  */
        status = pima -> ux_device_class_pima_device_prop_value_set(pima, device_property_code,
                                    device_property_value_pointer, device_property_value_length);

        /* Now we return a response with success.  */
        _ux_device_class_pima_response_send(pima, (status == UX_SUCCESS) ?
                                    UX_DEVICE_CLASS_PIMA_RC_OK : status, 0, 0, 0, 0);
    }
    else
    {

        /* We return an error.  */
        _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_INVALID_PARAMETER, 0, 0, 0, 0);

        /* Status error.  */
        status = UX_ERROR;
    }

    /* Return completion status.  */
    return(status);
}
