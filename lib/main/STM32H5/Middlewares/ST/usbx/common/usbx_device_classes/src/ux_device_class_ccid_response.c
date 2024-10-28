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
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   Device CCID Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_ccid.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_ccid_response                      PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function sends bulk IN response of the USB CCID device.        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ccid                                  Pointer to ccid instance      */
/*    buffer                                Pointer to data buffer        */
/*    length                                Data length                   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Source Code                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_ccid_response(UX_DEVICE_CLASS_CCID *ccid, UCHAR *buffer, ULONG length)
{

UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_TRANSFER               *transfer;
UINT                            status;

    /* Get bulk IN endpoint.  */
    endpoint = ccid -> ux_device_class_ccid_endpoint_in;

    /* Get transfer request.  */
    transfer = &endpoint -> ux_slave_endpoint_transfer_request;

    /* Lock bulk IN.  */
    _ux_device_mutex_on(&ccid -> ux_device_class_ccid_response_mutex);

    /* Prepare data to transfer.  */
    if (length > 0 && buffer != UX_NULL &&
        transfer -> ux_slave_transfer_request_data_pointer != buffer)
    {
        _ux_utility_memory_copy(transfer -> ux_slave_transfer_request_data_pointer,
                                buffer, length); /* Use case of memcpy is verified. */
    }

    /* Transfer data.  */
    status = _ux_device_stack_transfer_request(transfer, length, length);

    /* Unlock bulk IN.  */
    _ux_device_mutex_off(&ccid -> ux_device_class_ccid_response_mutex);

    /* Return transfer status.  */
    return(status);
}
