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
/*    _ux_device_class_ccid_control_request               PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function manages the requests sent by the host on the control  */
/*    endpoints with a CLASS or VENDOR SPECIFIC type.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ccid                                  Pointer to ccid class         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_device_class_ccid_request_abort   Abort slot                    */
/*    _ux_utility_memory_copy               Copy memory                   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    CCID Class                                                          */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_ccid_control_request(UX_SLAVE_CLASS_COMMAND *command)
{
UX_DEVICE_CLASS_CCID                    *ccid;
UX_DEVICE_CLASS_CCID_PARAMETER          *parameter;
UX_SLAVE_CLASS                          *ccid_class;
UX_SLAVE_TRANSFER                       *transfer_request;
UX_SLAVE_DEVICE                         *device;
ULONG                                   request;
UCHAR                                   seq, slot;
ULONG                                   request_length;
ULONG                                   transmit_length;
UCHAR                                   *transmit_buffer;

    /* Get the class container.  */
    ccid_class =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    ccid = (UX_DEVICE_CLASS_CCID *) ccid_class -> ux_slave_class_instance;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the pointer to the transfer request associated with the control endpoint.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Extract all necessary fields of the request.  */
    request =  *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

    /* Pickup the request length.  */
    request_length =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

    /* Here we proceed only the standard request we know of at the device level.  */
    switch (request)
    {

    case UX_DEVICE_CLASS_CCID_ABORT:
        slot = transfer_request -> ux_slave_transfer_request_setup[UX_SETUP_VALUE];
        seq = transfer_request -> ux_slave_transfer_request_setup[UX_SETUP_VALUE + 1];
        return _ux_device_class_ccid_control_abort(ccid, slot, seq);

    case UX_DEVICE_CLASS_CCID_GET_CLOCK_FREQUENCIES:
        parameter = &ccid -> ux_device_class_ccid_parameter;

        /* Check bNumClockSuppored.  */
        if (parameter -> ux_device_class_ccid_clocks == UX_NULL ||
            parameter -> ux_device_class_ccid_n_clocks == 0)
            return(UX_ERROR);

        /* Calculate transmit length.  */
        transmit_length = parameter -> ux_device_class_ccid_n_clocks;
        if (UX_OVERFLOW_CHECK_MULC_ULONG(transmit_length, 4))
            return(UX_ERROR);
        transmit_length <<= 2;

        /* Update transmit buffer.  */
        transmit_buffer = (UCHAR *)parameter -> ux_device_class_ccid_clocks;
        break;

    case UX_DEVICE_CLASS_CCID_GET_DATA_RATES:
        parameter = &ccid -> ux_device_class_ccid_parameter;

        /* Check bNumDataRateSuppored.  */
        if (parameter -> ux_device_class_ccid_data_rates == UX_NULL ||
            parameter -> ux_device_class_ccid_n_data_rates == 0)
            return(UX_ERROR);

        /* Calculate transmit length.  */
        transmit_length = parameter -> ux_device_class_ccid_n_data_rates;
        if (UX_OVERFLOW_CHECK_MULC_ULONG(transmit_length, 4))
            return(UX_ERROR);
        transmit_length <<= 2;

        /* Update transmit buffer.  */
        transmit_buffer = (UCHAR *)parameter -> ux_device_class_ccid_data_rates;
        break ;

    default:

        /* Unknown function. It's not handled.  */
        return(UX_ERROR);
    }

    /* Limit transmit length.  */
    transmit_length = UX_MIN(transmit_length, UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH);
    transmit_length = UX_MIN(transmit_length, request_length);

    /* Copy data to transmit.  */
    _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer,
            transmit_buffer, transmit_length); /* Use case of memcpy is verified. */

    /* Transmit.  */
    _ux_device_stack_transfer_request(transfer_request, transmit_length, request_length);

    /* It's handled.  */
    return(UX_SUCCESS);
}
