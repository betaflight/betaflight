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
/**   Device PIMA Class                                                   */
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
/*    _ux_device_class_pima_control_request               PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function manages the based sent by the host on the control     */
/*    endpoints with a CLASS or VENDOR SPECIFIC type.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    pima                               Pointer to pima class            */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_device_stack_transfer_abort       Transfer abort                */
/*    _ux_device_class_pima_event_set       Put PIMA event into queue     */
/*    _ux_utility_short_put                 Put 16-bit value into buffer  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    PIMA Class                                                          */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved request handling,  */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_control_request(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_DCD                *dcd;
UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_TRANSFER           *transfer_request;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_CLASS              *class_ptr;
ULONG                       request;
ULONG                       request_length;
ULONG                       length;
UCHAR                       *request_data;
UX_SLAVE_CLASS_PIMA         *pima;
ULONG                       transaction_id;

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the pointer to the transfer request associated with the control endpoint.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Extract all necessary fields of the request.  */
    request =  *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);
    request_data = transfer_request -> ux_slave_transfer_request_data_pointer;
    request_length = _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

     /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the storage instance from this class container.  */
    pima =  (UX_SLAVE_CLASS_PIMA *) class_ptr -> ux_slave_class_instance;

    /* Here we proceed only the standard request we know of at the device level.  */
    switch (request)
    {

        case UX_DEVICE_CLASS_PIMA_REQUEST_CANCEL_COMMAND:

            /* Validate length.  */
            if (request_length != 6)
                return(UX_ERROR);

            /* Validate Cancellation code (0x4001).  */
            if (request_data[0] != 0x01 || request_data[1] != 0x40)
                return(UX_ERROR);

            /* Obtain TransactionID.  */
            transaction_id = _ux_utility_long_get(request_data + 2);
            if (transaction_id != pima -> ux_device_class_pima_transaction_id)
                return(UX_ERROR);

            /* Cancel data phase only.  */
            if (pima -> ux_device_class_pima_state > UX_DEVICE_CLASS_PIMA_PHASE_RESPONSE)
            {

                /* Abort, device is busy until data thread end.  */
                pima -> ux_device_class_pima_device_status = UX_DEVICE_CLASS_PIMA_RC_DEVICE_BUSY;

                if (pima -> ux_device_class_pima_state == UX_DEVICE_CLASS_PIMA_PHASE_DATA_IN)
                    _ux_device_stack_transfer_abort(&pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request, UX_ABORTED);
                else
                    _ux_device_stack_transfer_abort(&pima -> ux_device_class_pima_bulk_out_endpoint -> ux_slave_endpoint_transfer_request, UX_ABORTED);

                /* Invoke callback if available, let application cancel/close related objects.  */
                if (pima -> ux_device_class_pima_cancel)
                    pima -> ux_device_class_pima_cancel(pima);

                /* Set state to idle, time consuming threads detect it to break the loop.  */
                pima -> ux_device_class_pima_state = UX_DEVICE_CLASS_PIMA_PHASE_IDLE;
            }
            break ;

        case UX_DEVICE_CLASS_PIMA_REQUEST_STATUS_COMMAND:

            /* Validate length.  */
            if (request_length < 2)
                return(UX_ERROR);

            /* Fill the status data payload. Initialize length to 4.  */
            length = 4;

            /* Fill status.  */
            _ux_utility_short_put(request_data + 2,
                                pima -> ux_device_class_pima_device_status);

            /* Fill device status actual length.  */
            _ux_utility_short_put(request_data, (USHORT)length);
            length = UX_MIN(request_length, length);

            /* Change status, error is reported only once.  */
            pima->ux_device_class_pima_device_status = UX_DEVICE_CLASS_PIMA_RC_OK;

            /* We have a request to obtain the status of the MTP. */
            _ux_device_stack_transfer_request(transfer_request, length, request_length);
            break ;

        case UX_DEVICE_CLASS_PIMA_REQUEST_RESET_DEVICE:

            /* Clear stall.  */
            endpoint = pima -> ux_device_class_pima_bulk_in_endpoint;
            if (endpoint->ux_slave_endpoint_state == UX_ENDPOINT_HALTED)
            {
                dcd -> ux_slave_dcd_function(dcd, UX_DCD_RESET_ENDPOINT, endpoint);
                endpoint -> ux_slave_endpoint_state = UX_ENDPOINT_RESET;
            }
            else
                _ux_device_stack_transfer_abort(&endpoint->ux_slave_endpoint_transfer_request, UX_ABORTED);
            endpoint = pima -> ux_device_class_pima_bulk_out_endpoint;
            if (endpoint->ux_slave_endpoint_state == UX_ENDPOINT_HALTED)
            {
                dcd -> ux_slave_dcd_function(dcd, UX_DCD_RESET_ENDPOINT, endpoint);
                endpoint -> ux_slave_endpoint_state = UX_ENDPOINT_RESET;
            }
            else
                _ux_device_stack_transfer_abort(&endpoint->ux_slave_endpoint_transfer_request, UX_ABORTED);

            /* Reset session.  */
            pima -> ux_device_class_pima_device_reset(pima);

            /* Reset status.  */
            pima -> ux_device_class_pima_session_id = 0;
            pima -> ux_device_class_pima_device_status = UX_DEVICE_CLASS_PIMA_RC_OK;
            break;

        default:

            /* Unknown function. It's not handled.  */
            return(UX_ERROR);
    }

    /* It's handled.  */
    return(UX_SUCCESS);
}
