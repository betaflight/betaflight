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
/**   Device DFU Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_dfu.h"
#include "ux_device_stack.h"


#if UX_DEVICE_CLASS_DFU_STATUS_MODE == 1
static inline VOID _ux_device_class_dfu_status_get(UX_SLAVE_CLASS_DFU *,
    UX_SLAVE_TRANSFER *, UCHAR, UCHAR, UCHAR, UCHAR);
#endif


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_dfu_control_request                PORTABLE C      */
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
/*    dfu                                       Pointer to dfu class      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_endpoint_stall       Endpoint stall                */
/*    _ux_device_stack_transfer_request     Transfer request              */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    DFU Class                                                           */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added DFU_UPLOAD support,   */
/*                                            removed block count (it's   */
/*                                            from host request wValue),  */
/*                                            resulting in version 6.1.6  */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warning,      */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added UPLOAD length check,  */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            checked r/w callback status,*/
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            added UPLOAD length check   */
/*                                            in _UPLOAD_IDLE state,      */
/*                                            added DNLOAD REQ            */
/*                                            validation,                 */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_dfu_control_request(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_TRANSFER       *transfer_request;
UX_SLAVE_DEVICE         *device;
UX_SLAVE_CLASS          *class_ptr;
UX_SLAVE_CLASS_DFU      *dfu;

ULONG                   request;
ULONG                   request_type;
ULONG                   request_value;
ULONG                   request_length;
ULONG                   actual_length;
UINT                    status;
#if defined(UX_DEVICE_CLASS_DFU_CUSTOM_REQUEST_ENABLE) || (UX_DEVICE_CLASS_DFU_STATUS_MODE != 1)
ULONG                   media_status;
#endif


    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

     /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the storage instance from this class container.  */
    dfu =  (UX_SLAVE_CLASS_DFU *) class_ptr -> ux_slave_class_instance;

    /* Get the pointer to the transfer request associated with the control endpoint.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

#ifdef UX_DEVICE_CLASS_DFU_CUSTOM_REQUEST_ENABLE
    if (dfu -> ux_device_class_dfu_custom_request)
    {

        /* The status simply tells us if the registered callback handled the
            request - if there was an issue processing the request, it would've
            stalled the control endpoint, notifying the host (and not us).  */
        media_status = dfu -> ux_device_class_dfu_custom_request(dfu, transfer_request);

        /* Custom request handled.  */
        if (media_status == UX_SUCCESS)
            return(media_status);

        /* Try to handle with standard handler.  */
    }
#endif

    /* Extract all necessary fields of the request.  */
    request =  *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);
    request_type = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST_TYPE);

    /* Pickup the request wValue.  */
    request_value =    _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE);

    /* Pickup the request wLength.  */
    request_length =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

    /* What state are we in ? */
    switch (_ux_system_slave -> ux_system_slave_device_dfu_state_machine)
    {

        case UX_SYSTEM_DFU_STATE_APP_IDLE         :


            /* Here we process only the request we can accept in the APP IDLE state.  */
            switch (request)
            {

                case UX_SLAVE_CLASS_DFU_COMMAND_DETACH :

                    /* The host is asking for a Detach and switch to the DFU mode. Either we force the reset here or
                       we wait for a specified timer. If there is no reset while this timer is running we abandon
                       the DFU Detach.*/
                    if (_ux_system_slave -> ux_system_slave_device_dfu_capabilities &  UX_SLAVE_CLASS_DFU_CAPABILITY_WILL_DETACH)
                    {

                        /* Wake up the DFU thread and send a detach request..  */
                        _ux_device_class_dfu_event_flags_set(dfu, UX_DEVICE_CLASS_DFU_THREAD_EVENT_DISCONNECT);
                    }
                    else
                    {

                        /* We expect the host to issue a reset.  Arm a timer in the DFU thread.  */
                        _ux_device_class_dfu_event_flags_set(dfu, UX_DEVICE_CLASS_DFU_THREAD_EVENT_WAIT_RESET);
                    }

                    /* We can switch dfu state machine.  */
                    _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_APP_DETACH;

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATUS :

#if UX_DEVICE_CLASS_DFU_STATUS_MODE == 1
                    _ux_device_class_dfu_status_get(dfu, transfer_request, UX_FALSE, 0, 0, 0);
#else
                    /* Fill the status data payload.  First with status.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) dfu -> ux_slave_class_dfu_status;

                    /* Poll time out value is set to 1ms.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT) = UX_DW0(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 1) = UX_DW1(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 2) = UX_DW2(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);

                    /* Next state.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STATE) = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* String index set to 0.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STRING) = 0;
#endif

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH);

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATE :

                    /* Fill the status data payload.  First with state.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH);

                    break;

                default:

                    /* Unknown function. Stall the endpoint.  */
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                    break;
            }

            break;

        case UX_SYSTEM_DFU_STATE_APP_DETACH         :

            /* Here we process only the request we can accept in the APP DETACH state.  */
            switch (request)
            {

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATUS :

#if UX_DEVICE_CLASS_DFU_STATUS_MODE == 1
                    _ux_device_class_dfu_status_get(dfu, transfer_request, UX_FALSE, 0, 0, 0);
#else

                    /* Fill the status data payload.  First with status.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) dfu -> ux_slave_class_dfu_status;

                    /* Poll time out value is set to 1ms.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT) = UX_DW0(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 1) = UX_DW1(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 2) = UX_DW2(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);

                    /* Next state.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STATE) = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* String index set to 0.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STRING) = 0;
#endif

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH);

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATE :

                    /* Fill the status data payload.  First with state.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH);

                    break;

                default:

                    /* Unknown function. Stall the endpoint.  */
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                    break;
            }

            break;

        case UX_SYSTEM_DFU_STATE_DFU_IDLE         :

            /* Here we process only the request we can accept in the DFU mode IDLE state.  */
            switch (request)
            {

                case UX_SLAVE_CLASS_DFU_COMMAND_DOWNLOAD :

                    /* Command verify: check bmRequestType and data length.  */
                    if ((request_type != UX_DEVICE_CLASS_DFU_REQTYPE_INTERFACE_SET) ||
                        (request_length != transfer_request -> ux_slave_transfer_request_actual_length))
                    {

                        /* Zero length download is not accepted. Stall the endpoint.  */
                        _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);

                        /* In the system, state the DFU state machine to dfu ERROR.  */
                        _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;
                    }

                    /* We received a DOWNLOAD command. Check the length field of the request. It cannot be 0.  */
                    else if (request_length == 0)
                    {

                        /* Zero length download is not accepted. Stall the endpoint.  */
                        _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);

                        /* In the system, state the DFU state machine to dfu ERROR.  */
                        _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;

                    }
                    else
                    {

                        /* Have we declared a DOWNLOAD possible in our device framework ? */
                        if (_ux_system_slave -> ux_system_slave_device_dfu_capabilities & UX_SLAVE_CLASS_DFU_CAPABILITY_CAN_DOWNLOAD)
                        {

                            /* Send a notification to the application.  Begin of transfer.  */
                            dfu -> ux_slave_class_dfu_notify(dfu, UX_SLAVE_CLASS_DFU_NOTIFICATION_BEGIN_DOWNLOAD);

                            /* Write the first block to the firmware.  */
                            status = dfu -> ux_slave_class_dfu_write(dfu, request_value,
                                                                transfer_request -> ux_slave_transfer_request_data_pointer,
                                                                request_length,
                                                                &actual_length);

                            /* Application can actively reject and set error state.  */
                            if (status != UX_SUCCESS)
                            {
                                _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                                _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;
                                break;
                            }

                            /* In the system, state the DFU state machine to dfu DOWNLOAD SYNC.  */
                            _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_DNLOAD_SYNC;

                        }
                        else
                        {

                            /* Download is not accepted. Stall the endpoint.  */
                            _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);

                            /* In the system, state the DFU state machine to dfu ERROR.  */
                            _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;

                        }

                    }
                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_ABORT :

                        /* Send a notification to the application.  */
                        dfu -> ux_slave_class_dfu_notify(dfu, UX_SLAVE_CLASS_DFU_NOTIFICATION_ABORT_DOWNLOAD);

                        /* In the system, state the DFU state machine to dfu IDLE.  */
                        _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_IDLE;

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATUS :

#if UX_DEVICE_CLASS_DFU_STATUS_MODE == 1
                    _ux_device_class_dfu_status_get(dfu, transfer_request, UX_FALSE, 0, 0, 0);
#else

                    /* Fill the status data payload.  First with status.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) dfu -> ux_slave_class_dfu_status;

                    /* Poll time out value is set to 1ms.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT) = UX_DW0(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 1) = UX_DW1(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 2) = UX_DW2(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);

                    /* Next state.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STATE) = (UCHAR) UX_SYSTEM_DFU_STATE_DFU_IDLE;

                    /* String index set to 0.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STRING) = 0;
#endif

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH);

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATE :

                    /* Fill the status data payload.  First with state.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH);

                    break;

#ifndef UX_DEVICE_CLASS_DFU_UPLOAD_DISABLE
                case UX_SLAVE_CLASS_DFU_COMMAND_UPLOAD:

                    /* bitCanUpload != 1, or length = 0, or length > wTransferSize (we can support max of control buffer size).  */
                    if (!(_ux_system_slave -> ux_system_slave_device_dfu_capabilities & UX_SLAVE_CLASS_DFU_CAPABILITY_CAN_UPLOAD) ||
                        (request_length == 0) ||
                        (request_length > UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH))
                    {
                        _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);

                        /* In the system, state the DFU state machine to dfu ERROR.  */
                        _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;

                        break;
                    }

                    /* bitCanUpload = 1.  */

                    /* Send a notification to the application.  Begin of transfer.  */
                    dfu -> ux_slave_class_dfu_notify(dfu, UX_SLAVE_CLASS_DFU_NOTIFICATION_BEGIN_UPLOAD);

                    /* Read the first block to the firmware.  */
                    status = dfu -> ux_slave_class_dfu_read(dfu, request_value,
                                                        transfer_request -> ux_slave_transfer_request_data_pointer,
                                                        request_length,
                                                        &actual_length);

                    /* Application can actively reject and set error state.  */
                    if (status != UX_SUCCESS)
                    {
                        _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                        _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;
                        break;
                    }

                    /* In the system, state the DFU state machine to dfu UPLOAD IDLE.  */
                    _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_UPLOAD_IDLE;

                    /* We have a request to upload DFU firmware block. */
                    _ux_device_stack_transfer_request(transfer_request, actual_length, request_length);

                    break;
#endif

                default:

                    /* Unknown function. Stall the endpoint.  */
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);

                    /* In the system, state the DFU state machine to dfu ERROR.  */
                       _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;

                    break;
            }

            break;


        case UX_SYSTEM_DFU_STATE_DFU_DNLOAD_SYNC         :

            /* Here we process only the request we can accept in the DFU mode DOWNLOAD state.  */
            switch (request)
            {

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATUS :

#if UX_DEVICE_CLASS_DFU_STATUS_MODE == 1
                    _ux_device_class_dfu_status_get(dfu, transfer_request, UX_TRUE,
                            UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_DNLOAD_IDLE,
                            UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_DNBUSY,
                            UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_ERROR);
#else

                    /* Check if the device is still buys performing the write. Write could be delayed.  */
                    dfu -> ux_slave_class_dfu_get_status(dfu, &media_status);

                    /* Check status of device.  */
                    switch (media_status)
                    {

                        case     UX_SLAVE_CLASS_DFU_MEDIA_STATUS_OK        :

                            /* Set the next state for idle and no error status.  */
                            dfu -> ux_slave_class_dfu_status = UX_SLAVE_CLASS_DFU_STATUS_OK ;
                               _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_DNLOAD_IDLE;
                            break;

                        case     UX_SLAVE_CLASS_DFU_MEDIA_STATUS_BUSY    :

                            /* Set the next state for busy but no error status.  */
                            dfu -> ux_slave_class_dfu_status = UX_SLAVE_CLASS_DFU_STATUS_OK ;
                               _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_DNBUSY;
                            break;

                        case    UX_SLAVE_CLASS_DFU_MEDIA_STATUS_ERROR   :

                            /* Set the next state for busy and error status.  */
                            dfu -> ux_slave_class_dfu_status = UX_SLAVE_CLASS_DFU_STATUS_ERROR_WRITE ;
                               _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_ERROR;
                            break;

                    }

                    /* Fill the status data payload.  First with status.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) dfu -> ux_slave_class_dfu_status;

                    /* Poll time out value is set to 1ms.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT) = UX_DW0(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 1) = UX_DW1(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 2) = UX_DW2(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);

                    /* Next state.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STATE) = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* String index set to 0.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STRING) = 0;
#endif

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH);

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATE :

                    /* Fill the status data payload.  First with state.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH);

                    break;

                default:

                    /* Unknown function. Stall the endpoint.  */
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);

                    /* In the system, state the DFU state machine to dfu ERROR.  */
                       _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;

                    break;
            }

            break;

        case UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_DNLOAD_IDLE         :

            /* Here we process only the request we can accept in the DFU mode DNLOAD state.  */
            switch (request)
            {

                case UX_SLAVE_CLASS_DFU_COMMAND_DOWNLOAD :

                    /* Command verify: check bmRequestType and data length.  */
                    if ((request_type != UX_DEVICE_CLASS_DFU_REQTYPE_INTERFACE_SET) ||
                        (request_length != transfer_request -> ux_slave_transfer_request_actual_length))
                    {

                        /* Zero length download is not accepted. Stall the endpoint.  */
                        _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);

                        /* In the system, state the DFU state machine to dfu ERROR.  */
                        _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;
                    }

                    /* We received a DOWNLOAD command. Check the length field of the request. If it is 0,
                       we are done with the transfer.  */
                    else if (request_length == 0)
                    {

                        /* Send the notification of end of download to application.  */
                        dfu -> ux_slave_class_dfu_notify(dfu, UX_SLAVE_CLASS_DFU_NOTIFICATION_END_DOWNLOAD);

                        /* In the system, state the DFU state machine to DFU MANIFEST SYNCH.  */
                        _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_MANIFEST_SYNC;

                    }

                    else
                    {

                        /* Write the next block to the firmware.  */
                        status = dfu -> ux_slave_class_dfu_write(dfu, request_value,
                                                            transfer_request -> ux_slave_transfer_request_data_pointer,
                                                            request_length,
                                                            &actual_length);

                        /* Application can actively reject and set error state.  */
                        if (status != UX_SUCCESS)
                        {
                            _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                            _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;
                            break;
                        }

                        /* In the system, state the DFU state machine to dfu DOWNLOAD SYNC.  */
                        _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_DNLOAD_SYNC;
                    }

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_ABORT :

                    /* Send a notification to the application.  */
                    dfu -> ux_slave_class_dfu_notify(dfu, UX_SLAVE_CLASS_DFU_NOTIFICATION_ABORT_DOWNLOAD);

                    /* In the system, state the DFU state machine to dfu IDLE.  */
                    _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_IDLE;

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATUS :

#if UX_DEVICE_CLASS_DFU_STATUS_MODE == 1
                    _ux_device_class_dfu_status_get(dfu, transfer_request, UX_FALSE, 0, 0, 0);
#else

                    /* Fill the status data payload.  First with status.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) dfu -> ux_slave_class_dfu_status;

                    /* Poll time out value is set to 1ms.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT) = UX_DW0(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 1) = UX_DW1(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 2) = UX_DW2(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);

                    /* Next state.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STATE) = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* String index set to 0.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STRING) = 0;
#endif

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH);

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATE :

                    /* Fill the status data payload.  First with state.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH);

                    break;

                default:

                    /* Unknown function. Stall the endpoint.  */
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);

                    /* In the system, state the DFU state machine to dfu ERROR.  */
                       _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;

                    break;
            }

            break;

        case UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_MANIFEST_SYNC         :

            /* Here we process only the request we can accept in the MANIFEST SYNCH state.  */
            switch (request)
            {

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATUS :

#if UX_DEVICE_CLASS_DFU_STATUS_MODE == 1
                    _ux_device_class_dfu_status_get(dfu, transfer_request, UX_TRUE,
                            UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_MANIFEST_WAIT_RESET,
                            UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_MANIFEST,
                            UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_ERROR);
                    if ((_ux_system_slave -> ux_system_slave_device_dfu_state_machine ==
                            UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_MANIFEST_WAIT_RESET) &&
                        (_ux_system_slave -> ux_system_slave_device_dfu_capabilities &
                            UX_SLAVE_CLASS_DFU_CAPABILITY_WILL_DETACH))
                    {

                        /* Wake up the DFU thread and send a detach request..  */
                        _ux_device_class_dfu_event_flags_set(dfu, UX_DEVICE_CLASS_DFU_THREAD_EVENT_DISCONNECT);
                    }
#else

                    /* Check if the device is still buys performing the write. Write could be delayed.  */
                    dfu -> ux_slave_class_dfu_get_status(dfu, &media_status);

                    /* Check status of device.  */
                    switch (media_status)
                    {

                        case     UX_SLAVE_CLASS_DFU_MEDIA_STATUS_OK        :

                            /* Set the next state for wait reset and no error status.  */
                            dfu -> ux_slave_class_dfu_status = UX_SLAVE_CLASS_DFU_STATUS_OK ;
                               _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_MANIFEST_WAIT_RESET;

                            /* Check who is responsible for the RESET.  */
                            if (_ux_system_slave -> ux_system_slave_device_dfu_capabilities &  UX_SLAVE_CLASS_DFU_CAPABILITY_WILL_DETACH)
                            {

                                /* Wake up the DFU thread and send a detach request..  */
                                _ux_device_class_dfu_event_flags_set(dfu, UX_DEVICE_CLASS_DFU_THREAD_EVENT_DISCONNECT);

                            }

                            break;

                        case     UX_SLAVE_CLASS_DFU_MEDIA_STATUS_BUSY    :

                            /* Set the next state for busy but no error status.  */
                            dfu -> ux_slave_class_dfu_status = UX_SLAVE_CLASS_DFU_STATUS_OK ;
                               _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_MANIFEST;
                            break;

                        case    UX_SLAVE_CLASS_DFU_MEDIA_STATUS_ERROR   :

                            /* Set the next state for busy and error status.  */
                            dfu -> ux_slave_class_dfu_status = UX_SLAVE_CLASS_DFU_STATUS_ERROR_WRITE ;
                               _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_ERROR;
                            break;
                    }

                    /* Fill the status data payload.  First with status.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) dfu -> ux_slave_class_dfu_status;

                    /* Poll time out value is set to 1ms.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT) = UX_DW0(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 1) = UX_DW1(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 2) = UX_DW2(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);

                    /* Next state.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STATE) = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* String index set to 0.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STRING) = 0;
#endif

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH);

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATE :

                    /* Fill the status data payload.  First with state.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH);

                    break;

                default:

                    /* Unknown function. Stall the endpoint.  */
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                    break;
            }

            break;

        case UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_ERROR         :

            /* Here we process only the request we can accept in the ERROR state.  */
            switch (request)
            {

#ifdef UX_DEVICE_CLASS_DFU_ERROR_GET_ENABLE
                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATUS :

                    /* Fill the status data payload.  First with status.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) dfu -> ux_slave_class_dfu_status;

                    /* Poll time out value is set to 1ms.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT) = 1;
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 1) = 0;
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 2) = 0;

                    /* Next state.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STATE) = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* String index set to 0.  */
                    *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STRING) = 0;

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH);

                    break;

                case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATE :

                    /* Fill the status data payload.  First with state.  */
                    *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                    /* We have a request to obtain the status of the DFU instance. */
                    _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH);

                    break;
#endif

                case UX_SLAVE_CLASS_DFU_COMMAND_CLEAR_STATUS :

                    /* In the system, state the DFU state machine to dfu IDLE.  */
                    dfu -> ux_slave_class_dfu_status = UX_SLAVE_CLASS_DFU_MEDIA_STATUS_OK;
                    _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_IDLE;

                    break;

                default:

                    /* Unknown function. Stall the endpoint.  */
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                    break;

            }

            break;

#ifndef UX_DEVICE_CLASS_DFU_UPLOAD_DISABLE
        case UX_SLAVE_CLASS_DFU_STATUS_STATE_DFU_UPLOAD_IDLE: /* bitCanUpload == 1.  */

            /* Here we process only the request we can accept in the DFU mode UPLOAD IDLE state.  */
            switch (request)
            {

            case UX_SLAVE_CLASS_DFU_COMMAND_UPLOAD:

                /* Check if length > wTransferSize (we can support max of control buffer size).  */
                if (request_length > UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH)
                {
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                    _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;
                    break;
                }

                /* Length 0 case undefined, just keep state.  */
                if (request_length == 0)
                    break;

                /* We received a UPLOAD command with length > 0.  */

                /* Read the next block from the firmware.  */
                status = dfu -> ux_slave_class_dfu_read(dfu, request_value,
                                                    transfer_request -> ux_slave_transfer_request_data_pointer,
                                                    request_length,
                                                    &actual_length);

                /* Application can actively reject and set error state.  */
                if (status != UX_SUCCESS)
                {
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                    _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;
                    break;
                }

                /* If it's short frame, switch to dfu IDLE.  */
                if (actual_length < request_length)
                {

                    /* Send a notification to the application.  */
                    dfu -> ux_slave_class_dfu_notify(dfu, UX_SLAVE_CLASS_DFU_NOTIFICATION_END_UPLOAD);

                    /* In the system, state the DFU state machine to dfu IDLE.  */
                    _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_IDLE;
                }

                /* We have a request to upload DFU firmware block. */
                _ux_device_stack_transfer_request(transfer_request, actual_length, request_length);

                break;

            case UX_SLAVE_CLASS_DFU_COMMAND_ABORT :

                    /* Send a notification to the application.  */
                    dfu -> ux_slave_class_dfu_notify(dfu, UX_SLAVE_CLASS_DFU_NOTIFICATION_ABORT_UPLOAD);

                    /* In the system, state the DFU state machine to dfu IDLE.  */
                    _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_IDLE;

                    break;

            case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATUS :

#if UX_DEVICE_CLASS_DFU_STATUS_MODE == 1
                    _ux_device_class_dfu_status_get(dfu, transfer_request, UX_FALSE, 0, 0, 0);
#else

                /* Fill the status data payload.  First with status.  */
                *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) dfu -> ux_slave_class_dfu_status;

                /* Poll time out value is set to 1ms.  */
                *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT) = UX_DW0(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 1) = UX_DW1(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);
                *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 2) = UX_DW2(UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT);

                /* Next state.  */
                *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STATE) = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                /* String index set to 0.  */
                *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_DFU_GET_STATUS_STRING) = 0;
#endif

                /* We have a request to obtain the status of the DFU instance. */
                _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATUS_LENGTH);

                break;

            case UX_SLAVE_CLASS_DFU_COMMAND_GET_STATE :

                /* Fill the status data payload.  First with state.  */
                *transfer_request -> ux_slave_transfer_request_data_pointer = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

                /* We have a request to obtain the status of the DFU instance. */
                _ux_device_stack_transfer_request(transfer_request, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH, UX_SLAVE_CLASS_DFU_GET_STATE_LENGTH);

                break;

            default:

                /* Unknown function. Stall the endpoint.  */
                _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);

                /* In the system, state the DFU state machine to dfu ERROR.  */
                _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_ERROR;

                break;
            }

            break;
#endif

        default:

            /* Unknown state. Should not happen.  */
            return(UX_ERROR);
    }

    return(UX_SUCCESS);
}

#if UX_DEVICE_CLASS_DFU_STATUS_MODE == 1
static inline VOID _ux_device_class_dfu_status_get(UX_SLAVE_CLASS_DFU *dfu,
    UX_SLAVE_TRANSFER *transfer,
    UCHAR move_state,
    UCHAR state_ok, UCHAR state_busy, UCHAR state_error)
{
ULONG media_status = ((UX_SLAVE_CLASS_DFU_MEDIA_STATUS_OK) |
                        (UX_SLAVE_CLASS_DFU_STATUS_OK << 4) |
                        (UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT << 8));
UCHAR *buffer = transfer -> ux_slave_transfer_request_data_pointer;
ULONG dfu_status, dfu_polltimeout;

    /* Get status from application.  */
    dfu -> ux_slave_class_dfu_get_status(dfu, &media_status);

    /* Extract bStatus and bwPollTimeout.  */
    dfu_status = (media_status >> 4) & 0xFu;
    dfu_polltimeout = (media_status >> 8) & 0xFFFFFFu;
    dfu -> ux_slave_class_dfu_status = dfu_status;

    /* Move state based on returned status.  */
    if (move_state)
    {

        /* OK/BUSY/ERROR ? */
        switch((media_status & 0xF))
        {
        case UX_SLAVE_CLASS_DFU_MEDIA_STATUS_OK:
            _ux_system_slave -> ux_system_slave_device_dfu_state_machine = state_ok;
            break;
        case UX_SLAVE_CLASS_DFU_MEDIA_STATUS_BUSY:
            _ux_system_slave -> ux_system_slave_device_dfu_state_machine = state_busy;
            break;
        case UX_SLAVE_CLASS_DFU_MEDIA_STATUS_ERROR:
        default:
            _ux_system_slave -> ux_system_slave_device_dfu_state_machine = state_error;
            break;
        }
    }

    /* Fill the status data payload.  First with status.  */
    *buffer = (UCHAR) dfu_status;

    /* Poll time out value is set to 1ms.  */
    *(buffer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT) = UX_DW0(dfu_polltimeout);
    *(buffer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 1) = UX_DW1(dfu_polltimeout);
    *(buffer + UX_SLAVE_CLASS_DFU_GET_STATUS_POLL_TIMEOUT + 2) = UX_DW2(dfu_polltimeout);

    /* Next state.  */
    *(buffer + UX_SLAVE_CLASS_DFU_GET_STATUS_STATE) = (UCHAR) _ux_system_slave -> ux_system_slave_device_dfu_state_machine;

    /* String index set to 0.  */
    *(buffer + UX_SLAVE_CLASS_DFU_GET_STATUS_STRING) = 0;
}
#endif
