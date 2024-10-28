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
/**   Device HID Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_hid.h"
#include "ux_device_stack.h"


#if defined(UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT) && defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_hid_receiver_tasks_run             PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is task function of the hid interrupt OUT endpoint.   */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hid                                       HID instance              */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_run         Request transfer              */
/*    _ux_utility_memory_copy               Copy memory                   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Device HID                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_hid_receiver_tasks_run(UX_SLAVE_CLASS_HID *hid)
{

UX_SLAVE_DEVICE                     *device;
UX_DEVICE_CLASS_HID_RECEIVER        *receiver;
UX_DEVICE_CLASS_HID_RECEIVED_EVENT  *pos;
UCHAR                               *next_pos;
UX_SLAVE_ENDPOINT                   *endpoint;
UX_SLAVE_TRANSFER                   *transfer;
UINT                                status;
UCHAR                               *buffer;
ULONG                               temp;


    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Check device state.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
        return(UX_STATE_EXIT);

    /* Get receiver instance.  */
    receiver = hid -> ux_device_class_hid_receiver;
    if (receiver == UX_NULL)
        return(UX_STATE_EXIT);

    /* Get endpoint.  */
    endpoint = hid -> ux_device_class_hid_read_endpoint;
    if (endpoint == UX_NULL)
        return(UX_STATE_EXIT);

    /* Event buffer available, issue request to get data.  */
    transfer = &endpoint -> ux_slave_endpoint_transfer_request;

    /* Run read/receiver states.  */
    switch(hid -> ux_device_class_hid_read_state)
    {
    case UX_DEVICE_CLASS_HID_RECEIVER_START:

        /* Check if there is buffer available.  */
        pos = receiver -> ux_device_class_hid_receiver_event_save_pos;
        if (pos -> ux_device_class_hid_received_event_length != 0)
        {

            /* Buffer is full, keep waiting.  */
            return(UX_STATE_IDLE);
        }

        /* Set request length.  */
        hid -> ux_device_class_hid_read_requested_length =
                    receiver -> ux_device_class_hid_receiver_event_buffer_size;

        /* Move state.  */
        hid -> ux_device_class_hid_read_state = UX_DEVICE_CLASS_HID_RECEIVER_WAIT;

        /* Fall through.  */
    case UX_DEVICE_CLASS_HID_RECEIVER_WAIT:

        /* Send the request to the device controller.  */
        status =  _ux_device_stack_transfer_run(transfer,
                            hid -> ux_device_class_hid_read_requested_length,
                            hid -> ux_device_class_hid_read_requested_length);

        /* Error case.  */
        if (status < UX_STATE_NEXT)
        {

            hid -> ux_device_class_hid_read_state = UX_DEVICE_CLASS_HID_RECEIVER_ERROR;
            hid -> ux_device_class_hid_read_status =
                transfer -> ux_slave_transfer_request_completion_code;
            return(UX_STATE_ERROR);
        }

        /* Success case.  */
        if (status == UX_STATE_NEXT)
        {

            /* Ignore ZLP case.  */
            if (transfer -> ux_slave_transfer_request_actual_length == 0)
            {

                /* Restart receiving.  */
                hid -> ux_device_class_hid_read_state = UX_DEVICE_CLASS_HID_RECEIVER_START;
                return(UX_STATE_NEXT);
            }

            /* Save received event data and length.  */
            pos = receiver -> ux_device_class_hid_receiver_event_save_pos;
            buffer = (UCHAR *)&pos -> ux_device_class_hid_received_event_data;
            temp = transfer -> ux_slave_transfer_request_actual_length;
            _ux_utility_memory_copy(buffer,
                            transfer -> ux_slave_transfer_request_data_pointer,
                            temp); /* Use case of memcpy is verified. */

            /* Advance the save position.  */
            next_pos = (UCHAR *)pos + receiver -> ux_device_class_hid_receiver_event_buffer_size + sizeof(ULONG);
            if (next_pos >= (UCHAR *)receiver -> ux_device_class_hid_receiver_events_end)
                next_pos = (UCHAR *)receiver -> ux_device_class_hid_receiver_events;
            receiver -> ux_device_class_hid_receiver_event_save_pos = (UX_DEVICE_CLASS_HID_RECEIVED_EVENT *)next_pos;

            /* Save received data length (it's valid now).  */
            pos -> ux_device_class_hid_received_event_length = temp;

            /* Notify application that a event is received.  */
            if (receiver -> ux_device_class_hid_receiver_event_callback)
                receiver -> ux_device_class_hid_receiver_event_callback(hid);

            /* Restart receiving.  */
            hid -> ux_device_class_hid_read_state = UX_DEVICE_CLASS_HID_RECEIVER_START;
            return(UX_STATE_NEXT);
        }

        /* Keep waiting.  */
        return(UX_STATE_WAIT);

    default: /* Nothing to do by default.  */
        break;
    }

    /* Task is idle.  */
    return(UX_STATE_IDLE);
}
#endif
