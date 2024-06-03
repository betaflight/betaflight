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


#if defined(UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT) && !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_hid_receiver_thread                PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the thread of the hid interrupt OUT endpoint       */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hid_class                                 Address of hid class      */
/*                                                container               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_event_flags_get           Get event flags               */
/*    _ux_device_class_hid_event_get        Get HID event                 */
/*    _ux_device_stack_transfer_request     Request transfer              */
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_utility_thread_suspend            Suspend thread                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX                                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added receiver callback,    */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_hid_receiver_thread(ULONG hid_instance)
{

UX_SLAVE_CLASS_HID                  *hid;
UX_SLAVE_DEVICE                     *device;
UX_DEVICE_CLASS_HID_RECEIVER        *receiver;
UX_DEVICE_CLASS_HID_RECEIVED_EVENT  *pos;
UCHAR                               *next_pos;
UX_SLAVE_TRANSFER                   *transfer;
UINT                                status;
UCHAR                               *buffer;
ULONG                               temp;


    /* Cast properly the hid instance.  */
    UX_THREAD_EXTENSION_PTR_GET(hid, UX_SLAVE_CLASS_HID, hid_instance)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get receiver instance.  */
    receiver = hid -> ux_device_class_hid_receiver;

    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {

        /* Check device state.  */
        if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
        {

            /* We need to suspend ourselves. We will be resumed by the device enumeration module.  */
            _ux_utility_thread_suspend(&receiver -> ux_device_class_hid_receiver_thread);
            continue;
        }

        /* Check if there is buffer available.  */
        pos = receiver -> ux_device_class_hid_receiver_event_save_pos;
        if (pos -> ux_device_class_hid_received_event_length != 0)
        {

            /* Wait before check again.  */
            status = _ux_utility_event_flags_get(
                                &hid -> ux_device_class_hid_event_flags_group,
                                UX_DEVICE_CLASS_HID_RECEIVER_RESTART,
                                UX_OR_CLEAR, &temp, 100);
            if (status != UX_SUCCESS)
            {

                /* Keep checking before a good state.  */
                continue;
            }
        }

        /* Event buffer available, issue request to get data.  */
        transfer = &hid -> ux_device_class_hid_read_endpoint -> ux_slave_endpoint_transfer_request;

        /* Protect read.  */
        _ux_device_mutex_on(&hid -> ux_device_class_hid_read_mutex);

        /* Issue the transfer request.  */
        status = _ux_device_stack_transfer_request(transfer, 
                    receiver -> ux_device_class_hid_receiver_event_buffer_size,
                    receiver -> ux_device_class_hid_receiver_event_buffer_size);

        /* Check status and ignore ZLPs.  */
        if ((status != UX_SUCCESS) ||
            (transfer -> ux_slave_transfer_request_actual_length == 0))
        {
            _ux_device_mutex_off(&hid -> ux_device_class_hid_read_mutex);
            continue;
        }

        /* Save received event data and length.  */
        buffer = (UCHAR *)&pos -> ux_device_class_hid_received_event_data;
        temp = transfer -> ux_slave_transfer_request_actual_length;
        _ux_utility_memory_copy(buffer,
                        transfer -> ux_slave_transfer_request_data_pointer,
                        temp); /* Use case of memcpy is verified. */

        /* Unprotect read.  */
        _ux_device_mutex_off(&hid -> ux_device_class_hid_read_mutex);

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
    }
}
#endif
