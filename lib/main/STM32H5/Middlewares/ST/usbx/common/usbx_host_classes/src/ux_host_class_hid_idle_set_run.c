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
/**   HID Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_stack.h"


#if defined(UX_HOST_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_hid_idle_set_run                     PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function performs a SET_IDLE to the HID device.                */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hid                                   Pointer to HID class          */
/*    idle_time                             Idle time                     */
/*    report_id                             Report ID                     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_class_instance_verify  Verify instance is valid      */
/*    _ux_host_stack_transfer_run           Process transfer states       */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    HID Class                                                           */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
UINT _ux_host_class_hid_idle_set_run(UX_HOST_CLASS_HID *hid, USHORT idle_time, USHORT report_id)
{

UX_INTERRUPT_SAVE_AREA
UX_DEVICE       *device;
UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UINT            status;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HID_IDLE_SET, hid, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_hid_name, (VOID *) hid) != UX_SUCCESS)
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

        hid -> ux_host_class_hid_status = UX_HOST_CLASS_INSTANCE_UNKNOWN;
        return(UX_STATE_EXIT);
    }

    /* Get device.  */
    device = hid -> ux_host_class_hid_device;

    /* Sanity check.  */
    if (device == UX_NULL ||
        device -> ux_device_handle != (ULONG)(ALIGN_TYPE)device)
    {
        hid -> ux_host_class_hid_status = UX_DEVICE_HANDLE_UNKNOWN;
        return(UX_STATE_EXIT);
    }

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Waiting transfer done.  */
    if (hid -> ux_host_class_hid_cmd_state == UX_STATE_WAIT)
    {

        /* Process background tasks.  */
        _ux_system_tasks_run();

        /* Check if transfer is done.  */
        if (transfer_request -> ux_transfer_request_state < UX_STATE_WAIT)
        {

            /* Unlock.  */
            hid -> ux_host_class_hid_flags &= ~UX_HOST_CLASS_HID_FLAG_LOCK;
            device -> ux_device_flags &= ~UX_DEVICE_FLAG_LOCK;

            /* We are done.  */
            hid -> ux_host_class_hid_cmd_state = UX_STATE_IDLE;
            hid -> ux_host_class_hid_status =
                        transfer_request -> ux_transfer_request_completion_code;
            return(UX_STATE_NEXT);
        }

        /* Keep waiting.  */
        return(UX_STATE_WAIT);
    }

    /* Check state or protection status.  */
    UX_DISABLE
    if ((hid -> ux_host_class_hid_flags & UX_HOST_CLASS_HID_FLAG_LOCK) ||
        (device -> ux_device_flags & UX_DEVICE_FLAG_LOCK))
    {

        /* Locked.  */
        UX_RESTORE
        return(UX_STATE_LOCK);
    }
    hid -> ux_host_class_hid_flags |= UX_HOST_CLASS_HID_FLAG_LOCK;
    device -> ux_device_flags |= UX_DEVICE_FLAG_LOCK;
    hid -> ux_host_class_hid_cmd_state = UX_STATE_WAIT;
    UX_RESTORE

    /* Create a transfer request for the SET_IDLE request.  */
    transfer_request -> ux_transfer_request_data_pointer =     UX_NULL;
    transfer_request -> ux_transfer_request_requested_length = 0;
    transfer_request -> ux_transfer_request_function =         UX_HOST_CLASS_HID_SET_IDLE;
    transfer_request -> ux_transfer_request_type =             UX_REQUEST_OUT | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =            (UINT)((idle_time << 8) | report_id);
    transfer_request -> ux_transfer_request_index =            hid -> ux_host_class_hid_interface -> ux_interface_descriptor.bInterfaceNumber;
    UX_TRANSFER_STATE_RESET(transfer_request);

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_run(transfer_request);
    if (status != UX_STATE_WAIT)
    {
        hid -> ux_host_class_hid_status =
                    transfer_request -> ux_transfer_request_completion_code;

        /* Unlock.  */
        hid -> ux_host_class_hid_flags &= ~UX_HOST_CLASS_HID_FLAG_LOCK;
        device -> ux_device_flags &= ~UX_DEVICE_FLAG_LOCK;

        hid -> ux_host_class_hid_cmd_state = (UCHAR)status;
    }

    /* Return the function status.  */
    return(status);
}
#endif
