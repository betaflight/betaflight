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


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_hid_report_get                     PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function returns a report to the host.                         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    descriptor_type                       Descriptor type               */
/*    descriptor_index                      Index of descriptor           */
/*    host_length                           Length requested by host      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_class_hid_event_get        Get HID event                 */
/*    _ux_device_stack_transfer_request     Process transfer request      */
/*    _ux_utility_memory_set                Set memory                    */
/*    _ux_utility_memory_copy               Copy memory                   */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    Device Stack                                                        */
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_hid_report_get(UX_SLAVE_CLASS_HID *hid, ULONG descriptor_type, 
                                            ULONG request_index, ULONG host_length)
{

UX_SLAVE_DEVICE                 *device;
UX_SLAVE_TRANSFER               *transfer_request;
UX_SLAVE_ENDPOINT               *endpoint;
UCHAR                           report_id;
UCHAR                           report_type;
UX_SLAVE_CLASS_HID_EVENT        hid_event;
ULONG                           hid_event_length;
UCHAR                           *buffer;
UINT                            status =  UX_ERROR;

    UX_PARAMETER_NOT_USED(descriptor_type);
    UX_PARAMETER_NOT_USED(request_index);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_HID_REPORT_GET, hid, descriptor_type, request_index, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* Get the control endpoint associated with the device.  */
    endpoint =  &device -> ux_slave_device_control_endpoint;

    /* Get the pointer to the transfer request associated with the endpoint.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Get report ID (wValue.lower) and report type (wValue.higher).  */
    report_id   = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE + 0);
    report_type = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE + 1);

    /* Set the direction to OUT.  */
    transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_OUT;

    /* Prepare the event data payload from the hid event structure.  Get a pointer to the buffer area.  */
    buffer =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Initialize event fields.  */
    hid_event.ux_device_class_hid_event_report_id   = report_id;
    hid_event.ux_device_class_hid_event_report_type = report_type;
    hid_event.ux_device_class_hid_event_length      = UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH;

    /* If it's input report without ID try to get it from event queue head.  */
    if (report_type == UX_DEVICE_CLASS_HID_REPORT_TYPE_INPUT &&
        hid -> ux_device_class_hid_report_id != UX_TRUE)

        /* Check if we have an event to report.  */
        status = _ux_device_class_hid_event_get(hid, &hid_event);

    /* Try to get event from application callback.  */
    else
    {
        
        /* Let application fill event.  */
        if (hid -> ux_device_class_hid_get_callback != UX_NULL)
            status = hid -> ux_device_class_hid_get_callback(hid, &hid_event);
    }

    if (status == UX_SUCCESS)
    {

        /* Get the length to send back to the host.  */
        if (host_length < hid_event.ux_device_class_hid_event_length)
            hid_event_length =  host_length;
        else
            hid_event_length =  hid_event.ux_device_class_hid_event_length;
        if (hid_event_length > UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH)
            hid_event_length = UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH;

        /* First reset it.  */
        _ux_utility_memory_set(buffer, 0, hid_event_length); /* Use case of memset is verified. */

        /* Copy the event buffer into the target buffer.  */
        _ux_utility_memory_copy(buffer, hid_event.ux_device_class_hid_event_buffer, hid_event_length); /* Use case of memcpy is verified. */
    }
    else
    {

        /* There's no event, so send back zero'd memory.  */

        /* Get the length to send back to the host.  */
        if (host_length < UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH)
            hid_event_length =  host_length;
        else
            hid_event_length =  UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH;

        /* Reset it.  */
        _ux_utility_memory_set(buffer, 0, hid_event_length); /* Use case of memset is verified. */
    }

    /* We can send the report.  */
    status =  _ux_device_stack_transfer_request(transfer_request, hid_event_length, host_length);

    /* Return the status to the caller.  */
    return(status);
}

