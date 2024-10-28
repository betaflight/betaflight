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
/*    _ux_device_class_hid_report_set                     PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function gets from the control pipe a report to be set from    */
/*    the host.                                                           */
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
/*    _ux_utility_memory_copy               Memory copy                   */ 
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
UINT  _ux_device_class_hid_report_set(UX_SLAVE_CLASS_HID *hid, ULONG descriptor_type, 
                                            ULONG request_index, ULONG host_length)
{

UX_SLAVE_DEVICE                 *device;
UX_SLAVE_TRANSFER               *transfer_request;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_CLASS_HID_EVENT        hid_event;
UCHAR                           *hid_buffer;

    UX_PARAMETER_NOT_USED(request_index);
    UX_PARAMETER_NOT_USED(host_length);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_HID_REPORT_SET, hid, descriptor_type, request_index, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* Get the control endpoint associated with the device.  */
    endpoint =  &device -> ux_slave_device_control_endpoint;

    /* Get the pointer to the transfer request associated with the endpoint.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;
    
    /* Set the event type to OUTPUT.  */
    hid_event.ux_device_class_hid_event_report_type =  descriptor_type;
    
    /* Get HID data address.  */
    hid_buffer = transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Check for report ID in this HID descriptor.  */
    if (hid -> ux_device_class_hid_report_id == UX_TRUE)
    {
        /* Set the report ID, First byte of data payload.  */
        hid_event.ux_device_class_hid_event_report_id = (ULONG) *hid_buffer;

        /* Set the length = total length - report ID. */
        hid_event.ux_device_class_hid_event_length = transfer_request -> ux_slave_transfer_request_actual_length -1;
    
        /* Set HID data after report ID.  */
        hid_buffer++;
    }
        
    else
    {    
        /* Set the report ID, not used here.  */
        hid_event.ux_device_class_hid_event_report_id = 0;

        /* Set the length.  */
        hid_event.ux_device_class_hid_event_length = transfer_request -> ux_slave_transfer_request_actual_length;
    }
        
    /* Copy the buffer received from the host.  Check for overflow. */
    if (hid_event.ux_device_class_hid_event_length > UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH)
    
        /* Overflow detected.  */
        hid_event.ux_device_class_hid_event_length = UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH;        
        
    /* Now we can safely copy the payload.  */
    _ux_utility_memory_copy(hid_event.ux_device_class_hid_event_buffer, hid_buffer, 
                                hid_event.ux_device_class_hid_event_length); /* Use case of memcpy is verified. */

    /* If there is a callback defined by the application, send the hid event to it.  */
    if (hid -> ux_device_class_hid_callback != UX_NULL)
    
        /* Callback exists. */
        hid -> ux_device_class_hid_callback(hid, &hid_event);
        
    /* Return the status to the caller.  */
    return(UX_SUCCESS);
}

