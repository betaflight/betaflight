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
/*    _ux_device_class_hid_descriptor_send                PORTABLE C      */
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function sends back the class descriptor required by the host. */
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
/*    (ux_slave_dcd_function)               DCD dispatch function         */ 
/*    _ux_device_stack_transfer_request     Process transfer request      */
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
/*  08-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed HID descriptor search,*/
/*                                            resulting in version 6.1.8  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_hid_descriptor_send(UX_SLAVE_CLASS_HID *hid, ULONG descriptor_type, 
                                            ULONG request_index, ULONG host_length)
{

UX_SLAVE_DCD                    *dcd;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_TRANSFER               *transfer_request;
UX_SLAVE_ENDPOINT               *endpoint;
UCHAR *                         device_framework;
UCHAR *                         device_framework_end;
ULONG                           descriptor_length;
UINT                            status =  UX_ERROR;
ULONG                           length;
UCHAR                           interface_number = 0xFF;

    UX_PARAMETER_NOT_USED(request_index);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_HID_DESCRIPTOR_SEND, hid, descriptor_type, request_index, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;
    
    /* Get the control endpoint associated with the device.  */
    endpoint =  &device -> ux_slave_device_control_endpoint;

    /* Get the pointer to the transfer request associated with the endpoint.  */
    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

    /* Set the direction to OUT.  */
    transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_OUT;
    
    /* Shift the descriptor type in the low byte field.  */
    descriptor_type =  (UCHAR) ((descriptor_type >> 8) & 0xff);

    /* What type of descriptor do we need to return?  */
    switch (descriptor_type)
    {

    case UX_DEVICE_CLASS_HID_DESCRIPTOR_HID:
    
        /* We should have a HID descriptor as part of the config descriptor.  */
        device_framework =  _ux_system_slave -> ux_system_slave_device_framework;
        device_framework_end = device_framework + _ux_system_slave -> ux_system_slave_device_framework_length;

        /* Parse the device framework and locate the HID descriptor.  
           There is only one HID descriptor.  */
        while (device_framework < device_framework_end)
        {

            /* Get the type of the current descriptor.  */
            descriptor_type =  *(device_framework + 1);

            /* And its length.  */
            descriptor_length =  (ULONG) *device_framework;

            /* Save interface number for later check.  */
            if (descriptor_type == UX_INTERFACE_DESCRIPTOR_ITEM)
                interface_number = *(device_framework + 2);

            /* Check if this is a HID report descriptor.  */
            if ((descriptor_type == UX_DEVICE_CLASS_HID_DESCRIPTOR_HID) &&
                (interface_number == (UCHAR)request_index))
            {

                /* Ensure the host does not demand a length beyond our descriptor (Windows does that)
                   and do not return more than what is allowed.  */
                if (descriptor_length < host_length)
                    length =  descriptor_length;
                else                            
                    length =  host_length;                

                /* Check buffer length, since descriptor length may exceed buffer...  */
                if (length > UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH)
                {

                    /* Error trap. */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

                    /* If trace is enabled, insert this event into the trace buffer.  */
                    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

                    /* Stall the endpoint.  */
                    status =  dcd -> ux_slave_dcd_function(dcd, UX_DCD_STALL_ENDPOINT, endpoint);
                    break;
                }

                /* Copy the device descriptor into the transfer request memory.  */
                _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer, 
                                            device_framework, length); /* Use case of memcpy is verified. */

                /* We can return the configuration descriptor.  */
                status =  _ux_device_stack_transfer_request(transfer_request, length, host_length);
                break;
                
            }

            /* Point to the next descriptor.  */
            device_framework +=  descriptor_length;
        }

        /* Stall the endpoint if not found or corrupt.  */
        if (device_framework >= device_framework_end)
            status =  dcd -> ux_slave_dcd_function(dcd, UX_DCD_STALL_ENDPOINT, endpoint);

        break;

    case UX_DEVICE_CLASS_HID_DESCRIPTOR_REPORT:
        
        /* Get the length of entire configuration descriptor.  */
        descriptor_length =  hid -> ux_device_class_hid_report_length;

        /* Ensure the host does not demand a length beyond our descriptor (Windows does that)
           and do not return more than what is allowed.  */
        if (descriptor_length < host_length)
            length =  descriptor_length;
        else                            
            length =  host_length;                

        /* Check buffer length, since total descriptors length may exceed buffer...  */
        if (length > UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, hid, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* Stall the endpoint.  */
            status =  dcd -> ux_slave_dcd_function(dcd, UX_DCD_STALL_ENDPOINT, endpoint);
            break;
        }

        /* Copy the device descriptor into the transfer request memory.  */
        _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer, 
                                    hid -> ux_device_class_hid_report_address, length); /* Use case of memcpy is verified. */

        /* We can return the report descriptor.  */
        status =  _ux_device_stack_transfer_request(transfer_request, length, host_length);
        break;
        
    case UX_DEVICE_CLASS_HID_DESCRIPTOR_PHYSICAL:
    
        /* Not treated for now.  Fall through and Stall endpoint.  */
        
    default:

        /* Stall the endpoint.  */
        dcd -> ux_slave_dcd_function(dcd, UX_DCD_STALL_ENDPOINT, endpoint);
        return(UX_ERROR);
    }

    /* Return the status to the caller.  */
    return(status);
}

