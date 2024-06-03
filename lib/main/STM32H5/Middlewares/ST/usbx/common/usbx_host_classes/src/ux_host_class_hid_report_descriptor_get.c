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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_report_descriptor_get            PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function gets the report descriptor and analyzes it.           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*    length                                Length of descriptor          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hid_global_item_parse  Parse global item             */ 
/*    _ux_host_class_hid_local_item_parse   Parse local item              */ 
/*    _ux_host_class_hid_report_item_analyse Analyze report               */ 
/*    _ux_host_class_hid_resources_free     Free HID resources            */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_report_descriptor_get(UX_HOST_CLASS_HID *hid, ULONG length)
{

UX_ENDPOINT             *control_endpoint;
UX_TRANSFER             *transfer_request;
UCHAR                   *descriptor;
UCHAR                   *start_descriptor;
UX_HOST_CLASS_HID_ITEM  item;
UINT                    status;


    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &hid -> ux_host_class_hid_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the report descriptor.  */
    descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, length);
    if (descriptor == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* We need to save the descriptor starting address.  */
    start_descriptor =  descriptor;

    /* Create a transfer_request for the GET_DESCRIPTOR request */
    transfer_request -> ux_transfer_request_data_pointer =      descriptor;
    transfer_request -> ux_transfer_request_requested_length =  length;
    transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =             UX_HOST_CLASS_HID_REPORT_DESCRIPTOR << 8;
    transfer_request -> ux_transfer_request_index =             hid -> ux_host_class_hid_interface -> ux_interface_descriptor.bInterfaceNumber;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check for correct transfer and entire descriptor returned.  */
    if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == length))
    {

        /* Parse the report descriptor and build the report items.  */
        while (length)
        {

            /* Get one item from the report and analyze it.  */
            _ux_host_class_hid_report_item_analyse(descriptor, &item);

            /* Point the descriptor right after the item identifier.  */
            descriptor +=  item.ux_host_class_hid_item_report_format;

            /* Process relative to the item type.  */
            switch (item.ux_host_class_hid_item_report_type)
            {

            case UX_HOST_CLASS_HID_TYPE_GLOBAL:

                /* This is a global item.  */
                status =  _ux_host_class_hid_global_item_parse(hid, &item, descriptor);
                break;

            
            case UX_HOST_CLASS_HID_TYPE_MAIN:

                /* This is a main item.  */
                status =  _ux_host_class_hid_main_item_parse(hid, &item, descriptor);
                break;


            case UX_HOST_CLASS_HID_TYPE_LOCAL:

                /* This is a local item.  */
                status =  _ux_host_class_hid_local_item_parse(hid, &item, descriptor);
                break;          

            default:

                /* This is a reserved item, meaning it shouldn't be used!  */

                /* Set status to error. The check after this switch statement 
                    will handle the rest.  */
                status =  UX_DESCRIPTOR_CORRUPTED;
                break;
            }

            /* Recheck the status code.  */
            if (status != UX_SUCCESS)
            {
                break;
            }

            /* Jump to the next item.  */
            descriptor +=  item.ux_host_class_hid_item_report_length;
        
            /* Verify that the report descriptor is not corrupted.  */
            if (length < item.ux_host_class_hid_item_report_length)
            {

                /* Return error status.  */
                status = (UX_DESCRIPTOR_CORRUPTED);
                break;
            }

            /* Adjust the length.  */
            length -=  (ULONG)(item.ux_host_class_hid_item_report_length + item.ux_host_class_hid_item_report_format);
        }
    }
    else

        /* Descriptor length error!  */
        status = UX_DESCRIPTOR_CORRUPTED;

    if (status != UX_SUCCESS)
    {
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* The descriptor is corrupted!  */
        _ux_host_class_hid_resources_free(hid);
    }

    /* Free used resources.  */
    _ux_utility_memory_free(start_descriptor);

    /* Return completion status.  */
    return(status);
}

