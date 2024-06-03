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
/*    _ux_host_class_hid_descriptor_parse                 PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains the HID descriptor and parses it.             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hid_report_descriptor_get                            */ 
/*                                          Get HID report descriptor     */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_descriptor_parse          Parse descriptor              */ 
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used shared device config   */
/*                                            descriptor for enum scan,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_descriptor_parse(UX_HOST_CLASS_HID *hid)
{

UX_DEVICE                       *device;
UX_CONFIGURATION                *configuration;
UCHAR                           *descriptor;
UCHAR                           *start_descriptor;
UX_ENDPOINT                     *control_endpoint;
UX_TRANSFER                     *transfer_request;
UX_HID_DESCRIPTOR               hid_descriptor;
UX_INTERFACE_DESCRIPTOR         interface_descriptor;
UINT                            status;
ULONG                           total_configuration_length;
UINT                            descriptor_length;
UINT                            descriptor_type;                
ULONG                           current_interface;

    /* Get device, current configuration.  */
    device = hid -> ux_host_class_hid_device;
    configuration = device -> ux_device_current_configuration;
    total_configuration_length = configuration -> ux_configuration_descriptor.wTotalLength;

    /* Check if descriptor is previously saved.  */
    if (device -> ux_device_packed_configuration != UX_NULL)
    {
        start_descriptor = device -> ux_device_packed_configuration;
        descriptor = start_descriptor;
        status = UX_SUCCESS;
    }
    else
    {

        /* We need to get the default control endpoint transfer request pointer.  */
        control_endpoint =  &device -> ux_device_control_endpoint;
        transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

        /* Need to allocate memory for the descriptor. Since we do not know the size of the 
        descriptor, we first read the first bytes.  */
        descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, total_configuration_length);
        if (descriptor == UX_NULL)
            return(UX_MEMORY_INSUFFICIENT);

        /* Memorize the descriptor start address.  */
        start_descriptor =  descriptor;

        /* Create a transfer request for the GET_DESCRIPTOR request.  */
        transfer_request -> ux_transfer_request_data_pointer =      descriptor;
        transfer_request -> ux_transfer_request_requested_length =  total_configuration_length;
        transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
        transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
        transfer_request -> ux_transfer_request_value =             UX_CONFIGURATION_DESCRIPTOR_ITEM << 8;
        transfer_request -> ux_transfer_request_index =             0;

        /* Send request to HCD layer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check descriptor length.  */
        if (transfer_request -> ux_transfer_request_actual_length != total_configuration_length)
            status = UX_DESCRIPTOR_CORRUPTED;
    }
    
    /* Reset the current interface to keep compiler warnings happy. */
    current_interface = 0;

    /* Check for correct transfer and entire descriptor returned.  */
    if (status == UX_SUCCESS)
    {

        /* The HID descriptor is embedded within the configuration descriptor. We parse the 
            entire descriptor to locate the HID portion.  */
        while (total_configuration_length)
        {
    
            /* Gather the length and type of the descriptor.   */
            descriptor_length =  *descriptor;
            descriptor_type =    *(descriptor + 1);
    
            /* Make sure this descriptor has at least the minimum length.  */
            if (descriptor_length < 3)
            {

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

                /* Error, release the memory and return an error.  */
                _ux_utility_memory_free(start_descriptor);

                return(UX_DESCRIPTOR_CORRUPTED);
            }

            /* Check the type for an interface descriptor.  */
            if (descriptor_type == UX_INTERFACE_DESCRIPTOR_ITEM)
            {

                /* Parse the interface descriptor and make it machine independent.  */
                _ux_utility_descriptor_parse(descriptor,
                            _ux_system_interface_descriptor_structure,
                            UX_INTERFACE_DESCRIPTOR_ENTRIES,
                            (UCHAR *) &interface_descriptor);

                /* Memorize the interface we are scanning.  */
                current_interface = interface_descriptor.bInterfaceNumber;
            }
    
            /* Check the type for an interface descriptor.  */
            /* Check if the descriptor belongs to interface attached to this instance.  */
            if (descriptor_type == UX_HOST_CLASS_HID_DESCRIPTOR)
            {

                /* Ensure this interface is the one we need to scan.  */
                if (current_interface == hid -> ux_host_class_hid_interface -> ux_interface_descriptor.bInterfaceNumber)
                {

                    /* Obtain the length of the HID descriptor. We need to make machine
                        independent first.  */
                    _ux_utility_descriptor_parse(descriptor, _ux_system_hid_descriptor_structure,
                                                    UX_HID_DESCRIPTOR_ENTRIES, (UCHAR *) &hid_descriptor);

                    /* We have found the hid descriptor. Now we should get the HID report descriptor.  */
                    status =  _ux_host_class_hid_report_descriptor_get(hid, hid_descriptor.wItemLength);

                    /* No release, the memory will be released after all interface scan done(enumeration done).  */
                    device -> ux_device_packed_configuration = start_descriptor;

                    /* Return completion status.  */
                    return(status);
                }
            }
    
            /* Verify if the descriptor is still valid.  */
            if (descriptor_length > total_configuration_length)
            {

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

                /* Release the memory.  */
                _ux_utility_memory_free(start_descriptor);

                /* Return an error.  */
                return(UX_DESCRIPTOR_CORRUPTED);
            }
    
            /* Jump to the next descriptor if we have not reached the end.  */
            descriptor +=  descriptor_length;
    
            /* And adjust the length left to parse in the descriptor.  */
            total_configuration_length -=  descriptor_length;
        }
    }

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, start_descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Free all used resources.  */
    if (device -> ux_device_packed_configuration == UX_NULL)
        _ux_utility_memory_free(start_descriptor);

    /* Return an error.  */
    return(UX_DESCRIPTOR_CORRUPTED);
}

