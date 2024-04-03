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
/**   Host Stack                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_stack_device_descriptor_read               PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function reads the device descriptor.                          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    device                                Pointer to device             */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_descriptor_parse          Parse descriptor              */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Free memory block             */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Components                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added bMaxPacketSize0 check,*/
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            added class code checking,  */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_device_descriptor_read(UX_DEVICE *device)
{

UX_TRANSFER     *transfer_request;
UINT            status;
UCHAR *         descriptor;
UX_ENDPOINT     *control_endpoint;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_DESCRIPTOR_READ, device, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Retrieve the pointer to the control endpoint.  */
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the descriptor.  */
    descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_DEVICE_DESCRIPTOR_LENGTH);
    if (descriptor == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Create a transfer_request for the GET_DESCRIPTOR request. The first transfer_request asks 
       for the first 8 bytes only. This way we will know the real MaxPacketSize
       value for the control endpoint.  */
    transfer_request -> ux_transfer_request_data_pointer =      descriptor;
    transfer_request -> ux_transfer_request_requested_length =  8;
    transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             UX_DEVICE_DESCRIPTOR_ITEM << 8;
    transfer_request -> ux_transfer_request_index =             0;

#if defined(UX_HOST_STANDALONE)
    device -> ux_device_enum_trans = transfer_request;
    status = UX_SUCCESS;
    return(status);
#else

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check for correct transfer and entire descriptor returned.  */
    if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == 8))
    {

        /* Parse the device descriptor and create the local descriptor.  */
        _ux_utility_descriptor_parse(descriptor, _ux_system_device_descriptor_structure, UX_DEVICE_DESCRIPTOR_ENTRIES,
                                                                                (UCHAR *) &device -> ux_device_descriptor);
    }
    else
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(descriptor);

        /* Return completion status.  */
        return(status);             
    }

    /* Validate the bMaxPacketSize0.  */
    if (device -> ux_device_descriptor.bMaxPacketSize0 != 8 &&
        device -> ux_device_descriptor.bMaxPacketSize0 != 16 &&
        device -> ux_device_descriptor.bMaxPacketSize0 != 32 &&
        device -> ux_device_descriptor.bMaxPacketSize0 != 64)
    {
        _ux_utility_memory_free(descriptor);
        return(UX_DESCRIPTOR_CORRUPTED);
    }

#if defined(UX_HOST_DEVICE_CLASS_CODE_VALIDATION_ENABLE)

    /* Validate the USB-IF bDeviceClass class code.  */
    switch(device -> ux_device_descriptor.bDeviceClass)
    {
    case 0x00: /* Fall through.  */
    case 0x02: /* Fall through.  */
    case 0x09: /* Fall through.  */
    case 0x11: /* Fall through.  */
    case 0xDC: /* Fall through.  */
    case 0xEF: /* Fall through.  */
    case 0xFF:
        break;
    default:

        /* Invalid device class code.  */
        _ux_utility_memory_free(descriptor);
        return(UX_DESCRIPTOR_CORRUPTED);
    }
#endif

    /* Update the max packet size value for the endpoint.  */
    control_endpoint -> ux_endpoint_descriptor.wMaxPacketSize =  device -> ux_device_descriptor.bMaxPacketSize0;

    /* Create a transfer_request for the GET_DESCRIPTOR request. This time, we have the complete length */
    transfer_request -> ux_transfer_request_data_pointer =      descriptor;
    transfer_request -> ux_transfer_request_requested_length =  UX_DEVICE_DESCRIPTOR_LENGTH;
    transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             UX_DEVICE_DESCRIPTOR_ITEM << 8;
    transfer_request -> ux_transfer_request_index =             0;
    transfer_request -> ux_transfer_request_packet_length =     device -> ux_device_descriptor.bMaxPacketSize0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check for correct transfer and entire descriptor returned.  */
    if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == UX_DEVICE_DESCRIPTOR_LENGTH))
    {

        /* Parse the device descriptor and create the local descriptor.  */
        _ux_utility_descriptor_parse(descriptor, _ux_system_device_descriptor_structure, UX_DEVICE_DESCRIPTOR_ENTRIES,
                                                                    (UCHAR *) &device -> ux_device_descriptor);
    }
    else
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_DESCRIPTOR_CORRUPTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* The device descriptor does not contain the right amount of data. Maybe corruption.  */
        status =  UX_DESCRIPTOR_CORRUPTED;
    }
    
    /* Free all used resources.  */
    _ux_utility_memory_free(descriptor);

    /* Return completion status.  */
    return(status);
#endif
}

