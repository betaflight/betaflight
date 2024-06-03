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
/**   CDC ACM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_cdc_acm_capabilities_get             PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function obtains the entire cdc_acm configuration descriptors. */
/*    This is needed because the cdc_acm class needs to know if commands  */
/*    are routed through the comm interface or the data class.            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    cdc_acm                                 Pointer to cdc_acm class    */
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
/*    _ux_utility_memory_free               Release memory block          */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    _ux_host_class_cdc_acm_activate                                     */
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
UINT  _ux_host_class_cdc_acm_capabilities_get(UX_HOST_CLASS_CDC_ACM *cdc_acm)
{

UCHAR                       *descriptor;
UCHAR                       *saved_descriptor;
UX_ENDPOINT                 *control_endpoint;
UX_TRANSFER                 *transfer_request;
UX_CONFIGURATION            configuration;
UX_INTERFACE_DESCRIPTOR     interface_descriptor;
UINT                        status;
ULONG                       total_descriptor_length;
UCHAR                       descriptor_length;
UCHAR                       descriptor_type;
UCHAR                       descriptor_subtype;
ULONG                       interface_found;

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &cdc_acm -> ux_host_class_cdc_acm_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the descriptor.  */
    descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_CONFIGURATION_DESCRIPTOR_LENGTH);
    if (descriptor == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Save this descriptor address since we need to free it. */
    saved_descriptor = descriptor;

    /* Create a transfer request for the GET_DESCRIPTOR request.  */
    transfer_request -> ux_transfer_request_data_pointer =      descriptor;
    transfer_request -> ux_transfer_request_requested_length =  UX_CONFIGURATION_DESCRIPTOR_LENGTH;
    transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             UX_CONFIGURATION_DESCRIPTOR_ITEM << 8;
    transfer_request -> ux_transfer_request_index =             0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check for correct transfer and entire descriptor returned.  */
    if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == UX_CONFIGURATION_DESCRIPTOR_LENGTH))
    {

        /* The descriptor is in a packed format, parse it locally.  */
        _ux_utility_descriptor_parse(descriptor, _ux_system_configuration_descriptor_structure,
                                    UX_CONFIGURATION_DESCRIPTOR_ENTRIES, (UCHAR *) &configuration.ux_configuration_descriptor);

        /* Now we have the configuration descriptor which will tell us how many
           bytes there are in the entire descriptor.  */
        total_descriptor_length =  configuration.ux_configuration_descriptor.wTotalLength;

        /* Free the previous descriptor.  */
        _ux_utility_memory_free(descriptor);

        /* Allocate enough memory to read all descriptors attached
          to this configuration.  */
        descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, total_descriptor_length);
        if (descriptor == UX_NULL)
            return(UX_MEMORY_INSUFFICIENT);

        /* Save this descriptor address since we need to free it. */
        saved_descriptor = descriptor;

        /* Set the length we need to retrieve.  */
        transfer_request -> ux_transfer_request_requested_length =  total_descriptor_length;

        /* And reprogram the descriptor buffer address.  */
        transfer_request -> ux_transfer_request_data_pointer =  descriptor;

        /* Send request to HCD layer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check for correct transfer and entire descriptor returned.  */
        if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == total_descriptor_length))
        {

            /* Default is Interface descriptor not yet found.  */
            interface_found =  UX_FALSE;

            /* Scan the descriptor for the CDC Comm interface.  */
            while (total_descriptor_length)
            {

                /* Gather the length, type and subtype of the descriptor.  */
                descriptor_length =   *descriptor;
                descriptor_type =     *(descriptor + 1);
                descriptor_subtype =  *(descriptor + 2);

                /* Make sure this descriptor has at least the minimum length.  */
                if (descriptor_length < 3)
                {

                    /* Error trap. */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

                    /* If trace is enabled, insert this event into the trace buffer.  */
                    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

                    /* We can free the resource now.  */
                    _ux_utility_memory_free(saved_descriptor);

                    /* Descriptor is corrupted.  */
                    return(UX_DESCRIPTOR_CORRUPTED);
                }

                /* Process relative to descriptor type.  */
                switch (descriptor_type)
                {


                case UX_INTERFACE_DESCRIPTOR_ITEM:

                    /* Parse the interface descriptor and make it machine independent.  */
                    _ux_utility_descriptor_parse(descriptor, _ux_system_interface_descriptor_structure,
                                                    UX_INTERFACE_DESCRIPTOR_ENTRIES, (UCHAR *) &interface_descriptor);

                    /* Ensure we have the correct interface for Audio streaming.  */
                    if ((interface_descriptor.bInterfaceClass == UX_HOST_CLASS_CDC_CONTROL_CLASS) &&
                        ((interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_CDC_ACM_SUBCLASS) ||
                        (interface_descriptor.bInterfaceSubClass == UX_HOST_CLASS_CDC_DLC_SUBCLASS)))
                    {

                        /* Mark we have found it.  */
                        interface_found =  UX_TRUE;

                    }
                    else
                    {

                        /* Haven't found it.  */
                        interface_found =  UX_FALSE;
                    }
                    break;


                case UX_HOST_CLASS_CDC_ACM_CS_INTERFACE:

                    /* First make sure we have found the correct generic interface descriptor.  */
                    if ((interface_found == UX_TRUE) && (descriptor_subtype == UX_HOST_CLASS_CDC_ACM_CALL_MANAGEMENT_DESCRIPTOR))
                    {

                        /* Retrieve the bmCapabilities field which indicates how ACM commands are sent to the device.  */
                        cdc_acm -> ux_host_class_cdc_acm_capabilities  = *(descriptor + UX_HOST_CLASS_CDC_ACM_CALL_MANAGEMENT_CAPABILITIES);


                    }
                    break;
                }

                /* Verify if the descriptor is still valid.  */
                if (descriptor_length > total_descriptor_length)
                {

                    /* Error trap. */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

                    /* If trace is enabled, insert this event into the trace buffer.  */
                    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

                    /* We can free the resource now.  */
                    _ux_utility_memory_free(saved_descriptor);

                    return(UX_DESCRIPTOR_CORRUPTED);
                }

                /* Jump to the next descriptor if we have not reached the end.  */
                descriptor +=  descriptor_length;

                /* And adjust the length left to parse in the descriptor.  */
                total_descriptor_length -=  descriptor_length;
            }

            /* We can free the resource now.  */
            _ux_utility_memory_free(saved_descriptor);

            return(UX_SUCCESS);
        }
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(saved_descriptor);

    /* Return completion status.  */
    return(status);
}

