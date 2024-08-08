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
/**   Audio Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_audio.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_audio_descriptor_get                 PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains the entire audio configuration descriptors.   */ 
/*    This is needed because the audio class has many class specific      */ 
/*    descriptors which describe the alternate settings that can be used. */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    audio                                 Pointer to audio class        */ 
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
/*    Audio Class                                                         */ 
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
UINT  _ux_host_class_audio_descriptor_get(UX_HOST_CLASS_AUDIO *audio)
{

UX_DEVICE               *device;
UX_CONFIGURATION        *configuration;
UCHAR *                 descriptor;
UX_ENDPOINT             *control_endpoint;
UX_TRANSFER             *transfer_request;
UINT                    status;
ULONG                   total_configuration_length;


    /* Get device, current configuration.  */
    device = audio -> ux_host_class_audio_device;
    configuration = device -> ux_device_current_configuration;

    /* Check if descriptor is previously saved.  */
    if (device -> ux_device_packed_configuration != UX_NULL)
    {
        audio -> ux_host_class_audio_configuration_descriptor =
                                    device -> ux_device_packed_configuration;
        audio -> ux_host_class_audio_configuration_descriptor_length =
                                    configuration -> ux_configuration_descriptor.wTotalLength;

        /* Descriptor must be kept after enum.  */
        device -> ux_device_packed_configuration_keep_count ++;
        return(UX_SUCCESS);
    }

    /* Get total length of configuration descriptors.  */
    total_configuration_length = configuration -> ux_configuration_descriptor.wTotalLength;

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the descriptor.  */
    descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, total_configuration_length);
    if (descriptor == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Create a transfer request for the GET_DESCRIPTOR request.  */
    transfer_request -> ux_transfer_request_data_pointer =      descriptor;
    transfer_request -> ux_transfer_request_requested_length =  total_configuration_length;
    transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             UX_CONFIGURATION_DESCRIPTOR_ITEM << 8;
    transfer_request -> ux_transfer_request_index =             0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check for correct transfer and entire descriptor returned.  */
    if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == total_configuration_length))
    {

        /* Save the address of the entire configuration descriptor of the audio device.  */            
        audio -> ux_host_class_audio_configuration_descriptor =  descriptor;

        /* Save the length of the entire descriptor too.  */            
        audio -> ux_host_class_audio_configuration_descriptor_length =  total_configuration_length;

        /* Keep descriptor to be shared for this device.  */
        device -> ux_device_packed_configuration = descriptor;
        device -> ux_device_packed_configuration_keep_count ++;

        /* We do not free the resource for the descriptor until the device is
            plugged out.  */
        return(UX_SUCCESS);
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(descriptor);

    /* Return completion status.  */
    return(status);
}

