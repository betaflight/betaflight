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
/**   Video Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_video.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_video_channel_start                  PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function starts the video channel.                             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    video                                 Pointer to video class        */ 
/*    channel_parameter                     Pointer to video channel      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_video_alternate_setting_locate                       */
/*                                          Search alternate setting      */
/*    _ux_host_stack_interface_setting_select                             */
/*                                          Select alternate setting      */
/*    _ux_host_stack_interface_endpoint_get Get interface endpoint        */
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_host_semaphore_get                Get semaphore                 */ 
/*    _ux_host_semaphore_put                Release semaphore             */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*    _ux_utility_long_get                  Get 32-bit value              */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Video Class                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed bandwidth check,      */
/*                                            saved max payload size,     */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_channel_start(UX_HOST_CLASS_VIDEO *video, UX_HOST_CLASS_VIDEO_PARAMETER_CHANNEL *video_parameter)
{

UX_ENDPOINT             *control_endpoint;
UX_TRANSFER             *transfer_request;
UINT                    status;
UCHAR                   *control_buffer;
UINT                    alternate_setting;
UX_CONFIGURATION        *configuration;
UX_INTERFACE            *interface_ptr;
UX_ENDPOINT             *endpoint;
ULONG                   endpoint_index;
UINT                    streaming_interface;
UINT                    max_payload_size;


    /* Protect thread reentry to this instance.  */
    status =  _ux_host_semaphore_get(&video -> ux_host_class_video_semaphore, UX_WAIT_FOREVER);

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &video -> ux_host_class_video_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Get the interface number of the video streaming interface.  */
    streaming_interface =  video -> ux_host_class_video_streaming_interface -> ux_interface_descriptor.bInterfaceNumber;

    /* Need to allocate memory for the control_buffer.  */
    control_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_VIDEO_PROBE_COMMIT_LENGTH);
    if (control_buffer == UX_NULL)
    {

        /* Unprotect thread reentry to this instance.  */
        _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore);

        /* Return error.  */        
        return(UX_MEMORY_INSUFFICIENT);
    }

    /* Check the result, did we find the alternate setting ? */
    if (status == UX_SUCCESS)
    {

        *(control_buffer + UX_HOST_CLASS_VIDEO_PROBE_COMMIT_FORMAT_INDEX) = (UCHAR)video_parameter -> ux_host_class_video_parameter_format_requested;
        *(control_buffer + UX_HOST_CLASS_VIDEO_PROBE_COMMIT_FRAME_INDEX) = (UCHAR)video_parameter -> ux_host_class_video_parameter_frame_requested;
        _ux_utility_long_put(control_buffer + UX_HOST_CLASS_VIDEO_PROBE_COMMIT_FRAME_INTERVAL,
                            video_parameter -> ux_host_class_video_parameter_frame_interval_requested);

        /* Create a transfer request for the GET_CUR_buffer request.  */
        transfer_request -> ux_transfer_request_data_pointer =      control_buffer;
        transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_VIDEO_PROBE_COMMIT_LENGTH;
        transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_VIDEO_SET_CUR;
        transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
        transfer_request -> ux_transfer_request_value =             UX_HOST_CLASS_VIDEO_VS_PROBE_CONTROL << 8;
        transfer_request -> ux_transfer_request_index =             streaming_interface;

        /* Send request to HCD layer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);
    
        /* Check for correct transfer. Buffer may not be all what we asked for.  */
        if (status == UX_SUCCESS)
        {
    
    
            /* Create a transfer request for the SET_CUR_buffer request.  */
            transfer_request -> ux_transfer_request_data_pointer =      control_buffer;
            transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_VIDEO_PROBE_COMMIT_LENGTH;
            transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_VIDEO_GET_CUR;
            transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
            transfer_request -> ux_transfer_request_value =             UX_HOST_CLASS_VIDEO_VS_PROBE_CONTROL << 8;
            transfer_request -> ux_transfer_request_index =             streaming_interface;
    
            /* Send request to HCD layer.  */
            status =  _ux_host_stack_transfer_request(transfer_request);
    
            /* Check for correct transfer.  */
            if (status == UX_SUCCESS)
            {
            
                /* We did the GET_CUR and SET_CUR for Probe Control. Now we can commit to the bandwidth.  */
                /* Create a transfer request for the SET_CUR_buffer request.  */
                transfer_request -> ux_transfer_request_data_pointer =      control_buffer;
                transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_VIDEO_PROBE_COMMIT_LENGTH;
                transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_VIDEO_SET_CUR;
                transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
                transfer_request -> ux_transfer_request_value =             UX_HOST_CLASS_VIDEO_VS_COMMIT_CONTROL << 8;
                transfer_request -> ux_transfer_request_index =             streaming_interface;
        
                /* Send request to HCD layer.  */
                status =  _ux_host_stack_transfer_request(transfer_request);
        
                /* Check for correct transfer.  */
                if (status == UX_SUCCESS)
                {
        
                    /* Check if user has request specific bandwidth selection.  */
                    if (video_parameter -> ux_host_class_video_parameter_channel_bandwidth_selection == 0)
                    {

                        /* Get the max payload transfer size returned from video device.  */
                        max_payload_size = _ux_utility_long_get(control_buffer + UX_HOST_CLASS_VIDEO_PROBE_COMMIT_MAX_PAYLOAD_TRANSFER_SIZE);
                    }
                    else
                    {

                        /* Set the max payload transfer size to the user requested one.  */
                        max_payload_size = video_parameter -> ux_host_class_video_parameter_channel_bandwidth_selection;
                    }

                    /* Search for the non zero alternate setting of the video stream.  */
                    status =  _ux_host_class_video_alternate_setting_locate(video, max_payload_size, &alternate_setting);

                    if (status == UX_SUCCESS)
                    {

                        /* Now the Commit has been done, the alternate setting can be requested.  */
                        /* We found the alternate setting for the sampling values demanded, now we need 
                            to search its container.  */
                        configuration =        video -> ux_host_class_video_streaming_interface -> ux_interface_configuration;
                        interface_ptr =        configuration -> ux_configuration_first_interface;  

                        /* Scan all interfaces.  */
                        while (interface_ptr != UX_NULL)     
                        {
                    
                            /* We search for both the right interface and alternate setting.  */
                            if ((interface_ptr -> ux_interface_descriptor.bInterfaceNumber == streaming_interface) &&
                                (interface_ptr -> ux_interface_descriptor.bAlternateSetting == alternate_setting))
                            {
                                
                                /* We have found the right interface/alternate setting combination 
                                The stack will select it for us.  */
                                status =  _ux_host_stack_interface_setting_select(interface_ptr);
                                
                                /* If the alternate setting for the streaming interface could be selected, we memorize it.  */
                                if (status == UX_SUCCESS)
                                {
                    
                                    /* Memorize the interface.  */
                                    video -> ux_host_class_video_streaming_interface =  interface_ptr;
                    
                                    /* We need to research the isoch endpoint now.  */
                                    for (endpoint_index = 0; endpoint_index < interface_ptr -> ux_interface_descriptor.bNumEndpoints; endpoint_index++)
                                    {                        
                    
                                        /* Get the list of endpoints one by one.  */
                                        status =  _ux_host_stack_interface_endpoint_get(video -> ux_host_class_video_streaming_interface,
                                                                                                        endpoint_index, &endpoint);
                    
                                        /* Check completion status.  */
                                        if (status == UX_SUCCESS)
                                        {
                    
                                            /* Check if endpoint is ISOCH, regardless of the direction.  */
                                            if ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_ISOCHRONOUS_ENDPOINT)
                                            {
                    
                                                /* We have found the isoch endpoint, save it.  */
                                                video -> ux_host_class_video_isochronous_endpoint =  endpoint;
                    
                                                /* Save the max payload size.
                                                It's not exceeding endpoint bandwidth since the interface alternate
                                                setting is located by max payload size.  */
                                                video -> ux_host_class_video_current_max_payload_size = max_payload_size;
                        
                                                /* Free all used resources.  */
                                                _ux_utility_memory_free(control_buffer);

                                                /* Unprotect thread reentry to this instance.  */
                                                _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore);
                    
                                                /* Return successful completion.  */
                                                return(UX_SUCCESS);             
                                            }
                                        }                
                                    }            
                                }
                            }
                    
                            /* Move to next interface.  */
                            interface_ptr =  interface_ptr -> ux_interface_next_interface;
                        }
                    }
                }
            }
        }
    }
    /* Free all used resources.  */
    _ux_utility_memory_free(control_buffer);

    /* Unprotect thread reentry to this instance.  */
    _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore);

    /* Return completion status.  */
    return(status);
}

