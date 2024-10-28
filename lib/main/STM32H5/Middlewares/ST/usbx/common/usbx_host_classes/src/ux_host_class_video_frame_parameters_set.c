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
/*    _ux_host_class_video_frame_parameters_set           PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function sets the video parameters for the video device.       */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    video                                 Pointer to video class        */ 
/*    frame_format                          Video format to use           */
/*    width                                 Video frame width             */
/*    height                                Video frame height            */
/*    frame_interval                        Video frame interval          */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_video_format_data_get  Get format data               */
/*    _ux_host_class_video_frame_data_get   Get frame data                */
/*    _ux_host_stack_transfer_request       Process transfer request      */
/*    _ux_utility_descriptor_parse          Parse descriptor              */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_free               Release memory block          */
/*    _ux_host_semaphore_get                Get semaphore                 */
/*    _ux_host_semaphore_put                Put semaphore                 */
/*    _ux_utility_long_get                  Read 32-bit value             */
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
/*                                            validated max payload size, */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_frame_parameters_set(UX_HOST_CLASS_VIDEO *video, ULONG frame_format, ULONG width, ULONG height, ULONG frame_interval)
{

UX_HOST_CLASS_VIDEO_PARAMETER_FORMAT_DATA       format_parameter;
UX_HOST_CLASS_VIDEO_PARAMETER_FRAME_DATA        frame_parameter;
UX_HOST_CLASS_VIDEO_FRAME_DESCRIPTOR            frame_descriptor;
ULONG                                           format_index;
ULONG                                           frame_index;
UINT                                            status;
ULONG                                           min_frame_interval;
ULONG                                           max_frame_interval;
ULONG                                           frame_interval_step;
ULONG                                           i;
UX_ENDPOINT                                     *control_endpoint;
UX_TRANSFER                                     *transfer_request;
UINT                                            streaming_interface;
UCHAR                                           *control_buffer;
ULONG                                           max_payload_size;


    /* Find the requested frame format.  */
    for (format_index = 1; format_index <= video -> ux_host_class_video_number_formats; format_index++)
    {

        /* Get format data for current format index.  */
        format_parameter.ux_host_class_video_parameter_format_requested = format_index;
        status = _ux_host_class_video_format_data_get(video, &format_parameter);
        if (status != UX_SUCCESS)
        {
            return(status);
        }

        /* Check if the format type is the one requested.  */
        if (format_parameter.ux_host_class_video_parameter_format_subtype == frame_format)
        {

            /* Save number of frames in the video instance.  */
            video -> ux_host_class_video_number_frames = format_parameter.ux_host_class_video_parameter_number_frame_descriptors;

            /* The format is found and break the loop.  */
            break;
        }
    }

    /* Check if the requested format is found.  */
    if (format_index > video -> ux_host_class_video_number_formats)
    {
        return(UX_HOST_CLASS_VIDEO_PARAMETER_ERROR);
    }

    /* Find the requested frame resolution.  */
    for (frame_index = 1; frame_index <= video -> ux_host_class_video_number_frames; frame_index++)
    {

        /* Get frame data for current frame index.  */
        frame_parameter.ux_host_class_video_parameter_frame_requested = frame_index;
        status = _ux_host_class_video_frame_data_get(video, &frame_parameter);
        if (status != UX_SUCCESS)
        {
            return(status);
        }

        /* Check the frame resolution.  */
        if (frame_parameter.ux_host_class_video_parameter_frame_width == width &&
            frame_parameter.ux_host_class_video_parameter_frame_height == height)
        {

            /* Save the current frame index.  */
            video -> ux_host_class_video_current_frame = frame_index;

            /* The requested resolution is found, break the loop.  */
            break;
        }
    }

    /* Check if the requested resolution is found.  */
    if (frame_index > video -> ux_host_class_video_number_frames)
    {
        return(UX_HOST_CLASS_VIDEO_PARAMETER_ERROR);
    }

    /* Make the descriptor machine independent.  */
    _ux_utility_descriptor_parse(video -> ux_host_class_video_current_frame_address,
                                 _ux_system_class_video_frame_descriptor_structure,
                                 UX_HOST_CLASS_VIDEO_FRAME_DESCRIPTOR_ENTRIES, (UCHAR *) &frame_descriptor);

    /* Initial status for frame interval checking.  */
    status = UX_HOST_CLASS_VIDEO_PARAMETER_ERROR;

    /* Check the frame interval type.  */
    if (frame_descriptor.bFrameIntervalType == 0)
    {

        /* Frame interval type is continuous.  */
        min_frame_interval = _ux_utility_long_get(video -> ux_host_class_video_current_frame_address + 26);
        max_frame_interval = _ux_utility_long_get(video -> ux_host_class_video_current_frame_address + 30);
        frame_interval_step = _ux_utility_long_get(video -> ux_host_class_video_current_frame_address + 34);

        /* Check if the frame interval is valid.  */
        if (frame_interval >= min_frame_interval && frame_interval <= max_frame_interval &&
           ((frame_interval_step == 0) ||
            (frame_interval - min_frame_interval) % frame_interval_step == 0))
        {

            /* Save the frame interval.  */
            video -> ux_host_class_video_current_frame_interval = frame_interval;
            status = UX_SUCCESS;
        }
    }
    else
    {

        /* Frame interval type is discrete.  */
        for(i = 0; i < frame_descriptor.bFrameIntervalType; i++)
        {

            /* Check if the requested frame interval is valid.  */
            if (frame_interval == _ux_utility_long_get(video -> ux_host_class_video_current_frame_address + 26 + i * sizeof(ULONG)))
            {

                /* Save the frame interval.  */
                video -> ux_host_class_video_current_frame_interval = frame_interval;
                status = UX_SUCCESS;
            }
        }
    }

    if (status != UX_SUCCESS)
        return(UX_HOST_CLASS_VIDEO_PARAMETER_ERROR);

    /* Protect thread reentry to this instance.  */
    status =  _ux_host_semaphore_get(&video -> ux_host_class_video_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)
        return(status);

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

    *(control_buffer + UX_HOST_CLASS_VIDEO_PROBE_COMMIT_FORMAT_INDEX) = (UCHAR)format_index;
    *(control_buffer + UX_HOST_CLASS_VIDEO_PROBE_COMMIT_FRAME_INDEX) = (UCHAR)frame_index;
    _ux_utility_long_put(control_buffer + UX_HOST_CLASS_VIDEO_PROBE_COMMIT_FRAME_INTERVAL, frame_interval);

    /* Create a transfer request for the SET_CUR buffer request.  */
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


        /* Create a transfer request for the GET_CUR buffer request.  */
        transfer_request -> ux_transfer_request_data_pointer =      control_buffer;
        transfer_request -> ux_transfer_request_requested_length =  UX_HOST_CLASS_VIDEO_PROBE_COMMIT_LENGTH;
        transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_VIDEO_GET_CUR;
        transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
        transfer_request -> ux_transfer_request_value =             UX_HOST_CLASS_VIDEO_VS_PROBE_CONTROL << 8;
        transfer_request -> ux_transfer_request_index =             streaming_interface;

        /* Send request to HCD layer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

    }

    /* Get the max payload transfer size returned from video device.  */
    max_payload_size = _ux_utility_long_get(control_buffer + UX_HOST_CLASS_VIDEO_PROBE_COMMIT_MAX_PAYLOAD_TRANSFER_SIZE);

    /* Validate if the payload size is inside isochronouse packet payload.  */
    if (max_payload_size == 0)
        status = UX_HOST_CLASS_VIDEO_PARAMETER_ERROR;
    else
    {
        if (video -> ux_host_class_video_device -> ux_device_speed != UX_HIGH_SPEED_DEVICE)
        {
            if (max_payload_size > 1023)
                status = UX_HOST_CLASS_VIDEO_PARAMETER_ERROR;
        }
        else
        {
            if (max_payload_size > (1024 * 3))
                status = UX_HOST_CLASS_VIDEO_PARAMETER_ERROR;
        }
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(control_buffer);

    /* Unprotect thread reentry to this instance.  */
    _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore);

    /* Check the transfer status.  */
    if (status == UX_SUCCESS)
    {

        /* Save the maximum payload size returned by the device.  */
        video -> ux_host_class_video_current_max_payload_size = max_payload_size;
        return(UX_SUCCESS);
    }
    else
    {

        /* The probe process failed. Return error.  */
        return(UX_HOST_CLASS_VIDEO_PARAMETER_ERROR);
    }
}

