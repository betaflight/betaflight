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
/**   Device Video Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_video.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_video_write_payload_commit         PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function set payload buffer valid to send in the Video class.  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    stream                                Address of video stream       */
/*                                            instance                    */
/*    buffer                                Pointer to buffer to save     */
/*                                            payload data                */
/*    length                                Frame length in bytes         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX                                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_video_write_payload_commit(UX_DEVICE_CLASS_VIDEO_STREAM *stream, ULONG length)
{

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UCHAR                       *next_pos;


    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* As long as the device is in the CONFIGURED state.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

        /* Cannot proceed with command, the interface is down.  */
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }

    /* Check if endpoint is available.  */
    endpoint = stream -> ux_device_class_video_stream_endpoint;
    if (endpoint == UX_NULL)
        return(UX_ERROR);

    /* Check if endpoint direction is OK.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT)
        return(UX_ERROR);

    /* Check overflow!!  */
    if (stream -> ux_device_class_video_stream_access_pos == stream -> ux_device_class_video_stream_transfer_pos &&
        stream -> ux_device_class_video_stream_access_pos -> ux_device_class_video_payload_length != 0)
        return(UX_BUFFER_OVERFLOW);

    /* Check payload length.  */
    if ((stream -> ux_device_class_video_stream_payload_buffer_size - 4) < length)
        return(UX_ERROR);

    /* Calculate next payload buffer.  */
    next_pos = (UCHAR *)stream -> ux_device_class_video_stream_access_pos;
    next_pos += stream -> ux_device_class_video_stream_payload_buffer_size;
    if (next_pos >= stream -> ux_device_class_video_stream_buffer + stream -> ux_device_class_video_stream_buffer_size)
        next_pos = stream -> ux_device_class_video_stream_buffer;

    /* Commit payload length.  */
    stream -> ux_device_class_video_stream_access_pos -> ux_device_class_video_payload_length = length;

    /* Move payload position.  */
    stream -> ux_device_class_video_stream_access_pos = (UX_DEVICE_CLASS_VIDEO_PAYLOAD *)next_pos;

    return(UX_SUCCESS);
}

