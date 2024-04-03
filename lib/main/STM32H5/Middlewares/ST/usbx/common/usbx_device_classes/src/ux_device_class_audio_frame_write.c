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
/**   Device Audio Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_audio.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio_frame_write                  PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function writes frame to the Audio class.                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    stream                                Address of audio stream       */
/*                                            instance                    */
/*    frame                                 Pointer to buffer to save     */
/*                                            frame data                  */
/*    length                                Frame length in bytes         */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_copy               Copy data                     */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX                                                             */
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed frame length check,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_audio_frame_write(UX_DEVICE_CLASS_AUDIO_STREAM *stream, UCHAR *frame, ULONG length)
{

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UCHAR                       *next_frame_buffer;
ULONG                       frame_buffer_size;


    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* As long as the device is in the CONFIGURED state.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {

        /* Cannot proceed with command, the interface is down.  */
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }

    /* Check if endpoint is available.  */
    endpoint = stream -> ux_device_class_audio_stream_endpoint;
    if (endpoint == UX_NULL)
        return(UX_ERROR);

    /* Check if endpoint direction is OK (IN).  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT)
        return(UX_ERROR);

    /* Check frame length.  */
    frame_buffer_size = stream -> ux_device_class_audio_stream_frame_buffer_size;
    if ((frame_buffer_size - 8) < length)
        return(UX_ERROR);

    /* Check overflow!!  */
    if (stream -> ux_device_class_audio_stream_access_pos == stream -> ux_device_class_audio_stream_transfer_pos &&
        stream -> ux_device_class_audio_stream_access_pos -> ux_device_class_audio_frame_length != 0)
        return(UX_BUFFER_OVERFLOW);

    /* Calculate next frame buffer.  */
    next_frame_buffer = (UCHAR *)stream -> ux_device_class_audio_stream_access_pos;
    next_frame_buffer += frame_buffer_size;
    if (next_frame_buffer >= stream -> ux_device_class_audio_stream_buffer + stream -> ux_device_class_audio_stream_buffer_size)
        next_frame_buffer = stream -> ux_device_class_audio_stream_buffer;

    /* Copy frame.  */
    _ux_utility_memory_copy(stream -> ux_device_class_audio_stream_access_pos -> ux_device_class_audio_frame_data, frame, length); /* Use case of memcpy is verified. */
    stream -> ux_device_class_audio_stream_access_pos -> ux_device_class_audio_frame_length = length;

    /* Move frame position.  */
    stream -> ux_device_class_audio_stream_access_pos = (UX_DEVICE_CLASS_AUDIO_FRAME *)next_frame_buffer;

    return(UX_SUCCESS);
}

