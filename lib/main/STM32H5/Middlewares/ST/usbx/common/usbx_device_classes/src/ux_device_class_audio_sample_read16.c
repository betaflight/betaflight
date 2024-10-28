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
/*    _ux_device_class_audio_sample_read16                PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function reads 16-bit sample value from the Audio class.       */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    stream                                Address of audio stream       */
/*                                            instance                    */
/*    buffer                                Pointer to buffer to save     */
/*                                            sample data                 */
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
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_audio_sample_read16(UX_DEVICE_CLASS_AUDIO_STREAM *stream,
                                          USHORT *buffer)
{

UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_DEVICE             *device;
UCHAR                       *sample_ptr;
UCHAR                       *next_frame_buffer;
ULONG                       next_frame_sample;


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

    /* Check if endpoint direction is OK.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_OUT)
        return(UX_ERROR);

    /* Underflow!!  */
    if (stream -> ux_device_class_audio_stream_access_pos -> ux_device_class_audio_frame_length == 0)
    {
        return(UX_BUFFER_OVERFLOW);
    }

    /* Try to read a sample.  */
    sample_ptr = stream -> ux_device_class_audio_stream_access_pos -> ux_device_class_audio_frame_data +
                    stream -> ux_device_class_audio_stream_access_pos -> ux_device_class_audio_frame_pos;
    if (buffer)
        *buffer = *(USHORT *)(sample_ptr);

    /* Update sample read state.  */
    next_frame_sample = stream -> ux_device_class_audio_stream_access_pos -> ux_device_class_audio_frame_pos + 2;
    if (next_frame_sample >= stream -> ux_device_class_audio_stream_access_pos -> ux_device_class_audio_frame_length)
    {

        /* Set frame length to 0 to indicate no data.  */
        stream -> ux_device_class_audio_stream_access_pos -> ux_device_class_audio_frame_length = 0;

        /* Move to next frame buffer.  */
        next_frame_sample = 0;

        /* Move frame if it's not the last one.  */
        if (stream -> ux_device_class_audio_stream_access_pos != stream -> ux_device_class_audio_stream_transfer_pos)
        {
            next_frame_buffer = (UCHAR *)stream -> ux_device_class_audio_stream_access_pos;
            next_frame_buffer += stream -> ux_device_class_audio_stream_frame_buffer_size;
            if (next_frame_buffer >= stream -> ux_device_class_audio_stream_buffer + stream -> ux_device_class_audio_stream_buffer_size)
                next_frame_buffer = stream -> ux_device_class_audio_stream_buffer;
            stream -> ux_device_class_audio_stream_access_pos = (UX_DEVICE_CLASS_AUDIO_FRAME *)next_frame_buffer;
        }
    }

    /* Update next sample position.  */
    stream -> ux_device_class_audio_stream_access_pos -> ux_device_class_audio_frame_pos = next_frame_sample;

    return(UX_SUCCESS);
}

