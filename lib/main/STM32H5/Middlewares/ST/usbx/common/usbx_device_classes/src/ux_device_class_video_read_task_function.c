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


#if defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_video_read_task_function           PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the background task of the video stream read.      */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    stream                                Pointer to video stream       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    State machine status                                                */
/*    UX_STATE_EXIT                         Device not configured         */
/*    UX_STATE_IDLE                         No streaming transfer running */
/*    UX_STATE_WAIT                         Streaming transfer running    */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_run         Run transfer state machine    */
/*    _ux_utility_memory_copy               Copy memory                   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Device Stack                                                   */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  10-31-2022     Chaoqiong Xiao           Initial Version 6.2.0         */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_video_read_task_function(UX_DEVICE_CLASS_VIDEO_STREAM *stream)
{
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_TRANSFER               *transfer;
UCHAR                           *next_pos;
UX_DEVICE_CLASS_VIDEO_PAYLOAD   *next_payload;
ULONG                           max_packet_size;
ULONG                           actual_length;
UINT                            status;


    /* Get the pointer to the device.  */
    device = stream -> ux_device_class_video_stream_video -> ux_device_class_video_device;

    /* Check if the device is configured.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {
        stream -> ux_device_class_video_stream_task_state = UX_STATE_EXIT;
        return(UX_STATE_EXIT);
    }

    /* Get the endpoint.  */
    endpoint = stream -> ux_device_class_video_stream_endpoint;

    /* No endpoint ready, maybe it's alternate setting 0.  */
    if (endpoint == UX_NULL)
        return(UX_STATE_IDLE);

    /* Check if background transfer task is started.  */
    if (stream -> ux_device_class_video_stream_task_state == UX_DEVICE_CLASS_VIDEO_STREAM_RW_STOP)
        return(UX_STATE_IDLE);

    /* Get transfer instance.  */
    transfer = &endpoint -> ux_slave_endpoint_transfer_request;

    /* If not started yet, reset transfer and start polling.  */
    if (stream -> ux_device_class_video_stream_task_state == UX_DEVICE_CLASS_VIDEO_STREAM_RW_START)
    {

        /* Next state: transfer wait.  */
        stream -> ux_device_class_video_stream_task_state = UX_DEVICE_CLASS_VIDEO_STREAM_RW_WAIT;

        /* Reset transfer state.  */
        UX_SLAVE_TRANSFER_STATE_RESET(transfer);
    }

    /* Run transfer state machine.  */
    max_packet_size = endpoint -> ux_slave_endpoint_transfer_request.
                                ux_slave_transfer_request_transfer_length;
    status = _ux_device_stack_transfer_run(transfer, max_packet_size, max_packet_size);

    /* Error case.  */
    if (status < UX_STATE_NEXT)
    {

        /* Error on background transfer task start.  */
        stream -> ux_device_class_video_stream_task_state = UX_STATE_RESET;
        stream -> ux_device_class_video_stream_task_status =
                        transfer -> ux_slave_transfer_request_completion_code;

        /* Error notification!  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_ERROR);
        return(UX_STATE_EXIT);
    }

    /* Success case.  */
    if (status == UX_STATE_NEXT)
    {

        /* Next state: start.  */
        stream -> ux_device_class_video_stream_task_state = UX_DEVICE_CLASS_VIDEO_STREAM_RW_START;
        stream -> ux_device_class_video_stream_task_status =
                        transfer -> ux_slave_transfer_request_completion_code;

        /* Get actual transfer length.  */
        actual_length = transfer -> ux_slave_transfer_request_actual_length;

        /* Frame received, log it.  */
        stream -> ux_device_class_video_stream_transfer_pos -> ux_device_class_video_payload_length = actual_length;
        _ux_utility_memory_copy(stream -> ux_device_class_video_stream_transfer_pos -> ux_device_class_video_payload_data,
                        transfer -> ux_slave_transfer_request_data_pointer,
                        actual_length); /* Use case of memcpy is verified. */

        /* For simple, do not advance the transfer position if there is overflow.  */
        next_pos = (UCHAR *)stream -> ux_device_class_video_stream_transfer_pos;
        next_pos += stream -> ux_device_class_video_stream_payload_buffer_size;
        if (next_pos >= stream -> ux_device_class_video_stream_buffer + stream -> ux_device_class_video_stream_buffer_size)
            next_pos = stream -> ux_device_class_video_stream_buffer;
        next_payload = (UX_DEVICE_CLASS_VIDEO_PAYLOAD *)next_pos;

        /* Check overflow!  */
        if (next_payload -> ux_device_class_video_payload_length > 0)
        {

            /* Error notification!  */
            stream -> ux_device_class_video_stream_buffer_error_count ++;
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_BUFFER_OVERFLOW);
        }
        else

            /* Update transfer position.  */
            stream -> ux_device_class_video_stream_transfer_pos = next_payload;

        /* Invoke notification callback. */
        if (stream -> ux_device_class_video_stream_callbacks.ux_device_class_video_stream_payload_done != UX_NULL)
            stream -> ux_device_class_video_stream_callbacks.ux_device_class_video_stream_payload_done(stream, actual_length);
    }

    /* Keep waiting.  */
    return(UX_STATE_WAIT);
}
#endif
