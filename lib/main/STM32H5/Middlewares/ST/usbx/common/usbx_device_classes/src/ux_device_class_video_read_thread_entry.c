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


#if !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_video_read_thread_entry            PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is thread of ISO OUT from the Video class.            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    video_stream                          Address of video stream       */
/*                                            instance                    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_system_error_handler              System error trap             */
/*    _ux_utility_thread_suspend            Suspend thread used           */
/*    _ux_device_stack_transfer_request     Issue transfer request        */
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
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added error statistics,     */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
VOID _ux_device_class_video_read_thread_entry(ULONG video_stream)
{

UINT                            status;
UX_DEVICE_CLASS_VIDEO_STREAM    *stream;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_TRANSFER               *transfer;
UCHAR                           *next_pos;
UX_DEVICE_CLASS_VIDEO_PAYLOAD   *next_payload;
ULONG                           max_packet_size;
ULONG                           actual_length;


    /* Get Video class instance.  */
    UX_THREAD_EXTENSION_PTR_GET(stream, UX_DEVICE_CLASS_VIDEO_STREAM, video_stream)

    /* Get stack device instance.  */
    device = stream -> ux_device_class_video_stream_video -> ux_device_class_video_device;

    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {
        max_packet_size = 0;
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        {

            /* Get endpoint instance.  */
            endpoint = stream -> ux_device_class_video_stream_endpoint;

            /* Endpoint not available, maybe it's alternate setting 0.  */
            if (endpoint == UX_NULL)
                break;

            /* Calculate transfer size based on packet size and number transactions once endpoint is available.  */
            if (max_packet_size == 0)
                max_packet_size = endpoint -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_transfer_length;

            /* Get transfer instance.  */
            transfer = &endpoint -> ux_slave_endpoint_transfer_request;

            /* Start payload transfer anyway.  */
            status = _ux_device_stack_transfer_request(transfer, max_packet_size, max_packet_size);

            /* Check error.  */
            if (status != UX_SUCCESS)
            {

                /* Error notification!  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_ERROR);
                break;
            }

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

        /* We need to suspend ourselves. We will be resumed by the device enumeration module or when a change of alternate setting happens.  */
        _ux_utility_thread_suspend(&stream -> ux_device_class_video_stream_thread);
    }
}
#endif
