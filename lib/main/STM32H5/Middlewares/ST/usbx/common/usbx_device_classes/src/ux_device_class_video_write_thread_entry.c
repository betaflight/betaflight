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
/*    _ux_device_class_video_write_thread_entry           PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is thread of ISO IN for the Video class.              */
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
VOID _ux_device_class_video_write_thread_entry(ULONG video_stream)
{

UINT                            status;
UX_DEVICE_CLASS_VIDEO_STREAM    *stream;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_TRANSFER               *transfer;
UCHAR                           *next_pos;
UX_DEVICE_CLASS_VIDEO_PAYLOAD   *next_payload;
ULONG                           transfer_length;
ULONG                           actual_length;


    /* Get Video class stream instance.  */
    UX_THREAD_EXTENSION_PTR_GET(stream, UX_DEVICE_CLASS_VIDEO_STREAM, video_stream)

    /* Get stack device instance.  */
    device = stream -> ux_device_class_video_stream_video -> ux_device_class_video_device;

    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        {

            /* Get endpoint instance.  */
            endpoint = stream -> ux_device_class_video_stream_endpoint;

            /* Endpoint not available, maybe it's alternate setting 0.  */
            if (endpoint == UX_NULL)
                break;

            /* Get transfer instance.  */
            transfer = &endpoint -> ux_slave_endpoint_transfer_request;

            /* Start payload transfer anyway (even ZLP).  */
            transfer_length = stream -> ux_device_class_video_stream_transfer_pos -> ux_device_class_video_payload_length;
            if (transfer_length)
                _ux_utility_memory_copy(transfer -> ux_slave_transfer_request_data_pointer,
                    stream -> ux_device_class_video_stream_transfer_pos -> ux_device_class_video_payload_data, transfer_length); /* Use case of memcpy is verified. */

            /* Issue transfer request, thread blocked until transfer done.  */
            status = _ux_device_stack_transfer_request(transfer, transfer_length, transfer_length);

            /* Check error.  */
            if (status != UX_SUCCESS)
            {

                /* Error notification!  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_ERROR);
                break;
            }

            /* Frame sent, free it.  */
            stream -> ux_device_class_video_stream_transfer_pos -> ux_device_class_video_payload_length = 0;

            /* Get actual transfer length.  */
            actual_length = transfer -> ux_slave_transfer_request_actual_length;

            /* Calculate next position.  */
            next_pos = (UCHAR *)stream -> ux_device_class_video_stream_transfer_pos;
            next_pos += stream -> ux_device_class_video_stream_payload_buffer_size;
            if (next_pos >= stream -> ux_device_class_video_stream_buffer + stream -> ux_device_class_video_stream_buffer_size)
                next_pos = stream -> ux_device_class_video_stream_buffer;
            next_payload = (UX_DEVICE_CLASS_VIDEO_PAYLOAD *)next_pos;

            /* Underflow check!  */
            if (transfer_length)
            {

                /* Advance position.  */
                stream -> ux_device_class_video_stream_transfer_pos = next_payload;

                /* Error trap!  */
                if (next_payload -> ux_device_class_video_payload_length == 0)
                {
                    stream -> ux_device_class_video_stream_buffer_error_count ++;
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_BUFFER_OVERFLOW);
                }
            }
            else
            {

                /* Advance position if next payload available.  */
                if (next_payload -> ux_device_class_video_payload_length)
                    stream -> ux_device_class_video_stream_transfer_pos = next_payload;
                else
                {

                    /* Error trap!  */
                    stream -> ux_device_class_video_stream_buffer_error_count ++;
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_BUFFER_OVERFLOW);
                }
            }

            /* Invoke notification callback.  */
            if (stream -> ux_device_class_video_stream_callbacks.ux_device_class_video_stream_payload_done != UX_NULL)
                stream -> ux_device_class_video_stream_callbacks.ux_device_class_video_stream_payload_done(stream, actual_length);
        }

        /* We need to suspend ourselves. We will be resumed by the device enumeration module or when a change of alternate setting happens.  */
        _ux_utility_thread_suspend(&stream -> ux_device_class_video_stream_thread);
    }
}
#endif
