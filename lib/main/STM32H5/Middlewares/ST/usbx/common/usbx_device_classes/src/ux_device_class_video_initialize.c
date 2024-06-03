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
/*    _ux_device_class_video_initialize                   PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB Video device.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to video command      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_utility_thread_create             Create thread to use          */
/*    _ux_utility_thread_delete             Delete thread                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Video Class                                                  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_video_initialize(UX_SLAVE_CLASS_COMMAND *command)
{

UINT                                    status = UX_SUCCESS;
UX_DEVICE_CLASS_VIDEO                   *video;
UX_DEVICE_CLASS_VIDEO_PARAMETER         *video_parameter;
UX_DEVICE_CLASS_VIDEO_STREAM            *stream;
UX_DEVICE_CLASS_VIDEO_STREAM_PARAMETER  *stream_parameter;
UX_SLAVE_CLASS                          *class_inst;
ULONG                                   memory_size;
ULONG                                   streams_size;
ULONG                                   i;


    /* Get the class container.  */
    class_inst =  command -> ux_slave_class_command_class_ptr;

    /* Get the pointer to the application parameters for the video class.  */
    video_parameter = (UX_DEVICE_CLASS_VIDEO_PARAMETER *)command -> ux_slave_class_command_parameter;

    /* Create an instance of the device video class, with additional streams and controls instances.  */
    memory_size  = sizeof(UX_DEVICE_CLASS_VIDEO);

    /* Put 0 to default result.  */
    streams_size = 0;

    /* Confirm there is no overflow on multiply.  */
    UX_UTILITY_MULC_SAFE(video_parameter -> ux_device_class_video_parameter_streams_nb, (ULONG)sizeof(UX_DEVICE_CLASS_VIDEO_STREAM), streams_size, status);
    if (status != UX_SUCCESS)
        return(status);

    /* Confirm there is no overflow on add.  */
    UX_UTILITY_ADD_SAFE(memory_size, streams_size, memory_size, status);
    if (status != UX_SUCCESS)
        return(status);

    /* Create buffer for video and controls.  */
    video = (UX_DEVICE_CLASS_VIDEO *)_ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);

    /* Check for successful allocation.  */
    if (video == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Save streams.  */
    if (streams_size)
    {
        video -> ux_device_class_video_streams = (UX_DEVICE_CLASS_VIDEO_STREAM *)((UCHAR *)video + sizeof(UX_DEVICE_CLASS_VIDEO));
        video -> ux_device_class_video_streams_nb = video_parameter -> ux_device_class_video_parameter_streams_nb;
    }

    /* Allocate resources for streams.  */
    stream = video -> ux_device_class_video_streams;
    stream_parameter = video_parameter -> ux_device_class_video_parameter_streams;
    for (i = 0; i < video -> ux_device_class_video_streams_nb; i ++)
    {

        /* Create memory block based on max payload buffer size and max number of payloads buffered.
           Each payload require some additional header memory (8 bytes).  */
        stream -> ux_device_class_video_stream_payload_buffer_size = stream_parameter -> ux_device_class_video_stream_parameter_max_payload_buffer_size;

        if (UX_OVERFLOW_CHECK_ADD_USHORT(stream -> ux_device_class_video_stream_payload_buffer_size, 4))
        {
            status = UX_ERROR;
            break;
        }
        stream -> ux_device_class_video_stream_payload_buffer_size += 4;

        if (UX_OVERFLOW_CHECK_MULV_ULONG(stream -> ux_device_class_video_stream_payload_buffer_size,
            stream_parameter -> ux_device_class_video_stream_parameter_max_payload_buffer_nb))
        {
            status = UX_ERROR;
            break;
        }
        memory_size = stream -> ux_device_class_video_stream_payload_buffer_size *
                            stream_parameter -> ux_device_class_video_stream_parameter_max_payload_buffer_nb;

        /* Create block of buffer buffer is cache safe for USB transfer.  */
        stream -> ux_device_class_video_stream_buffer = (UCHAR *)_ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);

        /* Check for successful allocation.  */
        if (stream -> ux_device_class_video_stream_buffer == UX_NULL)
        {
            status = UX_MEMORY_INSUFFICIENT;
            break;
        }

        stream -> ux_device_class_video_stream_buffer_size = memory_size;
        stream -> ux_device_class_video_stream_transfer_pos = (UX_DEVICE_CLASS_VIDEO_PAYLOAD *)stream -> ux_device_class_video_stream_buffer;
        stream -> ux_device_class_video_stream_access_pos = stream -> ux_device_class_video_stream_transfer_pos;


#if !defined(UX_DEVICE_STANDALONE)

        /* Create memory block for streaming thread stack in addition.  */
        if (stream_parameter -> ux_device_class_video_stream_parameter_thread_stack_size == 0)
            memory_size = UX_DEVICE_CLASS_VIDEO_THREAD_STACK_SIZE;
        else
            memory_size = stream_parameter -> ux_device_class_video_stream_parameter_thread_stack_size;
        stream -> ux_device_class_video_stream_thread_stack = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);

        /* Check for successful allocation.  */
        if (stream -> ux_device_class_video_stream_thread_stack == UX_NULL)
        {
            status = UX_MEMORY_INSUFFICIENT;
            break;
        }

        /* Create streaming thread.  */
        status =  _ux_utility_thread_create(&stream -> ux_device_class_video_stream_thread , "ux_device_class_video_stream_thread",
                    stream_parameter -> ux_device_class_video_stream_parameter_thread_entry,
                    (ULONG)(ALIGN_TYPE)stream, (VOID *) stream -> ux_device_class_video_stream_thread_stack,
                    memory_size, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);

        /* Check for successful allocation.  */
        if (status != UX_SUCCESS)
            break;

        UX_THREAD_EXTENSION_PTR_SET(&(stream -> ux_device_class_video_stream_thread), stream)
#else

        /* Save task function for streaming.  */
        stream -> ux_device_class_video_stream_task_function = stream_parameter -> ux_device_class_video_stream_parameter_task_function;
#endif

        /* Save callbacks.  */
        _ux_utility_memory_copy(&stream -> ux_device_class_video_stream_callbacks,
                                &stream_parameter -> ux_device_class_video_stream_parameter_callbacks,
                                sizeof(UX_DEVICE_CLASS_VIDEO_STREAM_CALLBACKS)); /* Use case of memcpy is verified. */

        /* Save video instance.  */
        stream -> ux_device_class_video_stream_video = video;

        stream ++;
        stream_parameter ++;
    }

    /* Check for successful creation.  */
    if (status == UX_SUCCESS)
    {

        /* Save the address of the Video instance inside the Video container.  */
        class_inst -> ux_slave_class_instance = (VOID *) video;

        /* Link to class instance.  */
        video -> ux_device_class_video_class = class_inst;

        /* Save callbacks.  */
        _ux_utility_memory_copy(&video -> ux_device_class_video_callbacks,
            &video_parameter -> ux_device_class_video_parameter_callbacks,
            sizeof(UX_DEVICE_CLASS_VIDEO_CALLBACKS)); /* Use case of memcpy is verified. */

#if defined(UX_DEVICE_STANDALONE)

        /* Link task function.  */
        class_inst -> ux_slave_class_task_function = _ux_device_class_video_tasks_run;
#endif

        /* Return completion status.  */
        return(UX_SUCCESS);
    }

    /* Error trap!  */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

    /* Free allocated resources.  */
    stream = video -> ux_device_class_video_streams;
    for (i = 0; i < video -> ux_device_class_video_streams_nb; i ++)
    {

#if !defined(UX_DEVICE_STANDALONE)
        if (stream -> ux_device_class_video_stream_thread.tx_thread_id)
            _ux_utility_thread_delete(&stream -> ux_device_class_video_stream_thread);
        if (stream -> ux_device_class_video_stream_thread_stack)
            _ux_utility_memory_free(stream -> ux_device_class_video_stream_thread_stack);
#endif

        if (stream -> ux_device_class_video_stream_buffer)
            _ux_utility_memory_free(stream -> ux_device_class_video_stream_buffer);
        stream ++;
    }
    _ux_utility_memory_free(video);

    return(status);
}
