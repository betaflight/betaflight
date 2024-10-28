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
/*    _ux_device_class_audio_initialize                   PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB Audio device.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to audio command      */
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
/*    _ux_device_thread_create              Create thread to use          */
/*    _ux_device_thread_delete              Delete thread                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Audio Class                                                  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            added feedback support,     */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added interrupt support,    */
/*                                            refined internal logic,     */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Yajun Xia                Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_audio_initialize(UX_SLAVE_CLASS_COMMAND *command)
{

UINT                                    status = UX_SUCCESS;
UX_DEVICE_CLASS_AUDIO                   *audio;
UX_DEVICE_CLASS_AUDIO_PARAMETER         *audio_parameter;
UX_DEVICE_CLASS_AUDIO_STREAM            *stream;
UX_DEVICE_CLASS_AUDIO_STREAM_PARAMETER  *stream_parameter;
UX_SLAVE_CLASS                          *audio_class;
ULONG                                   memory_size;
ULONG                                   streams_size;
ULONG                                   i;


    /* Get the class container.  */
    audio_class =  command -> ux_slave_class_command_class_ptr;

    /* Get the pointer to the application parameters for the audio class.  */
    audio_parameter = (UX_DEVICE_CLASS_AUDIO_PARAMETER *)command -> ux_slave_class_command_parameter;

    /* Create an instance of the device audio class, with additional streams and controls instances.  */
    memory_size  = sizeof(UX_DEVICE_CLASS_AUDIO);

    /* Put 0 to default result.  */
    streams_size = 0;

    /* Confirm there is no overflow on multiply.  */
    UX_UTILITY_MULC_SAFE(audio_parameter -> ux_device_class_audio_parameter_streams_nb, (ULONG)sizeof(UX_DEVICE_CLASS_AUDIO_STREAM), streams_size, status);
    if (status != UX_SUCCESS)
        return(status);

    /* Confirm there is no overflow on add.  */
    UX_UTILITY_ADD_SAFE(memory_size, streams_size, memory_size, status);
    if (status != UX_SUCCESS)
        return(status);

    /* Create buffer for audio and controls.  */
    audio = (UX_DEVICE_CLASS_AUDIO *)_ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);

    /* Check for successful allocation.  */
    if (audio == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

#if defined(UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT)

    /* Create resources for interrupt endpoint support.  */

    /* Get status data size.  */
    audio -> ux_device_class_audio_status_size =
                audio_parameter -> ux_device_class_audio_parameter_status_size;

    /* Calculate queue size in bytes.  */
    if (UX_OVERFLOW_CHECK_MULV_ULONG(
        audio_parameter -> ux_device_class_audio_parameter_status_queue_size,
        audio_parameter -> ux_device_class_audio_parameter_status_size))
    {
        _ux_utility_memory_free(audio);
        return(UX_MATH_OVERFLOW);
    }
    memory_size = audio_parameter -> ux_device_class_audio_parameter_status_queue_size *
                audio_parameter -> ux_device_class_audio_parameter_status_size;
    audio -> ux_device_class_audio_status_queue_bytes = memory_size;

#if !defined(UX_DEVICE_STANDALONE)
    if (UX_OVERFLOW_CHECK_ADD_ULONG(memory_size, UX_DEVICE_CLASS_AUDIO_INTERRUPT_THREAD_STACK_SIZE))
    {
        _ux_utility_memory_free(audio);
        return(UX_MATH_OVERFLOW);
    }
    memory_size += UX_DEVICE_CLASS_AUDIO_INTERRUPT_THREAD_STACK_SIZE;

    audio_class -> ux_slave_class_thread_stack = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);
    if (audio_class -> ux_slave_class_thread_stack == UX_NULL)
        status = UX_MEMORY_INSUFFICIENT;

    if (status == UX_SUCCESS)
    {
        status = _ux_device_thread_create(&audio_class -> ux_slave_class_thread,
                        "ux_device_class_audio_status_thread",
                        _ux_device_class_audio_interrupt_thread_entry, (ULONG)(ALIGN_TYPE)audio,
                        audio_class -> ux_slave_class_thread_stack,
                        UX_DEVICE_CLASS_AUDIO_INTERRUPT_THREAD_STACK_SIZE,
                        UX_THREAD_PRIORITY_CLASS, UX_THREAD_PRIORITY_CLASS,
                        UX_NO_TIME_SLICE, UX_DONT_START);
        if (status == UX_SUCCESS)
        {
            UX_THREAD_EXTENSION_PTR_SET(&(audio_class -> ux_slave_class_thread), audio)

            status = _ux_device_semaphore_create(&audio -> ux_device_class_audio_status_semaphore,
                                "ux_device_class_audio_status_semaphore", 0);
            if (status == UX_SUCCESS)
            {
                status = _ux_device_mutex_create(&audio -> ux_device_class_audio_status_mutex,
                                "ux_device_class_audio_status_mutex");
                if (status != UX_SUCCESS)
                    status = UX_MUTEX_ERROR;

                if (status != UX_SUCCESS)
                    _ux_device_semaphore_delete(&audio -> ux_device_class_audio_status_semaphore);
            }
            else
                status = UX_SEMAPHORE_ERROR;

            if (status != UX_SUCCESS)
                _ux_device_thread_delete(&audio_class -> ux_slave_class_thread);
        }
        else
            status = UX_THREAD_ERROR;

        if (status != UX_SUCCESS)
        {
            _ux_utility_memory_free(audio_class -> ux_slave_class_thread_stack);
            audio_class -> ux_slave_class_thread_stack = UX_NULL;
        }
    }

    if (status != UX_SUCCESS)
    {
        _ux_utility_memory_free(audio);
        return(status);
    }

    /* Status queue locates right after status stack.  */
    audio -> ux_device_class_audio_status_queue =
                        (UCHAR *)audio_class -> ux_slave_class_thread_stack +
                        UX_DEVICE_CLASS_AUDIO_INTERRUPT_THREAD_STACK_SIZE;
#else
    audio -> ux_device_class_audio_status_queue = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);

    /* Check for successful allocation.  */
    if (audio -> ux_device_class_audio_status_queue == UX_NULL)
    {
        _ux_utility_memory_free(audio);
        return(UX_MEMORY_INSUFFICIENT);
    }
#endif

#endif

    /* Save streams.  */
    if (streams_size)
    {
        audio -> ux_device_class_audio_streams = (UX_DEVICE_CLASS_AUDIO_STREAM *)((UCHAR *)audio + sizeof(UX_DEVICE_CLASS_AUDIO));
        audio -> ux_device_class_audio_streams_nb = audio_parameter -> ux_device_class_audio_parameter_streams_nb;
    }

    /* Allocate resources for streams.  */
    stream = audio -> ux_device_class_audio_streams;
    stream_parameter = audio_parameter -> ux_device_class_audio_parameter_streams;
    for (i = 0; i < audio -> ux_device_class_audio_streams_nb; i ++)
    {

        /* Create memory block based on max frame buffer size and max number of frames buffered.
           Each frame require some additional header memory (8 bytes).  */
        stream -> ux_device_class_audio_stream_frame_buffer_size = stream_parameter -> ux_device_class_audio_stream_parameter_max_frame_buffer_size;

        if (UX_OVERFLOW_CHECK_ADD_USHORT(stream -> ux_device_class_audio_stream_frame_buffer_size, 8))
        {
            status = UX_ERROR;
            break;
        }
        stream -> ux_device_class_audio_stream_frame_buffer_size += 8;

        if (UX_OVERFLOW_CHECK_MULV_ULONG(stream -> ux_device_class_audio_stream_frame_buffer_size,
            stream_parameter -> ux_device_class_audio_stream_parameter_max_frame_buffer_nb))
        {
            status = UX_ERROR;
            break;
        }
        memory_size = stream -> ux_device_class_audio_stream_frame_buffer_size *
                            stream_parameter -> ux_device_class_audio_stream_parameter_max_frame_buffer_nb;

        /* Create block of buffer buffer is cache safe for USB transfer.  */
        stream -> ux_device_class_audio_stream_buffer = (UCHAR *)_ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);

        /* Check for successful allocation.  */
        if (stream -> ux_device_class_audio_stream_buffer == UX_NULL)
        {
            status = UX_MEMORY_INSUFFICIENT;
            break;
        }

        stream -> ux_device_class_audio_stream_buffer_size = memory_size;
        stream -> ux_device_class_audio_stream_transfer_pos = (UX_DEVICE_CLASS_AUDIO_FRAME *)stream -> ux_device_class_audio_stream_buffer;
        stream -> ux_device_class_audio_stream_access_pos = stream -> ux_device_class_audio_stream_transfer_pos;

#if !defined(UX_DEVICE_STANDALONE)

        /* Create memory block for streaming thread stack in addition.  */
        if (stream_parameter -> ux_device_class_audio_stream_parameter_thread_stack_size == 0)
            memory_size = UX_DEVICE_CLASS_AUDIO_FEEDBACK_THREAD_STACK_SIZE;
        else
            memory_size = stream_parameter -> ux_device_class_audio_stream_parameter_thread_stack_size;
        stream -> ux_device_class_audio_stream_thread_stack = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);

        /* Check for successful allocation.  */
        if (stream -> ux_device_class_audio_stream_thread_stack == UX_NULL)
        {
            status = UX_MEMORY_INSUFFICIENT;
            break;
        }

        /* Create streaming thread.  */
        status =  _ux_device_thread_create(&stream -> ux_device_class_audio_stream_thread , "ux_device_class_audio_stream_thread",
                    stream_parameter -> ux_device_class_audio_stream_parameter_thread_entry,
                    (ULONG)(ALIGN_TYPE)stream, (VOID *) stream -> ux_device_class_audio_stream_thread_stack,
                    memory_size, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);

        /* Check for successful allocation.  */
        if (status != UX_SUCCESS)
        {
            _ux_utility_memory_free(stream -> ux_device_class_audio_stream_thread_stack);
            stream -> ux_device_class_audio_stream_thread_stack = UX_NULL;
            break;
        }

        UX_THREAD_EXTENSION_PTR_SET(&(stream -> ux_device_class_audio_stream_thread), stream)
#else

        /* Save task function for streaming.  */
        stream -> ux_device_class_audio_stream_task_function = stream_parameter -> ux_device_class_audio_stream_parameter_task_function;
#endif

#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)

#if !defined(UX_DEVICE_STANDALONE)

        /* Check entry to confirm feedback is supported.  */
        if (stream_parameter -> ux_device_class_audio_stream_parameter_feedback_thread_entry)
        {
            /* Create memory block for streaming thread stack in addition.  */
            if (stream_parameter -> ux_device_class_audio_stream_parameter_feedback_thread_stack_size == 0)
                memory_size = UX_THREAD_STACK_SIZE;
            else
                memory_size = stream_parameter -> ux_device_class_audio_stream_parameter_feedback_thread_stack_size;
            stream -> ux_device_class_audio_stream_feedback_thread_stack = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, memory_size);

            /* Check for successful allocation.  */
            if (stream -> ux_device_class_audio_stream_feedback_thread_stack == UX_NULL)
            {
                status = UX_MEMORY_INSUFFICIENT;
                break;
            }

            /* Create streaming thread.  */
            status =  _ux_utility_thread_create(&stream -> ux_device_class_audio_stream_feedback_thread , "ux_device_class_audio_stream_feedback_thread",
                        stream_parameter -> ux_device_class_audio_stream_parameter_feedback_thread_entry,
                        (ULONG)(ALIGN_TYPE)stream, (VOID *) stream -> ux_device_class_audio_stream_feedback_thread_stack,
                        memory_size, UX_THREAD_PRIORITY_CLASS,
                        UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);

            /* Check for successful allocation.  */
            if (status != UX_SUCCESS)
            {
                _ux_utility_memory_free(stream -> ux_device_class_audio_stream_feedback_thread_stack);
                stream -> ux_device_class_audio_stream_feedback_thread_stack = UX_NULL;
                break;
            }

            UX_THREAD_EXTENSION_PTR_SET(&(stream -> ux_device_class_audio_stream_feedback_thread), stream)
        }
#else
        if (stream_parameter -> ux_device_class_audio_stream_parameter_feedback_task_function)
        {

            /* Save task function for streaming.  */
            stream -> ux_device_class_audio_stream_feedback_task_function = stream_parameter -> ux_device_class_audio_stream_parameter_feedback_task_function;
        }
#endif
#endif

        /* Save callbacks.  */
        _ux_utility_memory_copy(&stream -> ux_device_class_audio_stream_callbacks,
                                &stream_parameter -> ux_device_class_audio_stream_parameter_callbacks,
                                sizeof(UX_DEVICE_CLASS_AUDIO_STREAM_CALLBACKS)); /* Use case of memcpy is verified. */

        /* Save audio instance.  */
        stream -> ux_device_class_audio_stream_audio = audio;

        stream ++;
        stream_parameter ++;
    }

    /* Check for successful creation.  */
    if (status == UX_SUCCESS)
    {

        /* Save the address of the Audio instance inside the Audio container.  */
        audio_class -> ux_slave_class_instance = (VOID *) audio;

        /* Link to class instance.  */
        audio -> ux_device_class_audio_class = audio_class;

        /* Save callbacks.  */
        _ux_utility_memory_copy(&audio -> ux_device_class_audio_callbacks,
            &audio_parameter -> ux_device_class_audio_parameter_callbacks,
            sizeof(UX_DEVICE_CLASS_AUDIO_CALLBACKS)); /* Use case of memcpy is verified. */

#if defined(UX_DEVICE_STANDALONE)

        /* Link task function.  */
        audio_class -> ux_slave_class_task_function = _ux_device_class_audio_tasks_run;
#endif

        /* Return completion status.  */
        return(UX_SUCCESS);
    }

    /* Error trap!  */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

    /* Free allocated resources.  */
    stream = audio -> ux_device_class_audio_streams;
    for (i = 0; i < audio -> ux_device_class_audio_streams_nb; i ++)
    {
#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
#if !defined(UX_DEVICE_STANDALONE)
        if (stream -> ux_device_class_audio_stream_feedback_thread_stack)
        {
            _ux_device_thread_delete(&stream -> ux_device_class_audio_stream_feedback_thread);
            _ux_utility_memory_free(stream -> ux_device_class_audio_stream_feedback_thread_stack);
        }
#endif
#endif
#if !defined(UX_DEVICE_STANDALONE)
        if (stream -> ux_device_class_audio_stream_thread_stack)
        {
            _ux_device_thread_delete(&stream -> ux_device_class_audio_stream_thread);
            _ux_utility_memory_free(stream -> ux_device_class_audio_stream_thread_stack);
        }
#endif
        if (stream -> ux_device_class_audio_stream_buffer)
            _ux_utility_memory_free(stream -> ux_device_class_audio_stream_buffer);
        stream ++;
    }
#if defined(UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT)
#if !defined(UX_DEVICE_STANDALONE)
    if (audio_class -> ux_slave_class_thread_stack)
    {
        _ux_device_thread_delete(&audio_class -> ux_slave_class_thread);
        _ux_utility_memory_free(audio_class -> ux_slave_class_thread_stack);

        _ux_device_semaphore_delete(&audio -> ux_device_class_audio_status_semaphore);
        _ux_device_mutex_delete(&audio -> ux_device_class_audio_status_mutex);
    }
#else
    if (audio -> ux_device_class_audio_status_queue)
    {
        _ux_utility_memory_free(audio -> ux_device_class_audio_status_queue);
    }
#endif
#endif
    _ux_utility_memory_free(audio);

    return(status);
}
