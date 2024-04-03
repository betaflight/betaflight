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
/*    _ux_device_class_audio_change                       PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function changes the interface of the Audio device             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                           Pointer to audio command          */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_system_error_handler          System error trap                 */
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
/*                                            resulting in version 6.1    */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            replaced wMaxPacketSize by  */
/*                                            calculated payload size,    */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added feedback support,     */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            rx full packet for          */
/*                                            feedback,                   */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Yajun Xia                Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_audio_change(UX_SLAVE_CLASS_COMMAND *command)
{

UX_DEVICE_CLASS_AUDIO                   *audio;
UX_DEVICE_CLASS_AUDIO_STREAM            *stream;
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_INTERFACE                      *interface_ptr;
UX_SLAVE_ENDPOINT                       *endpoint;
UCHAR                                   *frame_buffer;
ULONG                                    stream_index;
ULONG                                    endpoint_dir;


    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    audio = (UX_DEVICE_CLASS_AUDIO *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;

    /* Get the interface number (base 0).  */
    if (audio -> ux_device_class_audio_interface)
    {

        /* If IAD used, calculate stream index based on interface number.  */
        stream_index  = interface_ptr -> ux_slave_interface_descriptor.bInterfaceNumber;
        stream_index -= audio -> ux_device_class_audio_interface -> ux_slave_interface_descriptor.bInterfaceNumber;
        stream_index --;
    }
    else

        /* One stream for one driver!  */
        stream_index  = 0;

    /* Get the stream instance.  */
    stream = &audio -> ux_device_class_audio_streams[stream_index];

    /* Update the interface.  */
    stream -> ux_device_class_audio_stream_interface = interface_ptr;

    /* If the interface to mount has a non zero alternate setting, the class is really active with
       the endpoints active.  If the interface reverts to alternate setting 0, it needs to have
       the pending transactions terminated.  */
    if (interface_ptr -> ux_slave_interface_descriptor.bAlternateSetting != 0)
    {

        /* Locate the endpoints.  ISO IN(write)/OUT(read) for Streaming Interface.  */
        endpoint = interface_ptr -> ux_slave_interface_first_endpoint;

        /* Parse all endpoints.  */
#if defined(UX_DEVICE_STANDALONE)

        endpoint_dir = (stream -> ux_device_class_audio_stream_task_function ==
                        _ux_device_class_audio_read_task_function) ?
                        UX_ENDPOINT_OUT: UX_ENDPOINT_IN;
#else

        endpoint_dir = (stream -> ux_device_class_audio_stream_thread.tx_thread_entry ==
                        _ux_device_class_audio_read_thread_entry) ?
                        UX_ENDPOINT_OUT : UX_ENDPOINT_IN;
#endif
        stream -> ux_device_class_audio_stream_endpoint = UX_NULL;

#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
        stream -> ux_device_class_audio_stream_feedback = UX_NULL;
#endif
        while(endpoint != UX_NULL)
        {

            /* Check the endpoint attributes.  */
            if((endpoint -> ux_slave_endpoint_descriptor.bmAttributes &
                UX_DEVICE_CLASS_AUDIO_EP_TRANSFER_TYPE_MASK) == UX_ISOCHRONOUS_ENDPOINT)
            {

                /* Check the endpoint direction.  */
                if ((endpoint->ux_slave_endpoint_descriptor.bEndpointAddress &
                     UX_ENDPOINT_DIRECTION) == endpoint_dir)
                {

                    /* We found the data endpoint, check its size.  */
                    if (endpoint -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_transfer_length > stream -> ux_device_class_audio_stream_frame_buffer_size - 8)
                    {

                        /* Error trap!  */
                        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

                        /* Frame buffer too small for endpoints.  */
                        return(UX_MEMORY_INSUFFICIENT);
                    }

                    /* Save it.  */
                    stream -> ux_device_class_audio_stream_endpoint = endpoint;
                }
#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
                else
                {

                    /* We found the feedback endpoint, check its size.  */
                    if (endpoint -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_transfer_length <
                        (_ux_system_slave->ux_system_slave_speed == UX_HIGH_SPEED_DEVICE ? 4 : 3))
                    {

                        /* Error trap!  */
                        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

                        /* Frame buffer too small for endpoints.  */
                        return(UX_MEMORY_INSUFFICIENT);
                    }

                    /* Set request length, uses full packet for OUT to avoid possible overflow.  */
                    endpoint -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_requested_length =
                        endpoint_dir == UX_ENDPOINT_OUT ?
                        endpoint -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_transfer_length :
                        ((_ux_system_slave -> ux_system_slave_speed == UX_HIGH_SPEED_DEVICE) ?
                         UX_FEEDBACK_SIZE_HIGH_SPEED : UX_FEEDBACK_SIZE_FULL_SPEED);

                    /* Save it.  */
                    stream -> ux_device_class_audio_stream_feedback = endpoint;
                }
#endif
            }

            /* Check if done.  */
            if (stream -> ux_device_class_audio_stream_endpoint
#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
                && stream -> ux_device_class_audio_stream_feedback
#endif
                )
                break;

            /* Next endpoint.  */
            endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
        }

        /* Now check if all endpoints have been found.  */
        if (stream -> ux_device_class_audio_stream_endpoint == UX_NULL)
        {

            /* Error trap!  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

            /* Not all endpoints have been found. Major error, do not proceed.  */
            return(UX_DESCRIPTOR_CORRUPTED);
        }

#if defined(UX_DEVICE_STANDALONE)

        /* Reset background transfer state.  */
        stream -> ux_device_class_audio_stream_task_state = UX_STATE_RESET;
#endif

        /* Now reset payload buffer error count.  */
        stream -> ux_device_class_audio_stream_buffer_error_count = 0;

        /* Now reset frame buffers.  */
        frame_buffer = stream -> ux_device_class_audio_stream_buffer;
        while(frame_buffer < stream -> ux_device_class_audio_stream_buffer + stream -> ux_device_class_audio_stream_buffer_size)
        {

            /* Reset header information.  */
            *((ULONG *) frame_buffer     ) = 0;
            *((ULONG *)(frame_buffer + 4)) = 0;

            /* Next.  */
            frame_buffer += stream -> ux_device_class_audio_stream_frame_buffer_size;
        }
        stream -> ux_device_class_audio_stream_transfer_pos = stream -> ux_device_class_audio_stream_access_pos;

#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT) && !defined(UX_DEVICE_STANDALONE)

        /* If feedback supported, resume the thread.  */
        if (stream -> ux_device_class_audio_stream_feedback_thread_stack)
            _ux_utility_thread_resume(&stream -> ux_device_class_audio_stream_feedback_thread);
#endif
    }
    else
    {

        /* There is no data endpoint.  */
        stream -> ux_device_class_audio_stream_endpoint = UX_NULL;
#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
        stream -> ux_device_class_audio_stream_feedback = UX_NULL;
#endif

        /* In this case, we are reverting to the Alternate Setting 0.  We need to terminate the pending transactions.  */
        /* Endpoints actually aborted and destroyed before change command.  */
        /*
        _ux_device_stack_transfer_all_request_abort(stream -> ux_device_class_audio_stream_endpoint, UX_TRANSFER_APPLICATION_RESET);
         */
    }

    /* Invoke stream change callback.  */
    if (stream -> ux_device_class_audio_stream_callbacks.ux_device_class_audio_stream_change)
        stream -> ux_device_class_audio_stream_callbacks.ux_device_class_audio_stream_change(stream, interface_ptr -> ux_slave_interface_descriptor.bAlternateSetting);

    /* Return completion status.  */
    return(UX_SUCCESS);
}
