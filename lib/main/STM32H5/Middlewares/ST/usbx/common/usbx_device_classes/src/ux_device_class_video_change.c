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
/*    _ux_device_class_video_change                       PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function changes the interface of the Video device             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                           Pointer to video command          */
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
/*    Device Video Class                                                  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_video_change(UX_SLAVE_CLASS_COMMAND *command)
{

UX_DEVICE_CLASS_VIDEO                   *video;
UX_DEVICE_CLASS_VIDEO_STREAM            *stream;
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_INTERFACE                      *interface_ptr;
UX_SLAVE_ENDPOINT                       *endpoint;
UCHAR                                   *payload_buffer;
ULONG                                    stream_index;


    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    video = (UX_DEVICE_CLASS_VIDEO *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr = (UX_SLAVE_INTERFACE *) command -> ux_slave_class_command_interface;

    /* Get the interface number (base 0).  */
    if (video -> ux_device_class_video_interface)
    {

        /* If IAD used, calculate stream index based on interface number.  */
        stream_index  = interface_ptr -> ux_slave_interface_descriptor.bInterfaceNumber;
        stream_index -= video -> ux_device_class_video_interface -> ux_slave_interface_descriptor.bInterfaceNumber;
        stream_index --;
    }
    else

        /* One stream for one driver!  */
        stream_index  = 0;

    /* Get the stream instance.  */
    stream = &video -> ux_device_class_video_streams[stream_index];

    /* Update the interface.  */
    stream -> ux_device_class_video_stream_interface = interface_ptr;

    /* If the interface to mount has a non zero alternate setting, the class is really active with
       the endpoints active.  If the interface reverts to alternate setting 0, it needs to have
       the pending transactions terminated.  */
    if (interface_ptr -> ux_slave_interface_descriptor.bAlternateSetting != 0)
    {

        /* Locate the endpoints.  ISO IN/OUT for Streaming Interface.  */
        endpoint = interface_ptr -> ux_slave_interface_first_endpoint;

        /* Parse all endpoints.  */
        stream -> ux_device_class_video_stream_endpoint = UX_NULL;
        while (endpoint != UX_NULL)
        {

            /* Check the endpoint.  */
            if ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_ISOCHRONOUS_ENDPOINT)
            {

                /* We found the endpoint, check its size.  */
                if (endpoint -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_transfer_length >
                    (stream -> ux_device_class_video_stream_payload_buffer_size - 4))
                {

                    /* Error trap!  */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

                    /* payload buffer too small for endpoints.  */
                    return(UX_MEMORY_INSUFFICIENT);
                }

                /* Save it.  */
                stream -> ux_device_class_video_stream_endpoint = endpoint;
                break;
            }

            /* Next endpoint.  */
            endpoint =  endpoint -> ux_slave_endpoint_next_endpoint;
        }

        /* Now check if all endpoints have been found.  */
        if (stream -> ux_device_class_video_stream_endpoint == UX_NULL)
        {

            /* Error trap!  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_DESCRIPTOR_CORRUPTED);

            /* Not all endpoints have been found. Major error, do not proceed.  */
            return(UX_DESCRIPTOR_CORRUPTED);
        }

#if defined(UX_DEVICE_STANDALONE)

        /* Reset background transfer state.  */
        stream -> ux_device_class_video_stream_task_state = UX_STATE_RESET;
#endif

        /* Now reset payload buffer error count.  */
        stream -> ux_device_class_video_stream_buffer_error_count = 0;

        /* Now reset payload buffers.  */
        payload_buffer = stream -> ux_device_class_video_stream_buffer;
        while(payload_buffer < stream -> ux_device_class_video_stream_buffer + stream -> ux_device_class_video_stream_buffer_size)
        {

            /* Reset header information.  */
            *((ULONG *) payload_buffer     ) = 0;

            /* Next.  */
            payload_buffer += stream -> ux_device_class_video_stream_payload_buffer_size;
        }
        stream -> ux_device_class_video_stream_transfer_pos = stream -> ux_device_class_video_stream_access_pos;
    }
    else
    {

        /* There is no data endpoint.  */
        stream -> ux_device_class_video_stream_endpoint = UX_NULL;

        /* In this case, we are reverting to the Alternate Setting 0.  We need to terminate the pending transactions.  */
        /* Endpoints actually aborted and destroyed before change command.  */
        /*
        _ux_device_stack_transfer_all_request_abort(stream -> ux_device_class_video_stream_endpoint, UX_TRANSFER_APPLICATION_RESET);
         */
    }

    /* Invoke stream change callback.  */
    if (stream -> ux_device_class_video_stream_callbacks.ux_device_class_video_stream_change)
        stream -> ux_device_class_video_stream_callbacks.ux_device_class_video_stream_change(stream, interface_ptr -> ux_slave_interface_descriptor.bAlternateSetting);

    /* Return completion status.  */
    return(UX_SUCCESS);
}
