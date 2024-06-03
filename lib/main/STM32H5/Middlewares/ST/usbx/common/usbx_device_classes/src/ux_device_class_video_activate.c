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
/*    _ux_device_class_video_activate                     PORTABLE C      */
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
/*    _ux_system_error_handler              System error trap             */
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
UINT  _ux_device_class_video_activate(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_DEVICE                         *device;
UX_SLAVE_INTERFACE                      *interface_ptr;
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_INTERFACE                      *control_interface;
UX_SLAVE_INTERFACE                      *stream_interface;
UX_DEVICE_CLASS_VIDEO                   *video;
UX_DEVICE_CLASS_VIDEO_STREAM            *stream;
ULONG                                    stream_index;


    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    video = (UX_DEVICE_CLASS_VIDEO *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;

    /* Get the device instance.  */
    device = &_ux_system_slave -> ux_system_slave_device;
    video -> ux_device_class_video_device = device;

    /* We only support video interface here.  */
    if (interface_ptr -> ux_slave_interface_descriptor.bInterfaceClass != UX_DEVICE_CLASS_VIDEO_CLASS)
        return(UX_NO_CLASS_MATCH);

    /* It's control interface?  */
    if (interface_ptr -> ux_slave_interface_descriptor.bInterfaceSubClass == UX_DEVICE_CLASS_VIDEO_SUBCLASS_CONTROL)
    {

        /* Store the interface in the class instance.  */
        video -> ux_device_class_video_interface = interface_ptr;

        /* Store the class instance into the interface.  */
        interface_ptr -> ux_slave_interface_class_instance = (VOID *)video;
    }
    else
    {

        /* It's streaming interface.  */
        stream_interface = interface_ptr;

        /* Separate driver for each interface (IAD not used)?  */
        if (video -> ux_device_class_video_interface == UX_NULL)
        {

            /* Always use just 1 stream for 1 interface.  */
            stream_index = 0;
        }

        /* Re-use the same driver (IAD used)?  */
        else
        {

            /* Re-use saved control interface.  */
            control_interface = video -> ux_device_class_video_interface;

            /* Calculate stream index.  */
            stream_index  = stream_interface -> ux_slave_interface_descriptor.bInterfaceNumber;
            stream_index -= control_interface -> ux_slave_interface_descriptor.bInterfaceNumber;
            stream_index --;

        }

        /* Sanity check.  */
        if (stream_index >= video -> ux_device_class_video_streams_nb)
        {

            /* Error trap!  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);
            return(UX_MEMORY_INSUFFICIENT);
        }

        /* Locate stream based on interface number.  */
        stream = &video -> ux_device_class_video_streams[stream_index];

        /* Store the interface for the stream.  */
        stream -> ux_device_class_video_stream_interface = stream_interface;

        /* Store the class instance into the interface.  */
        stream_interface -> ux_slave_interface_class_instance = (VOID *)video;

#if defined(UX_DEVICE_STANDALONE)

        /* Reset stream task state.  */
        stream -> ux_device_class_video_stream_task_state = UX_STATE_RESET;
        stream -> ux_device_class_video_stream_task_status = UX_SUCCESS;
#endif

    }

    /* If there is a activate function call it.  */
    if (video -> ux_device_class_video_callbacks.ux_slave_class_video_instance_activate != UX_NULL)
    {

        /* Invoke the application.  */
        video -> ux_device_class_video_callbacks.ux_slave_class_video_instance_activate(video);
    }

    /* Return completion status.  */
    return(UX_SUCCESS);
}
