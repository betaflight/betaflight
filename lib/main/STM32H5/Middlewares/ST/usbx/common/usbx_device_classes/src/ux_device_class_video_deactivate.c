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


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_video_deactivate                   PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function deactivate an instance of the video class.            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to a class command    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_all_request_abort                         */
/*                                          Abort all transfers           */
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_video_deactivate(UX_SLAVE_CLASS_COMMAND *command)
{

UX_DEVICE_CLASS_VIDEO           *video;
UX_DEVICE_CLASS_VIDEO_STREAM    *stream;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_CLASS                  *class_inst;
UINT                             i;


    /* Get the class container.  */
    class_inst =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    video = (UX_DEVICE_CLASS_VIDEO *) class_inst -> ux_slave_class_instance;

    /* Stop pending streams.  */
    stream = video -> ux_device_class_video_streams;
    for (i = 0; i < video -> ux_device_class_video_streams_nb; i ++)
    {

        /* Locate the endpoint.  */
        endpoint = stream -> ux_device_class_video_stream_endpoint;

        /* Terminate the transactions pending on the endpoint.  */
        if (endpoint)
            _ux_device_stack_transfer_all_request_abort(endpoint, UX_TRANSFER_BUS_RESET);

        /* Free the stream.  */
        stream -> ux_device_class_video_stream_endpoint = UX_NULL;
        stream -> ux_device_class_video_stream_interface = UX_NULL;

        stream ++;
    }

    /* Free the control.  */
    video -> ux_device_class_video_interface = UX_NULL;

    /* If there is a deactivate function call it.  */
    if (video -> ux_device_class_video_callbacks.ux_slave_class_video_instance_deactivate != UX_NULL)

        /* Invoke the application.  */
        video -> ux_device_class_video_callbacks.ux_slave_class_video_instance_deactivate(video);

    /* Return completion status.  */
    return(UX_SUCCESS);
}
