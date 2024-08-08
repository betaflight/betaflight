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
/*    _ux_device_class_video_uninitialize                 PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function uninitialize the Video class.                         */
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
/*    _ux_utility_thread_delete             Delete thread used            */
/*    _ux_utility_memory_free               Free used local memory        */
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
UINT  _ux_device_class_video_uninitialize(UX_SLAVE_CLASS_COMMAND *command)
{

UX_DEVICE_CLASS_VIDEO           *video;
UX_DEVICE_CLASS_VIDEO_STREAM    *stream;
UX_SLAVE_CLASS                  *class_inst;
ULONG                            i;


    /* Get the class container.  */
    class_inst =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    video = (UX_DEVICE_CLASS_VIDEO *) class_inst -> ux_slave_class_instance;

    /* Sanity check.  */
    if (video != UX_NULL)
    {

        /* Free the stream resources.  */
        stream = (UX_DEVICE_CLASS_VIDEO_STREAM *)((UCHAR *)video + sizeof(UX_DEVICE_CLASS_VIDEO));
        for (i = 0; i < video -> ux_device_class_video_streams_nb; i ++)
        {
#if !defined(UX_DEVICE_STANDALONE)
            _ux_utility_thread_delete(&stream -> ux_device_class_video_stream_thread);
            _ux_utility_memory_free(stream -> ux_device_class_video_stream_thread_stack);
#endif
            _ux_utility_memory_free(stream -> ux_device_class_video_stream_buffer);

            /* Next stream instance.  */
            stream ++;
        }

        /* Free the video instance with controls and streams.  */
        _ux_utility_memory_free(video);
    }

    /* Return completion status.  */
    return(UX_SUCCESS);
}
