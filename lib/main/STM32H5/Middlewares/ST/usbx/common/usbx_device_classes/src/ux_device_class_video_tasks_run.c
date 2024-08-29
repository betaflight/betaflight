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
/*    _ux_device_class_video_tasks_run                    PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the background task of the video.                  */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    instance                              Pointer to video class        */
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
/*    (ux_device_class_video_stream_task_function)                        */
/*                                          Run stream task               */
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
UINT _ux_device_class_video_tasks_run(VOID *instance)
{
UX_SLAVE_DEVICE                 *device;
UX_DEVICE_CLASS_VIDEO           *video;
UX_DEVICE_CLASS_VIDEO_STREAM    *stream;
ULONG                           stream_index;
UINT                            status;
ULONG                           running_count;


    /* Get Video instance.  */
    video = (UX_DEVICE_CLASS_VIDEO *) instance;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Check if the device is configured.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
        return(UX_STATE_EXIT);

    /* Run video streaming tasks.  */
    running_count = 0;
    for (stream_index = 0;
         stream_index < video -> ux_device_class_video_streams_nb;
         stream_index ++)
    {
        stream = &video -> ux_device_class_video_streams[stream_index];
        status = stream -> ux_device_class_video_stream_task_function(stream);
        if (status == UX_STATE_EXIT)
            return(status);
        if (status == UX_STATE_WAIT)
            running_count ++;
    }

    /* Return state running.  */
    return (running_count > 0) ? (UX_STATE_WAIT) : (UX_STATE_IDLE);
}
#endif
