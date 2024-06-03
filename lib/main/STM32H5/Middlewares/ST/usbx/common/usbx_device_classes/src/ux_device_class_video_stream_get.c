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
/*    _ux_device_class_video_stream_get                   PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function get the stream instance of Video class.               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    video                                 Address of video class        */
/*                                            instance                    */
/*    stream_index                          Stream instance index 0 based */
/*    stream                                Pointer to buffer to fill     */
/*                                            pointer to stream instance  */
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
/*    Video Class                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/
UINT    _ux_device_class_video_stream_get(UX_DEVICE_CLASS_VIDEO *video,
        ULONG stream_index, UX_DEVICE_CLASS_VIDEO_STREAM **stream)
{


    /* Sanity check.  */
    if (video == UX_NULL)
        return(UX_ERROR);

    /* Index validation.  */
    if (stream_index >= video -> ux_device_class_video_streams_nb)
        return(UX_ERROR);

    /* Store the stream instance found.  */
    if (stream)
        *stream = video -> ux_device_class_video_streams + stream_index;

    /* Return completion status.  */
    return(UX_SUCCESS);
}
