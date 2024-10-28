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
/**   Video Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_video.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_video_start                          PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function starts the video streaming.                           */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    video                                 Pointer to video class        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_video_channel_start    Start video device            */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_start(UX_HOST_CLASS_VIDEO *video)
{

UX_HOST_CLASS_VIDEO_PARAMETER_CHANNEL       channel_parameter;
UINT                                        status;


    /* Get current parameters from the video instance.  */
    channel_parameter.ux_host_class_video_parameter_format_requested = video -> ux_host_class_video_current_format;
    channel_parameter.ux_host_class_video_parameter_frame_requested = video -> ux_host_class_video_current_frame;
    channel_parameter.ux_host_class_video_parameter_frame_interval_requested = video -> ux_host_class_video_current_frame_interval;
    channel_parameter.ux_host_class_video_parameter_channel_bandwidth_selection = 0;

    /* Start the video with the parameters.  */
    status = _ux_host_class_video_channel_start(video, &channel_parameter);

    /* Reset indices for transfer requests.  */
    video -> ux_host_class_video_transfer_request_start_index = 0;
    video -> ux_host_class_video_transfer_request_end_index = 0;

    /* Return status.  */
    return(status);
}

