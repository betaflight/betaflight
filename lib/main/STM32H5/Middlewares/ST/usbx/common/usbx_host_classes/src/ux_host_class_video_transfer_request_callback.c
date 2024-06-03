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
/*    _ux_host_class_video_transfer_request_callback      PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function receives a completion call back on an isoch transfer  */
/*    request.                                                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_host_class_video_transfer_completion_function)                  */
/*                                          Transfer request completion   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Video Class                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            checked pending state,      */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_video_transfer_request_callback(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_VIDEO *video;
UX_ENDPOINT         *endpoint;
ULONG               transfer_index;


    /* Get the pointer to the video instance.  */
    video =  (UX_HOST_CLASS_VIDEO *) transfer_request -> ux_transfer_request_class_instance;

    /* Do a sanity check on the transfer request, if NULL something is wrong.  */
    if (video == UX_NULL)
        return;

    /* Check endpoint status.  */
    endpoint = video -> ux_host_class_video_isochronous_endpoint;
    if (endpoint -> ux_endpoint_transfer_request.ux_transfer_request_completion_code != UX_TRANSFER_STATUS_PENDING)
        return;

    /* The caller's transfer request needs to be updated.  */
    transfer_index = video -> ux_host_class_video_transfer_request_end_index;
    transfer_index++;
    if (transfer_index == UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST_COUNT)
        transfer_index = 0;

    /* Update the transfer index.  */
    video -> ux_host_class_video_transfer_request_end_index = transfer_index;

    /* Call the completion routine.  */
    if (video -> ux_host_class_video_transfer_completion_function)
        video -> ux_host_class_video_transfer_completion_function(transfer_request);

    /* Return to caller.  */
    return;
}
