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
/*    _ux_host_class_video_transfer_request_completed     PORTABLE C      */ 
/*                                                           6.1          */
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
/*    (ux_host_class_video_transfer_request_completion_function)          */ 
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
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_video_transfer_request_completed(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST     *video_transfer_request;
    

    /* Get the pointer to the video specific transfer request, by nature of the lined transfer requests,
       the corresponding transfer request has to be the head transfer request in the video instance.  */
    video_transfer_request =  (UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST *) transfer_request -> ux_transfer_request_user_specific;

    /* Do a sanity check on the transfer request, if NULL something is wrong.  */
    if (video_transfer_request == UX_NULL)
        return;

    /* The caller's transfer request needs to be updated.  */
    video_transfer_request -> ux_host_class_video_transfer_request_actual_length =    transfer_request -> ux_transfer_request_actual_length;
    video_transfer_request -> ux_host_class_video_transfer_request_completion_code =  transfer_request -> ux_transfer_request_completion_code;
    
    /* Call the completion routine.  */
    video_transfer_request -> ux_host_class_video_transfer_request_completion_function(video_transfer_request);

    /* Return to caller.  */
    return;
}

