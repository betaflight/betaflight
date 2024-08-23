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
/*    _ux_host_class_video_transfer_request               PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function submits an isochronous video transfer request or      */
/*    isochronous video transfer request list to the USBX stack.          */
/*                                                                        */
/*    Note if the transfer request is not linked (next pointer is NULL),  */
/*    a single request is submitted. If the transfer request links into a */
/*    list, the whole list is submitted.                                  */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    video                                 Pointer to video class        */ 
/*    video_transfer_request                Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
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
/*                                            set pending on endpoint,    */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_transfer_request(UX_HOST_CLASS_VIDEO *video,
                                            UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST *video_transfer_request)
{

UINT            status;
UX_ENDPOINT     *endpoint;
UX_TRANSFER     *transfer_list;
UX_TRANSFER     *transfer_request;
UX_TRANSFER     *previous_transfer;

    /* Get transfer request list head.  */
    transfer_list = &video_transfer_request -> ux_host_class_video_transfer_request;

    /* Get endpoint.  */
    endpoint = video -> ux_host_class_video_isochronous_endpoint;

    /* Process the transfer request list (if multiple found).  */
    previous_transfer = UX_NULL;
    while(video_transfer_request)
    {

        /* The transfer request is embedded in the application transfer request.  */
        transfer_request =  &video_transfer_request -> ux_host_class_video_transfer_request;

        /* Select the direction. We do this by taking the endpoint direction.  */
        transfer_request -> ux_transfer_request_type =  video -> ux_host_class_video_isochronous_endpoint -> 
            ux_endpoint_descriptor.bEndpointAddress & UX_REQUEST_DIRECTION;

        /* Fill the transfer request with all the required fields.  */
        transfer_request -> ux_transfer_request_endpoint =             endpoint;
        transfer_request -> ux_transfer_request_data_pointer =         video_transfer_request -> ux_host_class_video_transfer_request_data_pointer;
        transfer_request -> ux_transfer_request_requested_length =     video_transfer_request -> ux_host_class_video_transfer_request_requested_length;
        transfer_request -> ux_transfer_request_completion_function =  _ux_host_class_video_transfer_request_completed;
        transfer_request -> ux_transfer_request_class_instance =       video;

        /* We memorize the application transfer request in the local transfer request.  */
        transfer_request -> ux_transfer_request_user_specific =  (VOID *) video_transfer_request;

        /* Confirm transfer is not linking to others  */
        transfer_request -> ux_transfer_request_next_transfer_request = UX_NULL;

        /* Add it to transfer list tail.  */
        if (previous_transfer != UX_NULL)
            previous_transfer -> ux_transfer_request_next_transfer_request = transfer_request;

        /* Save as previous transfer.  */
        previous_transfer = transfer_request;

        /* Check next transfer request.  */
        video_transfer_request = video_transfer_request -> ux_host_class_video_transfer_request_next_video_transfer_request;
    }

    /* Set endpoint status to pending, for callback and abort to check.  */
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_completion_code = UX_TRANSFER_STATUS_PENDING;

    /* Transfer the transfer request (list).  */
    status =  _ux_host_stack_transfer_request(transfer_list);

    /* Return completion status.  */
    return(status);
}
