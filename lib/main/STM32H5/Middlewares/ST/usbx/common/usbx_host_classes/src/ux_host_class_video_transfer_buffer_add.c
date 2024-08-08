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
/*    _ux_host_class_video_transfer_buffer_add            PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function adds a buffer for video transfer requests.            */
/*                                                                        */ 
/*    Note check ux_host_class_video_max_payload_get to see minimum       */
/*    recommended buffer size.                                            */
/*                                                                        */
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    video                                 Pointer to video class        */ 
/*    buffer                                Pointer to data buffer        */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_instance_verify  Verify instance is valid      */
/*    _ux_host_stack_transfer_request       Process transfer request      */
/*    _ux_host_semaphore_get                Get semaphore                 */
/*    _ux_host_semaphore_put                Release semaphore             */
/*    _ux_system_error_handler              Log system error              */ 
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
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            use pre-calculated value    */
/*                                            instead of wMaxPacketSize,  */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            set pending on endpoint,    */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_video_transfer_buffer_add(UX_HOST_CLASS_VIDEO *video, UCHAR* buffer)
{

UINT            status;
UX_TRANSFER     *transfer_request;
UX_ENDPOINT     *endpoint;
ULONG           transfer_index;
ULONG           packet_size;

    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_video_name, (VOID *) video) != UX_SUCCESS)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
    //    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, video, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Protect thread reentry to this instance.  */
    status =  _ux_host_semaphore_get(&video -> ux_host_class_video_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)
        return(status);

    /* Get endpoint.  */
    endpoint = video -> ux_host_class_video_isochronous_endpoint;

    /* Ensure we have a selected interface that allows isoch transmission.  */
    if ((endpoint == UX_NULL) ||
        (endpoint -> ux_endpoint_descriptor.wMaxPacketSize == 0))
    {

        /* Unprotect thread reentry to this instance.  */
        _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore);

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_VIDEO_WRONG_INTERFACE);

        /* Return error status.  */
        return(UX_HOST_CLASS_VIDEO_WRONG_INTERFACE);
    }

    transfer_index = video->ux_host_class_video_transfer_request_start_index;
    transfer_index ++;
    if (transfer_index == UX_HOST_CLASS_VIDEO_TRANSFER_REQUEST_COUNT)
        transfer_index = 0;
    if (transfer_index == video->ux_host_class_video_transfer_request_end_index)
    {

        /* Unprotect thread reentry to this instance.  */
        _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore);

        /* Return error status.  */
        return(UX_MEMORY_ARRAY_FULL);
    }

    transfer_request = &video->ux_host_class_video_transfer_requests[video->ux_host_class_video_transfer_request_start_index];
    video->ux_host_class_video_transfer_request_start_index = transfer_index;

    /* Select the direction. We do this by taking the endpoint direction.  */
    transfer_request -> ux_transfer_request_type =  endpoint ->
                ux_endpoint_descriptor.bEndpointAddress & UX_REQUEST_DIRECTION;

    /* Calculate packet size.  */
    packet_size = video -> ux_host_class_video_current_max_payload_size;

    /* Fill the transfer request with all the required fields.  */
    transfer_request -> ux_transfer_request_endpoint =             endpoint;
    transfer_request -> ux_transfer_request_data_pointer =         buffer;
    transfer_request -> ux_transfer_request_requested_length =     packet_size;
    transfer_request -> ux_transfer_request_completion_function =  _ux_host_class_video_transfer_request_callback;
    transfer_request -> ux_transfer_request_class_instance =       video;

    /* Add single transfer.  */
    transfer_request -> ux_transfer_request_next_transfer_request = UX_NULL;

    /* Set endpoint to pending state, for callback and abort to check.  */
    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_completion_code = UX_TRANSFER_STATUS_PENDING;

    /* Transfer the transfer request.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Unprotect thread reentry to this instance.  */
    _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore);

    /* Return completion status.  */
    return(status);
}
