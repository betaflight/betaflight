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
/*    _ux_host_class_video_control_request                PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function issues a control request to a given video entity      */
/*    (unit or terminal).                                                 */
/*    The unit or terminal IDs can be parsed once the video device is     */
/*    ready, via ux_host_class_video_entities_parse.                      */
/*                                                                        */
/*    Note: only one VideoStreaming interface is supported for now.       */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    video                                 Pointer to video class        */
/*    request                               The request to issue          */
/*    interface_index                       The interface index           */
/*                                            0: VideoControl interface   */
/*                                            1: VideoStreaming 1         */
/*                                            N: VideoStreaming N         */
/*    entity_id                             The terminal or unit ID       */
/*    control_selector                      The control selector          */
/*    parameter                             Pointer to parameter block    */
/*    parameter_size                        Size of parameter block       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT _ux_host_class_video_control_request(UX_HOST_CLASS_VIDEO *video,
                                        UINT request, UCHAR interface_index,
                                        UINT entity_id, UINT control_selector,
                                        UCHAR *parameter, UINT parameter_size)
{

UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UINT            status;
UCHAR           *control_buffer;
UCHAR           request_direction;
UX_INTERFACE    *interface_ptr;
UCHAR           interface_number;


    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_video_name, (VOID *) video) != UX_SUCCESS)
        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);

    /* Protect thread reentry to this instance.  */
    status =  _ux_host_semaphore_get(&video -> ux_host_class_video_semaphore_control_request, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)
        return(status);

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &video -> ux_host_class_video_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the control buffer.  */
    control_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, parameter_size);
    if (control_buffer == UX_NULL)
    {

        /* Unprotect thread reentry to this instance.  */
        _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore_control_request);

        /* Return an error.  */
        return(UX_MEMORY_INSUFFICIENT);
    }

    /* Get the request direction from the control request ID.  */
    request_direction =  (request & 0x80);

    /* Are we setting a value?  */
    if (request_direction == 0)
    {

        /* Put parameter block in buffer.  */
        if (parameter_size)
            _ux_utility_memory_copy(control_buffer, parameter, parameter_size); /* Use case of memcpy is verified. */
    }

    /* Protect the control endpoint semaphore here.  It will be unprotected in the
       transfer request function.  */
    status =  _ux_host_semaphore_get(&video -> ux_host_class_video_device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);

    /* Check for status.  */
    if (status != UX_SUCCESS)
    {

        /* Something went wrong. */
        _ux_utility_memory_free(control_buffer);
        _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore_control_request);
        return(status);
    }

    /* Get the interface number.  */
    if (interface_index == 0)
        interface_number = (UCHAR)video -> ux_host_class_video_control_interface_number;
    else
    {

        /* Only one streaming interface is supported now.  */
        interface_ptr = video -> ux_host_class_video_streaming_interface;
        interface_number = (UCHAR)interface_ptr -> ux_interface_descriptor.bInterfaceNumber;
    }

    /* Create a transfer request for the request.  */
    transfer_request -> ux_transfer_request_data_pointer =      control_buffer;
    transfer_request -> ux_transfer_request_requested_length =  parameter_size;
    transfer_request -> ux_transfer_request_function =          request;
    transfer_request -> ux_transfer_request_type =              request_direction | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value =             (control_selector << 8);
    transfer_request -> ux_transfer_request_index =             interface_number | (entity_id << 8);

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check for correct transfer.  */
    if (status != UX_SUCCESS)
    {

        /* Free the previous control buffer.  */
        _ux_utility_memory_free(control_buffer);

        /* Unprotect thread reentry to this instance.  */
        _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore_control_request);

        /* Return completion status.  */
        return(status);
    }

    /* Are we getting a value?  */
    if (request_direction)
    {

        /* Then store it in the caller's buffer.  */
        if (parameter_size)
            _ux_utility_memory_copy(parameter, control_buffer, parameter_size); /* Use case of memcpy is verified. */
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(control_buffer);

    /* Unprotect thread reentry to this instance.  */
    _ux_host_semaphore_put(&video -> ux_host_class_video_semaphore_control_request);

    /* Return completion status.  */
    return(status);
}
