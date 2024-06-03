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
/**   Audio Class                                                         */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_audio.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_audio_endpoints_get                  PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function searches for the handle of the isochronous endpoints. */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Pointer to audio class        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_interface_endpoint_get Get interface endpoint        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Audio Class                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            saved transfer direction,   */
/*                                            added feedback support,     */
/*                                            refined ISO support logic,  */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_audio_endpoints_get(UX_HOST_CLASS_AUDIO *audio)
{

UINT            status;
UINT            endpoint_index;
UX_ENDPOINT     *endpoint;
#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
UX_TRANSFER     *transfer;
ULONG           packet_length;
#endif

    /* Reset endpoints.  */
    audio -> ux_host_class_audio_isochronous_endpoint = UX_NULL;
#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
    audio -> ux_host_class_audio_feedback_endpoint = UX_NULL;
#endif

    /* Search the ISO OUT endpoint. It is attached to the interface container.  */
    for (endpoint_index = 0; endpoint_index < audio -> ux_host_class_audio_streaming_interface -> ux_interface_descriptor.bNumEndpoints;
                        endpoint_index++)
    {

        /* Get interface endpoint.  */
        status =  _ux_host_stack_interface_endpoint_get(audio -> ux_host_class_audio_streaming_interface, endpoint_index, &endpoint);

        /* Check completion status.  */
        if (status == UX_SUCCESS)
        {

            /* Check if endpoint is ISO.  */
            if ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_ISOCHRONOUS_ENDPOINT)
            {

                /* Check endpoint direction.  */
                if ((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT)
                {

                    /* We have found the ISO OUT endpoint.  */
                    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type = UX_REQUEST_OUT;

                    /* For OUTPUT, it's data endpoint, else feedback.  */
                    if (audio -> ux_host_class_audio_type == UX_HOST_CLASS_AUDIO_OUTPUT)
                        audio -> ux_host_class_audio_isochronous_endpoint =  endpoint;
#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
                    else
                        audio -> ux_host_class_audio_feedback_endpoint = endpoint;
#endif
                }
                else
                {

                    /* We have found the ISO IN endpoint.  */
                    endpoint -> ux_endpoint_transfer_request.ux_transfer_request_type = UX_REQUEST_IN;

                    /* For INPUT, it's data endpoint, else feedback.  */
                    if (audio -> ux_host_class_audio_type == UX_HOST_CLASS_AUDIO_INPUT)
                        audio -> ux_host_class_audio_isochronous_endpoint =  endpoint;
#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
                    else
                        audio -> ux_host_class_audio_feedback_endpoint = endpoint;
#endif
                }
            }
        }
    }

    /* The isochronous endpoint is mandatory. If we didn't find it, return an error.  */
    if (audio -> ux_host_class_audio_isochronous_endpoint == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_ENDPOINT_HANDLE_UNKNOWN);
    }

#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)

    /* With feedback, allocate buffer for packed feedback data.  */
    if (audio -> ux_host_class_audio_feedback_endpoint)
    {

        /* Get transfer.  */
        endpoint = audio -> ux_host_class_audio_feedback_endpoint;
        transfer = &endpoint -> ux_endpoint_transfer_request;

        /* Allocate buffer for feedback transfer (minimum 4 to fill both FS and HS).  */
        packet_length = transfer -> ux_transfer_request_packet_length;
        if (packet_length < sizeof(audio -> ux_host_class_audio_feedback_buffer))
            packet_length = sizeof(audio -> ux_host_class_audio_feedback_buffer);
        transfer -> ux_transfer_request_data_pointer = _ux_utility_memory_allocate(
                                UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY,
                                packet_length);
        if (transfer -> ux_transfer_request_data_pointer == UX_NULL)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_MEMORY_INSUFFICIENT);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_MEMORY_INSUFFICIENT, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_MEMORY_INSUFFICIENT);
        }
        else
        {

            /* Sync previous feedback data.  */
            _ux_utility_memory_copy(transfer -> ux_transfer_request_data_pointer,
                                    audio -> ux_host_class_audio_feedback_buffer,
                                    sizeof(audio -> ux_host_class_audio_feedback_buffer)); /* Use case of memcpy is verified. */
        }

        /* Fill the transfer request with all the required fields.  */
        transfer -> ux_transfer_request_endpoint =                  audio -> ux_host_class_audio_feedback_endpoint;
        transfer -> ux_transfer_request_requested_length =          (endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) ?
                                                                    transfer -> ux_transfer_request_packet_length :
                                                                    ((_ux_host_class_audio_speed_get(audio) == UX_HIGH_SPEED_DEVICE) ?
                                                                     UX_FEEDBACK_SIZE_HIGH_SPEED : UX_FEEDBACK_SIZE_FULL_SPEED);
        transfer -> ux_transfer_request_completion_function =       _ux_host_class_audio_feedback_transfer_completed;
        transfer -> ux_transfer_request_class_instance =            audio;

        /* Start feedback transfer in background.  */
        status = _ux_host_stack_transfer_request(transfer);
        if (status != UX_SUCCESS)
            return(status);
    }
#endif

    /* Return successful status.  */
    return(UX_SUCCESS);
}
