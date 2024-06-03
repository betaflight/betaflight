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
/**   Device Audio Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_audio.h"
#include "ux_device_stack.h"


#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT) && defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio_feedback_task_function       PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yajun Xia, Microsoft Corporation                                    */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is background task of ISO OUT/IN (feedback of IN/OUT) */
/*    for the Audio class.                                                */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    stream                                Pointer to audio stream       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    State machine status                                                */
/*    UX_STATE_EXIT                         Device not configured         */
/*    UX_STATE_IDLE                         No feedback transfer running  */
/*    UX_STATE_WAIT                         Feedback transfer running     */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_system_error_handler              System error trap             */
/*    _ux_device_stack_transfer_run         Run transfer state machine    */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Audio Class (task)                                                  */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  10-31-2022     Yajun Xia                Initial Version 6.2.0         */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_audio_feedback_task_function(UX_DEVICE_CLASS_AUDIO_STREAM *stream)
{

UINT                            status;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_TRANSFER               *transfer;
ULONG                           transfer_length;


    /* Get stack device instance.  */
    device = stream -> ux_device_class_audio_stream_audio -> ux_device_class_audio_device;

    /* Check if the device is configured.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

        stream -> ux_device_class_audio_stream_feedback_task_state = UX_STATE_EXIT;
        return(UX_STATE_EXIT);
    }

    /* Get endpoint instance.  */
    endpoint = stream -> ux_device_class_audio_stream_feedback;

    /* Endpoint not available, maybe it's alternate setting 0.  */
    if (endpoint == UX_NULL)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_ENDPOINT_HANDLE_UNKNOWN);

        stream -> ux_device_class_audio_stream_feedback_task_state = UX_STATE_RESET;
        return(UX_STATE_IDLE);
    }

    /* Get transfer instance.  */
    transfer = &endpoint -> ux_slave_endpoint_transfer_request;

    /* Length is pre-set on interface alternate setting activate.  */
    transfer_length = transfer -> ux_slave_transfer_request_requested_length;

    switch (stream -> ux_device_class_audio_stream_feedback_task_state)
    {
    case UX_STATE_RESET:
        stream -> ux_device_class_audio_stream_feedback_task_state = UX_DEVICE_CLASS_AUDIO_STREAM_FEEDBACK_RW_WAIT;
        stream -> ux_device_class_audio_stream_feedback_task_status = UX_TRANSFER_NO_ANSWER;

    /* Fall through.  */
    case UX_DEVICE_CLASS_AUDIO_STREAM_FEEDBACK_RW_WAIT:

        /* Issue transfer request.  */
        status = _ux_device_stack_transfer_run(transfer, transfer_length, transfer_length);

        /* Any error or success case.  */
        if (status < UX_STATE_NEXT)
        {

            /* Error notification!  */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_ERROR);

            stream -> ux_device_class_audio_stream_feedback_task_state = UX_STATE_RESET;
            stream -> ux_device_class_audio_stream_feedback_task_status = transfer -> ux_slave_transfer_request_completion_code;
            return(UX_STATE_EXIT);
        }

        /* Success case.  */
        if (status == UX_STATE_NEXT)
        {
            stream -> ux_device_class_audio_stream_feedback_task_state = UX_DEVICE_CLASS_AUDIO_STREAM_FEEDBACK_RW_WAIT;
            stream -> ux_device_class_audio_stream_feedback_task_status = transfer -> ux_slave_transfer_request_completion_code;
        }

        /* Keep waiting.  */
        return(UX_STATE_WAIT);

    default:
        stream -> ux_device_class_audio_stream_feedback_task_state = UX_STATE_RESET;
        stream -> ux_device_class_audio_stream_feedback_task_status = UX_INVALID_STATE;
        break;
    }

    /* Error case.  */
    return(UX_STATE_EXIT);
}
#endif
