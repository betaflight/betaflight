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


#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio_feedback_thread_entry        PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is thread of ISO OUT/IN (feedback of IN/OUT)          */
/*    for the Audio class.                                                */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio_stream                          Address of audio stream       */
/*                                            instance                    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_system_error_handler              System error trap             */
/*    _ux_utility_thread_suspend            Suspend thread used           */
/*    _ux_device_stack_transfer_request     Issue transfer request        */
/*    _ux_utility_memory_copy               Copy data                     */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX                                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  10-31-2022     Yajun Xia                Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
VOID _ux_device_class_audio_feedback_thread_entry(ULONG audio_stream)
{
#if defined(UX_DEVICE_STANDALONE)
    UX_PARAMETER_NOT_USED(audio_stream);
    return;
#else
UINT                            status;
UX_DEVICE_CLASS_AUDIO_STREAM    *stream;
UX_SLAVE_DEVICE                 *device;
UX_SLAVE_ENDPOINT               *endpoint;
UX_SLAVE_TRANSFER               *transfer;
ULONG                           transfer_length;


    /* Get Audio class stream instance.  */
    UX_THREAD_EXTENSION_PTR_GET(stream, UX_DEVICE_CLASS_AUDIO_STREAM, audio_stream)

    /* Get stack device instance.  */
    device = stream -> ux_device_class_audio_stream_audio -> ux_device_class_audio_device;

    /* This thread runs forever but can be suspended or resumed.  */
    while (1)
    {
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        {

            /* Get endpoint instance.  */
            endpoint = stream -> ux_device_class_audio_stream_feedback;

            /* Endpoint not available, maybe it's alternate setting 0.  */
            if (endpoint == UX_NULL)
                break;

            /* Get transfer instance.  */
            transfer = &endpoint -> ux_slave_endpoint_transfer_request;

            /* Length is pre-set on interface alternate setting activate.  */
            transfer_length = transfer -> ux_slave_transfer_request_requested_length;

            /* Issue transfer request, thread blocked until transfer done.  */
            status = _ux_device_stack_transfer_request(transfer, transfer_length, transfer_length);

            /* Check error.  */
            if (status == UX_TRANSFER_BUS_RESET)
                continue;
            if (status != UX_SUCCESS)
            {

                /* Error notification!  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_TRANSFER_ERROR);
                break;
            }

            /* Transfer done, keep data in transfer buffer.  */
        }

        /* We need to suspend ourselves. We will be resumed by the device enumeration module or when a change of alternate setting happens.  */
        _ux_utility_thread_suspend(&stream -> ux_device_class_audio_stream_feedback_thread);
    }
#endif
}
#endif
