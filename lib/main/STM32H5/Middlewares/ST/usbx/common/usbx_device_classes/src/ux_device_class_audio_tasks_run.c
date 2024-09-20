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


#if defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio_tasks_run                    PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Yajun Xia, Microsoft Corporation                                    */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the background task of the audio.                  */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    instance                              Pointer to audio class        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    State machine status                                                */
/*    UX_STATE_EXIT                         Device not configured         */
/*    UX_STATE_IDLE                         No streaming transfer running */
/*    UX_STATE_WAIT                         Streaming transfer running    */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_device_class_audio_stream_task_function)                        */
/*                                          Run stream task               */
/*    (ux_device_class_audio_stream_feedback_task_function)               */
/*                                          Run stream feedback task      */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Device Stack                                                   */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  10-31-2022     Yajun Xia                Initial Version 6.2.0         */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_audio_tasks_run(VOID *instance)
{
UX_SLAVE_DEVICE                 *device;
UX_DEVICE_CLASS_AUDIO           *audio;
UX_DEVICE_CLASS_AUDIO_STREAM    *stream;
ULONG                           stream_index;
UINT                            status;
#if defined(UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT)
ULONG                           interrupt_running_count = 0;
#endif
ULONG                           stream_running_count = 0;
#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
ULONG                           feedback_running_count = 0;
#endif

    /* Get audio instance.  */
    audio = (UX_DEVICE_CLASS_AUDIO *) instance;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Check if the device is configured.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
        return(UX_STATE_EXIT);

#if defined(UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT)

    /* Run audio status tasks.  */
    status = _ux_device_class_audio_interrupt_task_function(audio);
    if (status == UX_STATE_EXIT)
        return(status);
    if (status == UX_STATE_WAIT)
        interrupt_running_count++;
#endif

    for (stream_index = 0;
         stream_index < audio -> ux_device_class_audio_streams_nb;
         stream_index ++)
    {
        stream = &audio -> ux_device_class_audio_streams[stream_index];
        status = stream -> ux_device_class_audio_stream_task_function(stream);
        if (status == UX_STATE_EXIT)
            return(status);
        if (status == UX_STATE_WAIT)
            stream_running_count ++;

#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
        if (stream -> ux_device_class_audio_stream_feedback_task_function != UX_NULL)
        {
            status = stream -> ux_device_class_audio_stream_feedback_task_function(stream);
            if (status == UX_STATE_EXIT)
                return(status);
            if (status == UX_STATE_WAIT)
                feedback_running_count ++;
        }
#endif
    }

    if ((stream_running_count > 0)
#if defined(UX_DEVICE_CLASS_AUDIO_INTERRUPT_SUPPORT)
        || (interrupt_running_count > 0)
#endif
#if defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
        || (feedback_running_count > 0)
#endif
        )
    {
        return UX_STATE_WAIT;
    }
    else
    {
        return UX_STATE_IDLE;
    }
}

#endif