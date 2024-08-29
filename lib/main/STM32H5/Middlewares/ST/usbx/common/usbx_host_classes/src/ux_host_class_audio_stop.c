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
/*    _ux_host_class_audio_stop                           PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function stops the audio streaming (select alt interface 0).   */
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
/*    _ux_host_stack_endpoint_transfer_abort                              */
/*                                          Abort transfer                */
/*    _ux_host_stack_interface_setting_select                             */
/*                                          Select interface              */
/*    _ux_host_mutex_on                     Get mutex                     */
/*    _ux_host_mutex_off                    Release mutex                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_audio_stop(UX_HOST_CLASS_AUDIO *audio)
{

UINT                    status;
UX_CONFIGURATION        *configuration;
UX_INTERFACE            *interface_ptr;
UINT                    streaming_interface;


    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_audio_name, (VOID *) audio) != UX_SUCCESS)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Protect thread reentry to this instance.  */
    _ux_host_mutex_on(&audio -> ux_host_class_audio_mutex);

    /* Get the interface number of the audio streaming interface.  */
    streaming_interface =  audio -> ux_host_class_audio_streaming_interface -> ux_interface_descriptor.bInterfaceNumber;

    /* We need to abort transactions on the ISO pipes.  */
    if (audio -> ux_host_class_audio_isochronous_endpoint != UX_NULL)
    {

        /* Abort the iso transfer.  */
        _ux_host_stack_endpoint_transfer_abort(audio -> ux_host_class_audio_isochronous_endpoint);
    }

#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
    if (audio -> ux_host_class_audio_feedback_endpoint)
        _ux_host_stack_endpoint_transfer_abort(audio -> ux_host_class_audio_feedback_endpoint);
#endif

    /* We found the alternate setting for the sampling values demanded, now we need
        to search its container.  */
    configuration =        audio -> ux_host_class_audio_streaming_interface -> ux_interface_configuration;
    interface_ptr =        configuration -> ux_configuration_first_interface;

    /* Scan all interfaces.  */
    while (interface_ptr != UX_NULL)
    {

        /* We search for both the right interface and alternate setting.  */
        if ((interface_ptr -> ux_interface_descriptor.bInterfaceNumber == streaming_interface) &&
            (interface_ptr -> ux_interface_descriptor.bAlternateSetting == 0))
        {

            /* We have found the right interface/alternate setting combination
               The stack will select it for us.  */
            status =  _ux_host_stack_interface_setting_select(interface_ptr);

            /* If the alternate setting for the streaming interface could be selected, we memorize it.  */
            if (status == UX_SUCCESS)
            {

                /* Memorize the interface.  */
                audio -> ux_host_class_audio_streaming_interface =  interface_ptr;

                /* There is no endpoint for the alternate setting 0.  */
                _ux_host_stack_endpoint_transfer_abort(audio -> ux_host_class_audio_isochronous_endpoint);
                audio -> ux_host_class_audio_isochronous_endpoint = UX_NULL;
#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
                if (audio -> ux_host_class_audio_feedback_endpoint)
                {
                    _ux_host_stack_endpoint_transfer_abort(audio -> ux_host_class_audio_feedback_endpoint);
                    audio -> ux_host_class_audio_feedback_endpoint = UX_NULL;
                }
#endif

                /* Unprotect thread reentry to this instance.  */
                _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

                /* Return successful completion.  */
                return(UX_SUCCESS);
            }
        }

        /* Move to next interface.  */
        interface_ptr =  interface_ptr -> ux_interface_next_interface;
    }

    /* Unprotect thread reentry to this instance.  */
    _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

    /* Return failed completion status.  */
    return(UX_NO_ALTERNATE_SETTING);
}
