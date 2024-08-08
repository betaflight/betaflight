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
/*    _ux_host_class_audio_entity_control_value_set       PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function updates the dynamic values for a single audio         */
/*    control on either the master channel or a specific channel.         */
/*                                                                        */
/*    Note only control value of BYTE, WORD and DWORD (<4) is supported.  */
/*    E.g., Graphic Equalizer Control is not supported.                   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Pointer to audio class        */
/*    audio_control                         Pointer to audio control      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_class_instance_verify  Verify instance is valid      */
/*    _ux_host_class_audio_control_request  Issue audio transfer request  */
/*    _ux_host_mutex_on                     Get mutex                     */
/*    _ux_host_mutex_off                    Release mutex                 */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_free               Release memory block          */
/*    _ux_utility_long_put                  Write 32-bit value            */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*    Audio Class                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_audio_entity_control_value_set(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_CONTROL *audio_control)
{

UINT            status;
UCHAR           *control_buffer;
ULONG           actual_size;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_AUDIO_CONTROL_VALUE_SET, audio, audio_control -> ux_host_class_audio_control_entity, audio_control -> ux_host_class_audio_control_cur, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_audio_name, (VOID *) audio) != UX_SUCCESS)
    {

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Protect thread reentry to this instance.  */
    _ux_host_mutex_on(&audio -> ux_host_class_audio_mutex);

    /* Need to allocate memory for the control buffer.  */
    control_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 4);
    if (control_buffer == UX_NULL)
    {

        /* Unprotect thread reentry to this instance.  */
        _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

        /* Return an error.  */
        return(UX_MEMORY_INSUFFICIENT);
    }

    /* The buffer should be aligned. Returned data is little endian so DWord can be used
        to copy any returned data of BYTE/WORD/DWORD.  */
    _ux_utility_long_put(control_buffer, audio_control -> ux_host_class_audio_control_cur);

    /* Issue SET CUR request.  */
    status = _ux_host_class_audio_control_request(audio, 0,
            UX_REQUEST_OUT | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE,
#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
            (_ux_host_class_audio_protocol_get(audio) == UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_02_00) ?
            UX_CLASS_AUDIO20_CUR : UX_HOST_CLASS_AUDIO_SET_CUR,
#else
            UX_HOST_CLASS_AUDIO_SET_CUR,
#endif
            audio_control -> ux_host_class_audio_control_channel |
                            (audio_control -> ux_host_class_audio_control << 8),
            audio_control -> ux_host_class_audio_control_entity,
            control_buffer, audio_control -> ux_host_class_audio_control_size, &actual_size);

    /* Free all used resources.  */
    _ux_utility_memory_free(control_buffer);

    /* Unprotect thread reentry to this instance.  */
    _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

    /* Return completion status.  */
    return(status);
}
