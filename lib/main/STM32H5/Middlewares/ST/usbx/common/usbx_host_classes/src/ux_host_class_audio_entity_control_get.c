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
/*    _ux_host_class_audio_entity_control_get             PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function obtains the static values for a single audio control  */
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
/*    _ux_host_stack_transfer_request       Process transfer request      */
/*    _ux_host_semaphore_get                Get semaphore                 */
/*    _ux_host_semaphore_put                Release semaphore             */
/*    _ux_host_mutex_on                     Get mutex                     */
/*    _ux_host_mutex_off                    Release mutex                 */
/*    _ux_utility_memory_allocate           Allocate memory block         */
/*    _ux_utility_memory_free               Release memory block          */
/*    _ux_utility_short_get                 Read 16-bit value             */
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
UINT  _ux_host_class_audio_entity_control_get(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_CONTROL *audio_control)
{

UINT            status;
UCHAR *         control_buffer;
ULONG           actual_size;
#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
ULONG           n_subs, sub, pos, min, max, res, size;
#endif


    /* Validate control size.  */
    if (audio_control -> ux_host_class_audio_control_size > 4)
        return(UX_INVALID_PARAMETER);

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

#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
    if (_ux_host_class_audio_protocol_get(audio) == UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_02_00)
    {

        /* Need to allocate enough memory for the control buffer.  */
        /* Max RANGE size (single item): 2+4*3=14.  */
        control_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 16);
        if (control_buffer == UX_NULL)
        {

            /* Return an error.  */
            _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
            return(UX_MEMORY_INSUFFICIENT);
        }

        /* Issue GET RANGE request.  */
        status = _ux_host_class_audio_control_request(audio, 0,
            UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE,
            UX_CLASS_AUDIO20_RANGE,
            audio_control -> ux_host_class_audio_control_channel |
                (audio_control -> ux_host_class_audio_control << 8),
            audio_control -> ux_host_class_audio_control_entity,
            control_buffer, 16, &actual_size);

        /* Check request status code.  */
        if (status == UX_SUCCESS)
        {

            /* Check returned size.  */
            if (actual_size < 2)
            {

                /* Return an error.  */
                _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                _ux_utility_memory_free(control_buffer);
                return(UX_TRANSFER_ERROR);
            }

            /* Check wNumSubRanges.  */
            n_subs = _ux_utility_short_get(control_buffer);
            if (n_subs > 1)
            {

                /* wNumSubRanges is short, control size max 4, no overflow.  */
                size = 2 + n_subs * audio_control -> ux_host_class_audio_control_size;
                if (size > 16)
                {

                    /* Issue GET RANGE again with larger buffer.  */
                    _ux_utility_memory_free(control_buffer);
                    control_buffer = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, size);
                    if (control_buffer == UX_NULL)
                    {

                        /* Return an error.  */
                        _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                        return(UX_MEMORY_INSUFFICIENT);
                    }

                    /* Issue GET RANGE request.  */
                    status = _ux_host_class_audio_control_request(audio, 0,
                        UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE,
                        UX_CLASS_AUDIO20_RANGE,
                        audio_control -> ux_host_class_audio_control_channel |
                            (audio_control -> ux_host_class_audio_control << 8),
                        audio_control -> ux_host_class_audio_control_entity,
                        control_buffer, size, &actual_size);

                    if ((status != UX_SUCCESS) || (actual_size != size))
                    {

                        /* Return an error.  */
                        _ux_utility_memory_free(control_buffer);
                        _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                        return(UX_TRANSFER_ERROR);
                    }
                }
            }

            /* Parse returned values.  */
            audio_control -> ux_host_class_audio_control_max = 0;
            audio_control -> ux_host_class_audio_control_min = 0xFFFFFFFF;
            audio_control -> ux_host_class_audio_control_res = 0;
            pos = 2;
            for (sub = 0; sub < n_subs; sub ++)
            {

                /* Get MIN, MAX and POS.  */
                switch(audio_control -> ux_host_class_audio_control_size)
                {
                case 1:
                    min = *(control_buffer + pos);
                    pos ++;
                    max = *(control_buffer + pos);
                    pos ++;
                    res = *(control_buffer + pos);
                    pos ++;
                    break;

                case 2:
                    min = _ux_utility_short_get(control_buffer + pos);
                    pos += 2;
                    max = _ux_utility_short_get(control_buffer + pos);
                    pos += 2;
                    res = _ux_utility_short_get(control_buffer + pos);
                    pos += 2;
                    break;

                default:
                    min = _ux_utility_long_get(control_buffer + pos);
                    pos += 4;
                    max = _ux_utility_long_get(control_buffer + pos);
                    pos += 4;
                    res = _ux_utility_long_get(control_buffer + pos);
                    pos += 4;
                    break;
                }

                /* Keep MAX.  */
                if (audio_control -> ux_host_class_audio_control_max < max)
                    audio_control -> ux_host_class_audio_control_max = max;

                /* Keep MIN.  */
                if (audio_control -> ux_host_class_audio_control_min > min)
                    audio_control -> ux_host_class_audio_control_min = min;

                /* If there is RES, keep the min of RES.  */
                if (audio_control -> ux_host_class_audio_control_res == 0)
                    audio_control -> ux_host_class_audio_control_res = res;
                else if (res < audio_control -> ux_host_class_audio_control_res)
                    audio_control -> ux_host_class_audio_control_res = res;
            }

            /* If there is list of RANGEs and no RES, guess it (average of max-min).  */
            if (n_subs > 1 && audio_control -> ux_host_class_audio_control_res == 0)
            {
                audio_control -> ux_host_class_audio_control_res =
                    (audio_control -> ux_host_class_audio_control_max -
                    audio_control -> ux_host_class_audio_control_min) / n_subs;
            }
        }
    }
    else
#endif
    {

        /* Need to allocate memory for the control buffer, max DWORD (4).  */
        control_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 4);
        if (control_buffer == UX_NULL)
        {

            /* Return an error.  */
            _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
            return(UX_MEMORY_INSUFFICIENT);
        }

        /* Issue GET_MIN request.  */
        * (ULONG *)control_buffer = 0;
        status = _ux_host_class_audio_control_request(audio, 0,
            UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE,
            UX_HOST_CLASS_AUDIO_GET_MIN,
            audio_control -> ux_host_class_audio_control_channel |
                (audio_control -> ux_host_class_audio_control << 8),
            audio_control -> ux_host_class_audio_control_entity,
            control_buffer, 4, &actual_size);

        /* Check for correct transfer and entire control buffer returned.
         * Start GET_MAX request.  */
        if (status == UX_SUCCESS)
        {

            /* Update the MIN static value for the caller.  */
            audio_control -> ux_host_class_audio_control_min = _ux_utility_long_get(control_buffer);

            /* Issue GET_MAX request.  */
            * (ULONG *)control_buffer = 0;
            status = _ux_host_class_audio_control_request(audio, 0,
                UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE,
                UX_HOST_CLASS_AUDIO_GET_MAX,
                audio_control -> ux_host_class_audio_control_channel |
                    (audio_control -> ux_host_class_audio_control << 8),
                audio_control -> ux_host_class_audio_control_entity,
                control_buffer, 4, &actual_size);
        }

        /* Check for correct transfer and entire control buffer returned.
         * Start GET_RES request.  */
        if (status == UX_SUCCESS)
        {

            /* Update the MAX static value for the caller.  */
            audio_control -> ux_host_class_audio_control_max = _ux_utility_long_get(control_buffer);

            /* Issue GET_RES request.  */
            * (ULONG *)control_buffer = 0;
            status = _ux_host_class_audio_control_request(audio, 0,
                UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE,
                UX_HOST_CLASS_AUDIO_GET_RES,
                audio_control -> ux_host_class_audio_control_channel |
                    (audio_control -> ux_host_class_audio_control << 8),
                audio_control -> ux_host_class_audio_control_entity,
                control_buffer, 4, &actual_size);
        }

        /* Check for correct transfer and entire control buffer returned.  */
        if (status == UX_SUCCESS)
        {

            /* Update the RES static value for the caller.  */
            audio_control -> ux_host_class_audio_control_res = _ux_utility_long_get(control_buffer);
        }
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(control_buffer);

    /* Unprotect thread reentry to this instance.  */
    _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

    /* Return completion status.  */
    return(status);
}
