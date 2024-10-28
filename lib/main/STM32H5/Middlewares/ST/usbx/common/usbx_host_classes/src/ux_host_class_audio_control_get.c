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
/*    _ux_host_class_audio_control_get                    PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains the static feature values for a single audio  */
/*    control on either the master channel or a specific channel.         */
/*                                                                        */ 
/*    Note only control value of BYTE, WORD and DWORD (<4) is supported.  */
/*    E.g., Graphic Equalizer Control is not supported.                   */
/*                                                                        */
/*    Note only first RANGE of a list is returned.                        */
/*                                                                        */
/*    Note it's not recommended for supporting audio device with complex  */
/*    pinout scheme, where ux_host_class_audio_descriptors_parse can be   */
/*    used to build audio pinout scheme, and settings can be changed by   */
/*    ux_host_class_audio_control_request or ux_host_class_audio_feature  */
/*    functions.                                                          */
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
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added audio 2.0 support,    */
/*                                            protect reentry with mutex, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_audio_control_get(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_CONTROL *audio_control)
{
#if defined(UX_HOST_CLASS_AUDIO_DISABLE_CONTROLS)
    UX_PARAMETER_NOT_USED(audio);
    UX_PARAMETER_NOT_USED(audio_control);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UINT            status;
UCHAR *         control_buffer;


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

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &audio -> ux_host_class_audio_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Protect the control endpoint semaphore here.  It will be unprotected in the
       transfer request function.  */
    status =  _ux_host_semaphore_get(&audio -> ux_host_class_audio_device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);

    /* Check for status.  */
    if (status != UX_SUCCESS)
    {

        /* Something went wrong. */
        _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
        return(status);
    }

#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
    if (_ux_host_class_audio_protocol_get(audio) == UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_02_00)
    {
        /*                          Size    RANGE Size
        D1..0: Mute                 1       2+1*3=5
        D3..2: Volume               2       2+2*3=8
        D5..4: Bass                 1
        D7..6: Mid                  1
        D9..8: Treble               1
        D11..10: Graphic Equalizer  4+NrBits
        D13..12: Automatic Gain     1
        D15..14: Delay              4       2+4*3=14
        D17..16: Bass Boost         1
        D19..18: Loudness           1
        D21..20: Input Gain         2
        D23..22: Input Gain Pad     2
        D25..24: Phase Inverter     1
        D27..26: Underflow          1
        D29..28: Overflow           1
        */

        /* Need to allocate enough memory for the control buffer.  */
        control_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 16);
        if (control_buffer == UX_NULL)
        {

            /* Return an error.  */
            _ux_host_semaphore_put(&audio -> ux_host_class_audio_device -> ux_device_protection_semaphore);
            _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
            return(UX_MEMORY_INSUFFICIENT);
        }

        /* Create a transfer request for the GET RANGE request.  */
        transfer_request -> ux_transfer_request_data_pointer =      control_buffer;
        transfer_request -> ux_transfer_request_requested_length =  16;
        transfer_request -> ux_transfer_request_function =          UX_CLASS_AUDIO20_RANGE;
        transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
        transfer_request -> ux_transfer_request_value =             audio_control -> ux_host_class_audio_control_channel | (audio_control -> ux_host_class_audio_control << 8);
        transfer_request -> ux_transfer_request_index =             audio -> ux_host_class_audio_control_interface_number | (audio -> ux_host_class_audio_feature_unit_id << 8);

        /* Send request to HCD layer.  */
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check request status code.  */
        if (status == UX_SUCCESS)
        {

            /* Check returned length for parameter layouts.  */
            switch(transfer_request -> ux_transfer_request_actual_length)
            {

            /* Layout 3: wNumSubranges, dMIN, dMAX, dRES.  */
            case 14:
                audio_control -> ux_host_class_audio_control_min = _ux_utility_long_get(control_buffer + 2);
                audio_control -> ux_host_class_audio_control_max = _ux_utility_long_get(control_buffer + 6);
                audio_control -> ux_host_class_audio_control_res = _ux_utility_long_get(control_buffer + 10);
                break;

            /* Layout 2: wNumSubranges, wMIN, wMAX, wRES.  */
            case 8:
                audio_control -> ux_host_class_audio_control_min = _ux_utility_short_get(control_buffer + 2);
                audio_control -> ux_host_class_audio_control_max = _ux_utility_short_get(control_buffer + 4);
                audio_control -> ux_host_class_audio_control_res = _ux_utility_short_get(control_buffer + 6);
                break;

            /* Layout 1: wNumSubranges, bMIN, bMAX, bRES.  */
            case 5:
                audio_control -> ux_host_class_audio_control_min = *(control_buffer + 2);
                audio_control -> ux_host_class_audio_control_max = *(control_buffer + 3);
                audio_control -> ux_host_class_audio_control_res = *(control_buffer + 4);
                break;

            /* Invalid data.  */
            default:
                status = UX_ERROR;
                break;
            }
        }
    }
    else
#endif
    {

        /*                      Size
        D0: Mute                1
        D1: Volume              2
        D2: Bass                1
        D3: Mid                 1
        D4: Treble              1
        D5: Graphic Equalizer   4+NrBits
        D6: Automatic Gain      1
        D7: Delay               2
        D8: Bass Boost          1
        D9: Loudness            1
        */

        /* Need to allocate memory for the control buffer.  */
        control_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 4);
        if (control_buffer == UX_NULL)
        {

            /* Return an error.  */
            _ux_host_semaphore_put(&audio -> ux_host_class_audio_device -> ux_device_protection_semaphore);
            _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
            return(UX_MEMORY_INSUFFICIENT);
        }

        /* The buffer should be aligned. Returned data is little endian so DWord can be used
           to copy any returned data of BYTE/WORD/DWORD.  */

        /* Create a transfer request for the GET_MIN request.  */
        transfer_request -> ux_transfer_request_data_pointer =      control_buffer;
        transfer_request -> ux_transfer_request_requested_length =  4;
        transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
        transfer_request -> ux_transfer_request_value =             audio_control -> ux_host_class_audio_control_channel | (audio_control -> ux_host_class_audio_control << 8);
        transfer_request -> ux_transfer_request_index =             audio -> ux_host_class_audio_control_interface_number | (audio -> ux_host_class_audio_feature_unit_id << 8);
        transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_AUDIO_GET_MIN;

        /* Send request to HCD layer.  */
        * (ULONG *)control_buffer = 0;
        status =  _ux_host_stack_transfer_request(transfer_request);

        /* Check for correct transfer and entire control buffer returned.
         * Start GET_MAX request.  */
        if (status == UX_SUCCESS)
        {

            /* Update the MIN static value for the caller.  */
            audio_control -> ux_host_class_audio_control_min = _ux_utility_long_get(control_buffer);

            /* Protect the control endpoint semaphore here.  It will be unprotected in the
               transfer request function.  */
            status =  _ux_host_semaphore_get(&audio -> ux_host_class_audio_device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
            if (status != UX_SUCCESS)
            {
                _ux_utility_memory_free(control_buffer);
                _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                return(status);
            }

            /* Create a transfer request for the GET_MAX request.  */
            transfer_request -> ux_transfer_request_data_pointer =      control_buffer;
            transfer_request -> ux_transfer_request_requested_length =  4;
            transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
            transfer_request -> ux_transfer_request_value =             audio_control -> ux_host_class_audio_control_channel | (audio_control -> ux_host_class_audio_control << 8);
            transfer_request -> ux_transfer_request_index =             audio -> ux_host_class_audio_control_interface_number | (audio -> ux_host_class_audio_feature_unit_id << 8);
            transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_AUDIO_GET_MAX;

            /* Send request to HCD layer.  */
            * (ULONG *)control_buffer = 0;
            status =  _ux_host_stack_transfer_request(transfer_request);
        }

        /* Check for correct transfer and entire control buffer returned.
         * Start GET_RES request.  */
        if (status == UX_SUCCESS)
        {

            /* Update the MAX static value for the caller.  */
            audio_control -> ux_host_class_audio_control_max = _ux_utility_long_get(control_buffer);

            /* Protect the control endpoint semaphore here.  It will be unprotected in the
               transfer request function.  */
            status =  _ux_host_semaphore_get(&audio -> ux_host_class_audio_device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
            if (status != UX_SUCCESS)
            {
                _ux_utility_memory_free(control_buffer);
                _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                return(status);
            }

            /* Create a transfer request for the GET_RES request.  */
            transfer_request -> ux_transfer_request_data_pointer =      control_buffer;
            transfer_request -> ux_transfer_request_requested_length =  4;
            transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
            transfer_request -> ux_transfer_request_value =             audio_control -> ux_host_class_audio_control_channel | (audio_control -> ux_host_class_audio_control << 8);
            transfer_request -> ux_transfer_request_index =             audio -> ux_host_class_audio_control_interface_number | (audio -> ux_host_class_audio_feature_unit_id << 8);
            transfer_request -> ux_transfer_request_function =          UX_HOST_CLASS_AUDIO_GET_RES;

            /* Send request to HCD layer.  */
            * (ULONG *)control_buffer = 0;
            status =  _ux_host_stack_transfer_request(transfer_request);
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
#endif
}
