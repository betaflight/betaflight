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
/*    _ux_host_class_audio_streaming_sampling_set         PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function selects the right alternate setting for the audio     */
/*    streaming interface based on the RAW (PCM like) sampling values     */
/*    specified by the user, and send sampling frequency requests to      */
/*    select expected sample rate (if necessary).                         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Pointer to audio class        */
/*    audio_sampling                        Pointer to audio sampling     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_class_audio_alternate_setting_locate                       */
/*                                          Locate alternate setting      */
/*    _ux_host_stack_class_instance_verify  Verify instance is valid      */
/*    _ux_host_stack_interface_endpoint_get Get interface endpoint        */
/*    _ux_host_stack_interface_setting_select Select interface            */
/*    _ux_host_semaphore_get                Get semaphore                 */
/*    _ux_host_semaphore_put                Put semaphore                 */
/*    _ux_host_mutex_on                     Get mutex                     */
/*    _ux_host_mutex_off                    Release mutex                 */
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
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            added audio 2.0 support,    */
/*                                            internal clean up,          */
/*                                            set sample rate if needed,  */
/*                                            protect reentry with mutex, */
/*                                            fixed error return code,    */
/*                                            used endpoints get API,     */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_audio_streaming_sampling_set(UX_HOST_CLASS_AUDIO *audio, UX_HOST_CLASS_AUDIO_SAMPLING *audio_sampling)
{

UINT                    status;
UINT                    alternate_setting;
UX_CONFIGURATION        *configuration;
UX_INTERFACE            *interface_ptr;
UINT                    streaming_interface;
UCHAR                   *descriptor;
ULONG                   set_frequency;
UCHAR                   *control_buffer;
UX_DEVICE               *device;
UX_TRANSFER             *transfer;
ULONG                   frequency = 0;
ULONG                   res_bytes;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_AUDIO_STREAMING_SAMPLING_SET, audio, audio_sampling, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_audio_name, (VOID *) audio) != UX_SUCCESS)
        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);

    /* Protect thread reentry to this instance.  */
    _ux_host_mutex_on(&audio -> ux_host_class_audio_mutex);

    /* Find the correct alternate setting for the sampling desired.  */
    status =  _ux_host_class_audio_alternate_setting_locate(audio, audio_sampling, &alternate_setting);

    /* Did we find the alternate setting?  */
    if (status != UX_SUCCESS)
    {

        /* Unprotect thread reentry to this instance.  */
        _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
        return(status);
    }

    /* We found the alternate setting for the sampling values demanded, now we need
        to search its container.  */
    configuration =        audio -> ux_host_class_audio_streaming_interface -> ux_interface_configuration;
    interface_ptr =        configuration -> ux_configuration_first_interface;
    streaming_interface =  audio -> ux_host_class_audio_streaming_interface -> ux_interface_descriptor.bInterfaceNumber;
    device =               audio -> ux_host_class_audio_device;

    /* Scan all interfaces.  */
    while (interface_ptr != UX_NULL)
    {

        /* We search for both the right interface and alternate setting.  */
        if ((interface_ptr -> ux_interface_descriptor.bInterfaceNumber == streaming_interface) &&
            (interface_ptr -> ux_interface_descriptor.bAlternateSetting == alternate_setting))
        {

            /* We have found the right interface/alternate setting combination
               The stack will select it for us.  */
            status =  _ux_host_stack_interface_setting_select(interface_ptr);

            /* If the alternate setting for the streaming interface could be selected, we memorize it.  */
            if (status == UX_SUCCESS)
            {

                /* Memorize the interface.  */
                audio -> ux_host_class_audio_streaming_interface =  interface_ptr;

                /* Get streaming endpoints.  */
                status = _ux_host_class_audio_endpoints_get(audio);
                if (status != UX_SUCCESS)
                {

                    /* Unprotect thread reentry to this instance.  */
                    _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                    return(status);
                }

                /* Check if clock frequency needs modification through requests.  */
                set_frequency = UX_TRUE;
                descriptor = audio -> ux_host_class_audio_sampling_descriptor;
#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
                if (_ux_host_class_audio_protocol_get(audio) != UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_01_00)
                {

                    /* Check CSD::bmControls@5.0~1.  */
                    /* If not programmable, no need to set.  */
                    if ((descriptor[5] & UX_CLASS_AUDIO20_CONTROL_MASK) !=
                        UX_CLASS_AUDIO20_CONTROL_PROGRAMMABLE)
                        set_frequency = UX_FALSE;
                }
                else
#endif
                {

                    /* Check FormatTypeI::bSamFreqType@7, tSamFreq@8, tSamFreq@11.  */
                    /* If there is only one frequency, no need to set.  */
                    if ((descriptor[7] == 1) || ((descriptor[7] == 0) &&
                        (descriptor[8] == descriptor[11]) &&
                        (descriptor[9] == descriptor[12]) &&
                        (descriptor[10] == descriptor[13])))
                        set_frequency = UX_FALSE;
                }

                /* Send requests to set frequency.  */
                if (set_frequency)
                {

                    /* Allocate buffer.  */
                    control_buffer = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, 4);
                    if (control_buffer == UX_NULL)
                    {
                        _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                        return(UX_MEMORY_INSUFFICIENT);
                    }

                    /* Get control transfer.  */
                    transfer = &device -> ux_device_control_endpoint.ux_endpoint_transfer_request;

                    /* Set data to send.  */
                    control_buffer[0] = (UCHAR)UX_DW0(audio_sampling -> ux_host_class_audio_sampling_frequency);
                    control_buffer[1] = (UCHAR)UX_DW1(audio_sampling -> ux_host_class_audio_sampling_frequency);
                    control_buffer[2] = (UCHAR)UX_DW2(audio_sampling -> ux_host_class_audio_sampling_frequency);

                    /* Protect the control endpoint semaphore here.  It will be unprotected in the
                        transfer request function.  */
                    status =  _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
                    if (status != UX_SUCCESS)
                    {
                        _ux_utility_memory_free(control_buffer);
                        _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                        return(status);
                    }

#if defined(UX_HOST_CLASS_AUDIO_2_SUPPORT)
                    if (_ux_host_class_audio_protocol_get(audio) != UX_HOST_CLASS_AUDIO_PROTOCOL_IP_VERSION_01_00)
                    {

                        /* Set last byte.  */
                        control_buffer[3] = (UCHAR)UX_DW3(audio_sampling -> ux_host_class_audio_sampling_frequency);

                        /* Create a transfer request for the CUR request.  */
                        transfer -> ux_transfer_request_data_pointer =      control_buffer;
                        transfer -> ux_transfer_request_requested_length =  4;
                        transfer -> ux_transfer_request_function =          UX_CLASS_AUDIO20_CUR;
                        transfer -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TARGET_INTERFACE | UX_REQUEST_TYPE_CLASS;
                        transfer -> ux_transfer_request_value =             UX_CLASS_AUDIO20_CS_SAM_FREQ_CONTROL << 8;
                        transfer -> ux_transfer_request_index =             audio -> ux_host_class_audio_control_interface_number | ((UINT)descriptor[3] << 8);
                        status = _ux_host_stack_transfer_request(transfer);
                        if (status != UX_SUCCESS)
                        {
                            _ux_utility_memory_free(control_buffer);
                            _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                            return(status);
                        }

                        /* Issue CUR request.  */
                        status =  _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
                        if (status != UX_SUCCESS)
                        {
                            _ux_utility_memory_free(control_buffer);
                            _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                            return(status);
                        }

                        /* Create a transfer request for the CUR request.  */
                        transfer -> ux_transfer_request_data_pointer =      control_buffer;
                        transfer -> ux_transfer_request_requested_length =  4;
                        transfer -> ux_transfer_request_function =          UX_CLASS_AUDIO20_CUR;
                        transfer -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TARGET_INTERFACE | UX_REQUEST_TYPE_CLASS;
                        transfer -> ux_transfer_request_value =             UX_CLASS_AUDIO20_CS_SAM_FREQ_CONTROL << 8;
                        transfer -> ux_transfer_request_index =             audio -> ux_host_class_audio_control_interface_number | ((UINT)descriptor[3] << 8);
                        status = _ux_host_stack_transfer_request(transfer);
                    }
                    else
#endif
                    {

                        /* Create a transfer request for the SET_CUR request.  */
                        transfer -> ux_transfer_request_data_pointer =      control_buffer;
                        transfer -> ux_transfer_request_requested_length =  3;
                        transfer -> ux_transfer_request_function =          UX_CLASS_AUDIO10_SET_CUR;
                        transfer -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TARGET_ENDPOINT | UX_REQUEST_TYPE_CLASS;
                        transfer -> ux_transfer_request_value =             UX_CLASS_AUDIO10_EP_SAMPLING_FREQ_CONTROL << 8;
                        transfer -> ux_transfer_request_index =             audio -> ux_host_class_audio_isochronous_endpoint -> ux_endpoint_descriptor.bEndpointAddress;
                        status = _ux_host_stack_transfer_request(transfer);
                        if (status != UX_SUCCESS)
                        {
                            _ux_utility_memory_free(control_buffer);
                            _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                            return(status);
                        }

                        /* Issue GET_CUR request.  */
                        status =  _ux_host_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
                        if (status != UX_SUCCESS)
                        {
                            _ux_utility_memory_free(control_buffer);
                            _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);
                            return(status);
                        }

                        /* Create a transfer request for the GET_CUR request.  */
                        transfer -> ux_transfer_request_data_pointer =      control_buffer;
                        transfer -> ux_transfer_request_requested_length =  3;
                        transfer -> ux_transfer_request_function =          UX_CLASS_AUDIO10_GET_CUR;
                        transfer -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TARGET_ENDPOINT | UX_REQUEST_TYPE_CLASS;
                        transfer -> ux_transfer_request_value =             UX_CLASS_AUDIO10_EP_SAMPLING_FREQ_CONTROL << 8;
                        transfer -> ux_transfer_request_index =             audio -> ux_host_class_audio_isochronous_endpoint -> ux_endpoint_descriptor.bEndpointAddress;
                        status = _ux_host_stack_transfer_request(transfer);
                    }

                    /* Free buffer.  */
                    _ux_utility_memory_free(control_buffer);

                    /* Check frequency.  */
                    if (status == UX_SUCCESS)
                    {
                        frequency = _ux_utility_long_get(control_buffer);
                        if (audio_sampling -> ux_host_class_audio_sampling_frequency != frequency)
                            status = UX_HOST_CLASS_AUDIO_WRONG_FREQUENCY;
                    }
                }

                /* Update packet processing parameters.  */
                audio -> ux_host_class_audio_packet_freq =
                        (device -> ux_device_speed == UX_HIGH_SPEED_DEVICE) ?
                                        8000 : 1000;
                audio -> ux_host_class_audio_packet_freq <<=
                        audio -> ux_host_class_audio_isochronous_endpoint ->
                                        ux_endpoint_descriptor.bInterval - 1;   /* Max 8000 << 15, no overflow.  */
                res_bytes = audio_sampling -> ux_host_class_audio_sampling_resolution;
                if (UX_OVERFLOW_CHECK_ADD_ULONG(res_bytes, 7))
                    status = UX_MATH_OVERFLOW;
                if (status == UX_SUCCESS)
                {
                    res_bytes += 7;
                    res_bytes >>= 3;

                    frequency = audio_sampling -> ux_host_class_audio_sampling_frequency;
                    if (UX_OVERFLOW_CHECK_MULV_ULONG(frequency, audio_sampling -> ux_host_class_audio_sampling_channels))
                        status = UX_MATH_OVERFLOW;
                }
                if (status == UX_SUCCESS)
                {
                    frequency *= audio_sampling -> ux_host_class_audio_sampling_channels;
                    if (UX_OVERFLOW_CHECK_MULV_ULONG(frequency, res_bytes))
                        status = UX_MATH_OVERFLOW;
                }
                if (status == UX_SUCCESS)
                {
                    frequency *= res_bytes;
                    audio -> ux_host_class_audio_packet_fraction =
                            frequency % audio -> ux_host_class_audio_packet_freq;
                    audio -> ux_host_class_audio_packet_size =
                            frequency / audio -> ux_host_class_audio_packet_freq;
                }

                /* Unprotect thread reentry to this instance.  */
                _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

                /* Return completion status.  */
                return(status);
            }
        }

        /* Move to next interface.  */
        interface_ptr =  interface_ptr -> ux_interface_next_interface;
    }

    /* Unprotect thread reentry to this instance.  */
    _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_NO_ALTERNATE_SETTING);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_NO_ALTERNATE_SETTING, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* We get here if we could not get the right alternate setting.  */
    return(UX_NO_ALTERNATE_SETTING);
}
