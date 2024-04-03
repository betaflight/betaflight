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
/**   Device Audio Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_audio.h"
#include "ux_device_class_audio20.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio20_control_process            PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function manages the based sent by the host on the control     */
/*    endpoints with a CLASS or VENDOR SPECIFIC type.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Address of audio class        */
/*                                            instance                    */
/*    transfer                              Address of transfer request   */
/*                                            instance                    */
/*    group                                 Request process data          */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_short_get                 Get 2-byte value from buffer  */
/*    _ux_utility_short_put                 Put 2-byte value to buffer    */
/*    _ux_utility_long_get                  Get 4-byte value from buffer  */
/*    _ux_utility_long_put                  Put 4-byte value to buffer    */
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_device_stack_transfer_request     Issue a transfer request      */
/*    _ux_device_stack_endpoint_stall       Endpoint stall                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX                                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added volume RES support,   */
/*                                            resulting in version 6.1.6  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            allowed answer length only  */
/*                                            when requesting range,      */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added support of multiple   */
/*                                            sampling frequencies,       */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_audio20_control_process(UX_DEVICE_CLASS_AUDIO *audio,
                                              UX_SLAVE_TRANSFER *transfer,
                                              UX_DEVICE_CLASS_AUDIO20_CONTROL_GROUP *group)
{

UX_SLAVE_ENDPOINT                   *endpoint;
UX_DEVICE_CLASS_AUDIO20_CONTROL     *control;
UCHAR                               request;
UCHAR                               request_type;
UCHAR                               unit_id;
UCHAR                               control_selector;
UCHAR                               channel_number;
ULONG                               request_length;
ULONG                               data_length;
ULONG                               i;
ULONG                               n_sub, pos, min, max, res, freq;


    /* Get instances.  */
    endpoint = &audio -> ux_device_class_audio_device -> ux_slave_device_control_endpoint;
    transfer = &endpoint -> ux_slave_endpoint_transfer_request;

    /* Extract all necessary fields of the request.  */
    request          = *(transfer -> ux_slave_transfer_request_setup + UX_DEVICE_CLASS_AUDIO_REQUEST_REQUEST);
    request_type     = *(transfer -> ux_slave_transfer_request_setup + UX_DEVICE_CLASS_AUDIO_REQUEST_REQUEST_TYPE);
    unit_id          = *(transfer -> ux_slave_transfer_request_setup + UX_DEVICE_CLASS_AUDIO_REQUEST_ENEITY_ID);
    control_selector = *(transfer -> ux_slave_transfer_request_setup + UX_DEVICE_CLASS_AUDIO_REQUEST_CONTROL_SELECTOR);
    channel_number   = *(transfer -> ux_slave_transfer_request_setup + UX_DEVICE_CLASS_AUDIO_REQUEST_CHANNEL_NUMBER);
    request_length   = _ux_utility_short_get(transfer -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

    for (i = 0; i < group -> ux_device_class_audio20_control_group_controls_nb; i ++)
    {
        control = &group -> ux_device_class_audio20_control_group_controls[i];

        /* Reset change map.  */
        control -> ux_device_class_audio20_control_changed = 0;

        /* Is this request a clock unit request?  */
        if (unit_id == control -> ux_device_class_audio20_control_cs_id)
        {

            /* Clock Source request.
             * We only support Sampling Frequency Control here.
             * The Sampling Frequency Control must support the CUR and RANGE(MIN, MAX, RES) attributes.
             */

            /* Sampling frequency control, SET request.  */
            if ((request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_OUT &&
                (control_selector == UX_DEVICE_CLASS_AUDIO20_CS_SAM_FREQ_CONTROL))
            {
                switch(request)
                {
                case UX_DEVICE_CLASS_AUDIO20_CUR:

                    /* Check request parameter.  */
                    if (request_length != 4)
                        break;

                    /* Check if multiple frequency supported.  */
                    if (control -> ux_device_class_audio20_control_sampling_frequency != 0)
                        break;

                    /* Sanity check.  */
                    UX_ASSERT(control -> ux_device_class_audio20_control_sampling_frequency_range != UX_NULL);

                    /* Get wNumSubRanges.  */
                    n_sub = _ux_utility_short_get(control -> ux_device_class_audio20_control_sampling_frequency_range);

                    /* Get first RES.  */
                    res = _ux_utility_long_get(control -> ux_device_class_audio20_control_sampling_frequency_range + 2 + 8);

                    /* Check if it's fixed single frequency.  */
                    if (n_sub <= 1 && res == 0)
                        break;

                    /* Get frequency to set.  */
                    freq = _ux_utility_long_get(transfer -> ux_slave_transfer_request_data_pointer);

                    /* Check if frequency to set is inside range.  */
                    for (pos = 2; pos < (2 + n_sub * 12); pos += 12)
                    {
                        min = _ux_utility_long_get(control -> ux_device_class_audio20_control_sampling_frequency_range + pos);
                        max = _ux_utility_long_get(control -> ux_device_class_audio20_control_sampling_frequency_range + pos + 4);
                        if (freq >= min && freq <= max)
                        {

                            /* SET_CUR is accepted.  */
                            if (control -> ux_device_class_audio20_control_sampling_frequency_cur != freq)
                            {
                                control -> ux_device_class_audio20_control_sampling_frequency_cur = freq;
                                control -> ux_device_class_audio20_control_changed = UX_DEVICE_CLASS_AUDIO20_CONTROL_FREQUENCY_CHANGED;
                            }
                            return(UX_SUCCESS);
                        }
                    }
                    break;

                default:
                    break;
                }
            }

            /* We just support sampling frequency control, GET request.  */
            if ((request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN &&
                (control_selector == UX_DEVICE_CLASS_AUDIO20_CS_SAM_FREQ_CONTROL))
            {

                switch(request)
                {
                case UX_DEVICE_CLASS_AUDIO20_CUR:

                    /* Check request parameter.  */
                    if (request_length < 4)
                        break;

                    /* Send sampling frequency.  */
                    if (control -> ux_device_class_audio20_control_sampling_frequency)
                        _ux_utility_long_put(transfer -> ux_slave_transfer_request_data_pointer, control -> ux_device_class_audio20_control_sampling_frequency);
                    else
                        _ux_utility_long_put(transfer -> ux_slave_transfer_request_data_pointer, control -> ux_device_class_audio20_control_sampling_frequency_cur);
                    _ux_device_stack_transfer_request(transfer, 4, request_length);
                    return(UX_SUCCESS);

                case UX_DEVICE_CLASS_AUDIO20_RANGE:

                    /* Check request parameter.  */
                    if (request_length < 2)
                        break;

                    if (control -> ux_device_class_audio20_control_sampling_frequency == 0)
                    {

                        /* Send range parameters, RANGE is customized.  */
                        UX_ASSERT(control -> ux_device_class_audio20_control_sampling_frequency_range != UX_NULL);

                        /* Get wNumSubRanges.  */
                        n_sub = _ux_utility_short_get(control -> ux_device_class_audio20_control_sampling_frequency_range);
                        UX_ASSERT(n_sub > 0);

                        /* Calculate length, n_sub is 16-bit width, result not overflows ULONG.  */
                        data_length = 2 + n_sub * 12;
                        UX_ASSERT(data_length <= UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH);

                        /* Copy data.  */
                        data_length = UX_MIN(data_length, request_length);
                        _ux_utility_memory_copy(transfer -> ux_slave_transfer_request_data_pointer,
                                control -> ux_device_class_audio20_control_sampling_frequency_range,
                                data_length); /* Use case of memcpy is verified. */
                    }
                    else
                    {

                        /* Send range parameters.
                         * We only support one here (from extension data).
                         * wNumSubRanges : 1
                         * dMIN          : sampling frequency
                         * dMAX          : sampling frequency
                         * dRES          : 1
                         */
                        _ux_utility_short_put(transfer -> ux_slave_transfer_request_data_pointer, 1);
                        _ux_utility_long_put(transfer -> ux_slave_transfer_request_data_pointer + 2, control -> ux_device_class_audio20_control_sampling_frequency);
                        _ux_utility_long_put(transfer -> ux_slave_transfer_request_data_pointer + 6, control -> ux_device_class_audio20_control_sampling_frequency);
                        _ux_utility_long_put(transfer -> ux_slave_transfer_request_data_pointer + 10, 0);
                        data_length = UX_MIN(14, request_length);
                    }

                    /* Send data.  */
                    _ux_device_stack_transfer_request(transfer, data_length, request_length);
                    return(UX_SUCCESS);

                default:
                    break;
                }
            }

            /* We are here when there is error on handling CU, break.  */
            break;
        } /* if (unit_id == control -> ux_device_class_audio20_control_cs_id) */

        /* Is this request a feature unit request?  */
        if (unit_id == control -> ux_device_class_audio20_control_fu_id)
        {

            /* Feature Unit request.
             * We only support master channel, mute and volume here.
             * Mute have only the CUR.
             * Volume must support CUR and RANGE(MIN,MAX,RES).
             */

            /* Check control selector.  */
            switch(control_selector)
            {
            case UX_DEVICE_CLASS_AUDIO20_FU_MUTE_CONTROL:

                /* Check channel number, we support master channel only.  */
                if (channel_number != 0)
                    break;

                /* Handle request.  */
                switch(request)
                {
                case UX_DEVICE_CLASS_AUDIO20_CUR:


                    /* Check buffer size.  */
                    if (request_length < 1)
                        break;

                    /* GET_CUR.  */
                    if ((request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)
                    {

                        /* Send mute.  */
                        transfer -> ux_slave_transfer_request_data_pointer[0] = (UCHAR)control -> ux_device_class_audio20_control_mute[0];
                        _ux_device_stack_transfer_request(transfer, 1, request_length);
                    }
                    else if (control -> ux_device_class_audio20_control_mute[0] != transfer -> ux_slave_transfer_request_data_pointer[0])
                    {

                        /* Update mute.  */
                        control -> ux_device_class_audio20_control_changed = UX_DEVICE_CLASS_AUDIO20_CONTROL_MUTE_CHANGED;
                        control -> ux_device_class_audio20_control_mute[0] = transfer -> ux_slave_transfer_request_data_pointer[0];
                    }
                    /* Done success.  */
                    return(UX_SUCCESS);

                default:
                    break;
                }
                break;

            case UX_DEVICE_CLASS_AUDIO20_FU_VOLUME_CONTROL:

                /* Check channel number, we support master channel only.  */
                if (channel_number != 0)
                    break;

                /* Handle request.  */
                switch(request)
                {
                case UX_DEVICE_CLASS_AUDIO20_CUR:

                    /* Check buffer size.  */
                    if (request_length < 2)
                        break;

                    /* GET_CUR.  */
                    if ((request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)
                    {

                        /* Send volume.  */
                        _ux_utility_short_put(transfer -> ux_slave_transfer_request_data_pointer, (USHORT)control -> ux_device_class_audio20_control_volume[0]);
                        _ux_device_stack_transfer_request(transfer, 2, request_length);
                    }
                    else if (control -> ux_device_class_audio20_control_volume[0] != (SHORT)_ux_utility_short_get(transfer -> ux_slave_transfer_request_data_pointer))
                    {

                        /* Update volume.  */
                        control -> ux_device_class_audio20_control_changed = UX_DEVICE_CLASS_AUDIO20_CONTROL_VOLUME_CHANGED;
                        control -> ux_device_class_audio20_control_volume[0] = (SHORT)_ux_utility_short_get(transfer -> ux_slave_transfer_request_data_pointer);
                    }

                    /* Done success.  */
                    return(UX_SUCCESS);

                case UX_DEVICE_CLASS_AUDIO20_RANGE:

                    /* Only support GET_CUR.  */
                    if ((request_type & UX_REQUEST_DIRECTION) != UX_REQUEST_IN)
                        break;

                    /* Check buffer size.  */
                    if (request_length < 8)
                        break;

                    /* Send wNumSubRanges,wMIN,wMAX,wRES.  */
                    _ux_utility_short_put(transfer -> ux_slave_transfer_request_data_pointer + 0, 1);
                    _ux_utility_short_put(transfer -> ux_slave_transfer_request_data_pointer + 2, (USHORT)control -> ux_device_class_audio20_control_volume_min[0]);
                    _ux_utility_short_put(transfer -> ux_slave_transfer_request_data_pointer + 4, (USHORT)control -> ux_device_class_audio20_control_volume_max[0]);
                    _ux_utility_short_put(transfer -> ux_slave_transfer_request_data_pointer + 6, (USHORT)UX_MAX(1, control -> ux_device_class_audio20_control_volume_res[0]));
                    _ux_device_stack_transfer_request(transfer, 8, request_length);

                    /* Done success.  */
                    return(UX_SUCCESS);

                default:
                    break;
                }
                break;

            default:
                break;
            }

            /* We are here when there is error on handling FU, break.  */
            break;
        } /* if (unit_id == control -> ux_device_class_audio20_control_fu_id) */
    }

    /* Request or parameter not supported.  */
    _ux_device_stack_endpoint_stall(endpoint);
    return(UX_ERROR);
}
