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


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio_feedback_get                 PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function obtain encoded feedback from the Audio Stream.        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    stream                                Address of audio stream       */
/*                                            instance                    */
/*    encoded_feedback                      Feedback data (3 or 4 bytes)  */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_audio_feedback_get(UX_DEVICE_CLASS_AUDIO_STREAM *stream,
                                           UCHAR *encoded_feedback)
{
#if !defined(UX_DEVICE_CLASS_AUDIO_FEEDBACK_SUPPORT)
    UX_PARAMETER_NOT_USED(stream);
    UX_PARAMETER_NOT_USED(encoded_feedback);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_SLAVE_DEVICE             *device;
UX_SLAVE_ENDPOINT           *endpoint;
UX_SLAVE_TRANSFER           *transfer;
UCHAR                       *buffer;


    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* As long as the device is in the CONFIGURED state.  */
    if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
    {

        /* Cannot proceed with command, the interface is down.  */
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }

    /* Check if endpoint is available.  */
    endpoint = stream -> ux_device_class_audio_stream_feedback;
    if (endpoint == UX_NULL)
        return(UX_ERROR);

    /* Check if endpoint direction is OK.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_OUT)
        return(UX_ERROR);

    /* Get transfer buffer.  */
    transfer = &endpoint -> ux_slave_endpoint_transfer_request;
    buffer = transfer -> ux_slave_transfer_request_data_pointer;

    /* Get packed data.  */
    *encoded_feedback ++ = *buffer ++;
    *encoded_feedback ++ = *buffer ++;
    *encoded_feedback ++ = *buffer ++;
    if (_ux_system_slave -> ux_system_slave_speed == UX_HIGH_SPEED_DEVICE)
        *encoded_feedback = *buffer;
    return(UX_SUCCESS);
#endif
}
