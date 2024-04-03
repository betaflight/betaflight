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
/*    _ux_host_class_audio_control_request                PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function issue audio specific control request.                 */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Pointer to audio class        */
/*    streaming_control                     0 for control requests        */
/*                                          1 for streaming requests      */
/*    request_type                          bmRequestType                 */
/*    request                               bRequest                      */
/*    request_value                         wValue can be:                */
/*                                          (CS << 8)  | CN or            */
/*                                          (ICN << 8) | OCN or           */
/*                                          (MU_MIXER_CONTROL << 8) | MCN */
/*                                          or other specific value.      */
/*    spec_id                               Entity/Language ID (if used)  */
/*    parameter                             Pointer to request parameters */
/*    parameter_size                        Request parameters length     */
/*    actual_size                           Actual processed size         */
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
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/
UINT    _ux_host_class_audio_control_request(UX_HOST_CLASS_AUDIO *audio,
                        UINT streaming_control,
                        UINT request_type, UINT request,
                        UINT request_value,
                        UINT spec_id,
                        UCHAR *parameter, ULONG parameter_size, ULONG *actual_size)
{

UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
ULONG           request_index = 0;
UINT            status;


    /* Ensure the instance is valid.  */
    if (_ux_host_stack_class_instance_verify(_ux_system_host_class_audio_name, (VOID *) audio) != UX_SUCCESS)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, audio, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Check request settings.  */
    switch(request_type & UX_REQUEST_TARGET)
    {
    case UX_REQUEST_TARGET_INTERFACE:
        request_index = streaming_control ?
                        audio -> ux_host_class_audio_streaming_interface -> ux_interface_descriptor.bInterfaceNumber :
                        audio -> ux_host_class_audio_control_interface_number;
        break;
    case UX_REQUEST_TARGET_ENDPOINT:
        request_index = audio -> ux_host_class_audio_isochronous_endpoint -> ux_endpoint_descriptor.bEndpointAddress;
        break;
    default:
        return(UX_INVALID_PARAMETER);
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

    /* Create a transfer request for the specific setting.  */
    transfer_request -> ux_transfer_request_data_pointer =      parameter;
    transfer_request -> ux_transfer_request_requested_length =  parameter_size;
    transfer_request -> ux_transfer_request_function =          request;
    transfer_request -> ux_transfer_request_type =              request_type;
    transfer_request -> ux_transfer_request_value =             request_value;
    transfer_request -> ux_transfer_request_index =             request_index | (spec_id << 8);

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Save actual size.  */
    if (actual_size)
        *actual_size = transfer_request -> ux_transfer_request_actual_length;

    /* Unprotect thread reentry to this instance.  */
    _ux_host_mutex_off(&audio -> ux_host_class_audio_mutex);

    /* Return completion status.  */
    return(status);
}
