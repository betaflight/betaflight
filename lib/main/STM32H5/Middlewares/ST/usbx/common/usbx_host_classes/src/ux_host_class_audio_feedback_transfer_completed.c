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


#if defined(UX_HOST_CLASS_AUDIO_FEEDBACK_SUPPORT)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_audio_feedback_transfer_completed    PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function receives a completion call back on a isoch transfer   */
/*    request.                                                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    transfer_request                      Pointer to transfer request   */
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
/*    HCD                                                                 */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  07-29-2022     Chaoqiong Xiao           Initial Version 6.1.12        */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_audio_feedback_transfer_completed(UX_TRANSFER *transfer_request)
{
UX_HOST_CLASS_AUDIO         *audio;
UX_ENDPOINT                 *endpoint;
ULONG                       endpoint_dir;

    /* Check status.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
        return;

    /* Sync with feedback buffer.  */
    audio = (UX_HOST_CLASS_AUDIO *) transfer_request -> ux_transfer_request_class_instance;
    endpoint = transfer_request -> ux_transfer_request_endpoint;
    endpoint_dir = endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION;
    if (endpoint_dir == UX_ENDPOINT_IN)
    {
        _ux_utility_memory_copy(audio -> ux_host_class_audio_feedback_buffer,
                                transfer_request -> ux_transfer_request_data_pointer,
                                sizeof(audio -> ux_host_class_audio_feedback_buffer)); /* Use case of memcpy is verified. */
    }

    /* Issue another request again.  */
    _ux_host_stack_transfer_request(transfer_request);
}
#endif
