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
/*    _ux_host_class_audio_transfer_request               PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function submits an isochronous audio transfer request to the  */ 
/*    USBX stack.                                                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    audio                                 Pointer to audio class        */ 
/*    audio_transfer_request                Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Audio Class                                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined transfer implement, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_audio_transfer_request(UX_HOST_CLASS_AUDIO *audio,
                            UX_HOST_CLASS_AUDIO_TRANSFER_REQUEST *audio_transfer_request)
{

UX_INTERRUPT_SAVE_AREA
UINT            status;
UX_TRANSFER     *transfer_request;

    /* The transfer request is embedded in the application transfer request.  */
    transfer_request =  &audio_transfer_request -> ux_host_class_audio_transfer_request;

    /* Saved the direction, by taking the endpoint transfer direction.  */
    transfer_request -> ux_transfer_request_type =
                    audio -> ux_host_class_audio_isochronous_endpoint ->
                        ux_endpoint_transfer_request.ux_transfer_request_type;

    /* Fill the transfer request with all the required fields.  */
    transfer_request -> ux_transfer_request_endpoint =              audio -> ux_host_class_audio_isochronous_endpoint;
    transfer_request -> ux_transfer_request_data_pointer =          audio_transfer_request -> ux_host_class_audio_transfer_request_data_pointer;
    transfer_request -> ux_transfer_request_requested_length =      audio_transfer_request -> ux_host_class_audio_transfer_request_requested_length;
    transfer_request -> ux_transfer_request_packet_length =         audio_transfer_request -> ux_host_class_audio_transfer_request_packet_size;
    transfer_request -> ux_transfer_request_completion_function =   _ux_host_class_audio_transfer_request_completed;
    transfer_request -> ux_transfer_request_class_instance =        audio;
    transfer_request -> ux_transfer_request_next_transfer_request = UX_NULL; /* Add one by one.  */

    /* We memorize the application transfer request in the local transfer request.  */
    transfer_request -> ux_transfer_request_user_specific =  (VOID *) audio_transfer_request;

    UX_DISABLE

    /* Hook the audio transfer request to the chain of transfer requests in the audio instance.  */
    audio_transfer_request -> ux_host_class_audio_transfer_request_next_audio_transfer_request =
                                                audio -> ux_host_class_audio_tail_transfer_request;
    audio -> ux_host_class_audio_tail_transfer_request =  audio_transfer_request;

    /* Check if this is the first time we have had a transfer request, if so update the head as well.  */
    if (audio -> ux_host_class_audio_head_transfer_request == UX_NULL)
        audio -> ux_host_class_audio_head_transfer_request =  audio_transfer_request;

    UX_RESTORE

    /* Transfer the transfer request (queued in lower level).  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Return completion status.  */
    return(status);
}
