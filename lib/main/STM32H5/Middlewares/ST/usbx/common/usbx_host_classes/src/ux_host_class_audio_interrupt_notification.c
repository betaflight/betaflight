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
/*    _ux_host_class_audio_interrupt_notification         PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function receives a completion callback on a interrupt         */
/*    transfer request.                                                   */
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
/*    (ux_host_class_audio_interrupt_callback)                            */
/*                                          App notification callback     */
/*    _ux_host_stack_transfer_request       Issue the transfer request    */
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
VOID  _ux_host_class_audio_interrupt_notification(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_AUDIO_AC *ac;

    /* Check status.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
        return;

    /* Get AC instance.  */
    ac = (UX_HOST_CLASS_AUDIO_AC *)transfer_request -> ux_transfer_request_class_instance;

    /* Invoke application callback.  */
    if (ac -> ux_host_class_audio_interrupt_callback)
    {
        ac -> ux_host_class_audio_interrupt_callback(ac,
                        transfer_request -> ux_transfer_request_data_pointer,
                        transfer_request -> ux_transfer_request_actual_length,
                        ac -> ux_host_class_audio_interrupt_callback_arg);
    }

    /* Issue another request again.  */
    _ux_host_stack_transfer_request(transfer_request);
}
