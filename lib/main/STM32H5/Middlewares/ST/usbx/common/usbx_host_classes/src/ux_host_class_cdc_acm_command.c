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
/**   CDC ACM Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_cdc_acm_command                      PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will send a command to the ACM device. The command    */
/*    can be one of the following :                                       */
/*    SET_CONTROL                                                         */
/*    SET_LINE                                                            */
/*    SEND_BREAK                                                          */
/*                                                                        */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    acm                                   Pointer to acm class          */
/*    command                               command value                 */
/*    value                                 value to be sent in the       */
/*                                          command request               */
/*    data_buffer                           buffer to be sent             */
/*    data_length                           length of the buffer to send  */
/*                                                                        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_host_stack_transfer_request       Process transfer request      */
/*    _ux_host_semaphore_get                Get semaphore                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Storage Class                                                       */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_cdc_acm_command(UX_HOST_CLASS_CDC_ACM *cdc_acm, ULONG command,
                                    ULONG value, UCHAR *data_buffer, ULONG data_length)
{

#if defined(UX_HOST_STANDALONE)
UX_INTERRUPT_SAVE_AREA
UX_DEVICE       *device;
#endif

UX_ENDPOINT     *control_endpoint;
UX_TRANSFER     *transfer_request;
UINT            status;
ULONG           request_direction;


    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &cdc_acm -> ux_host_class_cdc_acm_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Check the direction of the command.  */
    switch (command)
    {

    case UX_HOST_CLASS_CDC_ACM_REQ_SEND_ENCAPSULATED_COMMAND        :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SET_COMM_FEATURE                 :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_CLEAR_COMM_FEATURE               :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SET_AUX_LINE_STATE               :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SET_HOOK_STATE                   :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_PULSE_SETUP                      :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SEND_PULSE                       :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SET_PUSLE_TIME                   :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_RING_AUX_JACK                    :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SET_LINE_CODING                  :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SET_LINE_STATE                   :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SEND_BREAK                       :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SET_RINGER_PARMS                 :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SET_OPERATION_PARMS              :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_SET_LINE_PARMS                   :

        /* Direction is out */
        request_direction = UX_REQUEST_OUT;
        break;


    case UX_HOST_CLASS_CDC_ACM_REQ_GET_ENCAPSULATED_COMMAND         :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_GET_COMM_FEATURE                 :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_GET_LINE_CODING                  :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_GET_RINGER_PARMS                 :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_GET_OPERATION_PARMS              :
    /* Fall through.  */
    case UX_HOST_CLASS_CDC_ACM_REQ_GET_LINE_PARMS                   :

        /* Direction is in */
        request_direction = UX_REQUEST_IN;
        break;


    default :

        return(UX_ERROR);

    }

#if defined(UX_HOST_STANDALONE)

    /* Get device instance.  */
    device = cdc_acm -> ux_host_class_cdc_acm_device;

    /* Check device EP0 transfer lock flag.  */
    UX_DISABLE
    if (device -> ux_device_flags & UX_DEVICE_FLAG_LOCK)
    {
        UX_RESTORE
        return(UX_STATE_LOCK);
    }

    /* Lock the device for EP0 transfer.  */
    device -> ux_device_flags |= UX_DEVICE_FLAG_LOCK;
    transfer_request -> ux_transfer_request_flags |= UX_TRANSFER_FLAG_AUTO_DEVICE_UNLOCK;
    UX_RESTORE
#else

    /* Protect the control endpoint semaphore here.  It will be unprotected in the
       transfer request function.  */
    status =  _ux_host_semaphore_get(&cdc_acm -> ux_host_class_cdc_acm_device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);

    /* Check for status.  */
    if (status != UX_SUCCESS)

        /* Something went wrong. */
        return(status);
#endif

    /* Create a transfer_request for the request.  */
    transfer_request -> ux_transfer_request_data_pointer     =  data_buffer;
    transfer_request -> ux_transfer_request_requested_length =  data_length;
    transfer_request -> ux_transfer_request_function         =  command;
    transfer_request -> ux_transfer_request_type             =  request_direction | UX_REQUEST_TYPE_CLASS | UX_REQUEST_TARGET_INTERFACE;
    transfer_request -> ux_transfer_request_value            =  value;
    transfer_request -> ux_transfer_request_index            =  cdc_acm -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bInterfaceNumber;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Return completion status.  */
    return(status);
}
