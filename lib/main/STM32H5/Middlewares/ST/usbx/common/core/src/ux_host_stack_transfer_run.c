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
/**   Host Stack                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_stack.h"


#if defined(UX_HOST_STANDALONE)

#define UX_HOST_STACK_TRANSFER_LIST_IS_NULL               0
#define UX_HOST_STACK_TRANSFER_NOT_IN_LIST                1
#define UX_HOST_STACK_TRANSFER_AT_LIST_HEAD               2
#define UX_HOST_STACK_TRANSFER_IN_LIST                    3
static inline UINT _ux_host_stack_transfer_locate(UX_TRANSFER *transfer, UX_TRANSFER **previous);
static inline void _ux_host_stack_transfer_retire(UX_TRANSFER *transfer);


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_stack_transfer_run                         PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function performs a USB transaction. On entry the transfer     */
/*    request gives the endpoint pipe selected for this transaction and   */
/*    the parameters associated with the transfer (data payload, length   */
/*    of transaction)                                                     */
/*                                                                        */
/*    Note after transfer is done, it's in error or idle state. To ensure */
/*    transfer is ready for next round, use UX_TRANSFER_STATE_RESET().    */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    transfer_request                      Transfer request structure    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    State machine Status to check                                       */
/*    UX_STATE_NEXT                         Transfer done, to next state  */
/*    UX_STATE_EXIT                         Abnormal, to reset state      */
/*    (others)                              Keep running, waiting         */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    HCD Entry Function                                                  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_transfer_run(UX_TRANSFER *transfer_request)
{

UX_INTERRUPT_SAVE_AREA

UX_ENDPOINT     *endpoint;
UX_DEVICE       *device;
UX_HCD          *hcd;
ULONG           endpoint_type;
UINT            status;


    /* Get the endpoint container from the transfer_request */
    endpoint =  transfer_request -> ux_transfer_request_endpoint;

    /* Sanity check.  */
    if (endpoint == UX_NULL)
        return(UX_STATE_EXIT);

    /* Get the endpoint type.  */
    endpoint_type = endpoint -> ux_endpoint_descriptor.bmAttributes;
    endpoint_type &= UX_MASK_ENDPOINT_TYPE;

    /* Get the device container from the endpoint.  */
    device =  endpoint -> ux_endpoint_device;

    /* Sanity check.  */
    if (device == UX_NULL)
        return(UX_STATE_EXIT);

    /* Ensure we are not preempted by the enum thread while we check the device
       state and set the transfer status.  */
    UX_DISABLE

    /* Check if it's default endpoint 0.  */
    if (endpoint -> ux_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION)
    {

        /* It's not endpoint 0, check device state.  */
        if (device -> ux_device_state != UX_DEVICE_ATTACHED &&
            device -> ux_device_state != UX_DEVICE_ADDRESSED &&
            device -> ux_device_state != UX_DEVICE_CONFIGURED)
        {
            transfer_request -> ux_transfer_request_completion_code = UX_TRANSFER_NOT_READY;
            transfer_request -> ux_transfer_request_state = UX_STATE_EXIT;
            _ux_host_stack_transfer_retire(transfer_request);

            /* Restore interrupts.  */
            UX_RESTORE
            return(UX_STATE_EXIT);
        }
    }

    /* With the device we have the pointer to the HCD.  */
    hcd = UX_DEVICE_HCD_GET(device);

    /* Process states.  */
    switch(transfer_request -> ux_transfer_request_state)
    {
    case UX_STATE_RESET:

        /* Initialize request fields.  */
        transfer_request -> ux_transfer_request_actual_length = 0;
        transfer_request -> ux_transfer_request_status = UX_TRANSFER_STATUS_NOT_PENDING;
        transfer_request -> ux_transfer_request_completion_code = UX_TRANSFER_STATUS_PENDING;
        transfer_request -> ux_transfer_request_state = UX_STATE_WAIT;
        transfer_request -> ux_transfer_request_time_start = _ux_utility_time_get();

        /* Add request to system pending request list. Note request may be kept
           if transfer callback is used.  */
        if (_ux_host_stack_transfer_locate(transfer_request, UX_NULL) <
            UX_HOST_STACK_TRANSFER_AT_LIST_HEAD)
        {
            transfer_request -> ux_transfer_request_next_pending =
                            _ux_system_host -> ux_system_host_pending_transfers;
            _ux_system_host -> ux_system_host_pending_transfers = transfer_request;
        }

        /* Do immediate HCD call to start transfer in background.  */
        /* Fall through.  */
    case UX_STATE_WAIT:
        UX_RESTORE

        {

            /* Send the command to the controller.  */
            status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_TRANSFER_RUN, transfer_request);
        }

        /* Any error or end: simplify to idle.  */
        if (status < UX_STATE_WAIT)
        {
            UX_DISABLE
            UX_TRANSFER_STATE_IDLE(transfer_request);
            _ux_host_stack_transfer_retire(transfer_request);
            UX_RESTORE
            return(status);
        }

        /* Timeout check.  */
        if (transfer_request -> ux_transfer_request_timeout_value != UX_WAIT_FOREVER)
        {
            if (transfer_request -> ux_transfer_request_timeout_value <
                _ux_utility_time_elapsed(transfer_request -> ux_transfer_request_time_start,
                                         _ux_utility_time_get()))
            {

                /* All transfers pending need to abort. There may have been a partial transfer.  */
                _ux_host_stack_transfer_request_abort(transfer_request);
                
                /* Set the completion code.  */
                transfer_request -> ux_transfer_request_completion_code = UX_TRANSFER_TIMEOUT;
        
                /* There was an error: simplify to idle.  */
                UX_DISABLE
                UX_TRANSFER_STATE_IDLE(transfer_request);
                _ux_host_stack_transfer_retire(transfer_request);
                UX_RESTORE
                return(UX_STATE_ERROR);
            }
        }

        /* And return the status.  */
        return(status);

    case UX_STATE_EXIT:
        UX_RESTORE
        return(UX_STATE_EXIT);
    case UX_STATE_IDLE:
        UX_RESTORE
        return(UX_STATE_IDLE);

    default: /* Error case, return EXIT.  */
        break;
    }

    /* Error case, return EXIT.  */
    transfer_request -> ux_transfer_request_state = UX_STATE_RESET;
    _ux_host_stack_transfer_retire(transfer_request);
    UX_RESTORE
    return(UX_STATE_EXIT);
}
static inline UINT _ux_host_stack_transfer_locate(UX_TRANSFER *transfer, UX_TRANSFER **previous)
{
UX_TRANSFER     *prev;

    /* Case 0: pending list is NULL.  */
    if (_ux_system_host -> ux_system_host_pending_transfers == UX_NULL)
        return(UX_HOST_STACK_TRANSFER_LIST_IS_NULL);

    /* Case 1: it's list head.  */
    if (_ux_system_host -> ux_system_host_pending_transfers == transfer)
        return(UX_HOST_STACK_TRANSFER_AT_LIST_HEAD);

    /* Case 2: scan list.  */
    prev = _ux_system_host -> ux_system_host_pending_transfers;
    do
    {
        if (prev -> ux_transfer_request_next_pending == transfer)
        {
            if (previous)
                *previous = prev;
            return(UX_HOST_STACK_TRANSFER_IN_LIST);
        }
        prev = prev -> ux_transfer_request_next_pending;
    } while (prev);

    /* Case 3: not found.  */
    return(UX_HOST_STACK_TRANSFER_NOT_IN_LIST);
}
static inline void _ux_host_stack_transfer_retire(UX_TRANSFER *transfer)
{
ULONG               flags;
UX_TRANSFER         *previous;

    /* Locate the request in pending list.  */
    switch(_ux_host_stack_transfer_locate(transfer, &previous))
    {
    case UX_HOST_STACK_TRANSFER_AT_LIST_HEAD:

        /* Unlink from pending transfer list head.  */
        _ux_system_host -> ux_system_host_pending_transfers =
                                transfer -> ux_transfer_request_next_pending;
        break;
    case UX_HOST_STACK_TRANSFER_IN_LIST:

        /* Unlink from pending transfer list.  */
        previous -> ux_transfer_request_next_pending =
                                transfer -> ux_transfer_request_next_pending;
        break;

    default:
        break;
    }

    /* Process transfer flags.  */
    flags = transfer -> ux_transfer_request_flags;
    transfer -> ux_transfer_request_flags &=
                ~(UX_TRANSFER_FLAG_LOCK | UX_TRANSFER_FLAG_AUTO_DEVICE_UNLOCK);
    if (flags & UX_TRANSFER_FLAG_AUTO_DEVICE_UNLOCK)
    {
        transfer -> ux_transfer_request_endpoint ->
                ux_endpoint_device -> ux_device_flags &= ~UX_DEVICE_FLAG_LOCK;
    }
}
#endif
