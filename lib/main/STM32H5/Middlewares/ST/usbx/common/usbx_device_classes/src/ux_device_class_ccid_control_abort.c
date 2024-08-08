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
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   Device CCID Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_ccid.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_ccid_control_abort                 PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function handles control request Abort of the USB CCID device. */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ccid                                  Pointer to ccid instance      */
/*    slot                                  Slot to abort                 */
/*    seq                                   Sequence number of message    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Source Code                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_ccid_control_abort(UX_DEVICE_CLASS_CCID *ccid, ULONG slot, ULONG seq)
{

UX_DEVICE_CLASS_CCID_PARAMETER              *parameter;
UX_DEVICE_CLASS_CCID_HANDLES                *handles;
UX_DEVICE_CLASS_CCID_SLOT                   *ccid_slot;
UX_SLAVE_TRANSFER                           *transfer_in, *transfer_out;
UX_DEVICE_CLASS_CCID_MESSAGE_HEADER         *msg;

    /* Get parameter.  */
    parameter = &ccid -> ux_device_class_ccid_parameter;
    handles = parameter -> ux_device_class_ccid_handles;

    /* Check if slot is available.  */
    if (slot >= parameter -> ux_device_class_ccid_max_n_slots)
        return(UX_INVALID_PARAMETER);

    /* Check if command is supported.  */
    if (handles -> ux_device_class_ccid_handles_abort == UX_NULL)
        return(UX_FUNCTION_NOT_SUPPORTED);

    /* Get slot to process.  */
    ccid_slot = &ccid -> ux_device_class_ccid_slots[slot];

    /* Check if message header received.  */
    transfer_out = &ccid -> ux_device_class_ccid_endpoint_out ->
                                            ux_slave_endpoint_transfer_request;
    if (transfer_out -> ux_slave_transfer_request_actual_length <
        UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH)
    {

        /* Message header not available, cancel current bulk OUT transfer.  */
        _ux_device_stack_transfer_abort(transfer_out, UX_ABORTED);
    }
    else
    {

        /* Message header available, check message.  */
        msg = (UX_DEVICE_CLASS_CCID_MESSAGE_HEADER *)
                        transfer_out -> ux_slave_transfer_request_data_pointer;

        /* Check if abort applies to the slot.  */
        if (msg -> bSlot == (UCHAR)slot)
        {

            /* Yes, aborting current message, cancel current bulk OUT transfer.  */
            _ux_device_stack_transfer_abort(transfer_out, UX_ABORTED);
        }
        else
        {

            /* Aborting another slot.  */

            /* Check slot busy.  */
            if ((signed char)ccid_slot -> ux_device_class_ccid_slot_runner >= 0)
            {

                /* Possible sending something, abort IN transfer.  */
                transfer_in = &ccid -> ux_device_class_ccid_endpoint_in ->
                                                ux_slave_endpoint_transfer_request;
                _ux_device_stack_transfer_abort(transfer_in, UX_ABORTED);
            }
        }
    }

    /* Set aborting state.  */
    ccid_slot -> ux_device_class_ccid_slot_aborting = UX_TRUE;
    ccid_slot -> ux_device_class_ccid_slot_aborting_seq = (UCHAR)seq;

    /* Invoke control abort to let application do something.  */
    handles -> ux_device_class_ccid_handles_abort(slot, UX_NULL);
    return(UX_SUCCESS);
}
