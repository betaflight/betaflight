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
/**   Device CCID Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_ccid.h"
#include "ux_device_stack.h"


#if !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_ccid_runner_thread_entry           PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the thread of the CCID command handling. It        */
/*    is waiting for the event to start command processing.               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ccid_runner                           Address of CCID runner (32b)  */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_request     Request transfer              */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX                                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_ccid_runner_thread_entry(ULONG ccid_runner)
{

UX_SLAVE_DEVICE                                     *device;
UX_DEVICE_CLASS_CCID                                *ccid;
UX_DEVICE_CLASS_CCID_PARAMETER                      *parameter;
UX_DEVICE_CLASS_CCID_HANDLE                         *handles;
UX_DEVICE_CLASS_CCID_HANDLE                         handle;
UX_DEVICE_CLASS_CCID_RUNNER                         *runner;
UX_DEVICE_CLASS_CCID_SLOT                           *slot;
UX_DEVICE_CLASS_CCID_MESSAGE_HEADER                 *cmd;
UX_DEVICE_CLASS_CCID_COMMAND_SETT                   *cmd_sett;
UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS_HEADER   *rsp;
UX_DEVICE_CLASS_CCID_MESSAGES                       messages;
ULONG                                               event_mask, flags;
ULONG                                               cmd_checks;
UINT                                                status;

    /* Cast properly the ccid runner.  */
    UX_THREAD_EXTENSION_PTR_GET(runner, UX_DEVICE_CLASS_CCID_RUNNER, ccid_runner)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get CCID instance.  */
    ccid = runner -> ux_device_class_ccid_runner_ccid;

    /* Get CCID parameter.  */
    parameter = &ccid -> ux_device_class_ccid_parameter;

    /* Get event mask to wait.  */
    event_mask = 1u << runner -> ux_device_class_ccid_runner_id;

    /* This thread runs forever but can be suspended or resumed by the user application.  */
    status = UX_SUCCESS;
    while(1)
    {

        /* Check device state.  */
        if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
        {
            _ux_utility_thread_suspend(&runner -> ux_device_class_ccid_runner_thread);
            continue;
        }

        /* Wait signal.  */
        status = _ux_utility_event_flags_get(&ccid -> ux_device_class_ccid_events,
                            event_mask, UX_OR_CLEAR, &flags, UX_WAIT_FOREVER);
        if (status != UX_SUCCESS)
        {
            _ux_utility_thread_suspend(&runner -> ux_device_class_ccid_runner_thread);
            continue;
        }

        /* We have a command to process.  */
        cmd = (UX_DEVICE_CLASS_CCID_MESSAGE_HEADER *)
                                runner -> ux_device_class_ccid_runner_command;
        rsp = (UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS_HEADER *)
                                runner -> ux_device_class_ccid_runner_response;

        /* Slot to handle.  */
        slot  = ccid -> ux_device_class_ccid_slots;
        slot += runner -> ux_device_class_ccid_runner_slot;

        /* Command settings.  */
        cmd_sett  = (UX_DEVICE_CLASS_CCID_COMMAND_SETT *)_ux_device_class_ccid_command_sett;
        cmd_sett += runner -> ux_device_class_ccid_runner_command_index;

        /* Message to pass to application.  */
        messages.ux_device_class_ccid_messages_pc_to_rdr = (UCHAR *)cmd;
        messages.ux_device_class_ccid_messages_rdr_to_pc = (UCHAR *)rsp;

        /* Internal checks.  */
        cmd_checks  = (ULONG)cmd_sett -> ux_device_class_ccid_command_sett_flags;
        cmd_checks &= (ULONG)slot -> ux_device_class_ccid_slot_flags;

        /* Check hardware error!  */
        if (cmd_checks & UX_DEVICE_CLASS_CCID_FLAG_HW_ERROR)
        {

            /* Response: (1,1,HW_ERROR).  */
            rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(1, 1);
            rsp -> bError  = UX_DEVICE_CLASS_CCID_HW_ERROR;
        }

        /* Check auto sequencing!  */
        else if (cmd_checks & UX_DEVICE_CLASS_CCID_FLAG_AUTO_SEQUENCING)
        {

            /* Response: (1,1,BUSY_WITH_AUTO_SEQUENCE).  */
            rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(1, 1);
            rsp -> bError  = UX_DEVICE_CLASS_CCID_BUSY_WITH_AUTO_SEQUENCE;
        }

        /* Process command, application can fill status.  */
        else
        {

            handles = (UX_DEVICE_CLASS_CCID_HANDLE *)parameter -> ux_device_class_ccid_handles;
            handle = handles[(INT)cmd_sett -> ux_device_class_ccid_command_sett_handle_index];

            /* Initialize response length based on type.  */
            switch(rsp -> bMessageType)
            {
            case UX_DEVICE_CLASS_CCID_RDR_TO_PC_DATA_RATE_AND_CLOCK_FREQ:

                /* Length fixed to 10+8.  */
                messages.ux_device_class_ccid_messages_rdr_to_pc_length = 18;
                UX_DEVICE_CLASS_CCID_MESSAGE_LENGTH_SET(rsp, 8);
                break;

            case UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS:

                /* Length fixed to 10.  */
                messages.ux_device_class_ccid_messages_rdr_to_pc_length =
                                    UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH;
                break;

            default:

                /* There is possible data, set length to max transfer length.  */
                messages.ux_device_class_ccid_messages_rdr_to_pc_length =
                        parameter -> ux_device_class_ccid_max_transfer_length;
                break;
            }

            /* Invoke application callback.  */
            status = handle(cmd -> bSlot, &messages);

            /* Save application status updates.  */

            /* Save bmICCStatus.  */
            slot -> ux_device_class_ccid_slot_icc_status = rsp->bStatus &
                                    UX_DEVICE_CLASS_CCID_SLOT_STATUS_ICC_MASK;

            /* Save bClockStatus.  */
            slot -> ux_device_class_ccid_slot_clock_status = rsp->bClockStatus;
        }

        /* If failed (aborted), no response.  */
        if (status != UX_SUCCESS)
            continue;
        if (slot -> ux_device_class_ccid_slot_aborting)
            continue;

        /* Send response.  */
        _ux_device_class_ccid_response(ccid, (UCHAR *)rsp,
                    messages.ux_device_class_ccid_messages_rdr_to_pc_length);

        _ux_device_mutex_on(&ccid -> ux_device_class_ccid_mutex);

        /* Free runner.  */
        runner -> ux_device_class_ccid_runner_slot = -1;
        ccid -> ux_device_class_ccid_n_busy --;

        /* Clear slot busy.  */
        slot -> ux_device_class_ccid_slot_runner = -1;

        _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);
    }
}
#endif
