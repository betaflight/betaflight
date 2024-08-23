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
/*    _ux_device_class_ccid_thread_entry                  PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the thread of the CCID bulk out endpoint. It       */
/*    is waiting for the host to send message on the bulk out endpoint to */
/*    the device.                                                         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ccid_inst                             Address of ccid class         */
/*                                            container (32-bit)          */
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
VOID  _ux_device_class_ccid_thread_entry(ULONG ccid_inst)
{

UX_SLAVE_DEVICE                                     *device;
UX_DEVICE_CLASS_CCID                                *ccid;
UX_DEVICE_CLASS_CCID_SLOT                           *slot;
UX_DEVICE_CLASS_CCID_RUNNER                         *runner = UX_NULL;
UX_DEVICE_CLASS_CCID_PARAMETER                      *parameter;
UX_DEVICE_CLASS_CCID_MESSAGES                       messages;
UX_SLAVE_ENDPOINT                                   *endpoint;
UX_SLAVE_TRANSFER                                   *transfer_cmd;
UX_DEVICE_CLASS_CCID_MESSAGE_HEADER                 *cmd;
UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS_HEADER   *rsp;
UX_DEVICE_CLASS_CCID_COMMAND_SETT                   *cmd_sett;
CHAR                                                cmd_index;
UX_DEVICE_CLASS_CCID_HANDLE                         *handles;
INT                                                 i;
UINT                                                status;

    /* Cast properly the ccid instance.  */
    UX_THREAD_EXTENSION_PTR_GET(ccid, UX_DEVICE_CLASS_CCID, ccid_inst)

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* This thread runs forever but can be suspended or resumed by the user application.  */
    status = UX_SUCCESS;
    while(1)
    {

        /* Error cases.  */
        if (status != UX_SUCCESS)
        {

            /* We need to suspend ourselves. We will be resumed by the
               application if needed.  */
            _ux_utility_thread_suspend(&ccid -> ux_device_class_ccid_thread);
        }

        status = UX_ERROR;

        /* Check device state.  */
        if (device -> ux_slave_device_state != UX_DEVICE_CONFIGURED)
            continue;

        /* Check endpoint.  */
        endpoint = ccid -> ux_device_class_ccid_endpoint_out;
        if (endpoint == UX_NULL)
            continue;
        transfer_cmd = &endpoint -> ux_slave_endpoint_transfer_request;
        endpoint = ccid -> ux_device_class_ccid_endpoint_in;
        if (endpoint == UX_NULL)
            continue;

        /* Send the request to the device controller.  */
        parameter = &ccid -> ux_device_class_ccid_parameter;
        status = _ux_device_stack_transfer_request(transfer_cmd,
                        parameter -> ux_device_class_ccid_max_transfer_length,
                        parameter -> ux_device_class_ccid_max_transfer_length);

        /* Access to CCID command message header.  */
        cmd = (UX_DEVICE_CLASS_CCID_MESSAGE_HEADER *)
                        transfer_cmd -> ux_slave_transfer_request_data_pointer;

        /* Check the completion code and message length. */
        if ((status != UX_SUCCESS) ||
            (transfer_cmd -> ux_slave_transfer_request_completion_code != UX_SUCCESS) ||
            (transfer_cmd -> ux_slave_transfer_request_actual_length <
                UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH) ||
            (transfer_cmd -> ux_slave_transfer_request_actual_length >
                parameter -> ux_device_class_ccid_max_transfer_length))
        {

            /* Re-check device state and try transfer again.  */
            status = UX_SUCCESS;
            continue;
        }

        /* Get command setting.  */
        cmd_sett = (UX_DEVICE_CLASS_CCID_COMMAND_SETT *)_ux_device_class_ccid_command_sett;
        for (cmd_index = 0; cmd_index < UX_DEVICE_CLASS_CCID_N_COMMANDS;)
        {
            if (cmd -> bMessageType ==
                cmd_sett -> ux_device_class_ccid_command_sett_command_type)
                break;

            /* Next command setting.  */
            cmd_sett ++;
            cmd_index ++;
        }
        handles = (UX_DEVICE_CLASS_CCID_HANDLE *)parameter -> ux_device_class_ccid_handles;

        /* Lock global status resources.  */
        _ux_device_mutex_on(&ccid -> ux_device_class_ccid_mutex);

        /* Initialize response.  */
        rsp = (UX_DEVICE_CLASS_CCID_RDR_TO_PC_SLOT_STATUS_HEADER *)
                                            ccid -> ux_device_class_ccid_header;
        _ux_utility_memory_set(rsp, 0, UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH); /* Use case of memset is verified. */
        rsp -> bMessageType = cmd_sett -> ux_device_class_ccid_command_sett_response_type;
        rsp -> bSlot        = cmd -> bSlot;
        rsp -> bSeq         = cmd -> bSeq;

        /* Check command support (0,1,0).  */
        if (rsp -> bMessageType == 0 ||
            handles[(INT)cmd_sett -> ux_device_class_ccid_command_sett_handle_index] == UX_NULL)
        {

            /* Response: command not supported (0,1,0).  */
            rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(0, 1);
            _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);
            _ux_device_class_ccid_response(ccid, (UCHAR *)rsp, UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH);
            continue;
        }

        /* check Slot exist (2,1,5).  */
        if (cmd -> bSlot >= parameter -> ux_device_class_ccid_max_n_slots)
        {

            /* Response: Slot not exist.  */
            rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(2, 1);
            rsp -> bError  = 5;
            _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);
            _ux_device_class_ccid_response(ccid, (UCHAR *)rsp, UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH);
            continue;
        }

        /* Get slot instance for later usage.  */
        slot = &ccid -> ux_device_class_ccid_slots[cmd -> bSlot];

        /* Initialize response status from slot status.  */
        rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(
                            slot -> ux_device_class_ccid_slot_icc_status, 0);

        /* Initialize response clock status.  */
        if (cmd_sett -> ux_device_class_ccid_command_sett_response_type == 0x81)
            rsp -> bClockStatus = slot -> ux_device_class_ccid_slot_clock_status;

        /* Abort command
           - return slot status(OK) anyway
           - clear aborting status
           Aborting
           - return slot status(ABORTED)
           Other command (except SetDataRateAndClockFrequency)
           - Check busy  */

        /* Abort command is handled differently.  */
        if (cmd -> bMessageType != UX_DEVICE_CLASS_CCID_PC_TO_RDR_ABORT ||
            !slot -> ux_device_class_ccid_slot_aborting)
        {

            /* Check if slot is idle.  */
            if ((signed char)slot -> ux_device_class_ccid_slot_runner < 0)
            {

                /* Slot is idle, check if free runner available.  */
                if (ccid -> ux_device_class_ccid_n_busy <
                    parameter -> ux_device_class_ccid_max_n_busy_slots)
                {

                    /* Get a free runner for the command.  */
                    runner = ccid -> ux_device_class_ccid_runners;
                    for(i = 0;
                        i < parameter -> ux_device_class_ccid_max_n_busy_slots;
                        i ++)
                    {

                        /* Check if runner is free.  */
                        if ((signed char)runner -> ux_device_class_ccid_runner_slot < 0)
                            break;

                        /* Check next runner.  */
                        runner ++;
                    }

                    /* It's not possible no runner found here, just execute runner.  */

                    /* Runner is busy now.  */
                    runner -> ux_device_class_ccid_runner_slot = (CHAR)cmd->bSlot;
                    runner -> ux_device_class_ccid_runner_command_index = cmd_index;
                    ccid -> ux_device_class_ccid_n_busy ++;
                    slot -> ux_device_class_ccid_slot_runner = (CHAR)i;

                    /* Create a copy of command and response header.  */
                    _ux_utility_memory_copy(runner -> ux_device_class_ccid_runner_command,
                                            cmd,
                                            transfer_cmd -> ux_slave_transfer_request_actual_length); /* Use case of memcpy is verified. */
                    _ux_utility_memory_copy(runner -> ux_device_class_ccid_runner_response,
                                            rsp, UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH); /* Use case of memcpy is verified. */

                    /* Pre-process of command done.  */
                    _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);

                    /* Signal event to runner thread.  */
                    _ux_utility_thread_resume(&runner -> ux_device_class_ccid_runner_thread);
                    _ux_device_event_flags_set(&ccid -> ux_device_class_ccid_events,
                                                1u << i, UX_OR);
                    continue;
                }

                /* We are here if there is no runner available.  */
                /* It's busy!  */
            }

            /* Response: Slot Status(busy).  */
            rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(
                             slot -> ux_device_class_ccid_slot_icc_status, 1);
            rsp -> bError = UX_DEVICE_CLASS_CCID_CMD_SLOT_BUSY;
            _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);
            _ux_device_class_ccid_response(ccid, (UCHAR *)rsp, UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH);
            continue;
        }

        /* We are here when we see Abort command, or aborting.
            - Abort command : slot status (ok/fail)
            - Aborting : slot status (CMD_ABORTED)
         */

        /* Abort command.  */
        if (cmd -> bMessageType == UX_DEVICE_CLASS_CCID_PC_TO_RDR_ABORT)
        {

            /* Check sequence.  */
            if (cmd -> bSeq != slot -> ux_device_class_ccid_slot_aborting_seq)
            {

                /* Response: sequence error.  */
                rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(
                                 slot -> ux_device_class_ccid_slot_icc_status, 1);
                rsp -> bError  = 6;
            }
            else
            {

                /* Aborting.  */
                if (slot -> ux_device_class_ccid_slot_aborting)
                {

                    /* Call abort handle.  */
                    messages.ux_device_class_ccid_messages_pc_to_rdr = (VOID *)cmd;
                    messages.ux_device_class_ccid_messages_rdr_to_pc = (VOID *)rsp;
                    messages.ux_device_class_ccid_messages_rdr_to_pc_length = 0;
                    parameter -> ux_device_class_ccid_handles ->
                            ux_device_class_ccid_handles_abort(cmd -> bSlot, &messages);

                    /* Status(OK)  */
                    rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(
                                    slot -> ux_device_class_ccid_slot_icc_status, 0);
                }
                else
                {

                    /* Status(CMD_NOT_ABORTED)?  */
                    rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(
                                    slot -> ux_device_class_ccid_slot_icc_status, 1);
                    rsp -> bError  = UX_DEVICE_CLASS_CCID_CMD_SLOT_BUSY;
                }
            }

            /* Free runner.  */
            if ((signed char)slot -> ux_device_class_ccid_slot_runner >= 0)
            {
                runner = ccid -> ux_device_class_ccid_runners + slot -> ux_device_class_ccid_slot_runner;

                runner -> ux_device_class_ccid_runner_slot = -1;

                ccid -> ux_device_class_ccid_n_busy --;

                /* Clear slot busy and aborting.  */
                slot -> ux_device_class_ccid_slot_runner = -1;
                slot -> ux_device_class_ccid_slot_aborting = UX_FALSE;
            }

            _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);

            /* Send response any way.  */
            _ux_device_class_ccid_response(ccid, (UCHAR *)rsp, UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH);
            continue;
        }

        /* Aborting.  */

        /* Response: Slot Status(aborted).  */
        rsp -> bStatus = UX_DEVICE_CLASS_CCID_SLOT_STATUS(
                            slot -> ux_device_class_ccid_slot_icc_status, 1);
        rsp -> bError = UX_DEVICE_CLASS_CCID_CMD_ABORTED;

        _ux_device_mutex_off(&ccid -> ux_device_class_ccid_mutex);

        _ux_device_class_ccid_response(ccid, (UCHAR *)rsp, UX_DEVICE_CLASS_CCID_MESSAGE_HEADER_LENGTH);
    }
}
#endif
