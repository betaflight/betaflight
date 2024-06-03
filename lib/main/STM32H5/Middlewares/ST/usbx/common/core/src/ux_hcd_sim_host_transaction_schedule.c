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
/**   Host Simulator Controller Driver                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_hcd_sim_host.h"
#include "ux_dcd_sim_slave.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_hcd_sim_host_transaction_schedule               PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     This function bridges a transaction from the host to the slave     */
/*     simulation controller.                                             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_sim_host                          Pointer to host controller    */
/*    ed                                    Pointer to ED                 */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_transfer_request_completion_function)                           */
/*                                          Completion function           */
/*    _ux_device_stack_control_request_process                            */
/*                                          Process request               */
/*    _ux_utility_memory_copy               Copy memory block             */
/*    _ux_utility_semaphore_put             Semaphore put                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Host Simulator Controller Driver                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed control OUT transfer, */
/*                                            supported bi-dir-endpoints, */
/*                                            resulting in version 6.1.6  */
/*  10-15-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            improved check for tests,   */
/*                                            added error trap case,      */
/*                                            resulting in version 6.1.9  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            cleared transfer status     */
/*                                            before semaphore wakeup to  */
/*                                            avoid a race condition,     */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined device ZLP flow,    */
/*                                            adjusted control request    */
/*                                            data length handling,       */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_transaction_schedule(UX_HCD_SIM_HOST *hcd_sim_host, UX_HCD_SIM_HOST_ED *ed)
{

UX_DCD_SIM_SLAVE        *dcd_sim_slave;
UX_HCD_SIM_HOST_TD      *td;
UX_HCD_SIM_HOST_TD      *head_td;
UX_HCD_SIM_HOST_TD      *tail_td;
UX_HCD_SIM_HOST_TD      *data_td;
UX_ENDPOINT             *endpoint;
UX_SLAVE_ENDPOINT       *slave_endpoint;
UX_DCD_SIM_SLAVE_ED     *slave_ed;
ULONG                   slave_transfer_remaining;
UCHAR                   wake_host;
UCHAR                   wake_slave;
ULONG                   transaction_length;
ULONG                   td_length;
UX_SLAVE_TRANSFER       *slave_transfer_request;
UX_TRANSFER             *transfer_request;
ULONG                   endpoint_index;
UX_SLAVE_DCD            *dcd;

    UX_PARAMETER_NOT_USED(hcd_sim_host);

    /* Get the pointer to the DCD portion of the simulator.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Check the state of the controller if OPERATIONAL .  */
    if (dcd -> ux_slave_dcd_status !=  UX_DCD_STATUS_OPERATIONAL)
        return(UX_ERROR);

    /* Get the pointer to the candidate TD on the host.  */
    td =  ed -> ux_sim_host_ed_head_td;

    /* Get the pointer to the endpoint.  */
    endpoint =  ed -> ux_sim_host_ed_endpoint;

    /* Get the pointer to the transfer_request attached with this TD.  */
    transfer_request =  td -> ux_sim_host_td_transfer_request;

    /* Get the index of the endpoint from the host.  */
    endpoint_index = endpoint -> ux_endpoint_descriptor.bEndpointAddress & ~(ULONG)UX_ENDPOINT_DIRECTION;

    /* Get the address of the device controller.  */
    dcd_sim_slave =  (UX_DCD_SIM_SLAVE *) dcd -> ux_slave_dcd_controller_hardware;

    /* Get the endpoint as seen from the device side.  */
#ifdef UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT
    slave_ed = ((endpoint -> ux_endpoint_descriptor.bEndpointAddress == 0) ?
            &dcd_sim_slave -> ux_dcd_sim_slave_ed[0] :
            ((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) ?
                &dcd_sim_slave -> ux_dcd_sim_slave_ed_in[endpoint_index] :
                &dcd_sim_slave -> ux_dcd_sim_slave_ed[endpoint_index]));
#else
    slave_ed =  &dcd_sim_slave -> ux_dcd_sim_slave_ed[endpoint_index];
#endif

    /* Is this ED used?  */
    if ((slave_ed -> ux_sim_slave_ed_status & UX_DCD_SIM_SLAVE_ED_STATUS_USED) == 0)
        return(UX_ERROR);

    /* Is this ED ready for transaction or stalled ?  */
    if ((slave_ed -> ux_sim_slave_ed_status & (UX_DCD_SIM_SLAVE_ED_STATUS_TRANSFER | UX_DCD_SIM_SLAVE_ED_STATUS_STALLED)) == 0)
        return(UX_ERROR);

    /* Get the logical endpoint from the physical endpoint.  */
    slave_endpoint =  slave_ed -> ux_sim_slave_ed_endpoint;

    /* Get the pointer to the transfer request.  */
    slave_transfer_request =  &slave_endpoint -> ux_slave_endpoint_transfer_request;

    /* Check the phase for this transfer, if this is the SETUP phase, treatment is different.  Explanation of how 
       control transfers are handled in the simulator: if the data phase is OUT, we handle it immediately, meaning we 
       send all the data to the device and remove the STATUS TD in the same scheduler call. If the data phase is IN, we 
       only take out the SETUP TD and handle the data phase like any other non-control transactions (i.e. the scheduler 
       calls us again with the DATA TDs).  */
    if (td -> ux_sim_host_td_status &  UX_HCD_SIM_HOST_TD_SETUP_PHASE)
    {

        /* For control transfer, stall is for protocol error and it's cleared any time when SETUP is received */
        slave_ed -> ux_sim_slave_ed_status &= ~(ULONG)UX_DCD_SIM_SLAVE_ED_STATUS_STALLED;

        /* Validate the length to the setup transaction buffer.  */
        UX_ASSERT(td -> ux_sim_host_td_length == 8);

        /* Reset actual data length (not including SETUP received) so far.  */
        slave_transfer_request -> ux_slave_transfer_request_actual_length =  0;

        /* Move the buffer from the host TD to the device TD.  */
        _ux_utility_memory_copy(slave_transfer_request -> ux_slave_transfer_request_setup,
                                td -> ux_sim_host_td_buffer,
                                td -> ux_sim_host_td_length); /* Use case of memcpy is verified. */

#if defined(UX_HOST_STANDALONE)

        /* The setup buffer is allocated, release it since it's used.  */
        _ux_utility_memory_free(td -> ux_sim_host_td_buffer);
        td -> ux_sim_host_td_buffer = UX_NULL;
#endif

        /* The setup phase never fails. We acknowledge the transfer code here by taking the TD out of the endpoint.  */
        ed -> ux_sim_host_ed_head_td =  td -> ux_sim_host_td_next_td;

        /* Free the TD that was used here.  */
        td -> ux_sim_host_td_status =  UX_UNUSED;

        /* Check if the transaction is OUT from the host and there is data payload.  */
        if (((*slave_transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN) == 0) &&
            (*(slave_transfer_request -> ux_slave_transfer_request_setup + 6) != 0 ||
            *(slave_transfer_request -> ux_slave_transfer_request_setup + 7) != 0))
        {

            /* This is the case where there is a data payload OUT from host to device.
               the data needs to be copied into the device buffer first before invoking the control
               dispatcher.  */

            /* Get the length we expect from the SETUP packet (target the entire available control buffer).  */
            slave_transfer_request -> ux_slave_transfer_request_requested_length = _ux_utility_short_get(slave_transfer_request -> ux_slave_transfer_request_setup + 6);

            /* Avoid buffer overflow.  */
            if (slave_transfer_request -> ux_slave_transfer_request_requested_length > UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH)
            {
                /* Error trap.  */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_DCD, UX_TRANSFER_BUFFER_OVERFLOW);
                slave_transfer_request -> ux_slave_transfer_request_requested_length = UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH;
            }

            /* And reprogram the current buffer address to the beginning of the buffer.  */
            slave_transfer_request -> ux_slave_transfer_request_current_data_pointer =  slave_transfer_request -> ux_slave_transfer_request_data_pointer;

            /* Get the pointer to the first data TD. this is the TD right after the SETUP one.  */
            data_td =  td -> ux_sim_host_td_next_td;

            /* Get the data length we expect. */
            transaction_length = slave_transfer_request -> ux_slave_transfer_request_requested_length;

            /* It may have taken multiple TDs to send all the data. */
            while (transaction_length != 0)
            {

                /* Do a sanity check. TD must be data.  */
                if (data_td -> ux_sim_host_td_status & UX_HCD_SIM_HOST_TD_DATA_PHASE)
                {

                    /* Copy the amount of data in the td into the slave transaction buffer.  */
                    td_length = UX_MIN(data_td -> ux_sim_host_td_length, transaction_length);
                    _ux_utility_memory_copy(slave_transfer_request -> ux_slave_transfer_request_current_data_pointer,
                                            data_td -> ux_sim_host_td_buffer,
                                            td_length); /* Use case of memcpy is verified. */

                    /* Add to the actual payload length.  */
                    slave_transfer_request -> ux_slave_transfer_request_actual_length += td_length;

                    /* Update the host transfer's actual length. */
                    transfer_request -> ux_transfer_request_actual_length +=  transaction_length;

                    /* Decrement the total length.  */
                    transaction_length -= td_length;

                    /* Update buffer pointer.  */
                    slave_transfer_request -> ux_slave_transfer_request_current_data_pointer += td_length;

                    /* Update the td in the head.  */
                    ed -> ux_sim_host_ed_head_td =  data_td;

                    /* Get the pointer to the next data TD. */
                    data_td =  data_td -> ux_sim_host_td_next_td;

                    /* Free the TD that was used here.  */
                    ed -> ux_sim_host_ed_head_td -> ux_sim_host_td_status =  UX_UNUSED;
                }
            }

            /* Make the head TD point to the STATUS TD.  */
            ed -> ux_sim_host_ed_head_td =  ed -> ux_sim_host_ed_head_td -> ux_sim_host_td_next_td;
        }

        /* Is there no hub?  */
        if (dcd_sim_slave -> ux_dcd_sim_slave_dcd_control_request_process_hub == UX_NULL)
        {

            /* There's no hub to worry about. This control transfer is for the
               device itself.  */

            /* Pass the transfer to the regular device stack.  */
            _ux_device_stack_control_request_process(slave_transfer_request);
        }
        else
        {

            /* There is a hub. We need to call the correct Control Transfer dispatcher. 
               If the device is a hub and this transfer is for one of the devices on the 
               hub, then we must invoke a separate Control Transfer dispatcher besides 
               the regular device stack's. This is because the current device stack doesn't 
               handle control transfers to device's other than itself.  */

            /* Is this meant for the device itself?  */
            if (/* If the device isn't ADDRESSED yet, then the address may be invalid since we don't
                   clear it upon disconnection/reconnection. So we assume that if the device is RESET
                   or ATTACHED, the control transfer is meant for the device itself.  */
                (_ux_system_slave->ux_system_slave_device.ux_slave_device_state == UX_DEVICE_RESET ||
                 _ux_system_slave->ux_system_slave_device.ux_slave_device_state == UX_DEVICE_ATTACHED) ||

                /* If we get to this check, then the device has been ADDRESSED and we can compare addresses.  */
                endpoint -> ux_endpoint_device -> ux_device_address == dcd -> ux_slave_dcd_device_address)
            {

                /* Yes, this control transfer is meant for the device itself.  */

                /* Pass the transfer to the regular device stack.  */
                _ux_device_stack_control_request_process(slave_transfer_request);
            }
            else
            {

                /* No, this control transfer is meant for a device on the hub. */

                /* Pass the transfer to the callback.  */
                dcd_sim_slave -> ux_dcd_sim_slave_dcd_control_request_process_hub(slave_transfer_request);
            }
        }

        /* Check if the transaction is OUT from the host.  */
        if ((*slave_transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN) == 0)
        {

            /* Check if there is a problem with the endpoint (maybe stalled).  */
            if (slave_ed -> ux_sim_slave_ed_status & UX_DCD_SIM_SLAVE_ED_STATUS_STALLED)
            {

                /* Protocol error, stall the transaction.  */
                transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STALLED;
                if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                    transfer_request -> ux_transfer_request_completion_function(transfer_request);

                /* Error trap. */
                _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_TRANSFER_STALLED);

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_STALLED, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)
            }
            else
                /* No error in simulation.  */
                transfer_request -> ux_transfer_request_completion_code =  UX_SUCCESS;

            /* In this case the transfer is completed! We take out the status TD.  */
            td = ed -> ux_sim_host_ed_head_td;

            /* Adjust the ED.  */
            ed -> ux_sim_host_ed_head_td =  td -> ux_sim_host_td_next_td;

            /* Free the TD that was used here.  */
            td -> ux_sim_host_td_status =  UX_UNUSED;

            /* Then, we wake up the host.  */
            _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);
        }
    }
    else
    {

        /* Check if there is a problem with the endpoint (maybe stalled).  */
        if (slave_ed -> ux_sim_slave_ed_status & UX_DCD_SIM_SLAVE_ED_STATUS_STALLED)
        {

            /* Stall the transaction.  */
            transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STALLED;
            if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                transfer_request -> ux_transfer_request_completion_function(transfer_request);

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_TRANSFER_STALLED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_STALLED, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* Wake up the host side.  */
            _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);

            /* Clean up this ED.  */
            head_td =  ed -> ux_sim_host_ed_head_td;
            tail_td =  ed -> ux_sim_host_ed_tail_td;

            /* Free all TDs attached to the ED.  */
            while (head_td != tail_td)
            {

                /* Mark the current head TD as free. */
                head_td -> ux_sim_host_td_status =  UX_UNUSED;

                /* Update the head TD with the next TD.  */
                ed -> ux_sim_host_ed_head_td =  head_td -> ux_sim_host_td_next_td;

                /* Now the new head_td is the next TD in the chain.  */
                head_td =  ed -> ux_sim_host_ed_head_td;
            }
        }
        else
        {

            slave_transfer_remaining = 0;

            /* If the device tries to send a NULL packet, we don't reset the actual length to 0. */
            if (slave_transfer_request -> ux_slave_transfer_request_requested_length != 0)
                slave_transfer_remaining = slave_transfer_request -> ux_slave_transfer_request_requested_length - slave_transfer_request -> ux_slave_transfer_request_actual_length;

            /* Get the transaction length to be transferred.  It could be a ZLP condition.  */
            if (slave_transfer_remaining <= td -> ux_sim_host_td_length)
                transaction_length =  slave_transfer_remaining;
            else
                transaction_length =  td -> ux_sim_host_td_length;

            if (transaction_length)
            {
                if (td -> ux_sim_host_td_direction == UX_HCD_SIM_HOST_TD_OUT)

                    /* Send the requested host data to the device.  */
                    _ux_utility_memory_copy(slave_transfer_request -> ux_slave_transfer_request_current_data_pointer,
                                            td -> ux_sim_host_td_buffer,
                                            transaction_length); /* Use case of memcpy is verified. */

                else

                    /* Send the requested host data to the device.  */
                    _ux_utility_memory_copy(td -> ux_sim_host_td_buffer,
                                            slave_transfer_request -> ux_slave_transfer_request_current_data_pointer,
                                            transaction_length); /* Use case of memcpy is verified. */
            }

            /* Update buffers.  */
            td -> ux_sim_host_td_buffer +=  transaction_length;
            slave_transfer_request -> ux_slave_transfer_request_current_data_pointer +=  transaction_length;

            /* Update actual length values.  */
            td -> ux_sim_host_td_actual_length +=  transaction_length;
            transfer_request -> ux_transfer_request_actual_length +=  transaction_length;
            slave_transfer_request -> ux_slave_transfer_request_actual_length +=  transaction_length;

            /* Update requested length values.  */
            td -> ux_sim_host_td_length -=  transaction_length;

            /* Are we done with this TD (It's possible for the TD to expect more data; for example, the slave
               sent/received a smaller amount)?  */
            if (td -> ux_sim_host_td_length == 0)
            {

                /* Free the TD that was used here.  */
                td -> ux_sim_host_td_status =  UX_UNUSED;

                /* Adjust the ED.  */
                ed -> ux_sim_host_ed_head_td =  td -> ux_sim_host_td_next_td;
            }

            /* Reset wake booleans. */
            wake_host =  UX_FALSE;
            wake_slave =  UX_FALSE;

            /* Does the slave have absolutely no more data to send? */
            if (slave_endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize == 0) /* Special for tests (avoid DIV0/MOD0) */
            {

                /* This happens only if device descriptor bMaxPacketSize0 is zero, assume it's OK for the first control
                   requests to let host check the descriptor.
                   If wMaxPacketSize is zero, host reject the device and transfer never started to get here.  */
                wake_host =  UX_TRUE;
                wake_slave =  UX_TRUE;
            }
            else if ((transaction_length == 0) ||
                     (transaction_length % slave_endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize))
            {

                /* Host got ZLP or short packet.  */
                wake_host =  UX_TRUE;
                wake_slave =  UX_TRUE;
            }
            else
            {

                /* Is the host's transfer completed?  */
                if (transfer_request -> ux_transfer_request_actual_length == transfer_request -> ux_transfer_request_requested_length)
                    wake_host = UX_TRUE;

                /* Is the slaves's transfer completed?  */
                if (slave_transfer_request -> ux_slave_transfer_request_actual_length ==
                    slave_transfer_request -> ux_slave_transfer_request_requested_length)
                {
                    if (slave_transfer_request -> ux_slave_transfer_request_requested_length == 0 ||
                        slave_transfer_request -> ux_slave_transfer_request_force_zlp == 0)
                        wake_slave = UX_TRUE;
                    else
                        slave_transfer_request -> ux_slave_transfer_request_force_zlp = 0;
                }
            }

            if (wake_slave == UX_TRUE)
            {

                /* Set the completion code to no error.  */
                slave_transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

                /* Set the transfer status to COMPLETED.  */
                slave_transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_COMPLETED;

                /* Is this not the control endpoint? */
                if (slave_ed -> ux_sim_slave_ed_index != 0)
                {

                    /* Clear pending flag.  */
                    slave_ed -> ux_sim_slave_ed_status &= ~(ULONG)UX_DCD_SIM_SLAVE_ED_STATUS_TRANSFER;

                    /* Set done flag.  */
                    slave_ed -> ux_sim_slave_ed_status |= UX_DCD_SIM_SLAVE_ED_STATUS_DONE;

                    /* Wake up the slave side.  */
                    _ux_device_semaphore_put(&slave_transfer_request -> ux_slave_transfer_request_semaphore);
                }
            }

            if (wake_host == UX_TRUE)
            {

                /* If the slave has less data to send than the host wants to receive, then there may still be 
                   TDs left to free. Note that this should only happen for IN transactions, since the only way
                   an OUT transfer can complete is if all the data was sent i.e. all the TDs were sent and freed. */
                if (ed -> ux_sim_host_ed_head_td != ed -> ux_sim_host_ed_tail_td)
                {

                    /* Free all TDs associated with this transfer. Note that if this is a control transfer (which
                       means it must be IN), then this also gets rid of the STATUS phase, which is okay. Also note
                       that assumes that there is only one transfer occurring on this endpoint; if there were
                       multiple, then we'd erase that one's TDs as well. Luckily, with the way USBX is designed,
                       there should never be multiple transfers occurring simultaneously on a single endpoint
                       (tldr; data pointer for transfers is shared).  */
                    head_td =  ed -> ux_sim_host_ed_head_td;
                    while (head_td != ed -> ux_sim_host_ed_tail_td)
                    {

                        /* Free the TD that was used here.  */
                        head_td -> ux_sim_host_td_status =  UX_UNUSED;

                        /* Move to the next. */
                        head_td =  head_td -> ux_sim_host_td_next_td;
                    }

                    /* Update the head and tail TD. */
                    ed -> ux_sim_host_ed_head_td =  head_td;
                    ed -> ux_sim_host_ed_tail_td =  head_td;
                }

                /* Set the completion code to no error.  */
                transfer_request -> ux_transfer_request_completion_code =  UX_SUCCESS;

                /* Set the transfer status to COMPLETED.  */
                transfer_request -> ux_transfer_request_status =  UX_TRANSFER_STATUS_COMPLETED;

                /* Is there a callback on the host? */
                if (transfer_request -> ux_transfer_request_completion_function != UX_NULL)
                    transfer_request -> ux_transfer_request_completion_function(transfer_request);

                /* Wake up the host side.  */
                _ux_host_semaphore_put(&transfer_request -> ux_transfer_request_semaphore);
            }
        }
    }

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

