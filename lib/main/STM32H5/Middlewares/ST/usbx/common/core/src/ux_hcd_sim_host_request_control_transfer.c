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
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_request_control_transfer           PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function performs a control transfer from a transfer request. */ 
/*     The USB control transfer is in 3 phases (setup, data, status).     */
/*     This function will chain all phases of the control sequence before */
/*     setting the sim_host endpoint as a candidate for transfer.         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_sim_host                          Pointer to host controller    */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_regular_td_obtain    Obtain regular TD             */ 
/*    _ux_host_stack_transfer_request_abort Abort transfer request        */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*    _ux_utility_semaphore_get             Get semaphore                 */ 
/*    _ux_utility_short_put                 Write 16-bit value            */ 
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
/*                                            prefixed UX to MS_TO_TICK,  */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_request_control_transfer(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT             *endpoint;
UCHAR                   *setup_request;
UX_HCD_SIM_HOST_ED      *ed;
UX_HCD_SIM_HOST_TD      *setup_td;
UX_HCD_SIM_HOST_TD      *chain_td;
UX_HCD_SIM_HOST_TD      *data_td;
UX_HCD_SIM_HOST_TD      *tail_td;
UX_HCD_SIM_HOST_TD      *status_td;
UX_HCD_SIM_HOST_TD      *start_data_td;
UX_HCD_SIM_HOST_TD      *next_data_td;
ULONG                   transfer_request_payload_length;
ULONG                   control_packet_payload_length;
UCHAR                   *data_pointer;
#if !defined(UX_HOST_STANDALONE)
UINT                    status;
#endif


    /* Get the pointer to the endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

    /* Build the SETUP packet (phase 1 of the control transfer).  */
    setup_request =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_SETUP_SIZE);
    if (setup_request == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    *setup_request =                            (UCHAR)transfer_request -> ux_transfer_request_function;
    *(setup_request + UX_SETUP_REQUEST_TYPE) =  (UCHAR)transfer_request -> ux_transfer_request_type;
    *(setup_request + UX_SETUP_REQUEST) =       (UCHAR)transfer_request -> ux_transfer_request_function;
    _ux_utility_short_put(setup_request + UX_SETUP_VALUE, (USHORT)transfer_request -> ux_transfer_request_value);
    _ux_utility_short_put(setup_request + UX_SETUP_INDEX, (USHORT)transfer_request -> ux_transfer_request_index);
    _ux_utility_short_put(setup_request + UX_SETUP_LENGTH, (USHORT) transfer_request -> ux_transfer_request_requested_length);

    /* Use the TD pointer by ed -> tail for our setup TD and chain from this one on.  */
    setup_td =  ed -> ux_sim_host_ed_tail_td;
    setup_td -> ux_sim_host_td_buffer =  setup_request;
    setup_td -> ux_sim_host_td_length =  UX_SETUP_SIZE;
    chain_td =  setup_td;

    /* Attach the endpoint and transfer_request to the TD.  */
    setup_td -> ux_sim_host_td_transfer_request =  transfer_request;
    setup_td -> ux_sim_host_td_ed =  ed;

    /* Setup is OUT.  */
    setup_td -> ux_sim_host_td_direction =  UX_HCD_SIM_HOST_TD_OUT;

    /* Mark TD toggle as being DATA0.  */
    setup_td -> ux_sim_host_td_toggle =  0;

    /* Mark the TD with the SETUP phase.  */
    setup_td -> ux_sim_host_td_status |=  UX_HCD_SIM_HOST_TD_SETUP_PHASE;

    /* Check if there is a data phase, if not jump to status phase.  */
    data_td =  UX_NULL;    
    start_data_td =  UX_NULL;

    /* Use local variables to manipulate data pointer and length.  */
    transfer_request_payload_length =  transfer_request -> ux_transfer_request_requested_length;
    data_pointer =  transfer_request -> ux_transfer_request_data_pointer;

    /* Data starts with DATA1. For the data phase, we use the ED to contain the toggle.  */
    ed -> ux_sim_host_ed_toggle =  1;

    /* The Control data payload may be split into several smaller blocks.  */
    while (transfer_request_payload_length != 0)
    {

        /* Get a new TD to hook this payload.  */
        data_td =  _ux_hcd_sim_host_regular_td_obtain(hcd_sim_host);
        if (data_td == UX_NULL)
        {

            /* Free the Setup packet resources.  */
            _ux_utility_memory_free(setup_request);

            /* If there was already a TD chain in progress, free it.  */
            if (start_data_td != UX_NULL)
            {

                data_td =  start_data_td;
                while (data_td)
                {

                    next_data_td =  data_td -> ux_sim_host_td_next_td;
                    data_td -> ux_sim_host_td_status =  UX_UNUSED;
                    data_td =  next_data_td;
                }
            }

            return(UX_NO_TD_AVAILABLE);
        }

        /* Check the current payload requirement for the max size.  */
        if (transfer_request_payload_length > UX_HCD_SIM_HOST_MAX_PAYLOAD)

            control_packet_payload_length =  UX_HCD_SIM_HOST_MAX_PAYLOAD;
        else

            control_packet_payload_length =  transfer_request_payload_length;

        /* Store the beginning of the buffer address in the TD.  */
        data_td -> ux_sim_host_td_buffer =  data_pointer;

        /* Update the length of the transfer for this TD.  */
        data_td -> ux_sim_host_td_length =  control_packet_payload_length;

        /* Attach the endpoint and transfer request to the TD.  */
        data_td -> ux_sim_host_td_transfer_request =  transfer_request;
        data_td -> ux_sim_host_td_ed =  ed;

        /* Adjust the data payload length and the data payload pointer.  */
        transfer_request_payload_length -=  control_packet_payload_length;
        data_pointer +=  control_packet_payload_length;

        /* The direction of the transaction is set in the TD.  */
        if ((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)

            data_td -> ux_sim_host_td_direction =  UX_HCD_SIM_HOST_TD_IN;
        else

            data_td -> ux_sim_host_td_direction =  UX_HCD_SIM_HOST_TD_OUT;

        /* Mark the TD with the DATA phase.  */
        data_td -> ux_sim_host_td_status |=  UX_HCD_SIM_HOST_TD_DATA_PHASE;

        /* The Toggle value is in the ED.  */
        data_td -> ux_sim_host_td_toggle =  UX_HCD_SIM_HOST_TD_TOGGLE_FROM_ED;

        /* The first obtained TD in the chain has to be remembered.  */
        if (start_data_td == UX_NULL)
            start_data_td =  data_td;

        /* Attach this new TD to the previous one.  */                                
        chain_td -> ux_sim_host_td_next_td =  data_td;
        chain_td -> ux_sim_host_td_next_td_transfer_request =  data_td;
        chain_td =  data_td;
    }

    /* Now, program the status phase.  */
    status_td =  _ux_hcd_sim_host_regular_td_obtain(hcd_sim_host);

    if (status_td == UX_NULL)
    {

        _ux_utility_memory_free(setup_request);
        if (data_td != UX_NULL)
        {

            data_td =  start_data_td;
            while (data_td)
            {

                next_data_td =  data_td -> ux_sim_host_td_next_td;
                data_td -> ux_sim_host_td_status =  UX_UNUSED;
                data_td =  next_data_td;
            }
        }

        return(UX_NO_TD_AVAILABLE);
    }

    /* Attach the endpoint and transfer request to the TD.  */
    status_td -> ux_sim_host_td_transfer_request =  transfer_request;
    status_td -> ux_sim_host_td_ed =  ed;

    /* Mark the TD with the STATUS phase.  */
    status_td -> ux_sim_host_td_status |=  UX_HCD_SIM_HOST_TD_STATUS_PHASE;

    /* The direction of the status phase is IN if data phase is OUT and
       vice versa.  */
    if ((transfer_request -> ux_transfer_request_type&UX_REQUEST_DIRECTION) == UX_REQUEST_IN)

        status_td -> ux_sim_host_td_direction =  UX_HCD_SIM_HOST_TD_OUT;
    else

        status_td -> ux_sim_host_td_direction =  UX_HCD_SIM_HOST_TD_IN;

    /* No data payload for the status phase.  */
    status_td -> ux_sim_host_td_buffer =  0;
    status_td -> ux_sim_host_td_length =  0;

    /* Status Phase toggle is ALWAYS 1.  */
    status_td -> ux_sim_host_td_toggle =  1;

    /* Hook the status phase to the previous TD.  */
    chain_td -> ux_sim_host_td_next_td =  status_td;

    /* Since we have consumed out tail TD for the setup packet, we must get another one 
       and hook it to the ED's tail.  */
    tail_td =  _ux_hcd_sim_host_regular_td_obtain(hcd_sim_host);
    if (tail_td == UX_NULL)
    {

        _ux_utility_memory_free(setup_request);
        if (data_td != UX_NULL)
            data_td -> ux_sim_host_td_status =  UX_UNUSED;
        status_td -> ux_sim_host_td_status =  UX_UNUSED;
        return(UX_NO_TD_AVAILABLE);
    }

    /* Hook the new TD to the status TD.  */
    status_td -> ux_sim_host_td_next_td =  tail_td;
            
    /* At this stage, the Head and Tail in the ED are still the same and
       the host simulator controller will skip this ED until we have hooked the new
       tail TD.  */
    ed -> ux_sim_host_ed_tail_td =  tail_td;

    /* Now we can tell the scheduler to wake up.  */
    hcd_sim_host -> ux_hcd_sim_host_queue_empty =  UX_FALSE;

#if defined(UX_HOST_STANDALONE)
    /* Transfer started in background, fine.  */
    return(UX_SUCCESS);
#else
    /* Wait for the completion of the transfer request.  */
    status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_CONTROL_TRANSFER_TIMEOUT));

    /* If the semaphore did not succeed we probably have a time out.  */
    if (status != UX_SUCCESS)
    {

        /* All transfers pending need to abort. There may have been a partial transfer.  */
        _ux_host_stack_transfer_request_abort(transfer_request);
        
        /* There was an error, return to the caller.  */
        transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_TIMEOUT;
        
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_TRANSFER_TIMEOUT);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_TIMEOUT, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)
        
    }            

    /* Free the resources.  */
    _ux_utility_memory_free(setup_request);

    /* Return completion to caller.  */
    return(transfer_request -> ux_transfer_request_completion_code);           
#endif
}
