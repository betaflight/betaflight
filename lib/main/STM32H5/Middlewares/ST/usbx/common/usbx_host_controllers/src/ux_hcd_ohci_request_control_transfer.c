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
/**   OHCI Controller Driver                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_ohci.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_ohci_request_control_transfer               PORTABLE C      */ 
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
/*     setting the OHCI endpoint as a candidate for transfer.             */
/*                                                                        */
/*     The maximum aggregated size of a data payload in OHCI is 4K. We    */
/*     are assuming that this size will be sufficient to contain the      */ 
/*     control packet.                                                    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI controller    */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ohci_register_read            Read OHCI register            */ 
/*    _ux_hcd_ohci_register_write           Write OHCI register           */ 
/*    _ux_hcd_ohci_regular_td_obtain        Get regular TD                */ 
/*    _ux_host_stack_transfer_request_abort Abort transfer request        */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*    _ux_utility_physical_address          Get physical address          */ 
/*    _ux_host_semaphore_get                Get semaphore                 */ 
/*    _ux_utility_short_put                 Write 16-bit value            */ 
/*    _ux_utility_virtual_address           Get virtual address           */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    OHCI Controller Driver                                              */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            prefixed UX to MS_TO_TICK,  */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warnings,     */
/*                                            resulting in version 6.1.2  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ohci_request_control_transfer(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request)
{

UX_DEVICE       *device;
UX_ENDPOINT     *endpoint;
UCHAR *         setup_request;
UX_OHCI_ED      *ed;
UX_OHCI_TD      *setup_td;
UX_OHCI_TD      *chain_td;
UX_OHCI_TD      *data_td;
UX_OHCI_TD      *tail_td;
UX_OHCI_TD      *status_td;
ULONG           ohci_register;
UINT            status;
    
    
    /* Get the pointer to the Endpoint and the Device.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;
    device =    endpoint -> ux_endpoint_device;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

    /* Build the SETUP packet (phase 1 of the control transfer).  */
    setup_request =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, UX_SETUP_SIZE);
    if (setup_request == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    *setup_request =                            (UCHAR)transfer_request -> ux_transfer_request_function;
    *(setup_request + UX_SETUP_REQUEST_TYPE) =  (UCHAR)transfer_request -> ux_transfer_request_type;
    *(setup_request + UX_SETUP_REQUEST) =       (UCHAR)transfer_request -> ux_transfer_request_function;
    _ux_utility_short_put(setup_request + UX_SETUP_VALUE, (USHORT)transfer_request -> ux_transfer_request_value);
    _ux_utility_short_put(setup_request + UX_SETUP_INDEX, (USHORT)transfer_request -> ux_transfer_request_index);
    _ux_utility_short_put(setup_request + UX_SETUP_LENGTH, (USHORT) transfer_request -> ux_transfer_request_requested_length);

    /* Set the ED address and MPS values since they may have changed.
       The ED direction will be set from the TD.  */
    ed -> ux_ohci_ed_dw0 =  device -> ux_device_address |  ((ULONG) endpoint -> ux_endpoint_descriptor.bEndpointAddress << 7) |
                                                           ((ULONG) endpoint -> ux_endpoint_descriptor.wMaxPacketSize << 16);

    /* Refresh the speed.  */
    if (device -> ux_device_speed == UX_LOW_SPEED_DEVICE)
        ed -> ux_ohci_ed_dw0 |=  UX_OHCI_ED_LOW_SPEED;

    /* Use the TD pointer by ed -> tail for our setup TD and chain from this one on.  */
    setup_td =  _ux_utility_virtual_address(ed -> ux_ohci_ed_tail_td);
    setup_td -> ux_ohci_td_dw0 =  UX_OHCI_TD_DEFAULT_DW0 | UX_OHCI_TD_DATA0 | UX_OHCI_TD_R;
    setup_td -> ux_ohci_td_cbp =  _ux_utility_physical_address(setup_request);
    setup_td -> ux_ohci_td_be =   setup_td -> ux_ohci_td_cbp + UX_SETUP_SIZE - 1;
    chain_td =  setup_td;

    /* Attach the endpoint and transfer request to the TD.  */
    setup_td -> ux_ohci_td_transfer_request =  transfer_request;
    setup_td -> ux_ohci_td_ed =  ed;

    /* Mark the TD with the SETUP phase.  */
    setup_td -> ux_ohci_td_status |=  UX_OHCI_TD_SETUP_PHASE;

    /* Check if there is a data phase, if not jump to status phase.  */
    data_td =  UX_NULL;    
    if (transfer_request -> ux_transfer_request_requested_length != 0)
    {

        data_td =  _ux_hcd_ohci_regular_td_obtain(hcd_ohci);
        if (data_td == UX_NULL)
        {

            _ux_utility_memory_free(setup_request);
            return(UX_NO_TD_AVAILABLE);
        }

        /* Attach the endpoint and transfer request to the TD.  */
        data_td -> ux_ohci_td_transfer_request =  transfer_request;
        data_td -> ux_ohci_td_ed =  ed;

        /* Mark the TD with the DATA phase.  */
        data_td -> ux_ohci_td_status |=  UX_OHCI_TD_DATA_PHASE;

        /* Program the control bits of the TD.  */
        if ((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)

            data_td -> ux_ohci_td_dw0 =  UX_OHCI_TD_DEFAULT_DW0 | UX_OHCI_TD_IN | UX_OHCI_TD_DATA1 | UX_OHCI_TD_R;
        else

            data_td -> ux_ohci_td_dw0 =  UX_OHCI_TD_DEFAULT_DW0 | UX_OHCI_TD_OUT | UX_OHCI_TD_DATA1 | UX_OHCI_TD_R;

        /* Attach the CBP and BE values to the TD.  */
        data_td -> ux_ohci_td_cbp =  _ux_utility_physical_address(transfer_request -> ux_transfer_request_data_pointer);
        data_td -> ux_ohci_td_be =   data_td -> ux_ohci_td_cbp + transfer_request -> ux_transfer_request_requested_length - 1;

        /* Update the length of the transfer for this TD.  */
        data_td -> ux_ohci_td_length =  transfer_request -> ux_transfer_request_requested_length;

        /* Chain the TD.  */
        chain_td -> ux_ohci_td_next_td =  _ux_utility_physical_address(data_td);
        chain_td =  data_td;
    }

    /* Now, program the status phase.  */
    status_td =  _ux_hcd_ohci_regular_td_obtain(hcd_ohci);
    if (status_td == UX_NULL)
    {

        _ux_utility_memory_free(setup_request);
        if (data_td != UX_NULL)
            data_td -> ux_ohci_td_status =  UX_UNUSED;
        return(UX_NO_TD_AVAILABLE);
    }

    /* Attach the endpoint and transfer request to the TD.  */
    status_td -> ux_ohci_td_transfer_request =  transfer_request;
    status_td -> ux_ohci_td_ed =  ed;

    /* Mark the TD with the STATUS phase.  */
    status_td -> ux_ohci_td_status |=  UX_OHCI_TD_STATUS_PHASE;

    /* The direction of the status phase is IN if data phase is OUT and
       vice versa.  */
    if ((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)

        status_td -> ux_ohci_td_dw0 =  UX_OHCI_TD_DEFAULT_DW0 | UX_OHCI_TD_OUT | UX_OHCI_TD_DATA1;
    else

        status_td -> ux_ohci_td_dw0 =  UX_OHCI_TD_DEFAULT_DW0 | UX_OHCI_TD_IN | UX_OHCI_TD_DATA1;

    /* No data payload for the status phase.  */
    status_td -> ux_ohci_td_cbp =  0;
    status_td -> ux_ohci_td_be =  0;

    /* Hook the status phase to the previous TD.  */
    chain_td -> ux_ohci_td_next_td =  _ux_utility_physical_address(status_td);

    /* Since we have consumed out tail TD for the setup packet, we must get another 
       one and hook it to the ED's tail.  */
    tail_td =  _ux_hcd_ohci_regular_td_obtain(hcd_ohci);
    if (tail_td == UX_NULL)
    {

        _ux_utility_memory_free(setup_request);
        if (data_td != UX_NULL)
            data_td -> ux_ohci_td_status =  UX_UNUSED;
        status_td -> ux_ohci_td_status =  UX_UNUSED;
        return(UX_NO_TD_AVAILABLE);
    }

    /* Hook the new TD to the status TD.  */
    status_td -> ux_ohci_td_next_td =  _ux_utility_physical_address(tail_td);
            
    /* At this stage, the Head and Tail in the ED are still the same and
       the OHCI controller will skip this ED until we have hooked the new
       tail TD.  */
    ed -> ux_ohci_ed_tail_td =  _ux_utility_physical_address(tail_td);

    /* Now, we must tell the OHCI controller that there is something in the
       control queue.  */
    ohci_register =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_COMMAND_STATUS);
    ohci_register |=  OHCI_HC_CS_CLF;
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_COMMAND_STATUS, ohci_register);

   /* Wait for the completion of the transfer request.  */
    status =  _ux_host_semaphore_get(&transfer_request -> ux_transfer_request_semaphore, UX_MS_TO_TICK(UX_CONTROL_TRANSFER_TIMEOUT));

    /* If the semaphore did not succeed we probably have a time out.  */
    if (status != UX_SUCCESS)
    {

        /* All transfers pending need to abort. There may have been a partial transfer. */
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

    /* Return the completion status.  */
    return(transfer_request -> ux_transfer_request_completion_code);           
}

