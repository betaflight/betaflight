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
/**   EHCI Controller Driver                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_ehci.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_request_control_transfer               PORTABLE C      */ 
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
/*     setting the EHCI endpoint as a candidate for transfer.             */
/*                                                                        */
/*     The max aggregated size of a data payload in EHCI is 16K. We are   */
/*     assuming that this size will be sufficient to contain the control  */
/*     packet.                                                            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ehci                              Pointer to EHCI controller    */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_ed_clean                 Clean TDs                     */ 
/*    _ux_hcd_ehci_request_transfer_add     Add transfer to ED            */ 
/*    _ux_host_stack_transfer_request_abort Abort transfer request        */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*    _ux_host_semaphore_get                Get semaphore                 */ 
/*    _ux_utility_short_put                 Write a 16-bit value          */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    EHCI Controller Driver                                              */
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
/*                                            refined macros names,       */
/*                                            fixed compile warnings,     */
/*                                            resulting in version 6.1.2  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ehci_request_control_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request)
{

UX_DEVICE       *device;
UX_ENDPOINT     *endpoint;
UCHAR *         setup_request;
UX_EHCI_ED      *ed;
ULONG           td_component;
UINT            status;
UINT            pid;


    /* Get the pointer to the Endpoint and to the device.  */
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

    /* Reset the last TD pointer since it is the first time we hook a transaction.  */
    ed -> ux_ehci_ed_last_td =  UX_NULL;

    /* Set the ED address and MPS values since they may have changed from 0 to x
       The ED direction will be set from the TD.  */
    ed -> ux_ehci_ed_cap0 |=  (endpoint -> ux_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION) << UX_EHCI_QH_ED_AD_LOC;

    /* Set the endpoint address (this should have changed after address setting).  */
    ed -> ux_ehci_ed_cap0 |=  device -> ux_device_address;

    /* Set the default MPS Capability info in the ED.  */
    ed -> ux_ehci_ed_cap0 &=  ~UX_EHCI_QH_MPS_MASK;
    ed -> ux_ehci_ed_cap0 |=  endpoint -> ux_endpoint_descriptor.wMaxPacketSize << UX_EHCI_QH_MPS_LOC;

    /* On Control transfers, the toggle is set in the TD, not the QH.  */
    ed -> ux_ehci_ed_cap0 |=  UX_EHCI_QH_DTC;

    /* The overlay parameters should be reset now.  */
    ed -> ux_ehci_ed_current_td =     UX_NULL;
    ed -> ux_ehci_ed_queue_element =  (UX_EHCI_TD *) UX_EHCI_TD_T;
    ed -> ux_ehci_ed_alternate_td =   (UX_EHCI_TD *) UX_EHCI_TD_T;
    ed -> ux_ehci_ed_state &=         UX_EHCI_QH_TOGGLE;
    ed -> ux_ehci_ed_bp0 =            UX_NULL;
    ed -> ux_ehci_ed_bp1 =            UX_NULL;
    ed -> ux_ehci_ed_bp2 =            UX_NULL;
    ed -> ux_ehci_ed_bp3 =            UX_NULL;
    ed -> ux_ehci_ed_bp4 =            UX_NULL;

    /* Build and hook the setup phase to the ED.  */
    status =  _ux_hcd_ehci_request_transfer_add(hcd_ehci, ed, UX_EHCI_TD_SETUP_PHASE, UX_EHCI_PID_SETUP, UX_EHCI_TOGGLE_0,
                                    setup_request, UX_SETUP_SIZE, transfer_request);
    if (status != UX_SUCCESS)    
    {

        /* We need to clean the tds attached if any.  */
        _ux_hcd_ehci_ed_clean(ed);
        return(status);
    }

    /* Test if data phase required, if so decide the PID to use and build/hook it to the ED.  */
    if (transfer_request -> ux_transfer_request_requested_length != 0)
    {

        if ((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)

            pid =  UX_EHCI_PID_IN;
        else            
        
            pid =  UX_EHCI_PID_OUT;

        status =  _ux_hcd_ehci_request_transfer_add(hcd_ehci, ed, UX_EHCI_TD_DATA_PHASE, pid, UX_EHCI_TOGGLE_1,
                                                        transfer_request -> ux_transfer_request_data_pointer,
                                                        transfer_request -> ux_transfer_request_requested_length,
                                                        transfer_request);
        if (status != UX_SUCCESS)    
        {

            /* We need to clean the tds attached if any.  */
            _ux_hcd_ehci_ed_clean(ed);
            return(status);
        }
    }        

    /* Program the status phase. the PID is the opposite of the data phase.  */
    if ((transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)
        pid =  UX_EHCI_PID_OUT;
    else            
        pid =  UX_EHCI_PID_IN;

    status =  _ux_hcd_ehci_request_transfer_add(hcd_ehci, ed, UX_EHCI_TD_STATUS_PHASE, pid,
                                    UX_EHCI_TOGGLE_1, UX_NULL, 0, transfer_request);

    if (status != UX_SUCCESS)    
    {

        /* We need to clean the tds attached if any.  */
        _ux_hcd_ehci_ed_clean(ed);
        return(status);
    }

    /* Set the IOC bit in the last TD.  */
    ed -> ux_ehci_ed_last_td -> ux_ehci_td_control |=  UX_EHCI_TD_IOC;
    
    /* Ensure the IOC bit is set before activating the TD. This is necessary 
       for some processors that perform writes out of order as an optimization.  */
    UX_DATA_MEMORY_BARRIER

    /* Activate the first TD linked to the ED.  */
    td_component =  (ULONG) ed -> ux_ehci_ed_queue_element;
    td_component &=  ~UX_EHCI_TD_T;
    ed -> ux_ehci_ed_queue_element =  (UX_EHCI_TD *) td_component;

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

    /* Return completion status.  */
    return(transfer_request -> ux_transfer_request_completion_code);           
}

