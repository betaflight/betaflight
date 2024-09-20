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


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_sim_host.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_transfer_abort                     PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function will abort transactions attached to a transfer       */ 
/*     request.                                                           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_sim_host                         Pointer to host controller     */ 
/*    transfer_request                     Pointer to transfer request    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_delay_ms                  Delay                         */ 
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
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_transfer_abort(UX_HCD_SIM_HOST *hcd_sim_host, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT             *endpoint;
UX_HCD_SIM_HOST_ED      *ed;
UX_HCD_SIM_HOST_TD      *head_td;
UX_HCD_SIM_HOST_TD      *tail_td;
    
    UX_PARAMETER_NOT_USED(hcd_sim_host);

    /* Get the pointer to the endpoint associated with the transfer request.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;
    
    /* From the endpoint container, get the address of the physical endpoint.  */
    ed =  (UX_HCD_SIM_HOST_ED *) endpoint -> ux_endpoint_ed;
    
    /* Check if this physical endpoint has been initialized properly!  */
    if (ed == UX_NULL)
    {
    
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_ENDPOINT_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_ENDPOINT_HANDLE_UNKNOWN, endpoint, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_ENDPOINT_HANDLE_UNKNOWN);
    }

#if defined(UX_HOST_STANDALONE)
    ed -> ux_sim_host_ed_status |= UX_HCD_SIM_HOST_ED_SKIP;
#else
    /* The endpoint may be active. If so, set the skip bit.  */
    ed -> ux_sim_host_ed_status |=  UX_HCD_SIM_HOST_ED_SKIP;
    
    /* Wait for the controller to finish the current frame processing.  */
    _ux_utility_delay_ms(1);
#endif

    /* Remove all the TDs from this ED and leave the head and tail pointing to the dummy TD.  */
    head_td =  ed -> ux_sim_host_ed_head_td;
    tail_td =  ed -> ux_sim_host_ed_tail_td;

    /* Free all TDs attached to the ED.  */
    while (head_td != tail_td)
    {

        /* Mark the current head TD as free. */
        head_td -> ux_sim_host_td_status =  UX_UNUSED;

        /* Update the head TD with the next TD.  */
        ed -> ux_sim_host_ed_head_td =  head_td -> ux_sim_host_td_next_td;

        /* Now the new head TD is the next TD in the chain.  */
        head_td =  ed -> ux_sim_host_ed_head_td;
    }

#if defined(UX_HOST_STANDALONE)
    /* Remove the skip and transfer bits in the ED.  */
    ed -> ux_sim_host_ed_status &= ~(UX_HCD_SIM_HOST_ED_SKIP|UX_HCD_SIM_HOST_ED_TRANSFER);
#else
    /* Remove the reset bit in the ED.  */
    ed -> ux_sim_host_ed_status &=  (ULONG)~UX_HCD_SIM_HOST_ED_SKIP;
#endif

    /* Return successful completion.  */
    return(UX_SUCCESS);         
}

