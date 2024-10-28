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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_periodic_schedule                  PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function schedules new transfers from the periodic interrupt  */ 
/*     list.                                                              */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_sim_host                          Pointer to host controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_frame_number_get     Get frame number              */ 
/*    _ux_hcd_sim_host_transaction_schedule Schedule the transaction      */
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
/*                                                                        */
/**************************************************************************/
VOID  _ux_hcd_sim_host_periodic_schedule(UX_HCD_SIM_HOST *hcd_sim_host)
{

UX_HCD_SIM_HOST_ED      *ed;
ULONG                   frame_number;

    /* Get the current frame number.  */
    _ux_hcd_sim_host_frame_number_get(hcd_sim_host, &frame_number);
    
    /* Isolate the low bits to match an entry in the upper periodic entry list.  */
    frame_number &=  UX_HCD_SIM_HOST_PERIODIC_ENTRY_MASK;

    /* Get the first ED in the periodic list.  */
    ed =  hcd_sim_host -> ux_hcd_sim_host_interrupt_ed_list[frame_number];

    /* Search for an entry in the periodic tree.  */
    while (ed != UX_NULL) 
    {

        /* The ED has to be a real ED (not static) and has to have a different tail and head TD.  */
        if ((ed -> ux_sim_host_ed_status != UX_HCD_SIM_HOST_ED_STATIC) && (ed -> ux_sim_host_ed_tail_td != ed -> ux_sim_host_ed_head_td))
        {

            /* Ensure this ED does not have the SKIP bit set and no TD are in progress. */
            if ((ed -> ux_sim_host_ed_head_td -> ux_sim_host_td_status & UX_HCD_SIM_HOST_TD_ACK_PENDING) == 0)

                /* Insert this transfer in the list of scheduled TDs if possible.  */
                _ux_hcd_sim_host_transaction_schedule(hcd_sim_host, ed);

        }

        /* Point to the next ED in the list.  */
        ed =  ed -> ux_sim_host_ed_next_ed;
    }

    /* Return to caller.  */
    return;
}

