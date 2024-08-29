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
/*    _ux_hcd_sim_host_ed_td_clean                        PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function process cleans the ED of all TDs except the last      */ 
/*    dummy TD.                                                           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    ed                                    Pointer to ED                 */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
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
VOID  _ux_hcd_sim_host_ed_td_clean(UX_HCD_SIM_HOST_ED *ed)
{

UX_HCD_SIM_HOST_TD      *head_td;
UX_HCD_SIM_HOST_TD      *tail_td;


    /* Remove all the tds from this ED and leave the head and tail pointing
       to the dummy TD.  */
    head_td =  ed -> ux_sim_host_ed_head_td;
    tail_td =  ed -> ux_sim_host_ed_tail_td;

    /* Free all tds attached to the ED.  */
    while (head_td != tail_td)
    {

        /* Mark the current head TD as free.  */
        head_td -> ux_sim_host_td_status =  UX_UNUSED;

        /* Update the head TD with the next TD.  */
        ed -> ux_sim_host_ed_head_td =  head_td -> ux_sim_host_td_next_td;

        /* Now the new head_td is the next TD in the chain.  */
        head_td =  ed -> ux_sim_host_ed_head_td;
    }
}

