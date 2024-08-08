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
/*    _ux_hcd_sim_host_asynch_schedule                    PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function schedules new transfers from the control/bulk lists.  */ 
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
/*    _ux_hcd_sim_host_transaction_schedule Schedule simulator transaction*/ 
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
VOID  _ux_hcd_sim_host_asynch_schedule(UX_HCD_SIM_HOST *hcd_sim_host)
{

UX_HCD_SIM_HOST_ED      *ed;
UX_HCD_SIM_HOST_ED      *first_ed;
UINT                    status;
                        

    /* Get the pointer to the current ED in the asynchronous list.  */
    ed =  hcd_sim_host -> ux_hcd_sim_host_asynch_current_ed;

    /* Check if there is any ED candidate in the asynch list.  */
    if (ed == UX_NULL)
    {

        /* Check if there is any ED in the asynch list. If none, nothing to do.  */
        if (hcd_sim_host -> ux_hcd_sim_host_asynch_head_ed == UX_NULL)
            return;
        else
            ed = hcd_sim_host -> ux_hcd_sim_host_asynch_head_ed;
    }

    /* Remember this ED.  */
    first_ed =  ed;

    /* In simulation, we are not tied to bandwidth limitation.  */
    do 
    {

        /* Check if this ED has a tail and head TD different.  */
        if (ed -> ux_sim_host_ed_tail_td != ed -> ux_sim_host_ed_head_td)
        {

            /* Schedule this transaction with the device simulator.  */
            status =  _ux_hcd_sim_host_transaction_schedule(hcd_sim_host, ed);

            /* If the TD has been added to the list, we can memorize this ED has 
               being served and make the next ED as the one to be first scanned 
               at the next SOF.  */
            if (status == UX_SUCCESS)
            {

                if (ed -> ux_sim_host_ed_next_ed == UX_NULL)
                    hcd_sim_host -> ux_hcd_sim_host_asynch_current_ed =  hcd_sim_host -> ux_hcd_sim_host_asynch_head_ed;
                else            
                    hcd_sim_host -> ux_hcd_sim_host_asynch_current_ed =  ed -> ux_sim_host_ed_next_ed;
            }
        }

        /* Point to the next ED in the list. Check if at end of list.  */
        if (ed -> ux_sim_host_ed_next_ed == UX_NULL)
            ed =  hcd_sim_host -> ux_hcd_sim_host_asynch_head_ed;
        else            
            ed =  ed -> ux_sim_host_ed_next_ed;

    } while ((ed) && (ed != first_ed));
}

