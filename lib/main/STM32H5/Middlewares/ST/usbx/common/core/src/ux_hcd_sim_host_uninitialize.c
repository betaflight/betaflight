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
/*    _ux_hcd_sim_host_uninitialize                       PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will uninitialize the simulated host controller.      */
/*    The controller will release all its resources (memory, IO ...).     */
/*    After this, the controller will not send SOF any longer.            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_sim_host                          Pointer to host controller    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_free               Free memory block             */
/*    _ux_utility_timer_delete              Delete timer                  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Host Simulator Controller Driver                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  11-09-2020     Chaoqiong Xiao           Initial Version 6.1.2         */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_uninitialize(UX_HCD_SIM_HOST *hcd_sim_host)
{

UX_HCD                  *hcd = hcd_sim_host -> ux_hcd_sim_host_hcd_owner;
#if defined(UX_HOST_STANDALONE)
UX_HCD_SIM_HOST_TD      *td;
UINT                    td_index;
#endif

    /* Set the state of the controller to HALTED first.  */
    hcd -> ux_hcd_status =  UX_HCD_STATUS_HALTED;

    /* Get simulated host controller.  */
    hcd_sim_host = (UX_HCD_SIM_HOST *)hcd -> ux_hcd_controller_hardware;

    /* Delete timer.  */
    _ux_host_timer_delete(&hcd_sim_host -> ux_hcd_sim_host_timer);

#if defined(UX_HOST_STANDALONE)

    /* Check if there is pending SETUP TD, free buffers of them.  */
    for (td_index = 0; td_index < _ux_system_host -> ux_system_host_max_td; td_index++)
    {
        td = &hcd_sim_host -> ux_hcd_sim_host_td_list[td_index];

        /* Skip free TDs.  */
        if (td -> ux_sim_host_td_status == UX_UNUSED)
            continue;

        /* Skip TDs not for setup.  */
        if ((td -> ux_sim_host_td_status &  UX_HCD_SIM_HOST_TD_SETUP_PHASE) == 0)
            continue;

        /* Skip TDs already freed.  */
        if (td -> ux_sim_host_td_buffer == UX_NULL)
            continue;

        /* Free the TD buffer.  */
        _ux_utility_memory_free(td -> ux_sim_host_td_buffer);
    }
#endif

    /* Free TD/ED memories.  */
    _ux_utility_memory_free(hcd_sim_host -> ux_hcd_sim_host_iso_td_list);
    _ux_utility_memory_free(hcd_sim_host -> ux_hcd_sim_host_td_list);
    _ux_utility_memory_free(hcd_sim_host -> ux_hcd_sim_host_ed_list);

    /* Free simulated host controller memory.  */
    _ux_utility_memory_free(hcd_sim_host);
    hcd -> ux_hcd_controller_hardware = UX_NULL;

    /* Set the state of the controller to UNUSED first.  */
    hcd -> ux_hcd_status =  UX_UNUSED;

    /* Return successful completion.  */
    return(UX_SUCCESS);
}
