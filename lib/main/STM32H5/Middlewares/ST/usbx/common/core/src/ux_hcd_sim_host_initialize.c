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


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_initialize                         PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function initializes the simulated host controller             */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    HCD                                   Pointer to HCD                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_periodic_tree_create Create periodic tree          */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_semaphore_put             Semaphore put                 */ 
/*    _ux_utility_timer_create              Create timer                  */ 
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
/*                                            optimized based on compile  */
/*                                            definitions, used UX prefix */
/*                                            to refer to TX symbols      */
/*                                            instead of using them       */
/*                                            directly,                   */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added link with DCD,        */
/*                                            resulting in version 6.1.6  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_initialize(UX_HCD *hcd)
{

UX_SLAVE_DCD        *dcd;
UX_DCD_SIM_SLAVE    *dcd_sim_slave;
UX_HCD_SIM_HOST     *hcd_sim_host;
UINT                status;


    /* The controller initialized here is of host simulator type.  */
    hcd -> ux_hcd_controller_type =  UX_HCD_SIM_HOST_CONTROLLER;
    
    /* Allocate memory for this host simulator HCD instance.  */
    hcd_sim_host =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HCD_SIM_HOST));
    if (hcd_sim_host == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Set the pointer to the host simulator HCD.  */
    hcd -> ux_hcd_controller_hardware =  (VOID *) hcd_sim_host;

    /* Set the generic HCD owner for the host simulator HCD.  */
    hcd_sim_host -> ux_hcd_sim_host_hcd_owner =  hcd;

    /* Initialize the function collector for this HCD.  */
    hcd -> ux_hcd_entry_function =  _ux_hcd_sim_host_entry;

#if UX_MAX_DEVICES > 1
    /* Initialize the max bandwidth for periodic endpoints. In simulation this is
       not very important.  */
    hcd -> ux_hcd_available_bandwidth =  UX_HCD_SIM_HOST_AVAILABLE_BANDWIDTH;
#endif

    /* Set the state of the controller to HALTED first.  */
    hcd -> ux_hcd_status =  UX_HCD_STATUS_HALTED;

    /* Allocate the list of EDs. All EDs are allocated on 16 byte memory boundary.  */
    hcd_sim_host -> ux_hcd_sim_host_ed_list =  _ux_utility_memory_allocate(UX_ALIGN_16, UX_REGULAR_MEMORY, (ULONG)sizeof(UX_HCD_SIM_HOST_ED) * _ux_system_host -> ux_system_host_max_ed);
    if (hcd_sim_host -> ux_hcd_sim_host_ed_list == UX_NULL)
        status = UX_MEMORY_INSUFFICIENT;
    else
        status = UX_SUCCESS;

    /* Allocate the list of TDs. All TDs are allocated on 32 byte memory boundary.  */
    if (status == UX_SUCCESS)
    {
        hcd_sim_host -> ux_hcd_sim_host_td_list =  _ux_utility_memory_allocate(UX_ALIGN_32, UX_REGULAR_MEMORY, (ULONG)sizeof(UX_HCD_SIM_HOST_TD) * _ux_system_host -> ux_system_host_max_td);
        if (hcd_sim_host -> ux_hcd_sim_host_td_list == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
    }

    /* Allocate the list of isochronous TDs. All TDs are allocated on 32 byte memory boundary.  */
    if (status == UX_SUCCESS)
    {
        hcd_sim_host -> ux_hcd_sim_host_iso_td_list =  _ux_utility_memory_allocate(UX_ALIGN_32, UX_REGULAR_MEMORY, (ULONG)sizeof(UX_HCD_SIM_HOST_ISO_TD) * _ux_system_host -> ux_system_host_max_iso_td);
        if (hcd_sim_host -> ux_hcd_sim_host_iso_td_list == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
    }

    /* Initialize the periodic tree.  */
    if (status == UX_SUCCESS)
        status =  _ux_hcd_sim_host_periodic_tree_create(hcd_sim_host);

    /* Initialize the scheduler.  */
    if (status == UX_SUCCESS)
    {
        /* Set the host controller into the operational state.  */
        hcd -> ux_hcd_status =  UX_HCD_STATUS_OPERATIONAL;

        /* The asynchronous queues are empty for now.  */
        hcd_sim_host -> ux_hcd_sim_host_queue_empty =  UX_TRUE;

        /* The periodic scheduler is not active.  */
        hcd_sim_host -> ux_hcd_sim_host_periodic_scheduler_active =  0;
        
        /* We start a timer that will invoke the simulator every timer tick.  */
        status = _ux_host_timer_create(&hcd_sim_host -> ux_hcd_sim_host_timer, "USBX Simulation Timer",
                        _ux_hcd_sim_host_timer_function, (ULONG) (ALIGN_TYPE) hcd_sim_host, 1, 1, UX_AUTO_ACTIVATE);
    }

    UX_TIMER_EXTENSION_PTR_SET(&(hcd_sim_host -> ux_hcd_sim_host_timer), hcd_sim_host)

    /* Free up resources and return when there is error.  */
    if (status != UX_SUCCESS)
    {

        /* Set the host controller into the halt state.  */
        hcd -> ux_hcd_status =  UX_HCD_STATUS_HALTED;

        /* The last resource, timer is not created or created error,
         * no need to delete.  */

        if (hcd_sim_host -> ux_hcd_sim_host_iso_td_list)
            _ux_utility_memory_free(hcd_sim_host -> ux_hcd_sim_host_iso_td_list);
        if (hcd_sim_host -> ux_hcd_sim_host_td_list)
            _ux_utility_memory_free(hcd_sim_host -> ux_hcd_sim_host_td_list);
        if (hcd_sim_host -> ux_hcd_sim_host_ed_list)
            _ux_utility_memory_free(hcd_sim_host -> ux_hcd_sim_host_ed_list);
        _ux_utility_memory_free(hcd_sim_host);

        return(status);
    }

    /* Link the HCD to DCD driver.  */
    if (_ux_system_slave)
    {
        dcd = &_ux_system_slave -> ux_system_slave_dcd;
        if (dcd)
        {
            dcd_sim_slave = (UX_DCD_SIM_SLAVE *) dcd -> ux_slave_dcd_controller_hardware;
            if (dcd_sim_slave)
                dcd_sim_slave -> ux_dcd_sim_slave_hcd = (VOID *)hcd;
        }
    }

    /* Get the number of ports on the controller. The number of ports needs to be reflected both 
       for the generic HCD container and the local sim_host container. In the simulator,
       the number of ports is hardwired to 1 only.  */
    hcd -> ux_hcd_nb_root_hubs =  1;
    hcd_sim_host -> ux_hcd_sim_host_nb_root_hubs =  1;
    hcd_sim_host -> ux_hcd_sim_host_port_status[0] = UX_PS_CCS | UX_PS_DS_FS;

    /* Something happened on this port. Signal it to the root hub thread.  */
    hcd -> ux_hcd_root_hub_signal[0] =  1;

    /* We need to simulate a Root HUB Status Change for the USB stack since the simulator
       has not root HUB per se.  */
    status = _ux_host_semaphore_put_rc(&_ux_system_host -> ux_system_host_enum_semaphore);
    if (status != UX_SUCCESS)

        /* Resources are still ready but
         * failed to simulate Root HUB change!  */
        return(UX_SEMAPHORE_ERROR);

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

