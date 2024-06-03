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
/*    _ux_hcd_ohci_initialize                             PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function initializes the OHCI controller. It sets the dma     */ 
/*     areas, programs all the OHCI registers, setup the ED and TD        */ 
/*     containers, sets the control, and builds the periodic lists.       */ 
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
/*    _ux_hcd_ohci_periodic_tree_create     Create OHCI periodic tree     */ 
/*    _ux_hcd_ohci_power_root_hubs          Power root HUBs               */ 
/*    _ux_hcd_ohci_register_read            Read OHCI register            */ 
/*    _ux_hcd_ohci_register_write           Write OHCI register           */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_host_mutex_on                     Get mutex protection          */ 
/*    _ux_host_mutex_off                    Release mutex protection      */ 
/*    _ux_utility_physical_address          Get physical address          */ 
/*    _ux_utility_set_interrupt_handler     Setup interrupt handler       */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Host Stack                                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Xiuwen Cai               Modified comment(s),          */
/*                                            fixed HcPeriodicStart value,*/
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Yajun Xia                Modified comment(s),          */
/*                                            fixed OHCI PRSC issue,      */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_ohci_initialize(UX_HCD *hcd)
{

UX_HCD_OHCI     *hcd_ohci;
ULONG           ohci_register;
UINT            index_loop;
UINT            status;


    /* The controller initialized here is of OHCI type.  */
    hcd -> ux_hcd_controller_type =  UX_OHCI_CONTROLLER;

#if UX_MAX_DEVICES > 1
    /* Initialize the max bandwidth for periodic endpoints. On OHCI, the spec says no 
       more than 90% to be allocated for periodic.  */
    hcd -> ux_hcd_available_bandwidth =  UX_OHCI_AVAILABLE_BANDWIDTH;
#endif

    /* Allocate memory for this OHCI HCD instance.  */
    hcd_ohci =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HCD_OHCI));
    if (hcd_ohci == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Set the pointer to the OHCI HCD.  */
    hcd -> ux_hcd_controller_hardware =  (VOID *) hcd_ohci;

    /* Save the HCOR address.  */
    hcd_ohci -> ux_hcd_ohci_hcor =  (ULONG *) hcd -> ux_hcd_io;

    /* Set the generic HCD owner for the OHCI HCD.  */
    hcd_ohci -> ux_hcd_ohci_hcd_owner =  hcd;

    /* Initialize the function collector for this HCD.  */
    hcd -> ux_hcd_entry_function =  _ux_hcd_ohci_entry;

    /* Set the state of the controller to HALTED first.  */
    hcd -> ux_hcd_status =  UX_HCD_STATUS_HALTED;

    /* get an DMA safe address for the HCCA. This block of memory is to be aligned
       on 256 bytes.  */
    hcd_ohci -> ux_hcd_ohci_hcca =  _ux_utility_memory_allocate(UX_ALIGN_256, UX_CACHE_SAFE_MEMORY, sizeof(UX_HCD_OHCI_HCCA));
    if (hcd_ohci -> ux_hcd_ohci_hcca == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Allocate the list of eds. All eds are allocated on 16 byte memory boundary.  */
    hcd_ohci -> ux_hcd_ohci_ed_list =  _ux_utility_memory_allocate(UX_ALIGN_16, UX_CACHE_SAFE_MEMORY, sizeof(UX_OHCI_ED) * _ux_system_host -> ux_system_host_max_ed);
    if (hcd_ohci -> ux_hcd_ohci_ed_list == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Allocate the list of tds. All tds are allocated on 32 byte memory boundary.  */
    hcd_ohci -> ux_hcd_ohci_td_list =  _ux_utility_memory_allocate(UX_ALIGN_32, UX_CACHE_SAFE_MEMORY, sizeof(UX_OHCI_TD) * _ux_system_host -> ux_system_host_max_td);
    if (hcd_ohci -> ux_hcd_ohci_td_list == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Allocate the list of isochronous tds. All tds are allocated on 32 byte memory boundary.  */
    hcd_ohci -> ux_hcd_ohci_iso_td_list =  _ux_utility_memory_allocate(UX_ALIGN_32, UX_CACHE_SAFE_MEMORY, sizeof(UX_OHCI_ISO_TD) * _ux_system_host -> ux_system_host_max_iso_td);
    if (hcd_ohci -> ux_hcd_ohci_td_list == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Initialize the periodic tree.  */
    status =  _ux_hcd_ohci_periodic_tree_create(hcd_ohci);
    if (status != UX_SUCCESS)
        return(status);

#if UX_MAX_DEVICES > 1

    /* Read the OHCI controller version, it is either USB 1.0 or 1.1. This is important for 
       filtering INT out endpoints on a 1.0 OHCI.  */
    hcd -> ux_hcd_version =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_REVISION);
#endif

    /* Set the state of the OHCI controller to reset in the control register.
       This is not compulsory but some controllers demand to start in this state.  */
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_CONTROL, 0);

    /* The following is time critical. If we get interrupted here, the controller will go in 
       suspend mode. Get the protection mutex.  */
    _ux_host_mutex_on(&_ux_system -> ux_system_mutex);

    /* Send the reset command to the controller. The controller should ack
       this command within 10us. We try this several time and check for timeout.  */
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_COMMAND_STATUS, OHCI_HC_CS_HCR);

    for (index_loop = 0; index_loop < UX_OHCI_RESET_RETRY; index_loop++)
    {

        ohci_register =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_COMMAND_STATUS);
        if ((ohci_register & OHCI_HC_CS_HCR) == 0)
            break;
    } 
      
    /* Check if the controller is reset properly.  */
    if ((ohci_register & OHCI_HC_CS_HCR) != 0)
    {

        /* Release the thread protection.  */
        _ux_host_mutex_off(&_ux_system -> ux_system_mutex);

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_CONTROLLER_INIT_FAILED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONTROLLER_INIT_FAILED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_CONTROLLER_INIT_FAILED);
    }
    
    /* Set the HCCA pointer to the HCOR.  */
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_HCCA, (ULONG) _ux_utility_physical_address(hcd_ohci -> ux_hcd_ohci_hcca));

    /* For now and until we have control and bulk ED, reset the control and bulk head registers.  */
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_CONTROL_HEAD_ED, 0);
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_CONTROL_CURRENT_ED, 0);
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_BULK_HEAD_ED, 0);
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_BULK_CURRENT_ED, 0);
           
    /* Turn on the OHCI controller functional registers we will use after this operation, 
       the controller is operational.  */
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_CONTROL, OHCI_HC_CONTROL_VALUE);
    hcd -> ux_hcd_status =  UX_HCD_STATUS_OPERATIONAL;

    /* We can safely release the mutex protection.  */    
    _ux_host_mutex_off(&_ux_system -> ux_system_mutex);

    /* Set the controller interval.  */
    ohci_register =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_FM_INTERVAL) & OHCI_HC_FM_INTERVAL_CLEAR;
    ohci_register |=  OHCI_HC_FM_INTERVAL_SET;
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_FM_INTERVAL, ohci_register);
    
    /* Set HcPeriodicStart to a value that is 90% of the value in FrameInterval field of the HcFmInterval register.  */
    ohci_register =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_FM_INTERVAL) & OHCI_HC_FM_INTERVAL_FI_MASK;
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_PERIODIC_START, ohci_register * 9 / 10);

    /* Reset all the OHCI interrupts and re-enable only the ones we will use.  */
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_INTERRUPT_DISABLE, OHCI_HC_INTERRUPT_DISABLE_ALL);
    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_INTERRUPT_ENABLE, OHCI_HC_INTERRUPT_ENABLE_NORMAL);
    
    /* Get the number of ports on the controller. The number of ports needs to be reflected both 
       for the generic HCD container and the local OHCI container.  */
    ohci_register =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_RH_DESCRIPTOR_A);
    hcd -> ux_hcd_nb_root_hubs =  (UINT) (ohci_register & 0xff);
    if (hcd -> ux_hcd_nb_root_hubs > UX_MAX_ROOTHUB_PORT)
        hcd -> ux_hcd_nb_root_hubs = UX_MAX_ROOTHUB_PORT;
    hcd_ohci -> ux_hcd_ohci_nb_root_hubs =  hcd -> ux_hcd_nb_root_hubs;

    /* Create HCD event flags */
    status = _ux_host_event_flags_create(&hcd_ohci -> ux_hcd_ohci_event_flags_group, "ux_hcd_ohci_event_flags_group");
    if (status != UX_SUCCESS)
        return(status);

    /* All ports must now be powered to pick up device insertion.  */
    _ux_hcd_ohci_power_root_hubs(hcd_ohci);

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

