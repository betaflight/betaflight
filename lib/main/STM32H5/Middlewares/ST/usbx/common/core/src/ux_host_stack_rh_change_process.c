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
/**   Host Stack                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_stack_rh_change_process                    PORTABLE C      */ 
/*                                                           6.1.2        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function checks for changes in topology in each of the         */ 
/*    installed HCDs.                                                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_rh_device_insertion    Process device insertion      */ 
/*    _ux_host_stack_rh_device_extraction   Process device extraction     */ 
/*    (ux_hcd_entry_function)               HCD entry function            */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Components                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used new interrupt macros,  */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed registered HCD scan,  */
/*                                            resulting in version 6.1.2  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_stack_rh_change_process(VOID)
{

UX_HCD      *hcd;
UINT        hcd_index;
ULONG       port_status;
UINT        port_index;
UX_INTERRUPT_SAVE_AREA

    /* This thread was maybe awaken by one or more HCD controllers. Check each 
       of the HCD to see where there has been a change of topology.  */
    for(hcd_index = 0; hcd_index < UX_SYSTEM_HOST_MAX_HCD_GET(); hcd_index++)
    {

        /* Pickup HCD pointer.  */
        hcd =  &_ux_system_host -> ux_system_host_hcd_array[hcd_index];

        /* Is this HCD operational?  */
        if (hcd -> ux_hcd_status == UX_HCD_STATUS_OPERATIONAL)
        {

            /* On each port of the root hub of the controller, get the port status */
            for (port_index = 0; port_index < hcd -> ux_hcd_nb_root_hubs; port_index++)
            {

                /* Was there any activity on this port ? */
                if (hcd -> ux_hcd_root_hub_signal[port_index] != 0)
                {

                    /* If trace is enabled, insert this event into the trace buffer.  */
                    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_RH_CHANGE_PROCESS, port_index, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

                    /* Reset that port activity signal.  */
                    UX_DISABLE
                    hcd -> ux_hcd_root_hub_signal[port_index]--;
                    UX_RESTORE

                    /* Call HCD for port status.  */
                    port_status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_GET_PORT_STATUS, (VOID *)((ALIGN_TYPE)port_index));
    
                    /* Check return status.  */
                    if (port_status != UX_PORT_INDEX_UNKNOWN)
                    {
    
                        /* The port_status value is valid and will tell us if there is
                           a device attached\detached on the downstream port.  */
                        if (port_status & UX_PS_CCS)
                        {
    
                            /* There is a device attached to this port. Check if we know 
                               about it. If not, this is a device insertion signal.  */
                            if ((hcd -> ux_hcd_rh_device_connection & (ULONG)(1 << port_index)) == 0)
                            {
                              
                                /* We have a simple device insertion, we have not lost any signals.  
                                   the root hub and the stack enumeration module are in synch.  */
                                _ux_host_stack_rh_device_insertion(hcd,port_index);
                            }
                            else
                            {
                                /* We can get here when there has been multiple events on the hardware root hub
                                   but we may have missed them of they were too close or the stack got too busy.
                                   We check the number of events in the root hub signal. If it is not zero
                                   we are out of synch, meaning we got a disconnection followed very quickly
                                   by a insertion.  */
                                
                                if (hcd -> ux_hcd_root_hub_signal[port_index] != 0)
                                {

                                    /* We need to get back in synch now.  */
                                    UX_DISABLE
                                    hcd -> ux_hcd_root_hub_signal[port_index] = 0;
                                    UX_RESTORE
                            
                                    /* First extract the device.  */
                                    _ux_host_stack_rh_device_extraction(hcd,port_index);
                                    
                                    /* Now, insert it again.  */
                                    _ux_host_stack_rh_device_insertion(hcd,port_index);
                                   

                                }

                            }
                        }
                        else
                        {
                              
                            /* There is no device attached to this port. Check if we know 
                               about it. If not, this is a device removal signal.  */
                            if ((hcd -> ux_hcd_rh_device_connection & (ULONG)(1 << port_index)) !=0)
                            {
                                _ux_host_stack_rh_device_extraction(hcd,port_index);
                            }
                        }
                    }
                }
            }
        }               
    }
}

