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
/*    _ux_hcd_ohci_interrupt_handler                      PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function is the interrupt handler for the OHCI interrupts.    */
/*     Normally an interrupt occurs from the controller when there is     */ 
/*     either a EOF signal and there has been transfers within the frame  */ 
/*     or when there is a change on one of the downstream ports.          */
/*                                                                        */
/*     All we need to do in the ISR is scan the controllers to find out   */ 
/*     which one has issued a IRQ. If there is work to do for this        */ 
/*     controller we need to wake up the corresponding thread to take     */ 
/*     care of the job.                                                   */ 
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
/*    _ux_hcd_ohci_register_read            Read OHCI register            */ 
/*    _ux_hcd_ohci_register_write           Write OHCI register           */ 
/*    _ux_host_semaphore_put                Put semaphore                 */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    ThreadX Interrupt Handler                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Yajun Xia                Modified comment(s),          */
/*                                            fixed OHCI PRSC issue,      */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_hcd_ohci_interrupt_handler(VOID)
{

UINT            hcd_index;
UX_HCD          *hcd;
UX_HCD_OHCI     *hcd_ohci;
ULONG           ohci_register =  0;
ULONG           ohci_register_port_status;
ULONG           root_hub_thread_wakeup = 0;
ULONG           port_index;


    /* We need to parse the controller driver table to find all controllers that 
       registered as OHCI.  */
    for (hcd_index = 0; hcd_index < _ux_system_host -> ux_system_host_registered_hcd; hcd_index++)
    {

        /* Check type of controller.  */
        if (_ux_system_host -> ux_system_host_hcd_array[hcd_index].ux_hcd_controller_type == UX_OHCI_CONTROLLER)
        {

            /* Get the pointers to the generic HCD and OHCI specific areas.  */
            hcd =  &_ux_system_host -> ux_system_host_hcd_array[hcd_index];
            hcd_ohci =  (UX_HCD_OHCI *) hcd -> ux_hcd_controller_hardware;

            /* Check if the controller is operational, if not, skip it.  */
            if (hcd -> ux_hcd_status == UX_HCD_STATUS_OPERATIONAL)
            {

                /* We get the current interrupt status for this controller.   */
                ohci_register =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_INTERRUPT_STATUS);

                /* Examine the source of interrupts.  */
                if (ohci_register & OHCI_HC_INT_WDH)
                {

                    /* We have some transferred EDs in the done queue. The controller thread needs 
                       to wake up and process them.  */
                    hcd_ohci -> ux_hcd_ohci_done_head =  hcd_ohci -> ux_hcd_ohci_hcca -> ux_hcd_ohci_hcca_done_head;
                    hcd_ohci -> ux_hcd_ohci_hcca -> ux_hcd_ohci_hcca_done_head =  UX_NULL;                    
                    hcd -> ux_hcd_thread_signal++;
                    _ux_host_semaphore_put(&_ux_system_host -> ux_system_host_hcd_semaphore);

                    /* Since we have delayed the processing of the done queue to a thread.
                       We need to ensure the host controller will not overwrite the done
                       queue pointer. So we disable the WDH bit in the interrupt status
                       before we acknowledge the IRQ register.  */
                    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_INTERRUPT_DISABLE, OHCI_HC_INT_WDH);
                }
                    
                if (ohci_register & OHCI_HC_INT_UE)
                {

                    /* The controller has issued a Unrecoverable Error signal. The controller will 
                       be reset now, and we wake up the HCD thread.  */
                    _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_COMMAND_STATUS, OHCI_HC_CS_HCR);
                    hcd -> ux_hcd_thread_signal++;
                    hcd -> ux_hcd_status =  UX_HCD_STATUS_DEAD;
                    _ux_host_semaphore_put(&_ux_system_host -> ux_system_host_hcd_semaphore);
                }

                if (ohci_register & OHCI_HC_INT_RHSC)
                {

                    /* The controller has issued a Root HUB status change signal.  There may be one or more events
                       that caused this status change. Only device insertion/extraction are monitored here. */
                    for (port_index = 0; port_index < hcd_ohci -> ux_hcd_ohci_nb_root_hubs; port_index++)
                    {

                        /* Read the port status.  */
                        ohci_register_port_status =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_RH_PORT_STATUS + port_index);
                
                        /* Check for Connect Status Change signal.  */
                        if (ohci_register_port_status &  OHCI_HC_PS_CSC)
                        {                        
                            /* Something happened on this port. Signal it to the root hub thread.  */
                            hcd -> ux_hcd_root_hub_signal[port_index]++;
                            
                            /* Memorize wake up signal.  */
                            root_hub_thread_wakeup ++;
                        }
                        
                        if (ohci_register_port_status &  OHCI_HC_PS_PRSC)
                        {
                            _ux_host_event_flags_set(&hcd_ohci -> ux_hcd_ohci_event_flags_group, UX_OHCI_PRSC_EVENT, UX_OR);
                        }

                        /* Clear the root hub interrupt signal. */
                        _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_RH_PORT_STATUS + port_index,
                                                    (OHCI_HC_PS_CSC | OHCI_HC_PS_PESC | OHCI_HC_PS_PSSC | OHCI_HC_PS_OCIC | OHCI_HC_PS_PRSC));
                    }

                    /* We only wake up the root hub thread if there has been device insertion/extraction.  */
                    if (root_hub_thread_wakeup != 0)
                        _ux_host_semaphore_put(&_ux_system_host -> ux_system_host_enum_semaphore);
                }
            }

            /* We have processed the interrupts for this controller, acknowledge them
               so that the controller can continue to work.  */
            _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_INTERRUPT_STATUS, ohci_register);
        }
    }
}

