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
/*    _ux_hcd_ehci_power_root_hubs                        PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function powers individually or in gang mode the root HUBs     */ 
/*    attached to the EHCI controller.                                    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ehci                              Pointer to EHCI controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_register_read            Read EHCI register            */ 
/*    _ux_hcd_ehci_register_write           Write EHCI register           */ 
/*    _ux_utility_delay_ms                  Delay                         */ 
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
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
VOID  _ux_hcd_ehci_power_root_hubs(UX_HCD_EHCI *hcd_ehci)
{

ULONG       ehci_register;
UINT        port_index;


    /* Read the control PPC field. If the PPC field is set, the controller has 
       implemented port power.  */
    ehci_register =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCCR_HCS_PARAMS);

    if (ehci_register & EHCI_HC_RH_PPC)
    {

        /* We have power management in this controller. Apply power to each port.  */
        for (port_index = 0; port_index < hcd_ehci -> ux_hcd_ehci_nb_root_hubs; port_index++)
        {
            
            /* Read register first to preserve existing settings.  */
            ehci_register = _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_PORT_SC + port_index);

            /* Apply power to a port.  */
            _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_PORT_SC + port_index, ehci_register | EHCI_HC_PS_PP);
        }
    }        

    /* The EHCI needs some time for the power to be stable.  */
    _ux_utility_delay_ms(EHCI_HC_RH_POWER_STABLE_DELAY);

    /* Return to caller.  */
    return;
}    

