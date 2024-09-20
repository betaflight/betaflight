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
/*    _ux_hcd_ohci_power_root_hubs                        PORTABLE C      */ 
/*                                                           6.1.2        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function powers individually or in gang mode the root HUBs    */ 
/*     attached to the OHCI controller.                                   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ohci_register_read            OHCI register read            */ 
/*    _ux_hcd_ohci_register_write           OHCI register write           */ 
/*    _ux_utility_delay_ms                  Delay                         */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    OHCI Controller Driver                                              */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed compile warnings,     */
/*                                            resulting in version 6.1.2  */
/*                                                                        */
/**************************************************************************/
VOID  _ux_hcd_ohci_power_root_hubs(UX_HCD_OHCI *hcd_ohci)
{

ULONG       ohci_register_a;
ULONG       ohci_register_b;
ULONG       ohci_register_port_status;
UINT        port_index;


    /* Read the RH descriptor A. This will tell us if ports are always powered or not.  */
    ohci_register_a =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_RH_DESCRIPTOR_A);
    if (ohci_register_a & OHCI_HC_RH_NPS)
        return;

    /* Read the RH descriptor B. It will give us the characteristics of the root HUB.  */
    ohci_register_b =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_RH_DESCRIPTOR_B);
        
    /* The ports must be power switched. There are 3 possibilities:

            1) individual 
            2) gang mode 
            3) a combination of both

       The logic is as follows: 
       
       If the PSM bit is not set, gang mode is forced and we use the global power (LPSC) command.
       If PSM is set, each port is powered individually. 
       
       BUT we also need to look into the PPCM field to check if there is any ports
       that may still want to be powered by the global power command. If the bit for a port in 
       the mask is set, the power is applied by the local port command in the RH port status (PPS).  */
    if (ohci_register_a & OHCI_HC_RH_PSM)
    {

        /* Check the PPCM field to see if some existing ports need to be powered by the LPSC command.  */
        for (port_index = 0; port_index < hcd_ohci -> ux_hcd_ohci_nb_root_hubs; port_index++)
        {

            if ((ohci_register_b & (0x20000u << port_index)) == 0)
            {

                _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_RH_STATUS, OHCI_HC_RS_LPSC);
                break;
            }
        }            
        
        /* Ports have to be powered individually. This is done for each of the ports whose bit mask is 
           set in the PPCM field.  */
        for (port_index = 0; port_index < hcd_ohci -> ux_hcd_ohci_nb_root_hubs; port_index++)
        {

            if ((ohci_register_b & (0x20000u << port_index)) != 0)
            {

                ohci_register_port_status =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_RH_PORT_STATUS + port_index);
                
                ohci_register_port_status |=  OHCI_HC_PS_PPS;
                _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_RH_PORT_STATUS + port_index, ohci_register_port_status);
            }
        }
    }
    else
    {

        /* Ports have to be powered all at the same time.  */
        _ux_hcd_ohci_register_write(hcd_ohci, OHCI_HC_RH_STATUS, OHCI_HC_RS_LPSC);
    }
                    
    /* Wait for the power to be stable. the RH  descriptor contains the value POTPGT. We multiply this value by 2 
       and this is the number of milliseconds to wait for power to set.  */
    _ux_utility_delay_ms(ohci_register_a >> (OHCI_HC_RH_POTPGT - 1));

    /* Return to caller.  */
    return;
}    

