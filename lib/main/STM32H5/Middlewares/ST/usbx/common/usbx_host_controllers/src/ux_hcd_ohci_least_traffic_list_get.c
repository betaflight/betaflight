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
/*    _ux_hcd_ohci_least_traffic_list_get                 PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function return a pointer to the first ED in the periodic     */ 
/*     tree that has the least traffic registered.                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI               */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    UX_OHCI_ED  *                         Pointer to OHCI ED            */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_virtual_address           Get virtual address           */ 
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
/*                                                                        */
/**************************************************************************/
UX_OHCI_ED  *_ux_hcd_ohci_least_traffic_list_get(UX_HCD_OHCI *hcd_ohci)
{

UX_HCD_OHCI_HCCA        *ohci_hcca;
UX_OHCI_ED              *min_bandwidth_ed;
UX_OHCI_ED              *begin_ed;
UX_OHCI_ED              *ed;
UINT                    list_index;
ULONG                   min_bandwidth_used;
ULONG                   bandwidth_used;


    /* Get the pointer to the HCCA.  */
    ohci_hcca =  hcd_ohci -> ux_hcd_ohci_hcca;
    
    /* Set the min bandwidth used to a arbitrary maximum value.  */
    min_bandwidth_used =  0xffffffff;

    /* The first ED is the list candidate for now.  */
    min_bandwidth_ed =  _ux_utility_virtual_address(ohci_hcca -> ux_hcd_ohci_hcca_ed[0]);
    
    /* All list will be scanned.  */
    for (list_index = 0; list_index < 32; list_index++)
    {

        /* Reset the bandwidth for this list.  */
        bandwidth_used =  0;

        /* Get the ED of the beginning of the list we parse now.  */
        ed =  _ux_utility_virtual_address(ohci_hcca -> ux_hcd_ohci_hcca_ed[list_index]);

        /* We keep track of the first ED for the current list.  */
        begin_ed =  ed;

        /* Parse the eds in the list.  */
        while (ed -> ux_ohci_ed_next_ed != UX_NULL)
        {

            /* Add to the bandwidth used the max packet size pointed by this ED.  */
            bandwidth_used +=  (ed -> ux_ohci_ed_dw0 >> 16) & UX_OHCI_ED_MPS;

            /* Next ED.  */           
            ed =  _ux_utility_virtual_address(ed -> ux_ohci_ed_next_ed);
        }

        /* We have processed a list, check the bandwidth used by this list.
           If this bandwidth is the minimum, we memorize the ED.  */        
        if (bandwidth_used < min_bandwidth_used)
        {

            /* We have found a better list with a lower used bandwidth, 
               memorize the bandwidth for this list.  */
            min_bandwidth_used =  bandwidth_used;
            
            /* Memorize the begin ED for this list.  */
            min_bandwidth_ed =  begin_ed;
        }
    }
    
    /* Return the ED list with the lowest bandwidth.  */
    return(min_bandwidth_ed);   
}

