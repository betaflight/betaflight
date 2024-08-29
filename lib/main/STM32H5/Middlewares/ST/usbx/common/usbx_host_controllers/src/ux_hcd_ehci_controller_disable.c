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
/*    _ux_hcd_ehci_controller_disable                     PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will disable the EHCI controller. The controller will */
/*    release all its resources (memory, IO ...). After this, the         */ 
/*    controller will not send SOF any longer.                            */
/*                                                                        */
/*    All transactions should have been completed, all classes should     */ 
/*    have been closed.                                                   */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ehci                              Pointer to EHCI controller    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_register_read            Read EHCI register            */ 
/*    _ux_hcd_ehci_register_write           Write EHCI register           */ 
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
UINT  _ux_hcd_ehci_controller_disable(UX_HCD_EHCI *hcd_ehci)
{

UX_HCD      *hcd;
ULONG       ehci_register;
    

    /* Point to the generic portion of the host controller structure instance.  */
    hcd =  hcd_ehci -> ux_hcd_ehci_hcd_owner;
    
    /* Stop the controller.  */
    ehci_register =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCOR_USB_COMMAND);
    ehci_register =  EHCI_HC_IO_HCRESET;
    ehci_register &= ~EHCI_HC_IO_RS;
    _ux_hcd_ehci_register_write(hcd_ehci, EHCI_HCOR_USB_COMMAND, ehci_register);
    
    /* Wait for the Stop signal to be acknowledged by the controller.  */
    ehci_register =  0;
    while ((ehci_register&EHCI_HC_STS_HC_HALTED) == 0)
    {

        ehci_register =  _ux_hcd_ehci_register_read(hcd_ehci, EHCI_HCCR_HCS_PARAMS);
    }
         
    /* Reflect the state of the controller in the main structure.  */
    hcd -> ux_hcd_status =  UX_HCD_STATUS_HALTED;

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

