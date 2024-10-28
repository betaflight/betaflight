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
/*    _ux_hcd_ohci_port_status_get                        PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*     This function will return the status for each port attached to the */
/*     root HUB.                                                          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_ohci                              Pointer to OHCI controller    */ 
/*    port_index                            Port index                    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    port_status                                                         */ 
/*                                                                        */ 
/*      Status of the root HUB port with the following format:            */
/*                                                                        */ 
/*               bit 0         device connection status                   */
/*                             if 0 : no device connected                 */
/*                             if 1 : device connected to the port        */
/*               bit 1         port enable status                         */
/*                             if 0 : port disabled                       */
/*                             if 1 : port enabled                        */
/*               bit 2         port suspend status                        */
/*                             if 0 : port is not suspended               */
/*                             if 1 : port is suspended                   */
/*               bit 3         port overcurrent status                    */
/*                             if 0 : port has no overcurrent condition   */
/*                             if 1 : port has overcurrent condition      */
/*               bit 4         port reset status                          */
/*                             if 0 : port is not in reset                */
/*                             if 1 : port is in reset                    */
/*               bit 5         port power status                          */
/*                             if 0 : port power is off                   */
/*                             if 1 : port power is on                    */
/*               bit 6-7       device attached speed                      */
/*                             if 00 : low speed device attached          */
/*                             if 01 : full speed device attached         */
/*                             if 10 : high speed device attached         */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ohci_register_read            Read OHCI register            */ 
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
ULONG  _ux_hcd_ohci_port_status_get(UX_HCD_OHCI *hcd_ohci, ULONG port_index)
{

ULONG       ohci_register_port_status;
ULONG       port_status;


    /* Check to see if this port is valid on this controller */
    if (hcd_ohci -> ux_hcd_ohci_nb_root_hubs < port_index)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_PORT_INDEX_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_PORT_INDEX_UNKNOWN, port_index, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_PORT_INDEX_UNKNOWN);
    }
    
    /* The port is valid, build the status mask for this port. This function
       returns a controller agnostic bit field.  */
    port_status =  0;
    ohci_register_port_status =  _ux_hcd_ohci_register_read(hcd_ohci, OHCI_HC_RH_PORT_STATUS + port_index);
                                    
    /* Device Connection Status.  */
    if (ohci_register_port_status & OHCI_HC_PS_CCS)
        port_status |=  UX_PS_CCS;
                                    
    /* Port Enable Status */
    if (ohci_register_port_status & OHCI_HC_PS_PES)
        port_status |=  UX_PS_PES;

    /* Port Suspend Status */
    if (ohci_register_port_status & OHCI_HC_PS_PSS)
        port_status |=  UX_PS_PSS;

    /* Port Overcurrent Status */
    if (ohci_register_port_status & OHCI_HC_PS_POCI)
        port_status |=  UX_PS_POCI;

    /* Port Reset Status */
    if (ohci_register_port_status & OHCI_HC_PS_PRS)
        port_status |=  UX_PS_PRS;

    /* Port Power Status */
    if (ohci_register_port_status & OHCI_HC_PS_PPS)
        port_status |=  UX_PS_PPS;

    /* Port Device Attached speed. This field is valid only if the CCS bit is active. 
       On OHCI, only low speed or full speed are available.  */
    if (ohci_register_port_status & OHCI_HC_PS_CCS)
    {

        if (ohci_register_port_status & OHCI_HC_PS_LSDA)
            port_status |=  UX_PS_DS_LS;
        else
            port_status |=  UX_PS_DS_FS;
    }

    /* Return port status.  */
    return(port_status);            
}

