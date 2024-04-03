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
/*    _ux_hcd_sim_host_port_status_get                    PORTABLE C      */ 
/*                                                           6.1.6        */
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
/*    hcd_sim_host                          Pointer to host controller    */ 
/*    port_index                            Port index                    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Host Simulator Port Status                                          */ 
/*                                                                        */ 
/*    Where port status has the following format:                         */
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
/*    None                                                                */ 
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
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used changeable status,     */
/*                                            resulting in version 6.1.6  */
/*                                                                        */
/**************************************************************************/
ULONG  _ux_hcd_sim_host_port_status_get(UX_HCD_SIM_HOST *hcd_sim_host, ULONG port_index)
{

ULONG       port_status;


    /* Check to see if this port is valid on this controller.  */
    if (hcd_sim_host -> ux_hcd_sim_host_nb_root_hubs < port_index)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_PORT_INDEX_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_PORT_INDEX_UNKNOWN, port_index, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_PORT_INDEX_UNKNOWN);
    }
    
    /* Returns current port status.  */
    port_status = hcd_sim_host -> ux_hcd_sim_host_port_status[port_index];

    /* Return port status.  */
    return(port_status);            
}

