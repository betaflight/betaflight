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
/**   HUB Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hub.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hub_port_change_process              PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will process a port change indication.                */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hub                                   Pointer to HUB class          */ 
/*    port                                  Port number                   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hub_port_change_connection_process                   */ 
/*                                          Process connection            */ 
/*    _ux_host_class_hub_port_change_enable_process                       */ 
/*                                          Enable process                */ 
/*    _ux_host_class_hub_port_change_over_current_process                 */ 
/*                                          Change over current process   */ 
/*    _ux_host_class_hub_port_change_reset_process                        */ 
/*                                          Reset process                 */ 
/*    _ux_host_class_hub_port_change_suspend_process                      */ 
/*                                          Suspend process               */ 
/*    _ux_host_class_hub_status_get         Get HUB status                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HUB Class                                                           */ 
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
UINT  _ux_host_class_hub_port_change_process(UX_HOST_CLASS_HUB *hub, UINT port)
{

USHORT      port_status;
USHORT      port_change;
UINT        status;
    

    /* First step is to retrieve the status on the port with a GET_STATUS.  */
    status =  _ux_host_class_hub_status_get(hub, port, &port_status, &port_change);
    if (status != UX_SUCCESS)
        return(status);
            
    /* On return of the GET_STATUS, the port change field has been updated 
       check for each of the bits it may contain.  */
    if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_CONNECTION)
        _ux_host_class_hub_port_change_connection_process(hub, port, port_status);       

    if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_ENABLE)
        _ux_host_class_hub_port_change_enable_process(hub, port, port_status);       

    if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_SUSPEND)
        _ux_host_class_hub_port_change_suspend_process(hub, port, port_status);       

    if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_OVER_CURRENT)
        _ux_host_class_hub_port_change_over_current_process(hub, port, port_status);       

    if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_RESET)
        _ux_host_class_hub_port_change_reset_process(hub, port, port_status);       
           
    /* Return successful completion.  */
    return(UX_SUCCESS);
}

