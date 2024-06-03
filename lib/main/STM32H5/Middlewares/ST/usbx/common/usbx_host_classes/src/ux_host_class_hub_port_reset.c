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
/*    _ux_host_class_hub_port_reset                       PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will reset a downstream port of the HUB. When the     */ 
/*    port is reset, the hardware logic of the HUB also enables it.       */ 
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
/*    _ux_host_class_hub_port_change_reset_process Reset HUB              */ 
/*    _ux_host_class_hub_feature            Set HUB class feature         */ 
/*    _ux_host_class_hub_status_get         Get HUB status                */ 
/*    _ux_utility_delay_ms                  Thread sleep                  */ 
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
UINT  _ux_host_class_hub_port_reset(UX_HOST_CLASS_HUB *hub, UINT port)
{

UINT        status;
UINT        port_enable_retry;    
USHORT      port_status;
USHORT      port_change;


    /* Send the PORT_RESET command.  */
    status =  _ux_host_class_hub_feature(hub, port, UX_SET_FEATURE, UX_HOST_CLASS_HUB_PORT_RESET);

    /* Check the function result and update HUB status if there was a problem.  */
    if (status != UX_SUCCESS)
        return(status);

    /* We allow retrying of this as we may not get the port enable status bit
       set on the first try.  */
    port_enable_retry =  UX_HOST_CLASS_HUB_ENABLE_RETRY_COUNT;
    while (port_enable_retry--)
    {

        /* Now get the status until we have a port enabled.  */
        status =  _ux_host_class_hub_status_get(hub, port, &port_status, &port_change);
        if (status != UX_SUCCESS)
            return(status);
            
        /* On return of the GET_STATUS, the port_change field has been updated. 
           Check for each of the bits it may contain. */
        if (port_change & UX_HOST_CLASS_HUB_PORT_CHANGE_RESET)
        {
 
            /* The port has been successfully reset.  */

            /* Clear the RESET change bit.  */
            _ux_host_class_hub_port_change_reset_process(hub, port, port_status);       

            /* Return success.  */
            return(UX_SUCCESS);
        }

        /* We should wait a bit before retrying this operation.  */
        _ux_utility_delay_ms(UX_HOST_CLASS_HUB_ENABLE_RETRY_DELAY);
    }

    /* We get here when the port never reported RESET completion.  */

    /* Invoke error callback.  */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HUB, UX_PORT_RESET_FAILED);

    /* Return error.  */
    return(UX_PORT_RESET_FAILED);
}

