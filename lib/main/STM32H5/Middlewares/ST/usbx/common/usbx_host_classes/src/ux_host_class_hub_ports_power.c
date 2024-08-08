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
/*    _ux_host_class_hub_ports_power                      PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will power up each downstream port attached to the    */
/*    HUB. There is a delay after powering up each port, otherwise we may */ 
/*    not detect device insertion.                                        */
/*                                                                        */
/*    There are 3 port power modes:                                       */
/*                                                                        */
/*      1) Gang power:          In this case we only power the first      */ 
/*                              port and all ports should be powered at   */ 
/*                              the same time                             */
/*                                                                        */
/*      2) Individual power:    In this case we power individually each   */ 
/*                              port                                      */
/*                                                                        */
/*      3) No power switching:  In this case the power is applied to the  */
/*                              downstream ports when the upstream port   */
/*                              receives power.                           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hub                                   Pointer to HUB class          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hub_feature            Set HUB class feature         */ 
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
UINT  _ux_host_class_hub_ports_power(UX_HOST_CLASS_HUB *hub)
{

UINT        nb_ports;
UINT        port_index;
UINT        status;
    

    /* Check for the power management mode: no power switching.  */
    if(hub -> ux_host_class_hub_descriptor.wHubCharacteristics & UX_HOST_CLASS_HUB_NO_POWER_SWITCHING)
        return(UX_SUCCESS);

    /* All ports must be powered individually.  */
    nb_ports =  hub -> ux_host_class_hub_descriptor.bNbPorts;

    /* Perform the function to all ports: the port index starts from 1 as the port 0 is for the HUB.  */
    for (port_index = 1; port_index <= nb_ports; port_index++)
    {

        /* To apply port power, we send a SET_FEATURE to the port on the HUB.  */
        status =  _ux_host_class_hub_feature(hub, port_index, UX_SET_FEATURE, UX_HOST_CLASS_HUB_PORT_POWER);

        /* Check the function result and update HUB status if there was a problem.  */
        if (status != UX_SUCCESS)
        {

            /* Set the HUB status to not powered.  */
            hub -> ux_host_class_hub_port_power  &= (UINT)~(1 << port_index);

        }
        else
        {
        
            /* Now we need to wait for the power to be stable.  */
            _ux_utility_delay_ms(((ULONG) (hub -> ux_host_class_hub_descriptor.bPwrOn2PwrGood) * 2));
        
            /* Set the HUB status to powered.  */
            hub -> ux_host_class_hub_port_power  |= (UINT)(1 << port_index);
        }
    }

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

