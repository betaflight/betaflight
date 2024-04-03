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
#include "ux_dcd_sim_slave.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_port_reset                         PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    Implements the PORT_RESET request.                                  */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd_sim_host                          Pointer to host controller    */ 
/*    port_index                            Port index to reset           */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_disconnect           Simulate device disconnection */ 
/*    _ux_dcd_sim_slave_initialize_complete Complete device initialization*/
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
/*  08-02-2021     Wen Wang                 Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            resulting in version 6.1.8  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_port_reset(UX_HCD_SIM_HOST *hcd_sim_host, ULONG port_index)
{

UX_SLAVE_DEVICE     *device; 

#if defined(UX_HOST_STANDALONE)
    /* No port reset wait simulated, return _STATE_NEXT later to move state.  */
#endif

    UX_PARAMETER_NOT_USED(hcd_sim_host);
    UX_PARAMETER_NOT_USED(port_index);

    /* Get a pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Is this a connection?  */
    if (device -> ux_slave_device_state == UX_DEVICE_RESET)

        /* Complete the device initialization. Note that every time the device
           is disconnected, this must be called again for connection.  */
        _ux_dcd_sim_slave_initialize_complete();

    else
    {

        /* Host sent a PORT_RESET when the device is Attached, Addressed, or 
           Configured. Per the USB spec, we should go back to the default state. 
           We do this by 1) simulating disconnect to get rid of class and device 
           resources and 2) recreating necessary entities for control transfers.  */
        _ux_device_stack_disconnect();
        _ux_dcd_sim_slave_initialize_complete();
    }

    /* In either case, mark the device as default/attached now.  */
    device -> ux_slave_device_state =  UX_DEVICE_ATTACHED;

    /* This function should never fail.  */
#if defined(UX_HOST_STANDALONE)
    return(UX_STATE_NEXT);
#else
    return(UX_SUCCESS);
#endif
}

