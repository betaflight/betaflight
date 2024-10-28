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
/**   Slave Simulator Controller Driver                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_sim_slave.h"
#include "ux_hcd_sim_host.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_dcd_sim_slave_initialize                        PORTABLE C      */
/*                                                           6.1.6        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB simulation slave controller.      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    None                                                                */ 
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate memory               */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Slave Simulator Controller Driver                                   */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added link with HCD,        */
/*                                            set device to full speed,   */
/*                                            resulting in version 6.1.6  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_sim_slave_initialize(VOID)
{

UX_SLAVE_DCD            *dcd;
UX_DCD_SIM_SLAVE        *dcd_sim_slave;
UX_HCD                  *hcd;

                                                                            
    /* Get the pointer to the DCD.  */
    dcd = &_ux_system_slave -> ux_system_slave_dcd;

    /* The controller initialized here is of Slave simulation type.  */
    dcd -> ux_slave_dcd_controller_type =  UX_DCD_SIM_SLAVE_SLAVE_CONTROLLER;
    
    /* Allocate memory for this Slave simulation DCD instance.  */
    dcd_sim_slave =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_DCD_SIM_SLAVE));

    /* Check if memory was properly allocated.  */
    if(dcd_sim_slave == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Set the pointer to the Slave simulation DCD.  */
    dcd -> ux_slave_dcd_controller_hardware =  (VOID *) dcd_sim_slave;

    /* Set the generic DCD owner for the Slave simulation DCD.  */
    dcd_sim_slave -> ux_dcd_sim_slave_dcd_owner =  dcd;

    /* Initialize the function collector for this DCD.  */
    dcd -> ux_slave_dcd_function =  _ux_dcd_sim_slave_function;

    /* Link the HCD (always first registered one) to DCD driver.  */
    if (_ux_system_host)
    {
        hcd = _ux_system_host -> ux_system_host_hcd_array;
        if (hcd)
        {
            dcd_sim_slave -> ux_dcd_sim_slave_hcd = (VOID *)hcd;
        }
    }

    /* Set system to full speed by default.  */
    _ux_system_slave -> ux_system_slave_speed = UX_FULL_SPEED_DEVICE;

    /* Set the state of the controller to OPERATIONAL now.  */
    dcd -> ux_slave_dcd_status =  UX_DCD_STATUS_OPERATIONAL;

    /* This operation completed with success. */
    return(UX_SUCCESS);
}

