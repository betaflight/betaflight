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
/**   Device Data Pump Class                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_dpump.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_dpump_initialize                   PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function initializes the USB dpump device.                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                                 Pointer to dpump command    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate             Allocate memory             */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Data Pump Class                                              */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_dpump_initialize(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_CLASS_DPUMP                    *dpump;
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_CLASS_DPUMP_PARAMETER          *dpump_parameter;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Create an instance of the device dpump class.  */
    dpump =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_SLAVE_CLASS_DPUMP));

    /* Check for successful allocation.  */
    if (dpump == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Save the address of the DPUMP instance inside the DPUMP container.  */
    class_ptr -> ux_slave_class_instance = (VOID *) dpump;
    
    /* Get the pointer to the application parameters for the cdc class.  */
    dpump_parameter =  command -> ux_slave_class_command_parameter;

    /* Store the start and stop signals if needed by the application.  */
    dpump -> ux_slave_class_dpump_parameter.ux_slave_class_dpump_instance_activate = dpump_parameter -> ux_slave_class_dpump_instance_activate;
    dpump -> ux_slave_class_dpump_parameter.ux_slave_class_dpump_instance_deactivate = dpump_parameter -> ux_slave_class_dpump_instance_deactivate;

    /* Return completion status.  */
    return(UX_SUCCESS);
}


