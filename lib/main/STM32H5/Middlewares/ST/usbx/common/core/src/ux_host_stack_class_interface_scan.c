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
/**   Host Stack                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_stack_class_interface_scan                 PORTABLE C      */ 
/*                                                           6.1.4        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will scan all default interfaces for a single         */ 
/*    configuration and call the registered class with the                */ 
/*    Class/SubClass/Protocol of the interface.                           */
/*                                                                        */
/*    If the device has multiple configurations (like the Apple iPod),    */
/*    the first configuration is treated as the default configuration.    */
/*    If a device which has multiple configurations wants to control the  */
/*    configuration selection, it must ensure that the PID/VID based      */
/*    class at the device level claims the entire device.                 */
/*                                                                        */
/*                                                                        */
/*    For the interface, there is no reason to use the PID/VID has a      */ 
/*    binding element as classes that trigger on PID/VID will be called   */ 
/*    by the device descriptor scanning process.                          */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    device                                Device pointer                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Result of operation                                                 */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_class_call             Call class command            */
/*    _ux_host_stack_device_configuration_select                          */
/*                                          Select configuration          */ 
/*    (ux_host_stack_class_call)            Call class from host stack    */ 
/*    (ux_host_class_entry_function)        Class entry function          */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Components                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  02-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used internal function call */
/*                                            to scan interfaces,         */
/*                                            resulting in version 6.1.4  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_class_interface_scan(UX_DEVICE *device)
{

UX_CONFIGURATION        *configuration;
UINT                    status;


    /* Get the 1st and only configuration.  If the device has multiple
       configurations, we simply use the first one as default. */
    configuration =  device -> ux_device_first_configuration;
    if (configuration == UX_NULL)
        return(UX_ERROR);

    /* Scan interfaces for this configuration.  */
    status = _ux_host_stack_configuration_interface_scan(configuration);

    /* Return operation result.  */
    return(status);
}

