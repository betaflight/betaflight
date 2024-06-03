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
/*    _ux_host_stack_class_call                           PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will call all the registered classes to the USBX      */
/*    stack. Each class will have the possibility to own the device or    */
/*    one of the interfaces of a device.                                  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    class_command                         Class command structure       */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Number of owners                                                    */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UX_HOST_CLASS  *_ux_host_stack_class_call(UX_HOST_CLASS_COMMAND *class_command)
{

UINT            status = UX_NO_CLASS_MATCH;
UX_HOST_CLASS   *class_inst;
#if UX_MAX_CLASS_DRIVER > 1
ULONG           class_index;
#endif

    /* Start from the 1st registered classes with USBX.  */
    class_inst =  _ux_system_host -> ux_system_host_class_array;

    /* Parse all the class drivers.  */
#if UX_MAX_CLASS_DRIVER > 1
    for (class_index = 0; class_index < _ux_system_host -> ux_system_host_max_class; class_index++)
    {
#endif

        /* Check if this class driver is used.  */
        if (class_inst -> ux_host_class_status == UX_USED)
        {

            /* We have found a potential candidate. Call this registered class entry function.  */
            status = class_inst -> ux_host_class_entry_function(class_command);

            /* The status tells us if the registered class wants to own this class.  */
            if (status == UX_SUCCESS)
            {

                /* Yes, return this class pointer.  */
                return(class_inst); 
            }
        }    
#if UX_MAX_CLASS_DRIVER > 1
        /* Move to the next registered class. */
        class_inst ++;
    }
#endif

    /* There is no driver who want to own this class!  */
    return(UX_NULL);
}
