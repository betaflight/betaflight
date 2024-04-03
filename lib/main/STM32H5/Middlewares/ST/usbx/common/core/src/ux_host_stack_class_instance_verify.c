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
/*    _ux_host_stack_class_instance_verify                PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function ensures that a given instance exists. An application  */
/*    is not responsible for keeping the instance valid pointer. The      */ 
/*    class is responsible for the instance checks if the instance is     */ 
/*    still valid.                                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    class_name                            Name of class                 */ 
/*    class_instance                        Pointer to class instance     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_string_length_check       Check C string and return its */
/*                                          length if null-terminated     */
/*    _ux_utility_memory_compare            Compare blocks of memory      */ 
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
UINT  _ux_host_stack_class_instance_verify(UCHAR *class_name, VOID *class_instance)
{

UX_HOST_CLASS   *class_inst;
#if UX_MAX_CLASS_DRIVER > 1
ULONG           class_index;
#endif
VOID            **current_class_instance;
#if !defined(UX_NAME_REFERENCED_BY_POINTER)
UINT            status;
UINT            class_name_length =  0;
#endif

#if !defined(UX_NAME_REFERENCED_BY_POINTER)
    /* Get the length of the class name (exclude null-terminator).  */
    status =  _ux_utility_string_length_check(class_name, &class_name_length, UX_MAX_CLASS_NAME_LENGTH);
    if (status)
        return(status);
#endif

    /* Get first class.  */
    class_inst =  _ux_system_host -> ux_system_host_class_array;

#if UX_MAX_CLASS_DRIVER > 1
    /* We need to parse the class table.  */
    for(class_index = 0; class_index < _ux_system_host -> ux_system_host_max_class; class_index++)
    {
#endif

        /* Check if this class is already used.  */
        if (class_inst -> ux_host_class_status == UX_USED)
        {

            /* Start with the first class instance attached to the class container.  */
            current_class_instance =  class_inst -> ux_host_class_first_instance;
    
            /* Traverse the list of the class instances until we find the correct instance.  */        
            while (current_class_instance != UX_NULL)
            {

                /* Check the class instance attached to the container with the caller's
                   instance.  */
                if (current_class_instance == class_instance)
                {

                    /* We have found the class container. Check if this is the one we need (compare including null-terminator).  */
                    if (ux_utility_name_match(class_inst-> ux_host_class_name, class_name, class_name_length + 1))
                        return(UX_SUCCESS);
                }

                /* Points to the next class instance.  */
                current_class_instance =  *current_class_instance;
            }
        }
#if UX_MAX_CLASS_DRIVER > 1

        /* Move to the next class.  */
        class_inst ++;
    }    
#endif
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, class_instance, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* This class does not exist.  */    
    return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
}
