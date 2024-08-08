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
/*    _ux_host_stack_class_instance_destroy               PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function destroys a class instance for a class container.      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    class                                 Pointer to class              */ 
/*    class_instance                        Pointer to class instance     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    USBX Components                                                     */ 
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
UINT  _ux_host_stack_class_instance_destroy(UX_HOST_CLASS *host_class, VOID *class_instance)
{
    
VOID    **current_class_instance;
VOID    **next_class_instance;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_CLASS_INSTANCE_DESTROY, host_class, class_instance, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_UNREGISTER(class_instance);

    /* Get the pointer to the instance pointed by the instance to destroy.  */
    next_class_instance =  class_instance;
    next_class_instance =  *next_class_instance;
    
    /* Start with the first class instance attached to the class container.  */
    current_class_instance =  host_class -> ux_host_class_first_instance;
    
    /* Check if there are any instances attached.  */
    if (current_class_instance == UX_NULL)
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, class_instance, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* The first instance is a special case because it is attached to the class
       container.  */
    if (current_class_instance == class_instance)
    {

        /* Point to next class instance.  */
        host_class -> ux_host_class_first_instance = next_class_instance;
    
        /* Return success.  */
        return(UX_SUCCESS);
    }
       
    /* Traverse the list of the class instances until we found the right one.  */        
    while (*current_class_instance != UX_NULL)
    {

        /* Check to see if this class is the one we need to destroy.  */
        if(*current_class_instance == class_instance)
        {

            /* Point to next class instance.  */
            *current_class_instance =  next_class_instance;

            /* Return success.  */
            return(UX_SUCCESS);
        }

        /* Points to the next class instance.  */
        current_class_instance =  *current_class_instance;
    }
    
    /* Error trap. */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_HOST_CLASS_INSTANCE_UNKNOWN);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, class_instance, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* Return error to caller.  */
    return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
}

