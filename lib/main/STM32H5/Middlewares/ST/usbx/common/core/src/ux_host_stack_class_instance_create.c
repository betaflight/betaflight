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
/*    _ux_host_stack_class_instance_create                PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates a new class instance for a class container.   */
/*    The instance of a class is not contained in the class code to       */ 
/*    reduce the class driver complexity. Rather, each class instance is  */ 
/*    attached to class container.                                        */ 
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
UINT  _ux_host_stack_class_instance_create(UX_HOST_CLASS *host_class, VOID *class_instance)
{
    
VOID    **current_class_instance;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_CLASS_INSTANCE_CREATE, host_class, class_instance, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_CLASS_INSTANCE, class_instance, 0, 0, 0)

    /* Start with the first class instance attached to the class container.  */
    current_class_instance =  host_class -> ux_host_class_first_instance;
    
    /* Check if there are any instances attached.  */
    if (current_class_instance == UX_NULL)
    {

        /* Since it is the first class, attach it to the class container.  */
        host_class -> ux_host_class_first_instance =  class_instance;

        /* Return successful completion.  */
        return(UX_SUCCESS);
    }

    /* Traverse the list of the class instances until we find the last class.  */        
    while (*current_class_instance != UX_NULL)
    {

        /* Point to the next class instance.  */
        current_class_instance =  *current_class_instance;
    }

    /* We have reached the last class, hook the new class to the end. This way, we preserve
       the chronological order of the class instances.  */
    *current_class_instance =  class_instance;
    
    /* Return successful completion to caller.  */
    return(UX_SUCCESS);
}

