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
/*    _ux_host_stack_class_instance_get                   PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns a class instance pointer for a specific       */ 
/*    class. The instance of a class is not contained in the class code   */ 
/*    to reduce the class complexity. Rather, each class instance is      */ 
/*    attached to class container.                                        */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    class                                 Pointer to class              */ 
/*    class_index                           Index of class                */ 
/*    class_instance                        Destination of class instance */ 
/*                                            pointer                     */ 
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
UINT  _ux_host_stack_class_instance_get(UX_HOST_CLASS *host_class, UINT class_index, VOID **class_instance)
{
    
VOID    **current_class_instance;
    

    /* Start with the first class instance attached to the class container.  */
    current_class_instance =  host_class -> ux_host_class_first_instance;
    
    /* Check if there are any instances attached.  */
    if(current_class_instance == UX_NULL)
    {        

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* Traverse the list of the class instances until we found the right one.  */        
    while (class_index-- != 0)
    {

        /* Points to the next class instance.  */
        current_class_instance =  *current_class_instance;

        /* Check if we have reached the end of the list of the class instances.  */
        if (current_class_instance == UX_NULL)
        {        

            return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
        }
    }

    /* Update the class instance pointer from the caller.  */
    *class_instance =  current_class_instance;
    
    /* Return successful completion.  */
    return(UX_SUCCESS);
}

