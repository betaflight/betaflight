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
/*    _ux_host_stack_class_get                            PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns a pointer to the class container. A class     */ 
/*    needs to obtain its container from the USBX stack to search for     */ 
/*    instances when a driver or an application wants to open a device.   */ 
/*                                                                        */
/*    Note: The C string of class_name must be NULL-terminated and the    */
/*    length of it (without the NULL-terminator itself) must be no larger */
/*    than UX_MAX_CLASS_NAME_LENGTH.                                      */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    class_name                            Name of class                 */ 
/*    class                                 Class pointer                 */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_string_length_check       Check C string and return its */
/*                                          length if null-terminated     */
/*    _ux_utility_memory_compare            Compare memory blocks         */ 
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
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_class_get(UCHAR *class_name, UX_HOST_CLASS **host_class)
{

UX_HOST_CLASS       *class_ptr;
#if !defined(UX_NAME_REFERENCED_BY_POINTER)
UINT                status;
UINT                class_name_length =  0;
#endif
#if UX_MAX_CLASS_DRIVER > 1
ULONG               class_index;
#endif

#if !defined(UX_NAME_REFERENCED_BY_POINTER)
    /* Get the length of the class name (exclude null-terminator).  */
    status =  _ux_utility_string_length_check(class_name, &class_name_length, UX_MAX_CLASS_NAME_LENGTH);
    if (status)
        return(status);
#endif

    /* Get first class.  */
    class_ptr =  _ux_system_host -> ux_system_host_class_array;

#if UX_MAX_CLASS_DRIVER > 1
    /* We need to parse the class driver table.  */
    for (class_index = 0; class_index < _ux_system_host -> ux_system_host_max_class; class_index++)
    {
#endif

        /* Check if this class is already being used. */
        if (class_ptr -> ux_host_class_status == UX_USED)
        {

            /* We have found a container. Check if this is the one we need (compare including null-terminator).  */
            if (ux_utility_name_match(class_ptr -> ux_host_class_name, class_name, class_name_length + 1))
            {           

                /* The class container was found. Update the pointer to the class container for the caller.  */
                *host_class =  class_ptr;

                /* Return success.  */
                return(UX_SUCCESS);
            }
        }

#if UX_MAX_CLASS_DRIVER > 1
        /* Move to the next class.  */
        class_ptr++;
    }
#endif

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_UNKNOWN, class_name, 0, 0, UX_TRACE_ERRORS, 0, 0)

    /* This class does not exist, return an error.  */    
    return(UX_HOST_CLASS_UNKNOWN);
}

