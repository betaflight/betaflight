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
/*    _ux_host_stack_new_interface_create                 PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates a new interface for the current configuration */
/*    scanned. A device has at least 1 alternate setting per interface    */ 
/*    which is the default one.                                           */
/*                                                                        */
/*    The interface is hooked to the configuration that owns it.          */
/*                                                                        */
/*    From the interface descriptor, all the endpoints are hooked but     */
/*    not activated.                                                      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    configuration                         Configuration container that  */ 
/*                                            owns this interface         */
/*    interface_pointer                     Pointer to a unparsed         */ 
/*                                            interface descriptor        */
/*    length                                Length remaining in this      */ 
/*                                            descriptor                  */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_new_endpoint_create    Create new endpoint           */ 
/*    _ux_utility_descriptor_parse          Parse the descriptor          */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_new_interface_create(UX_CONFIGURATION *configuration,
                                            UCHAR * descriptor, ULONG length)
{

UX_INTERFACE        *list_interface;
UX_INTERFACE        *interface_ptr;
UINT                number_endpoints;
UINT                descriptor_length;
UINT                descriptor_type;
UINT                status;
UCHAR               *this_interface_descriptor;

    /* Obtain memory for storing this new interface.  */
    interface_ptr =  (UX_INTERFACE *) _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_INTERFACE));
    
    /* If no memory left, exit with error.  */        
    if (interface_ptr == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Save the interface handle in the container, this is for ensuring the
       interface container is not corrupted.  */
    interface_ptr -> ux_interface_handle =  (ULONG) (ALIGN_TYPE) interface_ptr;

    /* Parse the interface descriptor and make it machine independent.  */
    _ux_utility_descriptor_parse(descriptor,
                            _ux_system_interface_descriptor_structure,
                            UX_INTERFACE_DESCRIPTOR_ENTRIES,
                            (UCHAR *) &interface_ptr -> ux_interface_descriptor);

    /* The configuration that owns this interface is memorized in the 
       interface container itself, easier for back chaining.  */
    interface_ptr -> ux_interface_configuration =  configuration;
    
    /* If the interface belongs to an IAD, remember the IAD Class/SubClass/Protocol.  */
    interface_ptr -> ux_interface_iad_class    = configuration -> ux_configuration_iad_class;
    interface_ptr -> ux_interface_iad_subclass = configuration -> ux_configuration_iad_subclass;
    interface_ptr -> ux_interface_iad_protocol = configuration -> ux_configuration_iad_protocol;

    /* There is 2 cases for the creation of the interface descriptor 
       if this is the first one, the interface descriptor is hooked
       to the configuration. If it is not the first one, the interface 
       is hooked to the end of the chain of interfaces.  */
    if (configuration -> ux_configuration_first_interface == UX_NULL)
    {
        configuration -> ux_configuration_first_interface =  interface_ptr;
    }
    else
    {
    
        list_interface =  configuration -> ux_configuration_first_interface;
        
        /* Traverse the list until we reach the end */
        while (list_interface -> ux_interface_next_interface != UX_NULL)
        {

            list_interface =  list_interface -> ux_interface_next_interface;
        }

        /* Hook the interface.  */
        list_interface -> ux_interface_next_interface =  interface_ptr;
    }

    /* Traverse the interface in search of all endpoints that belong to it.
       We need the length remaining in the descriptor and the number of endpoints
       reported for this interface.  */
    number_endpoints =  interface_ptr -> ux_interface_descriptor.bNumEndpoints;

    this_interface_descriptor = descriptor;

    while (length && (number_endpoints != 0))
    {

        /* Gather the length and type of the descriptor.  */
        descriptor_length =  *descriptor;
        descriptor_type =    *(descriptor+1);

        /* make sure this descriptor has at least the minimum length.  */
        if (descriptor_length < 3)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_DESCRIPTOR_CORRUPTED);
        }            

        /* Check the type for an interface descriptor.  */
        if (descriptor_type == UX_ENDPOINT_DESCRIPTOR_ITEM)
        {

            /* We have found an endpoint descriptor for this interface.  */
            status =  _ux_host_stack_new_endpoint_create(interface_ptr, descriptor);

            /* Check return status.  */
            if(status != UX_SUCCESS)
                return(status);

            number_endpoints--;
        }       

        /* Verify if the descriptor is still valid, or we moved to next interface.  */
        if ((descriptor_length > length) || (descriptor_type == UX_INTERFACE_DESCRIPTOR_ITEM && descriptor != this_interface_descriptor))
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_DESCRIPTOR_CORRUPTED);
        }

        /* Jump to the next descriptor if we have not reached the end.  */
        descriptor +=  descriptor_length;
        length -=  descriptor_length;
    }

    /* Return success!  */
    return(UX_SUCCESS);
}

