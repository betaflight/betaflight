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
/*    _ux_host_stack_interfaces_scan                      PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function scans all the interfaces and alternate settings for   */
/*    particular configuration.                                           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    configuration                         Where the interface(s) will   */ 
/*                                            be attached                 */
/*    descriptor                            Contains the entire descriptor*/ 
/*                                            for this configuration      */
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_descriptor_parse          Parse interface descriptor    */
/*    _ux_host_stack_new_interface_create   Create new interface          */ 
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_interfaces_scan(UX_CONFIGURATION *configuration, UCHAR * descriptor)
{

ULONG                               total_configuration_length;
UINT                                descriptor_length;
UINT                                descriptor_type;                
ULONG                               status;
ULONG                               interface_association_descriptor_present;
ULONG                               interface_in_iad_count;
UX_INTERFACE_ASSOCIATION_DESCRIPTOR interface_association;

    /* Retrieve the size of all the configuration descriptor.  */
    total_configuration_length =  configuration -> ux_configuration_descriptor.wTotalLength;
    
    /* Set the IAD to false.  */
    interface_association_descriptor_present = UX_FALSE;

    /* Set the IAD interface count to zero.  */
    interface_in_iad_count = 0;

    /* Scan the entire descriptor and search for interfaces. We should also ensure that 
       the descriptor is valid by verifying the length of each descriptor scanned.  */
    while (total_configuration_length)
    {

        /* Gather the length and type of the descriptor.  */
        descriptor_length =  *descriptor;
        descriptor_type =    *(descriptor + 1);

        /* Make sure this descriptor has at least the minimum length.  */
        if (descriptor_length < 3)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_DESCRIPTOR_CORRUPTED);
        }            

        /* Check the type for an interface association descriptor.  */
        if (descriptor_type == UX_INTERFACE_ASSOCIATION_DESCRIPTOR_ITEM)
        {

            /* Parse the interface association descriptor and make it machine independent.  */
            _ux_utility_descriptor_parse(descriptor,
                            _ux_system_interface_association_descriptor_structure,
                            UX_INTERFACE_ASSOCIATION_DESCRIPTOR_ENTRIES,
                            (UCHAR *) &interface_association);

            /* Retrieve the CLASS/SUBCLASS from descriptor and store it in the configuration instance.  */
            configuration -> ux_configuration_iad_class    = interface_association.bFunctionClass;
            configuration -> ux_configuration_iad_subclass = interface_association.bFunctionSubClass;
            configuration -> ux_configuration_iad_protocol = interface_association.bFunctionProtocol;

            /* We have an IAD.  */
            interface_association_descriptor_present = UX_TRUE;
            
            /* Memorize the number of interfaces attached to this IAD.  */
            interface_in_iad_count = interface_association.bInterfaceCount;
        }
        
        /* Check the type for an interface descriptor.  */
        if (descriptor_type == UX_INTERFACE_DESCRIPTOR_ITEM)
        {

            /* We have found an interface descriptor. This descriptor contains at least 
               the default alternate setting (with value 0) and may have others.  */
            status =  _ux_host_stack_new_interface_create(configuration, descriptor, total_configuration_length);

            /* Are we within an IAD ? */
            if (interface_association_descriptor_present == UX_TRUE)
            {

                /* Decrement the number of interfaces attached here.  */
                interface_in_iad_count--;
                
                /* Are we at the end of the interface count ? */
                if (interface_in_iad_count == 0)
                {
    
                    /* Set the IAD to false now.  */
                    interface_association_descriptor_present = UX_FALSE;

                    /* Reset the IAD Class/Subclass/Protocol. */
                    configuration -> ux_configuration_iad_class    = 0;
                    configuration -> ux_configuration_iad_subclass = 0;
                    configuration -> ux_configuration_iad_protocol = 0;

                }
            }

            /* Check return status.  */
            if(status != UX_SUCCESS)
                return(status);
        }       

        /* Check the type for an OTG descriptor.  */
        if (descriptor_type == UX_OTG_DESCRIPTOR_ITEM)
        
            /* Retrieve the bmAttributes for SRP/HNP support.  */
            configuration -> ux_configuration_otg_capabilities = (ULONG) *(descriptor + UX_OTG_BM_ATTRIBUTES);

        /* Verify if the descriptor is still valid.  */
        if (descriptor_length > total_configuration_length)
        {

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_ENUMERATOR, UX_DESCRIPTOR_CORRUPTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_DESCRIPTOR_CORRUPTED, descriptor, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_DESCRIPTOR_CORRUPTED);
        }
        /* Jump to the next descriptor if we have not reached the end.  */
        descriptor +=  descriptor_length;

        /* And adjust the length left to parse in the descriptor.  */
        total_configuration_length -=  descriptor_length;
    }

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

