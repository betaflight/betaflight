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
/*    _ux_host_stack_configuration_instance_create        PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function will create a configuration instance. It scan all the */
/*    interfaces hooked to the configuration and enable each interface    */
/*    with the alternate setting 0.                                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    configuration                         Pointer to configuration      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_interface_instance_create  Create interface instance */ 
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
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added interface instance    */
/*                                            creation strategy control,  */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_configuration_instance_create(UX_CONFIGURATION *configuration)
{

UX_INTERFACE    *interface_ptr;
UINT            status;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_CONFIGURATION_INSTANCE_CREATE, configuration, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Obtain the first interface for this configuration.  */
    interface_ptr =  configuration -> ux_configuration_first_interface;

    /* Each selected alternate setting 0 for each interface must be created.  */
    while (interface_ptr != UX_NULL)
    {

        /* Check if we are dealing with the first alternate setting.  */
        if (interface_ptr -> ux_interface_descriptor.bAlternateSetting == 0)
        {

#if UX_HOST_STACK_CONFIGURATION_INSTANCE_CREATE_CONTROL == UX_HOST_STACK_CONFIGURATION_INSTANCE_CREATE_OWNED

            /* Create the interface, if it's usable. */
            if (interface_ptr -> ux_interface_class || configuration -> ux_configuration_device -> ux_device_class)
#endif
            {
                status = _ux_host_stack_interface_instance_create(interface_ptr);

                /* Check status, the controller may have refused the endpoint creation.  */
                if (status != UX_SUCCESS)
                
                    /* An error occurred.  The interface cannot be mounted.  */
                    return(status);
            }
        }

        /* Next interface.  */
        interface_ptr =  interface_ptr -> ux_interface_next_interface;
    }

    /* Return successful completion.  */
    return(UX_SUCCESS); 
}

