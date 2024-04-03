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
/*    _ux_host_stack_interface_setting_select             PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function selects a specific alternate setting for a given      */
/*    interface belonging to the selected configuration.                  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    interface                             Pointer to interface          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_interface_instance_create  Create interface instance */ 
/*    _ux_host_stack_interface_instance_delete  Delete interface instance */ 
/*    _ux_host_stack_interface_set              Set interface instance    */ 
/*    _ux_host_semaphore_get                    Get semaphore             */
/*    _ux_host_semaphore_put                    Put semaphore             */
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            protected control request,  */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_interface_setting_select(UX_INTERFACE *interface_ptr)
{

UX_CONFIGURATION    *configuration;
UX_INTERFACE        *current_interface;
UX_INTERFACE        *previous_interface;
UX_INTERFACE        *main_interface;
UINT                current_interface_number;
UINT                current_alternate_setting;
UINT                status;
    
    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_INTERFACE_SETTING_SELECT, interface_ptr, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Check this alternate setting container. It must be valid before
       we continue.  */
    if (interface_ptr -> ux_interface_handle != (ULONG) (ALIGN_TYPE) interface_ptr)
    {
        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_INTERFACE_HANDLE_UNKNOWN, interface_ptr, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_INTERFACE_HANDLE_UNKNOWN);
    }

    /* From the interface, get the configuration container and the first
       interface hooked to this configuration.  */    
    configuration =             interface_ptr -> ux_interface_configuration;
    current_interface_number =  interface_ptr -> ux_interface_descriptor.bInterfaceNumber;
    current_interface =         configuration -> ux_configuration_first_interface;

    /* Remember the main interface to store the next alternate setting.  We set the main interface
       to the first interface to keep the compiler happy. */
    main_interface =  current_interface;

    /* In order to keep the compiler happy, we reset the alternate setting.  */
    current_alternate_setting =  0;
    previous_interface = UX_NULL;

    /* Search for the current alternate setting for this interface.
       All its endpoints will need to be destroyed.
       Since interfaces in the parent configuration should be linked together
       in correct way, just find it in while loop.
     */
    while (1)
    {

        /* Try to locate the first interface container in the interface chain which
           has the same interface number as the one supplied by the caller.  */
        if (current_interface -> ux_interface_descriptor.bInterfaceNumber == current_interface_number)
        {

            /* The alternate setting 0 of this interface has the current selected
               alternate setting.  */
            if (current_interface -> ux_interface_descriptor.bAlternateSetting == 0)
            {

                /* Set the alternate setting.  */
                current_alternate_setting =  current_interface -> ux_interface_current_alternate_setting;

                /* Remember the main interface to store the next alternate setting.  */
                main_interface =  current_interface;

            }

            /* See if the current alternate setting matches that of the interface alternate setting.  */
            if (current_alternate_setting == current_interface -> ux_interface_descriptor.bAlternateSetting)
            {

                /* Yes, save the current alternate setting.  */
                previous_interface = current_interface;

                /* Then delete the current interface.  */
                _ux_host_stack_interface_instance_delete(current_interface);

                /* We are done in this loop.  */
                break;

            }
        }                    

        /* Move to the next interface. */
        current_interface =  current_interface -> ux_interface_next_interface;
    }

    /* Remember the new alternate setting.  */
    main_interface -> ux_interface_current_alternate_setting =  interface_ptr -> ux_interface_descriptor.bAlternateSetting;

    /* Now, the interface must be created with the new alternate setting.  */
    status =  _ux_host_stack_interface_instance_create(interface_ptr);

    /* If we could not create it, we return to the default one.  */
    if (status != UX_SUCCESS)
    {

        /* Then delete the failed interface.  */
        _ux_host_stack_interface_instance_delete(interface_ptr);

        /* Error, reset the main interface alternate setting to the default.  */
        main_interface -> ux_interface_current_alternate_setting =  current_alternate_setting;

        /* Re-create the previous interface with the old alternate setting.  */
        _ux_host_stack_interface_instance_create(previous_interface);

        /* Return error status.  */
        return(status);
    }

    /* Protect the control endpoint semaphore here.  It will be unprotected in the
       transfer request function.  */
    status =  _ux_host_semaphore_get(&configuration -> ux_configuration_device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);
    if (status != UX_SUCCESS)
        return(status);

    /* Issue a SET_INTERFACE command to the target device.  */
    status =  _ux_host_stack_interface_set(interface_ptr);

    /* Check completion status.  */
    if (status != UX_SUCCESS)
    {

        /* Error, reset the main interface alternate setting to the default.  */
        main_interface -> ux_interface_current_alternate_setting =  current_alternate_setting;

        /* Delete the current interface.  */
        _ux_host_stack_interface_instance_delete(interface_ptr);

        /* Re-create the previous interface with the old alternate setting.  */
        _ux_host_stack_interface_instance_create(previous_interface);

        /* Return error status.  */
        _ux_host_semaphore_put(&configuration -> ux_configuration_device -> ux_device_protection_semaphore);
        return(status);
    }
    
    /* Return to caller.  */
    return(status);
}

