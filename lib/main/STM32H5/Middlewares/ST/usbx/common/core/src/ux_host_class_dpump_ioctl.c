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
/**   Host Data Pump Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_dpump.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_dpump_ioctl                          PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called by the application to change the alternate  */ 
/*    setting of the dpump class.                                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    dpump                                 Pointer to the dpump class    */ 
/*    ioctl_function                        ioctl function                */ 
/*    parameter                             pointer to structure          */ 
/*                                                                        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_interface_setting_select                             */
/*                                          Select alternate setting      */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
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
UINT  _ux_host_class_dpump_ioctl(UX_HOST_CLASS_DPUMP *dpump, ULONG ioctl_function,
                                    VOID *parameter)
{

UX_CONFIGURATION            *configuration;
UX_INTERFACE                *interface_ptr;
UINT                           status;

    /* Ensure the instance is valid.  */
    if ((dpump -> ux_host_class_dpump_state !=  UX_HOST_CLASS_INSTANCE_LIVE) && 
        (dpump -> ux_host_class_dpump_state !=  UX_HOST_CLASS_INSTANCE_MOUNTING))
    {        

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_HOST_CLASS_INSTANCE_UNKNOWN, dpump, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
    }

    /* The command request will tell us we need to do here.  */
    switch (ioctl_function)
    {

        case UX_HOST_CLASS_DPUMP_SELECT_ALTERNATE_SETTING: 


            /* The parameter value has the alternate setting number. 
               We need to scan the entire device framework.  Only one configuration for data pump device framework.  */
            interface_ptr = dpump -> ux_host_class_dpump_interface;
            configuration = interface_ptr -> ux_interface_configuration;

            /* Do some verification just in case !  */
            if (configuration == UX_NULL)
            {            

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_HOST_CLASS_INSTANCE_UNKNOWN);

                return(UX_HOST_CLASS_INSTANCE_UNKNOWN);
            }
                        
            /* Point to the first interface.  */
            interface_ptr =  configuration -> ux_configuration_first_interface;

            /* Loop on all interfaces and alternate settings for this device in search of the right alternate setting.  */
            while (interface_ptr != UX_NULL)
            {

                /* Check the alternate setting.  */
                if (interface_ptr -> ux_interface_descriptor.bAlternateSetting == (ULONG) (ALIGN_TYPE) parameter)
                {

                    /* We have found the alternate setting. Select it now.  */
                    status =  _ux_host_stack_interface_setting_select(interface_ptr);
                    
                    /* We are done here.  */
                    return(status);
                }
            
                /* Next interface.  */
                interface_ptr = interface_ptr -> ux_interface_next_interface;
            }

            /* We come here when the alternate setting was not found.  */
            status = UX_INTERFACE_HANDLE_UNKNOWN;

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

            break;

        default: 

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

            /* Function not supported. Return an error.  */
            status =  UX_FUNCTION_NOT_SUPPORTED;

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

    }   

    /* Return status to caller.  */
    return(status);
}

