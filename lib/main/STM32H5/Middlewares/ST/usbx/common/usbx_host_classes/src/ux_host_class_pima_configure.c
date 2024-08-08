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
/**   PIMA Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_pima.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_pima_configure                       PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function calls the USBX stack to do a SET_CONFIGURATION to the */
/*    pima. Once the pima is configured, its interface will be            */ 
/*    activated. The bulk endpoints (1 IN, 1 OUT ) and the interrupt      */ 
/*    endpoint are enumerated.                                            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                     Pointer to pima class      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_configuration_interface_get  Get interface           */ 
/*    _ux_host_stack_device_configuration_get     Get configuration       */ 
/*    _ux_host_stack_device_configuration_select  Select configuration    */  
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_pima_activate                Pima class activate     */ 
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
UINT  _ux_host_class_pima_configure(UX_HOST_CLASS_PIMA *pima)
{

UINT                    status;
UX_CONFIGURATION        *configuration;
#if UX_MAX_DEVICES > 1
UX_DEVICE               *parent_device;
#endif


    /* If the device has been configured already, we don't need to do it
       again. */
    if (pima -> ux_host_class_pima_device -> ux_device_state == UX_DEVICE_CONFIGURED)
        return(UX_SUCCESS);

    /* A pima normally has one configuration. So retrieve the 1st configuration
       only.  */
    status =  _ux_host_stack_device_configuration_get(pima -> ux_host_class_pima_device, 0, &configuration);
    if (status != UX_SUCCESS)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONFIGURATION_HANDLE_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONFIGURATION_HANDLE_UNKNOWN, pima -> ux_host_class_pima_device, 0, 0, UX_TRACE_ERRORS, 0, 0)
    
        return(UX_CONFIGURATION_HANDLE_UNKNOWN);
    }

#if UX_MAX_DEVICES > 1
    /* Check the pima power source and check the parent power source for 
       incompatible connections.  */
    if (pima -> ux_host_class_pima_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED)
    {

        /* Get parent device pointer.  */
        parent_device =  pima -> ux_host_class_pima_device -> ux_device_parent;
        
        /* If the device is NULL, the parent is the root pima and we don't have to worry 
           if the parent is not the root pima, check for its power source.  */
        if ((parent_device != UX_NULL) && (parent_device -> ux_device_power_source == UX_DEVICE_BUS_POWERED))
        {                        

            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CONNECTION_INCOMPATIBLE);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONNECTION_INCOMPATIBLE, pima, 0, 0, UX_TRACE_ERRORS, 0, 0)

            return(UX_CONNECTION_INCOMPATIBLE);
        }            
    }
#endif

    /* We have the valid configuration. Ask the USBX stack to set this configuration.  */        
    status =  _ux_host_stack_device_configuration_select(configuration);
    if (status != UX_SUCCESS)
        return(status);

    /* If the operation went well, the pima default alternate setting for the pima interface is 
       active and the interrupt endpoint is now enabled. We have to memorize the first interface since 
       the interrupt endpoint is hooked to it. */
    status =  _ux_host_stack_configuration_interface_get(configuration, 0, 0, &pima -> ux_host_class_pima_interface);
    if (status != UX_SUCCESS)
    {

        /* Store the instance in the interface container, this is for the USB stack
           when it needs to invoke the class.  */        
        pima -> ux_host_class_pima_interface -> ux_interface_class_instance =  (VOID *) pima;
    }

    /* Return completion status.  */
    return(status);
}

