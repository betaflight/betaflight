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
/*    _ux_host_class_pima_activate                        PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the ACM instance, configure the device ...    */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                                Pima class command pointer   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_pima_configure           Configure pima class        */ 
/*    _ux_host_class_pima_endpoints_get       Get endpoints of pima       */ 
/*    _ux_host_stack_class_instance_create    Create class instance       */ 
/*    _ux_host_stack_class_instance_destroy   Destroy the class instance  */ 
/*    _ux_utility_memory_allocate             Allocate memory block       */ 
/*    _ux_utility_memory_free                 Free memory block           */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_pima_entry               Entry of pima class         */ 
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
UINT  _ux_host_class_pima_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_INTERFACE                        *interface_ptr;
UX_HOST_CLASS_PIMA                  *pima;
UINT                                 status;

    /* The PIMA class is always activated by the interface descriptor and not the
       device descriptor.  */
    interface_ptr =  (UX_INTERFACE *) command -> ux_host_class_command_container;

    /* Obtain memory for this class instance.  */
    pima =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HOST_CLASS_PIMA));
    if (pima == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Allocate some DMA safe memory for sending/receiving headers.  */
    pima -> ux_host_class_pima_container =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PIMA_CONTAINER_SIZE);
    if (pima -> ux_host_class_pima_container == UX_NULL)
        status = UX_MEMORY_INSUFFICIENT;
    else
        status = UX_SUCCESS;

    /* Allocate some DMA safe memory for receiving pima events.  */
    if (status == UX_SUCCESS)
    {

        pima -> ux_host_class_pima_event_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PIMA_AEI_MAX_LENGTH);
        if (pima -> ux_host_class_pima_event_buffer == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
    }

    /* Go on if no error.  */
    if (status == UX_SUCCESS)
    {

        /* Store the class container into this instance.  */
        pima -> ux_host_class_pima_class =  command -> ux_host_class_command_class_ptr;

        /* Store the interface container into the pima class instance.  */
        pima -> ux_host_class_pima_interface =  interface_ptr;

        /* Store the device container into the pima class instance.  */
        pima -> ux_host_class_pima_device =  interface_ptr -> ux_interface_configuration -> ux_configuration_device;

        /* This instance of the device must also be stored in the interface container.  */
        interface_ptr -> ux_interface_class_instance =  (VOID *) pima;

        /* Create this class instance.  */
        _ux_host_stack_class_instance_create(pima -> ux_host_class_pima_class, (VOID *) pima);

        /* Configure the pima.  */
        status =  _ux_host_class_pima_configure(pima);
    }

    /* Get the pima endpoint(s). We will need to search for Bulk Out, Bulk In and interrupt endpoints.  */
    if (status == UX_SUCCESS)
        status =  _ux_host_class_pima_endpoints_get(pima);

    /* Success things.  */
    if (status == UX_SUCCESS)
    {

        /* Mark the pima as live now.  */
        pima -> ux_host_class_pima_state =  UX_HOST_CLASS_INSTANCE_LIVE;

        /* If all is fine and the device is mounted, we may need to inform the application
        if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {
            
            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, pima -> ux_host_class_pima_class, (VOID *) pima);
        }

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PIMA_ACTIVATE, pima, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, pima, 0, 0, 0)

        /* Return success.  */
        return(UX_SUCCESS);
    }

    /* Free existing resources.  */
    if (pima -> ux_host_class_pima_event_buffer)
    {
        _ux_host_stack_class_instance_destroy(pima -> ux_host_class_pima_class, (VOID *) pima);
        interface_ptr -> ux_interface_class_instance = UX_NULL;
        _ux_utility_memory_free(pima -> ux_host_class_pima_event_buffer);
    }

    if (pima -> ux_host_class_pima_container)
        _ux_utility_memory_free(pima -> ux_host_class_pima_container);

    _ux_utility_memory_free(pima);

    /* Return completion status.  */
    return(status);    
}

