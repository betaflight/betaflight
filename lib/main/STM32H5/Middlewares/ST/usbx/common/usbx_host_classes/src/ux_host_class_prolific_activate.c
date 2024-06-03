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
/**   Prolific Class                                                      */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_prolific.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_prolific_activate                    PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function creates the prolific instance, configure the device.  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                                DLC  class command pointer   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_prolific_configure        Configure prolific class   */ 
/*    _ux_host_class_prolific_endpoints_get    Get endpoints of prolific  */
/*    _ux_host_class_prolific_setup            Set up prolific device     */ 
/*    _ux_host_stack_class_instance_create     Create class instance      */ 
/*    _ux_host_stack_class_instance_destroy    Destroy the class instance */ 
/*    _ux_utility_memory_allocate              Allocate memory block      */ 
/*    _ux_utility_memory_free                  Free memory block          */ 
/*    _ux_host_semaphore_create                Create prolific semaphore  */
/*    _ux_host_class_prolific_ioctl            IOCTL function for DLC     */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_prolific_entry            Entry of prolific class    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_prolific_activate(UX_HOST_CLASS_COMMAND *command)
{

UX_DEVICE                           *device;
UX_HOST_CLASS_PROLIFIC              *prolific;
UX_HOST_CLASS_PROLIFIC_LINE_CODING  line_coding;
UX_HOST_CLASS_PROLIFIC_LINE_STATE   line_state;
UINT                                status;

    /* The prolific class is always activated by the device descriptor. */
    device =  (UX_DEVICE *) command -> ux_host_class_command_container;

    /* Obtain memory for this class instance.  */
    prolific =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_CACHE_SAFE_MEMORY, sizeof(UX_HOST_CLASS_PROLIFIC));
    if (prolific == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Store the class container into this instance.  */
    prolific -> ux_host_class_prolific_class =  command -> ux_host_class_command_class_ptr;

    /* Store the device container into the prolific class instance.  */
    prolific -> ux_host_class_prolific_device =  device;

    /* Store the instance in the device container, this is for the USBX stack
       when it needs to invoke the class for deactivation.  */        
    device -> ux_device_class_instance =  (VOID *) prolific;

    /* Create this class instance.  */
    _ux_host_stack_class_instance_create(prolific -> ux_host_class_prolific_class, (VOID *) prolific);

    /* Configure the prolific.  */
    status =  _ux_host_class_prolific_configure(prolific);     

    /* Get the prolific endpoint(s). We need to search for Bulk Out and Bulk In endpoints 
       and the interrupt endpoint.  */
    if (status == UX_SUCCESS)
        status =  _ux_host_class_prolific_endpoints_get(prolific);

    /* Go on if success.  */
    if (status == UX_SUCCESS)
    {

        /* Store chip version for further reference.  */
        prolific -> ux_host_class_prolific_version = device -> ux_device_descriptor.bcdDevice; 

        /* Mark the prolific instance as mounting now.  */
        prolific -> ux_host_class_prolific_state =  UX_HOST_CLASS_INSTANCE_MOUNTING;

        /* The prolific chip needs to be setup properly.  */
        status = _ux_host_class_prolific_setup(prolific);
    }

    /* Set the default values to the device, first line coding.  */
    if (status == UX_SUCCESS)
    {
        line_coding.ux_host_class_prolific_line_coding_dter      = UX_HOST_CLASS_PROLIFIC_LINE_CODING_DEFAULT_RATE;
        line_coding.ux_host_class_prolific_line_coding_stop_bit  = UX_HOST_CLASS_PROLIFIC_LINE_CODING_STOP_BIT_0;
        line_coding.ux_host_class_prolific_line_coding_parity    = UX_HOST_CLASS_PROLIFIC_LINE_CODING_PARITY_NONE;
        line_coding.ux_host_class_prolific_line_coding_data_bits = UX_HOST_CLASS_PROLIFIC_LINE_CODING_DEFAULT_DATA_BIT;
        status = _ux_host_class_prolific_ioctl(prolific, UX_HOST_CLASS_PROLIFIC_IOCTL_SET_LINE_CODING, (VOID *) &line_coding);
    }

    /* Set the default values to the device, line state.  For the Prolific chip to detect disconnection
       and reconnection, RTS is low.  */
    if (status == UX_SUCCESS)
    {
        line_state.ux_host_class_prolific_line_state_rts      = 0;
        line_state.ux_host_class_prolific_line_state_dtr       = 1;
        status = _ux_host_class_prolific_ioctl(prolific, UX_HOST_CLASS_PROLIFIC_IOCTL_SET_LINE_STATE, (VOID *) &line_state);
    }
    
    /* Create the semaphore to protect 2 threads from accessing the same prolific instance.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_host_semaphore_create(&prolific -> ux_host_class_prolific_semaphore, "ux_host_class_prolific_semaphore", 1);
        if (status != UX_SUCCESS)
            status = UX_SEMAPHORE_ERROR;
    }

    /* Success things.  */
    if (status == UX_SUCCESS)
    {

        /* Mark the prolific instance as live now.  */
        prolific -> ux_host_class_prolific_state =  UX_HOST_CLASS_INSTANCE_LIVE;

        /* If all is fine and the device is mounted, we may need to inform the application
        if a function has been programmed in the system structure.  */
        if (_ux_system_host -> ux_system_host_change_function != UX_NULL)
        {
            
            /* Call system change function.  */
            _ux_system_host ->  ux_system_host_change_function(UX_DEVICE_INSERTION, prolific -> ux_host_class_prolific_class, (VOID *) prolific);
        }

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_PROLIFIC_ACTIVATE, prolific, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

        /* If trace is enabled, register this object.  */
        UX_TRACE_OBJECT_REGISTER(UX_TRACE_HOST_OBJECT_TYPE_INTERFACE, prolific, 0, 0, 0)

        /* Return success.  */
        return(UX_SUCCESS);
    }

    /* There was a problem during the configuration, so free the resources.  */
    /* The last resource, semaphore is not created or created error, no need to free.  */
    _ux_host_stack_class_instance_destroy(prolific -> ux_host_class_prolific_class, (VOID *) prolific);
    device -> ux_device_class_instance = UX_NULL;
    _ux_utility_memory_free(prolific);

    /* Return completion status.  */
    return(status);    
}

