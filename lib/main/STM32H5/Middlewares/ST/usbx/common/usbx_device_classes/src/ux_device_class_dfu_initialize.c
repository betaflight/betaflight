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
/**                                                                       */ 
/** USBX Component                                                        */ 
/**                                                                       */
/**   Device DFU Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_dfu.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_dfu_initialize                     PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function initializes the USB DFU device.                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to dfu command        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_utility_descriptor_parse          Parse a descriptor            */
/*    _ux_utility_event_flags_create        Create event flags            */
/*    _ux_utility_event_flags_delete        Delete event flags            */
/*    _ux_device_thread_create              Create thread                 */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Source Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed max transfer size,    */
/*                                            added max size limit check, */
/*                                            resulting in version 6.1.6  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_dfu_initialize(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_CLASS_DFU                      *dfu;
UX_SLAVE_CLASS_DFU_PARAMETER            *dfu_parameter;
UX_SLAVE_CLASS                          *class_ptr;
UINT                                    status = UX_DESCRIPTOR_CORRUPTED;
UX_DFU_FUNCTIONAL_DESCRIPTOR            dfu_functional_descriptor;
UCHAR                                   *dfu_framework; 
ULONG                                   dfu_framework_length;
UCHAR                                   descriptor_type;
ULONG                                   descriptor_length;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Create an instance of the device dfu class.  */
    dfu =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_SLAVE_CLASS_DFU));

    /* Check for successful allocation.  */
    if (dfu == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Save the address of the DFU instance inside the DFU container.  */
    class_ptr -> ux_slave_class_instance = (VOID *) dfu;

    /* Get the pointer to the application parameters for the dfu class.  */
    dfu_parameter =  command -> ux_slave_class_command_parameter;

    /* Save the calling parameters in the class instance.  */
    dfu -> ux_slave_class_dfu_instance_activate                 =  dfu_parameter -> ux_slave_class_dfu_parameter_instance_activate;
    dfu -> ux_slave_class_dfu_instance_deactivate               =  dfu_parameter -> ux_slave_class_dfu_parameter_instance_deactivate;
    dfu -> ux_slave_class_dfu_read                              =  dfu_parameter -> ux_slave_class_dfu_parameter_read;
    dfu -> ux_slave_class_dfu_write                             =  dfu_parameter -> ux_slave_class_dfu_parameter_write;
    dfu -> ux_slave_class_dfu_get_status                        =  dfu_parameter -> ux_slave_class_dfu_parameter_get_status;
    dfu -> ux_slave_class_dfu_notify                            =  dfu_parameter -> ux_slave_class_dfu_parameter_notify;
#ifdef UX_DEVICE_CLASS_DFU_CUSTOM_REQUEST_ENABLE
    dfu -> ux_device_class_dfu_custom_request                   =  dfu_parameter -> ux_device_class_dfu_parameter_custom_request;
#endif

    /* Store the device dfu in the project structure.  */
    _ux_system_slave -> ux_system_slave_dfu_framework           =  dfu_parameter -> ux_slave_class_dfu_parameter_framework;
    _ux_system_slave -> ux_system_slave_dfu_framework_length    =  dfu_parameter -> ux_slave_class_dfu_parameter_framework_length;

    /* There is a DFU descriptor.  It has a device descriptor, a configuration descriptor,
       an interface descriptor and finally a functional descriptor. */
    dfu_framework        =  _ux_system_slave -> ux_system_slave_dfu_framework;
    dfu_framework_length =  _ux_system_slave -> ux_system_slave_dfu_framework_length;
    
    /* Parse the device framework and locate interfaces and endpoint descriptor(s).  */
    while (dfu_framework_length != 0)
    {

        /* Get the length of this descriptor.  */
        descriptor_length =  (ULONG) *dfu_framework;

        /* Length validation.  */
        if (descriptor_length < 2 ||
            descriptor_length > dfu_framework_length)
            break;

        /* And its type.  */
        descriptor_type =  *(dfu_framework + 1);

        /* Is this the Functional descriptor ?  */
        if (descriptor_type == UX_DFU_FUNCTIONAL_DESCRIPTOR_ITEM)
        {

            /* Parse the DFU descriptor in something more readable.  */
            _ux_utility_descriptor_parse(dfu_framework,
                        _ux_system_dfu_functional_descriptor_structure,
                        UX_DFU_FUNCTIONAL_DESCRIPTOR_ENTRIES,
                        (UCHAR *) &dfu_functional_descriptor);

            /* Control transfer limit validation.  */
            if (dfu_functional_descriptor.wTransferSize > UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH)
                break;

            /* Retrieve the DFU capabilities and store them in the system.  */
            _ux_system_slave -> ux_system_slave_device_dfu_capabilities = dfu_functional_descriptor.bmAttributes;
            
            /* Retrieve the DFU timeout value. */
            _ux_system_slave -> ux_system_slave_device_dfu_detach_timeout = dfu_functional_descriptor.wDetachTimeOut;
            
            /* Retrieve the DFU transfer size value. */
            _ux_system_slave -> ux_system_slave_device_dfu_transfer_size = dfu_functional_descriptor.wTransferSize;

            /* In the system, state the DFU state machine.  */
            _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_APP_IDLE;

            /* DFU descriptor parsed, done.  */
            status = UX_SUCCESS;
            break;
        }

        /* Adjust what is left of the device framework.  */
        dfu_framework_length -=  descriptor_length;

        /* Point to the next descriptor.  */
        dfu_framework +=  descriptor_length;

    }

#if !defined(UX_DEVICE_STANDALONE)

    /* Create a event flag group for the dfu class to synchronize with the event interrupt thread.  */
    status =  _ux_utility_event_flags_create(&dfu -> ux_slave_class_dfu_event_flags_group, "ux_device_class_dfu_event_flag");

    /* Check status.  */
    if (status != UX_SUCCESS)
        status = UX_EVENT_ERROR;

    /* Allocate some memory for the dfu thread stack. */
    if (status == UX_SUCCESS)
    {
        dfu -> ux_slave_class_dfu_thread_stack =  
                _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);
        
        /* Check for successful allocation.  */
        if (dfu -> ux_slave_class_dfu_thread_stack  == UX_NULL)
            status = UX_MEMORY_INSUFFICIENT;
    }

    /* dfu needs a thread to watch for disconnect and timer event for the DFU_DETACH sequence.  */
    if (status == UX_SUCCESS)
    {
        status =  _ux_device_thread_create(&dfu -> ux_slave_class_dfu_thread , "ux_slave_class_dfu_thread", 
                    _ux_device_class_dfu_thread,
                    (ULONG) (ALIGN_TYPE) class_ptr, (VOID *) dfu -> ux_slave_class_dfu_thread_stack,
                    UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_AUTO_START);

        /* Check the creation of this thread.  */
        if (status != UX_SUCCESS)
            status = UX_THREAD_ERROR;
    }

    UX_THREAD_EXTENSION_PTR_SET(&(dfu -> ux_slave_class_dfu_thread), class_ptr)
#else

    /* Set task function.  */
    class_ptr -> ux_slave_class_task_function = _ux_device_class_dfu_tasks_run;
#endif

    /* Return completion status.  */
    if (status == UX_SUCCESS)
        return(UX_SUCCESS);
    
    /* There is error, free resources.  */

#if !defined(UX_DEVICE_STANDALONE)

    /* The last resource, thread is not created or created error, no need to free.  */
    if (dfu -> ux_slave_class_dfu_thread_stack)
        _ux_utility_memory_free(dfu -> ux_slave_class_dfu_thread_stack);
    if (dfu -> ux_slave_class_dfu_event_flags_group.tx_event_flags_group_id != 0)
        _ux_utility_event_flags_delete(&dfu -> ux_slave_class_dfu_event_flags_group);
#endif

    /* Detach from container and free instance memory.  */
    class_ptr -> ux_slave_class_instance = UX_NULL;
    _ux_utility_memory_free(dfu);

    return(status);
}

