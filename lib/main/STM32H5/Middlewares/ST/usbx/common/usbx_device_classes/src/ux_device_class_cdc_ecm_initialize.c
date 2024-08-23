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
/**   Device CDC_ECM Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_cdc_ecm.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_cdc_ecm_initialize                 PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB CDC_ECM device.                   */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to cdc_ecm command    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_allocate           Allocate memory               */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_utility_mutex_create              Create Mutex                  */
/*    _ux_device_mutex_delete               Delete Mutex                  */
/*    _ux_utility_event_flags_create        Create Flag group             */
/*    _ux_utility_event_flags_delete        Delete Flag group             */
/*    _ux_device_thread_create              Create Thread                 */
/*    _ux_device_thread_delete              Delete Thread                 */
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
/*                                            verified memset and memcpy  */
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*  10-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            removed internal NX pool,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_cdc_ecm_initialize(UX_SLAVE_CLASS_COMMAND *command)
{
#if defined(UX_DEVICE_STANDALONE)
    UX_PARAMETER_NOT_USED(command);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UX_SLAVE_CLASS_CDC_ECM                          *cdc_ecm;
UX_SLAVE_CLASS_CDC_ECM_PARAMETER                *cdc_ecm_parameter;
UX_SLAVE_CLASS                                  *class_ptr;
UINT                                            status;


    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Create an instance of the device cdc_ecm class.  */
    cdc_ecm =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_SLAVE_CLASS_CDC_ECM));

    /* Check for successful allocation.  */
    if (cdc_ecm == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Create a mutex to protect the CDC_ECM thread and the application messing up the transmit queue.  */
    status =  _ux_utility_mutex_create(&cdc_ecm -> ux_slave_class_cdc_ecm_mutex, "ux_slave_class_cdc_ecm_mutex");
    if (status != UX_SUCCESS)
    {
        _ux_utility_memory_free(cdc_ecm);
        return(UX_MUTEX_ERROR);
    }

    /* Assume good result.  */
    status = UX_SUCCESS;

    /* Allocate some memory for the bulk out thread stack. */
    cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread_stack =
            _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);
    if (cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread_stack == UX_NULL)
        status = (UX_MEMORY_INSUFFICIENT);

    /* Allocate some memory for the interrupt thread stack. */
    if (status == UX_SUCCESS)
    {
        cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread_stack =
                _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);

        /* Check for successful allocation.  */
        if (cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread_stack  == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }

    /* Allocate some memory for the bulk in thread stack. */
    if (status == UX_SUCCESS)
    {
        cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread_stack =
                _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);

        /* Check for successful allocation.  */
        if (cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread_stack == UX_NULL)
            status = (UX_MEMORY_INSUFFICIENT);
    }

    /* Interrupt endpoint treatment needs to be running in a different thread. So start
       a new thread. We pass a pointer to the cdc_ecm instance to the new thread.  This thread
       does not start until we have a instance of the class. */
    if (status == UX_SUCCESS)
    {
        status =  _ux_device_thread_create(&cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread , "ux_slave_class_cdc_ecm_interrupt_thread",
                    _ux_device_class_cdc_ecm_interrupt_thread,
                    (ULONG) (ALIGN_TYPE) class_ptr, (VOID *) cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread_stack ,
                    UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
        if (status != UX_SUCCESS)
            status = (UX_THREAD_ERROR);
    }

    UX_THREAD_EXTENSION_PTR_SET(&(cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread), class_ptr)

    /* Check the creation of this thread.  */
    if (status == UX_SUCCESS)
    {

        /* Bulk endpoint treatment needs to be running in a different thread. So start
        a new thread. We pass a pointer to the cdc_ecm instance to the new thread.  This thread
        does not start until we have a instance of the class. */
        status =  _ux_device_thread_create(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread , "ux_slave_class_cdc_ecm_bulkout_thread",
                    _ux_device_class_cdc_ecm_bulkout_thread,
                    (ULONG) (ALIGN_TYPE) class_ptr, (VOID *) cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread_stack ,
                    UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                    UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
        if (status != UX_SUCCESS)
            status = (UX_THREAD_ERROR);
        else
        {

            UX_THREAD_EXTENSION_PTR_SET(&(cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread), class_ptr)

            /* Bulk endpoint treatment needs to be running in a different thread. So start
            a new thread. We pass a pointer to the cdc_ecm instance to the new thread.  This thread
            does not start until we have a instance of the class. */
            status =  _ux_device_thread_create(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread , "ux_slave_class_cdc_ecm_bulkin_thread",
                        _ux_device_class_cdc_ecm_bulkin_thread,
                        (ULONG) (ALIGN_TYPE) class_ptr, (VOID *) cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread_stack ,
                        UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                        UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_DONT_START);
            if (status != UX_SUCCESS)
                status = (UX_THREAD_ERROR);
            else
            {

                UX_THREAD_EXTENSION_PTR_SET(&(cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread), class_ptr)

                /* Create a event flag group for the cdc_ecm class to synchronize with the event interrupt thread.  */
                status =  _ux_utility_event_flags_create(&cdc_ecm -> ux_slave_class_cdc_ecm_event_flags_group, "ux_device_class_cdc_ecm_event_flag");
                if (status != UX_SUCCESS)
                    status = (UX_EVENT_ERROR);
                else
                {

                    /* Save the address of the CDC_ECM instance inside the CDC_ECM container.  */
                    class_ptr -> ux_slave_class_instance = (VOID *) cdc_ecm;

                    /* Get the pointer to the application parameters for the cdc_ecm class.  */
                    cdc_ecm_parameter =  command -> ux_slave_class_command_parameter;

                    /* Store the start and stop signals if needed by the application.  */
                    cdc_ecm -> ux_slave_class_cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_activate = cdc_ecm_parameter -> ux_slave_class_cdc_ecm_instance_activate;
                    cdc_ecm -> ux_slave_class_cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_deactivate = cdc_ecm_parameter -> ux_slave_class_cdc_ecm_instance_deactivate;

                    /* Copy the local node ID.  */
                    _ux_utility_memory_copy(cdc_ecm -> ux_slave_class_cdc_ecm_local_node_id, cdc_ecm_parameter -> ux_slave_class_cdc_ecm_parameter_local_node_id,
                                            UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH); /* Use case of memcpy is verified. */

                    /* Copy the remote node ID.  */
                    _ux_utility_memory_copy(cdc_ecm -> ux_slave_class_cdc_ecm_remote_node_id, cdc_ecm_parameter -> ux_slave_class_cdc_ecm_parameter_remote_node_id,
                                            UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH); /* Use case of memcpy is verified. */

                    /* Store the rest of the parameters as they are in the local instance.  */
                    _ux_utility_memory_copy(&cdc_ecm -> ux_slave_class_cdc_ecm_parameter, cdc_ecm_parameter, sizeof (UX_SLAVE_CLASS_CDC_ECM_PARAMETER)); /* Use case of memcpy is verified. */

                    return(UX_SUCCESS);
                }

                _ux_device_thread_delete(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread);
            }

            _ux_device_thread_delete(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread);
        }

        _ux_device_thread_delete(&cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread);
    }

    /* Free allocated resources.  */

    if (cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread_stack)
        _ux_utility_memory_free(cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread_stack);
    if (cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread_stack)
        _ux_utility_memory_free(cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread_stack);
    if (cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread_stack)
        _ux_utility_memory_free(cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread_stack);
    _ux_device_mutex_delete(&cdc_ecm -> ux_slave_class_cdc_ecm_mutex);
    _ux_utility_memory_free(cdc_ecm);

    /* Return completion status.  */
    return(status);
#endif
}
