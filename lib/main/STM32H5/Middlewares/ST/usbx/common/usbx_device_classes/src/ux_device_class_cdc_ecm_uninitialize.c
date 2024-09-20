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
/**   Device CDC-ECM Class                                                */
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
/*    _ux_device_class_cdc_ecm_uninitialize               PORTABLE C      */ 
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function deinitializes the resources for the specified CDC-ECM */ 
/*    instance.                                                           */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to storage command    */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_mutex_delete               Delete mutex                  */ 
/*    _ux_device_thread_delete              Delete thread                 */ 
/*    _ux_utility_memory_free               Free memory                   */ 
/*    _ux_utility_event_flags_delete        Delete event flags            */ 
/*    _ux_device_semaphore_delete           Delete semaphore              */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device CDC-ECM Class                                                */ 
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
UINT  _ux_device_class_cdc_ecm_uninitialize(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_CLASS_CDC_ECM                  *cdc_ecm;
UX_SLAVE_CLASS                          *class_ptr;


    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    cdc_ecm = (UX_SLAVE_CLASS_CDC_ECM *) class_ptr -> ux_slave_class_instance;
    
    /* Sanity check.  */
    if (cdc_ecm != UX_NULL)
    {

        /* Deinitialize resources. We do not check if they have been allocated
           because if they weren't, the class register (called by the application)
           would have failed.  */

#if !defined(UX_DEVICE_STANDALONE)

        /* Delete the xmit queue mutex.  */
        _ux_device_mutex_delete(&cdc_ecm -> ux_slave_class_cdc_ecm_mutex);

        /* Delete bulk out thread .  */
        _ux_device_thread_delete(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread);

        /* Free bulk out thread stack.  */
        _ux_utility_memory_free(cdc_ecm -> ux_slave_class_cdc_ecm_bulkout_thread_stack);

        /* Delete interrupt thread.  */
        _ux_device_thread_delete(&cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread);

        /* Free interrupt thread stack.  */
        _ux_utility_memory_free(cdc_ecm -> ux_slave_class_cdc_ecm_interrupt_thread_stack);

        /* Delete bulk in thread.  */
        _ux_device_thread_delete(&cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread);

        /* Free bulk in thread stack.  */
        _ux_utility_memory_free(cdc_ecm -> ux_slave_class_cdc_ecm_bulkin_thread_stack);

        /* Delete the interrupt thread sync event flags group.  */
        _ux_device_event_flags_delete(&cdc_ecm -> ux_slave_class_cdc_ecm_event_flags_group);

#endif

        /* Free the resources.  */
        _ux_utility_memory_free(cdc_ecm);
    }
    
    /* Return completion status.  */
    return(UX_SUCCESS);
}
