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
/**   Device Stack                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_stack.h"


#if defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                 RELEASE      */
/*                                                                        */
/*    _ux_device_stack_tasks_run                            PORTABLE C    */
/*                                                             6.1.10     */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function runs device stack and registered classes tasks.       */
/*                                                                        */
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    UX_STATE_RESET                        Tasks suspended               */
/*    UX_STATE_IDLE                         Activated but no task ran     */
/*    (others > UX_STATE_IDLE)              Tasks running                 */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_slave_dcd_function)               run DCD function              */
/*    (ux_slave_class_task_function)        run Class tasks function      */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_tasks_run(VOID)
{

UX_SLAVE_DCD                *dcd;
UX_SLAVE_CLASS              *class_instance;
ULONG                       class_index;
UINT                        status;


    status = UX_STATE_RESET;

    /* Run all DCD tasks (pending ISR handle).  */
    dcd = &_ux_system_slave -> ux_system_slave_dcd;
    dcd -> ux_slave_dcd_function(dcd, UX_DCD_TASKS_RUN, UX_NULL);

    /* Run all Class instance tasks.  */
    class_instance =  _ux_system_slave -> ux_system_slave_class_array;
    for (class_index = 0; class_index < UX_SYSTEM_DEVICE_MAX_CLASS_GET(); class_index++)
    {

        /* Skip classes not used.  */
        if (class_instance -> ux_slave_class_status == UX_UNUSED)
            continue;

        /* Skip classes has no task function.  */
        if (class_instance -> ux_slave_class_task_function == UX_NULL)
            continue;

        /* Invoke task function.  */
        status |= class_instance -> ux_slave_class_task_function(class_instance -> ux_slave_class_instance);

        /* Move to the next class.  */
        class_instance ++;
    }

    /* Return overall status.  */
    return(status);
}
#endif
