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
/**   Device CCID Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_ccid.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_ccid_uninitialize                  PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function uninitialize the USB CCID device.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    command                               Pointer to ccid command       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_free               Free memory                   */
/*    _ux_device_mutex_delete               Delete mutex                  */
/*    _ux_utility_event_flags_delete        Delete event flags            */
/*    _ux_utility_thread_delete             Delete thread                 */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Source Code                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  04-25-2022     Chaoqiong Xiao           Initial Version 6.1.11        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_ccid_uninitialize(UX_SLAVE_CLASS_COMMAND *command)
{

UX_DEVICE_CLASS_CCID                    *ccid;
UX_SLAVE_CLASS                          *ccid_class;
#if !defined(UX_DEVICE_STANDALONE)
UX_DEVICE_CLASS_CCID_RUNNER             *runner;
ULONG                                   i;
#endif

    /* Get the class container.  */
    ccid_class =  command -> ux_slave_class_command_class_ptr;

    /* Get the CCID instance.  */
    ccid = (UX_DEVICE_CLASS_CCID *)ccid_class -> ux_slave_class_instance;

    /* Sanity check.  */
    if (ccid != UX_NULL)
    {

        /* Free allocated resources.  */
#if !defined(UX_DEVICE_STANDALONE)
        _ux_device_thread_delete(&ccid -> ux_device_class_ccid_notify_thread);
        _ux_device_thread_delete(&ccid -> ux_device_class_ccid_thread);
        for (i = 0;
            i < ccid -> ux_device_class_ccid_parameter.ux_device_class_ccid_max_n_busy_slots;
            i ++)
        {
            runner = &ccid -> ux_device_class_ccid_runners[i];
            _ux_device_thread_delete(&runner -> ux_device_class_ccid_runner_thread);
        }
        _ux_device_event_flags_delete(&ccid -> ux_device_class_ccid_events);
        _ux_device_mutex_delete(&ccid -> ux_device_class_ccid_mutex);
        _ux_device_semaphore_delete(&ccid -> ux_device_class_ccid_notify_semaphore);
        _ux_device_mutex_delete(&ccid -> ux_device_class_ccid_response_mutex);
#endif

        /* Free instance memory.  */
        _ux_utility_memory_free(ccid);
    }

    /* Return completion status.  */
    return(UX_SUCCESS);
}
