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
/*    _ux_host_stack_uninitialize                         PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function uninitializes all the host code for USBX to work on a */
/*    specific platform.                                                  */
/*                                                                        */
/*    Note following steps must be done before host stack uninitialize:   */
/*    All HCDs must be unregistered (devices also removed).               */
/*    All classes unregistered (clients attached also removed).           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*    UX_SUCCESS                            Uninitialized success         */
/*                                                                        */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_memory_free               Free host memory              */
/*    _ux_utility_thread_delete             Delete host thread            */
/*    _ux_utility_semaphore_delete          Delete host semaphore         */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application                                                         */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  09-30-2020     Chaoqiong Xiao           Initial Version 6.1           */
/*  08-02-2021     Xiuwen Cai               Modified comment(s),          */
/*                                            fixed compile issue,        */
/*                                            resulting in version 6.1.8  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_stack_uninitialize(VOID)
{

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_UNINITIALIZE, 0, 0, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

#if !defined(UX_HOST_STANDALONE)
    /* Delete enumeration thread.  */
    _ux_utility_thread_delete(&_ux_system_host -> ux_system_host_enum_thread);

    /* Delete enumeration semaphore.  */
    _ux_utility_semaphore_delete(&_ux_system_host -> ux_system_host_enum_semaphore);

    /* Free enumeration thread stack.  */
    _ux_utility_memory_free(_ux_system_host -> ux_system_host_enum_thread_stack);

    /* Delete HCD thread.  */
    _ux_utility_thread_delete(&_ux_system_host -> ux_system_host_hcd_thread);

    /* Delete HCD semaphore.  */
    _ux_utility_semaphore_delete(&_ux_system_host -> ux_system_host_hcd_semaphore);

    /* Free HCD thread stack.  */
    _ux_utility_memory_free(_ux_system_host -> ux_system_host_hcd_thread_stack);
#endif

#if defined(UX_OTG_SUPPORT) && !defined(UX_OTG_STANDALONE)

    /* Delete HNP thread.  */
    _ux_utility_thread_delete(&_ux_system_host -> ux_system_host_hnp_polling_thread);

    /* Free HNP thread stack.  */
    _ux_utility_memory_free(_ux_system_host -> ux_system_host_hnp_polling_thread_stack);
#endif

    /* Free HCD array.  */
    _ux_utility_memory_free(_ux_system_host -> ux_system_host_hcd_array);

    /* Free Class array.  */
    _ux_utility_memory_free(_ux_system_host -> ux_system_host_class_array);

    /* Free Device array.  */
    _ux_utility_memory_free(_ux_system_host -> ux_system_host_device_array);

    /* Return success to caller.  */
    return(UX_SUCCESS);
}
