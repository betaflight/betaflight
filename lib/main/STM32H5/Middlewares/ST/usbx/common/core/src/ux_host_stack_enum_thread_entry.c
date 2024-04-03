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
/*    _ux_host_stack_enum_thread_entry                    PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains the enum thread for USBX. It is in charge of     */
/*    the topology changes either from device insertion\extraction on     */ 
/*    the root hub or on a regular hub.                                   */
/*                                                                        */
/*    This thread ensures we never have more that 2 instances trying to   */
/*    perform a change to the topology (mostly enumeration) for fear that */ 
/*    more than one device could answer to address 0.                     */
/*                                                                        */
/*    This function is the entry point of the topology thread. It waits   */ 
/*    until one of the HCDs or a hub sets the semaphore to indicate       */
/*    there has been a change in the USB topology which could be either   */
/*    a insertion or extraction or eventually a hub downstream port       */ 
/*    signal.                                                             */
/*                                                                        */
/*    This is for RTOS mode.                                              */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    input                                 Not used input                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_rh_change_process      Root hub processing           */ 
/*    _ux_utility_semaphore_get             Get signal semaphore          */ 
/*    (ux_system_host_enum_hub_function)    HUB enum processing function  */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    ThreadX                                                             */ 
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
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_stack_enum_thread_entry(ULONG input)
{

    UX_PARAMETER_NOT_USED(input);

    /* Loop forever waiting for changes signaled through the semaphore. */     
    while (1)
    {   

        /* Wait for the semaphore to be put by the root hub or a regular hub.  */
        _ux_host_semaphore_get_norc(&_ux_system_host -> ux_system_host_enum_semaphore, UX_WAIT_FOREVER);

#if UX_MAX_DEVICES > 1
        /* We try the hub first. For this we look into the USBX project
           structure to see if there is at least one hub.  */
        if (_ux_system_host -> ux_system_host_enum_hub_function != UX_NULL)
        {

            /* Yes, there is a HUB function, call it!  */
            _ux_system_host -> ux_system_host_enum_hub_function();
        }
#endif

        /* The signal may be also coming from the root hub, call the root hub handler.  */
        _ux_host_stack_rh_change_process();
    }
}

