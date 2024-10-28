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
/**   Utility                                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"


#if !defined(UX_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_utility_semaphore_get                           PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function gets a semaphore signal.                              */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    semaphore                             Semaphore to get signal from  */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    tx_thread_identify                    ThreadX identify thread       */
/*    tx_thread_info_get                    ThreadX get thread info       */
/*    tx_semaphore_get                      ThreadX semaphore get         */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Components                                                     */ 
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            off in standalone build,    */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_semaphore_get(UX_SEMAPHORE *semaphore, ULONG semaphore_signal)
{

UINT        status;
UX_THREAD   *my_thread;
CHAR        *name;
UINT        state;
ULONG       run_count;
UINT        priority;
UINT        preemption_threshold;
ULONG       time_slice;
UX_THREAD   *next_thread;
UX_THREAD   *suspended_thread;

    /* Call TX to know my own tread.  */
    my_thread = tx_thread_identify();

    /* Retrieve information about the previously created thread "my_thread." */
    tx_thread_info_get(my_thread, &name, &state, &run_count,
                       &priority, &preemption_threshold,
                       &time_slice, &next_thread,&suspended_thread);

    /* Is this the lowest priority thread in the system trying to use TX services ? */
    if (priority > _ux_system -> ux_system_thread_lowest_priority)
    {

        /* We need to remember this thread priority.  */
        _ux_system -> ux_system_thread_lowest_priority = priority;
        
    }

    /* Get ThreadX semaphore instance.  */
    status =  tx_semaphore_get(semaphore, semaphore_signal);

    /* Return completion status.  */
    return(status);
}
#endif
