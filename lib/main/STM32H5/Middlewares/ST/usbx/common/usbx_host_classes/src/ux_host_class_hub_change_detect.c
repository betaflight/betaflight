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
/**   HUB Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hub.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hub_change_detect                    PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called by the enumeration thread when there has    */ 
/*    been activity on the HUB.                                           */ 
/*                                                                        */
/*    In standalone mode there is nothing to do here, activities are      */
/*    processed in hub tasks function.                                    */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_class_hub_change_process     Process HUB change            */ 
/*    _ux_host_stack_class_get              Get class                     */ 
/*    _ux_host_stack_class_instance_get     Get class instance            */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HUB Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_hub_change_detect(VOID)
{
#if defined(UX_HOST_STANDALONE)

    /* Things are done in hub task nothing to do here.  */
#else

UX_HOST_CLASS           *class;
UX_HOST_CLASS_HUB       *hub;
UINT                    status;
UINT                    class_index;

    /* Get the class container first.  */
    _ux_host_stack_class_get(_ux_system_host_class_hub_name, &class);

    /* We start with the first index of the class instance.  */
    class_index =  0;

    /* We have found the class, now parse the instances.  */
    do
    {

        /* Get class instance.  */
        status =  _ux_host_stack_class_instance_get(class, class_index++, (VOID **) &hub);

        /* Check completion status.  */
        if (status == UX_SUCCESS)
        {

            /* We have found an instance of a HUB, check if it is live and if the HUB has 
               detected a change before we proceed.  */
            if (hub -> ux_host_class_hub_change_semaphore != 0)
            {

                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_CLASS_HUB_CHANGE_DETECT, hub, 0, 0, 0, UX_TRACE_HOST_CLASS_EVENTS, 0, 0)

                /* Call the HUB function that will diagnose the origin of the change.  */
                _ux_host_class_hub_change_process(hub);

                /* Decrement the HUB instance semaphore change so we don't get awaken again.  */
                hub -> ux_host_class_hub_change_semaphore--;
            }
        }
    } while (status == UX_SUCCESS);    

    /* We have parsed all the HUB instances.  */
    return;
#endif
}
