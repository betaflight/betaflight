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
/**   HID Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_stack.h"


#if defined(UX_HOST_STANDALONE)


static inline VOID _ux_host_class_hid_inst_tasks_run(UX_HOST_CLASS_HID *hid);

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_hid_tasks_run                        PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function runs HID background tasks.                            */
/*                                                                        */
/*    This function is for standalone mode.                               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hid                                       Pointer to hid instance   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_host_class_hid_client_function)   Client task function          */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Host Stack                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_tasks_run(UX_HOST_CLASS *hid_class)
{
UX_HOST_CLASS_HID           *hid;

    /* Validate class entry.  */
    if (hid_class -> ux_host_class_status != UX_USED ||
        hid_class -> ux_host_class_entry_function != _ux_host_class_hid_entry)
        return(UX_STATE_IDLE);

    /* Run for class instances.  */
    hid = (UX_HOST_CLASS_HID *)hid_class -> ux_host_class_first_instance;
    while(hid)
    {

        /* Run tasks for each hid instance.  */
        if ((hid -> ux_host_class_hid_flags & UX_HOST_CLASS_HID_FLAG_PROTECT) == 0)
        {
            hid -> ux_host_class_hid_flags |= UX_HOST_CLASS_HID_FLAG_PROTECT;
            _ux_host_class_hid_inst_tasks_run(hid);
            hid -> ux_host_class_hid_flags &= ~UX_HOST_CLASS_HID_FLAG_PROTECT;
        }
        hid = hid -> ux_host_class_hid_next_instance;
    }
    return(UX_STATE_WAIT);
}

static inline VOID _ux_host_class_hid_inst_tasks_run(UX_HOST_CLASS_HID *hid)
{
UX_HOST_CLASS_HID_CLIENT        *client = hid -> ux_host_class_hid_client;
    if (client && client -> ux_host_class_hid_client_function)
    {
        client -> ux_host_class_hid_client_function(client);
    }
}
#endif
