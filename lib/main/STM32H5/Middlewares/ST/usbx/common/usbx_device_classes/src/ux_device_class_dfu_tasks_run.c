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
/**   Device dfu Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_dfu.h"
#include "ux_device_stack.h"


#if defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_dfu_tasks_run                      PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the background task of the dfu class. It waits for */
/*    the dfu command to signal a DFU_DETACH stage and either force a     */
/*    disconnect from the device or wait for the host to detach.          */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    class_instance                              DFU class instance      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_delay_ms                  Delay for some time           */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX                                                                */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_dfu_tasks_run(VOID *class_instance)
{

UX_SLAVE_CLASS_DFU              *dfu;
UX_SLAVE_DCD                    *dcd;
ULONG                           actual_flags;

    /* Get the dfu instance from this class container.  */
    dfu =  (UX_SLAVE_CLASS_DFU *) class_instance;

    /* Process when there is flags.  */
    if (dfu -> ux_device_class_dfu_flags == 0)
        return(UX_STATE_IDLE);
    actual_flags = dfu -> ux_device_class_dfu_flags;

    /* Check the source of event.  */
    if (actual_flags & UX_DEVICE_CLASS_DFU_THREAD_EVENT_DISCONNECT)
    {
        dfu -> ux_device_class_dfu_flags &= ~UX_DEVICE_CLASS_DFU_THREAD_EVENT_DISCONNECT;

        /* We need to disconnect.  The control command for DETACH is still being processed, wait 2-3 ms. */
        _ux_utility_delay_ms(2);

        /* Get the pointer to the DCD.  */
        dcd =  &_ux_system_slave -> ux_system_slave_dcd;

        /* Issue a Soft Disconnect.  */
        dcd -> ux_slave_dcd_function(dcd, UX_DCD_CHANGE_STATE, (VOID *) UX_DEVICE_FORCE_DISCONNECT);

    }

    /* Check the source of event.  */
    if (actual_flags & UX_DEVICE_CLASS_DFU_THREAD_EVENT_WAIT_RESET)
    {
        dfu -> ux_device_class_dfu_flags &= ~UX_DEVICE_CLASS_DFU_THREAD_EVENT_WAIT_RESET;

        /* We need to wait for reset.  Arm a timer.  The timeout value is indicated in ms from
            the device framework.  */
        _ux_utility_delay_ms(_ux_system_slave -> ux_system_slave_device_dfu_detach_timeout);

        /* Check the mode.  */
        if (_ux_system_slave -> ux_system_slave_device_dfu_mode ==  UX_DEVICE_CLASS_DFU_MODE_RUNTIME)

            /* We are still in RunTime mode. The host never reset. Revert to AppIdle state.  */
            _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_APP_IDLE;

    }
    return(UX_STATE_WAIT);
}
#endif
