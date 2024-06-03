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


#if !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_dfu_thread                         PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the thread of the dfu class. It waits for the      */
/*    dfu command to signal a DFU_DETACH stage and either force a         */ 
/*    disconnect from the device or wait for the host to detach.          */ 
/*                                                                        */
/*    It's for RTOS mode.                                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    dfu_class                                   Address of dfu class    */ 
/*                                                container               */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_event_flags_get           Get event flags               */ 
/*    _ux_utility_delay_ms                  Delay in milliseconds         */ 
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
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_dfu_thread(ULONG dfu_class)
{

UX_SLAVE_CLASS                  *class_ptr;
UX_SLAVE_CLASS_DFU              *dfu;
UX_SLAVE_DCD                    *dcd;
UINT                            status;
ULONG                           actual_flags;

    /* Cast properly the dfu instance.  */
    UX_THREAD_EXTENSION_PTR_GET(class_ptr, UX_SLAVE_CLASS, dfu_class)
    
    /* Get the dfu instance from this class container.  */
    dfu =  (UX_SLAVE_CLASS_DFU *) class_ptr -> ux_slave_class_instance;
    
    /* This thread runs forever.  */
    while(1)
    {
        

        /* Wait until we have a event sent by the application. */
        status =  _ux_utility_event_flags_get(&dfu -> ux_slave_class_dfu_event_flags_group, (UX_DEVICE_CLASS_DFU_THREAD_EVENT_DISCONNECT |
                                                                                        UX_DEVICE_CLASS_DFU_THREAD_EVENT_WAIT_RESET), 
                                                                                        UX_OR_CLEAR, &actual_flags, UX_WAIT_FOREVER);
        
        /* Check the completion code and the actual flags returned. */
        if (status == UX_SUCCESS)
        {
    
            /* Check the source of event.  */
            if (actual_flags & UX_DEVICE_CLASS_DFU_THREAD_EVENT_DISCONNECT)
            {
            
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
            
                /* We need to wait for reset.  Arm a timer.  The timeout value is indicated in ms from
                   the device framework.  */
                _ux_utility_delay_ms(_ux_system_slave -> ux_system_slave_device_dfu_detach_timeout);

                /* Check the mode.  */
                if (_ux_system_slave -> ux_system_slave_device_dfu_mode ==  UX_DEVICE_CLASS_DFU_MODE_RUNTIME)
                
                    /* We are still in RunTime mode. The host never reset. Revert to AppIdle state.  */
                    _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_APP_IDLE;
                    
            } 
        }
    }         
}
#endif
