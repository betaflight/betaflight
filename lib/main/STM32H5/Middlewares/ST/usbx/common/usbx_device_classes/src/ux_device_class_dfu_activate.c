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
/**   Device DFU Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_dfu.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_dfu_activate                       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function initializes the USB DFU device.                       */ 
/*    This class can be activated either as part of the device primary    */ 
/*    framework or after a PORT_RESET detected.                           */ 
/*    This is detected through the protocol field. If 1, we are in the    */ 
/*    device regular mode. If 2, we are activated through the DFU         */ 
/*    mode.                                                               */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to dfu command        */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate memory               */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Source Code                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            removed block count (it's   */
/*                                            from host request wValue),  */
/*                                            resulting in version 6.1.6  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_dfu_activate(UX_SLAVE_CLASS_COMMAND *command)
{
                                          
UX_SLAVE_INTERFACE                      *interface_ptr;         
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_CLASS_DFU                      *dfu;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    dfu = (UX_SLAVE_CLASS_DFU *) class_ptr -> ux_slave_class_instance;

    /* Get the interface that owns this instance.  */
    interface_ptr =  (UX_SLAVE_INTERFACE  *) command -> ux_slave_class_command_interface;
    
    /* Store the class instance into the interface.  */
    interface_ptr -> ux_slave_interface_class_instance =  (VOID *)dfu;
         
    /* Now the opposite, store the interface in the class instance.  */
    dfu -> ux_slave_class_dfu_interface =  interface_ptr;

    /* Check the protocol activation field to determine in which state of the DFU class
       we are.  */
    switch (command -> ux_slave_class_command_protocol)
    {
    
        case UX_SLAVE_CLASS_DFU_PROTOCOL_RUNTIME    :
        
            /* In the system, state the DFU state machine to application idle.  */
            _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_APP_IDLE;

            /* Set the mode to Runtime.  */
            _ux_system_slave -> ux_system_slave_device_dfu_mode =  UX_DEVICE_CLASS_DFU_MODE_RUNTIME ;

            break;


        case UX_SLAVE_CLASS_DFU_PROTOCOL_DFU_MODE    :
        
            /* In the system, state the DFU state machine to DFU idle.  */
            _ux_system_slave -> ux_system_slave_device_dfu_state_machine = UX_SYSTEM_DFU_STATE_DFU_IDLE;

            /* Set the mode to DFU mode.  */
            _ux_system_slave -> ux_system_slave_device_dfu_mode =  UX_DEVICE_CLASS_DFU_MODE_DFU ;

            break;       

        default :
        
            /* We should never get here.  */
            return(UX_ERROR);
            
    }
    
    
    /* If there is a activate function call it.  */
    if (dfu -> ux_slave_class_dfu_instance_activate != UX_NULL)
    {        
        /* Invoke the application.  */
        dfu -> ux_slave_class_dfu_instance_activate(dfu);
    }

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_DFU_ACTIVATE, dfu, 0, 0, 0, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, dfu, 0, 0, 0)

    /* Return completion status.  */
    return(UX_SUCCESS);
}

