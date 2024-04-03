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
/**   Device CDC_ECM Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_cdc_ecm.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_cdc_ecm_entry                      PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the entry point of the cdc_ecm class. It           */ 
/*    will be called by the device stack enumeration module when the      */ 
/*    host has sent a SET_CONFIGURATION command and the cdc_ecm interface */
/*    needs to be mounted.                                                */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    command                               Pointer to class command      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_class_cdc_ecm_initialize       Initialize cdc_ecm class  */
/*    _ux_device_class_cdc_ecm_uninitialize     Uninitialize cdc_ecm class*/
/*    _ux_device_class_cdc_ecm_activate         Activate cdc_ecm class    */ 
/*    _ux_device_class_cdc_ecm_deactivate       Deactivate cdc_ecm class  */ 
/*    _ux_device_class_cdc_ecm_change           Alternate setting change  */
/*    _ux_device_class_cdc_ecm_control_request  Request control           */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    CDC_ECM Class                                                       */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_cdc_ecm_entry(UX_SLAVE_CLASS_COMMAND *command)
{

UINT        status;

    /* The command request will tell us we need to do here, either a enumeration
       query, an activation or a deactivation.  */
    switch (command -> ux_slave_class_command_request)
    {

    case UX_SLAVE_CLASS_COMMAND_INITIALIZE:

        /* Call the init function of the CDC_ECM class.  */
        status =  _ux_device_class_cdc_ecm_initialize(command);
        
        /* Return the completion status.  */
        return(status);

    case UX_SLAVE_CLASS_COMMAND_UNINITIALIZE:

        /* Call the init function of the CDC_ECM class.  */
        status =  _ux_device_class_cdc_ecm_uninitialize(command);
        
        /* Return the completion status.  */
        return(status);

    case UX_SLAVE_CLASS_COMMAND_QUERY:

        /* Check the CLASS definition in the interface descriptor. */
        if (command -> ux_slave_class_command_class == UX_DEVICE_CLASS_CDC_ECM_CLASS_COMMUNICATION_CONTROL || 
                command -> ux_slave_class_command_class == UX_DEVICE_CLASS_CDC_ECM_CLASS_COMMUNICATION_DATA)
            return(UX_SUCCESS);
        else
            return(UX_NO_CLASS_MATCH);

    case UX_SLAVE_CLASS_COMMAND_ACTIVATE:

        /* The activate command is used when the host has sent a SET_CONFIGURATION command
           and this interface has to be mounted. In CDC ECM, the alternate setting 0 has no endpoints.  
           Only the Alternate Setting 1 has the Bulk IN and OUT endpoints active.  */
        status =  _ux_device_class_cdc_ecm_activate(command);

        /* Return the completion status.  */
        return(status);
        
    case UX_SLAVE_CLASS_COMMAND_CHANGE:

        /* The change command is used when the host has sent a SET_INTERFACE command
           to go from Alternate Setting 0 to 1 or revert to the default mode.  */
        status =  _ux_device_class_cdc_ecm_change(command);

        /* Return the completion status.  */
        return(status);
        

    case UX_SLAVE_CLASS_COMMAND_DEACTIVATE:

        /* The deactivate command is used when the device has been extracted.
           The device endpoints have to be dismounted and the cdc_ecm thread canceled.  */
        status =  _ux_device_class_cdc_ecm_deactivate(command);
        
        /* Return the completion status.  */
        return(status);

    case UX_SLAVE_CLASS_COMMAND_REQUEST:

        /* The request command is used when the host sends a command on the control endpoint.  */
        status = _ux_device_class_cdc_ecm_control_request(command);

        /* Return the completion status.  */
        return(status);

    default: 

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Return an error.  */
        return(UX_FUNCTION_NOT_SUPPORTED);
    }   
}

