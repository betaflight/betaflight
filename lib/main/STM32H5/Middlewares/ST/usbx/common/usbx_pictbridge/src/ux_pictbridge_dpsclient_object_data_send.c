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
/**   Pictbridge Application                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_pictbridge.h"
#include "ux_device_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpsclient_object_data_send           PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function accepts the data of the object.                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                   Pima instance associated     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    user application                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used macros for RTOS calls, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpsclient_object_data_send(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle, 
                                                ULONG phase, 
                                                UCHAR *object_buffer, 
                                                ULONG object_offset,
                                                ULONG object_length)
{
UINT                            status;
UX_PICTBRIDGE                   *pictbridge;
UX_SLAVE_CLASS_PIMA_OBJECT      *object_info;
ULONG                           event_flag = 0;
UCHAR                           *pima_object_buffer;


    /* Get the pointer to the Pictbridge instance.  */
    pictbridge = (UX_PICTBRIDGE *)pima -> ux_device_class_pima_application;
    
    /* Get the pointer to the pima object.  */
    object_info = (UX_SLAVE_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_host;

    /* Is this the correct handle ? */
    if (object_info -> ux_device_class_pima_object_handle_id == object_handle)
    {
    
        /* Get the pointer to the object buffer.  */
        pima_object_buffer = object_info -> ux_device_class_pima_object_buffer;
        
        /* Check the phase. We should wait for the object to be completed and the response sent back
           before parsing the object.  */
        if (phase == UX_DEVICE_CLASS_PIMA_OBJECT_TRANSFER_PHASE_ACTIVE)
        {

            /* Check if it will fit in our buffer.  */
            if (object_offset + object_length > UX_PICTBRIDGE_MAX_PIMA_OBJECT_BUFFER)
                return(UX_MEMORY_INSUFFICIENT);

            /* Copy the demanded object data portion.  */
            _ux_utility_memory_copy(pima_object_buffer + object_offset, object_buffer, object_length); /* Use case of memcpy is verified. */
        
            /* Save the length of this object.  */
            object_info -> ux_device_class_pima_object_length = object_length;
        
            /* We are not done yet.  */
            return(UX_SUCCESS);
        }
        
        /* We now have the entire script into memory. Parse the object and execute the script.  */
        status = _ux_pictbridge_object_parse(pictbridge, pima_object_buffer,
                                    object_info -> ux_device_class_pima_object_compressed_size);
        
        /* Analyze the status from the parsing and determine the result code to be sent back.  */
        switch (status)
        {        
            case UX_SUCCESS                                     :
        
                /* Set the result code to OK.  */
                pictbridge -> ux_pictbridge_operation_result =  UX_PICTBRIDGE_ACTION_RESULT_OK;
                break;
        
            case UX_PICTBRIDGE_ERROR_SCRIPT_SYNTAX_ERROR        :
        
                /* Set the result code to Error.  */
                pictbridge -> ux_pictbridge_operation_result =  UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_IP;
                break;
            
            case UX_PICTBRIDGE_ERROR_PARAMETER_UNKNOWN          :
        
                /* Set the result code to Error.  */
                pictbridge -> ux_pictbridge_operation_result =  UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_UP;
                break;
            
            case UX_PICTBRIDGE_ERROR_PARAMETER_MISSING          :
        
                /* Set the result code to Error.  */
                pictbridge -> ux_pictbridge_operation_result =  UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_MP;
                break;
            
            default :
        
                /* Set the result code to Error.  */
                pictbridge -> ux_pictbridge_operation_result =  UX_PICTBRIDGE_ACTION_RESULT_NOT_SUPPORTED_DEFAULT;
                break;
        }            
        
        /* We may have a complete event. We only raise the event flag if the completion was successful.  */
        if (pictbridge -> ux_pictbridge_operation_result ==  UX_PICTBRIDGE_ACTION_RESULT_OK)
        {

            /* Check what command we got back from the host.  */
            switch (pictbridge -> ux_pictbridge_input_request) 
            {
                        
                case UX_PICTBRIDGE_IR_CONFIGURE_PRINT_SERVICE           :
        
                    /* Set event to configure print service.  */
                    event_flag = UX_PICTBRIDGE_EVENT_FLAG_CONFIGURE_PRINT_SERVICE;
                    break;
                
                case UX_PICTBRIDGE_IR_GET_CAPABILITY                    :
        
                    /* Set event to capability.  */
                    event_flag = UX_PICTBRIDGE_EVENT_FLAG_CAPABILITY;
                    break;

                case UX_PICTBRIDGE_IR_GET_JOB_STATUS                    :

                    /* Set event to capability.  */
                    event_flag = UX_PICTBRIDGE_EVENT_FLAG_JOB_STATUS;
                    break;
                    
                case UX_PICTBRIDGE_IR_GET_DEVICE_STATUS                 :
        
                    /* Set event to device status.  */
                    event_flag = UX_PICTBRIDGE_EVENT_FLAG_DEVICE_STATUS;
                    break;

                case UX_PICTBRIDGE_IR_START_JOB                         :
        
                    /* Set event to start job .  */
                    event_flag = UX_PICTBRIDGE_EVENT_FLAG_START_JOB;
                    break;
                    
                case UX_PICTBRIDGE_IR_ABORT_JOB                         :
                    
                    /* Set event to abort job.  */
                    event_flag = UX_PICTBRIDGE_EVENT_FLAG_ABORT_JOB;
                    break;
                    
                case UX_PICTBRIDGE_IR_CONTINUE_JOB                      :
                    
                    /* Set event to continue job.  */
                    event_flag = UX_PICTBRIDGE_EVENT_FLAG_CONTINUE_JOB;
                    break;
                    
                case UX_PICTBRIDGE_IR_NOTIFY_JOB_STATUS                 :

                    /* Set event to notify job status.  */
                    event_flag = UX_PICTBRIDGE_EVENT_FLAG_NOTIFY_JOB_STATUS;
                    break;
                
                case UX_PICTBRIDGE_IR_NOTIFY_DEVICE_STATUS              :

                    /* Set event to notify device status.  */
                    event_flag = UX_PICTBRIDGE_EVENT_FLAG_NOTIFY_DEVICE_STATUS;
                    break;
                
        
                default                                                 :
                    /* Function not yet supported.  */    
                    break;
            }    
            
            /* Send event to the application. */
            _ux_system_event_flags_set(&pictbridge -> ux_pictbridge_event_flags_group, event_flag, UX_OR);
                        

        }
        else
        {

            /* We have an error of some kind. Wake up the application with error event.  */
            _ux_system_event_flags_set(&pictbridge -> ux_pictbridge_event_flags_group, UX_PICTBRIDGE_EVENT_FLAG_ERROR, UX_OR);
            
        }
        
        /* What cycle are we in ? */
        if (pictbridge -> ux_pictbridge_host_client_state_machine & UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST)
        {

            /* Since we are in client request, this indicates we are done with the cycle.  */
            pictbridge -> ux_pictbridge_host_client_state_machine = UX_PICTBRIDGE_STATE_MACHINE_IDLE;
    
        }            

        /* We have executed the script.  */
        return(UX_SUCCESS);

    }

    /* Could not find the handle.  */
    return(UX_DEVICE_CLASS_PIMA_RC_INVALID_OBJECT_HANDLE);
}

