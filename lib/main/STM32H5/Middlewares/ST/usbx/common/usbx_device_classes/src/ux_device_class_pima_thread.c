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
/**   Device PIMA Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_pima.h"
#include "ux_device_stack.h"


#if !defined(UX_DEVICE_STANDALONE)
/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_pima_thread                        PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the thread of the pima class.                      */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima_class                               Address of pima class      */ 
/*                                                container               */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Request transfer              */ 
/*    _ux_device_stack_endpoint_stall       Stall endpoint                */
/*    _ux_utility_memory_allocate           Allocate memory               */ 
/*    _ux_device_semaphore_create           Create semaphore              */
/*    _ux_device_thread_create              Create thread                 */
/*    _ux_device_thread_suspend             Suspend thread                */ 
/*    _ux_utility_short_get                 Get 16-bit value              */
/*    _ux_utility_long_get                  Get 32-bit value              */
/*    _ux_device_class_pima_device_info_send                              */
/*                                          Send PIMA device info         */
/*    _ux_device_class_pima_storage_id_send Send PIMA storage ID          */
/*    _ux_device_class_pima_storage_info_get                              */
/*                                          Get PIMA storage info get     */
/*    _ux_device_class_pima_objects_number_send                           */
/*                                          Send number of PIMA objects   */
/*    _ux_device_class_pima_object_handles_send                           */
/*                                          Send PIMA object handlers     */
/*    _ux_device_class_pima_object_info_get Get PIMA object info          */
/*    _ux_device_class_pima_object_data_get Get PIMA object data          */
/*    _ux_device_class_pima_object_delete   Delete PIMA object            */
/*    _ux_device_class_pima_object_info_send                              */
/*                                          Send PIMA object info         */
/*    _ux_device_class_pima_object_data_send                              */
/*                                          Send PIMA object data         */
/*    _ux_device_class_pima_storage_format  Format storage                */
/*    _ux_device_class_pima_device_reset    Reset device                  */
/*    _ux_device_class_pima_object_props_supported_get                    */
/*                                          Get support PIMA object       */
/*                                          properties                    */
/*    _ux_device_class_pima_object_prop_desc_get                          */
/*                                          Get PIMA object property      */
/*                                          descriptor                    */
/*    _ux_device_class_pima_object_prop_value_get                         */
/*                                          Get PIMA object property value*/
/*    _ux_device_class_pima_object_prop_value_set                         */
/*                                          Set PIMA object property value*/
/*    _ux_device_class_pima_object_references_get                         */
/*                                          Get PIMA object references    */
/*    _ux_device_class_pima_device_prop_desc_get                          */
/*                                          Get PIMA device property      */
/*                                          descriptor                    */
/*    _ux_device_class_pima_device_prop_value_get                         */
/*                                          Get PIMA device property value*/
/*    _ux_device_class_pima_device_prop_value_set                         */
/*                                          Set PIMA device property value*/
/*    _ux_device_class_pima_response_send   Send PIMA response            */
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
/*                                            refined macros names,       */
/*                                            updated phase states,       */
/*                                            refined internal function,  */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_pima_thread(ULONG pima_class)
{

UX_SLAVE_CLASS              *class_ptr;
UX_SLAVE_CLASS_PIMA         *pima;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_TRANSFER           *transfer_request;
UCHAR                       *pima_command;
ULONG                       pima_command_code;
ULONG                       pima_parameter_1;
ULONG                       pima_parameter_2;
ULONG                       pima_parameter_3;
UINT                        status;

    
    /* Cast properly the pima instance.  */
    UX_THREAD_EXTENSION_PTR_GET(class_ptr, UX_SLAVE_CLASS, pima_class)
    
    /* Get the pima instance from this class container.  */
    pima =  (UX_SLAVE_CLASS_PIMA *) class_ptr -> ux_slave_class_instance;
    
    /* Allocate some memory for the thread stack. */
    pima -> ux_device_class_pima_interrupt_thread_stack =  
            _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_THREAD_STACK_SIZE);

    /* Check for successful allocation.  */
    if (pima -> ux_device_class_pima_interrupt_thread_stack == UX_NULL)
    {

        /* Do not proceed.  */
        return;
    }        

    /* Allocate a semaphore to this thread.  */
    status =  _ux_device_semaphore_create(&pima -> ux_device_class_pima_interrupt_thread_semaphore,
                                          "ux_device_class_interrupt_thread_semaphore", 0);

    /* Check completion status.  */
    if (status != UX_SUCCESS)
    {

        /* Do not proceed.  */
        return;
    }        
    
    /* The Pima device class needs 2 threads, one is activated by default for the command\response and one needs to be
       created here for the interrupt pipe event.  */
    status =  _ux_device_thread_create(&pima -> ux_device_class_pima_interrupt_thread, "ux_slave_class_thread_pima_interrupt", 
                _ux_device_class_pima_interrupt_thread,
                (ULONG) (ALIGN_TYPE) pima, (VOID *) pima -> ux_device_class_pima_interrupt_thread_stack,
                UX_THREAD_STACK_SIZE, UX_THREAD_PRIORITY_CLASS,
                UX_THREAD_PRIORITY_CLASS, UX_NO_TIME_SLICE, UX_AUTO_START);
            
    /* Check the creation of this thread.  */
    if (status != UX_SUCCESS)
    {

        /* Do not proceed.  */
        return;
    }        

    UX_THREAD_EXTENSION_PTR_SET(&(pima -> ux_device_class_pima_interrupt_thread), pima)

    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {


        /* Get the pointer to the device.  */
        device =  &_ux_system_slave -> ux_system_slave_device;
        
        /* All PIMA commands are on the endpoint OUT, from the host.  */
        transfer_request =  &pima -> ux_device_class_pima_bulk_out_endpoint -> ux_slave_endpoint_transfer_request;
        
        /* As long as the device is in the CONFIGURED state.  */
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        { 
        
            /* Phase idle.  */
            pima -> ux_device_class_pima_state = UX_DEVICE_CLASS_PIMA_PHASE_IDLE;

            /* Send the request to the device controller.  */
            status =  _ux_device_stack_transfer_request(transfer_request, 64, 64);
     
            /* Check the status */    
            if (status == UX_SUCCESS)
            {

                /* Obtain the buffer address containing the PIMA command.  */
                pima_command =  transfer_request -> ux_slave_transfer_request_data_pointer;
                
                /* Check to make sure we have a command block.  */
                if (_ux_utility_short_get(pima_command + UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_TYPE) == UX_DEVICE_CLASS_PIMA_CT_COMMAND_BLOCK)
                {

                    /* Save the transaction ID.  */
                    pima -> ux_device_class_pima_transaction_id = _ux_utility_long_get(pima_command + 
                                                                    UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_TRANSACTION_ID);

                    /* Retrieve the command stored in the command block.  */
                    pima_command_code = _ux_utility_short_get(pima_command + UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_CODE);
                    
                    /* Retrieve the parameter 1.  */
                    pima_parameter_1 = _ux_utility_long_get(pima_command + UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_PARAMETER_1);
                    
                    /* Retrieve the parameter 2.  */
                    pima_parameter_2 = _ux_utility_long_get(pima_command + UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_PARAMETER_2);
                    
                    /* Retrieve the parameter 3.  */
                    pima_parameter_3 = _ux_utility_long_get(pima_command + UX_DEVICE_CLASS_PIMA_COMMAND_HEADER_PARAMETER_3);
                    
                    /* Phase command.  */
                    pima -> ux_device_class_pima_state = UX_DEVICE_CLASS_PIMA_PHASE_COMMAND;

                    /* We check first if this is a GET_DEVICE_INFO as this is the only command which does not require 
                       a session to be opened.  */
                    
                    switch (pima_command_code)
                    {
                    
                        case UX_DEVICE_CLASS_PIMA_OC_GET_DEVICE_INFO :

                            /* Return the device info to the host.  */
                            status = _ux_device_class_pima_device_info_send(pima);
                            break;                            

                        case UX_DEVICE_CLASS_PIMA_OC_OPEN_SESSION               :       
                        
                            /* If the first parameter is 0x00000000,
                                 the operation should fail with a response of Invalid_Parameter.
                               If a session is already open, and the device does not support multiple sessions,
                                 the response Session_Already_Open should be returned,
                                 with the SessionID of the already open session as the first response parameter.
                               The response Session_Already_Open should also be used if the device supports multiple sessions,
                                 but a session with that ID is already open.
                               If the device supports multiple sessions, and the maximum number of sessions are open,
                                 the device should respond with Device_Busy  */
                            if (pima_parameter_1 == 0)
                            {
                                _ux_device_class_pima_response_send(pima,
                                            UX_DEVICE_CLASS_PIMA_RC_INVALID_PARAMETER, 0, 0, 0, 0);
                                break;
                            }
                            /* Check if session is already opened.  */
                            if (pima -> ux_device_class_pima_session_id != 0)
                            {
                                _ux_device_class_pima_response_send(pima,
                                            UX_DEVICE_CLASS_PIMA_RC_SESSION_ALREADY_OPENED,
                                            pima -> ux_device_class_pima_session_id, 0, 0, 0);
                                break;
                            }
                        
                                /* Session can be opened.  */
                                pima -> ux_device_class_pima_session_id =  pima_parameter_1;
                            pima -> ux_device_class_pima_transaction_id = 0;
                            _ux_device_class_pima_response_send(pima,
                                        UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);
                            break;

                        default :
                        
                            /* Check if a session is opened.  */
                            if (pima -> ux_device_class_pima_session_id == 0)
                            {
                            
                                /* We cannot proceed since the session is not opened.  */
                                _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_SESSION_NOT_OPEN, 0, 0, 0, 0);
                            }
                            else
                            {
                        
                                /* Analyze the command stored in the command block.  */
                                switch (pima_command_code)
                                {
                                    
                                    case UX_DEVICE_CLASS_PIMA_OC_CLOSE_SESSION              :       
                                    
                                        /* We close the session. Return OK.  */
                                        _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OK, 0, 0, 0, 0);
                            
                                        /* Session is now closed.  */
                                        pima -> ux_device_class_pima_session_id = 0;
                                    break;

                                    case UX_DEVICE_CLASS_PIMA_OC_GET_STORAGE_IDS            :           

                                        /* Return the array of storage IDs to the host.  In this version, we support
                                           only one storage media.  */
                                        status = _ux_device_class_pima_storage_id_send(pima);
                                        break;                            
                                        
                                    case UX_DEVICE_CLASS_PIMA_OC_GET_STORAGE_INFO           :           

                                        /* Return the storage info to the host.  */
                                        status = _ux_device_class_pima_storage_info_get(pima, pima_parameter_1);
                                        break;                            

                                    case UX_DEVICE_CLASS_PIMA_OC_GET_NUM_OBJECTS            :           

                                        /* Return the number of objects found in the system.  */
                                        status = _ux_device_class_pima_objects_number_send(pima, 
                                                                        pima_parameter_1, pima_parameter_2, pima_parameter_3);
                                        break;                            
                                    
                                    case UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_HANDLES         :           

                                        /* Return the object handles found in the system.  */
                                        status = _ux_device_class_pima_object_handles_send(pima, 
                                                                        pima_parameter_1, pima_parameter_2, pima_parameter_3);
                                        
                                        break;                            
                                    
                                    case UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_INFO            :           
                                    
                                        /* Return the object info data set.  */
                                        status = _ux_device_class_pima_object_info_get(pima, pima_parameter_1);
                                        break;                            
                                    
                                    case UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT                 :       

                                        /* Return the object data.  */
                                        status = _ux_device_class_pima_object_data_get(pima, pima_parameter_1);
                                        break;                            
                                    
                                    case UX_DEVICE_CLASS_PIMA_OC_DELETE_OBJECT              :       

                                        /* Delete one or more objects.  */
                                    status = _ux_device_class_pima_object_delete(pima, pima_parameter_1, pima_parameter_2);
                                        break;                            
                                    
                                    case UX_DEVICE_CLASS_PIMA_OC_SEND_OBJECT_INFO           :           

                                        /* Accept an object info data set.  */
                                        status = _ux_device_class_pima_object_info_send(pima, pima_parameter_1, pima_parameter_2);
                                        break;                            
                                    
                                    case UX_DEVICE_CLASS_PIMA_OC_SEND_OBJECT                :       
                                        /* Accept the object data.  */
                                        status = _ux_device_class_pima_object_data_send(pima);
                                        break;                            
                                    
                                    case UX_DEVICE_CLASS_PIMA_OC_GET_PARTIAL_OBJECT         :           

                                        /* Return the partial object data.  */
                                        status = _ux_device_class_pima_partial_object_data_get(pima, pima_parameter_1, pima_parameter_2, pima_parameter_3);
                                        break;                            

                                    case UX_DEVICE_CLASS_PIMA_OC_FORMAT_STORE               :       

                                        /* Format the storage device. This calls the application to reset all object handles stored
                                           on the media.   */
                                        status = _ux_device_class_pima_storage_format(pima, pima_parameter_1);
                                        break;

                                    case UX_DEVICE_CLASS_PIMA_OC_RESET_DEVICE               :       

                                        /* Reset the device. This calls the application to reset the device. The session is closed
                                           but all objects retain their properties.  */
                                        status = _ux_device_class_pima_device_reset(pima);
                                        break;
                                        

                                    case UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROPS_SUPPORTED :       

                                        /* Return an Object Property Code array of supported object properties for the object format that is indicated 
                                        in the first parameter.  */
                                        status = _ux_device_class_pima_object_props_supported_get(pima, pima_parameter_1);
                                        break;

                                    case UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROP_DESC       :       

                                        /* Returns the appropriate property that describes the dataset that is indicated in the first parameter.  */
                                        status = _ux_device_class_pima_object_prop_desc_get(pima, pima_parameter_1, pima_parameter_2);
                                        break;

                                    case UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_PROP_VALUE      :       

                                        /* Returns the Object property value.  */
                                        status = _ux_device_class_pima_object_prop_value_get(pima, pima_parameter_1, pima_parameter_2);
                                        break;

                                    case UX_DEVICE_CLASS_PIMA_OC_SET_OBJECT_PROP_VALUE      :       

                                        /* Sets the current value of the object property.  */
                                        status = _ux_device_class_pima_object_prop_value_set(pima, pima_parameter_1, pima_parameter_2);
                                        break;

                                    case UX_DEVICE_CLASS_PIMA_OC_GET_OBJECT_REFERENCES      :       

                                        /* Returns the object handle references.  */
                                        status = _ux_device_class_pima_object_references_get(pima, pima_parameter_1);
                                        break;

                                    case UX_DEVICE_CLASS_PIMA_OC_SET_OBJECT_REFERENCES      :       

                                        /* Set the object handle references.  */
                                        status = _ux_device_class_pima_object_references_set(pima, pima_parameter_1);
                                        break;

                                    case UX_DEVICE_CLASS_PIMA_OC_GET_DEVICE_PROP_DESC       :       

                                        /* Returns the appropriate device property.  */
                                        status = _ux_device_class_pima_device_prop_desc_get(pima, pima_parameter_1);
                                        break;

                                    case UX_DEVICE_CLASS_PIMA_OC_GET_DEVICE_PROP_VALUE      :       

                                        /* Returns the device property value.  */
                                        status = _ux_device_class_pima_device_prop_value_get(pima, pima_parameter_1);
                                        break;

                                    case UX_DEVICE_CLASS_PIMA_OC_SET_DEVICE_PROP_VALUE      :       

                                        /* Sets the current value of the device property.  */
                                        status = _ux_device_class_pima_device_prop_value_set(pima, pima_parameter_1);
                                        break;

                                    case UX_DEVICE_CLASS_PIMA_OC_INITIATE_OPEN_CAPTURE      :           
                                    case UX_DEVICE_CLASS_PIMA_OC_GET_THUMB                  :       
                                    case UX_DEVICE_CLASS_PIMA_OC_INITIATE_CAPTURE           :       
                                    case UX_DEVICE_CLASS_PIMA_OC_SELF_TEST                  :       
                                    case UX_DEVICE_CLASS_PIMA_OC_SET_OBJECT_PROTECTION      :           
                                    case UX_DEVICE_CLASS_PIMA_OC_POWER_DOWN                 :       
                                    case UX_DEVICE_CLASS_PIMA_OC_RESET_DEVICE_PROP_VALUE    :       
                                    case UX_DEVICE_CLASS_PIMA_OC_TERMINATE_OPEN_CAPTURE     :           
                                    case UX_DEVICE_CLASS_PIMA_OC_MOVE_OBJECT                :       
                                    case UX_DEVICE_CLASS_PIMA_OC_COPY_OBJECT                :       

                                        /* Functions not yet supported.  */
                                        _ux_device_class_pima_response_send(pima, UX_DEVICE_CLASS_PIMA_RC_OPERATION_NOT_SUPPORTED, 0, 0, 0, 0);

                                        /* Set error code.  */
                                        status = UX_FUNCTION_NOT_SUPPORTED;
                                        
                                        break;

                                    default:
            
                                    /* The command is unknown, so we stall the endpoint.  */                                
                                    _ux_device_stack_endpoint_stall(pima -> ux_device_class_pima_bulk_out_endpoint);
                                }

                                /* Check error code. */
                                if (status != UX_SUCCESS)

                                    /* Error trap. */
                                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);

                            }
                        }
                    
                    /* Check error code. */
                    if (status != UX_SUCCESS)

                        /* Error trap. */
                        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);
                }
                else
                
                    /* We have a wrong buffer format. Stall the endpoint.  */                                    
                    _ux_device_stack_endpoint_stall(pima -> ux_device_class_pima_bulk_out_endpoint);
            }
        }

    /* We need to suspend ourselves. We will be resumed by the device enumeration module.  */
    _ux_device_thread_suspend(&class_ptr -> ux_slave_class_thread);
    }
}
#endif
