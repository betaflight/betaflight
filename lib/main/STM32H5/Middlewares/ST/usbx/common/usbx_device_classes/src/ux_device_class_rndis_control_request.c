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
/**   Device RNDIS Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_rndis.h"
#include "ux_device_stack.h"


#if UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH < UX_DEVICE_CLASS_RNDIS_INTERRUPT_RESPONSE_LENGTH ||\
    UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH < UX_DEVICE_CLASS_RNDIS_MAX_CONTROL_RESPONSE_LENGTH
#error UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH too small, please check
#endif

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_rndis_control_request              PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function manages the based sent by the host on the control     */ 
/*    endpoints with a CLASS or VENDOR SPECIFIC type.                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    rndis                           Pointer to rndis class              */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_endpoint_stall       Endpoint stall                */ 
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_utility_memory_set                Set memory                    */
/*    _ux_utility_memory_copy               Copy memory                   */
/*    _ux_utility_long_get                  Get 32-bit value              */
/*    _ux_device_event_flags_set            Set event flags               */
/*    _ux_device_class_rndis_msg_initialize Command initialize            */
/*    _ux_device_class_rndis_msg_query      Command query                 */
/*    _ux_device_class_rndis_msg_set        Command set                   */
/*    _ux_device_class_rndis_msg_reset      Command reset                 */
/*    _ux_device_class_rndis_msg_keep_alive Command keep alive            */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    RNDIS Class                                                         */
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
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_rndis_control_request(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_TRANSFER       *transfer_request;
UX_SLAVE_TRANSFER       *transfer_request_in;
UX_SLAVE_DEVICE         *device;
ULONG                   request;
ULONG                   request_length;
ULONG                   rndis_command;
UX_SLAVE_CLASS          *class_ptr;
UX_SLAVE_CLASS_RNDIS    *rndis;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the pointer to the transfer request associated with the control endpoint.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Extract all necessary fields of the request.  */
    request =  *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);
    request_length =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;
    
    /* Get the rndis instance from this class container.  */
    rndis =  (UX_SLAVE_CLASS_RNDIS *) class_ptr -> ux_slave_class_instance;
    
    /* Here we proceed only the standard request we know of at the device level.  */
    switch (request)
    {

            
        case UX_DEVICE_CLASS_RNDIS_SEND_ENCAPSULATED_COMMAND    :
        
            
            /* We have received a command. Check if the command is valid and dispatch it.  */
            rndis_command = _ux_utility_long_get(transfer_request -> ux_slave_transfer_request_data_pointer);

            switch (rndis_command)
            {

                case    UX_DEVICE_CLASS_RNDIS_MSG_INITIALIZE            :
                
                    _ux_device_class_rndis_msg_initialize(rndis, transfer_request);
                    break;
                    
                case    UX_DEVICE_CLASS_RNDIS_MSG_HALT                  :
                    break;
                    
                
                case    UX_DEVICE_CLASS_RNDIS_MSG_QUERY                 :
                    _ux_device_class_rndis_msg_query(rndis, transfer_request);
                    break;
                    
                
                case    UX_DEVICE_CLASS_RNDIS_MSG_SET                   :
                    _ux_device_class_rndis_msg_set(rndis, transfer_request);
                    break;
                    
                
                case    UX_DEVICE_CLASS_RNDIS_MSG_RESET                 :
                    _ux_device_class_rndis_msg_reset(rndis, transfer_request);
                    break;
                    
                
                case    UX_DEVICE_CLASS_RNDIS_MSG_INDICATE_STATUS    :
                    break;
                    
                
                case    UX_DEVICE_CLASS_RNDIS_MSG_KEEP_ALIVE            :
                    _ux_device_class_rndis_msg_keep_alive(rndis, transfer_request);
                    break;
                    
                default                                                :

                    /* Unknown function. Stall the endpoint.  */
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                    break;

            }
            
            /* Check the return status. If no error, we set the interrupt pipe to reply response available.  
              All RNDIS events are on the interrupt endpoint IN, from the host.  */
            transfer_request_in =  &rndis -> ux_slave_class_rndis_interrupt_endpoint -> ux_slave_endpoint_transfer_request;

            /* Reset the buffer.  */
            _ux_utility_memory_set(transfer_request_in -> ux_slave_transfer_request_data_pointer, 0, UX_DEVICE_CLASS_RNDIS_INTERRUPT_RESPONSE_LENGTH); /* Use case of memset is verified. */

            /* Set the buffer of this transfer request with the flag for response available.  */
            transfer_request_in -> ux_slave_transfer_request_data_pointer[0] = UX_DEVICE_CLASS_RNDIS_INTERRUPT_RESPONSE_AVAILABLE_FLAG;

            /* Set an event to wake up the interrupt thread.  */
            _ux_device_event_flags_set(&rndis -> ux_slave_class_rndis_event_flags_group, UX_DEVICE_CLASS_RNDIS_NEW_INTERRUPT_EVENT, UX_OR);                
            
            break;
            
        case UX_DEVICE_CLASS_RNDIS_GET_ENCAPSULATED_RESPONSE    :
        
           
            /* Copy the response into the request data buffer.  */
            _ux_utility_memory_copy(transfer_request -> ux_slave_transfer_request_data_pointer, rndis -> ux_slave_class_rndis_response,
                                    rndis -> ux_slave_class_rndis_response_length); /* Use case of memcpy is verified. */

            /* Set the phase of the transfer to data out.  */
            transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_OUT;

            /* We can return the RNDIS response.  */
            _ux_device_stack_transfer_request(transfer_request, rndis -> ux_slave_class_rndis_response_length, request_length);
        
            break ;
            
        default:

            /* Unknown function. It's not handled.  */
            return(UX_ERROR);
    }

    /* It's handled.  */
    return(UX_SUCCESS);
}

