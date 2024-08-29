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
/**   Device CDC Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_cdc_acm_control_request            PORTABLE C      */ 
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
/*    cdc_acm                               Pointer to cdc_acm class      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    CDC Class                                                           */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_cdc_acm_control_request(UX_SLAVE_CLASS_COMMAND *command)
{
UX_SLAVE_CLASS_CDC_ACM                  *cdc_acm;
UX_SLAVE_CLASS                          *class_ptr;
UX_SLAVE_TRANSFER                       *transfer_request;
UX_SLAVE_DEVICE                         *device;
ULONG                                   request;
ULONG                                   value;
ULONG                                   request_length;
ULONG                                   transmit_length;

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;

    /* Get the class instance in the container.  */
    cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *) class_ptr -> ux_slave_class_instance;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the pointer to the transfer request associated with the control endpoint.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Extract all necessary fields of the request.  */
    request =  *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

    /* Extract all necessary fields of the value.  */
    value =  _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE);

    /* Pickup the request length.  */
    request_length =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

    transmit_length = request_length ;
    
    /* Here we proceed only the standard request we know of at the device level.  */
    switch (request)
    {

        case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE:

            /* Reset current line state values. */
            cdc_acm -> ux_slave_class_cdc_acm_data_dtr_state = 0;
            cdc_acm -> ux_slave_class_cdc_acm_data_rts_state = 0;

            /* Get the line state parameters from the host.  DTR signal. */
            if (value & UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_DTR)
                cdc_acm -> ux_slave_class_cdc_acm_data_dtr_state = UX_TRUE;               

            /* Get the line state parameters from the host.  RTS signal. */
            if (value & UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_RTS)
                cdc_acm -> ux_slave_class_cdc_acm_data_rts_state = UX_TRUE;               
                
            /* If there is a parameter change function call it.  */
            if (cdc_acm -> ux_slave_class_cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change != UX_NULL)
            {        
        
                /* Invoke the application.  */
                cdc_acm -> ux_slave_class_cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change(cdc_acm);
            }

            break ;

        case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING:

            /* Setup the length appropriately.  */
            if (request_length >  UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_RESPONSE_SIZE) 
                transmit_length = UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_RESPONSE_SIZE;
    
            /* Send the line coding default parameters back to the host.  */
            _ux_utility_long_put(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_BAUDRATE_STRUCT, 
                                    cdc_acm -> ux_slave_class_cdc_acm_baudrate);
            *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_STOP_BIT_STRUCT) = cdc_acm -> ux_slave_class_cdc_acm_stop_bit;
            *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARITY_STRUCT)   = cdc_acm -> ux_slave_class_cdc_acm_parity;
            *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_DATA_BIT_STRUCT) = cdc_acm -> ux_slave_class_cdc_acm_data_bit;

            /* Set the phase of the transfer to data out.  */
            transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_OUT;
            
            /* Perform the data transfer.  */
            _ux_device_stack_transfer_request(transfer_request, transmit_length, request_length);
            break; 
            
        case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING:

            /* Get the line coding parameters from the host.  */
            cdc_acm -> ux_slave_class_cdc_acm_baudrate  = _ux_utility_long_get(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_BAUDRATE_STRUCT);
            cdc_acm -> ux_slave_class_cdc_acm_stop_bit  = *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_STOP_BIT_STRUCT);
            cdc_acm -> ux_slave_class_cdc_acm_parity    = *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARITY_STRUCT);
            cdc_acm -> ux_slave_class_cdc_acm_data_bit  = *(transfer_request -> ux_slave_transfer_request_data_pointer + UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_DATA_BIT_STRUCT);

            /* If there is a parameter change function call it.  */
            if (cdc_acm -> ux_slave_class_cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change != UX_NULL)
            {        
        
                /* Invoke the application.  */
                cdc_acm -> ux_slave_class_cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change(cdc_acm);
            }

            break ;

        default:

            /* Unknown function. It's not handled.  */
            return(UX_ERROR);
    }

    /* It's handled.  */
    return(UX_SUCCESS);
}

