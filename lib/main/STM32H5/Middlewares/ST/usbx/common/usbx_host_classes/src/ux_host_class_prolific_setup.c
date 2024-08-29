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
/**   Prolific Class                                                      */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_prolific.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_prolific_setup                       PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function obtains the entire prolific configuration descriptors.*/ 
/*    This is needed because the prolific class needs to know if commands */ 
/*    are routed through the comm interface or the data class.            */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    prolific                                 Pointer to prolific class  */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_stack_transfer_request       Process transfer request      */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*    _ux_utility_memory_free               Release memory block          */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    _ux_host_class_prolific_activate                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_prolific_setup(UX_HOST_CLASS_PROLIFIC *prolific)
{

UCHAR                       *setup_buffer;
UX_ENDPOINT                 *control_endpoint;
UX_TRANSFER                 *transfer_request;
UINT                        status;

    /* We need to get the default control endpoint transfer request pointer.  */
    control_endpoint =  &prolific -> ux_host_class_prolific_device -> ux_device_control_endpoint;
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Need to allocate memory for the buffer.  */
    setup_buffer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, UX_HOST_CLASS_PROLIFIC_SETUP_BUFFER_SIZE);
    if (setup_buffer == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Set the device type. Can be either 0, 1 or HX.  */
    if (prolific -> ux_host_class_prolific_device -> ux_device_descriptor.bDeviceClass == 0x02)        
                
        /* Device is of type 0. */
        prolific -> ux_host_class_prolific_device_type = UX_HOST_CLASS_PROLIFIC_DEVICE_TYPE_0;
    
    else
    {

        /* Check packet size. If 64, we are dealing with HX type device.  */
        if (prolific -> ux_host_class_prolific_device -> ux_device_descriptor.bMaxPacketSize0 == 64)

            /* Device is of type HX. */
            prolific -> ux_host_class_prolific_device_type = UX_HOST_CLASS_PROLIFIC_DEVICE_TYPE_HX;
        
        else
        
            /* Default case : type 1.  */
            prolific -> ux_host_class_prolific_device_type = UX_HOST_CLASS_PROLIFIC_DEVICE_TYPE_1;

    }        
    
    /* Create a transfer request for the prolific setup request # 1.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  1;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_PROLIFIC_COMMAND_EEPROM_READ;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(status);

    }
    
    /* Create a transfer request for the prolific setup request # 2.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0x0404;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(status);

    }
    
    /* Create a transfer request for the prolific setup request # 3.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  1;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_PROLIFIC_COMMAND_EEPROM_READ;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(status);

    }

    /* Create a transfer request for the prolific setup request # 4.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  1;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0x8383;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(status);

    }
    
    /* Create a transfer request for the prolific setup request # 5.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  1;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_PROLIFIC_COMMAND_EEPROM_READ;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(status);

    }
    
    /* Create a transfer request for the prolific setup request # 6.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0x0404;
    transfer_request -> ux_transfer_request_index               =  1;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(status);

    }
    
    /* Create a transfer request for the prolific setup request # 7.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  1;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_PROLIFIC_COMMAND_EEPROM_READ;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(status);

    }

    /* Create a transfer request for the prolific setup request # 8.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  1;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_IN | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0x8383;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(status);

    }

    /* Create a transfer request for the prolific setup request # 9.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  0;
    transfer_request -> ux_transfer_request_index               =  1;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(status);

    }

    /* Create a transfer request for the prolific setup request # 9.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  1;
    transfer_request -> ux_transfer_request_index               =  0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS)
    {

        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);

        /* Return completion status.  */
        return(status);

    }

    /* Create a transfer request for the prolific setup request # 10.  */
    if (prolific -> ux_host_class_prolific_device_type == UX_HOST_CLASS_PROLIFIC_DEVICE_TYPE_HX)

        /* Chip is HX.  */
        transfer_request -> ux_transfer_request_index           =  0x44;

    else

        /* Chip is not HX.  */
        transfer_request -> ux_transfer_request_index           =  0x24;
    
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  1;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_PROLIFIC_COMMAND_REG_CONFIGURE;

    /* Send request to HCD layer.  */
    _ux_host_stack_transfer_request(transfer_request);


    /* Reset upstream data pipes part 1.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_PROLIFIC_VENDOR_WRITE_REQUEST;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_PROLIFIC_COMMAND_PIPE1_RESET;
    transfer_request -> ux_transfer_request_index               =  0;
    
    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);
    
    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {
    
        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);
    
        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);
    
    }

    /* Reset upstream data pipes part 2.  */
    transfer_request -> ux_transfer_request_data_pointer        =  setup_buffer;
    transfer_request -> ux_transfer_request_requested_length    =  0;
    transfer_request -> ux_transfer_request_function            =  UX_HOST_CLASS_PROLIFIC_VENDOR_WRITE_REQUEST;
    transfer_request -> ux_transfer_request_type                =  UX_REQUEST_OUT | UX_REQUEST_TYPE_VENDOR | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value               =  UX_HOST_CLASS_PROLIFIC_COMMAND_PIPE2_RESET;
    transfer_request -> ux_transfer_request_index               =  0;
    
    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);
    
    /* Check status, if error, do not proceed.  */
    if (status != UX_SUCCESS || transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {
    
        /* Free all used resources.  */
        _ux_utility_memory_free(setup_buffer);
    
        /* Return completion status.  */
        return(UX_TRANSFER_ERROR);
    
    }

    /* Free all used resources.  */
    _ux_utility_memory_free(setup_buffer);

    /* Return completion status.  */
    return(status);
}

