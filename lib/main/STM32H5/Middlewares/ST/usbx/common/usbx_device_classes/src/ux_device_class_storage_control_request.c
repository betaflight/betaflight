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
/**   Device Storage Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_storage.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_storage_control_request            PORTABLE C      */ 
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
/*    storage                               Pointer to storage class      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Transfer request              */ 
/*    _ux_device_stack_transfer_abort       Abort Transfer                */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Storage Class                                                */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            optimized command logic,    */
/*                                            resulting in version 6.1    */
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed USB CV test issues,   */
/*                                            resulting in version 6.1.3  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_storage_control_request(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_TRANSFER           *transfer_request;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_CLASS              *class_ptr;
ULONG                       request;
ULONG                       request_value;
ULONG                       request_length;
UX_SLAVE_CLASS_STORAGE      *storage;
#if !defined(UX_DEVICE_STANDALONE)
UX_SLAVE_INTERFACE          *interface_ptr;
#endif
UX_SLAVE_ENDPOINT           *endpoint_in;
UX_SLAVE_ENDPOINT           *endpoint_out;


    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the pointer to the transfer request associated with the control endpoint.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;
    
    /* Extract the request type from the SETUP packet..   */
    request =  *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);
    request_value = _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE);
    request_length = _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

    /* Check if wValue is valid.  */
    if (request_value != 0)
        return(UX_ERROR);

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;
    
    /* Get the storage instance from this class container.  */
    storage =  (UX_SLAVE_CLASS_STORAGE *) class_ptr -> ux_slave_class_instance;

    /* Here we proceed only the standard request we know of at the device level.  */
    switch (request)
    {

    case UX_SLAVE_CLASS_STORAGE_RESET:

        /* Check if wLength is valid.  */
        if (request_length != 0)
            return(UX_ERROR);

#if defined(UX_DEVICE_STANDALONE)
        endpoint_in = storage -> ux_device_class_storage_ep_in;
        endpoint_out = storage -> ux_device_class_storage_ep_out;
#else

        /* We need the interface to the class.  */
        interface_ptr =  storage -> ux_slave_class_storage_interface;

        /* Locate the endpoints.  */
        endpoint_in =  interface_ptr -> ux_slave_interface_first_endpoint;
        
        /* Check the endpoint direction, if IN we have the correct endpoint.  */
        if ((endpoint_in -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_IN)
        {

            /* Wrong direction, we found the OUT endpoint first.  */
            endpoint_out =  endpoint_in;
                
            /* So the next endpoint has to be the IN endpoint.  */
            endpoint_in =  endpoint_out -> ux_slave_endpoint_next_endpoint;
        }
        else
        {

            /* We found the endpoint IN first, so next endpoint is OUT.  */
            endpoint_out =  endpoint_in -> ux_slave_endpoint_next_endpoint;
        }
#endif

        /* First cancel any transfer on the endpoint OUT, from the host.  */
        transfer_request =  &endpoint_out -> ux_slave_endpoint_transfer_request;
        _ux_device_stack_transfer_abort(transfer_request, UX_TRANSFER_APPLICATION_RESET);

        /* Then cancel any transfer on the endpoint IN, from the host.  */
        transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;
        _ux_device_stack_transfer_abort(transfer_request, UX_TRANSFER_APPLICATION_RESET);

        /* Reset phase error.  */
        storage -> ux_slave_class_storage_csw_status = UX_SLAVE_CLASS_STORAGE_CSW_PASSED;

        break;

    case UX_SLAVE_CLASS_STORAGE_GET_MAX_LUN:

        /* Check if wLength is valid.  */
        if (request_length < 1)
            return(UX_ERROR);

        /* Set the value of the number of LUN in the buffer. The max number of LUN is the
           number of declared LUN - 1.  */
        *transfer_request -> ux_slave_transfer_request_data_pointer =  (UCHAR)(storage -> ux_slave_class_storage_number_lun -1);

        /* Set the phase of the transfer to data out.  */
        transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_OUT;

        /* We can return the LUN number.  */
        _ux_device_stack_transfer_request(transfer_request, 1, 1);
        break;

    default:

        /* Unknown function. It's not handled.  */
        return(UX_ERROR);
    }

    /* It's handled.  */
    return(UX_SUCCESS);
}
