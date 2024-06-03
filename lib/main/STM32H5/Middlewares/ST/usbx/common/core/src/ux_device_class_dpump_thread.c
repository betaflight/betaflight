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
/**   Device Data Pump Class                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_dpump.h"
#include "ux_device_stack.h"

/* Remove compiling warning. */
VOID  _ux_device_class_dpump_thread(ULONG dpump_class);

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_dpump_thread                       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is the thread of the dpump class.                     */ 
/*                                                                        */
/*    It's for RTOS mode.                                                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    class                                   Address of dpump class      */ 
/*                                            container                   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Request transfer              */ 
/*    _ux_utility_memory_copy               Copy memory                   */ 
/*    _ux_utility_thread_suspend            Suspend thread                */ 
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
/*                                            verified memset and memcpy  */
/*                                            cases,                      */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_device_class_dpump_thread(ULONG dpump_class)
{

UX_SLAVE_CLASS              *class_ptr;
UX_SLAVE_INTERFACE          *interface_ptr;
UX_SLAVE_CLASS_DPUMP        *dpump;
UX_SLAVE_TRANSFER           *transfer_request;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_ENDPOINT           *endpoint_in;
UX_SLAVE_ENDPOINT           *endpoint_out;
UINT                        status;
ULONG                       length;

    /* This thread runs forever but can be suspended or resumed.  */
    while(1)
    {

        /* Cast properly the dpump instance.  */
        UX_THREAD_EXTENSION_PTR_GET(class_ptr, UX_SLAVE_CLASS, dpump_class)
        
        /* Get the dpump instance from this class container.  */
        dpump =  (UX_SLAVE_CLASS_DPUMP *) class_ptr -> ux_slave_class_instance;
    
        /* Get the pointer to the device.  */
        device =  &_ux_system_slave -> ux_system_slave_device;
        
        /* This is the first time we are activated. We need the interface to the class.  */
        interface_ptr =  dpump -> ux_slave_class_dpump_interface;
        
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
            
        /* As long as the device is in the CONFIGURED state.  */
        while (device -> ux_slave_device_state == UX_DEVICE_CONFIGURED)
        { 
        
            /* We prepare to receive from the host on the OUT endpoint.  */
            transfer_request =  &endpoint_out -> ux_slave_endpoint_transfer_request;
    
            /* Send the request to the device controller.  */
            status =  _ux_device_stack_transfer_request(transfer_request, UX_DEVICE_CLASS_DPUMP_PACKET_SIZE, UX_DEVICE_CLASS_DPUMP_PACKET_SIZE);
    
            /* Check the status */    
            if (status == UX_SUCCESS)
            {

                /* Obtain the length of the transaction.  */
                length =  transfer_request -> ux_slave_transfer_request_actual_length;

                /* Copy the buffer to the target in endpoint.  */
                _ux_utility_memory_copy(endpoint_in -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer,
                                        endpoint_out -> ux_slave_endpoint_transfer_request.ux_slave_transfer_request_data_pointer,
                                        length); /* Use case of memcpy is verified. */

                /* Now we send the packet back to the host. On the endpoint In.  */
                transfer_request =  &endpoint_in -> ux_slave_endpoint_transfer_request;
    
                /* Sends the data payload back to the caller.  */
                status =  _ux_device_stack_transfer_request(transfer_request, length, length);
                                    
                /* Check error code. */
                if (status != UX_SUCCESS)

                    /* Error trap. */
                    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, status);
            }
        }

        /* We need to suspend ourselves. We will be resumed by the 
           device enumeration module.  */
        _ux_device_thread_suspend(&class_ptr -> ux_slave_class_thread);
    }
}

