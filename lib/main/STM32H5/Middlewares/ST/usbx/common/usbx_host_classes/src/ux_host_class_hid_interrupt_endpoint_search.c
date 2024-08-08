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
/**  HID Class                                                            */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hid_interrupt_endpoint_search        PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function searches for the handle of the only interrupt         */ 
/*    endpoint in the default alternate setting of the HID interface.     */
/*    The interrupt endpoint should always be there. The actual first     */ 
/*    transfer on the interrupt endpoint does not start until a HID       */ 
/*    client has claimed ownership of the HID device.                     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hid                                   Pointer to HID class          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_memory_allocate           Allocate memory block         */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HID Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Xiuwen Cai, CQ Xiao      Modified comment(s),          */
/*                                            added interrupt OUT support,*/
/*                                            added timeout initialize,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_hid_interrupt_endpoint_search(UX_HOST_CLASS_HID *hid)
{

UINT            status = UX_ENDPOINT_HANDLE_UNKNOWN;
UX_INTERFACE    *interface_ptr;
UX_ENDPOINT     *endpoint;
UX_TRANSFER     *transfer_request;


    /* Search the interrupt endpoint. It is attached to the interface container.  */
    interface_ptr = hid -> ux_host_class_hid_interface;
    endpoint = interface_ptr -> ux_interface_first_endpoint;
    while(endpoint != UX_NULL)
    {

        /* Find interrupt IN endpoint.  */
        if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN) &&
            ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT))
        {

            /* The endpoint is correct, save it.  */
            hid -> ux_host_class_hid_interrupt_endpoint = endpoint;

            /* Fill in the transfer request with the length requested for this endpoint.  */
            transfer_request =  &endpoint -> ux_endpoint_transfer_request;
            transfer_request -> ux_transfer_request_requested_length =  hid -> ux_host_class_hid_interrupt_endpoint -> ux_endpoint_descriptor.wMaxPacketSize;
            transfer_request -> ux_transfer_request_actual_length =     0;

            /* The direction is always IN for the HID interrupt endpoint.  */
            transfer_request -> ux_transfer_request_type =  UX_REQUEST_IN;

            /* There is a callback function associated with the transfer request, so we need the class instance.  */
            transfer_request -> ux_transfer_request_class_instance =  (VOID *) hid;

            /* Interrupt transactions have a completion routine. */
            transfer_request -> ux_transfer_request_completion_function =  _ux_host_class_hid_transfer_request_completed;

            /* Transfer timeout : wait forever.  */
            transfer_request -> ux_transfer_request_timeout_value = UX_WAIT_FOREVER;

            /* Obtain a buffer for this transaction. The buffer will always be reused.  */
            transfer_request -> ux_transfer_request_data_pointer =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, 
                                                                        transfer_request -> ux_transfer_request_requested_length);

            /* If the endpoint is available and we have memory, we mark the interrupt endpoint as ready.  */
            if (transfer_request -> ux_transfer_request_data_pointer != UX_NULL)
            {
                hid -> ux_host_class_hid_interrupt_endpoint_status =  UX_HOST_CLASS_HID_INTERRUPT_ENDPOINT_READY;
                status = UX_SUCCESS;
#if !defined(UX_HOST_CLASS_HID_INTERRUPT_OUT_SUPPORT)
                
                /* We have found the interrupt IN endpoint, just stop searching.  */
                break;
#endif
            }
            else
            {
                status = UX_MEMORY_INSUFFICIENT;
                break;
            }
        }
#if defined(UX_HOST_CLASS_HID_INTERRUPT_OUT_SUPPORT)
        else if (((endpoint -> ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT) &&
            ((endpoint -> ux_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT))
        {

            /* Found the interrupt OUT endpoint, save it.  */
            hid -> ux_host_class_hid_interrupt_out_endpoint = endpoint;
        }
#endif

        /* Check next endpoint.  */
        endpoint = endpoint -> ux_endpoint_next_endpoint;
    }

    /* Return completion status.  */
    return(status);
}

