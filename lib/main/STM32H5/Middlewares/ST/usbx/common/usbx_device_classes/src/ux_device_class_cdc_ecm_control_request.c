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
/*    _ux_device_class_cdc_ecm_control_request            PORTABLE C      */ 
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
/*    cdc_ecm                           Pointer to cdc_ecm class          */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_short_get                 Get 16-bit value              */ 
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed parameter/variable    */
/*                                            names conflict C++ keyword, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_cdc_ecm_control_request(UX_SLAVE_CLASS_COMMAND *command)
{

UX_SLAVE_TRANSFER       *transfer_request;
UX_SLAVE_DEVICE         *device;
ULONG                   request;
ULONG                   request_value;
UX_SLAVE_CLASS          *class_ptr;
UX_SLAVE_CLASS_CDC_ECM  *cdc_ecm;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the pointer to the transfer request associated with the control endpoint.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Extract all necessary fields of the request.  */
    request =  *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);
    request_value  =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE);

    /* Get the class container.  */
    class_ptr =  command -> ux_slave_class_command_class_ptr;
    
    /* Get the cdc_ecm instance from this class container.  */
    cdc_ecm =  (UX_SLAVE_CLASS_CDC_ECM *) class_ptr -> ux_slave_class_instance;
    
    /* Here we proceed only the standard request we know of at the device level.  */
    switch (request)
    {

        case UX_DEVICE_CLASS_CDC_ECM_SET_ETHERNET_MULTICAST_FILTER                :

            /* Save the multicast filter.  */                
            cdc_ecm -> ux_slave_class_cdc_ecm_ethernet_multicast_filter =  request_value;
            break ;

        case UX_DEVICE_CLASS_CDC_ECM_SET_ETHERNET_POWER_MANAGEMENT_FILTER        :

            /* Save the power management filter.  */
            cdc_ecm -> ux_slave_class_cdc_ecm_ethernet_power_management_filter =  request_value;
            break ;

        case UX_DEVICE_CLASS_CDC_ECM_SET_ETHERNET_PACKET_FILTER                    :

            /* Save the packet filter.  */
            cdc_ecm -> ux_slave_class_cdc_ecm_ethernet_packet_filter =  request_value;
            break ;
            
        case UX_DEVICE_CLASS_CDC_ECM_GET_ETHERNET_POWER_MANAGEMENT_FILTER        :
        default:

            /* Unknown function. It's not handled.  */
            return(UX_ERROR);
    }

    /* It's handled.  */
    return(UX_SUCCESS);
}

