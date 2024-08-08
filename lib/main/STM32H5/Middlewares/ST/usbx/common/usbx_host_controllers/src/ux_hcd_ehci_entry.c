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
/**   EHCI Controller Driver                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_ehci.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_entry                                  PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function dispatch the HCD function internally to the EHCI      */
/*    controller.                                                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    HCD                                   Pointer to HCD                */ 
/*    function                              Requested function            */ 
/*    parameter                             Pointer to parameter(s)       */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_hcd_ehci_asynchronous_endpoint_create     Create endpoint       */ 
/*    _ux_hcd_ehci_asynchronous_endpoint_destroy    Destroy endpoint      */ 
/*    _ux_hcd_ehci_controller_disable               Disable controller    */ 
/*    _ux_hcd_ehci_done_queue_process               Process done queue    */ 
/*    _ux_hcd_ehci_endpoint_reset                   Reset endpoint        */ 
/*    _ux_hcd_ehci_frame_number_get                 Get frame number      */ 
/*    _ux_hcd_ehci_frame_number_set                 Set frame number      */ 
/*    _ux_hcd_ehci_interrupt_endpoint_create        Endpoint create       */ 
/*    _ux_hcd_ehci_interrupt_endpoint_destroy       Endpoint destroy      */ 
/*    _ux_hcd_ehci_isochronous_endpoint_create      Endpoint create       */ 
/*    _ux_hcd_ehci_isochronous_endpoint_destroy     Endpoint destroy      */
/*    _ux_hcd_ehci_port_disable                     Disable port          */ 
/*    _ux_hcd_ehci_port_reset                       Reset port            */ 
/*    _ux_hcd_ehci_port_resume                      Resume port           */ 
/*    _ux_hcd_ehci_port_status_get                  Get port status       */ 
/*    _ux_hcd_ehci_port_suspend                     Suspend port          */ 
/*    _ux_hcd_ehci_power_down_port                  Power down port       */ 
/*    _ux_hcd_ehci_power_on_port                    Power on port         */ 
/*    _ux_hcd_ehci_request_transfer                 Request transfer      */ 
/*    _ux_hcd_ehci_transfer_abort                   Abort transfer        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    EHCI Controller Driver                                              */
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
UINT  _ux_hcd_ehci_entry(UX_HCD *hcd, UINT function, VOID *parameter)
{

UINT            status;
UX_HCD_EHCI     *hcd_ehci;
    

    /* Check the status of the controller.  */
    if (hcd -> ux_hcd_status == UX_UNUSED)
    {
    
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_CONTROLLER_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONTROLLER_UNKNOWN, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_CONTROLLER_UNKNOWN);
    }        

    /* Get the pointer to the EHCI HCD.  */
    hcd_ehci =  (UX_HCD_EHCI *) hcd -> ux_hcd_controller_hardware;

    /* Look at the function and route it.  */
    switch(function)
    {

    case UX_HCD_DISABLE_CONTROLLER:
    
        status =  _ux_hcd_ehci_controller_disable(hcd_ehci);
        break;
    
    
    case UX_HCD_GET_PORT_STATUS:
        status =  _ux_hcd_ehci_port_status_get(hcd_ehci, (ULONG) parameter);
        break;
    
    
    case UX_HCD_ENABLE_PORT:

        status =  UX_SUCCESS;
        break;
    
    
    case UX_HCD_DISABLE_PORT:

        status =  _ux_hcd_ehci_port_disable(hcd_ehci, (ULONG) parameter);
        break;
    
    
    case UX_HCD_POWER_ON_PORT:

        status =  _ux_hcd_ehci_power_on_port(hcd_ehci, (ULONG) parameter);
        break;
    
    
    case UX_HCD_POWER_DOWN_PORT:

        status =  _ux_hcd_ehci_power_down_port(hcd_ehci, (ULONG) parameter);
        break;
    
    
    case UX_HCD_SUSPEND_PORT:
        status =  _ux_hcd_ehci_port_suspend(hcd_ehci, (ULONG) parameter);
        break;
    
    
    case UX_HCD_RESUME_PORT:

        status =  _ux_hcd_ehci_port_resume(hcd_ehci, (UINT) parameter);
        break;
    
    
    case UX_HCD_RESET_PORT:

        status =  _ux_hcd_ehci_port_reset(hcd_ehci, (ULONG) parameter);
        break;
    
    
    case UX_HCD_GET_FRAME_NUMBER:

        status =  _ux_hcd_ehci_frame_number_get(hcd_ehci, (ULONG *) parameter);
        break;
    
    
    case UX_HCD_SET_FRAME_NUMBER:

        _ux_hcd_ehci_frame_number_set(hcd_ehci, (ULONG) parameter);
        status =  UX_SUCCESS;
        break;
    
    
    case UX_HCD_TRANSFER_REQUEST:

        status =  _ux_hcd_ehci_request_transfer(hcd_ehci, (UX_TRANSFER *) parameter);
        break;
    
    
    case UX_HCD_TRANSFER_ABORT:

        status =  _ux_hcd_ehci_transfer_abort(hcd_ehci, (UX_TRANSFER *) parameter);
        break;
    
    
    case UX_HCD_CREATE_ENDPOINT:

        switch ((((UX_ENDPOINT*) parameter) -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE)
        {

        case UX_CONTROL_ENDPOINT:
        case UX_BULK_ENDPOINT:
            status =  _ux_hcd_ehci_asynchronous_endpoint_create(hcd_ehci, (UX_ENDPOINT*) parameter);
            break;

        case UX_INTERRUPT_ENDPOINT:
            status =  _ux_hcd_ehci_interrupt_endpoint_create(hcd_ehci, (UX_ENDPOINT*) parameter);
            break;

        case UX_ISOCHRONOUS_ENDPOINT:
            status =  _ux_hcd_ehci_isochronous_endpoint_create(hcd_ehci, (UX_ENDPOINT*) parameter);
            break;

        }
        break;


    case UX_HCD_DESTROY_ENDPOINT:

        switch ((((UX_ENDPOINT*) parameter) -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE)
        {

        case UX_CONTROL_ENDPOINT:
        case UX_BULK_ENDPOINT:
            status =  _ux_hcd_ehci_asynchronous_endpoint_destroy(hcd_ehci, (UX_ENDPOINT*) parameter);
            break;

        case UX_INTERRUPT_ENDPOINT:
            status =  _ux_hcd_ehci_interrupt_endpoint_destroy(hcd_ehci, (UX_ENDPOINT*) parameter);
            break;

        case UX_ISOCHRONOUS_ENDPOINT:
            status =  _ux_hcd_ehci_isochronous_endpoint_destroy(hcd_ehci, (UX_ENDPOINT*) parameter);
            break;

        }
        break;


    case UX_HCD_RESET_ENDPOINT:

        status =  _ux_hcd_ehci_endpoint_reset(hcd_ehci, (UX_ENDPOINT*) parameter);
        break;


    case UX_HCD_PROCESS_DONE_QUEUE:

        _ux_hcd_ehci_done_queue_process(hcd_ehci);
        status =  UX_SUCCESS;
        break;


    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        status =  UX_FUNCTION_NOT_SUPPORTED;
        break;
    }        

    /* Return status to caller.  */
    return(status);
}

