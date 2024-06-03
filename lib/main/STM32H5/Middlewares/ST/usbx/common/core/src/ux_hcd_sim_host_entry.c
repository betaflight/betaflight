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
/**   Host Simulator Controller Driver                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_hcd_sim_host.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_hcd_sim_host_entry                              PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function dispatch the HCD function internally to the simulator */
/*    controller driver.                                                  */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    hcd                                   Pointer to HCD                */ 
/*    function                              Function for driver to perform*/ 
/*    parameter                             Pointer to parameter(s)       */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*  _ux_hcd_sim_host_asynch_queue_process          Process asynch queue   */ 
/*  _ux_hcd_sim_host_asynch_schedule               Schedule async work    */ 
/*  _ux_hcd_sim_host_asynchronous_endpoint_create  Create async endpoint  */ 
/*  _ux_hcd_sim_host_asynchronous_endpoint_destroy Destroy async endpoint */ 
/*  _ux_hcd_sim_host_endpoint_reset                Reset endpoint         */ 
/*  _ux_hcd_sim_host_frame_number_get              Get frame number       */ 
/*  _ux_hcd_sim_host_interrupt_endpoint_create     Create endpoint        */ 
/*  _ux_hcd_sim_host_iso_queue_process             Process iso queue      */ 
/*  _ux_hcd_sim_host_iso_schedule                  Schedule iso work      */ 
/*  _ux_hcd_sim_host_isochronous_endpoint_create   Create iso endpoint    */ 
/*  _ux_hcd_sim_host_periodic_endpoint_destroy     Destroy endpoint       */ 
/*  _ux_hcd_sim_host_periodic_schedule             Schedule periodic      */ 
/*  _ux_hcd_sim_host_port_status_get               Get port status        */ 
/*  _ux_hcd_sim_host_port_reset                    Reset port             */
/*  _ux_hcd_sim_host_request_transfer              Request transfer       */ 
/*  _ux_hcd_sim_host_transfer_abort                Abort transfer         */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Host Stack                                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added controller disable,   */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added HCD uninitialize,     */
/*                                            resulting in version 6.1.2  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_sim_host_entry(UX_HCD *hcd, UINT function, VOID *parameter)
{

UINT                status = 0;
UX_HCD_SIM_HOST     *hcd_sim_host;
    

    /* Check the status of the controller.  */
    if (hcd -> ux_hcd_status == UX_UNUSED)
    {
    
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_CONTROLLER_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONTROLLER_UNKNOWN, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_CONTROLLER_UNKNOWN);
    }
        
    /* Get the pointer to the host simulator HCD.  */
    hcd_sim_host =  (UX_HCD_SIM_HOST *) hcd -> ux_hcd_controller_hardware;

    /* look at the function and route it.  */
    switch(function)
    {

    case UX_HCD_UNINITIALIZE:
        status =  _ux_hcd_sim_host_uninitialize(hcd_sim_host);
        break;


    case UX_HCD_DISABLE_CONTROLLER:

        status =  _ux_hcd_sim_host_controller_disable(hcd_sim_host);
        break;


    case UX_HCD_GET_PORT_STATUS:

        status =  _ux_hcd_sim_host_port_status_get(hcd_sim_host, (ULONG) (ALIGN_TYPE) parameter);
        break;
    
    
    case UX_HCD_ENABLE_PORT:
    
        status =  UX_SUCCESS;
        break;
    
    
    case UX_HCD_DISABLE_PORT:

        status =  UX_SUCCESS;
        break;


    case UX_HCD_POWER_ON_PORT:
        
        status =  UX_SUCCESS;
        break;


    case UX_HCD_POWER_DOWN_PORT:

        status =  UX_SUCCESS;
        break;


    case UX_HCD_SUSPEND_PORT:

        status =  UX_SUCCESS;
        break;


    case UX_HCD_RESUME_PORT:

        status =  UX_SUCCESS;
        break;


    case UX_HCD_RESET_PORT:

        status =  _ux_hcd_sim_host_port_reset(hcd_sim_host, (ULONG) (ALIGN_TYPE) parameter);
        break;


    case UX_HCD_GET_FRAME_NUMBER:

        status =  _ux_hcd_sim_host_frame_number_get(hcd_sim_host, (ULONG *) parameter);
        break;


    case UX_HCD_SET_FRAME_NUMBER:

        status =  UX_SUCCESS;
        break;


    case UX_HCD_TRANSFER_REQUEST:

#if defined(UX_HOST_STANDALONE)
        status =  _ux_hcd_sim_host_transfer_run(hcd_sim_host, (UX_TRANSFER *) parameter);
#else
        status =  _ux_hcd_sim_host_request_transfer(hcd_sim_host, (UX_TRANSFER *) parameter);
#endif
        break;


    case UX_HCD_TRANSFER_ABORT:

        status =  _ux_hcd_sim_host_transfer_abort(hcd_sim_host, (UX_TRANSFER *) parameter);
        break;


    case UX_HCD_CREATE_ENDPOINT:

        switch ((((UX_ENDPOINT*) parameter) -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE)
        {

        case UX_CONTROL_ENDPOINT:
        case UX_BULK_ENDPOINT:
        
            status =  _ux_hcd_sim_host_asynchronous_endpoint_create(hcd_sim_host, (UX_ENDPOINT*) parameter);
            break;

            
        case UX_INTERRUPT_ENDPOINT:
        
            status =  _ux_hcd_sim_host_interrupt_endpoint_create(hcd_sim_host, (UX_ENDPOINT*) parameter);
            break;


        case UX_ISOCHRONOUS_ENDPOINT:
        
            status =  _ux_hcd_sim_host_isochronous_endpoint_create(hcd_sim_host, (UX_ENDPOINT*) parameter);
            break;

        }
        break;


    case UX_HCD_DESTROY_ENDPOINT:

        switch ((((UX_ENDPOINT*) parameter) -> ux_endpoint_descriptor.bmAttributes) & UX_MASK_ENDPOINT_TYPE)
        {

        case UX_CONTROL_ENDPOINT:
        case UX_BULK_ENDPOINT:

            status =  _ux_hcd_sim_host_asynchronous_endpoint_destroy(hcd_sim_host, (UX_ENDPOINT*) parameter);
            break;


        case UX_INTERRUPT_ENDPOINT:
        case UX_ISOCHRONOUS_ENDPOINT:

            status =  _ux_hcd_sim_host_periodic_endpoint_destroy(hcd_sim_host, (UX_ENDPOINT*) parameter);
            break;

        }
        break;


    case UX_HCD_RESET_ENDPOINT:

        status =  _ux_hcd_sim_host_endpoint_reset(hcd_sim_host, (UX_ENDPOINT*) parameter);
        break;


    case UX_HCD_PROCESS_DONE_QUEUE:

        _ux_hcd_sim_host_iso_queue_process(hcd_sim_host);
        _ux_hcd_sim_host_asynch_queue_process(hcd_sim_host);
        _ux_hcd_sim_host_iso_schedule(hcd_sim_host);
        _ux_hcd_sim_host_periodic_schedule(hcd_sim_host);
        _ux_hcd_sim_host_asynch_schedule(hcd_sim_host);
        status =  UX_SUCCESS;
        break;


    default:
    
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        /* Unknown request, return an error.  */
        status =  UX_FUNCTION_NOT_SUPPORTED;
    }        
    
    /* Return completion status.  */
    return(status);
}

