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
/**   Slave Simulator Controller Driver                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE

/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_sim_slave.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_dcd_sim_slave_function                          PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function dispatches the DCD function internally to the         */
/*    slave simulator controller.                                         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd                                   Pointer to device controller  */
/*    function                              Function requested            */
/*    parameter                             Pointer to parameter structure*/
/*                                                                        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_dcd_sim_slave_address_set         Set address                   */
/*    _ux_dcd_sim_slave_endpoint_create     Create endpoint               */
/*    _ux_dcd_sim_slave_endpoint_destroy    Destroy endpoint              */
/*    _ux_dcd_sim_slave_endpoint_reset      Reset endpoint                */
/*    _ux_dcd_sim_slave_endpoint_stall      Stall endpoint                */
/*    _ux_dcd_sim_slave_endpoint_status     Get endpoint status           */ 
/*    _ux_dcd_sim_slave_frame_number_get    Get frame number              */
/*    _ux_dcd_sim_slave_state_change        Change state                  */
/*    _ux_dcd_sim_slave_transfer_abort      Abort transfer                */
/*    _ux_dcd_sim_slave_transfer_request    Request transfer              */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Device Stack                                                   */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT   _ux_dcd_sim_slave_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter)
{

UINT                    status;
UX_DCD_SIM_SLAVE        *dcd_sim_slave;
                                                        

    /* Check the status of the controller.  */
    if (dcd -> ux_slave_dcd_status == UX_UNUSED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_DCD, UX_CONTROLLER_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONTROLLER_UNKNOWN, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_CONTROLLER_UNKNOWN);
    }
            
    /* Get the pointer to the Slave simulation DCD.  */
    dcd_sim_slave =  (UX_DCD_SIM_SLAVE *) dcd -> ux_slave_dcd_controller_hardware;

    /* Look at the function and route it.  */
    switch(function)
    {

    case UX_DCD_GET_FRAME_NUMBER:
            
        status =  _ux_dcd_sim_slave_frame_number_get(dcd_sim_slave, (ULONG *) parameter);
        break;

#if defined(UX_DEVICE_STANDALONE)
    case UX_DCD_TRANSFER_RUN:

        status =  _ux_dcd_sim_slave_transfer_run(dcd_sim_slave, (UX_SLAVE_TRANSFER *) parameter);
        break;
#else
    case UX_DCD_TRANSFER_REQUEST:

        status =  _ux_dcd_sim_slave_transfer_request(dcd_sim_slave, (UX_SLAVE_TRANSFER *) parameter);
        break;
#endif

    case UX_DCD_TRANSFER_ABORT:

        status =  _ux_dcd_sim_slave_transfer_abort(dcd_sim_slave, (UX_SLAVE_TRANSFER *) parameter);
        break;

    case UX_DCD_CREATE_ENDPOINT:

        status =  _ux_dcd_sim_slave_endpoint_create(dcd_sim_slave, parameter);
        break;

    case UX_DCD_DESTROY_ENDPOINT:

        status =  _ux_dcd_sim_slave_endpoint_destroy(dcd_sim_slave, parameter);
        break;

    case UX_DCD_RESET_ENDPOINT:

        status =  _ux_dcd_sim_slave_endpoint_reset(dcd_sim_slave, parameter);
        break;

    case UX_DCD_STALL_ENDPOINT:

        status =  _ux_dcd_sim_slave_endpoint_stall(dcd_sim_slave, parameter);
        break;

    case UX_DCD_SET_DEVICE_ADDRESS:

        status =  _ux_dcd_sim_slave_address_set(dcd_sim_slave, (ULONG) (ALIGN_TYPE) parameter);
        break;

    case UX_DCD_CHANGE_STATE:

        status =  _ux_dcd_sim_slave_state_change(dcd_sim_slave, (ULONG) (ALIGN_TYPE)  parameter);
        break;

    case UX_DCD_ENDPOINT_STATUS:

        status =  _ux_dcd_sim_slave_endpoint_status(dcd_sim_slave, (ULONG) (ALIGN_TYPE) parameter);
        break;

    case UX_DCD_ISR_PENDING:

        status = UX_SUCCESS;
        break;

    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_DCD, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        status = UX_FUNCTION_NOT_SUPPORTED;
    }        

    /* Return completion status.  */
    return(status);
}

