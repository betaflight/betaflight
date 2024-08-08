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
/*    _ux_dcd_sim_slave_transfer_abort                    PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will terminate a transfer.                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_sim_slave                         Pointer to device controller  */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Slave Simulator Controller Driver                                   */
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
UINT  _ux_dcd_sim_slave_transfer_abort(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_TRANSFER *transfer_request)
{

UX_DCD_SIM_SLAVE_ED     *ed;
UX_SLAVE_ENDPOINT       *endpoint;

    UX_PARAMETER_NOT_USED(dcd_sim_slave);

    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

    /* Keep the physical endpoint address in the endpoint container.  */
    ed =  (UX_DCD_SIM_SLAVE_ED *) endpoint -> ux_slave_endpoint_ed;

    /* Turn off the transfer bit.  */
    ed -> ux_sim_slave_ed_status &= ~(ULONG)
        (UX_DCD_SIM_SLAVE_ED_STATUS_TRANSFER | UX_DCD_SIM_SLAVE_ED_STATUS_DONE);

    /* This function never fails.  */
    return(UX_SUCCESS);         
}

