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
/*    _ux_dcd_sim_slave_transfer_request                  PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will initiate a transfer to a specific endpoint.      */
/*    If the endpoint is IN, the endpoint register will be set to accept  */
/*    the request.                                                        */
/*                                                                        */
/*    If the endpoint is IN, the endpoint FIFO will be filled with the    */
/*    buffer and the endpoint register set.                               */
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
/*    _ux_utility_semaphore_get             Get semaphore                 */ 
/*    _ux_dcd_sim_slave_transfer_abort      Abort transfer                */
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
/*                                            cleared transfer status     */
/*                                            before semaphore wakeup to  */
/*                                            avoid a race condition,     */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_sim_slave_transfer_request(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_TRANSFER *transfer_request)
{

UX_SLAVE_ENDPOINT       *endpoint;
UX_DCD_SIM_SLAVE_ED     *ed;
UINT                    status;


    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

    /* Get the slave endpoint.  */
    ed = (UX_DCD_SIM_SLAVE_ED *) endpoint -> ux_slave_endpoint_ed;
    
    /* We have a request for a OUT or IN transaction from the host.
       If the endpoint is a Control endpoint, all this is happening under Interrupt and there is no
       thread to suspend.  */
    if (ed -> ux_sim_slave_ed_index != 0)
    {

        /* Set the ED to TRANSFER status.  */
        ed -> ux_sim_slave_ed_status |= UX_DCD_SIM_SLAVE_ED_STATUS_TRANSFER;

        /* We should wait for the semaphore to wake us up.  */
        status =  _ux_device_semaphore_get(&transfer_request -> ux_slave_transfer_request_semaphore,
                                            transfer_request -> ux_slave_transfer_request_timeout);
           
        /* Check the completion code. */
        if (status != UX_SUCCESS)
        {
            _ux_dcd_sim_slave_transfer_abort(dcd_sim_slave, transfer_request);
            transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STATUS_COMPLETED;
            return(status);
        }

        /* ED TRANSFER has been cleared before semaphore wakeup.  */

        /* Check the transfer request completion code. We may have had a BUS reset or
           a device disconnection.  */
        if (transfer_request -> ux_slave_transfer_request_completion_code != UX_SUCCESS)
            return(transfer_request -> ux_slave_transfer_request_completion_code);
    }

    /* Return to caller with success.  */
    return(UX_SUCCESS);
}

