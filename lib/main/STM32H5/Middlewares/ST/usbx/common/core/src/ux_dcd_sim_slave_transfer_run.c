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


#if defined(UX_DEVICE_STANDALONE)
/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                RELEASE       */
/*                                                                        */
/*    _ux_dcd_sim_slave_transfer_run                       PORTABLE C     */
/*                                                            6.1.10      */
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
/*    It's for standalone mode.                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_sim_slave                         Pointer to device controller  */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    State machine Status to check                                       */
/*    UX_STATE_NEXT                         Transfer done, to next state  */
/*    UX_STATE_EXIT                         Abnormal, to reset state      */
/*    (others)                              Keep running, waiting         */
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
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_sim_slave_transfer_run(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_TRANSFER *transfer_request)
{

UX_INTERRUPT_SAVE_AREA

UX_SLAVE_ENDPOINT       *endpoint;
UX_DCD_SIM_SLAVE_ED     *ed;
ULONG                   ed_status;


    UX_PARAMETER_NOT_USED(dcd_sim_slave);

    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

    /* Get the slave endpoint.  */
    ed = (UX_DCD_SIM_SLAVE_ED *) endpoint -> ux_slave_endpoint_ed;

    UX_DISABLE

    /* Get current status.  */
    ed_status = ed -> ux_sim_slave_ed_status;

    /* ED freed, must disconnected.  */
    if (ed_status == UX_DCD_SIM_SLAVE_ED_STATUS_UNUSED)
    {
        transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_BUS_RESET;
        UX_RESTORE
        return(UX_STATE_EXIT);
    }

    /* For control endpoint, always go to next state.  */
    if(ed -> ux_sim_slave_ed_index == 0)
    {
        UX_RESTORE
        return(UX_STATE_NEXT);
    }

    /* ED stalled.  */
    if (ed_status & UX_DCD_SIM_SLAVE_ED_STATUS_STALLED)
    {
        ed -> ux_sim_slave_ed_status = UX_DCD_SIM_SLAVE_ED_STATUS_USED;
        transfer_request -> ux_slave_transfer_request_status = UX_TRANSFER_STALLED;
        UX_RESTORE
        return(UX_STATE_NEXT);
    }

    /* Transfer started.  */
    if (ed_status & UX_DCD_SIM_SLAVE_ED_STATUS_TRANSFER)
    {
        if (ed_status & UX_DCD_SIM_SLAVE_ED_STATUS_DONE)
        {
            ed -> ux_sim_slave_ed_status = UX_DCD_SIM_SLAVE_ED_STATUS_USED;
            UX_RESTORE
            return(UX_STATE_NEXT);
        }
        UX_RESTORE
        return(UX_STATE_WAIT);
    }

    /* Start transfer.  */
    ed->ux_sim_slave_ed_status |= UX_DCD_SIM_SLAVE_ED_STATUS_TRANSFER;
    UX_RESTORE
    return(UX_STATE_WAIT);

}
#endif
