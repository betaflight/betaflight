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
/*    _ux_dcd_sim_slave_endpoint_create                   PORTABLE C      */
/*                                                           6.1.6        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will create a physical endpoint.                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_sim_slave                         Pointer to device controller  */
/*    endpoint                              Pointer to endpoint container */
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
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            supported bi-dir-endpoints, */
/*                                            resulting in version 6.1.6  */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_sim_slave_endpoint_create(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_ENDPOINT *endpoint)
{

UX_DCD_SIM_SLAVE_ED     *ed;
ULONG                   sim_slave_endpoint_index;


    /* The simulator slave controller has 16 endpoints maximum. Endpoint 0 is always control. 
       The other endpoints are generic. We can use the endpoint number as an index.  */
    sim_slave_endpoint_index =  endpoint ->ux_slave_endpoint_descriptor.bEndpointAddress & ~(ULONG)UX_ENDPOINT_DIRECTION;

    /* Fetch the address of the physical endpoint.  */
#ifdef UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT
    ed = ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress == 0) ?
            &dcd_sim_slave -> ux_dcd_sim_slave_ed[0] :
            ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) ?
                &dcd_sim_slave -> ux_dcd_sim_slave_ed_in[sim_slave_endpoint_index] :
                &dcd_sim_slave -> ux_dcd_sim_slave_ed[sim_slave_endpoint_index]));
#else
    ed =  &dcd_sim_slave -> ux_dcd_sim_slave_ed[sim_slave_endpoint_index];
#endif

    /* Check the endpoint status, if it is free, reserve it. If not reject this endpoint.  */
    if ((ed -> ux_sim_slave_ed_status & UX_DCD_SIM_SLAVE_ED_STATUS_USED) == 0)
    {
        
        /* We can use this endpoint.  */
        ed -> ux_sim_slave_ed_status |=  UX_DCD_SIM_SLAVE_ED_STATUS_USED;

        /* Keep the physical endpoint address in the endpoint container.  */
        endpoint -> ux_slave_endpoint_ed =  (VOID *) ed;

        /* And its mask.  */
        ed -> ux_sim_slave_ed_index =  sim_slave_endpoint_index;

        /* Save the endpoint pointer.  */
        ed -> ux_sim_slave_ed_endpoint =  endpoint;

        /* If this is endpoint 0, it is always ready for transactions.  */
        if ( sim_slave_endpoint_index == 0)
            ed -> ux_sim_slave_ed_status |= UX_DCD_SIM_SLAVE_ED_STATUS_TRANSFER;

        /* Enable this endpoint.  */
        return(UX_SUCCESS);         
    }

    /* Notify application.  */
    _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_DCD, UX_MEMORY_INSUFFICIENT);

    /* Return error to caller.  */
    return(UX_NO_ED_AVAILABLE);
}

