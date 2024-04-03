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
/**   HUB Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_stack.h"
#include "ux_host_class_hub.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_hub_transfer_request_completed       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function is called by the completion thread when a transfer    */ 
/*    request has been completed either because the transfer is           */ 
/*    successful or there was an error.                                   */
/*                                                                        */
/*    Because the HUB influences the topology of the USB, the insertion   */
/*    or extraction of devices cannot be done during the transfer request */ 
/*    thread. We post a signal to the topology thread to wake up and      */ 
/*    treat these changes on the HUB status.                              */
/*                                                                        */
/*    In RTOS mode, the interrupt pipe is not reactivated here. We will   */
/*    do this when the topology thread has investigated the reason of the */
/*    transfer completion.                                                */
/*                                                                        */
/*    In standalone mode, the interrupt pipe is reactivated here. The     */
/*    bitmap in buffer is examined in hub tasks function.                 */
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    transfer_request                      Pointer to transfer request   */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_host_semaphore_put                Put the signaling semaphore   */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    HUB Class                                                           */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            refined macros names,       */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
VOID  _ux_host_class_hub_transfer_request_completed(UX_TRANSFER *transfer_request)
{

UX_HOST_CLASS_HUB        *hub;


    /* Get the class instance for this transfer request.  */
    hub =  (UX_HOST_CLASS_HUB *) transfer_request -> ux_transfer_request_class_instance;


    /* Check the state of the transfer.  If there is an error, we do not proceed with this report.  */
    if (transfer_request -> ux_transfer_request_completion_code != UX_SUCCESS)
    {

        /* We have an error. We do not rehook another transfer if the device instance is shutting down or
           if the transfer was aborted by the class.  */
        if ((hub -> ux_host_class_hub_state ==  UX_HOST_CLASS_INSTANCE_SHUTDOWN) || 
            (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_STATUS_ABORT) ||
            (transfer_request -> ux_transfer_request_completion_code == UX_TRANSFER_NO_ANSWER))

            /* We do not proceed.  */
            return;
        else

        {            

            /* Reactivate the HUB interrupt pipe.  */
            _ux_host_stack_transfer_request(transfer_request);
        
            /* We do not proceed.  */
            return;        
        }            
    }

#if defined(UX_HOST_STANDALONE)

    /* Reactivate the HUB interrupt pipe.  */
    _ux_host_stack_transfer_request(transfer_request);
#else

    /* We need to memorize which HUB instance has received a change signal.  */
    hub -> ux_host_class_hub_change_semaphore++;
    
    /* Now we can set the semaphore, the enum thread will wake up and will
       call the HUB instance which has a status change.  */
    _ux_host_semaphore_put(&_ux_system_host -> ux_system_host_enum_semaphore);
#endif

    /* Return to caller.  */
    return;
}
