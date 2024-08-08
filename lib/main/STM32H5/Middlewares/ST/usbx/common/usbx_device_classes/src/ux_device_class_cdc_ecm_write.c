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
/*    _ux_device_class_cdc_ecm_write                      PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function writes a packet into a queue for later thread         */ 
/*    processing.                                                         */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    cdc_ecm                               Address of cdc_ecm class      */ 
/*                                          instance                      */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*   _ux_device_stack_transfer_request      Transfer request              */ 
/*   _ux_device_mutex_off                   Release mutex                 */
/*   _ux_device_event_flags_set             Set event flags               */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    ThreadX                                                             */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used UX prefix to refer to  */
/*                                            TX symbols instead of using */
/*                                            them directly,              */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed standalone compile,   */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_cdc_ecm_write(VOID *cdc_ecm_class, NX_PACKET *packet)
{
#if defined(UX_DEVICE_STANDALONE)
    UX_PARAMETER_NOT_USED(cdc_ecm_class);
    UX_PARAMETER_NOT_USED(packet);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UINT                        status;
UX_SLAVE_CLASS_CDC_ECM      *cdc_ecm;

    /* Proper class casting.  */
    cdc_ecm = (UX_SLAVE_CLASS_CDC_ECM *) cdc_ecm_class;

    /* Protect this thread.  */
    _ux_device_mutex_on(&cdc_ecm -> ux_slave_class_cdc_ecm_mutex);

    /* We only want to send the packet if the link is up.  */
    if (cdc_ecm->ux_slave_class_cdc_ecm_link_state == UX_DEVICE_CLASS_CDC_ECM_LINK_STATE_UP)
    {

        /* Check the queue. See if there is something that is being sent.  */
        if (cdc_ecm -> ux_slave_class_cdc_ecm_xmit_queue == UX_NULL)

            /* Memorize this packet at the beginning of the queue.  */
            cdc_ecm -> ux_slave_class_cdc_ecm_xmit_queue =  packet;

        else
        
            /* Add the packet to the end of the queue.  */
            cdc_ecm -> ux_slave_class_cdc_ecm_xmit_queue_tail -> nx_packet_queue_next =  packet;

        /* Set the tail.  */
        cdc_ecm -> ux_slave_class_cdc_ecm_xmit_queue_tail =  packet;
        
        /* The packet to be sent is the last in the chain.  */
        packet -> nx_packet_queue_next =  NX_NULL;

        /* Free Mutex resource.  */
        _ux_device_mutex_off(&cdc_ecm -> ux_slave_class_cdc_ecm_mutex);

        /* Set an event to wake up the bulkin thread.  */
        _ux_device_event_flags_set(&cdc_ecm -> ux_slave_class_cdc_ecm_event_flags_group, UX_DEVICE_CLASS_CDC_ECM_NEW_BULKIN_EVENT, UX_OR);                

        /* Packet successfully added. Return success.  */
        status =  UX_SUCCESS;
    }
    else
    {

        /* Free Mutex resource.  */
        _ux_device_mutex_off(&cdc_ecm -> ux_slave_class_cdc_ecm_mutex);

        /* Report error to application.  */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_CLASS, UX_CLASS_CDC_ECM_LINK_STATE_DOWN_ERROR);

        /* Return error.  */
        status =  UX_ERROR;
    }

    /* We are done here.  */
    return(status);            
#endif
}
