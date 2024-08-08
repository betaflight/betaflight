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
/**   Device RNDIS Class                                                  */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_rndis.h"
#include "ux_device_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_rndis_write                        PORTABLE C      */ 
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
/*    rndis                                   Address of rndis class      */ 
/*                                                instance                */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    None                                                                */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*   _ux_device_mutex_on                     Take mutex                  */
/*   _ux_device_mutex_off                    Free mutex                  */
/*   _ux_device_event_flags_set              Set event flags             */
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
UINT  _ux_device_class_rndis_write(VOID *rndis_class, NX_PACKET *packet)
{
#if defined(UX_DEVICE_STANDALONE)
    UX_PARAMETER_NOT_USED(rndis_class);
    UX_PARAMETER_NOT_USED(packet);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

NX_PACKET               *current_packet;
NX_PACKET               *next_packet;
UX_SLAVE_CLASS_RNDIS     *rndis;

    /* Proper class casting.  */
    rndis = (UX_SLAVE_CLASS_RNDIS *) rndis_class;

    /* Protect this thread.  */
    _ux_device_mutex_on(&rndis -> ux_slave_class_rndis_mutex);
            
    /* Check the queue. See if there is something that is being sent. */
    if (rndis -> ux_slave_class_rndis_xmit_queue == UX_NULL)
        
        /* Memorize this packet at the beginning of the queue.  */
        rndis -> ux_slave_class_rndis_xmit_queue = packet;
        
    else
    
    {
    
        /* We get here when there is something in the queue.  */
        current_packet =  rndis -> ux_slave_class_rndis_xmit_queue;

        /* Get the next packet associated with the first packet.  */
        next_packet = current_packet -> nx_packet_queue_next;

        /* Parse the current chain for the end.  */
        while (next_packet != NX_NULL)
        {
            /* Remember the current packet.  */
            current_packet = next_packet;
            
            /* See what the next packet in the chain is.  */
            next_packet = current_packet -> nx_packet_queue_next;
        }

        /* Memorize the packet to be sent.  */
        current_packet -> nx_packet_queue_next = packet;

    }

    /* Free Mutex resource.  */
    _ux_device_mutex_off(&rndis -> ux_slave_class_rndis_mutex);
    
    /* The packet to be sent is the last in the chain.  */
    packet -> nx_packet_queue_next = NX_NULL;

    /* Set an event to wake up the bulkin thread.  */
    _ux_device_event_flags_set(&rndis -> ux_slave_class_rndis_event_flags_group, UX_DEVICE_CLASS_RNDIS_NEW_BULKIN_EVENT, UX_OR);                

    /* We are done here.  */
    return(UX_SUCCESS);            
#endif
}
