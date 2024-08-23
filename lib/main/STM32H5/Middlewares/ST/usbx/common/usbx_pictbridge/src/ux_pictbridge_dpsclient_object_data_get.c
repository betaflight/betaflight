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
/**   Pictbridge Application                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_pictbridge.h"
#include "ux_device_class_pima.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_pictbridge_dpsclient_object_data_get            PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns the data of the object.                       */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                   Pima instance associated     */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    user application                                                    */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            verified memset and memcpy  */
/*                                            cases, used UX prefix to    */
/*                                            refer to TX symbols instead */
/*                                            of using them directly,     */
/*                                            resulting in version 6.1    */
/*  04-25-2022     Yajun Xia                Modified comment(s),          */
/*                                            internal clean up,          */
/*                                            resulting in version 6.1.11 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used macros for RTOS calls, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpsclient_object_data_get(UX_SLAVE_CLASS_PIMA *pima, ULONG object_handle, UCHAR *object_buffer, ULONG object_offset,
                                                                ULONG object_length_requested, ULONG *object_actual_length)
{
UX_PICTBRIDGE                   *pictbridge;
UX_SLAVE_CLASS_PIMA_OBJECT      *object_info;
UCHAR                           *pima_object_buffer;
ULONG                           actual_length;
UINT                            status;

    /* Get the pointer to the Pictbridge instance.  */
    pictbridge = (UX_PICTBRIDGE *)pima -> ux_device_class_pima_application;

    /* Check the object handle. If this is handle 1 or 2 , we need to return the XML script object.
       If the handle is not 1 or 2, this is a JPEG picture or other object to be printed.  */
    if ((object_handle == UX_PICTBRIDGE_OBJECT_HANDLE_HOST_RESPONSE) || (object_handle == UX_PICTBRIDGE_OBJECT_HANDLE_CLIENT_REQUEST))
    {
    
        /* Check what XML object is requested. It is either a request script or a response.  */
        if (object_handle == UX_PICTBRIDGE_OBJECT_HANDLE_HOST_RESPONSE)       
            object_info = (UX_SLAVE_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_host;
        else        
            object_info = (UX_SLAVE_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_client;
    
        /* Is this the correct handle ? */
        if (object_info ->  ux_device_class_pima_object_handle_id == object_handle)
        {
    
            /* Get the pointer to the object buffer.  */
            pima_object_buffer = object_info -> ux_device_class_pima_object_buffer;
    
            /* Copy the demanded object data portion.  */
            _ux_utility_memory_copy(object_buffer, pima_object_buffer + object_offset, object_length_requested); /* Use case of memcpy is verified. */
    
            /* Update the length requested. for a demo, we do not do any checking.  */
            *object_actual_length = object_length_requested;
            
            /* What cycle are we in ? */
            if (pictbridge -> ux_pictbridge_host_client_state_machine & UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST)
            {
                /* Check if we are blocking for a client request.  */
                if (pictbridge -> ux_pictbridge_host_client_state_machine & UX_PICTBRIDGE_STATE_MACHINE_CLIENT_REQUEST_PENDING)
                
                    /* Yes we are pending, send an event to release the pending request.  */
                    _ux_system_event_flags_set(&pictbridge -> ux_pictbridge_event_flags_group, UX_PICTBRIDGE_EVENT_FLAG_STATE_MACHINE_READY, UX_OR);                

                /* Since we are in host request, this indicates we are done with the cycle.  */
                pictbridge -> ux_pictbridge_host_client_state_machine = UX_PICTBRIDGE_STATE_MACHINE_IDLE;
    
            }            
    
            /* We have copied the requested data. Return OK.  */
            return(UX_SUCCESS);
        }    
    }
    else
    {
        
        /* Obtain the data from the application jobinfo callback.  */
        status = pictbridge -> ux_pictbridge_jobinfo.ux_pictbridge_jobinfo_object_data_read(pictbridge, object_handle, object_buffer, object_offset,
                                                                                    object_length_requested, &actual_length);

        /* Save the length returned.  */
        *object_actual_length =  actual_length;

        /* Return the application status.  */
        return(status);

    }    
    /* Could not find the handle.  */
    return(UX_DEVICE_CLASS_PIMA_RC_INVALID_OBJECT_HANDLE);
}

