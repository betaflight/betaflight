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
/*    _ux_pictbridge_dpsclient_object_info_send           PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    Receive the info data set of a new object to be stored.             */ 
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
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed string length check,  */
/*                                            used macros for RTOS calls, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_pictbridge_dpsclient_object_info_send(UX_SLAVE_CLASS_PIMA *pima, UX_SLAVE_CLASS_PIMA_OBJECT *object, 
                                                ULONG storage_id,
                                                ULONG parent_object_handle,
                                                ULONG *object_handle)
{
UX_PICTBRIDGE                   *pictbridge;
UX_SLAVE_CLASS_PIMA_OBJECT      *object_info;
UCHAR                           string_discovery_name[UX_PICTBRIDGE_MAX_FILE_NAME_SIZE + 1]; /* +1 for null-terminator */
UINT                            length, length1;

    UX_PARAMETER_NOT_USED(storage_id);
    UX_PARAMETER_NOT_USED(parent_object_handle);

    /* Get the pointer to the Pictbridge instance.  */
    pictbridge = (UX_PICTBRIDGE *)pima -> ux_device_class_pima_application;

    /* We only have one object.  */
    object_info = (UX_SLAVE_CLASS_PIMA_OBJECT *) pictbridge -> ux_pictbridge_object_host;
    
     /* Copy the demanded object info set.  */
    _ux_utility_memory_copy(object_info, object, UX_SLAVE_CLASS_PIMA_OBJECT_DATA_LENGTH); /* Use case of memcpy is verified. */

    /* Store the object handle.  In Pictbridge we only receive XML scripts so the handle is hardwired to 1.  */
    object_info -> ux_device_class_pima_object_handle_id = 1;
    *object_handle =  1;

    /* Check state machine. If we are in discovery pending mode, check file name of this object.  */
    if (pictbridge -> ux_pictbridge_discovery_state == UX_PICTBRIDGE_DPSCLIENT_DISCOVERY_PENDING)
    {

        /* We are in the discovery mode. Check for file name. It must match HDISCVRY.DPS in Unicode mode.  */
        /* Check if this is a script.  */        
        if (object_info -> ux_device_class_pima_object_format == UX_DEVICE_CLASS_PIMA_OFC_SCRIPT)
        {

            /* Yes this is a script. We need to search for the HDISCVRY.DPS file name.
               Get the file name length (with null-terminator).  */
            length1 = (UINT)(*object_info -> ux_device_class_pima_object_filename);

            /* Now, compare it to the HDISCVRY.DPS file name.  Check length first.  */
            if (length1 <= UX_PICTBRIDGE_MAX_FILE_NAME_SIZE)
            {

                /* Invalidate length, on error it's untouched.  */
                length = UX_PICTBRIDGE_MAX_FILE_NAME_SIZE + 1;
                _ux_utility_string_length_check(_ux_pictbridge_hdiscovery_name, &length, UX_PICTBRIDGE_MAX_FILE_NAME_SIZE);
                if ((length + 1) == length1)
                {

                    /* Get the file name in a ascii format (with null-terminator). */
                    _ux_utility_unicode_to_string(object_info -> ux_device_class_pima_object_filename, string_discovery_name);
                
                    /* So far, the length of name of the files are the same.
                    Compare names now (since length is same just compare without null-terminator). */
                    if (_ux_utility_memory_compare(_ux_pictbridge_hdiscovery_name, string_discovery_name,
                                                    length) ==  UX_SUCCESS)
                    {
                        
                        /* We are done with discovery of the printer. We can now send notifications when the camera wants to print an object.  */        
                        pictbridge -> ux_pictbridge_discovery_state = UX_PICTBRIDGE_DPSCLIENT_DISCOVERY_COMPLETE;
                        
                        /* Set an event flag if the application is listening.  */
                        _ux_system_event_flags_set(&pictbridge -> ux_pictbridge_event_flags_group, UX_PICTBRIDGE_EVENT_FLAG_DISCOVERY, UX_OR);
                        
                        /* There is no object during the discovery cycle.  */
                        return(UX_SUCCESS);
                    }
                }
            }
        }            
    }

    /* What cycle are we in ? */
    if (pictbridge -> ux_pictbridge_host_client_state_machine == UX_PICTBRIDGE_STATE_MACHINE_IDLE)
    
        /* Since we are in idle state, we must have received a request from the host.  */
        pictbridge -> ux_pictbridge_host_client_state_machine = UX_PICTBRIDGE_STATE_MACHINE_HOST_REQUEST;
    

    /* We have copied the requested data. Return OK.  */
    return(UX_SUCCESS);
}

