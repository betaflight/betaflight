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
/**   PIMA Class                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_pima.h"
#include "ux_host_stack.h"


/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_host_class_pima_object_open                     PORTABLE C      */ 
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function opens an object for a read or write operation         */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                       Pointer to pima class    */ 
/*    pima_session                               Pointer to pima session  */ 
/*    object_handle                              The object handle        */ 
/*    object                                     Pointer to object info   */ 
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
/*    USB application                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_pima_object_open(UX_HOST_CLASS_PIMA *pima, 
                                        UX_HOST_CLASS_PIMA_SESSION *pima_session,
                                        ULONG object_handle, UX_HOST_CLASS_PIMA_OBJECT *object)
{

    UX_PARAMETER_NOT_USED(object_handle);
    UX_PARAMETER_NOT_USED(pima);

    /* Check if this session is valid or not.  */
    if (pima_session -> ux_host_class_pima_session_magic != UX_HOST_CLASS_PIMA_MAGIC_NUMBER)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check if this session is opened or not.  */
    if (pima_session -> ux_host_class_pima_session_state != UX_HOST_CLASS_PIMA_SESSION_STATE_OPENED)
        return (UX_HOST_CLASS_PIMA_RC_SESSION_NOT_OPEN);

    /* Check if the object is already opened.  */
    if (object -> ux_host_class_pima_object_state != UX_HOST_CLASS_PIMA_OBJECT_STATE_CLOSED)
        return (UX_HOST_CLASS_PIMA_RC_OBJECT_ALREADY_OPENED);    

    /* Open the object.  */
    object -> ux_host_class_pima_object_state = UX_HOST_CLASS_PIMA_OBJECT_STATE_OPENED;

    /* Reset the object offset.  */
    object -> ux_host_class_pima_object_offset =  0;
    
    /* Set the object transfer status to inactive.  */
    object -> ux_host_class_pima_object_transfer_status = UX_HOST_CLASS_PIMA_OBJECT_TRANSFER_STATUS_INACTIVE;
    
    /* Return completion status.  */
    return(UX_SUCCESS);
}

