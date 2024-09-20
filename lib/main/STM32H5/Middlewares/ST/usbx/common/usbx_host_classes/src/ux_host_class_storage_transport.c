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
/**   Storage Class                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"
#include "ux_host_class_storage.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_host_class_storage_transport                    PORTABLE C      */
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the transport layer for all protocols. It perform  */
/*    the error recovery and retries if needed.                           */
/*                                                                        */
/*    It's for RTOS mode only.                                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    storage                               Pointer to storage class      */
/*    data_pointer                          Pointer to data               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    (ux_host_class_storage_transport)     Class storage transport       */
/*    _ux_host_class_storage_device_reset   Reset device                  */
/*    _ux_host_class_storage_request_sense  Class request sense           */
/*    _ux_host_stack_endpoint_reset         Reset endpoint                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Storage Class                                                       */
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
/*                                                                        */
/**************************************************************************/
UINT  _ux_host_class_storage_transport(UX_HOST_CLASS_STORAGE *storage, UCHAR *data_pointer)
{
#if defined(UX_HOST_STANDALONE)
    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(data_pointer);
    return(UX_FUNCTION_NOT_SUPPORTED);
#else

UINT        status;
UINT        csw_status;


    /* Reset the sense code.  */
    storage -> ux_host_class_storage_sense_code =  UX_SUCCESS;

    /* Send the command to the appropriate transport.  */
    status =  storage -> ux_host_class_storage_transport(storage, data_pointer);

#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    /* The treatment of errors is different according to the protocol used. BO and CB belong
       to the CSW group. CBI is separate.  */
    if (storage -> ux_host_class_storage_interface -> ux_interface_descriptor.bInterfaceProtocol != UX_HOST_CLASS_STORAGE_PROTOCOL_CBI)
    {
#endif

    /* Check the status.  */
    if (status != UX_SUCCESS)

        /* There was a more serious error. Just give up!  */
        return(status);

    /* The command transfer was OK but maybe we have a CSW error.  */
    csw_status =  storage -> ux_host_class_storage_csw[UX_HOST_CLASS_STORAGE_CSW_STATUS];
    if (csw_status == 0)
        return(UX_SUCCESS);

    /* Check for a command failure. If so, we need to sense the error with
       a REQUEST SENSE command.  */
    status =  _ux_host_class_storage_request_sense(storage);

    /* If we have an transport failure here, we are in trouble! The storage device is in
       an unstable state and should be reset completely.  */
    if (status != UX_SUCCESS)
    {

        /* Reset device.  */
        _ux_host_class_storage_device_reset(storage);
        return(status);
    }

    /* The sense code is saved in the device instance. We can return safely.  */
    return(UX_SUCCESS);

#ifdef UX_HOST_CLASS_STORAGE_INCLUDE_LEGACY_PROTOCOL_SUPPORT
    }
    else
    {

        switch (status)
        {

        case UX_SUCCESS:
            return(UX_SUCCESS);

        case UX_TRANSFER_STALLED:

            /* The endpoint was halted by a stall condition and needs to be reset.  */
            _ux_host_stack_endpoint_reset(storage -> ux_host_class_storage_bulk_in_endpoint);

            /* The endpoint was halted by a stall condition and needs to be reset.  */
            _ux_host_stack_endpoint_reset(storage -> ux_host_class_storage_bulk_out_endpoint);

            /* Check for a command failure. If so, we need to sense the error with
               a REQUEST SENSE command.  */
            status =  _ux_host_class_storage_request_sense(storage);
            return(status);

        default:

            /* There was a more serious error. Just give up!  */
            return(status);
        }
    }
#endif
#endif
}
