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
/**   Device Printer Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_printer.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_printer_soft_reset                 PORTABLE C      */
/*                                                           6.2.0        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function Perform Soft Reset on the Printer class.              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    printer                               Address of printer class      */
/*                                            instance                    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_device_stack_transfer_all_request_abort                         */
/*                                          Abort transfers on endpoint   */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Device Printer Class                                                */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  01-31-2022     Chaoqiong Xiao           Initial Version 6.1.10        */
/*  10-31-2022     Yajun Xia                Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.2.0  */
/*                                                                        */
/**************************************************************************/
VOID _ux_device_class_printer_soft_reset(UX_DEVICE_CLASS_PRINTER *printer)
{

UX_SLAVE_ENDPOINT                       *endpoint;

    /* Stop OUT.  */
    endpoint = printer -> ux_device_class_printer_endpoint_out;
    _ux_device_stack_transfer_all_request_abort(endpoint, UX_ENDPOINT_RESET);
#if defined(UX_DEVICE_STANDALONE)
    printer -> ux_device_class_printer_write_state = UX_STATE_RESET;
#endif

    /* Stop IN (optional).  */
    endpoint = printer -> ux_device_class_printer_endpoint_in;
    if (endpoint != UX_NULL)
    {
        _ux_device_stack_transfer_all_request_abort(endpoint, UX_ENDPOINT_RESET);
#if defined(UX_DEVICE_STANDALONE)
        printer -> ux_device_class_printer_read_state = UX_STATE_RESET;
#endif
    }
}
