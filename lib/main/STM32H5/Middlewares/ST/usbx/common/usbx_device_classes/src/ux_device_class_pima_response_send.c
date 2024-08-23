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
/**   Device PIMA Class                                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_pima.h"
#include "ux_device_stack.h"

/**************************************************************************/ 
/*                                                                        */ 
/*  FUNCTION                                               RELEASE        */ 
/*                                                                        */ 
/*    _ux_device_class_pima_response_send                 PORTABLE C      */ 
/*                                                           6.1.10       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This function returns a response to a pima command to the host.     */ 
/*                                                                        */ 
/*  INPUT                                                                 */ 
/*                                                                        */ 
/*    pima                                  Pointer to pima class         */ 
/*    response_code                         Response code                 */ 
/*                                                                        */ 
/*  OUTPUT                                                                */ 
/*                                                                        */ 
/*    Completion Status                                                   */ 
/*                                                                        */ 
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_utility_long_put                  Put 32-bit value              */
/*    _ux_utility_short_put                 Put 32-bit value              */
/*    _ux_device_class_pima_response_send   Send PIMA response            */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Storage Class                                                */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            updated command phase,      */
/*                                            resulting in version 6.1.10 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_class_pima_response_send(UX_SLAVE_CLASS_PIMA *pima, ULONG response_code, 
                      ULONG number_parameters,
                      ULONG pima_parameter_1, 
                      ULONG pima_parameter_2, 
                      ULONG pima_parameter_3) 
{

UINT                    status;
UX_SLAVE_TRANSFER       *transfer_request;
UCHAR                   *response;
ULONG                   header_size;

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_CLASS_PIMA_RESPONSE_SEND, pima, response_code, number_parameters, pima_parameter_1, UX_TRACE_DEVICE_CLASS_EVENTS, 0, 0)

    /* Obtain the pointer to the transfer request.  */
    transfer_request =  &pima -> ux_device_class_pima_bulk_in_endpoint -> ux_slave_endpoint_transfer_request;

    /* Calculate the header size.  */
    header_size = UX_DEVICE_CLASS_PIMA_DATA_HEADER_SIZE + (ULONG)(sizeof(ULONG) * number_parameters);

    /* Obtain memory for this response. Use the transfer request pre-allocated memory.  */
    response =  transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Fill in the size of the response header.  */
    _ux_utility_long_put(response + UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_LENGTH, header_size);
    
    /* Fill in the response container type.  */
    _ux_utility_short_put(response + UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_TYPE,
                            UX_DEVICE_CLASS_PIMA_CT_RESPONSE_BLOCK);
    
    /* Fill in the response code.  */
    _ux_utility_short_put(response + UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_CODE,
            (USHORT)response_code);
    
    /* Fill in the Transaction ID.  */
    _ux_utility_long_put(response + UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_TRANSACTION_ID, 
                            pima -> ux_device_class_pima_transaction_id);

    /* Parse each parameter and insert it if needed.  */
    switch (number_parameters)
    {

        case 3 :
            _ux_utility_long_put(response + UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_PARAMETERS + (sizeof(ULONG) * 2), 
                                 pima_parameter_3);
            /* Intentionally fallthrough to "case 2" */
            /* fall through */
        case 2 :                            
            _ux_utility_long_put(response + UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_PARAMETERS + (sizeof(ULONG) * 1), 
                                 pima_parameter_2);
            /* Intentionally fallthrough to "case 1" */
            /* fall through */
        case 1 :                            
            _ux_utility_long_put(response + UX_DEVICE_CLASS_PIMA_RESPONSE_HEADER_PARAMETERS , 
                                 pima_parameter_1);
            /* Intentionally fallthrough to "default" */
            /* fall through */
        default :
            break;
            
    }    

    /* Set phase to response.  */
    pima -> ux_device_class_pima_state = UX_DEVICE_CLASS_PIMA_PHASE_RESPONSE;

    /* Send the response block.  */
    status =  _ux_device_stack_transfer_request(transfer_request, header_size, 0);
    
    /* Return completion status.  */
    return(status);
}

