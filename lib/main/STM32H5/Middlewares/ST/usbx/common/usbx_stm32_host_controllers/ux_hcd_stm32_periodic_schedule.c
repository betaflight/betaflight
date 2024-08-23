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
/**   STM32 Controller Driver                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE
#define UX_HCD_STM32_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_stm32.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_hcd_stm32_periodic_schedule                     PORTABLE C      */
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     This function schedules new transfers from the periodic interrupt  */
/*     list.                                                              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_stm32                           Pointer to STM32 controller     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    TRUE or FALSE                                                       */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    HAL_HCD_GetCurrentFrame             Get frame number                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added ISO transfer support, */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_stm32_periodic_schedule(UX_HCD_STM32 *hcd_stm32)
{

UX_HCD_STM32_ED     *ed;
UX_TRANSFER         *transfer_request;
ULONG               frame_index;
UX_DEVICE           *parent_device;
UX_ENDPOINT         *endpoint;
UX_ENDPOINT         *parent_endpoint;
ULONG               ep_schedule = 1U;
USHORT              port_status_change_bits;

    /* Get the current frame number.  */
    frame_index = HAL_HCD_GetCurrentFrame(hcd_stm32 -> hcd_handle);

    /* Get the first ED in the periodic list.  */
    ed =  hcd_stm32 -> ux_hcd_stm32_periodic_ed_head;

    /* Search for an entry in the periodic tree.  */
    while (ed != UX_NULL)
    {
#if defined (USBH_HAL_HUB_SPLIT_SUPPORTED)
      if (hcd_stm32 -> hcd_handle -> hc[ed -> ux_stm32_ed_channel].do_ssplit == 1U)
      {
        /* Get the transfer request.  */
        transfer_request = ed -> ux_stm32_ed_transfer_request;

        if (transfer_request != NULL)
        {
          if ((frame_index & ed -> ux_stm32_ed_interval_mask) == ed -> ux_stm32_ed_interval_position)
          {
            hcd_stm32 -> hcd_handle -> hc[ed -> ux_stm32_ed_channel].ep_ss_schedule = 1U;
          }

          /* Schedule Start & Complete split where the entire split transaction is completely bounded by a
          frame FS/LS devices */
          if (((((frame_index & 0x7U) < 0x3U) || ((frame_index & 0x7U) == 0x7U)) &&
               (hcd_stm32 -> hcd_handle -> hc[ed -> ux_stm32_ed_channel].ep_ss_schedule == 1U)) ||
                ((hcd_stm32 -> hcd_handle -> hc[ed -> ux_stm32_ed_channel].do_csplit == 1U) &&
                 (frame_index > (ed -> ux_stm32_ed_current_ss_frame + 1U))))
          {
            if (hcd_stm32 -> hcd_handle -> hc[ed -> ux_stm32_ed_channel].ep_ss_schedule == 1U)
            {
              hcd_stm32 -> hcd_handle -> hc[ed -> ux_stm32_ed_channel].ep_ss_schedule = 0U;
              ed -> ux_stm32_ed_current_ss_frame = frame_index;
            }

            /* Check if there is transfer needs schedule.  */
            if (ed -> ux_stm32_ed_sch_mode)
            {
              /* If it's scheduled each SOF/uSOF, the request should be submitted
              * immediately after packet is done. This is performed in callback.  */
              if (ed -> ux_stm32_ed_interval_mask == 0U)
                ed -> ux_stm32_ed_sch_mode = 0U;

              /* For ISO OUT, packet size is from request variable,
              * otherwise, use request length.  */
              if ((ed -> ux_stm32_ed_type == EP_TYPE_ISOC) && (ed -> ux_stm32_ed_dir == 0U))
                ed -> ux_stm32_ed_packet_length = transfer_request -> ux_transfer_request_packet_length;
              else
                ed -> ux_stm32_ed_packet_length = transfer_request -> ux_transfer_request_requested_length;

              /* Prepare transactions.  */
              _ux_hcd_stm32_request_trans_prepare(hcd_stm32, ed, transfer_request);

              /* Get the pointer to the Endpoint.  */
              endpoint = (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

              /* Check if device connected to hub  */
              if (endpoint->ux_endpoint_device->ux_device_parent != NULL)
              {
                parent_device = endpoint->ux_endpoint_device->ux_device_parent;
                if (parent_device->ux_device_current_configuration->ux_configuration_first_interface->ux_interface_descriptor.bInterfaceClass == 0x9U)
                {
                  parent_endpoint = parent_device->ux_device_current_configuration->ux_configuration_first_interface->ux_interface_first_endpoint;

                  if (parent_endpoint->ux_endpoint_transfer_request.ux_transfer_request_actual_length != 0U)
                  {
                    /* The interrupt pipe buffer contains the status change for each of the ports
                    the length of the buffer can be 1 or 2 depending on the number of ports.
                    Usually, since HUBs can be bus powered the maximum number of ports is 4.
                    We must be taking precautions on how we read the buffer content for
                    big endian machines.  */
                    if (parent_endpoint->ux_endpoint_transfer_request.ux_transfer_request_actual_length == 1U)
                      port_status_change_bits = *(USHORT *) parent_endpoint->ux_endpoint_transfer_request.ux_transfer_request_data_pointer;
                    else
                      port_status_change_bits = (USHORT)_ux_utility_short_get(parent_endpoint->ux_endpoint_transfer_request.ux_transfer_request_data_pointer);

                    if ((port_status_change_bits & (0x1U << endpoint->ux_endpoint_device->ux_device_port_location)) != 0U)
                    {
                      ep_schedule = 0U;
                    }
                  }
                }
              }

              if ((endpoint->ux_endpoint_device->ux_device_state == UX_DEVICE_CONFIGURED) && (ep_schedule != 0U))
              {
                /* Call HAL driver to submit the transfer request.  */
                HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel,
                                         ed -> ux_stm32_ed_dir,
                                         ed -> ux_stm32_ed_type, USBH_PID_DATA,
                                         ed -> ux_stm32_ed_data + transfer_request -> ux_transfer_request_actual_length,
                                         ed -> ux_stm32_ed_packet_length, 0U);
              }
            }
          }
        }
      }
      else
#endif /* defined (USBH_HAL_HUB_SPLIT_SUPPORTED) */
      {
        /* Check if the periodic transfer should be scheduled in this frame.  */
        /* Interval Mask is 0:     it's scheduled every SOF/uSOF.  */
        /* Interval Mask is not 0: check position to see if it's scheduled.  */
        if ((frame_index & ed -> ux_stm32_ed_interval_mask) == ed -> ux_stm32_ed_interval_position)
        {

          /* Get the transfer request.  */
          transfer_request = ed -> ux_stm32_ed_transfer_request;

          /* Check if there is transfer needs schedule.  */
          if (transfer_request && ed -> ux_stm32_ed_sch_mode)
          {

            /* If it's scheduled each SOF/uSOF, the request should be submitted
            * immediately after packet is done. This is performed in callback.  */
            if (ed -> ux_stm32_ed_interval_mask == 0U)
              ed -> ux_stm32_ed_sch_mode = 0U;

            /* For ISO OUT, packet size is from request variable,
            * otherwise, use request length.  */
            if ((ed -> ux_stm32_ed_type == EP_TYPE_ISOC) && (ed -> ux_stm32_ed_dir == 0U))
              ed -> ux_stm32_ed_packet_length = transfer_request -> ux_transfer_request_packet_length;
            else
              ed -> ux_stm32_ed_packet_length = transfer_request -> ux_transfer_request_requested_length;

            /* Prepare transactions.  */
            _ux_hcd_stm32_request_trans_prepare(hcd_stm32, ed, transfer_request);

            /* Get the pointer to the Endpoint.  */
            endpoint = (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

            /* Check if device connected to hub  */
            if (endpoint->ux_endpoint_device->ux_device_parent != NULL)
            {
              parent_device = endpoint->ux_endpoint_device->ux_device_parent;
              if (parent_device->ux_device_current_configuration->ux_configuration_first_interface->ux_interface_descriptor.bInterfaceClass == 0x9U)
              {
                parent_endpoint = parent_device->ux_device_current_configuration->ux_configuration_first_interface->ux_interface_first_endpoint;

                if (parent_endpoint->ux_endpoint_transfer_request.ux_transfer_request_actual_length != 0U)
                {
                  /* The interrupt pipe buffer contains the status change for each of the ports
                  the length of the buffer can be 1 or 2 depending on the number of ports.
                  Usually, since HUBs can be bus powered the maximum number of ports is 4.
                  We must be taking precautions on how we read the buffer content for
                  big endian machines.  */
                  if (parent_endpoint->ux_endpoint_transfer_request.ux_transfer_request_actual_length == 1U)
                    port_status_change_bits = *(USHORT *) parent_endpoint->ux_endpoint_transfer_request.ux_transfer_request_data_pointer;
                  else
                    port_status_change_bits = (USHORT)_ux_utility_short_get(parent_endpoint->ux_endpoint_transfer_request.ux_transfer_request_data_pointer);

                  if ((port_status_change_bits & (0x1U << endpoint->ux_endpoint_device->ux_device_port_location)) != 0U)
                  {
                    ep_schedule = 0U;
                  }
                }
              }
            }

            if ((endpoint->ux_endpoint_device->ux_device_state == UX_DEVICE_CONFIGURED) && (ep_schedule != 0U))
            {
              /* Call HAL driver to submit the transfer request.  */
              HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel,
                                       ed -> ux_stm32_ed_dir,
                                       ed -> ux_stm32_ed_type, USBH_PID_DATA,
                                       ed -> ux_stm32_ed_data + transfer_request -> ux_transfer_request_actual_length,
                                       ed -> ux_stm32_ed_packet_length, 0U);
            }
          }
        }
      }

        /* Point to the next ED in the list.  */
        ed =  ed -> ux_stm32_ed_next_ed;
    }

    /* Return to caller.  */
    return(UX_FALSE);
}

