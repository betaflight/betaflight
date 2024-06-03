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
/**   System                                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_system.h                                         PORTABLE C      */ 
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX main system component.                                         */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  12-31-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added BOS support,          */
/*                                            resulting in version 6.1.3  */
/*  01-31-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added standalone support,   */
/*                                            added device printer name,  */
/*                                            resulting in version 6.1.10 */
/*  04-25-2022     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added device CCID name,     */
/*                                            resulting in version 6.1.11 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_SYSTEM_HOST_H
#define UX_SYSTEM_HOST_H

/* Define System component function prototypes.  Note that since ux_api.h 
   includes this file, the APIs are only declared if this file is included 
   by internal code in order to prevent duplicate declarations for 
   applications.  */


#ifdef UX_SOURCE_CODE
UINT  _ux_system_initialize(VOID *regular_memory_pool_start, ULONG regular_memory_size, 
                            VOID *cache_safe_memory_pool_start, ULONG cache_safe_memory_size);
UINT  _ux_system_uninitialize(VOID);
UINT  _ux_system_tasks_run(VOID);
#endif

/* Define System component external data references.  */

extern UX_SYSTEM *_ux_system;
extern UX_SYSTEM_HOST *_ux_system_host;
extern UX_SYSTEM_SLAVE *_ux_system_slave;
extern UX_SYSTEM_OTG *_ux_system_otg;
extern UCHAR _ux_system_endpoint_descriptor_structure[];
extern UCHAR _ux_system_device_descriptor_structure[];
extern UCHAR _ux_system_configuration_descriptor_structure[];
extern UCHAR _ux_system_interface_descriptor_structure[];
extern UCHAR _ux_system_interface_association_descriptor_structure[];
extern UCHAR _ux_system_string_descriptor_structure[];
extern UCHAR _ux_system_dfu_functional_descriptor_structure[];
extern UCHAR _ux_system_hub_descriptor_structure[];
extern UCHAR _ux_system_hid_descriptor_structure[];
extern UCHAR _ux_system_class_audio_interface_descriptor_structure[];
extern UCHAR _ux_system_class_audio_input_terminal_descriptor_structure[];
extern UCHAR _ux_system_class_audio_output_terminal_descriptor_structure[];
extern UCHAR _ux_system_class_audio_feature_unit_descriptor_structure[];
extern UCHAR _ux_system_class_audio_streaming_interface_descriptor_structure[];
extern UCHAR _ux_system_class_audio_streaming_endpoint_descriptor_structure[];
extern UCHAR _ux_system_class_pima_storage_structure[];
extern UCHAR _ux_system_class_pima_object_structure[];
extern UCHAR _ux_system_ecm_interface_descriptor_structure[];

extern UCHAR _ux_system_bos_descriptor_structure[];
extern UCHAR _ux_system_usb_2_0_extension_descriptor_structure[];
extern UCHAR _ux_system_container_id_descriptor_structure[];

extern UINT  _ux_system_host_hcd_periodic_tree_entries[32]; 

extern UCHAR _ux_system_host_class_hub_name[];  
extern UCHAR _ux_system_host_class_printer_name[]; 
extern UCHAR _ux_system_host_class_storage_name[]; 
extern UCHAR _ux_system_host_class_hid_name[];     
extern UCHAR _ux_system_host_class_audio_name[];   
extern UCHAR _ux_system_host_class_cdc_acm_name[];   
extern UCHAR _ux_system_host_class_cdc_dlc_name[];   
extern UCHAR _ux_system_host_class_cdc_ecm_name[];   
extern UCHAR _ux_system_host_class_prolific_name[];   
extern UCHAR _ux_system_host_class_dpump_name[];  
extern UCHAR _ux_system_host_class_pima_name[];  
extern UCHAR _ux_system_host_class_asix_name[];   
extern UCHAR _ux_system_host_class_swar_name[];   
extern UCHAR _ux_system_host_class_gser_name[];   
extern UCHAR _ux_system_host_class_hid_client_remote_control_name[];
extern UCHAR _ux_system_host_class_hid_client_mouse_name[]; 
extern UCHAR _ux_system_host_class_hid_client_keyboard_name[]; 

extern UCHAR _ux_system_host_hcd_ohci_name[]; 
extern UCHAR _ux_system_host_hcd_ehci_name[]; 
extern UCHAR _ux_system_host_hcd_isp1161_name[]; 
extern UCHAR _ux_system_host_hcd_isp1362_name[]; 
extern UCHAR _ux_system_host_hcd_sh2_name[]; 
extern UCHAR _ux_system_host_hcd_rx_name[]; 
extern UCHAR _ux_system_host_hcd_pic32_name[]; 
extern UCHAR _ux_system_host_hcd_stm32_name[]; 
extern UCHAR _ux_system_host_hcd_musb_name[];
extern UCHAR _ux_system_host_hcd_atm7_name[];
extern UCHAR _ux_system_host_hcd_simulator_name[]; 

extern UCHAR _ux_system_slave_class_storage_name[]; 
extern UCHAR _ux_system_slave_class_storage_vendor_id[]; 
extern UCHAR _ux_system_slave_class_storage_product_id[]; 
extern UCHAR _ux_system_slave_class_storage_product_rev[]; 
extern UCHAR _ux_system_slave_class_storage_product_serial[]; 
extern UCHAR _ux_system_slave_class_audio_name[]; 
extern UCHAR _ux_system_slave_class_cdc_acm_name[]; 
extern UCHAR _ux_system_slave_class_dpump_name[]; 
extern UCHAR _ux_system_slave_class_pima_name[]; 
extern UCHAR _ux_system_slave_class_hid_name[]; 
extern UCHAR _ux_system_slave_class_rndis_name[]; 
extern UCHAR _ux_system_slave_class_cdc_ecm_name[]; 
extern UCHAR _ux_system_slave_class_dfu_name[];

extern UCHAR _ux_system_device_class_printer_name[];
extern UCHAR _ux_system_device_class_ccid_name[];

#if defined(UX_HOST_SIDE_ONLY)
#define _ux_system_host_tasks_run      _ux_host_stack_tasks_run
#else
#define _ux_system_host_tasks_run      _ux_system_tasks_run
#endif

#if defined(UX_DEVICE_SIDE_ONLY)
#define _ux_system_device_tasks_run    _ux_device_stack_tasks_run
#else
#define _ux_system_device_tasks_run    _ux_system_tasks_run
#endif

#endif

