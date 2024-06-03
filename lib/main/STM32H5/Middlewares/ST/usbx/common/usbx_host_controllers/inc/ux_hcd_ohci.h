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
/**   OHCI Controller Driver                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_hcd_ohci.h                                       PORTABLE C      */ 
/*                                                           6.1.12       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX host OHCI Controller.                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used unsigned defines,      */
/*                                            resulting in version 6.1.2  */
/*  08-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*  01-31-2022     Xiuwen Cai               Modified comment(s),          */
/*                                            fixed HcPeriodicStart value,*/
/*                                            resulting in version 6.1.10 */
/*  07-29-2022     Yajun Xia                Modified comment(s),          */
/*                                            fixed OHCI PRSC issue,      */
/*                                            resulting in version 6.1.12 */
/*                                                                        */
/**************************************************************************/

#ifndef UX_HCD_OHCI_H
#define UX_HCD_OHCI_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  


/* Define generic OHCI constants.  */

#define UX_OHCI_CONTROLLER                                  1
#define UX_OHCI_MAX_PAYLOAD                                 4096
#define UX_OHCI_FRAME_DELAY                                 4u   


/* Define OHCI HCOR register mapping.  */

#define OHCI_HC_REVISION                                    0x00
#define OHCI_HC_CONTROL                                     0x01
#define OHCI_HC_COMMAND_STATUS                              0x02
#define OHCI_HC_INTERRUPT_STATUS                            0x03
#define OHCI_HC_INTERRUPT_ENABLE                            0x04
#define OHCI_HC_INTERRUPT_DISABLE                           0x05
#define OHCI_HC_HCCA                                        0x06
#define OHCI_HC_PERIOD_CURRENT_ED                           0x07
#define OHCI_HC_CONTROL_HEAD_ED                             0x08
#define OHCI_HC_CONTROL_CURRENT_ED                          0x09
#define OHCI_HC_BULK_HEAD_ED                                0x0a
#define OHCI_HC_BULK_CURRENT_ED                             0x0b
#define OHCI_HC_DONE_HEAD                                   0x0c
#define OHCI_HC_FM_INTERVAL                                 0x0d
#define OHCI_HC_FM_REMAINING                                0x0e
#define OHCI_HC_FM_NUMBER                                   0x0f
#define OHCI_HC_PERIODIC_START                              0x10
#define OHCI_HC_LS_THRESHOLD                                0x11
#define OHCI_HC_RH_DESCRIPTOR_A                             0x12
#define OHCI_HC_RH_DESCRIPTOR_B                             0x13
#define OHCI_HC_RH_STATUS                                   0x14
#define OHCI_HC_RH_PORT_STATUS                              0x15


/* Define OHCI control register values.  */

#define OHCI_HC_CR_CBSR_0                                   0x00000000u
#define OHCI_HC_CR_CBSR_1                                   0x00000001u
#define OHCI_HC_CR_CBSR_2                                   0x00000002u
#define OHCI_HC_CR_CBSR_3                                   0x00000003u
#define OHCI_HC_CR_PLE                                      0x00000004u   
#define OHCI_HC_CR_IE                                       0x00000008u   
#define OHCI_HC_CR_CLE                                      0x00000010u   
#define OHCI_HC_CR_BLE                                      0x00000020u   
#define OHCI_HC_CR_RESET                                    0x00000000u
#define OHCI_HC_CR_RESUME                                   0x00000040u
#define OHCI_HC_CR_OPERATIONAL                              0x00000080u
#define OHCI_HC_CR_SUSPEND                                  0x000000c0u
#define OHCI_HC_CR_IR                                       0x00000100u   
#define OHCI_HC_CR_RWC                                      0x00000200u   
#define OHCI_HC_CR_RWE                                      0x00000400u   

#define OHCI_HC_CONTROL_VALUE                               (OHCI_HC_CR_CBSR_3 | OHCI_HC_CR_OPERATIONAL | OHCI_HC_CR_PLE | OHCI_HC_CR_IE | OHCI_HC_CR_CLE | OHCI_HC_CR_BLE)


/* Define OHCI HCOR command/status bitmaps.  */ 

#define OHCI_HC_CS_HCR                                      0x00000001u
#define OHCI_HC_CS_CLF                                      0x00000002u
#define OHCI_HC_CS_BLF                                      0x00000004u


#define OHCI_HC_RH_PSM                                      0x00000100u
#define OHCI_HC_RH_NPS                                      0x00000200u
#define OHCI_HC_RH_DT                                       0x00000400u
#define OHCI_HC_RH_OCPM                                     0x00000800u
#define OHCI_HC_RH_NOCP                                     0x00001000u
#define OHCI_HC_RH_POTPGT                                   24u

#define OHCI_HC_RS_LPS                                      0x00000001u
#define OHCI_HC_RS_OCI                                      0x00000002u
#define OHCI_HC_RS_DRWE                                     0x00008000u
#define OHCI_HC_RS_LPSC                                     0x00010000u
#define OHCI_HC_RS_OCIC                                     0x00020000u
#define OHCI_HC_RS_CRWE                                     0x80000000u

#define OHCI_HC_PS_CCS                                      0x00000001u
#define OHCI_HC_PS_CPE                                      0x00000001u
#define OHCI_HC_PS_PES                                      0x00000002u
#define OHCI_HC_PS_PSS                                      0x00000004u
#define OHCI_HC_PS_POCI                                     0x00000008u
#define OHCI_HC_PS_PRS                                      0x00000010u
#define OHCI_HC_PS_PPS                                      0x00000100u
#define OHCI_HC_PS_LSDA                                     0x00000200u
#define OHCI_HC_PS_CSC                                      0x00010000u
#define OHCI_HC_PS_PESC                                     0x00020000u
#define OHCI_HC_PS_PSSC                                     0x00040000u
#define OHCI_HC_PS_OCIC                                     0x00080000u
#define OHCI_HC_PS_PRSC                                     0x00100000u


/* Define OHCI interrupt status register definitions.  */

#define OHCI_HC_INT_SO                                      0x00000001u
#define OHCI_HC_INT_WDH                                     0x00000002u
#define OHCI_HC_INT_SF                                      0x00000004u    
#define OHCI_HC_INT_RD                                      0x00000008u
#define OHCI_HC_INT_UE                                      0x00000010u
#define OHCI_HC_INT_FNO                                     0x00000020u
#define OHCI_HC_INT_RHSC                                    0x00000040u
#define OHCI_HC_INT_OC                                      0x40000000u

#define OHCI_HC_INT_MIE                                     0x80000000u


#define OHCI_HC_INTERRUPT_ENABLE_NORMAL                     (OHCI_HC_INT_WDH | OHCI_HC_INT_RD | OHCI_HC_INT_UE | OHCI_HC_INT_RHSC | OHCI_HC_INT_MIE)     

#define OHCI_HC_INTERRUPT_DISABLE_ALL                       (OHCI_HC_INT_SO   |  \
                                                            OHCI_HC_INT_WDH  |  \
                                                            OHCI_HC_INT_SF   |  \
                                                            OHCI_HC_INT_RD   |  \
                                                            OHCI_HC_INT_UE   |  \
                                                            OHCI_HC_INT_FNO  |  \
                                                            OHCI_HC_INT_RHSC |  \
                                                            OHCI_HC_INT_OC   |  \
                                                            OHCI_HC_INT_MIE)


/* Define OHCI frame interval definition. */

#define OHCI_HC_FM_INTERVAL_CLEAR                           0x8000ffffu
#define OHCI_HC_FM_INTERVAL_SET                             0x27780000u
#define OHCI_HC_FM_INTERVAL_FI_MASK                         0x00003fffu


/* Define OHCI static definition.  */

#define UX_OHCI_AVAILABLE_BANDWIDTH                         6000u
#define UX_OHCI_INIT_DELAY                                  1000
#define UX_OHCI_RESET_RETRY                                 1000
#define UX_OHCI_RESET_DELAY                                 10
#define UX_OHCI_PORT_RESET_RETRY                            10
#define UX_OHCI_PORT_RESET_DELAY                            10


/* Define OHCI initialization values.  */

#define UX_OHCI_COMMAND_STATUS_RESET                        0
#define UX_OHCI_INIT_RESET_DELAY                            10

/* Define OHCI completion code errors.  */

#define UX_OHCI_NO_ERROR                                    0x00
#define UX_OHCI_ERROR_CRC                                   0x01
#define UX_OHCI_ERROR_BIT_STUFFING                          0x02
#define UX_OHCI_ERROR_DATA_TOGGLE                           0x03
#define UX_OHCI_ERROR_STALL                                 0x04
#define UX_OHCI_ERROR_DEVICE_NOT_RESPONDING                 0x05
#define UX_OHCI_ERROR_PID_FAILURE                           0x06
#define UX_OHCI_ERROR_PID_UNEXPECTED                        0x07
#define UX_OHCI_ERROR_DATA_OVERRRUN                         0x08
#define UX_OHCI_ERROR_DATA_UNDERRUN                         0x09
#define UX_OHCI_ERROR_BUFFER_OVERRRUN                       0x0c
#define UX_OHCI_ERROR_BUFFER_UNDERRUN                       0x0d
#define UX_OHCI_NOT_ACCESSED                                0x0e

#define UX_OHCI_PRSC_EVENT                                  0x1u

#define UX_OHCI_PRSC_EVENT_TIMEOUT                          100

/* Define OHCI HCCA structure.  */

typedef struct UX_HCD_OHCI_HCCA_STRUCT
{

    struct UX_OHCI_ED_STRUCT
                    *ux_hcd_ohci_hcca_ed[32];
    USHORT          ux_hcd_ohci_hcca_frame_number;
    USHORT          ux_hcd_ohci_hcca_reserved1;
    struct UX_OHCI_TD_STRUCT               
                    *ux_hcd_ohci_hcca_done_head;
    UCHAR           ux_hcd_ohci_hcca_reserved2[116];
} UX_HCD_OHCI_HCCA;


/* Define OHCI HCD structure.  */

typedef struct UX_HCD_OHCI_STRUCT
{

    struct UX_HCD_STRUCT     
                    *ux_hcd_ohci_hcd_owner;
    struct UX_HCD_OHCI_HCCA_STRUCT     
                    *ux_hcd_ohci_hcca;
    ULONG           *ux_hcd_ohci_hcor;
    UINT            ux_hcd_ohci_nb_root_hubs;
    struct UX_OHCI_TD_STRUCT               
                    *ux_hcd_ohci_done_head;
    struct UX_OHCI_ED_STRUCT           
                    *ux_hcd_ohci_ed_list;
    struct UX_OHCI_TD_STRUCT           
                    *ux_hcd_ohci_td_list;
    struct UX_OHCI_ISO_TD_STRUCT       
                    *ux_hcd_ohci_iso_td_list;
    UX_EVENT_FLAGS_GROUP
                    ux_hcd_ohci_event_flags_group;
} UX_HCD_OHCI;


/* Define OHCI ED structure.  */

typedef struct UX_OHCI_ED_STRUCT
{

    ULONG           ux_ohci_ed_dw0;
    struct UX_OHCI_TD_STRUCT               
                    *ux_ohci_ed_tail_td;
    struct UX_OHCI_TD_STRUCT               
                    *ux_ohci_ed_head_td;
    struct UX_OHCI_ED_STRUCT               
                    *ux_ohci_ed_next_ed;
    struct UX_OHCI_ED_STRUCT               
                    *ux_ohci_ed_previous_ed;
    ULONG           ux_ohci_ed_status;
    struct UX_ENDPOINT_STRUCT          
                    *ux_ohci_ed_endpoint;
    ULONG           ux_ohci_ed_frame;
} UX_OHCI_ED;


/* Define OHCI ED bitmap.  */

#define UX_OHCI_ED_LOW_SPEED                                0x00002000u
#define UX_OHCI_ED_SKIP                                     0x00004000u
#define UX_OHCI_ED_ISOCHRONOUS                              0x00008000u
#define UX_OHCI_ED_MPS                                      0x0000ffffu

#define UX_OHCI_ED_HALTED                                   0x00000001u
#define UX_OHCI_ED_TOGGLE_CARRY                             0x00000002u
#define UX_OHCI_ED_MASK_TD                                  (~0x00000003u)

#define UX_OHCI_ED_OUT                                      0x0800u
#define UX_OHCI_ED_IN                                       0x1000u   


/* Define OHCI TD structure.  */

typedef struct UX_OHCI_TD_STRUCT
{                                                
    ULONG           ux_ohci_td_dw0;
    UCHAR *         ux_ohci_td_cbp;
    struct UX_OHCI_TD_STRUCT              
                    *ux_ohci_td_next_td;
    UCHAR *         ux_ohci_td_be;
    ULONG           ux_ohci_td_reserved_1[4];
    struct UX_TRANSFER_STRUCT          
                    *ux_ohci_td_transfer_request;
    struct UX_OHCI_TD_STRUCT              
                    *ux_ohci_td_next_td_transfer_request;
    struct UX_OHCI_ED_STRUCT              
                    *ux_ohci_td_ed;
    ULONG           ux_ohci_td_length;
    ULONG           ux_ohci_td_status;
    ULONG           ux_ohci_td_reserved_2[3];
} UX_OHCI_TD;


/* Define OHCI TD bitmap.  */

#define UX_OHCI_TD_OUT                                      0x00080000u
#define UX_OHCI_TD_IN                                       0x00100000u
#define UX_OHCI_TD_DEFAULT_DW0                              0xf0000000u
#define UX_OHCI_TD_DATA0                                    0x02000000u   
#define UX_OHCI_TD_DATA1                                    0x03000000u   
#define UX_OHCI_TD_R                                        0x00040000u   

#define UX_OHCI_TD_SETUP_PHASE                              0x00010000u  
#define UX_OHCI_TD_DATA_PHASE                               0x00020000u   
#define UX_OHCI_TD_STATUS_PHASE                             0x00040000u  
#define UX_OHCI_TD_CC                                       28u


/* Define OHCI ISOCHRONOUS TD structure.  */

typedef struct UX_OHCI_ISO_TD_STRUCT
{

    ULONG           ux_ohci_iso_td_dw0;
    UCHAR *         ux_ohci_iso_td_bp0;
    struct UX_OHCI_TD_STRUCT              
                    *ux_ohci_iso_td_next_td;
    UCHAR *         ux_ohci_iso_td_be;
    USHORT          ux_ohci_iso_td_offset_psw[8];
    struct UX_TRANSFER_STRUCT          
                    *ux_ohci_iso_td_transfer_request;
    struct UX_OHCI_TD_STRUCT              
                    *ux_ohci_iso_td_next_td_transfer_request;
    struct UX_OHCI_ED_STRUCT              
                    *ux_ohci_iso_td_ed;
    ULONG           ux_ohci_iso_td_length;
    ULONG           ux_ohci_iso_td_status;
    ULONG           ux_ohci_iso_td_reserved[3];
} UX_OHCI_ISO_TD;


/* Define OHCI ISOCHRONOUS TD bitmap.  */

#define UX_OHCI_ISO_TD_BASE                                 0xfffff000u
#define UX_OHCI_ISO_TD_OFFSET                               0x00000fffu
#define UX_OHCI_ISO_TD_PSW_CC                               0x0000e000u
#define UX_OHCI_ISO_TD_FC                                   24u


/* Define OHCI function prototypes.  */

UINT    _ux_hcd_ohci_asynchronous_endpoint_create(UX_HCD_OHCI *hcd_ohci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ohci_asynchronous_endpoint_destroy(UX_HCD_OHCI *hcd_ohci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ohci_controller_disable(UX_HCD_OHCI *hcd_ohci);
VOID    _ux_hcd_ohci_done_queue_process(UX_HCD_OHCI *hcd_ohci);
UX_OHCI_ED  *_ux_hcd_ohci_ed_obtain(UX_HCD_OHCI *hcd_ohci);
UINT    _ux_hcd_ohci_endpoint_error_clear(UX_HCD_OHCI *hcd_ohci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ohci_endpoint_reset(UX_HCD_OHCI *hcd_ohci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ohci_entry(UX_HCD *hcd, UINT function, VOID *parameter);
UINT    _ux_hcd_ohci_frame_number_get(UX_HCD_OHCI *hcd_ohci, ULONG *frame_number);
VOID    _ux_hcd_ohci_frame_number_set(UX_HCD_OHCI *hcd_ohci, ULONG frame_number);
UINT    _ux_hcd_ohci_initialize(UX_HCD *hcd);
UINT    _ux_hcd_ohci_interrupt_endpoint_create(UX_HCD_OHCI *hcd_ohci, UX_ENDPOINT *endpoint);
VOID    _ux_hcd_ohci_interrupt_handler(VOID);
UINT    _ux_hcd_ohci_isochronous_endpoint_create(UX_HCD_OHCI *hcd_ohci, UX_ENDPOINT *endpoint);
UX_OHCI_ISO_TD  *_ux_hcd_ohci_isochronous_td_obtain(UX_HCD_OHCI *hcd_ohci);
UX_OHCI_ED  *_ux_hcd_ohci_least_traffic_list_get(UX_HCD_OHCI *hcd_ohci);
VOID    _ux_hcd_ohci_next_td_clean(UX_OHCI_TD *td);
UINT    _ux_hcd_ohci_periodic_endpoint_destroy(UX_HCD_OHCI *hcd_ohci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ohci_periodic_tree_create(UX_HCD_OHCI *hcd_ohci);
UINT    _ux_hcd_ohci_port_disable(UX_HCD_OHCI *hcd_ohci, ULONG port_index);
UINT    _ux_hcd_ohci_port_enable(UX_HCD_OHCI *hcd_ohci, ULONG port_index);
UINT    _ux_hcd_ohci_port_reset(UX_HCD_OHCI *hcd_ohci, ULONG port_index);
UINT    _ux_hcd_ohci_port_resume(UX_HCD_OHCI *hcd_ohci, UINT port_index);
ULONG   _ux_hcd_ohci_port_status_get(UX_HCD_OHCI *hcd_ohci, ULONG port_index);
UINT    _ux_hcd_ohci_port_suspend(UX_HCD_OHCI *hcd_ohci, ULONG port_index);
UINT    _ux_hcd_ohci_power_down_port(UX_HCD_OHCI *hcd_ohci, ULONG port_index);
UINT    _ux_hcd_ohci_power_on_port(UX_HCD_OHCI *hcd_ohci, ULONG port_index);
VOID    _ux_hcd_ohci_power_root_hubs(UX_HCD_OHCI *hcd_ohci);
ULONG   _ux_hcd_ohci_register_read(UX_HCD_OHCI *hcd_ohci, ULONG ohci_register);
VOID    _ux_hcd_ohci_register_write(UX_HCD_OHCI *hcd_ohci, ULONG ohci_register, ULONG value);
UX_OHCI_TD  *_ux_hcd_ohci_regular_td_obtain(UX_HCD_OHCI *hcd_ohci);
UINT    _ux_hcd_ohci_request_bulk_transfer(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ohci_request_control_transfer(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ohci_request_interrupt_transfer(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ohci_request_isochronous_transfer(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ohci_request_transfer(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ohci_transfer_abort(UX_HCD_OHCI *hcd_ohci, UX_TRANSFER *transfer_request);
VOID    _ux_hcd_ohci_transfer_request_process(UX_TRANSFER *transfer_request);

#define ux_hcd_ohci_initialize                      _ux_hcd_ohci_initialize
#define ux_hcd_ohci_interrupt_handler               _ux_hcd_ohci_interrupt_handler

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif

