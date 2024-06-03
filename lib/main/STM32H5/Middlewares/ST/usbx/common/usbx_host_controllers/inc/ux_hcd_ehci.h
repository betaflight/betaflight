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
/**   EHCI Controller                                                     */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_hcd_ehci.h                                       PORTABLE C      */ 
/*                                                           6.1.8        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX host EHCI Controller.                                          */ 
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
/*  11-09-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            used unsigned defines,      */
/*                                            named unions and structs,   */
/*                                            resulting in version 6.1.2  */
/*  08-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            fixed spelling error,       */
/*                                            added extern "C" keyword    */
/*                                            for compatibility with C++, */
/*                                            resulting in version 6.1.8  */
/*                                                                        */
/**************************************************************************/

#ifndef UX_HCD_EHCI_H
#define UX_HCD_EHCI_H

/* Determine if a C++ compiler is being used.  If so, ensure that standard 
   C is used to process the API information.  */ 

#ifdef   __cplusplus 

/* Yes, C++ compiler is present.  Use standard C.  */ 
extern   "C" { 

#endif  


/* Possible defined EHCI HCD extensions.  */

/* Extension for peripheral host mode select (function like).  */
/* #define UX_HCD_EHCI_EXT_USB_HOST_MODE_ENABLE(hcd_ehci) */

/* Extension for embedded TT (UX_TRUE/UX_FALSE).  */
/* #define UX_HCD_EHCI_EXT_EMBEDDED_TT_SUPPORT */

/* Extension for phy high speed mode select (function like).  */
/* #define UX_HCD_EHCI_EXT_USBPHY_HIGHSPEED_MODE_SET(hcd_ehci, on_off) */

/* Define EHCI generic definitions.  */

#define UX_EHCI_CONTROLLER                                  2
#define UX_EHCI_MAX_PAYLOAD                                 16384
#define UX_EHCI_FRAME_DELAY                                 4 
#define UX_EHCI_PAGE_SIZE                                   4096
#define UX_EHCI_PAGE_ALIGN                                  0xfffff000


/* Define EHCI host controller capability registers.  */

#define EHCI_HCCR_CAP_LENGTH                                0x00
#define EHCI_HCCR_HCS_PARAMS                                0x01
#define EHCI_HCCR_HCC_PARAMS                                0x02
#define EHCI_HCCR_HCSP_PORT_ROUTE                           0x03


/* Define EHCI host controller registers.  */

#define EHCI_HCOR_USB_COMMAND                               (hcd_ehci -> ux_hcd_ehci_hcor + 0x00)
#define EHCI_HCOR_USB_STATUS                                (hcd_ehci -> ux_hcd_ehci_hcor + 0x01)
#define EHCI_HCOR_USB_INTERRUPT                             (hcd_ehci -> ux_hcd_ehci_hcor + 0x02)
#define EHCI_HCOR_FRAME_INDEX                               (hcd_ehci -> ux_hcd_ehci_hcor + 0x03)
#define EHCI_HCOR_FRAME_LIST_BASE_ADDRESS                   (hcd_ehci -> ux_hcd_ehci_hcor + 0x05)
#define EHCI_HCOR_ASYNCH_LIST_ADDRESS                       (hcd_ehci -> ux_hcd_ehci_hcor + 0x06)
#define EHCI_HCOR_CONFIG_FLAG                               (hcd_ehci -> ux_hcd_ehci_hcor + 0x10)
#define EHCI_HCOR_PORT_SC                                   (hcd_ehci -> ux_hcd_ehci_hcor + 0x11)


/* Define EHCI IO control register values.  */

#define EHCI_HC_IO_RS                                       0x00000001u
#define EHCI_HC_IO_HCRESET                                  0x00000002u
#define EHCI_HC_IO_PSE                                      0x00000010u
#define EHCI_HC_IO_ASE                                      0x00000020u
#define EHCI_HC_IO_IAAD                                     0x00000040u
#define EHCI_HC_IO_ITC                                      0x00010000u
#define EHCI_HC_IO_FRAME_SIZE_1024                          0x00000000u
#define EHCI_HC_IO_FRAME_SIZE_512                           0x00000004u
#define EHCI_HC_IO_FRAME_SIZE_256                           0x00000008u
#define EHCI_HC_IO_FRAME_SIZE_128                           0x0000000Cu
#define EHCI_HC_IO_FRAME_SIZE_64                            0x00008000u
#define EHCI_HC_IO_FRAME_SIZE_32                            0x00008004u

/* The number if entries in the periodic tree can be changed to save space IF and only IF the PFLF flag in the HCCPARAMS register
   allows it. Setting values less than 1024 in controllers without the ability to change the Frame List Size leads to a EHCI crash.  */
   
#ifndef UX_EHCI_FRAME_LIST_ENTRIES
#define UX_EHCI_FRAME_LIST_ENTRIES                          1024
#endif
#define UX_EHCI_FRAME_LIST_MASK                             EHCI_HC_IO_FRAME_SIZE_1024

/* Define EHCI HCOR status register.  */

#define EHCI_HC_STS_USB_INT                                 0x00000001u
#define EHCI_HC_STS_USB_ERR_INT                             0x00000002u
#define EHCI_HC_STS_PCD                                     0x00000004u
#define EHCI_HC_STS_FLR                                     0x00000008u
#define EHCI_HC_STS_HSE                                     0x00000010u
#define EHCI_HC_STS_IAA                                     0x00000020u
#define EHCI_HC_STS_HC_HALTED                               0x00001000u
#define EHCI_HC_STS_RECLAMATION                             0x00002000u
#define EHCI_HC_STS_PSS                                     0x00004000u
#define EHCI_HC_STS_ASS                                     0x00008000u

#define EHCI_HC_INTERRUPT_ENABLE_NORMAL                     (EHCI_HC_STS_USB_INT|EHCI_HC_STS_USB_ERR_INT|EHCI_HC_STS_PCD|EHCI_HC_STS_HSE|EHCI_HC_STS_IAA)


/* Define EHCI HCOR root HUB command/status.  */

#define EHCI_HC_RH_PPC                                      0x00000010u
#define EHCI_HC_RH_PSM                                      0x00000100u
#define EHCI_HC_RH_NPS                                      0x00000200u
#define EHCI_HC_RH_DT                                       0x00000400u
#define EHCI_HC_RH_OCPM                                     0x00000800u
#define EHCI_HC_RH_NOCP                                     0x00001000u

#define EHCI_HC_PS_CCS                                      0x00000001u
#define EHCI_HC_PS_CSC                                      0x00000002u
#define EHCI_HC_PS_PE                                       0x00000004u
#define EHCI_HC_PS_PEC                                      0x00000008u
#define EHCI_HC_PS_OCA                                      0x00000010u
#define EHCI_HC_PS_OCC                                      0x00000020u
#define EHCI_HC_PS_FPR                                      0x00000040u
#define EHCI_HC_PS_SUSPEND                                  0x00000080u
#define EHCI_HC_PS_PR                                       0x00000100u
#define EHCI_HC_PS_PP                                       0x00001000u
#define EHCI_HC_PS_SPEED_MASK                               0x00000c00u
#define EHCI_HC_PS_SPEED_LOW                                0x00000400u
#define EHCI_HC_PS_PO                                       0x00002000u
#define EHCI_HC_PS_EMBEDDED_TT_SPEED_MASK                   0x0c000000u
#define EHCI_HC_PS_EMBEDDED_TT_SPEED_FULL                   0x00000000u
#define EHCI_HC_PS_EMBEDDED_TT_SPEED_LOW                    0x04000000u
#define EHCI_HC_PS_EMBEDDED_TT_SPEED_HIGH                   0x08000000u

#define EHCI_HC_RH_POWER_STABLE_DELAY                       25
#define EHCI_HC_RH_RESET_DELAY                              50
#define EHCI_HC_RH_RESET_SETTLE_DELAY                       5


/* Define EHCI interrupt status register definitions.  */

#define EHCI_HC_INT_IE                                      0x00000001u
#define EHCI_HC_INT_EIE                                     0x00000002u
#define EHCI_HC_INT_PCIE                                    0x00000004u
#define EHCI_HC_INT_FLRE                                    0x00000008u
#define EHCI_HC_INT_HSER                                    0x00000010u
#define EHCI_HC_INT_IAAE                                    0x00000020u


/* Define EHCI frame interval definition.  */

#define EHCI_HC_FM_INTERVAL_CLEAR                           0x8000ffff
#define EHCI_HC_FM_INTERVAL_SET                             0x27780000


/* Define EHCI static definition.  */

#define UX_EHCI_AVAILABLE_BANDWIDTH                         6000
#define UX_EHCI_STOP                                        0
#define UX_EHCI_START                                       1
#define UX_EHCI_ROUTE_TO_LOCAL_HC                           1
#define UX_EHCI_INIT_DELAY                                  1000
#define UX_EHCI_RESET_RETRY                                 1000
#define UX_EHCI_RESET_DELAY                                 100
#define UX_EHCI_PORT_RESET_RETRY                            10
#define UX_EHCI_PORT_RESET_DELAY                            50


/* Define EHCI initialization values.  */

#define UX_EHCI_COMMAND_STATUS_RESET                        0
#define UX_EHCI_INIT_RESET_DELAY                            10


/* Define EHCI completion code errors.  */

#define UX_EHCI_NO_ERROR                                    0x00
#define UX_EHCI_ERROR_CRC                                   0x01
#define UX_EHCI_ERROR_BIT_STUFFING                          0x02
#define UX_EHCI_ERROR_DATA_TOGGLE                           0x03
#define UX_EHCI_ERROR_STALL                                 0x04
#define UX_EHCI_ERROR_DEVICE_NOT_RESPONDING                 0x05
#define UX_EHCI_ERROR_PID_FAILURE                           0x06
#define UX_EHCI_ERROR_DATA_OVERRUN                          0x08
#define UX_EHCI_ERROR_DATA_UNDERRUN                         0x09
#define UX_EHCI_ERROR_BUFFER_OVERRUN                        0x0c
#define UX_EHCI_ERROR_BUFFER_UNDERRUN                       0x0d
#define UX_EHCI_ERROR_NOT_ACCESSED                          0x0f
#define UX_EHCI_ERROR_NAK                                   0x10
#define UX_EHCI_ERROR_BABBLE                                0x11

/* EHCI general descriptor type (Link Pointer).  */

#define UX_EHCI_LP_MASK                                    (0xFFFFFFE0u) /* 32-byte align.  */

#define UX_EHCI_TYP_MASK                                   (0x3u<<1)
#define UX_EHCI_TYP_ITD                                    (0x0u<<1)
#define UX_EHCI_TYP_QH                                     (0x1u<<1)
#define UX_EHCI_TYP_SITD                                   (0x2u<<1)
#define UX_EHCI_TYP_FSTN                                   (0x3u<<1)

#define UX_EHCI_T                                          (0x1u<<0)

/* EHCI general descriptor type (Capabilities).  */

#define UX_EHCI_ENDPT_MASK                                 (0xFu<<8)
#define UX_EHCI_ENDPT_SHIFT                                (8)

#define UX_EHCI_DEVICE_ADDRESS_MASK                        (0x3Fu<<0)

#define UX_EHCI_CMASK_MASK                                 (0xFFu<<8)
#define UX_EHCI_CMASK_0                                    (0x01u<<8)
#define UX_EHCI_CMASK_1                                    (0x02u<<8)
#define UX_EHCI_CMASK_2                                    (0x04u<<8)
#define UX_EHCI_CMASK_3                                    (0x08u<<8)
#define UX_EHCI_CMASK_4                                    (0x10u<<8)
#define UX_EHCI_CMASK_5                                    (0x20u<<8)
#define UX_EHCI_CMASK_6                                    (0x40u<<8)
#define UX_EHCI_CMASK_7                                    (0x80u<<8)
#define UX_EHCI_CMASK_ISOOUT_ANY                           (0x00u<<8)
#define UX_EHCI_CMASK_INT_Y0                               (0x1Cu<<8)
#define UX_EHCI_CMASK_INT_Y1                               (0x31u<<8)
#define UX_EHCI_CMASK_INT_Y2                               (0x70u<<8)
#define UX_EHCI_CMASK_INT_Y3                               (0xE0u<<8)
#define UX_EHCI_CMASK_INT_Y4                               (0xC1u<<8)
#define UX_EHCI_CMASK_INT_Y5                               (0x81u<<8)
#define UX_EHCI_CMASK_INT_Y7                               (0x07u<<8)
#define UX_EHCI_CMASK_ISOIN_C1                             (0x01u<<8)
#define UX_EHCI_CMASK_ISOIN_C2                             (0x03u<<8)
#define UX_EHCI_CMASK_ISOIN_C3                             (0x07u<<8)
#define UX_EHCI_CMASK_ISOIN_C4                             (0x0Fu<<8)
#define UX_EHCI_CMASK_ISOIN_C5                             (0x1Fu<<8)
#define UX_EHCI_CMASK_ISOIN_C6                             (0x3Fu<<8)

#define UX_EHCI_SMASK_MASK                                 (0xFFu<<0)
#define UX_EHCI_SMASK_0                                    (0x01u<<0)
#define UX_EHCI_SMASK_1                                    (0x02u<<0)
#define UX_EHCI_SMASK_2                                    (0x04u<<0)
#define UX_EHCI_SMASK_3                                    (0x08u<<0)
#define UX_EHCI_SMASK_4                                    (0x10u<<0)
#define UX_EHCI_SMASK_5                                    (0x20u<<0)
#define UX_EHCI_SMASK_6                                    (0x40u<<0)
#define UX_EHCI_SMASK_7                                    (0x80u<<0)
#define UX_EHCI_SMASK_INTERVAL_1                           (0xFFu<<0)
#define UX_EHCI_SMASK_INTERVAL_2                           (0x55u<<0)
#define UX_EHCI_SMASK_INTERVAL_3                           (0x11u<<0)
#define UX_EHCI_SMASK_INTERVAL_4                           (0x01u<<0)


/* EHCI general descriptor type (Buffer pointer page part).  */

#define UX_EHCI_BP_MASK                                    (0xFFFFF000u) /* 4K align.  */

/* Define EHCI pointers.  */

typedef union UX_EHCI_POINTER_UNION {
    ULONG                               value;
    VOID                                *void_ptr;
    UCHAR                               *u8_ptr;
    USHORT                              *u16_ptr;
    ULONG                               *u32_ptr;
} UX_EHCI_POINTER;

typedef union UX_EHCI_LINK_POINTER_UNION {
    ULONG                               value;
    VOID                                *void_ptr;
    UCHAR                               *u8_ptr;
    USHORT                              *u16_ptr;
    ULONG                               *u32_ptr;
    struct UX_EHCI_ED_STRUCT            *qh_ptr;
    struct UX_EHCI_ED_STRUCT            *ed_ptr;
    struct UX_EHCI_TD_STRUCT            *td_ptr;
    struct UX_EHCI_HSISO_TD_STRUCT      *itd_ptr;
    struct UX_EHCI_FSISO_TD_STRUCT      *sitd_ptr;
} UX_EHCI_LINK_POINTER;

typedef union UX_EHCI_PERIODIC_LINK_POINTER_UNION {
    ULONG                               value;
    VOID                                *void_ptr;
    UCHAR                               *u8_ptr;
    USHORT                              *u16_ptr;
    ULONG                               *u32_ptr;
    struct UX_EHCI_ED_STRUCT            *qh_ptr;
    struct UX_EHCI_ED_STRUCT            *ed_ptr;
    struct UX_EHCI_HSISO_TD_STRUCT      *itd_ptr;
    struct UX_EHCI_FSISO_TD_STRUCT      *sitd_ptr;
} UX_EHCI_PERIODIC_LINK_POINTER;


/* Define the EHCI structure.  */

typedef struct UX_HCD_EHCI_STRUCT
{                                      

    struct UX_HCD_STRUCT
                    *ux_hcd_ehci_hcd_owner;
    ULONG           ux_hcd_ehci_hcor;
    struct UX_EHCI_ED_STRUCT              
                    **ux_hcd_ehci_frame_list;
    ULONG           *ux_hcd_ehci_base;
    UINT            ux_hcd_ehci_nb_root_hubs;
    struct UX_EHCI_TD_STRUCT              
                    *ux_hcd_ehci_done_head;
    struct UX_EHCI_ED_STRUCT              
                    *ux_hcd_ehci_ed_list;
    struct UX_EHCI_TD_STRUCT              
                    *ux_hcd_ehci_td_list;
    struct UX_EHCI_FSISO_TD_STRUCT        
                    *ux_hcd_ehci_fsiso_td_list;
    struct UX_EHCI_HSISO_TD_STRUCT        
                    *ux_hcd_ehci_hsiso_td_list;
    struct UX_EHCI_ED_STRUCT              
                    *ux_hcd_ehci_asynch_head_list;
    struct UX_EHCI_ED_STRUCT              
                    *ux_hcd_ehci_asynch_first_list;
    struct UX_EHCI_ED_STRUCT              
                    *ux_hcd_ehci_asynch_last_list;
    struct UX_EHCI_HSISO_TD_STRUCT
                    *ux_hcd_ehci_hsiso_scan_list;
    struct UX_EHCI_FSISO_TD_STRUCT
                    *ux_hcd_ehci_fsiso_scan_list;
    struct UX_TRANSFER_STRUCT
                    *ux_hcd_ehci_iso_done_transfer_head;
    struct UX_TRANSFER_STRUCT
                    *ux_hcd_ehci_iso_done_transfer_tail;
    struct UX_EHCI_ED_STRUCT
                    *ux_hcd_ehci_interrupt_ed_list;
    UX_MUTEX        ux_hcd_ehci_periodic_mutex;
    UX_SEMAPHORE    ux_hcd_ehci_protect_semaphore;
    UX_SEMAPHORE    ux_hcd_ehci_doorbell_semaphore;
    ULONG           ux_hcd_ehci_frame_list_size;
    ULONG           ux_hcd_ehci_interrupt_count;
    ULONG           ux_hcd_ehci_embedded_tt;
} UX_HCD_EHCI;


/* Define EHCI ED structure.  */ 

typedef struct UX_EHCI_ED_STRUCT
{                                      

    struct UX_EHCI_ED_STRUCT              
                    *ux_ehci_ed_queue_head;
    ULONG           ux_ehci_ed_cap0;
    ULONG           ux_ehci_ed_cap1;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_ed_current_td;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_ed_queue_element;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_ed_alternate_td;
    ULONG           ux_ehci_ed_state;
    VOID            *ux_ehci_ed_bp0; 
    VOID            *ux_ehci_ed_bp1; 
    VOID            *ux_ehci_ed_bp2; 
    VOID            *ux_ehci_ed_bp3; 
    VOID            *ux_ehci_ed_bp4;
    /* 12 DWords, 48 bytes QH for controller end.  */

    ULONG           ux_ehci_ed_status;
    struct UX_EHCI_ED_STRUCT
                    *ux_ehci_ed_next_ed;
    struct UX_EHCI_ED_STRUCT
                    *ux_ehci_ed_previous_ed;
    struct UX_EHCI_TD_STRUCT
                    *ux_ehci_ed_first_td;
    struct UX_EHCI_TD_STRUCT
                    *ux_ehci_ed_last_td;
    union {
        struct {                                            /* For anchor.  */
            struct UX_EHCI_ED_STRUCT
                    *ux_ehci_ed_next_anchor;                /* + 1 DWord.   */
            USHORT  ux_ehci_ed_microframe_load[8];          /* + 4 DWords.  */
            UCHAR   ux_ehci_ed_microframe_ssplit_count[8];  /* + 2 DWords.  */
        } ANCHOR;
        struct {                                            /* As interrupt ED.  */
            struct UX_EHCI_ED_STRUCT
                        *ux_ehci_ed_anchor;                 /* + 1 DWord.  */
            struct UX_ENDPOINT_STRUCT
                        *ux_ehci_ed_endpoint;               /* + 1 Dword.  */
        } INTR;
        struct {                                            /* Space: 7 DWord.  */
            ULONG       ux_ehci_ed_reserved[7];
        } RESERVED;
    } REF_AS;
    /* 24 DWord aligned.  */
} UX_EHCI_ED;


/* Define EHCI ED bitmap.  */

#define UX_EHCI_QH_TYP_ITD                                  0u
#define UX_EHCI_QH_TYP_QH                                   2u
#define UX_EHCI_QH_TYP_SITD                                 4u
#define UX_EHCI_QH_TYP_FSTN                                 6u

#define UX_EHCI_QH_T                                        1u

#define UX_EHCI_QH_STATIC                                   0x80000000u
#define UX_EHCI_QH_SSPLIT_SCH_FULL_7                        0x40000000u
#define UX_EHCI_QH_SSPLIT_SCH_FULL_6                        0x20000000u
#define UX_EHCI_QH_SSPLIT_SCH_FULL_5                        0x10000000u
#define UX_EHCI_QH_SSPLIT_SCH_FULL_4                        0x08000000u
#define UX_EHCI_QH_SSPLIT_SCH_FULL_3                        0x04000000u
#define UX_EHCI_QH_SSPLIT_SCH_FULL_2                        0x02000000u
#define UX_EHCI_QH_SSPLIT_SCH_FULL_1                        0x01000000u
#define UX_EHCI_QH_SSPLIT_SCH_FULL_0                        0x00800000u

#define UX_EHCI_QH_MPS_LOC                                  16u
#define UX_EHCI_QH_MPS_MASK                                 0x07ff0000u
#define UX_EHCI_QH_NCR                                      0xf0000000u
#define UX_EHCI_QH_CEF                                      0x08000000u
#define UX_EHCI_QH_ED_AD_LOC                                8u 
#define UX_EHCI_QH_HBPM                                     0x40000000u
#define UX_EHCI_QH_HBPM_LOC                                 30u
#define UX_EHCI_QH_HEAD                                     0x00008000u

#define UX_EHCI_QH_HIGH_SPEED                               0x00002000u
#define UX_EHCI_QH_LOW_SPEED                                0x00001000u

#define UX_EHCI_QH_HUB_ADDR_LOC                             16u
#define UX_EHCI_QH_PORT_NUMBER_LOC                          23u
#define UX_EHCI_QH_MULT_LOC                                 30u
#define UX_EHCI_QH_MULT_MASK                                0xc0000000u
#define UX_EHCI_QH_C_MASK                                   0x00001c00u
#define UX_EHCI_QH_IS_MASK                                  0x00000001u

#define UX_EHCI_QH_SMASK_MASK                               0x000000FFu
#define UX_EHCI_QH_SMASK_0                                  0x00000001u
#define UX_EHCI_QH_SMASK_1                                  0x00000002u
#define UX_EHCI_QH_SMASK_2                                  0x00000004u
#define UX_EHCI_QH_SMASK_3                                  0x00000008u
#define UX_EHCI_QH_SMASK_4                                  0x00000010u
#define UX_EHCI_QH_SMASK_5                                  0x00000020u
#define UX_EHCI_QH_SMASK_6                                  0x00000040u
#define UX_EHCI_QH_SMASK_7                                  0x00000080u

#define UX_EHCI_QH_DTC                                      0x00004000u
#define UX_EHCI_QH_TOGGLE                                   0x80000000u
#define UX_EHCI_LINK_ADDRESS_MASK                           0xfffffff0u
#define UX_EHCI_TOGGLE_0                                    0u
#define UX_EHCI_TOGGLE_1                                    0x80000000u

/* Define EHCI TD structure.  */

typedef struct UX_EHCI_TD_STRUCT
{                                      

    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_td_link_pointer;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_td_alternate_link_pointer;
    ULONG           ux_ehci_td_control;
    VOID            *ux_ehci_td_bp0; 
    VOID            *ux_ehci_td_bp1; 
    VOID            *ux_ehci_td_bp2; 
    VOID            *ux_ehci_td_bp3; 
    VOID            *ux_ehci_td_bp4; 
    /* 8-DWords, 32-bytes qTD for controller.  */
    struct UX_TRANSFER_STRUCT             
                    *ux_ehci_td_transfer_request;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_td_next_td_transfer_request;
    struct UX_EHCI_ED_STRUCT              
                    *ux_ehci_td_ed;
    ULONG           ux_ehci_td_length;
    ULONG           ux_ehci_td_status;
    ULONG           ux_ehci_td_phase;
    ULONG           ux_ehci_td_reserved_2[2];
    /* 16-DWord aligned.  */
} UX_EHCI_TD;


/* Define EHCI TD bitmap.  */

#define UX_EHCI_TD_T                                        1u
#define UX_EHCI_TD_LG_LOC                                   16u
#define UX_EHCI_TD_LG_MASK                                  0x7fffu
#define UX_EHCI_TD_IOC                                      0x00008000u
#define UX_EHCI_TD_CERR                                     0x00000c00u

#define UX_EHCI_TD_PING                                     1u
#define UX_EHCI_TD_DO_COMPLETE_SPLIT                        2u
#define UX_EHCI_TD_MISSED_MICRO_FRAMES                      4u
#define UX_EHCI_TD_TRANSACTION_ERROR                        8u
#define UX_EHCI_TD_BABBLE_DETECTED                          0x10u
#define UX_EHCI_TD_DATA_BUFFER_ERROR                        0x20u
#define UX_EHCI_TD_HALTED                                   0x40u
#define UX_EHCI_TD_ACTIVE                                   0x80u

#define UX_EHCI_PID_OUT                                     0x00000000u
#define UX_EHCI_PID_IN                                      0x00000100u
#define UX_EHCI_PID_SETUP                                   0x00000200u
#define UX_EHCI_PID_MASK                                    0x00000300u

#define  UX_EHCI_TD_SETUP_PHASE                             0x00010000u 
#define  UX_EHCI_TD_DATA_PHASE                              0x00020000u 
#define  UX_EHCI_TD_STATUS_PHASE                            0x00040000u 

/* Define EHCI ISOCHRONOUS TD extension structure.  */

typedef struct UX_EHCI_HSISO_ED_STRUCT
{
    struct UX_ENDPOINT_STRUCT
                    *ux_ehci_hsiso_ed_endpoint;
    struct UX_EHCI_ED_STRUCT
                    *ux_ehci_hsiso_ed_anchor;
    struct UX_TRANSFER_STRUCT
                    *ux_ehci_hsiso_ed_transfer_head;
    struct UX_TRANSFER_STRUCT
                    *ux_ehci_hsiso_ed_transfer_tail;
    struct UX_TRANSFER_STRUCT
                    *ux_ehci_hsiso_ed_transfer_first_new;
    struct UX_EHCI_HSISO_TD_STRUCT
                    *ux_ehci_hsiso_ed_fr_td[4];
    UCHAR           ux_ehci_hsiso_ed_frindex;           /* 1st usable micro-frame.  */
    UCHAR           ux_ehci_hsiso_ed_frinterval;        /* Micro-frame interval.  */
    UCHAR           ux_ehci_hsiso_ed_frinterval_shift;  /* Shift for micro-frame interval.  */
    UCHAR           ux_ehci_hsiso_ed_nb_tds;
    USHORT          ux_ehci_hsiso_ed_frstart;           /* Start micro-frame.  */
    USHORT          ux_ehci_hsiso_ed_frload;
    USHORT          ux_ehci_hsiso_ed_fr_hc;             /* Micro-frame HC process count.  */
    USHORT          ux_ehci_hsiso_ed_fr_sw;             /* Micro-frame SW load count.  */
} UX_EHCI_HSISO_ED;

/* Define EHCI ISOCHRONOUS TD structure.  */

typedef struct UX_EHCI_HSISO_TD_STRUCT
{

    union UX_EHCI_PERIODIC_LINK_POINTER_UNION
                    ux_ehci_hsiso_td_next_lp;
    ULONG           ux_ehci_hsiso_td_control[8];
    VOID            *ux_ehci_hsiso_td_bp[7]; 
    /* 16 DWords, 64-bytes iTD for controller end.  */
    UCHAR           ux_ehci_hsiso_td_status;
    UCHAR           ux_ehci_hsiso_td_frload;            /* REQ load map.  */
    USHORT          ux_ehci_hsiso_td_max_trans_size;
    union UX_EHCI_PERIODIC_LINK_POINTER_UNION
                    ux_ehci_hsiso_td_previous_lp;
    struct UX_EHCI_HSISO_TD_STRUCT
                    *ux_ehci_hsiso_td_next_scan_td; 
    struct UX_EHCI_HSISO_TD_STRUCT
                    *ux_ehci_hsiso_td_previous_scan_td; 
    struct UX_TRANSFER_STRUCT
                    *ux_ehci_hsiso_td_fr_transfer[3];
    struct UX_EHCI_HSISO_ED_STRUCT
                    *ux_ehci_hsiso_td_ed;
    /* 24 DWord aligned.  */
} UX_EHCI_HSISO_TD;


/* Next Link Pointer(LP).  */

#define UX_EHCI_HSISO_LP_MASK                                  UX_EHCI_LP_MASK

#define UX_EHCI_HSISO_TYP_MASK                                 UX_EHCI_TYP_MASK
#define UX_EHCI_HSISO_TYP_ITD                                  UX_EHCI_TYP_ITD
#define UX_EHCI_HSISO_TYP_QH                                   UX_EHCI_TYP_QH
#define UX_EHCI_HSISO_TYP_SITD                                 UX_EHCI_TYP_SITD
#define UX_EHCI_HSISO_TYP_FSTN                                 UX_EHCI_TYP_FSTN

#define UX_EHCI_HSISO_T                                        UX_EHCI_T

/* Transaction Status and Control.  */

#define UX_EHCI_HSISO_STATUS_MASK                              0xF0000000u
#define UX_EHCI_HSISO_STATUS_ACTIVE                            0x80000000u
#define UX_EHCI_HSISO_STATUS_DATA_BUFFER_ERR                   0x40000000u
#define UX_EHCI_HSISO_STATUS_BABBLE_DETECTED                   0x20000000u
#define UX_EHCI_HSISO_STATUS_XACT_ERR                          0x10000000u

#define UX_EHCI_HSISO_XACT_LENGTH_MASK                         0x0FFF0000u
#define UX_EHCI_HSISO_XACT_LENGTH_SHIFT                        16
#define UX_EHCI_HSISO_XACT_LENGTH_VALUE_MAX                    0xC00

#define UX_EHCI_HSISO_IOC                                      0x00008000u
#define UX_EHCI_HSISO_IOC_SHIFT                                15

#define UX_EHCI_HSISO_PG_MASK                                  0x00007000u
#define UX_EHCI_HSISO_PG_SHIFT                                 12

#define UX_EHCI_HSISO_XACT_OFFSET_MASK                         0x00000FFFu

/* Buffer Page Pointer List.  */

#define UX_EHCI_HSISO_BP_MASK                                  UX_EHCI_BP_MASK

/* BP0  */

#define UX_EHCI_HSISO_ENDPT_MASK                               UX_EHCI_ENDPT_MASK
#define UX_EHCI_HSISO_ENDPT_SHIFT                              UX_EHCI_ENDPT_SHIFT

#define UX_EHCI_HSISO_DEVICE_ADDRESS_MASK                      UX_EHCI_DEVICE_ADDRESS_MASK

/* BP1  */

#define UX_EHCI_HSISO_DIRECTION                                (0x1u << 11)
#define UX_EHCI_HSISO_DIRECTION_IN                             UX_EHCI_HSISO_DIRECTION
#define UX_EHCI_HSISO_DIRECTION_OUT                            0

#define UX_EHCI_HSISO_MAX_PACKET_SIZE_MASK                     0x000007FFu
#define UX_EHCI_HSISO_MAX_PACKET_SIZE_MAX                      0x00000400

/* BP2  */

#define UX_EHCI_HSISO_MULTI_MASK                               0x00000003u
#define UX_EHCI_HSISO_MULTI_ONE                                1
#define UX_EHCI_HSISO_MULTI_TWO                                2
#define UX_EHCI_HSISO_MULTI_THREE                              3

/* Define EHCI FS ISOCHRONOUS TD structure.  */

typedef struct UX_EHCI_FSISO_TD_STRUCT
{
    union UX_EHCI_PERIODIC_LINK_POINTER_UNION
                    ux_ehci_fsiso_td_next_lp;
    ULONG           ux_ehci_fsiso_td_cap0;  /* endpoint */
    ULONG           ux_ehci_fsiso_td_cap1;  /* uFrame schedule */
    ULONG           ux_ehci_fsiso_td_state;
    VOID            *ux_ehci_fsiso_td_bp[2];
    VOID            *ux_ehci_fsiso_td_back_pointer;
    /* 7 DWords, 28-bytes siTD for controller end.  */

    UCHAR           ux_ehci_fsiso_td_status;
    UCHAR           ux_ehci_fsiso_td_frindex;
    UCHAR           ux_ehci_fsiso_td_nb_ed_tds;
    UCHAR           reserved[1];
    struct UX_ENDPOINT_STRUCT
                    *ux_ehci_fsiso_td_endpoint;
    struct UX_TRANSFER_STRUCT             
                    *ux_ehci_fsiso_td_transfer_head;
    struct UX_TRANSFER_STRUCT             
                    *ux_ehci_fsiso_td_transfer_tail;
    union UX_EHCI_PERIODIC_LINK_POINTER_UNION
                    ux_ehci_fsiso_td_previous_lp;
    struct UX_EHCI_FSISO_TD_STRUCT
                    *ux_ehci_fsiso_td_next_scan_td;
    struct UX_EHCI_FSISO_TD_STRUCT
                    *ux_ehci_fsiso_td_previous_scan_td;
    struct UX_EHCI_ED_STRUCT
                    *ux_ehci_fsiso_td_anchor;
    struct UX_EHCI_TD_STRUCT
                    *ux_ehci_fsiso_td_next_ed_td;
    /* 16-DWord aligned.  */
} UX_EHCI_FSISO_TD;

/* Next Link Pointer (LP).  */

#define UX_EHCI_FSISO_LP_MASK                              UX_EHCI_LP_MASK

#define UX_EHCI_FSISO_TYP_MASK                             UX_EHCI_TYP_MASK
#define UX_EHCI_FSISO_TYP_ITD                              UX_EHCI_TYP_ITD
#define UX_EHCI_FSISO_TYP_QH                               UX_EHCI_TYP_QH
#define UX_EHCI_FSISO_TYP_SITD                             UX_EHCI_TYP_SITD
#define UX_EHCI_FSISO_TYP_FSTN                             UX_EHCI_TYP_FSTN

#define UX_EHCI_FSISO_T                                    UX_EHCI_T

/* Endpoint Capabilities/Characteristics.  */

#define UX_EHCI_FSISO_DIRECTION                            0x80000000u
#define UX_EHCI_FSISO_DIRECTION_IN                         0x80000000u
#define UX_EHCI_FSISO_DIRECTION_OUT                        0x00000000

#define UX_EHCI_FSISO_PORT_NUMBER_MASK                     0x7F000000u
#define UX_EHCI_FSISO_PORT_NUMBER_SHIFT                    24

#define UX_EHCI_FSISO_HUB_ADDRESS_MASK                     0x003F0000u
#define UX_EHCI_FSISO_HUB_ADDRESS_SHIFT                    16

#define UX_EHCI_FSISO_ENDPT_MASK                           UX_EHCI_ENDPT_MASK
#define UX_EHCI_FSISO_ENDPT_SHIFT                          UX_EHCI_ENDPT_SHIFT

#define UX_EHCI_FSISO_DEVICE_ADDRESS_MASK                  UX_EHCI_DEVICE_ADDRESS_MASK

/* Micro-frame Schedule Control.  */

#define UX_EHCI_FSISO_UFRAME_CMASK_MASK                    UX_EHCI_CMASK_MASK
#define UX_EHCI_FSISO_UFRAME_CMASK_0                       UX_EHCI_CMASK_0
#define UX_EHCI_FSISO_UFRAME_CMASK_1                       UX_EHCI_CMASK_1
#define UX_EHCI_FSISO_UFRAME_CMASK_2                       UX_EHCI_CMASK_2
#define UX_EHCI_FSISO_UFRAME_CMASK_3                       UX_EHCI_CMASK_3
#define UX_EHCI_FSISO_UFRAME_CMASK_4                       UX_EHCI_CMASK_4
#define UX_EHCI_FSISO_UFRAME_CMASK_5                       UX_EHCI_CMASK_5
#define UX_EHCI_FSISO_UFRAME_CMASK_6                       UX_EHCI_CMASK_6
#define UX_EHCI_FSISO_UFRAME_CMASK_7                       UX_EHCI_CMASK_7

#define UX_EHCI_FSISO_UFRAME_SMASK_MASK                    UX_EHCI_SMASK_MASK
#define UX_EHCI_FSISO_UFRAME_SMASK_0                       UX_EHCI_SMASK_0
#define UX_EHCI_FSISO_UFRAME_SMASK_1                       UX_EHCI_SMASK_1
#define UX_EHCI_FSISO_UFRAME_SMASK_2                       UX_EHCI_SMASK_2
#define UX_EHCI_FSISO_UFRAME_SMASK_3                       UX_EHCI_SMASK_3
#define UX_EHCI_FSISO_UFRAME_SMASK_4                       UX_EHCI_SMASK_4
#define UX_EHCI_FSISO_UFRAME_SMASK_5                       UX_EHCI_SMASK_5
#define UX_EHCI_FSISO_UFRAME_SMASK_6                       UX_EHCI_SMASK_6
#define UX_EHCI_FSISO_UFRAME_SMASK_7                       UX_EHCI_SMASK_7

/* Transfer State.  */

/* Transfer Status and Control.  */

#define UX_EHCI_FSISO_IOC                                  0x80000000u

#define UX_EHCI_FSISO_P                                    0x40000000u

#define UX_EHCI_FSISO_TOTAL_BYTES_MASK                     0x03FF0000u
#define UX_EHCI_FSISO_TOTAL_BYTES_SHIFT                    16
#define UX_EHCI_FSISO_TOTAL_BYTES_MAX_VALUE                1023

#define UX_EHCI_FSISO_CPROMASK_MASK                        0x0000FF00u
#define UX_EHCI_FSISO_CPROMASK_SHIFT                       8

#define UX_EHCI_FSISO_STATUS_MASK                          0x000000FFu
#define UX_EHCI_FSISO_STATUS_ACTIVE                        0x00000080u
#define UX_EHCI_FSISO_STATUS_ERR                           0x00000040u
#define UX_EHCI_FSISO_STATUS_DATA_BUFFER_ERR               0x00000020u
#define UX_EHCI_FSISO_STATUS_BABBLE_DETECTED               0x00000010u
#define UX_EHCI_FSISO_STATUS_XACTERR                       0x00000008u
#define UX_EHCI_FSISO_STATUS_MISSED_MFRAME                 0x00000004u

#define UX_EHCI_FSISO_STATUS_SPLIT_STATE_MASK              0x00000002u
#define UX_EHCI_FSISO_STATUS_SPLIT_STATE_DO_START          0x00000000u
#define UX_EHCI_FSISO_STATUS_SPLIT_STATE_DO_COMPLETE       0x00000002u

/* Buffer Page Pointer List.  */

#define UX_EHCI_FSISO_BP_MASK                              UX_EHCI_BP_MASK

/* BP0  */

#define UX_EHCI_FSISO_CURRENT_OFFSET_MASK                  0x00000FFFu

/* BP1  */

#define UX_EHCI_FSISO_TP_MASK                              0x0000000Cu
#define UX_EHCI_FSISO_TP_ALL                               0x00000000u
#define UX_EHCI_FSISO_TP_BEGIN                             0x00000004u
#define UX_EHCI_FSISO_TP_MID                               0x00000008u
#define UX_EHCI_FSISO_TP_END                               0x0000000Cu

#define UX_EHCI_FSISO_TCOUNT_MASK                          0x00000007u
#define UX_EHCI_FSISO_TCOUNT_MAX                           6


/* Define EHCI function prototypes.  */

void _ux_hcd_ehci_periodic_descriptor_link(VOID* prev, VOID* prev_next, VOID* next_prev, VOID* next);
UX_EHCI_TD          *_ux_hcd_ehci_asynch_td_process(UX_EHCI_ED *ed, UX_EHCI_TD *td);
UX_EHCI_HSISO_TD    *_ux_hcd_ehci_hsisochronous_tds_process(UX_HCD_EHCI *hcd_ehci, UX_EHCI_HSISO_TD* itd);
UX_EHCI_FSISO_TD    *_ux_hcd_ehci_fsisochronous_tds_process(UX_HCD_EHCI *hcd_ehci, UX_EHCI_FSISO_TD* sitd);
UINT    _ux_hcd_ehci_asynchronous_endpoint_create(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ehci_asynchronous_endpoint_destroy(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ehci_controller_disable(UX_HCD_EHCI *hcd_ehci);
VOID    _ux_hcd_ehci_done_queue_process(UX_HCD_EHCI *hcd_ehci);
VOID    _ux_hcd_ehci_door_bell_wait(UX_HCD_EHCI *hcd_ehci);
UINT    _ux_hcd_ehci_ed_clean(UX_EHCI_ED *ed);
UX_EHCI_ED          *_ux_hcd_ehci_ed_obtain(UX_HCD_EHCI *hcd_ehci);
UINT    _ux_hcd_ehci_endpoint_reset(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ehci_entry(UX_HCD *hcd, UINT function, VOID *parameter);
UINT    _ux_hcd_ehci_frame_number_get(UX_HCD_EHCI *hcd_ehci, ULONG *frame_number);
VOID    _ux_hcd_ehci_frame_number_set(UX_HCD_EHCI *hcd_ehci, ULONG frame_number);
UX_EHCI_FSISO_TD    *_ux_hcd_ehci_fsisochronous_td_obtain(UX_HCD_EHCI *hcd_ehci);
UX_EHCI_HSISO_TD    *_ux_hcd_ehci_hsisochronous_td_obtain(UX_HCD_EHCI *hcd_ehci);
UINT    _ux_hcd_ehci_initialize(UX_HCD *hcd);
UINT    _ux_hcd_ehci_interrupt_endpoint_create(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ehci_interrupt_endpoint_destroy(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
VOID    _ux_hcd_ehci_interrupt_handler(VOID);
UINT    _ux_hcd_ehci_isochronous_endpoint_create(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ehci_isochronous_endpoint_destroy(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UX_EHCI_ED          *_ux_hcd_ehci_least_traffic_list_get(UX_HCD_EHCI *hcd_ehci, ULONG microframe_load[8], ULONG microframe_ssplit_count[8]);
UX_EHCI_ED          *_ux_hcd_ehci_poll_rate_entry_get(UX_HCD_EHCI *hcd_ehci, UX_EHCI_ED *ed_list, ULONG poll_depth);
VOID    _ux_hcd_ehci_next_td_clean(UX_EHCI_TD *td);
UINT    _ux_hcd_ehci_periodic_tree_create(UX_HCD_EHCI *hcd_ehci);
UINT    _ux_hcd_ehci_port_disable(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
UINT    _ux_hcd_ehci_port_reset(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
UINT    _ux_hcd_ehci_port_resume(UX_HCD_EHCI *hcd_ehci, UINT port_index);
ULONG   _ux_hcd_ehci_port_status_get(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
UINT    _ux_hcd_ehci_port_suspend(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
UINT    _ux_hcd_ehci_power_down_port(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
UINT    _ux_hcd_ehci_power_on_port(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
VOID    _ux_hcd_ehci_power_root_hubs(UX_HCD_EHCI *hcd_ehci);
ULONG   _ux_hcd_ehci_register_read(UX_HCD_EHCI *hcd_ehci, ULONG ehci_register);
VOID    _ux_hcd_ehci_register_write(UX_HCD_EHCI *hcd_ehci, ULONG ehci_register, ULONG value);
UX_EHCI_TD          *_ux_hcd_ehci_regular_td_obtain(UX_HCD_EHCI *hcd_ehci);
UINT    _ux_hcd_ehci_request_bulk_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_request_control_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_request_interrupt_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_request_isochronous_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_request_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_request_transfer_add(UX_HCD_EHCI *hcd_ehci, UX_EHCI_ED *ed, ULONG phase, ULONG pid,
                                    ULONG toggle, UCHAR * buffer_address, ULONG buffer_length, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_transfer_abort(UX_HCD_EHCI *hcd_ehci,UX_TRANSFER *transfer_request);
VOID    _ux_hcd_ehci_transfer_request_process(UX_TRANSFER *transfer_request);

#define ux_hcd_ehci_initialize                      _ux_hcd_ehci_initialize
#define ux_hcd_ehci_interrupt_handler               _ux_hcd_ehci_interrupt_handler

/* Determine if a C++ compiler is being used.  If so, complete the standard 
   C conditional started above.  */   
#ifdef __cplusplus
} 
#endif 

#endif

