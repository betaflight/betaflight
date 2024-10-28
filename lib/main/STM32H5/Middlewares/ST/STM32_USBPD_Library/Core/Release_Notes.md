---
pagetitle: Release Notes for STM32 USB-C Power Delivery Core Stack Library
lang: en
header-includes: <link rel="icon" type="image/x-icon" href="_htmresc/favicon.png" />
---

::: {.row}
::: {.col-sm-12 .col-lg-4}


<center>
# Release Notes for STM32 USB-C Power Delivery Core Stack Library
Copyright &copy; 2017(-2021) STMicroelectronics\

[![ST logo](_htmresc/st_logo_2020.png)](https://www.st.com){.logo}
</center>

# Purpose

The USB-PD core stack library component provides SW implementation of the USB-PD protocol stack, as described in USB-IF specifications.
This implementation covers features of modules as :

  - PE (Policy Engine)
  - PRL (Protocol Layer)
  - CAD (CAble Detection) for the high level of the type C detection state machine

The library is provided in binary format, comes on top of the STM32Cube HAL driver and offers all the APIs required to develop an USB PD application.

The USB-PD library is developed following the Universal Serial Bus Power Delivery Specification Revision 3.0, V2.0 (August 29, 2019) and Universal Serial Bus type-C Cable 
and Connector Specification, Revision 2.0 (August, 2019). It has passed successfully the official certification.

Here is the list of references to user documents:

- [ST page](https://www.st.com/en/applications/connectivity/usb-type-c-and-power-delivery.html): Key features of the new USB Type-C™ connector
- [UM2552](https://www.st.com/resource/en/user_manual/dm00598101-managing-usb-power-delivery-systems-with-stm32-microcontrollers-stmicroelectronics.pdf): USB power delivery User Manual
- [WIKI Page](https://wiki.st.com/stm32mcu/wiki/USB_Power_Delivery_overview): USB Power Delivery overview

:::

::: {.col-sm-12 .col-lg-8}
# Update History

::: {.collapse}
<input type="checkbox" id="collapse-section32" checked aria-hidden="true">
<label for="collapse-section32" aria-hidden="true">V4.1.1 / 21-Oct-2022</label>
<div>

## Main Changes

### Maintenance release


## Contents
**Fixed bugs list**

  Headline
  --------
  Codespell corrections
  
## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.6
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.31
- STM32CubeIDE V1.7.0

## Supported Devices and boards

## Backward compatibility

NA

## Dependencies

NA

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section31" aria-hidden="true">
<label for="collapse-section31" aria-hidden="true">V4.1.0 / 15-Dec-2021</label>
<div>

## Main Changes

### Maintenance release


## Contents
**Fixed bugs list**

  Headline
  --------
  [OS] Fix THREADX thread ID struct handling
  [OS] Fix define for CMSIS OS v2
  [PE] Update FRS AMS to manage power requirement
  [PE] Clear RX event before FRS ACCEPT
  [OS] update to kill PE if does not stop executing itself
  [OS] replace OS Free RTOS API with cmsis OS API
  [SNK] After a soft reset we shall wait for source capa
  Fix typo error, add legacy define for USPBPD_WRITE32
  [DEF] Add defines for STATUS message
  [PE] SOP' message needs to be sent only if sender is VCONN owner
  [PE] Add the support of the USB4 (message USB_ENTER and DATA_RESET)
  [PE] Revision message must be answered 
  [PE] Status message has 6 bytes whereas 7 are mandatory
  [PRL] Update to handle retry and crc timer for more than 2 ports
  [PRL] MessageId is not well incremented after retry
  [PRL] In PD3 GoodCRC is always sent with PD revision 1
  [CORE] Update the stack to manage 3 ports in parallel
  MCUAstyle, Codespell
  [CORE] Solve CubeIDE compilation warning in OS_CREATE_TASK macro use.
  [Licensing] Update the way to declare licenses in Cube and X-CUBE components
  {PE] SNK AMS HardReset : the USB stack shutdown must be done after VBUS has reached VSAFE_0V
  [PRL] PHY state BUSY is no taken into account
  [UCSI] Add new ID for UCSI trace
  [THREADX] remove pool allocation inside the stack and Add a pool pointer on the InitOS function
  
## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.6
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.31
- STM32CubeIDE V1.7.0

## Supported Devices and boards

## Backward compatibility

NA

## Dependencies

NA

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section30" aria-hidden="true">
<label for="collapse-section30" aria-hidden="true">V4.0.1 / 22-Jun-2021</label>
<div>

## Main Changes

### Maintenance release


## Contents
**Fixed bugs list**

  Headline
  --------
  Update Release Note 

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.6
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.31
- STM32CubeIDE V1.7.0

## Supported Devices and boards

## Backward compatibility

NA

## Dependencies

NA

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section29" aria-hidden="true">
<label for="collapse-section29" aria-hidden="true">V4.0.0 / 6-May-2021</label>
<div>

## Main Changes

### Maintenance release


## Contents
**Fixed bugs list**

  Headline
  --------
  Messages should be discarded when device was about to send HardReset
  Messages must be discarded when VBUS is out of range (like during HR)
  NO_PD mode only, a type C state machine shall runs therefore the CAD task must be created
  Add the data role reset during hard reset AMS
  Astyle update
  OS update around the thread creation
  OS Add the support of the ThreadX RTOS
  Stack state machine issue seen in PHY.E2/E3
  Codespell update 
  OS PE task management improvement to replace create/terminate by suspend/resume
  OS Add file usbpd_os_port.h to manage the OS portability
  Update in Task definition and creation macros for CMSIS RTOS V2
  OS improvement of the os porting layer
  [CAD] VconnCCIs shall be set for a snk attachment
  When an Hard Reset receive too quickly after a message the hardreset is not done"
  MISRAC2012 update 

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.6
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.31
- STM32CubeIDE V1.7.0

## Supported Devices and boards

## Backward compatibility

NA

## Dependencies

NA

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section28" aria-hidden="true">
<label for="collapse-section28" aria-hidden="true">V3.3.1 / 17-Feb-2021</label>
<div>

## Main Changes

### Maintenance release


## Contents
**Fixed bugs list**

  Headline
  --------
  Update timer in Low Power Mode not correctly set



## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.27
- STM32CubeIDE V1.4.0

## Supported Devices and boards

## Backward compatibility

NA

## Dependencies

NA

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section27" aria-hidden="true">
<label for="collapse-section27" aria-hidden="true">V3.3.0 / 26-Nov-2020</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  Enable VCONN support in MIN_SRC library
  [PE] Update to avoid PE lock when called PE_Wakeup inside a critical section
  Add a MIN_DRP configuration in the stack libraries
  [CAD] Hard fault issue detected with CortexM33 (L5)
  [LPM] replace the LPM call by tiny_lpm function
  [PE] new sop' msge shouldn't be sent after src_cap till DUT is not in explicit
  [PE] PE_SNK state machine state change abnormally
  Add mechanisme to guarantee the goodcrc timing
  Implement BIST Shared Capacity Test Mode message
  The RP resistor shall be reset to the default value during the HardReset AMS
  [PE] Hard fault in the usbpd stack library if USBPD_PE_GetDataInfo not implemented in the application
  [PE] Role alignment during HardReset
  [PE] TxOK/NG shouldn't be checked in PD2
  [DOC] Update to add timeout information inside the MSC 
  conflict when PE perform HardReset and CAD a detach
  Rework InitCore function 
  MISRA corrections
  Codespell and MCUAStyle corrections
  [DRD] update around the identical data role for SRC and SNK
  [SRC] add a notification USBPD_NOTIFY_STATE_SRC_READY
  [PE] PE_powerrole after power swap
  [CORE] Update to be aligned with latest USB-PD spec (remove the 2nd UFP VDO)
  [PE] Add defines for conditions on PE_Request_Control
  [PE] AMC and AMA deprecated in PD3.0. AMA only used in PD2.0.
  [PE] Soft_reset should be sent prior to HardReset in case of non response of src_cap post PR_swap
  Tests TD4.3.x failed on Lecroy with NO_PD Sink version
  Add a system of hook function to request an action of USB stack
  Avoid double definition with LE16 & LE32 macros used in usbh.def
  
  
  : Fixed bugs list


## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.27
- STM32CubeIDE V1.4.0

## Supported Devices and boards

## Backward compatibility

NA

## Dependencies

Dependencies with TRACER_EMB V1.4.0 or latest

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section26" aria-hidden="true">
<label for="collapse-section26" aria-hidden="true">V3.2.1 / 8-October-2020</label>
<div>

## Main Changes

### Patch release


## Contents

  Headline
  --------
  Conflict when PE perform HardReset and CAD a detach
  
  : Fixed bugs list


## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.27
- STM32CubeIDE V1.2.0

## Supported Devices and boards

## Backward compatibility

NA

## Dependencies

Dependencies with TRACER_EMB V1.4.0

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section26" aria-hidden="true">
<label for="collapse-section26" aria-hidden="true">V3.2.0 / 08-July-2020</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  SOPSupported not well reset after detach
  DR_swap management issue
  LeCroy new merge test : fail on PRSWAP because tester expecting Hard Reset
  Keep USBPD_CAD_Process API for all the configuration
  PE_datatrole is not aligned to PE_powerRole in DPM_UserCableDetection
  SOFT_RESET sent to SOP1 is not well manage
  cable_reset management to be improved
  Correct misspelled words
  Provide external API in CAD to allow application to know which RP present in Sink
  Should not answer GoodCRC to SOP1 soft reset
  TxOK / TxNG should not be toggled while in PD2 second part to reset the PD capabilities in case of HARDRESET
  Expose RX buffer size to the devices module (to avoid buffer overflow)
  Reduce size of USBPD_SettingsTypeDef in case of NO_PD configuration
  Remove notification USBPD_NOTIF_PROTOCOL_ERROR when stack has received a NAK message to VDM identity request
  Move before resetting VCONN status to be able to correctly switch off VCONN during detach
  Add UFP and VPD VDOs for VDM discovery identity
  Add ErrorRecovery API and remove CAD detection API
  Improve the AMS management to discard any PD send if a message is already present inside the fifo
  Message discarded wrongly while in PD2
  TEST.PD.VDM.SRC.01 fail on LeCroy
  Add new API USBPD_PHY_FastRoleSwapReception
  Set FRS as a PD3 feature
  Integration of the FRS on SRC and SNK state machine
  Remove trace process
  
  : Fixed bugs list


## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.27
- STM32CubeIDE V1.2.0

## Supported Devices and boards

## Backward compatibility

NA

## Dependencies

Dependencies with TRACER_EMB V1.4.0

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section25" aria-hidden="true">
<label for="collapse-section25" aria-hidden="true">V3.1.0 / 14-Apr-2020</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  BIST no more enabled in TCPM
  Indicate when trace is lost
  Bad answer to UVDM msge.
  Hard reset not detected by PE during power negotiation
  Update to avoid issue to send a control message
  Add a check on Resistor in case of VBUS lost (manage HardReset Sequence)
  Update to use a new ID for TCPM trace
  
  : Fixed bugs list


## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.27
- STM32CubeIDE V1.2.0

## Supported Devices and boards

## Backward compatibility

For TCPM implementation, updates have been done on FUSB307 component (tag v3.0.0).

## Dependencies

Dependencies with TRACER_EMB V1.4.0

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section24" aria-hidden="true">
<label for="collapse-section24" aria-hidden="true">V3.0.0 / 26-Mar-2020</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  [CORE] Explicit_Contract information not properly sent to UCPD Monitor on DETACH
  [CORE] Add inside PRL a mechanism to avoid RX buffer overwrite in case of multiple RX
  [PE] Discard shall be done only if there is incoming message
  [TCPM] Update for TCPM compilation
  [PRL] update for TCPC mode
  [PRL] Add an optional mechansime to manage the tx discard and tx abort by UCPD IP
  [PE] Lecroy TEST.PD.VDM.SNK.7 Unrecognized VID in Unstructured VDM
  [PE] TD PD.SRC3.E27 and TD PD.SRC3.E28 - Testing Downstream Port
  [PE] Reset the spec revision + data role in case of HardReset
  [PE] MQP regression on TD.PD.VNDI3.E10
  [PE-PRL] Add critical section on the cluster management
  [PE] state machine SRC wrongly managed at reset
  [PRL] Rework to avoid usage of reentrant state machine
  [PE] keep GPTimer only for TCPC compilation
  [PE] rework of HardReset + ResetDuringSwap + PE Reset
  [PRL] Code optimizations
  [PRL] remove the disable RX to avoid overwrite of rx buffer
  [CORE][TCPM] Reset Supported SOPs to SOP when disconnect
  Merge of modifications done in usbpd_dpm_core.c files in CubeMx generated projects 
  
  : Fixed bugs list


## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.27
- STM32CubeIDE V1.2.0

## Supported Devices and boards

## Backward compatibility
In order to use this version of USBPD Core module, please ensure version of USBPD Device driver module mentions compatibility with this v3.0.0 USBPD Core.
(Example for STM32G4xx => use g4_v4.0.0 or higher version of USBPD Device
         for STM32L5xx => use l5_v2.0.0 or higher version of USBPD Device
		 ...

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section23" aria-hidden="true">
<label for="collapse-section23" aria-hidden="true">V2.10.0 / 19-Feb-2020</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  correction to manage correstly the SNK wakeup
  [CORE] Add dedicated SNK state machine to handler VPD identification PRL update to manage SOP*
  [CORE] Add PD message In in TCPM mode
  [CORE] Add switch USE_STM32_UTILITY_OS to integrate SEQUENCER utility
  [CORE] Update done to integrate NO PD stack
  [CORE][PE] Do not send SOFT_RESET if NOT_SUPPORTED answered to GET_BATTERY messages
  [CORE][PRL] Extended message GET_BATTERY_CAPA should not be seen as BIST message
  [CORE][TCPM] Manage TX complete alert before RX alert
  [NRTOS] Integration of the utilities SEQ and TIMER_SERVER to handle the non RTOS mode
  [PE] Get Source and sink capability extended message should be sent in DRP
  [PRL] patch TD.PD.VNDI.E4 SOP* Handling
  [TCPC] remove specific management of SOP1, SOP2 and let PE manage
  Update done to integrate NO PD stack
  Handle incoming message in discard case
  AMS not correctly set for GET_SNK_CAPA_EXT message
  Trace impacts timing during unchunk test
  Core library file generates warnings with ARM Compiler V6
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.27
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section23" aria-hidden="true">
<label for="collapse-section23" aria-hidden="true">V2.9.0 / 15-Nov.-2019</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  Core library file generates warnings with ARM Compiler V6
  Product_type_PSD needs to be added
  [CORE][PE] SNK_extended_cap must be answered while SNK or DRP
  [CORE][PE] Increase PE_TSRCRECOVERHARDRESET_MAX timing to be less restrictive during hard reset
  [CORE]Increase Stack size for TRACE task (issue with Nucleo_L5)
  Enabled the switch USBPDCORE_ERROR_RECOVERY for the libraries configuration
  Rework the error recovery + disable RX in error recovery and disable state
  Add library NO_PD with the switch USBPDCORE_LIB_NO_PD
  [CORE] Change TRACE priority and change condition on NRTOS to trig the PE scheduling
  Remove un-needed notification
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.27
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section22" aria-hidden="true">
<label for="collapse-section22" aria-hidden="true">V2.8.0 / 06-Sept-2019</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  [USBPD][CORE] Rework of the hard reset procedure to avoid issue with TD.PD.LL.E4
  [USBPD][CORE] CMSIS OS V2 adaptations
  [USBPD][CORE] Code optimization + notification moved before the swapongoing reset to avoid wrong VBUS detach level
  [USBPD][CORE][PE] GRL issue SNK3.E7 - Do not send request in SinkTXOK is not set
  [USBPD][CORE] Remove dependency with max number of ports defined in the library (USBPD_MAXPORT_COUNT)
  [USBPD][CORE] Manage correctly errors during core stack initialization
  [USBPD][CORE] Remove notification inside function PE_StateMachine_SRC_NegocCapa: avoid duplicate information because it is application decision
  [USBPD][CORE] Few updates to integrate the TCPC G0
  [USBPD][CORE] [CubeMx-UBSPD] Update generated code to make it compatible with CMSIS-RTOS v2.
  [USBPD][CORE][PE] Fix an issue with Elissys test 4.9.4
  [USBPD][CORE] Add specific tag for tracing TCPM
  [USBPD][CORE][TCPM] remove Wait for VBUS in TCPM file
  [USBPD][CORE][TCPM] Update to manage G0 as TCPC
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.26
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section21" aria-hidden="true">
<label for="collapse-section21" aria-hidden="true">V2.7.0 / 11-July-2019</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  [USBPD][CORE] Update for TCPM after integration of TCPC solution on G0
  [USBPD][CORE] Add management of new LIB : USBPDCORE_LIB_PD3_CONFIG_MINSNK
  [USBPD][DOC] MSC and documentation update
  [USBPD][CORE] Few updates for TCPM
  [USBPD][CORE] Correction for allowing 2 ports in Non RTOS configuration
  [USBPD][CORE] [CubeMx-UBSPD] Update generated code to make it compatible with CMSIS-RTOS v2.
  [USBPD][CORE] Update define name in usbpd_dpm_core.c
  [UCPD][PE] add a flag to support or not Battery feature in the stack
  [USBPD][CORE][PE] calculate correct timing for request in Sink
  [USBPD][CORE] Improve the disconnection timing to avoid issue with Ellisys (4.8.3) - put PE lower prio than CAD and increase mailbox of CAD
  [USBPD][CORE] Change core version
  [USBPD][CORE][PE] Should accept DR swap if no enter in VDM mode (GRL issue)
  [USBPD][1602] Add USBPD_HR_STATUS_START_REQ and USBPD_HR_STATUS_START_ACK inside the Hardreset process of the sink
  [USBPD][CORE] Remove static from TraceQueueId declaration to allow using TRACE in DISCO spy mode
  [USBPD][LOWPOWER] update to allow stop mode in attached cases
  [USBPD][LOWPOWER] add a link between the task and the low power capability
  [USBPD][CORE] Allow to enable only UVDM and not SVDM and/or VCONN_SUPPORT
  [USBPD][CORE] remove the lowpower, must be handled on application side
  [USBPD][CORE][PE] MQP Regression linked to update of cable revision.
  [USBPD][TRACE] wakeup trace process only in task context
  [USBPD][G0] update to manage correctly the SwapOngoing in SRC cases
  [USBPD] reset SwapOngoing to avoid CAD lock
  [USBPD] update to avoid regression link with trace optim
  [USBPD] change the way to exit PE task
  [LOWPOWER] update to avoid issue to manage repetition timing
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.32.3
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section20" aria-hidden="true">
<label for="collapse-section20" aria-hidden="true">V2.6.0 / 11-April-2019</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  [LOWPOWER] improve dpm core + add deinit interface to usbpd trace
  [LOWPOWER] add interface to update the PE timer according lowpower time
  [PE] SOP'/SOP'' should be enabled on PRL only if SOP'/SOP" communication is started (not linked to VCONN source) - Bis
  [PE] rework of cable reset management is needed (vconn must be impacted)
  [PE] Cable information (using SOP prime) only works once
  [PE] Soft reset should be used when extended message received after tSenderResponse
  [PE] Alert data could be overwritten before the PE sent
  [PE] Alert reception can generate an hardfault
  [PE] Enable RX after sending a cable reset
  Communicate in PD2 with cable if no respond to PD3
  USBPD_HR_STATUS_FAILED needs to be kept
  SINK needs to be able to issue a Hard reset
  [DEF] Missing PPS Power Limited Bit
  [PE] AMS should be started only if message has been sent to port partner
  [PE] PE state machine in should be called immediately during PPS negotiation to enter correctly in ready_wait state
  [PE] Fix few MISRA warnings
  [PE] Add Prorocol_error notification in case of wrong received VDM Discovery identity.
  [PE] remove callback trace for VDM specific (move to dpm_user)
  [TRACE] fix trace length (because of a MISRA update)
  [TRACE] wakeup trace process only in task context
  [TRACE] improve the trace system to free CPU time
  update to manage correctly the SwapOngoing in SRC cases
  reset SwapOngoing to avoid CAD lock
  change the way to exit PE task
  USBPD_DPM_IsPowerReady called in SRC and SNK
  Move PDFU files to another folder

  : Fixed bugs list

## Known Limitations

  Outstanding bugs list : None

  Requirements not met or planned in a forthcoming release : None

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

  All STM32 devices embedding USBPD IP
  All STM32 devices implementing USB-PD feature as TCPC

## Backward compatibility

  No compatibility break with previous version

## Dependencies

  No identified dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section19" aria-hidden="true">
<label for="collapse-section19" aria-hidden="true">V2.5.2 / 02-April-2019</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  Forbidden word detected in usbpdfu_responder.c (files removed from delivery)

  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section18" aria-hidden="true">
<label for="collapse-section18" aria-hidden="true">V2.5.1 / 15-March-2019</label>
<div>

## Main Changes

### Maintenance release


## Contents

  Headline
  --------
  Remove dependencies with CMSIS drivers version  

  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section17" aria-hidden="true">
<label for="collapse-section17" aria-hidden="true">V2.5.0 / 19-December-2018</label>
<div>

## Main Changes

### Maintenance release

  Headline
  --------
  Add prototype definition for HAL_GetTick() (needed for non rtos applications)
  Simplify function USBPD_TRACE_Add which is no more used by GUI
  Update link with rx patch: certification regression
  Update to manage Rx Enable
  Callback USBPD_DPM_HardReset used only in SRC or DRP config and removed unused enum for HR (USBPD_HR_STATUS_MSG_RECEIVED & USBPD_HR_STATUS_FAILED)
  Add switch _TRACE to exclude usbpd_trace.c if not _TRACE not enabled

  : Fixed bugs list
  
## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies


</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section16" aria-hidden="true">
<label for="collapse-section16" aria-hidden="true">V2.4.0 / 12-Dec-2018 </label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  [USBPD] renaming inside USBPD_SettingsTypeDef
  [USBPD][CORE][DEF] Wrong value for SVDM ATTENTION message
  [USBPD][CORE][PE] Remove return FAIL if no AMA is supported
  [USBPD][CORE][PE] Need notification when ALERT message has been sent by PE
  [USBPD][CORE][PE] ALERT message shouldn't be considered as AMS message
  [USBPD][CORE][PRL] Add debug switch _PRL_DEBUG_DISABLE_BIST_TRACE (not enabled) to disable BIST message in the traces
  [USBPD][CORE][PE] Minor updates due to compilation issue when __DEBUG_LEVEL is set to 2
  [USBPD][CORE][PE] Disable RX during SNK hard reset sequence
  [USBPD][CORE][PE] SRC3.E25 fail: issue in chunking tests
  [USBPD][CORE][PE] Collision avoidance once an Explicit Contract is in place
  [USBPD][CORE][DEF] TD.PD.SNK3.E15 Status Fields Check - add power status field in SCB structure
  [USBPD][G0] update to allow the treatment of an incoming message when waiting for sinkTxNG timeout
  [USBPD][G0] increase tsendertimer to 29ms and update the HardReset state change to use notimeout

  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section15" aria-hidden="true">
<label for="collapse-section15" aria-hidden="true">V2.3.0 / 15-Nov-2018 </label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  [USBPD][CORE] Increase PE Stack size for Authentication configs
  [G0] correction retry patch
  [G0] manage retry according the current spec revision value
  [G0] Patch ellisys test TD.PD.SRC3.E26 Soft reset sent regardless of Rp value
  [G0] report gotomin + CtrlMessage capability from G0
  [PRL] PD revision should be set to 0 in GoodCRC message only if PD2.0
  add the cable reset tracking inside the tracer
  API update for the following function USBPD_VDM_InformSVID : remove the return which is not tested inside PE USBPD_VDM_InformMode : remove the return which is not tested inside PE USBPD_VDM_InformModeEnter : remove the return which is not tested inside PE USBPD_VDM_SendAttention : remove the return which is not tested inside PE USBPD_VDM_SendSpecific : remove the return which is not tested inside PE USBPD_VDM_InformSpecific : remove the return which is not tested inside PE USBPD_VDM_SendUVDM : remove the return which is not tested inside PE
  Correct wrong destination state in SNK state machine (case PE_SNK_REQUEST_GETSNKCAP_WAITANSWER) : go to PE_SNK_SEND_SOFT_RESET instead of PE_SRC_SEND_SOFT_RESET in case of timeout.
  increase stack size to avoid hardfault with SW4STM32
  Solve one issue in PRL_Received() in case of extended messages. Extended Header was not properly retrieved from RX buffer.
  [3.0] Add battery status management
  [3.0] Authentication : Modifications after end of PlugFest : - Use NOTIMEOUT state update from PE_Sxx_EXTENDEDMESSAGE_SENDING_COMPLETE to PE_Sxx_EXTENDED_WAITRESPONSE in order to avoid losing next chunk request. - Addition of SOPType input parameter in USBPD_PE_SendExtendedMessage() to allow authentication messages to be carried on SOP' - Extension of PE trace buffer from 20 to 30 (useful when chunking is used).
  [3.0] Correction of TD.PD.SRC3.E25 and TD.PD.SNK3.E25 (Receiving Chunked Extended Message)
  [3.0] fix ellisys issue regarding answering NOT_SUPPORTED for SRC_CAPA_EXT in SNK config + NON_SUPPORTED if VDM is not enabled (but test VNDI3.E3 is still failed)
  [3.0] Update SetData and GetData info to provide size in bytes and no more words
  [3.0][PE] Send Not_supported message for DR_SWAP & PR_SWAP in case of SRC only (MQP issues)
  [Core] : Solve critical section issue met on G0 Cut2.0 validation : SRC cap transmitted again despite reception of GoodCRC of previous sending. A critical section has been added before moving in RETRY state.
  [Core] : Update in CMSIS include policy to fit with CMSIS core v4.5 or v5
  [CORE] : update Vconn Support switch name to _VCONN_SUPPORT (as in Application side).
  [CORE] Add missing Extended Message handling in case of SECURITY RESPONSE message type
  [CORE] Align combination of compilation switches used in Collision avoidance functions between H and C
  [CORE] check return on USBPD_DPM_UserInit function
  [CORE] Merge back modifications done for G0 FW v0.9.0 delivery (Stack size updates for PE + CAD, Release_Notes.html)
  [CORE] Move GUI init in GUI_interface module
  [CORE] remove Disable RX during hard reset sequence (fix pb with hard reset on FUSB307 when RX is disabled)
  [CORE] Need to have 'MAX' function as the 'MIN' already present
  [CORE] MQP TD.PD.SRC.E12 Test Failed
  [CORE] Update done for MISRAC2012
  [CORE][DEF] TD.PD.SNK3.E15 Status Fields Check - add power status field in SCB structure
  [CORE][DEF] Wrong value for SVDM ATTENTION message
  [CORE][PE/DPMCORE] : End of Data Role Swap procedure implementation. Solving of SoftReset issue after Datarole swap executed.
  [CORE][PE] Answer to SVDM message only if PE_RespondsToDiscovSOP is enabled
  [CORE][PE] Change state when TPPS timer expires (directly go through PE_SEND_REQUEST) + update lib version to v2.3.0
  [CORE][PE] Disable RX during SNK hard reset sequence
  [CORE][PE] Remove return FAIL if no AMA is supported
  [CORE][PE] Send not supported when VDM enabled in the stack but not in the VIF (DISCO)
  [CORE][PE] Need notification when ALERT message has been sent by PE
  [CORE][PE] Collision avoidance once an Explicit Contract is in place
  [CORE][PE] SRC3.E25 fail: issue in chunking tests
  [CORE][PE] ALERT message shouldn't be considered as AMS message
  [CORE][PE] Update for memory optimizations
  [CORE][PRL] Restore default value for PRL_RETRY_TIMER_VALUE in case of Keil or SW4STM32
  [CORE][PRL] remove 2nd call of USBPD_PHY_IsResistor_SinkTxOk
  [CORE][PRL] Remove interface 'USBPD_PHY_ChannelIdleAfterBusy between PHY & PRL'
  [DPM] Remove semaphore to send message to DPM mailbox
  [DPM] Update DPM param and settings to save Src Extended capa
  [G0] Correct reset of response timer in few state machine
  [G0] Fix Ellisys issue SRC3.E29
  [G0] increase tsendertimer to 29ms and update the HardReset state change to use notimeout
  [G0] remove GUI and User application init, move inside main.c
  [G0] separate the emb tracer and usbpd application
  [G0] update to allow the treatment of an incoming message when waiting for sinkTxNG timeout
  [G0][PPS] Move start of the timer PE_SRCPPSCommTimer in PE_SRC_READY state
  [G0]Move PE_UnchunkSupport & PE_FastRoleSwapSupport in PD3 structure + add PE_FirmUpdateResponseSupport
  [GUI] Add field in DPM_Settings structure
  [GUI] add PD3 structure
  [GUI] Add USBPD_TRACE_SendNotification API
  [GUI] Integrate _GUI_INTERFACE
  [GUI] Move GUI processing from Core to GUI utilities
  [GUI] Save Data in FLASH
  [MB1357] Add check on VBUS when start SNK
  [PD30] Fix ellisys issues (BATTERY_CAPA + SECURITY REQUEST) + add flag PE_SecuResponseSupport
  [PD30] Modifications done to be able to send GET_SRC_CAPA_EXT message
  [PDO] : 1st step in PDO initialisation rework : Addition of defines usad in APDO values building.
  [PE/PRL] Add interfaces to enable / disable RX + modify callbacks for VDM message ATTENTION
  [PE] Add macro for manage not support or reject message depending
  [PE] Add not supported by GET_SNK_CAPA for SRC
  [PE] Align SNK_READY to SRC_READY state
  [PE] Change VDM specific callbacks
  [PE] fix an issue with SRC extended message
  [PE] Fix Ellisys issue TD.PD.VNDI3.E3 VDM Identity
  [PE] Increase PE_TSNKWAITCAP from 500 to 600 to decrease probability of the test TD4.3.4 but test still failed :-(
  [PE] Need to reset PE variables after hard reset
  [PE] Put again PE_TSNKWAITCAP to 500 as regressions!!!
  [PE] Reset AMS in SRC_READY state
  [PE] Reset PE_HardResetCounter when SRC capa has been received
  [PE] Few modifications to be done in PE stack after review
  [PE] Issue in mode off when product presents default Rp
  [PPS] Fix issues on Ellisys (SCR3.E28, E30, E31...)
  [PPS] Fix issues on Ellisys (SNK.E13, E14)
  [PPS]Reduce value of PE_TPPSTIMEOUT to 14s
  [STACK] Align with modifications done onb G0
  [STACK] Align with new inclusion model done on G0
  [VCONN] Add USBPD_PE_EvaluateVconnSwap function + change switch from _VCONN_SWAP to _VCONN_SUPPORT
  [VCONN] Evaluate VConn swap in VCONN substate machine
  [VDM] Add check on VDM init function
  [VDM] Change USBPD_VDM_UserInit interface + update functions descriptions
  [VDM] Fix issues on VDM identity and reply to specific
  [VDM] reset PD_Request when Attention message is sent
  [VDM] Wrong return used for USBPD_VDM_UserInit
  [VCONN] start to implement Vconn swap. SNK mode only
  [VDM] Solve MQP PROT-R3-DISCOV test failed on VDM3.E1
  add a function ptr to handle automatic update of the trace if NULL means no trace
  Add support of VCONN and some PD3.0 features (Manufacturer info...)
  considered REJECTED/NOT SUPPORTED valid in case of GETSOURCE_CAP done by source
  correction for gotomin scenario
  correction for TD.PD.SNK3.E12
  correction for VDM3.E2
  correction patch retry
  correction TD.PD.SRC.E3
  DataRole swap test correction
  disable message reception during HARD reset AMS
  Increase PE stack size following crash observed on MQP and Ellisys.
  Lecroy patch TDA 2.2.6 2.2.9
  Lecroy patch TDA.2.2.9 Get Source Cap Receive test
  Merge "[USBPD][PE] Reset PE_HardResetCounter when SRC capa has been received"
  Modifications for VNDI.E7 (Sec Msgs). Still failed on MQP due to GetExtendedSRCCaps request sent by us, but question sent to MQP).
  move CC line assignment before calling DPM_UserCableDetection
  Move trace init after creation of the trace threads
  Notification done with 1 param
  Notification done with only one parameter
  patch TD.PD.DPU.E2 Status Update Command
  PD3.0 manufacturer info management
  PRL alignment with GO
  remove inside PE dpm dependence + lib regeneration
  remove patch to disable RX incompatible with hard reset management
  [DPM] VBUS should be kept enabled when NonPD capable with Rd is present
  Add error recory management inside USBPD stack
  Update PDO typedefs and constants for reflecting Rev3.0 specs (addition of Unchunk support in PDO Source Fixed, and FRS required current in PDO Sink Fixed).
  update USBPD_PE_Request_CtrlMessage to handle more CTRL message
  Vconn Swap and PD3 integration (Core)


  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section14" aria-hidden="true">
<label for="collapse-section14" aria-hidden="true">V2.2.5 / 10-October-2018</label>
<div>

## Main Changes

### Maintenance release

  Headline
  --------
  Addition of CORE stack API in chm file : \doc\USBPD_CORE_RELEASE_User_Manual.chm
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section13" aria-hidden="true">
<label for="collapse-section13" aria-hidden="true">V2.2.3 / 12-Sept.-2018</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  Add the check of the value returned by USBPD_DPM_UserInit.
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section12" aria-hidden="true">
<label for="collapse-section12" aria-hidden="true">V2.2.2 / 06-Sept.-2018</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  Corrections done link with tester update Ellisys, MQP, GRP and Lecroy.
  Code improvements
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section11" aria-hidden="true">
<label for="collapse-section11" aria-hidden="true">V2.2.1 / 19-June-2018</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  Correction for compilation warning under Keil IDE.
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section10" aria-hidden="true">
<label for="collapse-section10" aria-hidden="true">V2.2.0 / 20-Apr-2018</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  Alignment between USBPD Core stack delivered in STM32Cube firmware packages and X-Cube USBPD Expansion packages.
  Addition of TCPM feature support. Dedicated libs provided.
  Libraries provided for IAR v7, IAR v8, Keil and SW4STM32.
  Maintenance fixes.
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section9" aria-hidden="true">
<label for="collapse-section9" aria-hidden="true">V2.1.3 / 20-Mars-2018</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  IAR lib compiled with IAR v8.20.2
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section8" aria-hidden="true">
<label for="collapse-section8" aria-hidden="true">V2.1.2 / 22-Jan-2018</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  Add library for SW4STM32 patch
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section7" aria-hidden="true">
<label for="collapse-section7" aria-hidden="true">V2.1.0 / 20-Sep-2017</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  PD3.0 Full.
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section6" aria-hidden="true">
<label for="collapse-section6" aria-hidden="true">V2.1.1 / 8-Dec-2017</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  Patch to accept message not supported in case of data role swap
  Patch to abort AMS power role swap in case of rejection
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section5" aria-hidden="true">
<label for="collapse-section5" aria-hidden="true">V2.0.0 / 13-Sep-2017</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  First official version as element of a STM32Cube FW package.
  Rework in order to enable support of PD3.0 (not activated yet).
  Addition of trace system.
  Reorganization between Core and User/application related code parts and functionalities.
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section4" aria-hidden="true">
<label for="collapse-section4" aria-hidden="true">V1.5.0 / 20-Fev-2017</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  USBPD Core Stack version delivery for X-CUBE-USBPD V2.0.0.
  Code clean-up and code comments review. Code documentation added (Doxygen).  Compilation switch renaming, ...
  Addition of PD3.0 revision support (specific libs created)
  Rework of Application capabilities definition by user : Now handled in new structures to be personalized by user (usbpd_dpm_conf.h, usbpd_dpm_user.h and usbpd_pdo_defs.h files located on User side could be updated).
  New set of libraries delivered :
  PD2 Config_1 : Standard support of PD2.0 (Snk, Source or DRP, 1 or 2 ports, PR Swap, DR Swap, VCONN management support, ...)
  PD2 Full : equal to PD2 Config_1 + support of VDM feature.
  PD3 Config_1 : Standard support of PD3.0 (Snk, Source or DRP, 1 or 2 ports, PR Swap, DR Swap, VCONN management support, support of specific PD3.0 features as ALERT, SRC_CAPA_EXT, STATUS, BATTERY_STATUS, BATTERY_CAPA, MANU_INFO, COUNTRY_MSG messages, support of Extended messages,...) 
  PD3 Full : equal to PD3 Config_1 + support of VDM feature.
  CAD rework with management of event queue.
  Several updates done for fixing issues discovered during continuous integration of Core Stack vs official Conformance test tools (MQP, Ellisys, LeCroy).
  Addition of Trace feature (for debugging purpose).
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::
::: {.collapse}
<input type="checkbox" id="collapse-section3" aria-hidden="true">
<label for="collapse-section3" aria-hidden="true">V1.4.0 / 24-Jul-2017</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  Updates after compliance testing completion on STUSB1602 Device
  Implementation of Error Recovery management
  Code clean, solving of some compilation warnings in SW4STM32.
  [VDM] Addition of API for starting USB Billboard driver.
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section2" aria-hidden="true">
<label for="collapse-section2" aria-hidden="true">V1.2.1 / 24-Apr-2017</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  Second official version after USBPD library split into Core and Device parts. This item only refers to Core stack and is device independent.
  Updates for support of STUSB1602 Device
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section1" aria-hidden="true">
<label for="collapse-section1" aria-hidden="true">V1.2.0 / 16-Jan-2017</label>
<div>

## Main Changes

### Maintenance release

Maintenance release

## Contents

  Headline
  --------
  First official version after USBPD library split into Core and Device parts. This item only refers to Core stack and is device independent.
  Main Changes compared  to USB-C Power Delivery Library delivered in previous versions 
  Addition of VDM support
  Various corrections in regards with test updates and test additions in USBPD Conformance test tools
  Move PDO definition/capabilities management inside DPM/PWR_IF
  Remove HW dependency with Timerserver (Timerserver feature now handled in Device)
  
  : Fixed bugs list

## Known limitations

## Development Toolchains and Compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.25
- System Workbench STM32 (SW4STM32) toolchain V2.7.2

## Supported Devices and boards

## Backward compatibility

## Dependencies

</div>
:::

:::
:::

<footer class="sticky">
For complete documentation on STM32,visit: [[www.st.com/stm32](http://www.st.com)]

This release note uses up to date web standards and, for this reason, should not be opened with Internet Explorer
but preferably with popular browsers such as Google Chrome, Mozilla Firefox, Opera or Microsoft Edge.
</footer>
