/**
  ******************************************************************************
  * @file    readme.txt
  * @author  MCD Application Team
  * @brief   This file lists the main changes done by STMicroelectronics on
  *          USBX host controller driver

  ******************************************************************************
  */

### V3.0.2 / 05-January-2024 ###
===============================
Main changes
-------------
- Fix periodic scheduling when FS devices plugged to hub

Dependencies:
-------------
- Azure RTOS USBX V6.1.12 or higher

### V3.0.1 / 14-July-2023 ###
===============================
Main changes
-------------
- Fix compile error when UX_MAX_DEVICES = 1

Dependencies:
-------------
- Azure RTOS USBX V6.1.12 or higher

### V3.0.0 / 23-December-2022 ###
===============================
Main changes
-------------
- Add ISO transfer support
- Add HUB Split HS Transactions support
- Fix compile warnings in standalone mode

Dependencies:
-------------
- Azure RTOS USBX V6.1.12 or higher

### V2.0.0 / 30-September-2022 ###
===============================
Main changes
-------------
- Align USBX host controller driver against Azure RTOS USBX 6.1.10
- Add standalone mode support

Dependencies:
-------------
- Azure RTOS USBX V6.1.10 or higher

### V1.0.3 / 24-December-2021 ###
=================================
Main changes
-------------
- Free HCD memory resource during HCD DeInit

Dependencies:
-------------
- Azure RTOS USBX V6.1.7 or higher

### V1.0.2 / 22-November-2021 ###
=================================
Main changes
-------------
- Avoid halting channels during URB notification
- Avoid compilation issue with USB_DRD IP (DMA not supported)
- Fix endpoint type for isochronous transfer

Dependencies:
-------------
- Azure RTOS USBX V6.1.7 or higher

### V1.0.1 / 21-June-2021 ###
=============================
Main changes
-------------
- Remove trailing spaces.

Dependencies:
-------------
- Azure RTOS USBX V6.1.7

### V1.0.0 / 25-February-2021 ###
=================================
Main changes
-------------
- First official release of Azure RTOS USBX Host controller driver for STM32 MCU series

Dependencies:
-------------
- Azure RTOS USBX V6.1.3
- STM32Cube HCD HAL drivers
