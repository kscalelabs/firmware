# KIT_XMC48_RELAX_ECAT_V1 BSP

## Overview

The XMC4800 Microcontroller evaluation kit; EtherCAT node; Ethernet connection; CAN node and microSD Card slot included; Hardware compatible with compatible with 3.3V Arduino Shields.     
**Note:**
Programming this kit requires installing 
[SEGGER J-Link software](https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack)

![](docs/html/board.png)

To use code from the BSP, simply include a reference to `cybsp.h`.

## Features

### Kit Features:

* XMC4800-F144 Microcontroller based on ARM® Cortex®-M4 @144MHz, integrated EtherCAT Slave Controller, 2MB Flash and 352KB RAM
* On-board Debug Probe with USB interface supporting SWD + SWO
* Detachable J-Link debugger and UART virtual COM port, with micro USB connector
* Virtual COM Port via Debug Probe
* Power over USB
* ESD and reverse current protection
* 2 x user button and 2 x user LED
* Arduino hardware compatible 3.3V pinout
* Real Time Clock crystal
* Quad SPI Flash
* microSD Card Slot
* Ethernet PHY and RJ45 Jack
* CAN node
* EtherCAT node

### Kit Contents:

* KIT_XMC48_RELAX_ECAT_V1 evaluation board

## BSP Configuration

The BSP has a few hooks that allow its behavior to be configured. Some of these items are enabled by default while others must be explicitly enabled. Items enabled by default are specified in the KIT_XMC48_RELAX_ECAT_V1.mk file. The items that are enabled can be changed by creating a custom BSP or by editing the application makefile.

Components:
* Device specific category reference (e.g.: CAT1) - This component, enabled by default, pulls in any device specific code for this board.

Defines:
* CYBSP_WIFI_CAPABLE - This define, disabled by default, causes the BSP to initialize the interface to an onboard wireless chip if it has one.
* CY_USING_HAL - This define, enabled by default, specifies that the HAL is intended to be used by the application. This will cause the BSP to include the applicable header file and to initialize the system level drivers.
* CYBSP_CUSTOM_SYSCLK_PM_CALLBACK - This define, disabled by default, causes the BSP to skip registering its default SysClk Power Management callback, if any, and instead to invoke the application-defined function `cybsp_register_custom_sysclk_pm_callback` to register an application-specific callback.



See the [BSP Setttings][settings] for additional board specific configuration settings.

## API Reference Manual

The KIT_XMC48_RELAX_ECAT_V1 Board Support Package provides a set of APIs to configure, initialize and use the board resources.

See the [BSP API Reference Manual][api] for the complete list of the provided interfaces.

## More information
* [KIT_XMC48_RELAX_ECAT_V1 BSP API Reference Manual][api]
* [KIT_XMC48_RELAX_ECAT_V1 Documentation](https://www.infineon.com/cms/en/product/evaluation-boards/kit_xmc48_relax_ecat_v1/)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Infineon GitHub](https://github.com/infineon)
* [ModusToolbox™](https://www.cypress.com/products/modustoolbox-software-environment)

[api]: https://infineon.github.io/TARGET_KIT_XMC48_RELAX_ECAT_V1/html/modules.html
[settings]: https://infineon.github.io/TARGET_KIT_XMC48_RELAX_ECAT_V1/html/md_bsp_settings.html

---
© Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation, 2019-2022.