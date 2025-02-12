# STM32F413 Drivers (and FreeRTOS)

This repository contains drivers for various STM32F413 peripherals.

It also contains the FreeRTOS source, since all boards must depend on it.

This repository is intended to be used as a submodule as part of the build
process for each board's firmware.

## File manifest

### `CMR/`

This directory contains our wrapper interfaces over the HAL (see below). They
encode various assumptions we make about our STM32F413 MCUs, and provide a
convenient interface with their various peripherals. They also provide some
integration with FreeRTOS, making use of synchronization objects and tasks to
aid concurrent usage.

Because these drivers depend on corresponding HAL modules, they will only be
included in the build if the respective modules are enabled in your board's
`stm32f4xx_hal_conf.h`. To determine what modules you need to enable in your
configuration, look for lines like the following in each driver's header file:

```c
// stm32f413-drivers/CMR/can.h

#ifdef HAL_CAN_MODULE_ENABLED
```

Then, in your board's HAL configuration, un-comment the respective line:

```c
// stm32f4xx_hal_conf.h

#define HAL_CAN_MODULE_ENABLED
```

### `HAL/`

This directory contains ST's [Hardware Abstraction Layer (HAL)][HAL]. This layer
is a low-level configuration and control interface for the STM32F4xx series'
hardware and peripherals.

The HAL is divided into many subsystem modules; to reduce build size, most
modules are not enabled by default. Consult your board's individual
`stm32f4xx_hal_conf.h` to check which modules are enabled, or to activate new
modules.

[HAL]: https://www.st.com/en/embedded-software/stm32cubef4.html

### `FreeRTOS/`

This directory contains the [FreeRTOS][FreeRTOS] implementation. As a real-time
operating system, FreeRTOS provides us with scheduling, concurrency, and
synchronization that is suitable for the embedded platforms we are working on.

[FreeRTOS]: https://www.freertos.org/

### `CMSIS/`

This directory contains ST's [Cortex Microcontroller Software Interface
Standard (CMSIS)][CMSIS], which contains standardized names and definitions for
memory-mapped peripherals, interrupts, and other MCU hardware details.

[CMSIS]: https://developer.arm.com/embedded/cmsis

### `PCAN/`

This directory contains our [PCAN Explorer][PCAN] project and symbol
definitions. While source code does not use these files, the symbols should be
kept up-to-date with the CAN IDs and types defined in `CMR/can_ids.h` and
`CMR/can_types.h`, respectively.

[PCAN]: https://www.peak-system.com/PCAN-Explorer-5.249.0.html?&L=1
