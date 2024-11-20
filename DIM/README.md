# Driver Interface Module

**NOTE**: This project uses [Git submodules][submodules].
- When cloning this repository for the first time `git clone git@github.com:carnegiemellonracing/DIM.git --recursive`
- When updating an existing clone: `git submodule update --init --recursive`

This repository contains firmware for the Driver Interface Module.

[submodules]: https://git-scm.com/book/en/v2/Git-Tools-Submodules

## TODO

- What modules do you use, and why?
- Catalog of device and pin configurations
    - What settings did you choose, and why?
- Catalog of node-specific RTOS tasks
    - What do they do, and why?
- Update the file manifest below with more files/directories as necessary!

## File manifest

- [`stm32f413-drivers/`][drivers]: [Shared device drivers/system code.][drivers]
- `.cproject`: [TrueStudio][truestudio] C project file.
    - Managed by TrueStudio project configuration GUI.
- `.project`: [TrueStudio][truestudio] project file.
    - Managed by TrueStudio project configuration GUI.
- `stm32f4xx_hal_conf.h`: [STM32 hardware abstraction layer (HAL)][hal] config.
    - Mostly used for enabling HAL modules.
- `FreeRTOSConfig.h`: [FreeRTOS][freertos] config.
    - Mostly used for enabling various RTOS features.
- `main.c`: Firmware entry point.
    - Some small node-specific RTOS tasks can go here.
    - However, if they become unwieldly, they should go into their own modules.
- `*.{c,h}`: Board-specific implementation modules.
    - Device peripheral management, including RTOS tasks when applicable.
    - Node-specific APIs/RTOS tasks.

[drivers]: https://github.com/carnegiemellonracing/stm32f413-drivers
[truestudio]: https://atollic.com/truestudio/
[hal]: https://www.st.com/en/embedded-software/stm32cubef4.html
[freertos]: https://freertos.org/

