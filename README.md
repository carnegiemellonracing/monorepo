# FSM

This repository holds the firmware for use on the Front Sensor Module.
It supports the use of 5 ADC channels: See adc.c for their pinouts.
The primary point of configuration for this module is via CAN heartbeat.
Its internal heirarchy is derived from the STM32 template repository.
See: https://github.com/carnegiemellonracing/stm32f413-template

[submodules]: https://git-scm.com/book/en/v2/Git-Tools-Submodules

You can clone this repository via:
`git clone --recursive git@github.com:carnegiemellonracing/FSM.git`

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

