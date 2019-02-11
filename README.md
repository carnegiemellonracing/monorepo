# STM32F413 firmware template

This repository contains a template project for STM32F413-based CMR boards.

<!-- XXX Edit the title and section description for your board! -->

## Using the template

1. Create a new repository for the board.
    - Please select an appropriate repo name specific to your board!
    - **DO NOT** add any files - the repo must be empty.
2. Clone the new repository to your workstation:
    - `git clone <new repo path>`
    - The resulting repository should be empty.
3. Temporarily add the template as a remote for the new repo, pull from it, then
   push to the new repo's origin:
    - `cd path/to/new/repo`
    - `git remote add template git@github.com:carnegiemellonracing/stm32f413-template.git`
    - `git pull template master`
    - `git push -u origin master`
4. `grep -Hn 'XXX' .* *` will provide a list of locations that should be edited.
    - For each location, please do the right thing for your board.
    - Much of this is peripheral configuration (e.g. pin setup, IDs, etc.).
    - Some of these are RTOS tasks that should get more fleshed-out behavior.
5. **Replace this README section** with something specific to your board! Ideas:
    - What modules do you use, and why?
    - Catalog of device and pin configurations
        - What settings did you choose, and why?
    - Catalog of node-specific RTOS tasks
        - What do they do, and why?
    - Update the file manifest below with more files/directories as necessary!

<!-- XXX Edit this section as described above! -->

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

