# RAM

This repository implements firmware for the Remote Access Module.
It does some wonky things to stream data slices at low bandwidth off
of the car. It is not totally dissimilar to the TOM firmware in this way,
but it is by design less general purpose.

You can clone this repository via:
`git clone --recursive git@github.com:carnegiemellonracing/RAM.git`

## Project Structure

The project directory is organized as follows:

- `.env`: Environment variables file.
- `.gitignore`: Git ignore file.
- `.gitmodules`: Git submodules configuration.
- `.project`: Project configuration file.
- `.vscode/`: Visual Studio Code configuration files.
- `build/`: Build output directory.
- `build.sh`: Build script.
- `can_fmt.json`: CAN format configuration file.
- `ci-scripts/`: Continuous integration scripts.
- `cJSON/`: cJSON library directory.
- `cmake/`: CMake configuration files.
- `CMakeLists.txt`: CMake build configuration file.
- `CMakePresets.json`: CMake presets configuration file.
- `cn-cbor/`: CN-CBOR library directory.
- `doxygen.conf`: Doxygen configuration file.
- `Inc/`: Header files directory.
- `marshall.py`: Marshalling script.
- `README.md`: Project README file.
- `requirements.txt`: Python dependencies file.
- `simulatecar.py`: Car simulation script.
- `Src/`: Source files directory.
- `Startup/`: Startup files directory.
- `static.sh`: Static analysis script.
- `stm32f413-drivers/`: STM32F413 drivers directory.
- `stm32f413rgtx_FLASH.ld`: STM32F413RG linker script.
- `tests/`: Test files directory.