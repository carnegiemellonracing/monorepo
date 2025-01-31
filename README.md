# Carnegie Mellon Racing Monolithic Repository

This repository consolidates individual board codebases into a single, unified structure. Each board retains its distinct directory, preserving the concept of individual board repositories while benefiting from a centralized codebase.

---

## Overview of Boards

- **DIM**: Driver Interface Module
- **HVC**: High Voltage Controller
- **PTC**: Powertrain Controller
- **RAM**: Remote Access Module
- **VSM**: Vehicle Safety Module

---

## Requirements

- **CMake**
- **STM32CubeCLT**
If you are part of CMR [this](https://cmr.red/confluence/display/ENR/Flashing+25e) is a comprehensive guide for setup.

---

## Why Use a Monorepo?

### Unified Driver Code

Previously, each board maintained its own detached version of the `stm32f413-drivers` repository. This fragmentation led to:

- Slightly different versions of hotfixes across boards.
- Debugging nightmare when using PCAN, where symbol files could differ between boards.
- New members not understanding how to use submodules

By transitioning to a monorepo, we eliminate this fragmentation, ensuring a single source of truth for all driver code.

### Simplified Car Flashing

With individual repositories, flashing the entire car was complex and error-prone. The monorepo provides:

- A unified process for flashing all boards.
- Consistency in the version of code deployed across the car.

### Alignment with Industry Practices

Adopting a monorepo mirrors the practices of many modern engineering teams, enhancing collaboration, maintainability, and scalability.

### CI-CD

With monorepo we can implement Github Actions, which allow simple build tests and checks to have a weak guarantee that code will work.

---
