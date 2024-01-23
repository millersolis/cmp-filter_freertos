Complementary Filer - FreeRTOS
==============
This repository contains a complementary filter implementation for the **LSM6DSM** sensor using **FreeRTOS** on an **STM32 ARM M4** development board.

### File Structure
* Application: Application Layer Code
  * App: Handles data and events coming from threads (facilitates inter-thread communication logic)
  * I2C Thread: Handles I2C bus (samples sensor)
  * Main Thread: Implements Complementary Filter
* BSP: Board Support Package
  * AccGyr: Abstraction to use the Sensor IC
  * Terminal: Abstraction to use the Serial terminal
* Core: Peripherals, Clocks, misc. configurations
* Drivers:
  * CMSIS (unused)
  * Sensor driver by STM32
  * HAL: Low-level Hardware Abstraction Layer by STM32
* Middlewares: FreeRTOS
* printf-6.0.0: printf implementation for embedded
* Documentation: SW and HW diagrams, misc. docs and videos

### Demo
Oscilloscope channels 1 and 2 showing real-time values for Roll and Pitch angles as the breadboard with the LSM6DSM Breakout Board is rotated in multiple directions.

https://github.com/millersolis/cmp-filter_freertos/assets/73608859/ee602f95-498d-4aa4-9246-d598a94c1e12

### Hardware
* [STM32F429ZI MCU DevBoard: NUCLEO-F429ZI](https://www.st.com/en/evaluation-tools/nucleo-f429zi.html)
  * [Debugging Probe: STLink v2 integrated in Devboard](https://www.st.com/en/development-tools/st-link-v2.html)
* [LSM6DSM Breakout Board: STEVAL-MKI189V1](https://www.st.com/en/evaluation-tools/steval-mki189v1.html)

### Software
* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
* FreeRTOS
* [LSM6DSM Driver](https://github.com/STMicroelectronics/lsm6dsm-pid)
* [Printf for embedded](https://github.com/eyalroz/printf)
* [Serial Oscilloscope](https://x-io.co.uk/serial-oscilloscope/)
