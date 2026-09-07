# Can_communication_sim
This project is a design simulation of the CAN communication protocol with automotive safety control and monitoring system on ECUs using different STM32 microcontrollers.

## Project Overview
In the modern automotive industry, vehicles have evolved into sophisticated electronic systems requiring deterministic, high-speed, and noise-resistant communication networks. This project simulates a distributed automotive safety control and monitoring system using a Controller Area Network (CAN) 2.0A at 500kbps. It features three independent Electronic Control Units (ECUs) using the STM32 microcontroller family to simulate two core safety systems: the Anti-lock Braking System (ABS) and the Airbag system.

## Hardware & Technical Stack
*   **Microcontrollers:** STM32F446RE NUCLEO (Node Main), STM32F407-DISC1 (Node Airbag), STM32F103C8T6 (Node ABS).
*   **Protocols:** High-speed CAN 2.0A at 500 kbps, SPI, I2C, USART.
*   **Architecture:** Event-driven Bare-metal programming, bypassing complex AUTOSAR layers to optimize latency.
*   **Safety Standards:** ISO 26262 (ASIL D) and MISRA C 2012 compliance.

## Core Contributions
In this system, I took charge of designing the CAN communication architecture and developing the safety-critical Airbag node.

### 1. Airbag ECU Firmware Development
*   Developed an independent control algorithm that reads the LIS3DSH accelerometer to calculate the 3D resultant collision force.
*   Implemented a strict G-force collision threshold to accurately differentiate between real impacts and normal driving vibrations.

### 2. CAN Data Transmission & Arbitration Logic
*   Designed the CAN 11-bit ID priority scheme where the Airbag node holds the highest priority for emergency signals.
*   Leveraged CAN's bitwise arbitration to ensure collision data instantly overrides standard traffic without data loss.

### 3. System Debugging & Validation
*   Achieved an emergency data stream response time of approximately 10ms, successfully meeting the strict 15ms fault-tolerant time interval (FTTI) required by the ISO 26262 ASIL D standard.
*   Optimized the source code to adhere to MISRA C safety guidelines, preventing memory leaks and undefined behaviors.

## System Architecture & Data Flow
*   **Node Airbag:** Continuously processes sensor data and issues emergency deployment signals upon collision detection.
*   **Node ABS:** Measures wheel speed via a motor encoder using a hardware Timer, and executes emergency braking commands.
*   **Node Main:** Acts as the central data collection point to monitor driver operations and coordinate the network's emergency data streams.

## How to Build, Flash, and Verify
1. Open the respective projects (`/F407_AIRBAG_2`, `/F446_MAIN_2`, `/F103_ABS`) using STM32CubeIDE.
2. Compile and flash the firmware to each STM32 board.
3. On the hardware, connect the CAN_TX and CAN_RX pins of the 3 boards to the CAN bus via the SN65HVD230 CAN Transceivers. In addition, connect an external DC motor to the STM32F103C8T6 board through TIMER peripherals a supply it with a 12V DC source. On the STM32F446RE Nucleo board, connect a potentiometer to pin PA4 to control the speed and 2 button to pin PC2 and PC3 to control the braking and reversing respectively.
4. Press the reset button on each of the STM32 boards. If you're supplying the Nucleo board from your desktop, you can open a terminal emulator such as puTTY or MobaXterm to watch the status of each packages send through USART with baud rate of 115200bps.
* Additionally, you can watch the CAN signals live using a tool like oscilloscope or a logic analyzer by connecting the meassuring channel to the CAN_RX pin of any board, you can watch the CAN frame transmitting and receiving and compare the time cost with the configurations.
