# Can_communication_sim
Simulation of the CAN communication protocol on automotive ECUs using different STM32 microcontrollers.

## Project Overview
In the modern automotive industry, vehicles have evolved into sophisticated electronic systems requiring deterministic, high-speed, and noise-resistant communication networks[cite: 1]. This project simulates a distributed automotive safety control and monitoring system using a Controller Area Network (CAN) 2.0A at 500kbps[cite: 1]. It features three independent Electronic Control Units (ECUs) using the STM32 microcontroller family to simulate two core safety systems: the Anti-lock Braking System (ABS) and the Airbag system[cite: 1].

## Hardware & Technical Stack
*   **Microcontrollers:** STM32F446RE (Node Main), STM32F407VG (Node Airbag), STM32F103C8T6 (Node ABS)[cite: 1].
*   **Protocol:** High-speed CAN 2.0A at 500 kbps[cite: 1].
*   **Architecture:** Event-driven Bare-metal programming, bypassing complex AUTOSAR layers to optimize latency[cite: 1].
*   **Safety Standards:** ISO 26262 (ASIL D) and MISRA C 2012 compliance[cite: 1].

## My Core Contributions
In this system, I took charge of designing the CAN communication architecture and developing the safety-critical Airbag node.

### 1. Airbag ECU Firmware Development
*   Developed an independent control algorithm that reads the LIS3DSH accelerometer to calculate the 3D resultant collision force[cite: 1].
*   Implemented a strict 2.5g collision threshold to accurately differentiate between real impacts and normal driving vibrations[cite: 1].

### 2. CAN Data Transmission & Arbitration Logic
*   Designed the CAN 11-bit ID priority scheme where the Airbag node holds the highest priority for emergency signals.
*   Leveraged CAN's bitwise arbitration to ensure collision data instantly overrides standard traffic without data loss.

### 3. System Debugging & Validation
*   Achieved an emergency data stream response time of approximately 10ms, successfully meeting the strict 15ms fault-tolerant time interval (FTTI) required by the ISO 26262 ASIL D standard[cite: 1].
*   Optimized the source code to adhere to MISRA C safety guidelines, preventing memory leaks and undefined behaviors[cite: 1].

## System Architecture & Data Flow
*   **Node Airbag:** Continuously processes sensor data and issues emergency deployment signals upon collision detection[cite: 1].
*   **Node ABS:** Measures wheel speed via a motor encoder using a hardware Timer, and executes emergency braking commands[cite: 1].
*   **Node Main:** Acts as the central data collection point to monitor driver operations and coordinate the network's emergency data streams[cite: 1].

## How to Build and Flash
1. Open the respective projects (`/Airbag_ECU`, `/Main_ECU`, `/ABS_ECU`) using STM32CubeIDE.
2. Compile and flash the firmware to each STM32 board.
3. Connect the CAN_TX and CAN_RX pins of the 3 boards to the CAN bus via the SN65HVD230 CAN Transceivers.
