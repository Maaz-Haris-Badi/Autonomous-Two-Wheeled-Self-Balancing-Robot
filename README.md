# Autonomous Two-Wheeled Self-Balancing Robot: Microcontroller and Interfacing Project

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![Platform](https://img.shields.io/badge/platform-STM32-blue)
![Architecture](https://img.shields.io/badge/architecture-ARM%20Cortex--M4-orange)
![Domain](https://img.shields.io/badge/domain-Sensor%20Fusion%20%7C%20Embedded%20Systems-purple)

## Abstract
The **Autonomous Two-Wheeled Self-Balancing Robot** is a comprehensive course project developing an embedded software architecture on the **STM32F303** (ARM Cortex-M4) to investigate real-time sensor fusion, hardware-level data acquisition, and constrained environment optimizations. 

Designed as a cohesive project built through sequential implementation modules, this robot demonstrates how low-level microcontroller primitives (interrupts, direct memory access, hardware timers) can be orchestrated to build a robust foundation for spatial orientation estimation (Angle Estimation). The project serves as a practical application for integrating MEMS sensors and serves as a precursor to more complex applications in autonomous robotics, drones, and wearable IoT devices.

## 🔬 Technical Objectives
In embedded systems, predictable timing, low latency, and efficient memory utilization are paramount. This project systematically tackles these challenges by avoiding high-overhead operating systems in favor of bare-metal and HAL-driven architectures. 

Key engineering objectives achieved in this project include:
* **Deterministic Data Acquisition:** Transitioning from CPU-intensive polling to asynchronous, interrupt-driven paradigms to minimize jitter in sensor sampling rates.
* **Algorithmic Sensor Fusion:** Implementing real-time angle estimation algorithms (combining accelerometer and gyroscope data) while leveraging the Cortex-M4's DSP instructions and hardware Floating Point Unit (FPU) to accelerate mathematical computations.
* **Protocol Abstraction & Bus Management:** Developing robust, low-level drivers for I2C, SPI, and UART to interface with complex external ICs (e.g., 6-DoF/9-DoF IMUs) while managing bus arbitration.
* **Mixed-Signal Integrity:** Characterizing analog-to-digital conversions (ADC), signal conditioning, and noise mitigation in raw sensor data prior to digital filtering.

## 🛠️ System Architecture

### Hardware Platform
* **Processing Core:** ARM 32-bit Cortex-M4 CPU with FPU (Floating Point Unit) operating at 72 MHz.
* **Target MCU:** STM32F303VCT6 Discovery/Nucleo (or equivalent).
* **Hardware Interfacing:** Direct manipulation of Timers, ADC, GPIO, USART/UART, I2C, and SPI peripherals.

### Software Stack
* **Language:** Embedded C (MISRA-inspired strictness for predictable execution)
* **Toolchain/Build System:** GCC ARM Embedded, CMake, Ninja, STM32CubeIDE/VS Code
* **Hardware Abstraction Layer (HAL):** STM32 HAL / CMSIS core registers.

## 📂 Project Development Modules

The architecture of the robot was constructed incrementally. The codebase is organized into sequential modules (originally developed as laboratory phases), each building upon the previous to form the final sensor fusion engine:

### Phase I: Deterministic Execution and Clocking (Modules 1-4)
* **System Clock & GPIO:** Established the system tick mechanism and fundamental I/O toggling, analyzing electrical characteristics via oscilloscope.
* **Event-Driven Architectures:** Engineered precise Interrupt Service Routines (ISRs) to decouple execution from the main loop, reducing power consumption.
* **Actuation & Timing:** Developed high-resolution PWM signals and precise hardware delays necessary for motor control and synchronization.

### Phase II: Signal Acquisition and Conditioning (Modules 5-7)
* **Continuous ADC Pipelines:** Configured high-speed, triggered Analog-to-Digital conversions to sample dynamic environmental changes.
* **Noise Mitigation:** Implemented raw data calibration and moving-average/low-pass filtering to condition analog signals before algorithmic processing.

### Phase III: High-Speed Digital Interfacing (Modules 8-10)
* **Telemetry via USART:** Established reliable serial communication for real-time data logging and host-PC visualization.
* **I2C & SPI Bus Integration:** Successfully interfaced with external digital sensors (e.g., MEMS accelerometers, EEPROMs) by writing custom register-level read/write transaction handlers.

### Phase IV: The Fusion Engine (`angle_estimation`)
* **IMU Data Harvesting:** Synchronous extraction of multi-axis acceleration and angular velocity data.
* **Spatial Estimation Algorithm:** Developed a continuous loop that fuses incoming IMU data using trigonometric algorithms to estimate roll, pitch, and yaw in real-time. 
* **FPU Acceleration:** Demonstrated a significant reduction in computation time by offloading floating-point math to the dedicated hardware FPU, freeing the ALU for concurrent tasks.

## 🚀 Build & Deployment

This project utilizes modern cross-compilation tools (`CMake` and `Ninja`) for reproducibility and CI/CD readiness.

```bash
# 1. Clone the repository
git clone https://github.com/yourusername/MCI-COURSE.git
cd MCI-COURSE/angle_estimation # Navigate to the core fusion engine

# 2. Configure the build environment
cmake --preset debug # Initializes GCC ARM toolchain

# 3. Compile the Firmware
cmake --build build

# 4. Flash to Target
# Flash the generated .elf or .bin file using STM32CubeProgrammer or OpenOCD via SWD.
```

## 📈 Future Enhancements
This project lays the groundwork for advanced applications in embedded control:
1. **RTOS Integration:** Migrating the bare-metal architecture to FreeRTOS to analyze the overhead of task scheduling, mutexes, and semaphore-based resource sharing on latency.
2. **Extended Kalman Filter (EKF):** Upgrading the current fusion algorithm to a full state-estimation EKF to account for non-linearities and sensor drift over time.
3. **Power-State Optimization:** Instrumenting the codebase to actively switch the MCU between active and deep-sleep modes, mathematically profiling the energy footprint for wearable applications.

## ✉️ Author & Contact
This architecture was engineered to explore the theoretical and practical intersections of microcontroller hardware and control theory. For inquiries regarding the technical implementation or hardware challenges, please reach out.

---
*Developed as part of the Microcontrollers and Interfacing (MCI) coursework at Habib University.*
