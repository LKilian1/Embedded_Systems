# STM32 Position Determination with MEMS Sensors
###Project Overview
This project focuses on creating a system for relative position determination using MEMS (Micro-Electro-Mechanical Systems) sensors. By integrating acceleration and magnetic field measurements, we aim to monitor and analyze physical parameters with modern technology.

###Components
STM32 Nucleo-144 Development Board: The core of our project, offering a robust platform for real-time sensor data processing.
Magnetometer (LIS2MDL) and Accelerometer (LIS2DW12): Essential sensors for capturing magnetic orientations and acceleration values.
I2C Communication: Ensures efficient data transfer between the STM32 board and the sensors.
Setup and Configuration
Hardware Setup
Follow the schematics in the Embedded_Systems_new.pdf to connect the sensors to the STM32 Nucleo-144 board.

###Software Requirements
STM32CubeIDE and STM32CubeMX: For project configuration and development.
MEMS Software Packages: Includes libraries for sensor integration.
Implementation
Data Acquisition
Sensor data is acquired through I2C, processed, and used to determine the device's relative position.

###Position Determination
Combines sensor data using a Kalman Filter algorithm for accurate position estimation.

###Challenges and Solutions
Software and hardware integration challenges were addressed through detailed configuration and testing.
A Kalman Filter approach was adopted to compensate for sensor inaccuracies.
Future Directions
Enhancements will focus on improving position accuracy through additional sensor integration and algorithm optimization.

###Appendix
Includes detailed information on the workstation setup, interface functions, sensor configuration registers, and a low-level function for register configuration.

###References
Refer to Embedded_Systems_new.pdf for a comprehensive literature review and appendix material.



