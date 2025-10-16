# Electric Vehicle Embedded Control System
[![PlatformIO Build](https://github.com/James712346/EGH456Project/actions/workflows/platformio.yml/badge.svg)](https://github.com/James712346/EGH456Project/actions/workflows/platformio.yml)
[![.github/workflows/doxygen.yml](https://github.com/James712346/EGH456Project/actions/workflows/doxygen.yml/badge.svg)](https://github.com/James712346/EGH456Project/actions/workflows/doxygen.yml)
## Overview

This project is part of the **EGH456 Assignment** for **Semester 1, 2025**. It involves the design and implementation of real-time embedded software for a battery-electric vehicle (BEV). The goal is to ensure safe and efficient control of a 3-phase Brushless DC (BLDC) motor, including system monitoring and user interaction, using the **Tiva TM4C1294NCPDT** microcontroller.

## Setup Enviroments
### Windows
Usally the path is `/ti/TivaWare_C_Series-2.2.0.295/` for windows.
Run the setTilibEnv.bat file as administrator, and follow the instructions 

### Linux
Add the below line to your `.bashrc` file
```bash
TILIB=#Your Path to TILIB
```
## Project Objectives

- Monitor the state of a BLDC motor including rotational speed and power.
- Implement safe startup, braking, and emergency shutdown procedures.
- Design real-time multitasking behavior using FreeRTOS.
- Acquire and filter sensor data.
- Handle motor fault detection and recovery.
- Provide a user interface to display critical system status.

## Hardware Used

- **Tiva TM4C1294NCPDT** microcontroller development kit.
- Motor testing kit including:
  - 3-phase BLDC motor
  - Motor driver board
  - Sensor board
- Electrical interface board for signal compatibility.

## Software Features

- Real-time task scheduling via FreeRTOS.
- UART-based serial communication for system monitoring.
- Motor control logic for various states (start, stop, fault).
- Sensor input filtering and signal processing.
- Modular and portable C code with structured Doxygen documentation.

## Directory Structure
[Docs](https://james712346.github.io/EGH456Project)

