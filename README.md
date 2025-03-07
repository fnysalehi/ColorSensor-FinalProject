# Color Sensor Project

A color detection system implemented on ATmega16A microcontroller using CodeVisionAVR.

## Hardware Specifications

- Microcontroller: ATmega16A
- Clock Frequency: 1MHz
- LCD Display: Alphanumeric LCD (4-bit mode)
- Memory Usage:
  - RAM: 25 bytes for global variables
  - Flash: 2190 bytes (13.4% of total)
  - Data Stack: 256 bytes
  - Hardware Stack: 743 bytes

## Features

- Real-time color detection (RED, GREEN, BLUE, CLEAR)
- LCD display output
- Timer-based sampling
- Interrupt-driven operation

## Pin Configuration

LCD Connection:
- RS: PORT0.0
- RD: PORT0.1
- EN: PORT0.2
- DATA4-7: PORT0.4-7

## Building and Flashing

This project is built using CodeVisionAVR with the following settings:
- Optimization Level: Size (-Os)
- Memory Model: Small
- Printf Features: int, width
- Scanf Features: int, width

## Project Structure

```
.
├── src/
│   └── color_sensor.c    # Main source code
├── include/
│   └── color_sensor.h    # Header files

```
