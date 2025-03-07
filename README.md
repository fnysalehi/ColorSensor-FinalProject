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

Color Sensor Connection:
- S2: PORTB.2
- S3: PORTB.4
- Output: Connected to interrupt pin

Color Filter Selection:
| S2 | S3 | Filter Type |
|----|----| ------------|
| 0  | 0  | RED        |
| 1  | 1  | GREEN      |
| 0  | 1  | BLUE       |
| 1  | 0  | CLEAR      |

## Building

### Using CodeVisionAVR
1. Open project in CodeVisionAVR
2. Set the following configuration:
   - Optimization Level: Size (-Os)
   - Memory Model: Small
   - Printf Features: int, width
   - Scanf Features: int, width
3. Build project

## Development

This project has migrated from CodeVisionAVR to a standard Makefile-based build system for better portability and maintainability.

## Project Structure

```
.
├── src/
│   └── color_sensor.c    # Main source code
└── include/
    └── color_sensor.h    # Header files

```
