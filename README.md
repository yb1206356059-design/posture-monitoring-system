# ESP32-C3 Posture Monitoring System

This repository contains the firmware for an ESP32-C3 based wearable posture monitoring and reminder system.

The firmware implements:
- MPU6050 inertial sensor data acquisition
- posture angle estimation
- user calibration
- threshold-based poor posture detection
- vibration motor and buzzer feedback
- Wi-Fi web dashboard for live monitoring and configuration

## Hardware

Main components:
- ESP32-C3FH4 microcontroller
- MPU6050 IMU
- vibration motor
- buzzer
- LiPo battery charging circuit
- custom PCB

## Firmware

The project was developed using PlatformIO with the Arduino framework.

Main source file:

```text
src/main.cpp
