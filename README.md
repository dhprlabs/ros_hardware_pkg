# ROS Hardware Package

ROS 2 hardware interface and ESP32 firmware for a differential drive base using `ros2_control`.

## Overview

This repository contains two parts:

- `ros_hardware_plugin`: A `hardware_interface::SystemInterface` plugin that exposes wheel position and velocity state interfaces and accepts wheel velocity commands.
- `esp_code`: ESP32 firmware that runs dual motor PID speed control with encoder feedback and a serial command interface.

## Repository Structure

- `esp_code/esp_code.ino`: ESP32 firmware (motor control, PID loop, encoder reads, serial protocol).
- `ros_hardware_plugin/diffbot_system.cpp`: ROS 2 hardware plugin implementation.
- `ros_hardware_plugin/include/ros2_control_demo_example_2/diffbot_system.hpp`: plugin class definition and config.
- `ros_hardware_plugin/include/ros2_control_demo_example_2/esp_comms.hpp`: serial communication layer using LibSerial.
- `ros_hardware_plugin/include/ros2_control_demo_example_2/wheel.hpp`: wheel state and conversion helpers.

## ROS 2 Hardware Interface Behavior

The plugin:

- Loads hardware parameters: left/right wheel names, serial device, baud rate, timeout, encoder counts per revolution.
- Exports state interfaces per wheel:
  - `position` (rad)
  - `velocity` (rad/s)
- Exports command interfaces per wheel:
  - `velocity` (rad/s)
- On activate, opens serial communication to ESP32.
- On read, parses encoder ticks and computes:
  - `position_rad = ticks * rad_per_count`
  - `velocity_rad_s = (position_rad - previous_position_rad) / dt_s`
- On write, converts wheel command from rad/s to rev/s and sends `CMD_VEL <left> <right>`.

## ESP32 Firmware Behavior

The firmware implements:

- Quadrature encoder counting via interrupts.
- Dual PID motor speed control loop (50 ms period).
- PWM motor drive output with direction control.
- Safety timeout in ROS mode (motors stop if no command for 2 seconds).
- Optional continuous status streaming in ROS mode.

## Serial Command Protocol

Commands accepted by firmware in ROS mode:

- `CMD_VEL <m1_rev_s> <m2_rev_s>`: set target motor speeds.
- `START_STREAM`: start periodic status output.
- `STOP_STREAM`: stop periodic status output.
- `GET_STATUS`: send one status line.
- `STOP`: stop motors.
- `EXIT_ROS`: exit ROS mode and return to menu mode.

Typical status line:

- `STATUS <m1_target> <m1_actual> <m2_target> <m2_actual> <enc1> <enc2>`

## Prerequisites

- ROS 2 with `ros2_control`.
- `pluginlib` and hardware interface dependencies.
- `LibSerial` available to build the plugin.
- ESP32 toolchain (Arduino framework) for firmware upload.

## Integration Notes

- Keep `enc_counts_per_rev` in ROS parameters aligned with firmware `COUNTS_PER_REV`.
- Keep serial baud rate consistent on both sides.
- Wheel command units in ROS are rad/s; firmware commands are rev/s (already converted in plugin).

## Current Limitations

- `readEncoderValues` extracts the last two integers from incoming serial text; ensure encoder/status output format remains consistent.
- `on_activate` does not currently validate serial open failures with explicit error handling.
- Protocol acknowledgments are not used by the plugin for closed-loop communication validation.
