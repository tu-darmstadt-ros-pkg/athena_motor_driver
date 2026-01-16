# Athena Motor Driver

This repository contains the drivers and firmware for the [Athena](https://github.com/tu-darmstadt-ros-pkg/athena) robot's motor controller.

## Repository Structure

- **athena_motor_driver**: ROS 2 driver node that communicates with the motor controller hardware.
- **athena_motor_firmware**: PlatformIO project containing the firmware for the Teensy-based motor controller.
- **athena_motor_interface**: ROS 2 package containing custom message and service definitions used by the driver.

## Getting Started

### Prerequisites

- ROS 2 (Humble or newer recommended)
- `libserial-dev`

### Installation

1.  Clone this repository into your ROS 2 workspace `src` directory.
2.  Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  Build the workspace:
    ```bash
    colcon build
    ```

## Usage

To launch the driver with default configuration:

```bash
ros2 launch athena_motor_driver athena_motor_driver_launch.yml
```
