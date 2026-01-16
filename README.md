# Athena Motor Driver

This repository contains the drivers and firmware for the [Athena](https://github.com/tu-darmstadt-ros-pkg/athena) robot's motor controller.

Athena uses Unitree A1 motors to drive the main tracks, with two per side, for a total torque of approximately 67 Nm.
The motors are torque-controlled using a Teensy 4.0 board with two RS-485 boards, one for the front motors and one for the back motors, to enable redundancy.
The Teensy takes velocity commands for the left and right tracks and controls the motors to achieve the given velocity.
In both the ROS driver on the PC and the Teensy, timeouts are implemented to stop the motors if no commands are received.

>[!NOTE]
> This library uses [Crosstalk](https://github.com/StefanFabian/crosstalk) to exchange data between PC and Teensy over serial.

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
