# athena_motor_interface

This package contains custom message and service definitions used by the `athena_motor_driver` to communicate status and commands.

## Interfaces

### Messages

| Message Type           | Description                                                                                         |
| ---------------------- | --------------------------------------------------------------------------------------------------- |
| `FullMotorStatus`      | Aggregated status for the drive system, typically containing status for both left and right motors. |
| `MotorStatus`          | Status information for a single motor, including position, velocity, and torque/current.            |
| `TorqueCommand`        | Command message for sending direct torque requests to the left and right motors.                    |
| `DebugData`            | Container for various debug information from the microcontroller.                                   |
| `PIDDebugData`         | Detailed state of the PID controllers (error, terms, output) for tuning validation.                 |
| `MotorStatusDebugData` | Extended motor status information for debugging purposes.                                           |
