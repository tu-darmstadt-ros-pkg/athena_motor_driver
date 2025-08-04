uint8 MODE_INVALID = 0
uint8 MODE_BRAKE = 1
uint8 MODE_FOC = 2
uint8 MODE_CALIBRATE = 3

uint8 NO_ERROR = 0

bool valid
uint8 mode
int8 temperature
uint8 error_code

float32 target_velocity
float32 target_torque

float32 torque
# Velocity in rad/s
# High and low speed are used by the Unitree motors, but it is not clear what the difference is.
float32 velocity_high
float32 velocity_low
float32 position
float32 acceleration

builtin_interfaces/Time stamp
