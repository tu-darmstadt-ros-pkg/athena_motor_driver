#include <Arduino.h>
#include "../../athena_motor_interface/include/athena_motor_interface/athena_motor_interfaces.h"
#include <vector>

// Handles communication with the host computer
class HostComm {
public:
  HostComm();

  void init();

  //! Checks if a command is available. Does not block.
  bool hasCommand();

  CommandType getType() { return type_; }

  MotorCommand getMotorCommand() { return motor_command_; }

  void acknowledgeCommand(CommandType type);

  void sendStatus( const FullMotorStatus &status );

private:
  std::vector<char> buffer_;
  CommandType receiving_type_;

  MotorCommand motor_command_;
  CommandType type_;
};