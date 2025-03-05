#include "host_comm.h"
#include <usb_serial.h>

HostComm::HostComm() = default;

void HostComm::init() {
  Serial.begin( -1 );
}

bool HostComm::hasCommand()
{
  if (receiving_type_ == CommandType::NONE) {
    while ( Serial.available() > 0 && Serial.peek() != START_BYTE ) {
      // Skip bytes until the start byte is found
      Serial.read();
    }
    if ( Serial.available() < 2 ) {
      // Not enough bytes to read the type
      return false;
    }
    Serial.read(); // Discard the start byte
    receiving_type_ = CommandType(Serial.read());
  }

  // Determine if we have enough bytes to read the command
  int bytes_needed = 0;
  switch ( receiving_type_ ) {
  case CommandType::MOTOR_COMMAND:
    bytes_needed = MotorCommand::get_size();
    break;
  default:
    // Ignore command
    receiving_type_ = CommandType::NONE;
    return false;
  }
  if ( Serial.available() < bytes_needed ) {
    return false;
  }

  type_ = receiving_type_;
  receiving_type_ = CommandType::NONE; // Reset the type, so we can get the next command
  // Once we have enough bytes, read the command
  switch ( type_ ) {
  case CommandType::MOTOR_COMMAND: {
    buffer_.resize( MotorCommand::get_size() );
    Serial.readBytes( buffer_.data(), MotorCommand::get_size() );
    motor_command_ =
        MotorCommand::deserialize( reinterpret_cast<const uint8_t *>( buffer_.data() ) );
    if ( !motor_command_.valid ) {
      // Ignore commands if they did not pass validity checks
      type_ = CommandType::NONE;
      return false;
    }
    return true;
  }
  default:
    // TODO Report error
    break;
  }
  return false;
}

void HostComm::sendStatus( const FullMotorStatus &status )
{
  std::vector<uint8_t> buffer;
  serialize( status, buffer );
  Serial.write( buffer.data(), buffer.size() );
}
