#include "motor_msg_A1B1.h"
#include <HardwareSerial.h>
#include <vector>

enum class MotorMode : uint8_t { INVALID, BRAKE, FOC, CALIBRATE };

struct MotorCommCommand {
  uint8_t motor_id = 0;
  MotorMode mode = MotorMode::BRAKE;

  float torque = 0;
  float velocity = 0;
  float position = 0;

  //! Join stiffness coefficient
  //! When transmitting, this is multiplied by 2048 and transmitted as integer
  float k_p = 0;
  //! Join stiffness coefficient
  //! When transmitting, this is multiplied by 2048 and transmitted as integer
  float k_w = 0;
};

struct MotorCommStatus {
  bool valid = false;
  uint8_t motor_id = 0;
  MotorMode mode;
  int temperature;
  int error_code;

  float torque;
  float velocity;
  float position;
  float acceleration;
};

// Only tested for A1, B1Motor is called different to avoid conflict with Binary.h
enum class MotorType { A1, B1Motor };

class MotorComm
{
public:
  MotorComm( HardwareSerialIMXRT *serial, int direction_pin, MotorType type = MotorType::A1 );

  void sendReceive( const MotorCommCommand &command, MotorCommStatus &status );
  void sendReceive( const std::vector<MotorCommCommand> &commands, std::vector<MotorCommStatus> &statuses );

private:
  MotorCommStatus readStatus();
  void writeData( const uint8_t *data, size_t size );

  HardwareSerialIMXRT *serial_;
  int direction_pin_;
  MotorType type_;

  uint8_t send_buffer_[3 * 34];
  uint8_t receive_buffer_[256];
};