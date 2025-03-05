#ifndef ATHENA_MOTOR_INTERFACES_H
#define ATHENA_MOTOR_INTERFACES_H

#include <cstddef>
#include <cstdint>
#include <endian.h>
#include <vector>

const int BAUD_RATE = 115200;
const char START_BYTE = 0x42;

enum class CommandType : uint8_t {
  NONE = 0,
  MOTOR_COMMAND = 0b11011,
  MOTOR_STATUS = 0b00101,
  MOTOR_ACK = 0b01010,
  MOTOR_ERROR = 0b10110
};

struct MotorCommand {
  float left_velocity;
  float right_velocity;
  bool valid = false;

  static constexpr size_t get_size() { return sizeof( float ) * 2 + 1; }

  void _serialize( uint8_t *buffer ) const;

  static MotorCommand deserialize( const uint8_t *buffer );
};

void serialize( const MotorCommand &command, std::vector<uint8_t> &buffer, int offset = 0 );

struct MotorStatus {
  bool valid = false;
  enum Mode : uint8_t { INVALID, BRAKE, FOC, CALIBRATE } mode;
  int8_t temperature;
  uint8_t error_code;

  float torque;
  float velocity;
  float position;
  float acceleration;

  static constexpr size_t get_size() { return 4 + 4 * sizeof( float ) + 1; }

  void _serialize( uint8_t *buffer ) const;

  static MotorStatus deserialize( const uint8_t *buffer );
};

struct FullMotorStatus {
  MotorStatus front_left;
  MotorStatus front_right;
  MotorStatus rear_left;
  MotorStatus rear_right;

  static constexpr size_t get_size() { return 4 * MotorStatus::get_size(); }

  void _serialize( uint8_t *buffer ) const;

  static FullMotorStatus deserialize( const uint8_t *buffer );
};

void serialize( const FullMotorStatus &command, std::vector<uint8_t> &buffer, int offset = 0 );

// Implementation

namespace impl
{
void writeHeader( uint8_t *&buffer, CommandType type );

uint8_t crc8( const uint8_t *buffer, int count );

void serialize( float value, uint8_t *buffer );
void deserialize( const uint8_t *buffer, float &value );
} // namespace impl

inline void MotorCommand::_serialize( uint8_t *buffer ) const
{
  uint8_t *start = buffer;
  impl::serialize( left_velocity, buffer );
  buffer += sizeof( float );
  impl::serialize( right_velocity, buffer );
  buffer += sizeof( float );
  *buffer = impl::crc8( start, get_size() - 1 );
}

inline MotorCommand MotorCommand::deserialize( const uint8_t *buffer )
{
  uint8_t crc8 = impl::crc8( buffer, get_size() - 1 );
  if ( crc8 != buffer[2 * sizeof( float )] ) {
    return {};
  }
  MotorCommand command;
  impl::deserialize( buffer, command.left_velocity );
  buffer += sizeof( float );
  impl::deserialize( buffer, command.right_velocity );
  command.valid = true;
  return command;
}

inline void serialize( const MotorCommand &command, std::vector<uint8_t> &buffer, int offset )
{
  buffer.resize( offset + 2 + MotorCommand::get_size() );
  auto *data = buffer.data() + offset;
  impl::writeHeader( data, CommandType::MOTOR_COMMAND );
  command._serialize( data );
}

inline void MotorStatus::_serialize( uint8_t *buffer ) const
{
  uint8_t *start = buffer;
  *buffer = valid ? 1 : 0;
  buffer++;
  *buffer = static_cast<uint8_t>( mode );
  buffer++;
  *buffer = temperature;
  buffer++;
  *buffer = error_code;
  buffer++;
  impl::serialize( torque, buffer );
  buffer += sizeof( float );
  impl::serialize( velocity, buffer );
  buffer += sizeof( float );
  impl::serialize( position, buffer );
  buffer += sizeof( float );
  impl::serialize( acceleration, buffer );
  buffer += sizeof( float );
  *buffer = impl::crc8( start, get_size() - 1 );
}

inline MotorStatus MotorStatus::deserialize( const uint8_t *buffer )
{
  uint8_t crc8 = impl::crc8( buffer, get_size() - 1 );
  if ( crc8 != buffer[get_size() - 1] ) {
    return {};
  }
  MotorStatus status;
  status.valid = *buffer == 1;
  buffer++;
  status.mode = static_cast<Mode>( *buffer );
  buffer++;
  status.temperature = *buffer;
  buffer++;
  status.error_code = *buffer;
  buffer++;
  impl::deserialize( buffer, status.torque );
  buffer += sizeof( float );
  impl::deserialize( buffer, status.velocity );
  buffer += sizeof( float );
  impl::deserialize( buffer, status.position );
  buffer += sizeof( float );
  impl::deserialize( buffer, status.acceleration );
  return status;
}

inline void FullMotorStatus::_serialize( uint8_t *buffer ) const
{
  front_left._serialize( buffer );
  buffer += MotorStatus::get_size();
  front_right._serialize( buffer );
  buffer += MotorStatus::get_size();
  rear_left._serialize( buffer );
  buffer += MotorStatus::get_size();
  rear_right._serialize( buffer );
}

inline FullMotorStatus FullMotorStatus::deserialize( const uint8_t *buffer )
{
  FullMotorStatus status;
  status.front_left = MotorStatus::deserialize( buffer );
  buffer += MotorStatus::get_size();
  status.front_right = MotorStatus::deserialize( buffer );
  buffer += MotorStatus::get_size();
  status.rear_left = MotorStatus::deserialize( buffer );
  buffer += MotorStatus::get_size();
  status.rear_right = MotorStatus::deserialize( buffer );
  return status;
}

inline void serialize( const FullMotorStatus &status, std::vector<uint8_t> &buffer, int offset )
{
  buffer.resize( offset + 2 + FullMotorStatus::get_size() );
  auto *data = buffer.data() + offset;
  impl::writeHeader( data, CommandType::MOTOR_STATUS );
  status._serialize( data );
}

namespace impl
{
inline void writeHeader( uint8_t *&buffer, CommandType type )
{
  *buffer = START_BYTE;
  buffer++;
  *buffer = static_cast<uint8_t>( type );
  buffer++;
}

inline void serialize( float value, uint8_t *buffer )
{
  auto data = reinterpret_cast<uint32_t *>( buffer );
  union {
    float value;
    uint32_t data;
  } tmp;
  tmp.value = value;
  *data = htole32( tmp.data );
}

inline void deserialize( const uint8_t *buffer, float &value )
{
  auto data = reinterpret_cast<const uint32_t *>( buffer );
  union {
    float value;
    uint32_t data;
  } tmp;
  tmp.data = le32toh( *data );
  value = tmp.value;
}

inline uint8_t crc8( const uint8_t *buffer, int count )
{
  static const unsigned char CRC_TABLE[] = {
      0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01,
      0x96, 0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95,
      0xBB, 0x2C, 0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B,
      0xCC, 0xE2, 0x75, 0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A,
      0xE1, 0x76, 0x58, 0xCF, 0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5,
      0x22, 0xE9, 0x7E, 0x50, 0xC7, 0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21,
      0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D, 0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF,
      0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24, 0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3,
      0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E, 0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE,
      0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34, 0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A,
      0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E, 0x41, 0xD6, 0xF8, 0x6F, 0xA4,
      0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7, 0xFB, 0x6C, 0x42, 0xD5,
      0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D, 0xF3, 0x64, 0x4A,
      0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65, 0x49, 0xDE,
      0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF, 0x10,
      0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
      0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB,
      0x3C };
  uint8_t crc = 0;
  while ( count-- ) {
    crc = crc ^ *buffer++;       // Apply Byte
    crc = CRC_TABLE[crc & 0xFF]; // One round of 8-bits
  }

  return crc;
}
} // namespace impl

#endif // ATHENA_MOTOR_INTERFACES_H
