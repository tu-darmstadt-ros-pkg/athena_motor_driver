#include "motor_comm.h"
#include "crc32.h"

#include <Arduino.h>

constexpr int getBaudRate( MotorType type )
{
  // These baudrates compensate that the cycles are calculated for 24MHz UART clock but we changed this to 80MHz
  switch ( type ) {
  case MotorType::A1Motor:
    return 4800000;
  case MotorType::B1Motor:
    return 6000000;
  case MotorType::GO_M8010_6_Motor:
    return 4000000;
  default:
    return 0;
  }
}

constexpr float getGearRatio( MotorType type )
{
  switch ( type ) {
  case MotorType::A1Motor:
    return 9.100000381469727;
  case MotorType::B1Motor:
    return 8.65999984741211;
  case MotorType::GO_M8010_6_Motor:
    return 6.329999923706055;
  }
  return 1;
}

uint8_t getMotorModeCode( MotorType type, MotorMode mode )
{
  switch ( mode ) {
  case MotorMode::BRAKE:
    return 0;
  case MotorMode::FOC:
    return ( type == MotorType::A1Motor || type == MotorType::B1Motor ) ? 10 : 1;
  case MotorMode::CALIBRATE:
    return ( type == MotorType::A1Motor || type == MotorType::B1Motor ) ? 4 : 2;
  case MotorMode::INVALID:
  default:
    return 0;
  }
  return 0;
}

MotorMode getMotorMode( MotorType type, uint8_t code )
{
  switch ( type ) {
  case MotorType::A1Motor:
  case MotorType::B1Motor:
    switch ( code ) {
    case 0:
      return MotorMode::BRAKE;
    case 10:
      return MotorMode::FOC;
    case 4:
      return MotorMode::CALIBRATE;
    }
    break;
  case MotorType::GO_M8010_6_Motor:
    switch ( code ) {
    case 0:
      return MotorMode::BRAKE;
    case 1:
      return MotorMode::FOC;
    case 2:
      return MotorMode::CALIBRATE;
    }
    break;
  }
  return MotorMode::INVALID;
}

MotorComm::MotorComm( HardwareSerialIMXRT *serial, int direction_pin, MotorType type )
    : serial_( serial ), direction_pin_( direction_pin ), type_( type )
{
  serial_->transmitterEnable( direction_pin );
  serial_->addMemoryForWrite( send_buffer_, sizeof( send_buffer_ ) );
  serial_->addMemoryForRead( receive_buffer_, sizeof( receive_buffer_ ) );
  serial_->begin( getBaudRate( type ), SERIAL_8N1 );
}

void MotorComm::sendReceive( const MotorCommCommand &command, MotorCommStatus &status )
{
  MasterComdDataV3 data;
  memset( &data, 0, sizeof( data ) );
  data.head.start[0] = 0xFE;
  data.head.start[1] = 0xEE;
  data.head.motorID = command.motor_id;
  data.Mdata.mode = getMotorModeCode( type_, command.mode );
  data.Mdata.ModifyBit = 0xFF;
  const float gear_ratio = getGearRatio( type_ );
  data.Mdata.T = static_cast<int16_t>( 256 * command.torque / gear_ratio );
  data.Mdata.W = static_cast<int16_t>( 128 * command.velocity * gear_ratio );
  data.Mdata.Pos =
      command.position * ( 16384.0f / 6.2831853f ) * gear_ratio; // 14-bit encoder, 6.2832 = 2*PI
  // Divide kp and kd by gear ratio squared to get the values at the rotor
  // Dividing by 26.7 and multiplying
  data.Mdata.K_P = static_cast<int16_t>( command.k_p / ( gear_ratio * gear_ratio ) / 26.7 * 2048 );
  data.Mdata.K_W = static_cast<int16_t>( command.k_w / ( gear_ratio * gear_ratio ) * 100.0 * 1024 );
  data.Mdata.Res[0].u32 = 256;
  data.CRCdata.u32 = crc32_core( (uint32_t *)&data, sizeof( data ) / sizeof( uint32_t ) - 1 );
  serial_->clear();
  writeData( (uint8_t *)&data, sizeof( MasterComdDataV3 ) );
  status = readStatus();
}

void MotorComm::sendReceive( const MotorCommCommand &left_command,
                             const MotorCommCommand &right_command, MotorCommStatus &left_status,
                             MotorCommStatus &right_status )
{
  sendReceive( left_command, left_status );
  delayMicroseconds( 100 ); // Wait a bit to avoid collisions
  sendReceive( right_command, right_status );
  delayMicroseconds( 100 ); // Wait a bit to avoid collisions
}

MotorCommStatus MotorComm::readStatus()
{
  MotorCommStatus result;
  result.valid = false;
  status_buffer_[0] = 0xFE;
  status_buffer_[1] = 0xEE;
  elapsedMicros time;
  while ( serial_->available() < 2 || serial_->read() != 0xFE || serial_->peek() != 0xEE ) {
    if ( time > 200 ) {
      return result;
    }
  }
  while ( serial_->available() < 77 ) {
    if ( time > 400 )
      return result;
  }
  serial_->readBytes( status_buffer_ + 1, 77 );
  serial_->clear();
  uint32_t crc = crc32_core( (uint32_t *)status_buffer_, 74 / sizeof( uint32_t ) );
  ServoComdDataV3 *status = (ServoComdDataV3 *)status_buffer_;
  if ( crc != status->CRCdata.u32 ) {
    return result;
  }

  result.valid = true;
  result.motor_id = status->head.motorID;
  result.mode = getMotorMode( type_, status->Mdata.mode );
  result.temperature = status->Mdata.Temp;
  result.error_code = status->Mdata.MError;
  result.torque = status->Mdata.T / 256.0f * getGearRatio( type_ );
  result.velocity_high = status->Mdata.W / 128.0f / getGearRatio( type_ );
  result.velocity_low = status->Mdata.LW / getGearRatio( type_ );
  result.position = status->Mdata.Pos * ( 6.2831853f / 16384.0f ) / getGearRatio( type_ );
  result.acceleration = status->Mdata.Acc;
  return result;
}

void MotorComm::writeData( const uint8_t *data, size_t size )
{
  serial_->clearWriteError();
  while ( serial_->availableForWrite() < static_cast<int>( size ) ) delayMicroseconds( 5 );
  serial_->write( data, size );
  serial_->flush();
}
