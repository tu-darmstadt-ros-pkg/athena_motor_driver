#include "motor_comm.h"
#include "crc32.h"

#include <Arduino.h>

int getSerialRate( MotorType type )
{
  // These baudrates compensate that the cycles are calculated for 24MHz UART clock but we changed this to 80MHz
  switch ( type ) {
  case MotorType::A1:
    return 1445000;
  case MotorType::B1Motor:
  default:
    return 1800000;
  }
}

float getGearRatio( MotorType type )
{
  switch ( type ) {
  case MotorType::A1:
    return 9.100000381469727;
  case MotorType::B1Motor:
    return 8.65999984741211;
    // case MotorType::GO_M8010_6:
    //   return 6.329999923706055;
  }
  return 1;
}

uint8_t getMotorModeCode( MotorType type, MotorMode mode )
{
  switch ( mode ) {
  case MotorMode::BRAKE:
    return 0;
  case MotorMode::FOC:
    return ( type == MotorType::A1 || type == MotorType::B1Motor ) ? 10 : 1;
  case MotorMode::CALIBRATE:
    return ( type == MotorType::A1 || type == MotorType::B1Motor ) ? 4 : 2;
  case MotorMode::INVALID:
  default:
    return 0;
  }
  return 0;
}

MotorMode getMotorMode( MotorType type, uint8_t code )
{
  switch ( type ) {
  case MotorType::A1:
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
  }
  return MotorMode::INVALID;
}

MotorComm::MotorComm( HardwareSerialIMXRT *serial, int direction_pin, MotorType type )
    : serial_( serial ), direction_pin_( direction_pin ), type_( type )
{
  serial_->transmitterEnable( direction_pin );
  if ( type == MotorType::A1 ) {
    serial_->addMemoryForWrite( send_buffer_, sizeof( send_buffer_ ) );
    serial_->addMemoryForRead( receive_buffer_, sizeof( receive_buffer_ ) );
    // Serial rate is 4,8Mbauds but begin calculates with uart clock of 24MHz and we use 80MHz
    serial_->begin( 1445000, SERIAL_8N1 );
  }
}

void MotorComm::sendReceive( const MotorCommCommand &command, MotorCommStatus &status )
{
  sendReceive( { command }, { status } );
}

void MotorComm::sendReceive( const std::vector<MotorCommCommand> &commands,
                             std::vector<MotorCommStatus> &status )
{
  if ( status.size() < commands.size() ) {
    status.resize( commands.size() );
  }
  for ( size_t i = 0; i < commands.size(); i++ ) {
    const auto &command = commands[i];
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
    data.Mdata.Pos = command.position;
    // Divide kp and kd by gear ratio squared to get the values at the rotor
    // Dividing by 26.7 and multiplying
    data.Mdata.K_P = static_cast<int16_t>( command.k_p / ( gear_ratio * gear_ratio ) / 26.7 * 2048 );
    data.Mdata.K_W = static_cast<int16_t>( command.k_w / ( gear_ratio * gear_ratio ) * 100.0 * 1024 );
    data.Mdata.Res[0].u32 = 256;
    data.CRCdata.u32 = crc32_core( (uint32_t *)&data, sizeof( data ) / sizeof( uint32_t ) - 1 );
    writeData( (uint8_t *)&data, sizeof( MasterComdDataV3 ) );
    status[i] = readStatus();
  }
}

MotorCommStatus MotorComm::readStatus()
{
  MotorCommStatus result;
  result.valid = false;
  uint8_t buffer[78];
  buffer[0] = 0xFE;
  buffer[1] = 0xEE;
  elapsedMicros time;
  while ( serial_->available() < 2 || serial_->read() != 0xFE || serial_->peek() != 0xEE ) {
    if ( time > 100 ) {
      return result;
    }
  }
  while ( serial_->available() < 77 ) {
    if ( time > 500 )
      return result;
  }
  serial_->readBytes( buffer + 1, 77 );
  uint32_t crc = crc32_core( (uint32_t *)buffer, 74 / sizeof( uint32_t ) );
  ServoComdDataV3 *status = (ServoComdDataV3 *)buffer;
  if ( crc != status->CRCdata.u32 ) {
    return result;
  }

  result.valid = true;
  result.motor_id = status->head.motorID;
  result.mode = getMotorMode( type_, status->Mdata.mode );
  result.temperature = status->Mdata.Temp;
  result.error_code = status->Mdata.MError;
  result.torque = status->Mdata.T / 256.0f * getGearRatio( type_ );
  result.velocity = status->Mdata.LW / getGearRatio( type_ );
  result.position = status->Mdata.Pos;
  result.acceleration = status->Mdata.Acc;
  return result;
}

void MotorComm::writeData( const uint8_t *data, size_t size ) { serial_->write( data, size ); }
