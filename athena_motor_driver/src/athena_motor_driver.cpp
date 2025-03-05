#include "athena_motor_driver/athena_motor_driver.hpp"

#include "athena_motor_driver/controllers/diff_drive_controller.hpp"
#include <algorithm>
#include <athena_motor_interface/athena_motor_interfaces.h>

using namespace std::chrono_literals;

namespace athena_motor_driver
{

struct AthenaMotorDriver::InternalData {
  FullMotorStatus status;
};

AthenaMotorDriver::AthenaMotorDriver( const rclcpp::NodeOptions &options )
    : Node( "athena_motor_driver", options ), data_( std::make_unique<InternalData>() )
{
  // Declare serial parameters
  declare_readonly_parameter( "port_name", port_name_, "Serial port name" );
  declare_readonly_parameter( "baud_rate", baud_rate_, "Serial baud rate" );
  declare_reconfigurable_parameter(
      "controller", controller_type_, "Controller type",
      hector::ReconfigurableParameterOptions<std::string>()
          .additionalConstraints( "Allowed values: diff_drive" )
          .onValidate( []( const auto &value ) { return value == "diff_drive"; } )
          .onUpdate( [this]( const std::string &value ) { setupController( value ); } ) );

  setupController( controller_type_ );
  setupSerial();
  timer_ = create_wall_timer( 10ms, [this]() { update(); } );
  last_twist_ = now();
  twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, [this]( const geometry_msgs::msg::Twist::ConstSharedPtr msg ) {
        twist_msg_ = msg;
        last_twist_ = now();
      } );
  status_pub_ = create_publisher<athena_motor_interface::msg::FullMotorStatus>( "motor_status", 1 );
}

AthenaMotorDriver::~AthenaMotorDriver() { closeSerial(); }

void AthenaMotorDriver::setupSerial()
{
  serial_ = std::make_unique<LibSerial::SerialPort>();
  if ( serial_->Open( port_name_ ); !serial_->IsOpen() ) {
    RCLCPP_ERROR( get_logger(), "Failed to open serial port." );
    return;
  }
  serial_->SetBaudRate( LibSerial::BaudRate::BAUD_1152000 );
  RCLCPP_INFO( get_logger(), "Serial port '%s' configured with baud rate %d", port_name_.c_str(),
               baud_rate_ );
}

void AthenaMotorDriver::closeSerial() { serial_->Close(); }

namespace
{
void updateStatus( athena_motor_interface::msg::MotorStatus &msg, const MotorStatus &status,
                   const rclcpp::Time &timestamp )
{
  if ( !status.valid )
    return;
  msg.valid = status.valid;
  msg.mode = status.mode;
  msg.error_code = status.error_code;
  msg.temperature = status.temperature;
  msg.torque = status.torque;
  msg.velocity = status.velocity;
  msg.position = status.position;
  msg.acceleration = status.acceleration;
  msg.stamp = timestamp;
}
} // namespace

void AthenaMotorDriver::update()
{
  if ( !serial_->IsOpen() )
    return;
  //  if ( serial_->IsDataAvailable() ) {
  //    RCLCPP_INFO( get_logger(), "Data available. Flushing." );
  //    serial_->FlushIOBuffers();
  //  }
  try {
    if ( last_twist_ + 100ms < now() ) {
      MotorCommand command{ 0, 0 };
      sendCommand( command );
    } else if ( twist_msg_ ) {
      MotorCommand command =
          controller_->computeMotorCommand( twist_msg_->linear.x, twist_msg_->angular.z );
      RCLCPP_DEBUG( get_logger(), "Sending command: %f, %f", command.left_velocity,
                    command.right_velocity );
      sendCommand( command );
    }

    unsigned char start_byte;
    while ( serial_->IsDataAvailable() ) {
      serial_->ReadByte( start_byte, 0 );
      if ( start_byte == START_BYTE )
        break;
    }
    if ( start_byte != START_BYTE ) {
      return;
    }
    uint8_t command_type;
    serial_->ReadByte( command_type, 0 );
    switch ( static_cast<CommandType>( command_type ) ) {
    case CommandType::MOTOR_STATUS: {
      LibSerial::DataBuffer buffer( FullMotorStatus::get_size() );
      serial_->Read( buffer, FullMotorStatus::get_size() );
      FullMotorStatus status = FullMotorStatus::deserialize( buffer.data() );
      rclcpp::Time stamp = this->now();
      updateStatus( status_msg_.front_left, status.front_left, stamp );
      updateStatus( status_msg_.front_right, status.front_right, stamp );
      updateStatus( status_msg_.rear_left, status.rear_left, stamp );
      updateStatus( status_msg_.rear_right, status.rear_right, stamp );
      if ( status.front_left.valid || status.front_right.valid || status.rear_left.valid ||
           status.rear_right.valid ) {
        status_pub_->publish( status_msg_ );
      }
      break;
    }
    default:
      RCLCPP_WARN( get_logger(), "Unknown command type: %d", command_type );
    }
  } catch ( std::exception &e ) {
    RCLCPP_ERROR( get_logger(), "Error reading from serial port: %s. Trying to reopen.", e.what() );
    try {
      serial_.reset();
    } catch ( std::exception &e ) {
      RCLCPP_ERROR( get_logger(), "Error closing serial port: %s", e.what() );
    }
    setupSerial();
  }
}
void AthenaMotorDriver::setupController( const std::string &controller_type )
{
  auto node = std::shared_ptr<rclcpp::Node>(this, [](const rclcpp::Node *) {});
  if ( controller_type == "diff_drive" ) {
    controller_ = std::make_shared<DiffDriveController>( node );
  }
}

template<typename T>
void AthenaMotorDriver::sendCommand( const T &command )
{
  LibSerial::DataBuffer buffer;
  buffer.clear();
  buffer.resize( T::get_size() );
  serialize( command, buffer );
  serial_->Write( buffer );
}

} // namespace athena_motor_driver

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE( athena_motor_driver::AthenaMotorDriver )
