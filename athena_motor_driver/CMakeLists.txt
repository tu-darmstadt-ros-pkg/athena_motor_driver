cmake_minimum_required(VERSION 3.8)
project(athena_motor_driver)
set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hector_ros2_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(athena_motor_interface REQUIRED)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

set(TARGET_NAME athena_motor_driver_component)

set(SOURCES
  src/athena_motor_driver.cpp
  src/controllers/diff_drive_controller.cpp
)

add_library(${TARGET_NAME} SHARED ${SOURCES})

rclcpp_components_register_node(${TARGET_NAME}
  PLUGIN "athena_motor_driver::AthenaMotorDriver"
  EXECUTABLE athena_motor_driver
)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${TARGET_NAME}
  athena_motor_interface
  geometry_msgs
  hector_ros2_utils
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_srvs
  SERIAL
)

install(TARGETS ${TARGET_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
