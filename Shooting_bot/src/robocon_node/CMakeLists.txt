cmake_minimum_required(VERSION 3.0.2)
project(robocon_node)

## Compile as C++11 (required for ROS Noetic)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Declare message files (if you have any)
add_message_files(
  FILES
  Encoders.msg
)

## Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare the C++ executable
#add_executable(encoder_uart_node src/encoder_uart_node.cpp)

## Add dependencies to generated messages
#add_dependencies(encoder_uart_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link libraries (including serial)
#target_link_libraries(encoder_uart_node
#  ${catkin_LIBRARIES}
#  serial
#)
catkin_install_python(PROGRAMS
  scripts/scan_to_coordinates.py
  scripts/position_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
