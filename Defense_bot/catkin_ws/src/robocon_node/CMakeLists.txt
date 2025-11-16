cmake_minimum_required(VERSION 3.0.2)
project(robocon_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy                # <-- Add if using Python scripts
  sensor_msgs
  std_msgs             # <-- Add if using Float32MultiArray
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bno085_data_node src/bno085_data_node.cpp)
target_link_libraries(bno085_data_node ${catkin_LIBRARIES})
add_dependencies(bno085_data_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install Python script
catkin_install_python(PROGRAMS
  scripts/adxrs_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
