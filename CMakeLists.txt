cmake_minimum_required(VERSION 2.8.3)
project(arm_camera_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  actionlib_msgs
  actionlib
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  nav_msgs
  arm_control)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(arm_camera_calibration src/arm_camera_calibration.cpp)
target_link_libraries(arm_camera_calibration ${catkin_LIBRARIES})

