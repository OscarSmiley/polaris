cmake_minimum_required(VERSION 2.8.3)
project(periph)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  #message_generation
  roscpp
  rospy
  std_msgs
  message_generation
  serial
  monitor
  dsp
  geometry_msgs
)

add_service_files(
  FILES
  #default.srv
)

add_message_files(
  FILES
  #default.msg
)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES peripherals
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dsp_INCLUDE_DIRS}
)

# find_package(serial REQUIRED)

add_executable(peripheralManager src/peripheralManager.cpp)
target_link_libraries(peripheralManager -lfftw3 ${catkin_LIBRARIES})

