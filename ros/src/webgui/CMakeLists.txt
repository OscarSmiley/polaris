cmake_minimum_required(VERSION 2.8.3)
project(webgui)
set(CMAKE_CXX_FLAGS "-std=c++11")

# catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge 
)


catkin_package(
# INCLUDE_DIRS include
  #LIBRARIES vision
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs message_runtime std_msgs message_generation
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
##############################################################################
# Binaries                                                                   #
##############################################################################
find_package(OpenCV 3 REQUIRED)

add_executable(sender src/sender.cpp)
add_dependencies(sender ${webgui_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sender ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(reciever src/reciever.cpp)
add_dependencies(reciever ${webgui_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reciever ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
