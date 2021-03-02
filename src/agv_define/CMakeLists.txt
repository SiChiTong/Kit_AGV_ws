cmake_minimum_required(VERSION 3.0.2)
project(agv_define)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
  geometry_msgs
)

add_message_files(
  FILES
  agv_action.msg
  agv_flexisoft.msg
)

add_service_files(
  FILES
  lift.srv
  convertJsonToRos_srv.srv
)

add_action_files(
  DIRECTORY action
  FILES line.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(agvlib src/agvlib.cpp)
target_link_libraries(agvlib ${catkin_LIBRARIES})


add_executable(testLib src/testLib.cpp)
add_dependencies(testLib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testLib ${catkin_LIBRARIES})

