cmake_minimum_required(VERSION 3.0.2)
project(nav_converter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  agv_define
  libraryparam
)
## Generate messages in the 'msg' folder
add_message_files(
    FILES
    speed_wheel.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nav_converter
   CATKIN_DEPENDS roscpp std_msgs libraryparam
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


#nav_converter
add_executable(nav_converter src/nav_converter_node.cpp)
add_dependencies(nav_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nav_converter ${catkin_LIBRARIES})


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
