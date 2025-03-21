cmake_minimum_required(VERSION 3.0.2)
project(project_truck)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  yhs_can_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenSSL REQUIRED)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RobotPose.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  hmh_task.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  #   nav_msgs#   std_msgs#   yhs_can_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES project_truck
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime

#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/common_control.h
  src/common_control.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(car_test src/car_test.cpp)
target_link_libraries(car_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
add_executable(car1_control src/car1_control.cpp)
target_link_libraries(car1_control
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


add_executable(car2_control src/car2_control.cpp)
target_link_libraries(car2_control
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(head_web src/head_web.cpp)
target_link_libraries(head_web
  ${catkin_LIBRARIES}
  Boost::system
  OpenSSL::SSL
  OpenSSL::Crypto
  # nlohmann_json::nlohmann_json
)

add_executable(client2 src/client2.cpp)
target_link_libraries(client2
  ${catkin_LIBRARIES}
  Boost::system
  OpenSSL::SSL
  OpenSSL::Crypto
  # nlohmann_json::nlohmann_json
)
