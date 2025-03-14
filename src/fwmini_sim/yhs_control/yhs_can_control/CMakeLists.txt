cmake_minimum_required(VERSION 2.8.3)
project(yhs_can_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  yhs_can_msgs
)

#find_package(yhs_can_msgs REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yhs_can_control
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########

link_directories(
# include
  include/yhs_can_control
  ${catkin_INCLUDE_DIRS} yhs_can_control
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 # include
  include/yhs_can_control
  ${catkin_INCLUDE_DIRS} yhs_can_control
)

## Declare a C++ library
# add_library(yhs_can_control
#   include/${PROJECT_NAME}/yhs_can_control.cpp
#   src/yhs_can_control.cpp
# )


#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(yhs_can_control src/yhs_can_control.cpp)
# add_executable(yhs_control src/yhs_control.cpp)
add_executable(fwmini_control src/fwmini_control.cpp)
add_executable(fwmini_pub_sub src/fwmini_pub_sub.cpp)
# add_executable(yhs_pub_sub src/yhs_pub_sub.cpp)









## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(yhs_can_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})





## Specify libraries to link a library or executable target against
target_link_libraries(yhs_can_control
   ${catkin_LIBRARIES}
)
# target_link_libraries(yhs_control
#    ${catkin_LIBRARIES}
# )
target_link_libraries(fwmini_control
   ${catkin_LIBRARIES}
)
target_link_libraries(fwmini_pub_sub
   ${catkin_LIBRARIES}
)
# target_link_libraries(yhs_pub_sub
#    ${catkin_LIBRARIES}
# )

