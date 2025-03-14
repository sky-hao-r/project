cmake_minimum_required(VERSION 3.0.2)
project(odom1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gps_common
  message_generation
  sensor_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)
find_package(nlohmann_json REQUIRED)
# set(Ceres_DIR "/home/yxz/Library/INSTALL/ceres/lib/cmake/Ceres")
# find_package(Ceres REQUIRED)
# find_package(Glog REQUIRED)
include_directories(${PROJ_INCLUDE_DIRS})
link_directories(${PROJ_LIBRARY_DIRS})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_message_files(
  FILES
  underpan_speed.msg
  underpan_code_w.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rtk
#  CATKIN_DEPENDS roscpp std_msgs
CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

add_executable(trajSave src/trajSave.cpp)


target_link_libraries(trajSave
  ${catkin_LIBRARIES}
  nlohmann_json::nlohmann_json
)
add_executable(underpan src/underpan.cpp)
target_link_libraries(underpan
  ${catkin_LIBRARIES}
)
add_executable(head_underpan src/head_underpan.cpp)
target_link_libraries(head_underpan
  ${catkin_LIBRARIES}
)

