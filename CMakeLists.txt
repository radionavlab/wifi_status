cmake_minimum_required(VERSION 2.8.3)
project(wifi_watchdog)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Qt stuff (needed for an rviz plugin)

find_package(catkin REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  sensor_msgs
  tf
  mg_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS mg_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

##### wifi_watchdog ROS node #####
add_executable(wifi_watchdog
               src/wifi_watchdog.cpp
               )
target_link_libraries(wifi_watchdog ${catkin_LIBRARIES})
add_dependencies(wifi_watchdog ${catkin_EXPORTED_TARGETS})

add_dependencies(wifi_watchdog mg_msgs_gencpp)
