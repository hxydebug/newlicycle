cmake_minimum_required(VERSION 3.0.2)
project(bikebot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)

find_package(Eigen3 REQUIRED)

include_directories (${EIGEN3_INCLUDE_DIRS})
INCLUDE_DIRECTORIES("/usr/include/python3.6m")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bikebot
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(leg_controller	src/leg_controller.cpp)
add_library(bicycle_controller	src/bicycle_controller.cpp)
add_library(opmath	src/opmath.cpp)

target_link_libraries(opmath PUBLIC)
target_include_directories(opmath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(leg_controller PUBLIC)
target_include_directories(leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(bicycle_controller PUBLIC)
target_include_directories(bicycle_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(motor_drive	src/motor_drive.cpp)
target_link_libraries(motor_drive PUBLIC)
target_include_directories(motor_drive PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(bikebot_control node/bikebot_control.cpp)
target_link_libraries(bikebot_control ${catkin_LIBRARIES} pthread motor_drive bicycle_controller leg_controller opmath -lpython3.6m)

add_executable(uart_test src/uart_test.cpp)
target_link_libraries(uart_test ${catkin_LIBRARIES} pthread -lpython3.6m)
