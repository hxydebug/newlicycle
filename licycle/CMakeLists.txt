cmake_minimum_required(VERSION 3.0.2)
project(licycle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)

find_package(Eigen3 REQUIRED)

# set(QPOASES_INLUDE_DIRS /home/hxy/catkin_ws/src/licycle/third_party/qpOASES/include/qpOASES)
set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
include_directories (${QPOASES_INLUDE_DIRS})
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


add_executable(qptest 	src/qptest.cpp)
target_link_libraries(qptest qpOASES)

add_library(newleg_controller	src/leg_controller.cpp)
add_library(newbicycle_controller	src/bicycle_controller.cpp)
add_library(newopmath	src/opmath.cpp)

add_library(swing_leg_controller	src/swing_leg_controller.cpp)
add_library(stance_leg_controller	src/stance_leg_controller.cpp)
add_library(control	src/control.cpp)
add_library(gait_generator	src/gait_generator.cpp)

target_link_libraries(newopmath PUBLIC)
target_include_directories(newopmath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(gait_generator PUBLIC)
target_include_directories(gait_generator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(stance_leg_controller PUBLIC qpOASES gait_generator)
target_include_directories(stance_leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(swing_leg_controller PUBLIC gait_generator)
target_include_directories(swing_leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(control PUBLIC)
target_include_directories(control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(newleg_controller PUBLIC swing_leg_controller stance_leg_controller gait_generator)
target_include_directories(leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(newbicycle_controller PUBLIC)
target_include_directories(bicycle_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(newmotor_drive	src/motor_drive.cpp)
target_link_libraries(newmotor_drive PUBLIC)
target_include_directories(newmotor_drive PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(licycle_control node/bikebot_control.cpp)
target_link_libraries(licycle_control ${catkin_LIBRARIES} pthread newmotor_drive newbicycle_controller newleg_controller newopmath -lpython3.6m swing_leg_controller stance_leg_controller gait_generator control)

