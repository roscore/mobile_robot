cmake_minimum_required(VERSION 2.8.3)
project(mobile_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ssoni_mobile_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES edie_motor
  CATKIN_DEPENDS roscpp std_msgs ssoni_mobile_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mobile_robot src/test.cpp src/motor_algorithm.cpp)
add_dependencies(mobile_robot ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(mobile_robot ${catkin_LIBRARIES} pigpiod_if2)
