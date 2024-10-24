cmake_minimum_required(VERSION 3.10)
project(cubic_polynomial_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin 
  REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    actionlib
    highlevel_msgs
)

## Find Eigen3 for matrix and vector computations
find_package(Eigen3 REQUIRED)

## Find pinocchio
find_package(pinocchio REQUIRED)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

catkin_package(
  INCLUDE_DIRS
    include
   #${EIGEN_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
  LIBRARIES 
    pinocchio
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    actionlib
    highlevel_msgs
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  #${EIGEN_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

add_executable(kinematic_controller src/inverse_kinematic_controller.cpp)

target_link_libraries(kinematic_controller 
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
)

add_executable(pose_command_action_server src/pose_command_action_server.cpp)

target_link_libraries(  pose_command_action_server 
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
)


#############
## Install ##
#############

# Mark hpp header files for installation
install(
  DIRECTORY include/pose_command_action_server/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
