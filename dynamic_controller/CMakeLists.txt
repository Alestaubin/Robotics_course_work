cmake_minimum_required(VERSION 3.10)
project(dynamic_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin 
  REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    highlevel_msgs
)

## Find Eigen3 for matrix and vector computations
find_package(Eigen3 REQUIRED)

## Find pinocchio
find_package(pinocchio REQUIRED)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

catkin_package(
  INCLUDE_DIRS
#    include
   #${EIGEN_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
  LIBRARIES 
    pinocchio
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    highlevel_msgs
  DEPENDS
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}  
  #${EIGEN_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

add_executable(joint_dynamic_controller src/joint_dynamic_controller.cpp)
add_executable(inverse_dynamic_controller src/inverse_dynamic_controller.cpp)

target_link_libraries(	joint_dynamic_controller 
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
)
target_link_libraries(  inverse_dynamic_controller 
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
)

#############
## Install ##
#############

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)