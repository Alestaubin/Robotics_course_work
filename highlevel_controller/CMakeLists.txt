cmake_minimum_required(VERSION 3.10)
project(highlevel_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)



## Find Eigen3 for matrix and vector computations
find_package(Eigen3 REQUIRED)

## Find pinocchio
find_package(pinocchio REQUIRED)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")


## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp  
    actionlib 			# need this to execute ROS action
    highlevel_msgs
	sensor_msgs
	cubic_polynomial_planner
	kortex_gazebo
)

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
	  cubic_polynomial_planner
	  kortex_gazebo
	DEPENDS
)

include_directories(
	include
	#${EIGEN_INCLUDE_DIRS}	
	${catkin_INCLUDE_DIRS}	
	${Eigen3_INCLUDE_DIRS}
	${PINOCCHIO_INCLUDE_DIRS}
)

add_executable(pose_command_action_client
  	src/ActionClient.cpp  
)

target_link_libraries(pose_command_action_client
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